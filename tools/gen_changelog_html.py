#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
把 `修改日志.md` 转成**单文件** HTML：左侧按日期分支的时间轴 + 右侧全文。

设计要点
--------
1. **日期从标题里抽**：本日志的小节标题形如 `### 7.10 到位判据加固：… · 2026/10/10`，
   或形如 `## 一、第一轮：提纯出 KWQ3 · 2026/10/8`（其下 `### 1.1` 继承该日期）。
   标题自带日期优先，否则向上继承章节日期，再没有就归到「常驻」。
2. **正文保持文档原顺序**：侧边栏是"按日期"的索引，正文是"按发生顺序"的流程，
   两种看法互不干扰 —— 想看某一天就点日期，想看一路怎么改就往下滚。
3. **单文件、离线可用**：CSS/JS 全部内联，不依赖 CDN，双击即可打开。

用法
----
    python tools/gen_changelog_html.py            # 生成 修改日志.html
    python tools/gen_changelog_html.py --src xxx.md --out yyy.html

依赖：`markdown`（pip install markdown）
"""

from __future__ import annotations

import argparse
import datetime as _dt
import html
import os
import re
import sys

try:
    import markdown
except ImportError:  # pragma: no cover
    sys.exit("缺少依赖：pip install markdown")

# --------------------------------------------------------------------------- #
# 常量
# --------------------------------------------------------------------------- #

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SRC = os.path.join(ROOT, "修改日志.md")
DEFAULT_OUT = os.path.join(ROOT, "修改日志.html")

# `2026/10/8` / `2026/10/08`，两侧允许 `·` 或 `~` 分隔
DATE_RE = re.compile(r"2026/(\d{1,2})/(\d{1,2})")
RANGE_RE = re.compile(
    r"2026/(\d{1,2})/(\d{1,2})\s*[~～]\s*(?:2026/)?(\d{1,2})/(\d{1,2})"
)
# 标题尾部的日期后缀（渲染前要剥掉，改成徽标显示）
DATE_TAIL_RE = re.compile(
    r"\s*[·・]\s*2026/\d{1,2}/\d{1,2}"
    r"(?:\s*[~～]\s*(?:2026/)?\d{1,2}/\d{1,2})?\s*$"
)

NUM_DOTTED_RE = re.compile(r"^((?:\d+|[A-Za-z])(?:\.\d+)+)\s+(.+)$")
NUM_CN_RE = re.compile(r"^([〇一二三四五六七八九十]+)、\s*(.+)$")
NUM_APPX_RE = re.compile(r"^附录\s*([A-Za-z])?\s*[：:]?\s*(.*)$")

WEEKDAY_CN = ["周一", "周二", "周三", "周四", "周五", "周六", "周日"]


# --------------------------------------------------------------------------- #
# 解析
# --------------------------------------------------------------------------- #


class Unit:
    """一个可导航单元：二级章节 或 三级小节。"""

    __slots__ = (
        "level", "heading", "lines", "lineno", "num", "title",
        "date_key", "date_disp", "is_range", "chap", "chap_key", "uid", "body",
        "has_children",
    )

    def __init__(self, level: int, heading: str, lineno: int):
        self.level = level
        self.heading = heading
        self.lines: list[str] = []
        self.lineno = lineno
        self.num = ""
        self.title = heading
        self.date_key = None      # 'YYYY-MM-DD' 或 None
        self.date_disp = ""       # '2026/10/10' 或 '10/8 ~ 10/9'
        self.is_range = False
        self.chap = ""            # 所属二级章节标题（含序号）
        self.chap_key = ""
        self.uid = ""
        self.body = ""            # 渲染后的 HTML
        self.has_children = False  # 二级章节：其下是否有三级小节

    @property
    def has_body(self) -> bool:
        return any(ln.strip() and ln.strip() != "---" for ln in self.lines)


def split_units(text: str):
    """按二级/三级标题切分，围栏代码块内的 `#` 不算标题。"""
    lines = text.split("\n")
    preamble: list[str] = []
    units: list[Unit] = []
    cur: Unit | None = None
    in_fence = False

    for i, ln in enumerate(lines):
        if ln.lstrip().startswith("```"):
            in_fence = not in_fence

        if not in_fence and (ln.startswith("## ") or ln.startswith("### ")):
            level = 3 if ln.startswith("### ") else 2
            cur = Unit(level, ln[level + 1:].strip(), i + 1)
            units.append(cur)
            continue

        if cur is None:
            preamble.append(ln)
        else:
            cur.lines.append(ln)

    return preamble, units


def plain(text: str) -> str:
    """标题里的 markdown 装饰去掉，只留纯文本（`**x**` → x，`` `x` `` → x）。"""
    t = text.replace("**", "").replace("__", "").replace("~~", "")
    t = t.replace("`", "")
    return t.strip()


def parse_num_title(heading: str):
    """`7.10 到位判据加固：…` → ('7.10', '到位判据加固：…')。"""
    m = NUM_DOTTED_RE.match(heading)
    if m:
        return m.group(1), m.group(2).strip()
    m = NUM_CN_RE.match(heading)
    if m:
        return m.group(1), m.group(2).strip()
    m = NUM_APPX_RE.match(heading)
    if m and heading.startswith("附录"):
        letter = (m.group(1) or "").upper()
        return ("附录" + letter), (m.group(2) or "").strip()
    return "", heading


def extract_date(heading: str):
    """返回 (date_key, date_disp, is_range)。"""
    m = RANGE_RE.search(heading)
    if m:
        m1, d1, m2, d2 = (int(x) for x in m.groups())
        key = f"2026-{m1:02d}-{d1:02d}"
        return key, f"{m1}/{d1} ~ {m2}/{d2}", True

    m = DATE_RE.search(heading)
    if m:
        mo, dy = int(m.group(1)), int(m.group(2))
        return f"2026-{mo:02d}-{dy:02d}", f"2026/{mo}/{dy}", False

    return None, "", False


def slug_uid(num: str, title: str, used: dict) -> str:
    base = (num or title)[:24]
    base = re.sub(r"[^\w\u4e00-\u9fff]+", "-", base).strip("-").lower()
    if not base:
        base = "sec"
    base = "sec-" + base
    used[base] = used.get(base, 0) + 1
    if used[base] > 1:
        base = f"{base}-{used[base]}"
    return base


def prepare(text: str):
    preamble, units = split_units(text)

    chap_unit: Unit | None = None
    used: dict = {}
    chap_dates: dict = {}

    for u in units:
        u.num, u.title = parse_num_title(u.heading)
        key, disp, is_range = extract_date(u.heading)

        if u.level == 2:
            chap_unit = u
            u.chap = u.heading
            u.chap_key = u.uid or ""
            if key:
                chap_dates[id(u)] = (key, disp, is_range)
        else:
            if chap_unit is not None:
                u.chap = chap_unit.heading
                u.chap_key = chap_unit.uid or ""

        if key:
            u.date_key, u.date_disp, u.is_range = key, disp, is_range
        elif u.level == 3 and chap_unit is not None and id(chap_unit) in chap_dates:
            u.date_key, u.date_disp, u.is_range = chap_dates[id(chap_unit)]
        elif u.level == 2 and id(u) in chap_dates:
            u.date_key, u.date_disp, u.is_range = chap_dates[id(u)]

        # 标题去掉尾部日期，日期改由徽标呈现；顺手剥掉 markdown 装饰
        u.title = DATE_TAIL_RE.sub("", u.title).strip() or u.title
        u.title = plain(u.title)
        u.uid = slug_uid(u.num, u.title, used)

        if u.level == 2:
            u.chap_key = u.uid

    # 章节 uid 建立后回填给子单元，并标记"有子节"
    chap_map = {u.heading: u for u in units if u.level == 2}
    for u in units:
        if u.level == 3:
            ch = chap_map.get(u.chap)
            u.chap_key = ch.uid if ch else ""
            if ch is not None:
                ch.has_children = True

    return preamble, units


# --------------------------------------------------------------------------- #
# 渲染 markdown
# --------------------------------------------------------------------------- #

MD_EXT = ["tables", "fenced_code", "sane_lists"]


def md2html(src: str, uid: str, nl2br: bool = False) -> str:
    exts = list(MD_EXT) + (["nl2br"] if nl2br else [])
    body = markdown.markdown(src, extensions=exts, output_format="html5")

    # 表格横向滚动容器
    body = body.replace("<table>", '<div class="tw"><table>').replace(
        "</table>", "</table></div>"
    )

    # 给 h4/h5 加锚点 id，方便 `#sec-7-10-h3` 这种深链
    counter = [0]

    def _repl(m):
        lvl, inner = m.group(1), m.group(2)
        counter[0] += 1
        return f'<h{lvl} id="{uid}-h{counter[0]}">{inner}</h{lvl}>'

    body = re.sub(r"<h([45])>(.*?)</h\1>", _repl, body, flags=re.S)
    return body


def short_chap(chap: str) -> str:
    """`七、添加任务（SEQ ADD）：方案与第一步落地` → `七、添加任务`。"""
    if not chap:
        return ""
    head = chap.split("：")[0].split("(")[0].strip()
    return head


# --------------------------------------------------------------------------- #
# 组装 HTML 片段
# --------------------------------------------------------------------------- #


def render_section(u: Unit) -> str:
    date_attr = u.date_key or "none"
    badges = []
    if u.date_disp:
        cls = "badge date range" if u.is_range else "badge date"
        badges.append(f'<span class="{cls}">{html.escape(u.date_disp)}</span>')
    else:
        badges.append('<span class="badge date none">常驻</span>')
    if u.chap and u.level == 3:
        badges.append(
            f'<span class="badge chap" title="{html.escape(u.chap)}">'
            f"{html.escape(short_chap(u.chap))}</span>"
        )
    if u.num:
        badges.append(f'<span class="badge num">§{html.escape(u.num)}</span>')

    title = html.escape(u.title)
    num_html = f'<span class="sec-num">{html.escape(u.num)}</span>' if u.num else ""

    return (
        f'<section class="sec" id="{u.uid}" data-date="{date_attr}" '
        f'data-num="{html.escape(u.num)}" data-chap="{html.escape(u.chap)}">\n'
        f'  <div class="sec-head">\n'
        f'    <div class="sec-badges">{"".join(badges)}'
        f'<a class="sec-anchor" href="#{u.uid}" title="复制本节链接">#</a></div>\n'
        f'    <h2 class="sec-title">{num_html}{title}</h2>\n'
        f"  </div>\n"
        f'  <div class="md">{u.body}</div>\n'
        f"</section>"
    )


def render_chapter_divider(u: Unit) -> str:
    date_html = (
        f'<span class="badge date">{html.escape(u.date_disp)}</span>'
        if u.date_disp
        else ""
    )
    intro = f'<div class="md cd-intro">{u.body}</div>' if u.body else ""
    return (
        f'<div class="chap-div" id="{u.uid}">'
        f'<div class="cd-line">'
        f'<span class="cd-num">{html.escape(u.num)}</span>'
        f'<span class="cd-title">{html.escape(u.title)}</span>'
        f"{date_html}</div>{intro}</div>"
    )


def day_label(key: str) -> str:
    try:
        d = _dt.date.fromisoformat(key)
    except ValueError:
        return key
    return f"{d.month}/{d.day}"


def day_weekday(key: str) -> str:
    try:
        d = _dt.date.fromisoformat(key)
    except ValueError:
        return ""
    return WEEKDAY_CN[d.weekday()]


def build_nav(units: list[Unit]):
    """按日期分组，返回 (sidebar_html, strip_html, days_meta)。"""
    groups: dict[str, list[Unit]] = {}
    order: list[str] = []

    for u in units:
        # 有子节的二级章节只作分隔，不进索引（避免和 1.1/2.1 这类小节重复）
        if not u.has_body or u.has_children:
            continue
        key = u.date_key or "none"
        if key not in groups:
            groups[key] = []
            order.append(key)
        groups[key].append(u)

    dated = sorted([k for k in order if k != "none"])
    ordered = dated + (["none"] if "none" in groups else [])

    nav_parts = []
    strip_parts = []
    days_meta = []

    for key in ordered:
        items = groups[key]
        if key == "none":
            name, wd = "常驻", ""
            disp = "常驻"
        else:
            name, wd = day_label(key), day_weekday(key)
            disp = key

        li = []
        for u in items:
            num = (
                f'<span class="ni-num">{html.escape(u.num)}</span>'
                if u.num
                else '<span class="ni-num">·</span>'
            )
            li.append(
                f'<li class="nav-item" data-date="{key}" data-target="{u.uid}" '
                f'data-chap="{html.escape(short_chap(u.chap))}">'
                f'<a href="#{u.uid}">{num}'
                f'<span class="ni-t">{html.escape(u.title)}</span></a></li>'
            )

        nav_parts.append(
            f'<div class="day-group open" data-date="{key}">'
            f'<button class="day-head" type="button">'
            f'<span class="day-name">{html.escape(name)}</span>'
            f'<span class="day-wd">{wd}</span>'
            f'<span class="day-cnt">{len(items)}</span>'
            f'<span class="day-caret"></span>'
            f"</button>"
            f'<ul class="day-list">{"".join(li)}</ul>'
            f"</div>"
        )

        if key != "none":
            strip_parts.append(
                f'<button class="chip" type="button" data-date="{key}">'
                f'<b>{html.escape(name)}</b><i>{len(items)}</i></button>'
            )
            days_meta.append(
                {"key": key, "label": name, "wd": wd, "count": len(items)}
            )

    if "none" in groups:
        strip_parts.append(
            f'<button class="chip" type="button" data-date="none">'
            f"<b>常驻</b><i>{len(groups['none'])}</i></button>"
        )

    return "".join(nav_parts), "".join(strip_parts), days_meta


# --------------------------------------------------------------------------- #
# 页面模板
# --------------------------------------------------------------------------- #

TEMPLATE = """<!DOCTYPE html>
<html lang="zh-CN" data-theme="light">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>@@TITLE@@</title>
<style>
@@CSS@@
</style>
</head>
<body>

<header class="top">
  <div class="brand">
    <span class="logo">KWQ3</span>
    <span class="bt">@@TITLE@@</span>
  </div>
  <div class="search">
    <input id="q" type="search" placeholder="搜索标题 / 正文…（按 / 聚焦，Esc 清除）" autocomplete="off">
    <span id="qcount" class="qcount"></span>
  </div>
  <div class="tools">
    <button id="theme" class="tbtn" type="button" title="切换深色/浅色">◐</button>
  </div>
</header>

<div class="shell">
  <aside class="side">
    <div class="side-hd">
      <span>按日期</span>
      <span class="side-stat">@@STATS@@</span>
    </div>
    <nav id="nav">
@@NAV@@
    </nav>
    <div class="side-ft">生成于 @@GENERATED@@</div>
  </aside>

  <main class="main" id="main">
    <div class="doc-head md">@@DOCINFO@@</div>

    <div class="strip-card">
      <div class="strip-title">修改时间轴 <span>点某一天可只看当天</span></div>
      <div class="strip" id="strip">@@STRIP@@</div>
    </div>

    <div id="filterbar" class="filterbar hide">
      <span id="filtertext"></span>
      <button id="clearfilter" type="button">清除筛选</button>
    </div>

    <div id="content">
@@CONTENT@@
    </div>

    <footer class="foot">— 共 @@COUNT@@ 节 —</footer>
  </main>
</div>

<div id="toast" class="toast"></div>

<script>
@@JS@@
</script>
</body>
</html>
"""

CSS = """
*,*::before,*::after{box-sizing:border-box}
:root{
  --bg:#f7f8fa; --panel:#ffffff; --ink:#1b1f24; --ink2:#5a6472; --ink3:#8a94a0;
  --line:#e4e7ec; --line2:#eef0f3; --accent:#2563eb; --accent-soft:#eaf1ff;
  --code-bg:#f4f6f8; --code-ink:#334155; --shadow:0 1px 2px rgba(16,24,40,.05);
  --hl:#fff2b8;
}
html[data-theme="dark"]{
  --bg:#0e1116; --panel:#151a21; --ink:#e6edf3; --ink2:#9aa4b2; --ink3:#6b7480;
  --line:#232a33; --line2:#1c222a; --accent:#5b9bff; --accent-soft:#16233a;
  --code-bg:#0a0d11; --code-ink:#cbd5e1; --shadow:0 1px 2px rgba(0,0,0,.4);
  --hl:#4a3f1a;
}
html,body{height:100%}
body{
  margin:0; background:var(--bg); color:var(--ink);
  font:15px/1.75 -apple-system,BlinkMacSystemFont,"Segoe UI","Microsoft YaHei","PingFang SC",sans-serif;
  display:flex; flex-direction:column; overflow:hidden;
  -webkit-font-smoothing:antialiased;
}
a{color:var(--accent); text-decoration:none}
code,pre,kbd{font-family:"Cascadia Mono",Consolas,"SF Mono",Menlo,monospace}

/* ---------- 顶栏 ---------- */
.top{
  flex:0 0 auto; height:56px; display:flex; align-items:center; gap:18px;
  padding:0 18px; background:var(--panel); border-bottom:1px solid var(--line);
  box-shadow:var(--shadow); z-index:20;
}
.brand{display:flex; align-items:center; gap:10px; min-width:0}
.logo{
  font-size:11px; font-weight:700; letter-spacing:.06em; color:#fff;
  background:linear-gradient(135deg,#2563eb,#7c3aed); padding:4px 8px; border-radius:6px;
}
.bt{font-weight:600; font-size:15px; white-space:nowrap; overflow:hidden; text-overflow:ellipsis}
.search{flex:1; display:flex; align-items:center; gap:10px; max-width:620px}
.search input{
  flex:1; height:34px; padding:0 12px; border:1px solid var(--line); border-radius:8px;
  background:var(--bg); color:var(--ink); font-size:14px; outline:none; transition:.15s;
}
.search input:focus{border-color:var(--accent); background:var(--panel); box-shadow:0 0 0 3px var(--accent-soft)}
.qcount{font-size:12px; color:var(--ink3); white-space:nowrap; font-variant-numeric:tabular-nums}
.tools{display:flex; gap:8px}
.tbtn{
  width:34px; height:34px; border:1px solid var(--line); background:var(--panel);
  color:var(--ink2); border-radius:8px; cursor:pointer; font-size:15px; line-height:1;
}
.tbtn:hover{border-color:var(--accent); color:var(--accent)}

/* ---------- 布局 ---------- */
.shell{flex:1; display:flex; min-height:0}
.side{
  flex:0 0 326px; display:flex; flex-direction:column; min-height:0;
  background:var(--panel); border-right:1px solid var(--line);
}
.side-hd{
  flex:0 0 auto; display:flex; justify-content:space-between; align-items:baseline;
  padding:14px 16px 10px; font-size:12px; font-weight:600; color:var(--ink2);
  letter-spacing:.04em; border-bottom:1px solid var(--line2);
}
.side-stat{font-weight:400; color:var(--ink3); font-size:11.5px}
#nav{flex:1; overflow-y:auto; padding:8px 8px 20px}
.side-ft{flex:0 0 auto; padding:8px 16px; font-size:11px; color:var(--ink3); border-top:1px solid var(--line2)}

/* ---------- 日期分组 ---------- */
.day-group{margin-bottom:2px}
.day-group.hide{display:none}
.day-head{
  width:100%; display:flex; align-items:center; gap:8px; padding:7px 10px;
  background:none; border:0; border-radius:7px; cursor:pointer; text-align:left;
  color:var(--ink); font-size:13.5px; font-weight:600;
}
.day-head:hover{background:var(--bg)}
.day-name{font-variant-numeric:tabular-nums}
.day-wd{font-size:11.5px; font-weight:400; color:var(--ink3)}
.day-cnt{
  margin-left:auto; font-size:11px; font-weight:600; color:var(--accent);
  background:var(--accent-soft); border-radius:9px; padding:1px 7px;
  font-variant-numeric:tabular-nums;
}
.day-caret{
  width:0; height:0; border-left:4px solid transparent; border-right:4px solid transparent;
  border-top:5px solid var(--ink3); transition:transform .15s; flex:0 0 auto;
}
.day-group.open .day-caret{transform:rotate(180deg)}
.day-list{list-style:none; margin:0; padding:0 0 6px 0; display:none}
.day-group.open .day-list{display:block}
.nav-item a{
  display:flex; gap:8px; padding:5px 10px 5px 22px; border-radius:6px;
  color:var(--ink2); font-size:13px; line-height:1.5; position:relative;
}
.nav-item a:hover{background:var(--bg); color:var(--ink)}
.nav-item.active a{background:var(--accent-soft); color:var(--accent); font-weight:600}
.nav-item.hide{display:none}
.ni-num{
  flex:0 0 42px; color:var(--ink3); font-size:11.5px; font-variant-numeric:tabular-nums;
  padding-top:1px;
}
.nav-item.active .ni-num{color:var(--accent)}
.ni-t{flex:1; overflow:hidden; text-overflow:ellipsis; white-space:nowrap}

/* ---------- 正文 ---------- */
.main{flex:1; overflow-y:auto; padding:26px 40px 80px}
.main > *{max-width:1080px; margin-left:auto; margin-right:auto}

.doc-head{
  background:var(--panel); border:1px solid var(--line); border-radius:12px;
  padding:16px 22px; margin-bottom:18px; box-shadow:var(--shadow);
}
.doc-head.md{font-size:13.5px; color:var(--ink2); line-height:1.85}
.doc-head.md blockquote{
  margin:0; padding:0; border:0; color:var(--ink2);
}
.doc-head.md p{margin:0 0 4px}
.doc-head.md strong{color:var(--ink)}
.doc-head.md code{font-size:12.5px}

.strip-card{
  background:var(--panel); border:1px solid var(--line); border-radius:12px;
  padding:14px 18px 16px; margin-bottom:18px; box-shadow:var(--shadow);
}
.strip-title{font-size:12.5px; font-weight:600; color:var(--ink2); margin-bottom:10px}
.strip-title span{font-weight:400; color:var(--ink3); margin-left:8px}
.strip{display:flex; flex-wrap:wrap; gap:8px}
.chip{
  display:inline-flex; align-items:center; gap:6px; padding:5px 10px;
  border:1px solid var(--line); border-radius:999px; background:var(--bg);
  color:var(--ink2); cursor:pointer; font-size:12.5px; transition:.12s;
}
.chip b{font-weight:600; font-variant-numeric:tabular-nums}
.chip i{
  font-style:normal; font-size:10.5px; color:var(--ink3);
  background:var(--panel); border-radius:8px; padding:0 5px;
}
.chip:hover{border-color:var(--accent); color:var(--accent)}
.chip.on{background:var(--accent); border-color:var(--accent); color:#fff}
.chip.on i{background:rgba(255,255,255,.22); color:#fff}

.filterbar{
  display:flex; align-items:center; gap:12px; max-width:1080px;
  margin:0 auto 16px; padding:9px 14px; border-radius:9px;
  background:var(--accent-soft); color:var(--accent); font-size:13px;
}
.filterbar.hide{display:none}
.filterbar button{
  margin-left:auto; border:1px solid currentColor; background:transparent;
  color:inherit; border-radius:6px; padding:3px 10px; cursor:pointer; font-size:12px;
}

/* 章节分隔 */
.chap-div{
  margin:44px auto 6px; padding:0 0 10px;
  border-bottom:2px solid var(--line);
  font-size:13px; color:var(--ink3); scroll-margin-top:70px;
}
.chap-div:first-child{margin-top:6px}
.cd-line{display:flex; align-items:baseline; gap:10px; flex-wrap:wrap}
.cd-num{
  font-size:12px; font-weight:700; color:#fff; background:var(--ink3);
  border-radius:5px; padding:1px 7px;
}
.cd-title{font-size:16px; font-weight:700; color:var(--ink)}
.cd-intro{
  margin-top:10px; padding:10px 14px; border-radius:8px;
  background:var(--panel); border:1px solid var(--line); font-size:13px;
}
.cd-intro blockquote{margin:0; padding:0; border:0; background:none; color:var(--ink2)}
.cd-intro p{margin:0 0 4px}
.cd-intro p:last-child{margin-bottom:0}

/* 小节 */
.sec{
  background:var(--panel); border:1px solid var(--line); border-radius:12px;
  padding:18px 24px 20px; margin:14px auto 0; box-shadow:var(--shadow);
  scroll-margin-top:70px;
}
.sec.hide{display:none}
.sec-head{margin-bottom:12px; padding-bottom:12px; border-bottom:1px solid var(--line2)}
.sec-badges{display:flex; flex-wrap:wrap; align-items:center; gap:7px; margin-bottom:7px}
.badge{
  font-size:11px; padding:1.5px 8px; border-radius:5px; font-weight:600;
  background:var(--bg); color:var(--ink2); border:1px solid var(--line);
  white-space:nowrap;
}
.badge.date{background:var(--accent-soft); color:var(--accent); border-color:transparent; font-variant-numeric:tabular-nums}
.badge.date.range{background:#fff4e5; color:#b45309; border-color:transparent}
html[data-theme="dark"] .badge.date.range{background:#3a2a12; color:#f0b06a}
.badge.date.none{background:var(--bg); color:var(--ink3)}
.badge.chap{color:var(--ink3); font-weight:500}
.badge.num{color:var(--ink3); font-weight:500; font-variant-numeric:tabular-nums}
.sec-anchor{
  margin-left:auto; color:var(--ink3); font-size:13px; opacity:0; transition:.12s;
  padding:0 4px;
}
.sec:hover .sec-anchor{opacity:1}
.sec-anchor:hover{color:var(--accent)}
.sec-title{
  margin:0; font-size:18px; line-height:1.5; font-weight:700; letter-spacing:.01em;
}
.sec-num{
  color:var(--accent); font-variant-numeric:tabular-nums; margin-right:9px;
  font-size:15px; font-weight:700;
}

/* markdown 内容 */
.md{font-size:14.5px; color:var(--ink)}
.md > *:first-child{margin-top:0}
.md > *:last-child{margin-bottom:0}
.md p{margin:0 0 11px}
.md h4{
  font-size:15px; margin:22px 0 10px; padding-left:10px;
  border-left:3px solid var(--accent); line-height:1.4;
}
.md h5{font-size:14px; margin:18px 0 8px; color:var(--ink2)}
.md ul,.md ol{margin:0 0 12px; padding-left:24px}
.md li{margin:3px 0}
.md li > p{margin:0 0 4px}
.md blockquote{
  margin:12px 0; padding:8px 14px; border-left:3px solid var(--line);
  background:var(--bg); border-radius:0 6px 6px 0; color:var(--ink2);
}
.md blockquote p:last-child{margin-bottom:0}
.md hr{border:0; border-top:1px solid var(--line); margin:20px 0}
.md code{
  background:var(--code-bg); color:var(--code-ink); padding:1.5px 5px;
  border-radius:4px; font-size:12.8px; border:1px solid var(--line2);
}
.md pre{
  background:var(--code-bg); border:1px solid var(--line); border-radius:8px;
  padding:12px 14px; overflow-x:auto; margin:12px 0; line-height:1.6;
}
.md pre code{background:none; border:0; padding:0; font-size:12.6px; color:var(--code-ink)}
.tw{overflow-x:auto; margin:12px 0; border:1px solid var(--line); border-radius:8px}
.md table{border-collapse:collapse; width:100%; font-size:13px; background:var(--panel)}
.md th,.md td{border-bottom:1px solid var(--line2); border-right:1px solid var(--line2); padding:6px 10px; text-align:left; vertical-align:top; min-width:4.6em}
.md th{background:var(--bg); font-weight:600; white-space:nowrap}
.md tr:last-child td{border-bottom:0}
.md th:last-child,.md td:last-child{border-right:0}
.md a{word-break:break-word}
.md mark{background:var(--hl); color:inherit; border-radius:2px; padding:0 1px}

.foot{text-align:center; color:var(--ink3); font-size:12px; margin-top:44px}

/* toast */
.toast{
  position:fixed; left:50%; bottom:28px; transform:translate(-50%,14px);
  background:var(--ink); color:var(--panel); padding:8px 16px; border-radius:8px;
  font-size:13px; opacity:0; pointer-events:none; transition:.2s; z-index:99;
}
.toast.on{opacity:.95; transform:translate(-50%,0)}

/* 滚动条 */
::-webkit-scrollbar{width:10px; height:10px}
::-webkit-scrollbar-thumb{background:var(--line); border-radius:6px; border:2px solid transparent; background-clip:content-box}
::-webkit-scrollbar-thumb:hover{background:var(--ink3); background-clip:content-box}
::-webkit-scrollbar-track{background:transparent}

/* 窄屏 */
@media (max-width:900px){
  .side{display:none}
  .main{padding:18px 16px 60px}
  .bt{display:none}
}

/* 打印 */
@media print{
  .top,.side,.strip-card,.filterbar,.sec-anchor{display:none!important}
  body{display:block; overflow:visible; background:#fff}
  .main{overflow:visible; padding:0}
  .sec{border:0; box-shadow:none; break-inside:avoid-page; page-break-inside:avoid}
  .md pre,.tw{overflow:visible}
}
"""

JS = """
(function(){
  var q = document.getElementById('q');
  var qcount = document.getElementById('qcount');
  var nav = document.getElementById('nav');
  var main = document.getElementById('main');
  var secs = Array.prototype.slice.call(document.querySelectorAll('.sec'));
  var items = Array.prototype.slice.call(document.querySelectorAll('.nav-item'));
  var groups = Array.prototype.slice.call(document.querySelectorAll('.day-group'));
  var chips = Array.prototype.slice.call(document.querySelectorAll('.chip'));
  var filterbar = document.getElementById('filterbar');
  var filtertext = document.getElementById('filtertext');
  var toast = document.getElementById('toast');

  var state = { q: '', date: null };
  // 缓存正文小写文本，避免每次按键都读 DOM
  var hay = {};
  secs.forEach(function(s){ hay[s.id] = (s.textContent || '').toLowerCase(); });

  function showToast(msg){
    toast.textContent = msg;
    toast.classList.add('on');
    clearTimeout(showToast._t);
    showToast._t = setTimeout(function(){ toast.classList.remove('on'); }, 1400);
  }

  function apply(){
    var terms = state.q.trim().toLowerCase().split(/\\s+/).filter(Boolean);
    var shown = 0;

    secs.forEach(function(s){
      var okDate = !state.date || s.dataset.date === state.date;
      var okQ = true;
      if (terms.length){
        var t = hay[s.id] || '';
        okQ = terms.every(function(w){ return t.indexOf(w) >= 0; });
      }
      var vis = okDate && okQ;
      s.classList.toggle('hide', !vis);
      if (vis) shown++;
    });

    items.forEach(function(li){
      var okDate = !state.date || li.dataset.date === state.date;
      // 与正文同一套判据（含小节全文），避免侧栏命中数比正文少
      var t = hay[li.dataset.target] || '';
      var okQ = terms.length === 0 || terms.every(function(w){ return t.indexOf(w) >= 0; });
      li.classList.toggle('hide', !(okDate && okQ));
    });

    groups.forEach(function(g){
      var any = g.querySelector('.nav-item:not(.hide)');
      g.classList.toggle('hide', !any);
      if (any && (terms.length || state.date)) g.classList.add('open');
    });

    chips.forEach(function(c){
      c.classList.toggle('on', state.date === c.dataset.date);
    });

    if (state.date){
      var label = state.date === 'none' ? '常驻章节'
        : (function(){
            var p = state.date.split('-');
            return p[1] + '/' + p[2];
          })();
      filtertext.textContent = '只看 ' + label + ' · ' + shown + ' 节';
      filterbar.classList.remove('hide');
    } else {
      filterbar.classList.add('hide');
    }

    if (terms.length || state.date){
      qcount.textContent = shown + ' 节命中';
    } else {
      qcount.textContent = '';
    }
  }

  var t;
  q.addEventListener('input', function(){
    clearTimeout(t);
    t = setTimeout(function(){ state.q = q.value; apply(); }, 110);
  });
  q.addEventListener('keydown', function(e){
    if (e.key === 'Enter'){
      var first = document.querySelector('.sec:not(.hide)');
      if (first) first.scrollIntoView({behavior:'smooth', block:'start'});
    }
    if (e.key === 'Escape'){
      q.value = ''; state.q = ''; apply();
    }
  });

  document.addEventListener('keydown', function(e){
    if (e.key === '/' && document.activeElement !== q){
      e.preventDefault(); q.focus(); q.select();
    }
  });

  chips.forEach(function(c){
    c.addEventListener('click', function(){
      var d = c.dataset.date;
      state.date = (state.date === d) ? null : d;
      apply();
      var first = document.querySelector('.sec:not(.hide)');
      if (state.date && first) first.scrollIntoView({behavior:'smooth', block:'start'});
    });
  });

  document.getElementById('clearfilter').addEventListener('click', function(){
    state.date = null; state.q = ''; q.value = ''; apply();
  });

  // 折叠
  groups.forEach(function(g){
    var head = g.querySelector('.day-head');
    head.addEventListener('click', function(){ g.classList.toggle('open'); });
  });

  // 目录点击：平滑滚动（不依赖 CSS scroll-behavior，避免深链首次加载被动画拖住）
  function goTo(id, smooth){
    var el = document.getElementById(id);
    if (!el) return;
    if (el.classList.contains('hide')){
      state.date = null; state.q = ''; q.value = ''; apply();
    }
    el.scrollIntoView({behavior: smooth ? 'smooth' : 'auto', block: 'start'});
    history.replaceState(null, '', '#' + id);
  }

  items.forEach(function(li){
    var a = li.querySelector('a');
    a.addEventListener('click', function(e){
      e.preventDefault();
      goTo(li.dataset.target, true);
    });
  });

  // 锚点复制
  document.querySelectorAll('.sec-anchor').forEach(function(a){
    a.addEventListener('click', function(e){
      e.preventDefault();
      var url = location.origin + location.pathname + a.getAttribute('href');
      var done = function(){ showToast('已复制本节链接'); };
      if (navigator.clipboard && navigator.clipboard.writeText){
        navigator.clipboard.writeText(url).then(done, function(){ showToast(url); });
      } else {
        showToast(url);
      }
      history.replaceState(null, '', a.getAttribute('href'));
    });
  });

  // 主题
  var themeBtn = document.getElementById('theme');
  var saved = localStorage.getItem('kwq3-theme');
  if (saved) document.documentElement.dataset.theme = saved;
  themeBtn.addEventListener('click', function(){
    var cur = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
    document.documentElement.dataset.theme = cur;
    localStorage.setItem('kwq3-theme', cur);
  });

  // 滚动高亮
  var activeId = null;
  function setActive(id){
    if (id === activeId) return;
    activeId = id;
    var hit = null;
    items.forEach(function(li){
      var on = li.dataset.target === id;
      li.classList.toggle('active', on);
      if (on) hit = li;
    });
    if (hit){
      var g = hit.closest('.day-group');
      if (g && !g.classList.contains('open')) g.classList.add('open');
      var r = hit.getBoundingClientRect();
      var nr = nav.getBoundingClientRect();
      if (r.top < nr.top + 40 || r.bottom > nr.bottom - 40){
        hit.scrollIntoView({block:'center'});
      }
    }
  }

  if ('IntersectionObserver' in window){
    var io = new IntersectionObserver(function(entries){
      var best = null;
      entries.forEach(function(en){
        if (en.isIntersecting){
          if (!best || en.boundingClientRect.top < best.boundingClientRect.top) best = en;
        }
      });
      if (best) setActive(best.target.id);
    }, { root: main, rootMargin: '-64px 0px -68% 0px', threshold: 0 });
    secs.forEach(function(s){ io.observe(s); });
  }

  // 深链进入：立即定位（若目标被筛选隐藏则先清筛选）
  if (location.hash && location.hash.length > 1){
    goTo(location.hash.slice(1), false);
  }

  apply();
})();
"""


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=DEFAULT_SRC)
    ap.add_argument("--out", default=DEFAULT_OUT)
    args = ap.parse_args()

    with open(args.src, encoding="utf-8") as f:
        text = f.read()

    preamble, units = prepare(text)

    # 标题：取 preamble 里的 `# xxx`
    title = "修改日志"
    docinfo_lines = []
    for ln in preamble:
        if ln.startswith("# "):
            title = ln[2:].strip()
        else:
            docinfo_lines.append(ln)
    docinfo_src = "\n".join(docinfo_lines).strip()

    # 渲染正文
    content_parts = []
    for u in units:
        u.body = md2html("\n".join(u.lines).strip(), u.uid) if u.has_body else ""
        if u.level == 2 and (u.has_children or not u.has_body):
            content_parts.append(render_chapter_divider(u))
        else:
            content_parts.append(render_section(u))

    nav_html, strip_html, days_meta = build_nav(units)
    sec_count = sum(
        1 for u in units if u.has_body and not (u.level == 2 and u.has_children)
    )

    stats = f"{sec_count} 节 · {len(days_meta)} 个日期"
    generated = _dt.datetime.now().strftime("%Y-%m-%d %H:%M")

    out = TEMPLATE
    out = out.replace("@@TITLE@@", html.escape(title))
    out = out.replace("@@CSS@@", CSS)
    out = out.replace("@@JS@@", JS)
    out = out.replace("@@NAV@@", nav_html)
    out = out.replace("@@STRIP@@", strip_html)
    out = out.replace("@@STATS@@", stats)
    out = out.replace("@@COUNT@@", str(sec_count))
    out = out.replace("@@GENERATED@@", generated)
    out = out.replace(
        "@@DOCINFO@@",
        md2html(docinfo_src, "docinfo", nl2br=True) if docinfo_src else "",
    )
    out = out.replace("@@CONTENT@@", "\n".join(content_parts))

    with open(args.out, "w", encoding="utf-8") as f:
        f.write(out)

    # 简单自检
    missing = [
        u.num for u in units
        if u.has_body and f'id="{u.uid}"' not in out
    ]
    print(f"[OK] {args.out}")
    print(f"     小节 {sec_count} 个 / 日期 {len(days_meta)} 个 / "
          f"{len(out) / 1024:.0f} KB")
    if missing:
        print(f"[!] 有 {len(missing)} 节未写入：{missing[:8]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
