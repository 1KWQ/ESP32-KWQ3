/**
 * @file version.cpp 
 * @author your name (you@domain.com)
 * @brief  编译日期时间版本信息获取
 * @version 0.1
 * @date 2026-03-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "version.h"
#include <string.h>
#include <stdlib.h>

/**
 * @brief  解析编译日期字符串 __DATE__，提取年月日
 * 
 * @param Year  年份指针
 * @param Month 月份指针
 * @param Day  日期指针
 */
void Get_Compile_Date_Base(uint8_t *Year, uint8_t *Month, uint8_t *Day)
{
	//Sep 11 2023
	const char *pMonth[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
	const char Date[12] = __DATE__;//取编译日期
	uint8_t i;
	for(i = 0; i < 12; i++)if(memcmp(Date, pMonth[i], 3) == 0)*Month = i + 1, i = 12;
	*Year = (uint8_t)atoi(Date + 9); //Date[9]为２位年份，Date[7]为完整年份
	*Day = (uint8_t)atoi(Date + 4);
}


/**
 * @brief  解析编译时间字符串 __TIME__，提取时分秒
 * 
 * @param HH  小时指针
 * @param MM  分钟指针
 * @param SS  秒钟指针
 */
void Get_Compile_Time_Base(uint8_t *HH, uint8_t *MM, uint8_t *SS)
{
	//15:40:23
	const char Time[10] = __TIME__;//取编译时间
	*HH = (uint8_t)atoi(Time + 0);
	*MM = (uint8_t)atoi(Time + 3);
	*SS = (uint8_t)atoi(Time + 6);
}

char g_date_buf[10];
char g_time_buf[10];
char g_version_buf[20];


/**
 * @brief  获取编译日期字符串，格式为 "YYYY-MM-DD"
 * 
 * @return char* 
 */
char* Get_Compile_Date(void)
{
	uint8_t  Year, Month, Day;
	Get_Compile_Date_Base(&Year, &Month, &Day);//取编译日期
	sprintf(g_date_buf, "20%02d-%02d-%02d", Year, Month, Day);//任意格式化
	return g_date_buf;
}

/**
 * @brief  获取编译时间字符串，格式为 "HH:MM:SS"
 * 
 * @return char* 编译时间字符串指针
 */
char* Get_Compile_Time(void)
{
	uint8_t  HH, MM, SS;
	Get_Compile_Time_Base(&HH, &MM, &SS);//取编译时间
	sprintf(g_time_buf, "%02d:%02d:%02d", HH, MM, SS);//任意格式化
	return g_time_buf;
}

/**
 * @brief  获取编译版本字符串，格式为 "YYYY-MM-DD-HH-MM-SS"
 * 
 * @return char*    编译版本字符串指针
 */
char* Get_Compile_Ver(void)
{
	uint8_t  Year, Month, Day;
	uint8_t  HH, MM, SS;
	Get_Compile_Date_Base(&Year, &Month, &Day);//取编译日期
	Get_Compile_Time_Base(&HH, &MM, &SS);//取编译时间
	sprintf(g_version_buf, "20%02d-%02d-%02d-%02d-%02d-%02d",Year, Month, Day, HH, MM, SS);//任意格式化
	return g_version_buf;
}
