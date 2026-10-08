

#include "temp.h"

#include "pca_gpio.h"
#include "exio.h"
extern PCAGPIO pca; // define at main.cpp

/**
 公式法：
NTC 热敏电阻温度计算公式：Rt = Rp *EXP(B*(1/T1-1/T2))
可以得到温度T1与电阻Rt的关系T1=1/（log（Rt/Rp）/B+1/T2）

T1和T2指的是K度，即开尔文温度,K度=273.15(绝对温度)+摄氏度。
T2=(273.15+25)
Rt 是热敏电阻在T1温度下的阻值。
Rp 是热敏电阻在T2常温下的标称阻值。100K的热敏电阻25℃的值为100K（即R=100K）。T2=(273.15+25)
B值是热敏电阻的重要参数  B25/50= 3950K ±1%
最终对应的摄氏温度t=T1-273.15,同时+0.5的误差矫正。
ADC采样率12位 串联电阻R1=10K R2=热敏电阻 电压3.3V
已知
Rt = R*(3.3-VR)/VR
VR = 3.3*ADC_Value/4095
得出
ADC_Value = VR*4095/3.3 =3.3*R/(Rt+R)*4095/3.3 = R/(Rt+R)*1024
最终根据Rt算处温度值
Rt=100*exp(3950*(1/(273.15+T1)-1/(273.15+25)))
*/

/**
 * @brief 阻值转换温度
 *
 * @param Rt 热敏电阻阻值
 * @return float 返回摄氏度温度
 */
float em_temp_calculate(float Rt)
{
    float T2 = 273.15 + 25; 
    
    float Rp = 10000; // 10k NTC常温阻值
    float Bx = 3950;  // B值
    float Ka = 273.15;
    float temp = 0.0f;

    temp = 1 / (log(Rt / Rp) / Bx + 1 / T2) - Ka + 0.5;
    return temp;
}


/*
CD4051BM96
 A - pca IO1
 B - pca IO0
 C - pca IO8
*/ 

/**
 * @brief Get the adc temperatrue object
 * 
 * @param ch CD4051BM96 通道 [1...5]
 * @return float  温度值
 */
float get_adc_temperatrue(uint8_t ch)
{

    switch (ch)  // 控制 CD4051BM 选择具体通道
    {
    case 1: // 
        pca.digitalWrite(CD4015_C, 0); // C
        pca.digitalWrite(CD4015_B, 1); // B
        pca.digitalWrite(CD4015_A, 0); // A
        break;
    case 2:
        pca.digitalWrite(CD4015_C, 0); // C
        pca.digitalWrite(CD4015_B, 0); // B
        pca.digitalWrite(CD4015_A, 1); // A
        break;
    case 3:
        pca.digitalWrite(CD4015_C, 0); // C
        pca.digitalWrite(CD4015_B, 0); // B
        pca.digitalWrite(CD4015_A, 0); // A
        break;
    case 4:
        pca.digitalWrite(CD4015_C, 0); // C
        pca.digitalWrite(CD4015_B, 1); // B
        pca.digitalWrite(CD4015_A, 1); // A
        break;
    case 5:
        pca.digitalWrite(CD4015_C, 1); // C
        pca.digitalWrite(CD4015_B, 0); // B
        pca.digitalWrite(CD4015_A, 0); // A
        break;
    case 6:  // 电阻检测通道
        pca.digitalWrite(CD4015_C, 1); // C
        pca.digitalWrite(CD4015_B, 1); // B
        pca.digitalWrite(CD4015_A, 0); // A
        break;
    default:
        break;
    }
    delay(10);

    float temp = 0.0f;
    float Rt = 0;
    float vol = 0;
    // 直接读取电压 mv -> v
    vol = analogReadMilliVolts(TEM_PIN) / 1000.0;
    // 电压转换为阻值 原理图为10k 1%精度  vol/VCC=Rt/(R+Rt)  vol/3.3=Rt/(10000+Rt)
    Rt = (vol * 10000) / (3.3 - vol);

    if(ch == 6){
       // ESP_LOGI("RESISTOR", "vol = %.2f", vol);
        // 电阻检测
        float r;
         // 电路放大了10倍，这里需要电压缩小，1/10，再除以电流100ma
        r = (vol / 1.0) / 0.1;  // 
        return r;

    }

    temp = em_temp_calculate(Rt);
    return temp;
}

/**
 * @brief ADC 初始化
 * 
 */
void adc_init()
{
    analogReadResolution(TEMPERATRUE_ADC_BIT);
}


/*
    float a = get_adc_temperatrue(1);
    Serial.printf("T:%.2f \n", a);
*/