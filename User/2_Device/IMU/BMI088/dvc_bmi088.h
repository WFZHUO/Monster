/**
 * @file dvc_bmi088.h
 * @author WFZ
 * @brief  BMI088 IMU
 * @version 0.0
 * @date 2026-01-03
 *
 *
 */

#ifndef __DVC_BMI088_H
#define __DVC_BMI088_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include <string.h>
#include "dvc_buzzer.h"
#include "drv_math.h"

/* Exported macros -----------------------------------------------------------*/

// BMI088 设备ID发送寄存器地址
#define BMI088_ACC_CHIP_ID          0x00
#define BMI088_GYRO_CHIP_ID         0x0F

// BMI088 加速度计寄存器地址
#define BMI088_ACC_CHIP_ID_ADDR     0x00
#define BMI088_ACC_ERR_REG_ADDR     0x02
#define BMI088_ACC_STATUS_ADDR      0x03
#define BMI088_ACC_X_LSB_ADDR       0x12
#define BMI088_ACC_X_MSB_ADDR       0x13
#define BMI088_ACC_Y_LSB_ADDR       0x14
#define BMI088_ACC_Y_MSB_ADDR       0x15
#define BMI088_ACC_Z_LSB_ADDR       0x16
#define BMI088_ACC_Z_MSB_ADDR       0x17
#define BMI088_SENSORTIME_0_ADDR    0x18
#define BMI088_SENSORTIME_1_ADDR    0x19
#define BMI088_SENSORTIME_2_ADDR    0x1A
#define BMI088_ACC_INT_STAT_1_ADDR  0x1D
#define BMI088_TEMP_MSB_ADDR        0x22
#define BMI088_TEMP_LSB_ADDR        0x23
#define BMI088_ACC_CONF_ADDR        0x40
#define BMI088_ACC_RANGE_ADDR       0x41
#define BMI088_INT1_IO_CTRL_ADDR    0x53
#define BMI088_INT2_IO_CTRL_ADDR    0x54
#define BMI088_ACC_SELF_TEST_ADDR   0x6D
#define BMI088_ACC_PWR_CONF_ADDR    0x7C
#define BMI088_ACC_PWR_CTRL_ADDR    0x7D
#define BMI088_ACC_SOFT_RESET_ADDR  0x7E

// BMI088 陀螺仪寄存器地址
#define BMI088_GYRO_CHIP_ID_ADDR    0x00
#define BMI088_GYRO_X_LSB_ADDR      0x02
#define BMI088_GYRO_X_MSB_ADDR      0x03
#define BMI088_GYRO_Y_LSB_ADDR      0x04
#define BMI088_GYRO_Y_MSB_ADDR      0x05
#define BMI088_GYRO_Z_LSB_ADDR      0x06
#define BMI088_GYRO_Z_MSB_ADDR      0x07
#define BMI088_GYRO_RANGE_ADDR      0x0F
#define BMI088_GYRO_BANDWIDTH_ADDR  0x10
#define BMI088_GYRO_LPM1_ADDR       0x11
#define BMI088_GYRO_SOFTRESET_ADDR  0x14
#define BMI088_GYRO_INT_CTRL_ADDR   0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF_ADDR 0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP_ADDR  0x18

// BMI088 命令
#define BMI088_ACC_SOFTRESET_CMD    0xB6
#define BMI088_GYRO_SOFTRESET_CMD   0xB6

// BMI088 加速度计量程配置
#define BMI088_ACC_RANGE_3G         0x00  // ±3g
#define BMI088_ACC_RANGE_6G         0x01  // ±6g
#define BMI088_ACC_RANGE_12G        0x02  // ±12g
#define BMI088_ACC_RANGE_24G        0x03  // ±24g

// BMI088 陀螺仪量程配置
#define BMI088_GYRO_RANGE_2000      0x00  // ±2000°/s
#define BMI088_GYRO_RANGE_1000      0x01  // ±1000°/s
#define BMI088_GYRO_RANGE_500       0x02  // ±500°/s
#define BMI088_GYRO_RANGE_250       0x03  // ±250°/s
#define BMI088_GYRO_RANGE_125       0x04  // ±125°/s

// BMI088 加速度计带宽配置
#define BMI088_ACC_BW_12_5          0x05  // 12.5 Hz
#define BMI088_ACC_BW_25            0x06  // 25 Hz
#define BMI088_ACC_BW_50            0x07  // 50 Hz
#define BMI088_ACC_BW_100           0x08  // 100 Hz
#define BMI088_ACC_BW_200           0x09  // 200 Hz
#define BMI088_ACC_BW_400           0x0A  // 400 Hz
#define BMI088_ACC_BW_800           0x0B  // 800 Hz
#define BMI088_ACC_BW_1600          0x0C  // 1600 Hz

// BMI088 陀螺仪带宽配置
#define BMI088_GYRO_BW_2000         0x00  // 2000 Hz (ODR 2000 Hz)
#define BMI088_GYRO_BW_1000         0x01  // 1000 Hz (ODR 1000 Hz)
#define BMI088_GYRO_BW_400          0x02  // 400 Hz  (ODR 400 Hz)
#define BMI088_GYRO_BW_200          0x03  // 200 Hz  (ODR 200 Hz)
#define BMI088_GYRO_BW_100          0x04  // 100 Hz  (ODR 100 Hz)
#define BMI088_GYRO_BW_50           0x05  // 50 Hz   (ODR 50 Hz)

// 单位转换常量
#define DEG_TO_RAD (PI / 180.0f)  // 度转弧度
#define GRAVITY                     9.80665f  // m/s²

// 片选引脚定义 (根据实际硬件连接修改)
#define BMI088_ACC_CS_PORT          GPIOA
#define BMI088_ACC_CS_PIN           GPIO_PIN_4
#define BMI088_GYRO_CS_PORT         GPIOB
#define BMI088_GYRO_CS_PIN          GPIO_PIN_0

// SPI超时时间
#define BMI088_SPI_TIMEOUT          10  // ms

/* Exported types ------------------------------------------------------------*/

// BMI088 配置结构体
typedef struct {
    uint8_t acc_range;      // 加速度计量程
    uint8_t acc_bw;         // 加速度计带宽
    uint8_t gyro_range;     // 陀螺仪量程
    uint8_t gyro_bw;        // 陀螺仪带宽
} BMI088_Config_t;

// BMI088 原始数据
typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temp;
} BMI088_RawData_t;

// BMI088 处理后的数据
typedef struct {
    float acc_x;           // m/s²
    float acc_y;           // m/s²
    float acc_z;           // m/s²
    float gyro_x;          // rad/s
    float gyro_y;          // rad/s
    float gyro_z;          // rad/s
    float temperature;     // °C
} BMI088_Data_t;

/**
 * @brief BMI088类
 * 
 */
class Class_BMI088
{
public:

    void Init(void);

    uint8_t CheckConnection(void);

    void Calibrate(uint16_t sample_count);

    void TIM_Calculate_PeriodElapsedCallback();

    inline float Get_Gyro_X(void);

    inline float Get_Gyro_Y(void);

    inline float Get_Gyro_Z(void);

    inline float Get_Acc_X(void);

    inline float Get_Acc_Y(void);

    inline float Get_Acc_Z(void);

    inline float Get_Temperature(void);

private:

    //配置加速度/陀螺仪的量程/带宽
    BMI088_Config_t Config = {
        BMI088_ACC_RANGE_12G,     // ±12g（降低量程，提高分辨率）
        BMI088_ACC_BW_200,           // 200Hz（降低带宽，减少噪声）
        BMI088_GYRO_RANGE_1000,  // ±1000°/s（降低量程，提高分辨率）
        BMI088_GYRO_BW_400          // 400Hz（平衡响应和噪声）
    };

    //加速度，角速度，温度原始数据
    BMI088_RawData_t RawData;

    //加速度，角速度，温度处理后的数据
    BMI088_Data_t Data;

    //上一时刻陀螺仪数据
    float prev_gyro_x = 0.0f;
    float prev_gyro_y = 0.0f;
    float prev_gyro_z = 0.0f;

    //上一时刻加速度计数据
    float prev_acc_x = 0.0f;
    float prev_acc_y = 0.0f;
    float prev_acc_z = 0.0f;

    // 上一次的陀螺仪滤波值
    float prev_filtered_gyro_x = 0.0f;
    float prev_filtered_gyro_y = 0.0f;
    float prev_filtered_gyro_z = 0.0f;

    // 上一次的加速度计滤波值
    float prev_filtered_acc_x = 0.0f;
    float prev_filtered_acc_y = 0.0f;
    float prev_filtered_acc_z = 0.0f;

    // 滤波相关变量
    // 常规滤波系数
    float Filter_Alpha_Light = 0.2f;    

    // 陀螺仪异常值阈值 (rad/s)，根据实际情况调整
    float Gyro_Outlier_Threshold = 10.0f;
    
    // 加速度异常值阈值 (m/s²)，根据实际情况调整
    float Acc_Outlier_Threshold = 50.0f;

    // 加速度计灵敏度常量 (根据量程设置)
    float acc_sensitivity = 0.0f;    // LSB/g

    // 陀螺仪灵敏度常量 (根据量程设置)
    float gyro_sensitivity = 0.0f;   // LSB/(°/s)

    // 陀螺仪x轴校准偏移
    float gyro_offset_x = 0.0f;

    // 陀螺仪y轴校准偏移
    float gyro_offset_y = 0.0f;

    // 陀螺仪z轴校准偏移
    float gyro_offset_z = 0.0f;

    // 加速度计x轴校准偏移
    float acc_offset_x = 0.0f;

    // 加速度计y轴校准偏移
    float acc_offset_y = 0.0f;

    // 加速度计z轴校准偏移
    float acc_offset_z = 0.0f;

    void AccChipSelect(uint8_t state);

    void GyroChipSelect(uint8_t state);

    uint8_t ReadAccReg(uint8_t reg_addr);

    void WriteAccReg(uint8_t reg_addr, uint8_t data);

    uint8_t ReadGyroReg(uint8_t reg_addr);

    void WriteGyroReg(uint8_t reg_addr, uint8_t data);

    void ReadAccRegs(uint8_t reg_addr, uint8_t *data, uint16_t len);

    void ReadGyroRegs(uint8_t reg_addr, uint8_t *data, uint16_t len);

    void SetAccSensitivity(void);

    void SetGyroSensitivity(void);

    void GetRawData(void);

    void GetData(void);

    void ErrorHandler(void);

    void Filter_data(void);
};

/* Exported variables --------------------------------------------------------*/

extern SPI_HandleTypeDef hspi1; // BMI088 SPI句柄

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取陀螺仪X轴数据
 * @return float 陀螺仪X轴数据 (rad/s)
 */
inline float Class_BMI088::Get_Gyro_X(void) 
{ 
    return Data.gyro_x; 
}

/**
 * @brief 获取陀螺仪Y轴数据
 * @return float 陀螺仪Y轴数据 (rad/s)
 */
inline float Class_BMI088::Get_Gyro_Y(void) 
{ 
    return Data.gyro_y; 
}

/**
 * @brief 获取陀螺仪Z轴数据
 * @return float 陀螺仪Z轴数据 (rad/s)
 */
inline float Class_BMI088::Get_Gyro_Z(void) 
{ 
    return Data.gyro_z; 
}

/**
 * @brief 获取加速度计X轴数据
 * @return float 加速度计X轴数据 (m/s²)
 */
inline float Class_BMI088::Get_Acc_X(void) 
{ 
    return Data.acc_x; 
}

/**
 * @brief 获取加速度计Y轴数据
 * @return float 加速度计Y轴数据 (m/s²)
 */
inline float Class_BMI088::Get_Acc_Y(void) 
{ 
    return Data.acc_y; 
}

/**
 * @brief 获取加速度计Z轴数据
 * @return float 加速度计Z轴数据 (m/s²)
 */
inline float Class_BMI088::Get_Acc_Z(void) 
{ 
    return Data.acc_z; 
}

/**
 * @brief 获取温度数据
 * @return float 温度数据 (°C)
 */
inline float Class_BMI088::Get_Temperature(void) 
{ 
    return Data.temperature; 
}

#endif

