/**
 * @file dvc_MPU6050.c
 * @author wfz 
 * @brief MPU6050陀螺仪
 * @version 0.0
 * @date 2026-1-4
 *
 */

#include "dvc_MPU6050.h"
#include <math.h>

#define MPU6050_ADDRESS    0x68  // 7位地址，HAL 库会自动左移一位

// 换算常数
#define GRAVITY 9.80665f        // 重力加速度，m/s²
#define DEG_TO_RAD (3.14159265358979f / 180.0f)  // 度转弧度

// 零点校准相关变量
static float gyro_z_offset = 0.0f;

// 陀螺仪滤波相关变量
#define FILTER_ALPHA_LIGHT 0.2f    // 常规滤波系数
#define FILTER_ALPHA_HEAVY 0.05f   // 强滤波系数
#define OUTLIER_THRESHOLD 10.0f    // 异常值阈值 (rad/s)，根据实际情况调整


/**
  * @brief  向 MPU6050 寄存器写入数据
  * @param  RegAddress: 寄存器地址
  * @param  Data: 要写入的数据
  * @retval None
  */
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    HAL_I2C_Mem_Write(&hi2c2, 
                      MPU6050_ADDRESS << 1,  // 左移一位，加上写位
                      RegAddress,
                      I2C_MEMADD_SIZE_8BIT,
                      &Data,
                      1,
                      1000);  // 超时时间 1000ms
}

/**
  * @brief  从 MPU6050 寄存器读取数据
  * @param  RegAddress: 寄存器地址
  * @retval 读取到的数据
  */
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data = 0;
    
    HAL_I2C_Mem_Read(&hi2c2,
                     MPU6050_ADDRESS << 1,  // 左移一位，加上读位
                     RegAddress,
                     I2C_MEMADD_SIZE_8BIT,
                     &Data,
                     1,
                     1000);  // 超时时间 1000ms
    
    return Data;
}

/**
  * @brief  MPU6050 初始化
  * @retval None
  */
void MPU6050_Init(void)
{
    uint8_t retry = 0;
    
    // 确保I2C通信正常
    while(retry < 3)
    {
        // 1. 解除睡眠模式，选择陀螺仪时钟源
        MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
        HAL_Delay(10);
        
        // 2. 六个轴均不待机
        MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
        HAL_Delay(10);
        
			// 3. 采样分频为0，采样频率为1000Hz
        MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x00);
        HAL_Delay(10);
        
			// 4. 滤波参数不给
        MPU6050_WriteReg(MPU6050_CONFIG, 0x00);
        HAL_Delay(10);
        
        // 5. 陀螺仪满量程 ±2000°/s
        MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
        HAL_Delay(10);
        
        // 6. 加速度计满量程 ±16g
        MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
        HAL_Delay(10);
        
        // 7. 验证配置是否写入成功
        uint8_t gyro_config = MPU6050_ReadReg(MPU6050_GYRO_CONFIG);
        uint8_t accel_config = MPU6050_ReadReg(MPU6050_ACCEL_CONFIG);
        
        if((gyro_config & 0x18) == 0x18 && (accel_config & 0x18) == 0x18)
        {
            // 配置成功
            break;
        }
        
        retry++;
        HAL_Delay(50);
    }
    
    if(retry >= 3)
    {
			for(int i=0;i<2;i++)
			{
				dvc_buzzer_SetOn();
				HAL_Delay(100);
				dvc_buzzer_SetOff();
				HAL_Delay(100);
			}

			HAL_NVIC_SystemReset();
    }

    // 等待传感器稳定
    HAL_Delay(100);
    
    // 上电自动零点校准（500次采样，约1秒）
    float sum = 0.0f;
    uint16_t calibration_samples = 500;

    for(uint16_t i = 0; i < calibration_samples; i++)
    {
        float raw_acc_x, raw_acc_y, raw_acc_z;
        float raw_gyro_x, raw_gyro_y, raw_gyro_z;
        
        MPU6050_GetData(&raw_acc_x, &raw_acc_y, &raw_acc_z, 
                           &raw_gyro_x, &raw_gyro_y, &raw_gyro_z);
        
        // 累积Z轴陀螺仪原始数据
        sum += (float)raw_gyro_z;
        
        HAL_Delay(2);  // 2ms间隔，总共约1秒
    }
    
    // 计算Z轴陀螺仪零点偏移值
    gyro_z_offset = sum / (float)calibration_samples;
}
/**
  * @brief  获取 MPU6050 设备 ID
  * @retval 设备 ID
  */
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

/**
  * @brief  获取 MPU6050 数据（一次读取所有数据，效率更高）
  * @param  AccX, AccY, AccZ: 加速度计 XYZ 轴数据
  * @param  GyroX, GyroY, GyroZ: 陀螺仪 XYZ 轴数据
  * @retval None
  */
void MPU6050_GetRawData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                         int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    uint8_t buffer[14];
    
    // 从加速度计 X 轴高字节开始，连续读取 14 个字节
    HAL_I2C_Mem_Read(&hi2c2,
                     MPU6050_ADDRESS << 1,
                     MPU6050_ACCEL_XOUT_H,
                     I2C_MEMADD_SIZE_8BIT,
                     buffer,
                     14,
                     1000);
    
    // 解析加速度计数据
    *AccX = (buffer[0] << 8) | buffer[1];
    *AccY = (buffer[2] << 8) | buffer[3];
    *AccZ = (buffer[4] << 8) | buffer[5];
    
    // 温度数据（buffer[6] 和 buffer[7]）
    
    // 解析陀螺仪数据
    *GyroX = (buffer[8] << 8) | buffer[9];
    *GyroY = (buffer[10] << 8) | buffer[11];
    *GyroZ = (buffer[12] << 8) | buffer[13];
}

/**
  * @brief  获取换算后的物理量数据
  * @param  AccX, AccY, AccZ: 加速度 XYZ 轴 (m/s²)
  * @param  GyroX, GyroY, GyroZ: 陀螺仪 XYZ 轴 (rad/s)
  * @retval None
  */
void MPU6050_GetData(float *AccX, float *AccY, float *AccZ,
                          float *GyroX, float *GyroY, float *GyroZ)
{
    int16_t accelRaw[3], gyroRaw[3];
    
    // 使用高效的方式获取原始数据
    MPU6050_GetRawData(&accelRaw[0], &accelRaw[1], &accelRaw[2],
                        &gyroRaw[0], &gyroRaw[1], &gyroRaw[2]);
    
    // 根据初始化配置进行换算：
    // 加速度计量程：±16g，灵敏度：2048 LSB/g
    // 陀螺仪量程：±2000°/s，灵敏度：16.4 LSB/(°/s)
    
    // 加速度换算：原始数据 -> g -> m/s²
    // g = raw / 2048
    // m/s² = g * 9.80665
    *AccX = ((float)accelRaw[0] / 2048.0f) * GRAVITY;
    *AccY = ((float)accelRaw[1] / 2048.0f) * GRAVITY;
    *AccZ = ((float)accelRaw[2] / 2048.0f) * GRAVITY;
    
    // 陀螺仪换算：原始数据 -> °/s -> rad/s
    // °/s = raw / 16.4
    // rad/s = °/s * (π/180)
    *GyroX = ((float)gyroRaw[0] / 16.4f) * DEG_TO_RAD;
    *GyroY = ((float)gyroRaw[1] / 16.4f) * DEG_TO_RAD;

    //z轴应用零点补偿
    *GyroZ = ((float)gyroRaw[2] / 16.4f) * DEG_TO_RAD - gyro_z_offset;
}
/*
动态检测量程并自适应计算
void MPU6050_GetData(float *AccX, float *AccY, float *AccZ,
                     float *GyroX, float *GyroY, float *GyroZ)
{
    int16_t accelRaw[3], gyroRaw[3];
    
    // 读取原始数据
    MPU6050_GetRawData(&accelRaw[0], &accelRaw[1], &accelRaw[2],
                       &gyroRaw[0], &gyroRaw[1], &gyroRaw[2]);
    
    // 读取当前陀螺仪量程设置
    uint8_t gyro_range = MPU6050_ReadReg(MPU6050_GYRO_CONFIG) & 0x18;
    float gyro_sensitivity;
    
    switch(gyro_range)
    {
        case 0x00:  // ±250°/s
            gyro_sensitivity = 131.0f;  // LSB/(°/s)
            break;
        case 0x08:  // ±500°/s
            gyro_sensitivity = 65.5f;   // LSB/(°/s)
            break;
        case 0x10:  // ±1000°/s
            gyro_sensitivity = 32.8f;   // LSB/(°/s)
            break;
        case 0x18:  // ±2000°/s
            gyro_sensitivity = 16.4f;   // LSB/(°/s)
            break;
        default:    // 默认为 ±2000°/s
            gyro_sensitivity = 16.4f;
            break;
    }
    
    // 读取当前加速度计量程设置
    uint8_t accel_range = MPU6050_ReadReg(MPU6050_ACCEL_CONFIG) & 0x18;
    float accel_sensitivity;
    
    switch(accel_range)
    {
        case 0x00:  // ±2g
            accel_sensitivity = 16384.0f;  // LSB/g
            break;
        case 0x08:  // ±4g
            accel_sensitivity = 8192.0f;   // LSB/g
            break;
        case 0x10:  // ±8g
            accel_sensitivity = 4096.0f;   // LSB/g
            break;
        case 0x18:  // ±16g
            accel_sensitivity = 2048.0f;   // LSB/g
            break;
        default:    // 默认为 ±16g
            accel_sensitivity = 2048.0f;
            break;
    }
    
    // 加速度换算：原始数据 -> g -> m/s²
    *AccX = ((float)accelRaw[0] / accel_sensitivity) * GRAVITY;
    *AccY = ((float)accelRaw[1] / accel_sensitivity) * GRAVITY;
    *AccZ = ((float)accelRaw[2] / accel_sensitivity) * GRAVITY;
    
    // 陀螺仪换算：原始数据 -> °/s -> rad/s
    *GyroX = ((float)gyroRaw[0] / gyro_sensitivity) * DEG_TO_RAD;
    *GyroY = ((float)gyroRaw[1] / gyro_sensitivity) * DEG_TO_RAD;
    *GyroZ = ((float)gyroRaw[2] / gyro_sensitivity) * DEG_TO_RAD;
}
*/

// 陀螺仪滤波函数
float filter_gyro_data(float raw_gyro_z, float prev_filtered) {
    static float prev_raw = 0.0f;
    
    // 1. 异常值检测
    if (fabsf(raw_gyro_z - prev_raw) > OUTLIER_THRESHOLD) {
        // 数据突变过大，认为是异常值，使用上一次的有效值
        raw_gyro_z = prev_raw;
    }
    
    // 2. 动态调整滤波系数
    float alpha = FILTER_ALPHA_LIGHT;
    if (fabsf(raw_gyro_z - prev_filtered) > 2.0f) {
        // 数据变化大时使用更强的滤波
        alpha = FILTER_ALPHA_HEAVY;
    }
    
    // 3. 一阶低通滤波
    float filtered = alpha * raw_gyro_z + (1.0f - alpha) * prev_filtered;
    
    prev_raw = raw_gyro_z;
    return filtered;
}
