/**
 * @file dvc_bmi088.cpp
 * @author WFZ
 * @brief  BMI088 IMU
 * @version 0.0
 * @date 2026-01-03
 *
 *
 */

#include "dvc_bmi088.h"

/* 片选控制函数 -------------------------------------------------------------*/

/**
  * @brief  加速度计片选控制
  * @param  state: 0-取消片选(高电平), 1-片选(低电平)
  */
void Class_BMI088::AccChipSelect(uint8_t state)
{
    if (state) {
        HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(BMI088_ACC_CS_PORT, BMI088_ACC_CS_PIN, GPIO_PIN_SET);
    }
}

/**
  * @brief  陀螺仪片选控制
  * @param  state: 0-取消片选(高电平), 1-片选(低电平)
  */
void Class_BMI088::GyroChipSelect(uint8_t state)
{
    if (state) {
        HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(BMI088_GYRO_CS_PORT, BMI088_GYRO_CS_PIN, GPIO_PIN_SET);
    }
}

/* 底层SPI通信函数 ----------------------------------------------------------*/

/**
  * @brief  读取加速度计寄存器
  * @param  reg_addr: 寄存器地址
  * @retval 读取到的数据
  */
uint8_t Class_BMI088::ReadAccReg(uint8_t reg_addr)
{
    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};
    
    // 加速度计读取: 地址最高位置1表示读取
    tx_data[0] = reg_addr | 0x80;
    
    AccChipSelect(1);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, BMI088_SPI_TIMEOUT);
    AccChipSelect(0);
    
    return rx_data[1];
}

/**
  * @brief  写入加速度计寄存器
  * @param  reg_addr: 寄存器地址
  * @param  data: 要写入的数据
  */
void Class_BMI088::WriteAccReg(uint8_t reg_addr, uint8_t data)
{
    uint8_t tx_data[2] = {0};
    
    // 加速度计写入: 地址最高位置0表示写入
    tx_data[0] = reg_addr & 0x7F;
    tx_data[1] = data;
    
    AccChipSelect(1);
    HAL_SPI_Transmit(&hspi1, tx_data, 2, BMI088_SPI_TIMEOUT);
    AccChipSelect(0);
}

/**
  * @brief  读取陀螺仪寄存器
  * @param  reg_addr: 寄存器地址
  * @retval 读取到的数据
  */
uint8_t Class_BMI088::ReadGyroReg(uint8_t reg_addr)
{
    uint8_t tx_data[2] = {0};
    uint8_t rx_data[2] = {0};
    
    // 陀螺仪读取: 地址最高位置1表示读取
    tx_data[0] = reg_addr | 0x80;
    
    GyroChipSelect(1);
    HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, BMI088_SPI_TIMEOUT);
    GyroChipSelect(0);
    
    return rx_data[1];
}

/**
  * @brief  写入陀螺仪寄存器
  * @param  reg_addr: 寄存器地址
  * @param  data: 要写入的数据
  */
void Class_BMI088::WriteGyroReg(uint8_t reg_addr, uint8_t data)
{
    uint8_t tx_data[2] = {0};
    
    // 陀螺仪写入: 地址最高位置0表示写入
    tx_data[0] = reg_addr & 0x7F;
    tx_data[1] = data;
    
    GyroChipSelect(1);
    HAL_SPI_Transmit(&hspi1, tx_data, 2, BMI088_SPI_TIMEOUT);
    GyroChipSelect(0);
}

/**
  * @brief  连续读取加速度计寄存器
  * @param  reg_addr: 起始寄存器地址
  * @param  data: 数据缓冲区
  * @param  len: 要读取的字节数
  */
void Class_BMI088::ReadAccRegs(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    uint8_t tx = reg_addr | 0x80;          // read mask
    uint8_t tmp[1 + 32];                   // 如果你len最大不超过32（acc 6字节/温度2字节完全够）
    if (len > 32) return;                  // 或者做更合理的错误处理

    AccChipSelect(1);

    HAL_SPI_Transmit(&hspi1, &tx, 1, BMI088_SPI_TIMEOUT);
    HAL_SPI_Receive(&hspi1, tmp, len + 1, BMI088_SPI_TIMEOUT);  // 多读1字节dummy

    AccChipSelect(0);

    memcpy(data, &tmp[1], len);            // 跳过dummy
}

/**
  * @brief  连续读取陀螺仪寄存器
  * @param  reg_addr: 起始寄存器地址
  * @param  data: 数据缓冲区
  * @param  len: 要读取的字节数
  */
void Class_BMI088::ReadGyroRegs(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    uint8_t tx_data = reg_addr | 0x80;  // 读取命令
    
    GyroChipSelect(1);
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, BMI088_SPI_TIMEOUT);
    HAL_SPI_Receive(&hspi1, data, len, BMI088_SPI_TIMEOUT);
    GyroChipSelect(0);
}

/* 灵敏度设置函数 -----------------------------------------------------------*/

/**
  * @brief  设置加速度计灵敏度
  */
void Class_BMI088::SetAccSensitivity(void)
{
    switch (Config.acc_range) {
        case BMI088_ACC_RANGE_3G:
            acc_sensitivity = 10920.0f;  // LSB/g
            break;
        case BMI088_ACC_RANGE_6G:
            acc_sensitivity = 5460.0f;   // LSB/g
            break;
        case BMI088_ACC_RANGE_12G:
            acc_sensitivity = 2730.0f;   // LSB/g
            break;
        case BMI088_ACC_RANGE_24G:
            acc_sensitivity = 1365.0f;   // LSB/g
            break;
        default:
            acc_sensitivity = 1365.0f;   // 默认24G
            break;
    }
}

/**
  * @brief  设置陀螺仪灵敏度
  */
void Class_BMI088::SetGyroSensitivity(void)
{
    switch (Config.gyro_range) {
        case BMI088_GYRO_RANGE_2000:
            gyro_sensitivity = 16.384f;  // LSB/(°/s)
            break;
        case BMI088_GYRO_RANGE_1000:
            gyro_sensitivity = 32.768f;
            break;
        case BMI088_GYRO_RANGE_500:
            gyro_sensitivity = 65.536f;
            break;
        case BMI088_GYRO_RANGE_250:
            gyro_sensitivity = 131.072f;
            break;
        case BMI088_GYRO_RANGE_125:
            gyro_sensitivity = 262.144f;
            break;
        default:
            gyro_sensitivity = 16.384f;  // 默认2000dps
            break;
    }
}

/**
  * @brief  获取处理后的数据
  */
void Class_BMI088::GetData(void)
{
    GetRawData();
    
    // 加速度转换: 原始值 -> g -> m/s²
    Data.acc_x = ((float)RawData.acc_x / acc_sensitivity) * GRAVITY - acc_offset_x;
    Data.acc_y = ((float)RawData.acc_y / acc_sensitivity) * GRAVITY - acc_offset_y;
    Data.acc_z = ((float)RawData.acc_z / acc_sensitivity) * GRAVITY - acc_offset_z;
    
    // 陀螺仪转换: 原始值 -> °/s -> rad/s
    Data.gyro_x = ((float)RawData.gyro_x / gyro_sensitivity) * DEG_TO_RAD - gyro_offset_x;
    Data.gyro_y = ((float)RawData.gyro_y / gyro_sensitivity) * DEG_TO_RAD - gyro_offset_y;
    Data.gyro_z = ((float)RawData.gyro_z / gyro_sensitivity) * DEG_TO_RAD - gyro_offset_z;
    
    // 温度：0.125°C/LSB，偏置 23°C
    Data.temperature = 23.0f + (float)RawData.temp * 0.125f;
}

/**
  * @brief  获取原始数据
  * @param  raw_data: 原始数据指针
  */
void Class_BMI088::GetRawData(void)
{
    uint8_t acc_data[6] = {0};
    uint8_t gyro_data[6] = {0};
    uint8_t temp_data[2] = {0};
    
    // 读取加速度计数据
    ReadAccRegs(BMI088_ACC_X_LSB_ADDR, acc_data, 6);
    
    // 读取陀螺仪数据
    ReadGyroRegs(BMI088_GYRO_X_LSB_ADDR, gyro_data, 6);
    
    // 读取温度
    ReadAccRegs(BMI088_TEMP_MSB_ADDR, temp_data, 2);
    
    // 解析加速度计数据 (LSB在前)
    RawData.acc_x = (int16_t)((acc_data[1] << 8) | acc_data[0]);
    RawData.acc_y = (int16_t)((acc_data[3] << 8) | acc_data[2]);
    RawData.acc_z = (int16_t)((acc_data[5] << 8) | acc_data[4]);
    
    // 解析陀螺仪数据 (LSB在前)
    RawData.gyro_x = (int16_t)((gyro_data[1] << 8) | gyro_data[0]);
    RawData.gyro_y = (int16_t)((gyro_data[3] << 8) | gyro_data[2]);
    RawData.gyro_z = (int16_t)((gyro_data[5] << 8) | gyro_data[4]);
    
    // 解析温度（MSB在前）
    int16_t t = (int16_t)((temp_data[0] << 3) | (temp_data[1] >> 5));
    if (t > 1023) t -= 2048;
    RawData.temp = t;
}

/*
 * @brief  数据滤波函数
 * 
 * 
*/ 
void Class_BMI088::Filter_data(void) {
    
    // 1. 异常值检测
    if (fabsf(Data.gyro_x - prev_filtered_gyro_x) > Gyro_Outlier_Threshold) {
        // 数据突变过大，认为是异常值，使用上一次的有效值
        Data.gyro_x = prev_filtered_gyro_x;
    }

    if (fabsf(Data.gyro_y - prev_filtered_gyro_y) > Gyro_Outlier_Threshold) {
        // 数据突变过大，认为是异常值，使用上一次的有效值
        Data.gyro_y = prev_filtered_gyro_y;
    }
    
    if (fabsf(Data.gyro_z - prev_filtered_gyro_z) > Gyro_Outlier_Threshold) {
        // 数据突变过大，认为是异常值，使用上一次的有效值
        Data.gyro_z = prev_filtered_gyro_z;
    }

    if (fabsf(Data.acc_x - prev_filtered_acc_x) > Acc_Outlier_Threshold) {
        // 数据突变过大，认为是异常值，使用上一次的有效值
        Data.acc_x = prev_filtered_acc_x;
    }
    
    if (fabsf(Data.acc_y - prev_filtered_acc_y) > Acc_Outlier_Threshold) {
        // 数据突变过大，认为是异常值，使用上一次的有效值
        Data.acc_y = prev_filtered_acc_y;
    }
    
    if (fabsf(Data.acc_z - prev_filtered_acc_z) > Acc_Outlier_Threshold) {
        // 数据突变过大，认为是异常值，使用上一次的有效值
        Data.acc_z = prev_filtered_acc_z;
    }
        
    // 2. 一阶低通滤波
    Data.gyro_x = Filter_Alpha_Light * Data.gyro_x + (1.0f - Filter_Alpha_Light) * prev_filtered_gyro_x;
    Data.gyro_y = Filter_Alpha_Light * Data.gyro_y + (1.0f - Filter_Alpha_Light) * prev_filtered_gyro_y;
    Data.gyro_z = Filter_Alpha_Light * Data.gyro_z + (1.0f - Filter_Alpha_Light) * prev_filtered_gyro_z;
    
    Data.acc_x = Filter_Alpha_Light * Data.acc_x + (1.0f - Filter_Alpha_Light) * prev_filtered_acc_x;
    Data.acc_y = Filter_Alpha_Light * Data.acc_y + (1.0f - Filter_Alpha_Light) * prev_filtered_acc_y;
    Data.acc_z = Filter_Alpha_Light * Data.acc_z + (1.0f - Filter_Alpha_Light) * prev_filtered_acc_z;
    
    prev_filtered_gyro_x = Data.gyro_x;
    prev_filtered_gyro_y = Data.gyro_y;
    prev_filtered_gyro_z = Data.gyro_z;
    
    prev_filtered_acc_x = Data.acc_x;
    prev_filtered_acc_y = Data.acc_y;
    prev_filtered_acc_z = Data.acc_z;
}

/**
  * @brief  错误处理函数
  */
void Class_BMI088::ErrorHandler(void)
{
    // 可以添加复位操作或错误标志
    // 这里我们简单地进行系统复位
    HAL_NVIC_SystemReset();
}


/* 公共函数实现 -------------------------------------------------------------*/

/**
  * @brief  BMI088初始化
  */
void Class_BMI088::Init(void)
{
    uint8_t acc_id = 0, gyro_id = 0;
    uint8_t retry = 0;
        
    // 设置灵敏度
    SetAccSensitivity();
    SetGyroSensitivity();
    
    /* 1. 加速度计初始化 */
    HAL_Delay(10);
    
    // 软复位加速度计
    WriteAccReg(BMI088_ACC_SOFT_RESET_ADDR, BMI088_ACC_SOFTRESET_CMD);
    HAL_Delay(10);
    
    // 上电加速度计
    WriteAccReg(BMI088_ACC_PWR_CONF_ADDR, 0x00);  // 进入激活模式
    HAL_Delay(10);
    WriteAccReg(BMI088_ACC_PWR_CTRL_ADDR, 0x04);  // 打开加速度计
    HAL_Delay(10);
    
    // 配置加速度计
    WriteAccReg(BMI088_ACC_CONF_ADDR, Config.acc_bw);
    WriteAccReg(BMI088_ACC_RANGE_ADDR, Config.acc_range);
    HAL_Delay(10);
    
    /* 2. 陀螺仪初始化 */
    HAL_Delay(10);
    
    // 软复位陀螺仪
    WriteGyroReg(BMI088_GYRO_SOFTRESET_ADDR, BMI088_GYRO_SOFTRESET_CMD);
    HAL_Delay(10);
    
    // 配置陀螺仪
    WriteGyroReg(BMI088_GYRO_BANDWIDTH_ADDR, Config.gyro_bw);
    WriteGyroReg(BMI088_GYRO_RANGE_ADDR, Config.gyro_range);
    WriteGyroReg(BMI088_GYRO_LPM1_ADDR, 0x00);  // 正常模式
    
    HAL_Delay(10);
    
    /* 3. 检查通信 */
    for (retry = 0; retry < 3; retry++) {
        acc_id = ReadAccReg(BMI088_ACC_CHIP_ID_ADDR);
        gyro_id = ReadGyroReg(BMI088_GYRO_CHIP_ID_ADDR);
        
        // 加速度计ID应为0x1E，陀螺仪ID应为0x0F
        if (acc_id == 0x1E && gyro_id == 0x0F) {
            break;
        }
        HAL_Delay(10);
    }
    
    if (acc_id != 0x1E || gyro_id != 0x0F) {
        // 初始化失败，发出错误提示音
        for(int i = 0; i < 3; i++) {
            dvc_buzzer_SetOn();
            HAL_Delay(100);
            dvc_buzzer_SetOff();
            HAL_Delay(100);
        }
        ErrorHandler();
    }
    
    // 等待传感器稳定
    HAL_Delay(50);
    
}

/**
  * @brief  检查BMI088连接
  * @retval 0-连接正常，1-加速度计异常，2-陀螺仪异常，3-两者都异常
  */
uint8_t Class_BMI088::CheckConnection(void)
{
    uint8_t acc_id = ReadAccReg(BMI088_ACC_CHIP_ID_ADDR);
    uint8_t gyro_id = ReadGyroReg(BMI088_GYRO_CHIP_ID_ADDR);
    
    if (acc_id == 0x1E && gyro_id == 0x0F) {
        return 0;  // 正常
    } else if (acc_id != 0x1E && gyro_id == 0x0F) {
        return 1;  // 加速度计异常
    } else if (acc_id == 0x1E && gyro_id != 0x0F) {
        return 2;  // 陀螺仪异常
    } else {
        return 3;  // 两者都异常
    }
}

/**
  * @brief  漂移校准
  * @note 校准时，需保证陀螺仪处于静止状态，加速度计处于水平静止状态
  * @param  sample_count: 采样点数
  */
void Class_BMI088::Calibrate(uint16_t sample_count)
{
    float sum_gx = 0.0f, sum_gy = 0.0f, sum_gz = 0.0f;
//    float sum_ax = 0.0f, sum_ay = 0.0f, sum_az = 0.0f;

    for (uint16_t i = 0; i < sample_count; i++) {

        GetData();
        
        sum_gx += Data.gyro_x;
        sum_gy += Data.gyro_y;
        sum_gz += Data.gyro_z;
        
        // sum_ax += Data.acc_x;
        // sum_ay += Data.acc_y;
        // sum_az += Data.acc_z;
        
        HAL_Delay(2);  // 2ms间隔
    }
    
    gyro_offset_x = sum_gx / sample_count;
    gyro_offset_y = sum_gy / sample_count;
    gyro_offset_z = sum_gz / sample_count;
    
    // acc_offset_x = sum_ax / sample_count;
    // acc_offset_y = sum_ay / sample_count;
    // acc_offset_z = sum_az / sample_count - 9.80665f;
}

/**
  * @brief  bmi088数据回调函数
  */
void Class_BMI088::TIM_Calculate_PeriodElapsedCallback()
{
    GetData();

    // 对IMU数据进行滤波
    Filter_data();
}

