#include "mpu6050.h"
#include "delay.h"

void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t txData[2] = {reg, data};

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_fillControllerTXFIFO(MPU6050_INST, txData, 2);
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
}

uint8_t MPU6050_ReadReg(uint8_t reg)
{
    uint8_t rxData;

    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_fillControllerTXFIFO(MPU6050_INST, &reg, 1);
    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 1);
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));

    DL_I2C_startControllerTransfer(MPU6050_INST, MPU6050_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_RX, 1);
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS));
    while (!(DL_I2C_getControllerStatus(MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    rxData = DL_I2C_receiveControllerData(MPU6050_INST);

    return rxData;
}

void MPU6050_Init(void)
{
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);  /* 复位 */
    delay_ms(100);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);  /* 唤醒 */
    delay_ms(10);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 9);      /* 采样率 = 1kHz/(9+1) = 100Hz */
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);        /* 低通滤波 ~43Hz */
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x08);   /* ±500°/s，灵敏度 65.5 LSB/°/s */
}

int16_t MPU6050_ReadGyroZ(void)
{
    uint8_t high = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H + 4);
    uint8_t low  = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H + 5);
    return (int16_t)((high << 8) | low);
}
