#ifndef MPU__H
#define MPU__H

#include <stdio.h>
#include "ti/lib_aux/funcMath.h"
#include "ti/lib_aux/i2c.h"

#define ERRO                    0.09

#define MPU_6050_SLAVE_ADDR     0x68
#define I2C_MPU_6065_INSTANCE   1

#define PWR_MGMT_1              0x6b
#define WHOAMI                  0x75
#define ACCEL_CONFIG            0x1c
#define GYRO_CONFIG             0x1b
#define CONFIG                  0x1a
#define SMPLRT_DIV              0x19
#define FIFO_EN                 0x23

#define ACCEL_XOUT_H            0x3b
#define ACCEL_XOUT_L            0x3c
#define ACCEL_YOUT_H            0x3d
#define ACCEL_YOUT_L            0x3e
#define ACCEL_ZOUT_H            0x3f
#define ACCEL_ZOUT_L            0x40

#define GYRO_XOUT_H             0x43
#define GYRO_XOUT_L             0x44
#define GYRO_YOUT_H             0x45
#define GYRO_YOUT_L             0x46
#define GYRO_ZOUT_H             0x47
#define GYRO_ZOUT_L             0x48

#define MPU6050_RANGE_16G       0x03
#define MPU6050_RANGE_8G        0x02
#define MPU6050_RANGE_4G        0x01
#define MPU6050_RANGE_2G        0x00

#define MPU6050_SCALE_250DPS    0x03
#define MPU6050_SCALE_500DPS    0x02
#define MPU6050_SCALE_1000DPS   0x01
#define MPU6050_SCALE_2000DPS   0x00

/*
* Função de configuração do MPU 6050
*/

void SetupMPU();

/*
* Retorna o scale do gyro baseado no range
*/

float getScaleGyro(uint8_t range);

/*
* Retorna o scale do acelerometro baseado no range
*/

float getScaleAccel (uint8_t range);

/*
* Converte valor de gyro do sensor para um float com valor de gyro valido
*/

float ConvertTwosComplementByteToFloatGyro(int rawValue, uint8_t range);

/*
* Converte valor de aceleracao do sensor para um float com valor de aceleracao valido
*/

float ConvertTwosComplementByteToFloatAccel(int rawValue, uint8_t range);

#endif /* _MPU_H */
