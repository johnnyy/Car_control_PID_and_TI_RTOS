#include "ti/lib_aux/mpu.h"

void SetupMPU(){

    I2C_Params i2cParams;
    I2C_Handle handle;
    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);
    writeSensor(handle, PWR_MGMT_1, 0x00, MPU_6050_SLAVE_ADDR);
    writeSensor(handle, ACCEL_CONFIG, 0x10, MPU_6050_SLAVE_ADDR);
    writeSensor(handle, GYRO_CONFIG, 0x10, MPU_6050_SLAVE_ADDR);
    I2C_close(handle);

}

float getScaleGyro(uint8_t range){

    float result = 0;

    switch (range) {
        case MPU6050_SCALE_250DPS:
            result = 1/131.0;
            break;
        case MPU6050_SCALE_500DPS:
            result = 1/65.5;
            break;
        case MPU6050_SCALE_1000DPS:
            result = 1/32.8;
            break;
        case MPU6050_SCALE_2000DPS:
            result = 1/16.4;
            break;
        default:
            break;
    }

    return result;
}

float getScaleAccel (uint8_t range){

    float result = 0;

    switch (range) {
        case MPU6050_RANGE_2G:
            result = 1/16384.0;
            break;
        case MPU6050_RANGE_4G:
            result = 1/8192.0;
            break;
        case MPU6050_RANGE_8G:
            result = 1/4096.0;
            break;
        case MPU6050_RANGE_16G:
            result = 1/2048.0;
            break;
        default:
            break;
    }

    return result;
}

float ConvertTwosComplementByteToFloatGyro(int rawValue, uint8_t range){
    float result = 0;

    if ((rawValue & 0x00008000) == 0){
        result = ( (float) rawValue) * getScaleGyro(range);
        return result;
    }else{
        rawValue |= 0xffff8000;
        result = ( (float) rawValue) * getScaleGyro(range);
        return result;
    }
}

float ConvertTwosComplementByteToFloatAccel(int rawValue, uint8_t range){
    float result = 0;

    if ((rawValue & 0x00008000) == 0){
        result = ( (float) rawValue) * getScaleAccel(range);
        return result;
    }else{
        rawValue |= 0xffff8000;
        result = ( (float) rawValue) * getScaleAccel(range);
        return result;
    }
}

