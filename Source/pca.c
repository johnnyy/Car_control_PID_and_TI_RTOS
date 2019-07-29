#include "ti/lib_aux/pca.h"

void SetupPCA(uint16_t pwm_config_roda_1, uint16_t pwm_config_roda_2, uint16_t pwm_config_roda_3, uint16_t pwm_config_roda_4){

    if(pwm_config_roda_1 > PWM_MAX){
        pwm_config_roda_1 = PWM_MAX;
    }

    if(pwm_config_roda_2 > PWM_MAX){
        pwm_config_roda_2 = PWM_MAX;
    }

    if(pwm_config_roda_3 > PWM_MAX){
        pwm_config_roda_3 = PWM_MAX;
    }

    if(pwm_config_roda_4 > PWM_MAX){
        pwm_config_roda_4 = PWM_MAX;
    }

    if(pwm_config_roda_1 < 0){
        pwm_config_roda_1 = 0;
    }

    if(pwm_config_roda_2 < 0){
        pwm_config_roda_2 = 0;
    }

    if(pwm_config_roda_3 < 0){
        pwm_config_roda_3 = 0;
    }

    if(pwm_config_roda_4 < 0){
        pwm_config_roda_4 = 0;
    }

    uint8_t pwm_l_roda_1 = pwm_config_roda_1 & 0x00FF;
    uint8_t pwm_h_roda_1 = (pwm_config_roda_1 >> 8);

    uint8_t pwm_l_roda_2 = pwm_config_roda_2 & 0x00FF;
    uint8_t pwm_h_roda_2 = (pwm_config_roda_2 >> 8);

    uint8_t pwm_l_roda_3 = pwm_config_roda_3 & 0x00FF;
    uint8_t pwm_h_roda_3 = (pwm_config_roda_3 >> 8);

    uint8_t pwm_l_roda_4 = pwm_config_roda_4 & 0x00FF;
    uint8_t pwm_h_roda_4 = (pwm_config_roda_4 >> 8);

    I2C_Params i2cParams;
    I2C_Handle handle = NULL;

    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_PCA9685_INSTANCE, &i2cParams);
    writeSensor(handle, MODE1, 0x10, PCA9685_SLAVE_ADDR);

    Task_sleep(1);

    writeSensor(handle, PRE_SCALE, 0x78, PCA9685_SLAVE_ADDR);

    writeSensor(handle, LED0_ON_H, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED0_ON_L, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED0_OFF_H, pwm_h_roda_2, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED0_OFF_L, pwm_l_roda_2, PCA9685_SLAVE_ADDR);

    writeSensor(handle, LED1_ON_H, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED1_ON_L, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED1_OFF_H, pwm_h_roda_1, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED1_OFF_L, pwm_l_roda_1, PCA9685_SLAVE_ADDR);

    writeSensor(handle, LED2_ON_H, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED2_ON_L, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED2_OFF_H, pwm_h_roda_4, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED2_OFF_L, pwm_l_roda_4, PCA9685_SLAVE_ADDR);

    writeSensor(handle, LED3_ON_H, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED3_ON_L, 0x00, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED3_OFF_H, pwm_h_roda_3, PCA9685_SLAVE_ADDR);
    writeSensor(handle, LED3_OFF_L, pwm_l_roda_3, PCA9685_SLAVE_ADDR);

    writeSensor(handle, MODE1, 0x00, PCA9685_SLAVE_ADDR);

    I2C_close(handle);
}
