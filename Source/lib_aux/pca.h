#ifndef PCA__H
#define PCA__H

#include "ti/lib_aux/i2c.h"
#include <ti/sysbios/knl/Task.h>

#define PWM_MAX                 4095

#define I2C_PCA9685_INSTANCE    2
#define PCA9685_SLAVE_ADDR      0x40

#define MODE1                   0x00
#define MODE2                   0x01

// roda 2

#define LED0_ON_L               0x06
#define LED0_ON_H               0x07
#define LED0_OFF_L              0x08
#define LED0_OFF_H              0x09

// roda 1

#define LED1_ON_L               0x0A
#define LED1_ON_H               0x0B
#define LED1_OFF_L              0x0C
#define LED1_OFF_H              0x0D

// roda 4

#define LED2_ON_L               0x0E
#define LED2_ON_H               0x0F
#define LED2_OFF_L              0x10
#define LED2_OFF_H              0x11

// roda 3

#define LED3_ON_L               0x12
#define LED3_ON_H               0x13
#define LED3_OFF_L              0x14
#define LED3_OFF_H              0x15

#define PRE_SCALE               0xFE
#define PWM_COUNTER_SIZE        4096
#define PWM_DELAY_COUNT         0

/*
* Função de configuração do PCA que configura LED0, LED1, LED2 e LED3 
*/

void SetupPCA(uint16_t pwm_config_roda_1, uint16_t pwm_config_roda_2, uint16_t pwm_config_roda_3, uint16_t pwm_config_roda_4);

#endif /* _PCA_H */
