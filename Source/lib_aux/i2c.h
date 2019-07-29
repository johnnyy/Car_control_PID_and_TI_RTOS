#ifndef I2C__H
#define I2C__H

#include <stdio.h>
#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>

/*
* Função de escrita em um sensor i2c
*/

void writeSensor(I2C_Handle h, uint8_t reg, uint8_t val, uint8_t SLAVE_ADDR);

/*
* Função de leitura em um sensor i2c
*/

uint8_t readSensor(I2C_Handle h, uint8_t reg, uint8_t SLAVE_ADDR);

#endif /* _I2C_H */
