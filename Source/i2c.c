#include "ti/lib_aux/i2c.h"

void writeSensor(I2C_Handle h, uint8_t reg, uint8_t val, uint8_t SLAVE_ADDR){
    uint8_t txData[2] = {0,0};
    I2C_Transaction t;
    int16_t transferStatus;
    I2C_transactionInit(&t);
    //memset(&txData, 0x00, sizeof(txData));

    t.slaveAddress = SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 2;
    t.readCount = 0;
    t.timeout = 1000U;
    txData[0] = reg;
    txData[1] = val;

    transferStatus = I2C_transfer(h, &t);

    if(I2C_STS_SUCCESS != transferStatus){
       UART_printf("\n Data Transfer failed with transfer status %d \n",transferStatus);
    }
}

uint8_t readSensor(I2C_Handle h, uint8_t reg, uint8_t SLAVE_ADDR){

    uint8_t rxData = 0;
    uint8_t txData = 0;
    I2C_Transaction t;
    int16_t transferStatus;
    I2C_transactionInit(&t);
    memset(&txData, 0x00, sizeof(txData));
    t.slaveAddress = SLAVE_ADDR;
    t.writeBuf = &txData;
    t.writeCount = 1;
    t.readBuf = &rxData;
    t.readCount = 1;
    t.timeout = 1000U;
    txData = reg;
    transferStatus = I2C_transfer(h, &t);
    if(I2C_STS_SUCCESS != transferStatus){
        UART_printf("\n Data Transfer failed with transfer status %d \n",transferStatus);
    }
    return rxData;
}
