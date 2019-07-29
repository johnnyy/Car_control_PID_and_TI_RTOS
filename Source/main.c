/**
 *  \file   main.c
 *
 *  \brief  Example application main file. This application will toggle the led.
 *          The led toggling will be done inside an callback function, which
 *          will be called by Interrupt Service Routine. Interrupts are
 *          triggered manually and no external source is used to trigger
 *          interrupts.
 *
 */

/*
 * Copyright (C) 2014 - 2018 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifdef USE_BIOS
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#if defined(SOC_AM65XX) || defined(SOC_J721E)
#if defined (__aarch64__)
#include <ti/sysbios/family/arm/v8a/Mmu.h>
#endif
#endif
#endif /* #ifdef USE_BIOS */

#include <stdio.h>

/* TI-RTOS Header files */
#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

#include "GPIO_log.h"
#include "GPIO_board.h"

#include <ti/board/board.h>
#include <ti/sysbios/family/arm/a8/intcps/Hwi.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drv/i2c/I2C.h>
#include <ti/drv/i2c/soc/I2C_soc.h>

#if defined(SOC_AM65XX) || defined(SOC_J721E)
#include <ti/drv/sciclient/sciclient.h>
#endif

/**********************************************************************
 ************************** Macros ************************************
 **********************************************************************/
#define DELAY_VALUE       (500U)   /* 500 msec */

/**********************************************************************
 ************************** Internal functions ************************
 **********************************************************************/

/* Delay function */
void AppDelay(unsigned int delayVal);

/* Callback function */
void AppGpioCallbackFxn(void);


#include "ti/lib_aux/configuracaoInit.h"


// roda 1 => frente esquerda
// roda 2 => frente direita
// roda 3 => traseira esquerda
// roda 4 => traseira direita


/*
 *  ======== Board_initI2C ========
 */

static void Board_initGPIO(void)
{
    Board_initCfg boardCfg;

#if defined(SOC_K2H) || defined(SOC_K2K) || defined(SOC_K2E) || defined(SOC_K2L) || defined(SOC_K2G) || defined(SOC_C6678) || defined(SOC_C6657) || defined(SOC_OMAPL137) || defined(SOC_OMAPL138) || defined(SOC_AM65XX) || defined(SOC_J721E)
    GPIO_v0_HwAttrs gpio_cfg;

    /* Get the default SPI init configurations */
    GPIO_socGetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);


#if defined(SOC_K2G)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetIntMux(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL, GPIO_MUX_SEL);
#endif
#if defined(SOC_OMAPL137) || defined(SOC_OMAPL138)
    /* Setup GPIO interrupt configurations */
    GPIO_socSetBankInt(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, NULL);
#endif
#endif

#if defined(evmK2E) || defined(evmC6678)
    boardCfg = BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#else
    boardCfg = BOARD_INIT_PINMUX_CONFIG |
        BOARD_INIT_MODULE_CLOCK |
        BOARD_INIT_UART_STDIO;
#endif
    Board_init(boardCfg);

#if defined(idkAM572x) || defined(idkAM574x)
    GPIOApp_UpdateBoardInfo();
#endif

    /* Modify the default GPIO configurations if necessary */
#if defined (am65xx_evm) || defined (am65xx_idk) || defined (j721e_sim)

    GPIO_configIntRouter(GPIO_LED0_PORT_NUM, GPIO_LED0_PIN_NUM, 0, &gpio_cfg);

    /* Set the default GPIO init configurations */
    GPIO_socSetInitCfg(GPIO_LED0_PORT_NUM, &gpio_cfg);

#endif
}

/**********************************************************************
 ************************** Global Variables **************************
 **********************************************************************/
volatile uint32_t gpio_intr_triggered = 0;
uint32_t gpioBaseAddr;
uint32_t gpioPin;


#ifdef USE_BIOS
/*
 *  ======== main ========
 */
int main(void)
{
 #if defined(SOC_AM65XX) || defined(SOC_J721E)
    Sciclient_ConfigPrms_t  sciClientCfg;
#endif

#if defined(SOC_AM65XX) || defined(SOC_J721E)
    Sciclient_configPrmsInit(&sciClientCfg);
    Sciclient_init(&sciClientCfg);
#endif
    /* Call board init functions */
    Board_initGPIO();

#if defined(idkAM574x) || defined(idkAM572x) || defined(idkAM571x)
    AppGPIOInit();
#endif

    UART_printf("Config...\n");

    I2C_HwAttrs   i2c_cfg;

    I2C_socGetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg);
    I2C_socSetInitCfg(I2C_MPU_6065_INSTANCE, &i2c_cfg);

    I2C_socGetInitCfg(I2C_PCA9685_INSTANCE, &i2c_cfg);
    I2C_socSetInitCfg(I2C_PCA9685_INSTANCE, &i2c_cfg);

    I2C_init();


    /* Create Clock */

    Clock_Params clkParamsClockPID;
    Clock_Params_init(&clkParamsClockPID);
    clkParamsClockPID.startFlag = 1;
    clkParamsClockPID.period = 100;

    Clock_PID = Clock_create(swiFuncPID, 100, &clkParamsClockPID, NULL);

    Clock_Params clkParamsEspaco;
    Clock_Params_init(&clkParamsEspaco);
    clkParamsEspaco.startFlag = 1;
    clkParamsEspaco.period = 10;

    Clock_Espaco = Clock_create(swiFuncEspaco, 10, &clkParamsEspaco, NULL);


    /* Create Semaphore */

    Semaphore_Params semParamsPID;
    Semaphore_Params_init(&semParamsPID);
    semParamsPID.mode = Semaphore_Mode_BINARY;

    sem_PID = Semaphore_create(0, &semParamsPID, NULL);

    Semaphore_Params semParamsEspaco;
    Semaphore_Params_init(&semParamsEspaco);
    semParamsEspaco.mode = Semaphore_Mode_BINARY;

    sem_Espaco = Semaphore_create(0, &semParamsEspaco, NULL);

    Semaphore_Params semParamsConfigInit;
    Semaphore_Params_init(&semParamsConfigInit);
    semParamsConfigInit.mode = Semaphore_Mode_BINARY;

    sem_ConfigInit = Semaphore_create(0, &semParamsConfigInit, NULL);

    /* Config GPIO */

    GPIODirModeSet(GPIO_REGS_RODA_1_DIR_1, GPIO_PIN_RODA_1_DIR_1, GPIO_CFG_OUTPUT);
    GPIODirModeSet(GPIO_REGS_RODA_1_DIR_2, GPIO_PIN_RODA_1_DIR_2, GPIO_CFG_OUTPUT);
    GPIODirModeSet(GPIO_REGS_RODA_2_DIR_1, GPIO_PIN_RODA_2_DIR_1, GPIO_CFG_OUTPUT);
    GPIODirModeSet(GPIO_REGS_RODA_2_DIR_2, GPIO_PIN_RODA_2_DIR_2, GPIO_CFG_OUTPUT);
    GPIODirModeSet(GPIO_REGS_RODA_3_DIR_1, GPIO_PIN_RODA_3_DIR_1, GPIO_CFG_OUTPUT);
    GPIODirModeSet(GPIO_REGS_RODA_3_DIR_2, GPIO_PIN_RODA_3_DIR_2, GPIO_CFG_OUTPUT);
    GPIODirModeSet(GPIO_REGS_RODA_4_DIR_1, GPIO_PIN_RODA_4_DIR_1, GPIO_CFG_OUTPUT);
    GPIODirModeSet(GPIO_REGS_RODA_4_DIR_2, GPIO_PIN_RODA_4_DIR_2, GPIO_CFG_OUTPUT);

    GPIODirModeSet(GPIO_REGS_TRAJETORIA_1, GPIO_PIN_TRAJETORIA_1, GPIO_CFG_OUTPUT);
    GPIODirModeSet(GPIO_REGS_TRAJETORIA_2, GPIO_PIN_TRAJETORIA_2, GPIO_CFG_OUTPUT);
    GPIODirModeSet(GPIO_REGS_TRAJETORIA_3, GPIO_PIN_TRAJETORIA_3, GPIO_CFG_OUTPUT);
    GPIODirModeSet(GPIO_REGS_TRAJETORIA_4, GPIO_PIN_TRAJETORIA_4, GPIO_CFG_OUTPUT);

    /* Create Tasks */

    Task_Params taskParamsPID;
    Task_Params taskParamsEspaco;
    Task_Params taskParamsConfigInit;

    Task_Params_init(&taskParamsPID);
    Task_Params_init(&taskParamsEspaco);
    Task_Params_init(&taskParamsConfigInit);

    taskParamsPID.stackSize = 0x1400;
    taskParamsEspaco.stackSize = 0x1400;
    taskParamsConfigInit.stackSize = 0x1400;

    taskParamsPID.priority = 12;
    taskParamsEspaco.priority = 5;
    taskParamsConfigInit.priority = 14;

    taskPID = Task_create(PID, &taskParamsPID, NULL);
    taskEspaco = Task_create(Espaco, &taskParamsEspaco, NULL);
    taskConfigInit = Task_create(ConfigInit, &taskParamsConfigInit, NULL);

    /* Create Hwi*/

    Hwi_Params hwiParams;
    Hwi_Params_init(&hwiParams);

    /* Create parameters */

    hwiParams.enableInt = FALSE;
    Hwi_create(62, isrFunc, &hwiParams, NULL);
    Hwi_create(63, isrFunc, &hwiParams, NULL);
    Hwi_create(96, isrFunc, &hwiParams, NULL);
    Hwi_create(97, isrFunc, &hwiParams, NULL);
    Hwi_create(98, isrFunc, &hwiParams, NULL);
    Hwi_create(99, isrFunc, &hwiParams, NULL);

    /* enable interrupts */

    Hwi_enableInterrupt(62);
    Hwi_enableInterrupt(63);
    Hwi_enableInterrupt(96);
    Hwi_enableInterrupt(97);
    Hwi_enableInterrupt(98);
    Hwi_enableInterrupt(99);

    GPIODirModeSet(GPIO_REGS_RODA_1, GPIO_PIN_RODA_1, GPIO_CFG_INPUT);
    GPIOIntTypeSet(GPIO_REGS_RODA_1, GPIO_PIN_RODA_1, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(GPIO_REGS_RODA_1, GPIO_INT_LINE_RODA_1, GPIO_PIN_RODA_1);

    GPIODirModeSet(GPIO_REGS_RODA_2, GPIO_PIN_RODA_2, GPIO_CFG_INPUT);
    GPIOIntTypeSet(GPIO_REGS_RODA_2, GPIO_PIN_RODA_2, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(GPIO_REGS_RODA_2, GPIO_INT_LINE_RODA_2, GPIO_PIN_RODA_2);

    GPIODirModeSet(GPIO_REGS_RODA_3, GPIO_PIN_RODA_3, GPIO_CFG_INPUT);
    GPIOIntTypeSet(GPIO_REGS_RODA_3, GPIO_PIN_RODA_3, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(GPIO_REGS_RODA_3, GPIO_INT_LINE_RODA_3, GPIO_PIN_RODA_3);

    GPIODirModeSet(GPIO_REGS_RODA_4, GPIO_PIN_RODA_4, GPIO_CFG_INPUT);
    GPIOIntTypeSet(GPIO_REGS_RODA_4, GPIO_PIN_RODA_4, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(GPIO_REGS_RODA_4, GPIO_INT_LINE_RODA_4, GPIO_PIN_RODA_4);

    GPIODirModeSet(GPIO_REGS_CONTROLE, GPIO_PIN_CONTROLE, GPIO_CFG_INPUT);
    GPIOIntTypeSet(GPIO_REGS_CONTROLE, GPIO_PIN_CONTROLE, GPIO_INT_TYPE_RISE_EDGE);
    GPIOPinIntEnable(GPIO_REGS_CONTROLE, GPIO_LINE_CONTROLE, GPIO_PIN_CONTROLE);

    UART_printf("Config\n");

    /* Start BIOS */
    BIOS_start();
    return (0);
}
#endif

/*
 *  ======== AppDelay ========
 */
void AppDelay(unsigned int delayVal)
{
    Osal_delay(delayVal);
}

/*
 *  ======== AppLoopDelay ========
 */
void AppLoopDelay(uint32_t delayVal)
{
    volatile uint32_t i;

    for (i = 0; i < (delayVal * 1000); i++)
        ;
}

/*
 *  ======== Callback function ========
 */
void AppGpioCallbackFxn(void)
{
    /* Toggle LED1 */
    GPIO_toggle(USER_LED1);
    AppLoopDelay(DELAY_VALUE);
    gpio_intr_triggered = 1;
}



