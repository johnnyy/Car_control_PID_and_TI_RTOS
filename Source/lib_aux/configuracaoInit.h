#ifndef CONFIGURACAOINIT__H
#define CONFIGURACAOINIT__H

#include <stdio.h>

#include <ti/drv/uart/UART_stdio.h>

#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>

#include <ti/drv/gpio/GPIO.h>
#include <ti/drv/gpio/soc/GPIO_soc.h>

#include <ti/sysbios/knl/Task.h>
#include "ti/lib_aux/mpu.h"
#include "ti/lib_aux/pca.h"

#define Raio                    3.4
#define Pi                      3.1415

#define PWM_init                0

#define ti_sysbios_BIOS_WAIT_FOREVER (~(0U))
#define BIOS_WAIT_FOREVER ti_sysbios_BIOS_WAIT_FOREVER

#define SOC_GPIO_0_REGS                      (0x44E07000)
#define SOC_GPIO_1_REGS                      (0x4804C000)
#define SOC_GPIO_2_REGS                      (0x481AC000)
#define SOC_GPIO_3_REGS                      (0x481AE000)

// GPIOs LM393

#define GPIO_REGS_RODA_1        SOC_GPIO_3_REGS
#define GPIO_REGS_RODA_2        SOC_GPIO_1_REGS
#define GPIO_REGS_RODA_3        SOC_GPIO_1_REGS
#define GPIO_REGS_RODA_4        SOC_GPIO_1_REGS

#define GPIO_PIN_RODA_1         21
#define GPIO_PIN_RODA_2         17
#define GPIO_PIN_RODA_3         16
#define GPIO_PIN_RODA_4         28

#define GPIO_INT_LINE_RODA_1    GPIO_INT_LINE_1
#define GPIO_INT_LINE_RODA_2    GPIO_INT_LINE_1
#define GPIO_INT_LINE_RODA_3    GPIO_INT_LINE_1
#define GPIO_INT_LINE_RODA_4    GPIO_INT_LINE_1

// GPIOs Direcao Motores

#define GPIO_REGS_RODA_1_DIR_1  SOC_GPIO_2_REGS
#define GPIO_REGS_RODA_1_DIR_2  SOC_GPIO_2_REGS
#define GPIO_REGS_RODA_2_DIR_1  SOC_GPIO_2_REGS
#define GPIO_REGS_RODA_2_DIR_2  SOC_GPIO_2_REGS
#define GPIO_REGS_RODA_3_DIR_1  SOC_GPIO_2_REGS
#define GPIO_REGS_RODA_3_DIR_2  SOC_GPIO_2_REGS
#define GPIO_REGS_RODA_4_DIR_1  SOC_GPIO_2_REGS
#define GPIO_REGS_RODA_4_DIR_2  SOC_GPIO_2_REGS

#define GPIO_PIN_RODA_1_DIR_1   10
#define GPIO_PIN_RODA_1_DIR_2   11
#define GPIO_PIN_RODA_2_DIR_1   8
#define GPIO_PIN_RODA_2_DIR_2   9
#define GPIO_PIN_RODA_3_DIR_1   12
#define GPIO_PIN_RODA_3_DIR_2   13
#define GPIO_PIN_RODA_4_DIR_1   6
#define GPIO_PIN_RODA_4_DIR_2   7

// GPIO Botao Controle

#define GPIO_REGS_CONTROLE      SOC_GPIO_0_REGS
#define GPIO_PIN_CONTROLE       27
#define GPIO_LINE_CONTROLE      GPIO_INT_LINE_1

// GPIO placa

#define GPIO_REGS_TRAJETORIA_1  SOC_GPIO_1_REGS
#define GPIO_REGS_TRAJETORIA_2  SOC_GPIO_1_REGS
#define GPIO_REGS_TRAJETORIA_3  SOC_GPIO_1_REGS
#define GPIO_REGS_TRAJETORIA_4  SOC_GPIO_1_REGS

#define GPIO_PIN_TRAJETORIA_1   21
#define GPIO_PIN_TRAJETORIA_2   22
#define GPIO_PIN_TRAJETORIA_3   23
#define GPIO_PIN_TRAJETORIA_4   24

#define Kp                      10
#define Ki                      0
#define Kd                      8

#define QTD_IDEAL_INT_100m      6

Clock_Handle Clock_PID;
Clock_Handle Clock_Espaco;

Semaphore_Handle sem_PID;
Semaphore_Handle sem_Espaco;
Semaphore_Handle sem_ConfigInit;

Task_Handle taskPID;
Task_Handle taskEspaco;
Task_Handle taskConfigInit;

/*
* Configura pinos da trajetoria curva para direita
*/

void config_trajetoria_curva_direita();

/*
* Configura pinos da trajetoria curva para esquerda
*/

void config_trajetoria_curva_esquerda();

/*
* Configura pinos da trajetoria reta
*/

void config_trajetoria_reta();

/*
* Configura pinos da trajetoria curva para direita
*/

void config_trajetoria_parar();

/*
* Calcula a quantidade de interrupcoes necessarios pra percorrer uma certa distancia
*/

uint32_t calc_int_destino(float distancia_cm);

/*
* Configuração para uma parada temporaria
*/

void pare_temp();

/*
* Funcao que faz verificação de trajetoria completa e mudanca de trechos
*/

void Espaco_CONTROLE();

/*
* Função que faz o controle do PID nas rodas
*/

void PID_CONTROLE();

/*
* Calibra o sensor MPU (especificamente o gyro no eixo z)
*/

void calibracao(int qtd_iteracoes);

/*
* Ler parametros do sensor MPU (especificamente o gyro no eixo z e o angulo no eixo z)
*/

void readParams();

/*
* Função para tratamento de interrupções de hardware
*/

void isrFunc();

/*
* Função para tratamento do ClockPID
*/

void swiFuncPID();

/*
* Função para tratamento do ClockEspaco
*/

void swiFuncEspaco();

/*
* Função para tratamento da taskPID
*/

void PID();

/*
* Função para tratamento da taskEspaco
*/

void Espaco();

/*
* Função para tratamento da taskConfigInit
*/

void ConfigInit();


#endif /* _CONFIGURACAOINIT_H */
