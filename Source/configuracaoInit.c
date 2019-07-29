#include "ti/lib_aux/configuracaoInit.h"

uint32_t erro_roda_1 = 0;
uint32_t erro_roda_2 = 0;
uint32_t erro_roda_3 = 0;
uint32_t erro_roda_4 = 0;

uint32_t PID_roda_1 = 0;
uint32_t PID_roda_2 = 0;
uint32_t PID_roda_3 = 0;
uint32_t PID_roda_4 = 0;

uint32_t proporcional_1 = 0;
uint32_t proporcional_2 = 0;
uint32_t proporcional_3 = 0;
uint32_t proporcional_4 = 0;

uint32_t integral_1 = 0;
uint32_t integral_2 = 0;
uint32_t integral_3 = 0;
uint32_t integral_4 = 0;

uint32_t derivativo_1 = 0;
uint32_t derivativo_2 = 0;
uint32_t derivativo_3 = 0;
uint32_t derivativo_4 = 0;


uint32_t qtd_int_roda_1_geral = 0;
uint32_t qtd_int_roda_2_geral = 0;
uint32_t qtd_int_roda_3_geral = 0;
uint32_t qtd_int_roda_4_geral = 0;

uint32_t qtd_int_roda_1 = 0;
uint32_t qtd_int_roda_2 = 0;
uint32_t qtd_int_roda_3 = 0;
uint32_t qtd_int_roda_4 = 0;

uint32_t int_roda_1 = 0;
uint32_t int_roda_2 = 0;
uint32_t int_roda_3 = 0;
uint32_t int_roda_4 = 0;

uint32_t qtd_int_roda_1_anterior = 0;
uint32_t qtd_int_roda_2_anterior = 0;
uint32_t qtd_int_roda_3_anterior = 0;
uint32_t qtd_int_roda_4_anterior = 0;

uint16_t pwm_roda_1 = 0;
uint16_t pwm_roda_2 = 0;
uint16_t pwm_roda_3 = 0;
uint16_t pwm_roda_4 = 0;

int pare = 1;
int trajetoria = 0;
int trecho = 0;
int flag_botao_int = 0;
int angulo = 1;

uint32_t int_destino = 0;

float value_gyro_z = 0;
float angulo_z = 0;
float offset_gyro_z = 0;

void config_trajetoria_curva_direita(){

    GPIOPinWrite(GPIO_REGS_RODA_4_DIR_1, GPIO_PIN_RODA_4_DIR_1, 0);
    GPIOPinWrite(GPIO_REGS_RODA_4_DIR_2, GPIO_PIN_RODA_4_DIR_2, 1);
    GPIOPinWrite(GPIO_REGS_RODA_2_DIR_1, GPIO_PIN_RODA_2_DIR_1, 0);
    GPIOPinWrite(GPIO_REGS_RODA_2_DIR_2, GPIO_PIN_RODA_2_DIR_2, 1);
    GPIOPinWrite(GPIO_REGS_RODA_1_DIR_1, GPIO_PIN_RODA_1_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_1_DIR_2, GPIO_PIN_RODA_1_DIR_2, 0);
    GPIOPinWrite(GPIO_REGS_RODA_3_DIR_1, GPIO_PIN_RODA_3_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_3_DIR_2, GPIO_PIN_RODA_3_DIR_2, 0);

}

void config_trajetoria_curva_esquerda(){

    GPIOPinWrite(GPIO_REGS_RODA_4_DIR_1, GPIO_PIN_RODA_4_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_4_DIR_2, GPIO_PIN_RODA_4_DIR_2, 0);
    GPIOPinWrite(GPIO_REGS_RODA_2_DIR_1, GPIO_PIN_RODA_2_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_2_DIR_2, GPIO_PIN_RODA_2_DIR_2, 0);
    GPIOPinWrite(GPIO_REGS_RODA_1_DIR_1, GPIO_PIN_RODA_1_DIR_1, 0);
    GPIOPinWrite(GPIO_REGS_RODA_1_DIR_2, GPIO_PIN_RODA_1_DIR_2, 1);
    GPIOPinWrite(GPIO_REGS_RODA_3_DIR_1, GPIO_PIN_RODA_3_DIR_1, 0);
    GPIOPinWrite(GPIO_REGS_RODA_3_DIR_2, GPIO_PIN_RODA_3_DIR_2, 1);

}

void config_trajetoria_reta(){

    GPIOPinWrite(GPIO_REGS_RODA_4_DIR_1, GPIO_PIN_RODA_4_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_4_DIR_2, GPIO_PIN_RODA_4_DIR_2, 0);
    GPIOPinWrite(GPIO_REGS_RODA_2_DIR_1, GPIO_PIN_RODA_2_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_2_DIR_2, GPIO_PIN_RODA_2_DIR_2, 0);
    GPIOPinWrite(GPIO_REGS_RODA_1_DIR_1, GPIO_PIN_RODA_1_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_1_DIR_2, GPIO_PIN_RODA_1_DIR_2, 0);
    GPIOPinWrite(GPIO_REGS_RODA_3_DIR_1, GPIO_PIN_RODA_3_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_3_DIR_2, GPIO_PIN_RODA_3_DIR_2, 0);

}

void config_trajetoria_parar(){

    GPIOPinWrite(GPIO_REGS_RODA_4_DIR_1, GPIO_PIN_RODA_4_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_4_DIR_2, GPIO_PIN_RODA_4_DIR_2, 1);
    GPIOPinWrite(GPIO_REGS_RODA_2_DIR_1, GPIO_PIN_RODA_2_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_2_DIR_2, GPIO_PIN_RODA_2_DIR_2, 1);
    GPIOPinWrite(GPIO_REGS_RODA_1_DIR_1, GPIO_PIN_RODA_1_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_1_DIR_2, GPIO_PIN_RODA_1_DIR_2, 1);
    GPIOPinWrite(GPIO_REGS_RODA_3_DIR_1, GPIO_PIN_RODA_3_DIR_1, 1);
    GPIOPinWrite(GPIO_REGS_RODA_3_DIR_2, GPIO_PIN_RODA_3_DIR_2, 1);

}

uint32_t calc_int_destino(float distancia_cm){
    float aux = distancia_cm / (2 * Pi * Raio);
    aux *= 20;
    uint32_t qtd_int_ = aux;
    return qtd_int_;
}

void pare_temp(){

    pare = 1;

    config_trajetoria_parar();

    Osal_delay(500);

    pwm_roda_1 = 0;
    pwm_roda_2 = 0;
    pwm_roda_3 = 0;
    pwm_roda_4 = 0;

    SetupPCA(pwm_roda_1, pwm_roda_2, pwm_roda_3, pwm_roda_4);

    qtd_int_roda_1_geral = 0;
    qtd_int_roda_2_geral = 0;
    qtd_int_roda_3_geral = 0;
    qtd_int_roda_4_geral = 0;

    qtd_int_roda_1 = 0;
    qtd_int_roda_2 = 0;
    qtd_int_roda_3 = 0;
    qtd_int_roda_4 = 0;

    qtd_int_roda_1_anterior = 0;
    qtd_int_roda_2_anterior = 0;
    qtd_int_roda_3_anterior = 0;
    qtd_int_roda_4_anterior = 0;

    Osal_delay(500);

    pare = 0;
}

void Espaco_CONTROLE(){

    switch (trajetoria) {
        case 1:
            if(min(qtd_int_roda_1_geral, qtd_int_roda_2_geral) >= int_destino){
                pare = 1;
            }
            break;
        case 2:
            if(min(qtd_int_roda_1_geral, qtd_int_roda_2_geral) >= int_destino){
                pare = 1;
            }
            break;
        case 3:
            if((min(qtd_int_roda_1_geral, qtd_int_roda_2_geral) >= int_destino) && (trecho % 2 == 0)){
                pare_temp();
                if(trecho == 6) pare = 1;
                else{
                    config_trajetoria_curva_esquerda();
                }
                trecho++;

            }else if((angulo_z >= 90 * angulo) && (trecho % 2 == 1)){
                pare_temp();
                config_trajetoria_reta();
                trecho++;
                angulo++;
            }
            break;
        case 4:
            switch (trecho) {
                case 0:
                    if(min(qtd_int_roda_1_geral, qtd_int_roda_2_geral) >= int_destino){
                        pare_temp();
                        config_trajetoria_curva_direita();
                        trecho = 1;
                    }
                    break;
                case 1:
                    if(angulo_z <= -45){
                        pare_temp();
                        config_trajetoria_reta();
                        int_destino = calc_int_destino(42);
                        trecho = 2;
                    }
                    break;
                case 2:
                    if(min(qtd_int_roda_1_geral, qtd_int_roda_2_geral) >= int_destino){
                        pare_temp();
                        config_trajetoria_curva_direita();
                        trecho = 3;
                    }
                    break;
                case 3:
                    if(angulo_z <= -90){
                        pare_temp();
                        config_trajetoria_reta();
                        int_destino = calc_int_destino(30);
                        trecho = 4;
                    }
                    break;
                case 4:
                    if(min(qtd_int_roda_1_geral, qtd_int_roda_2_geral) >= int_destino){
                        pare_temp();
                        config_trajetoria_curva_direita();
                        trecho = 5;
                    }
                    break;
                case 5:
                    if(angulo_z <= -180){
                        pare_temp();
                        config_trajetoria_reta();
                        int_destino = calc_int_destino(30);
                        trecho = 6;
                    }
                    break;
                case 6:
                    if(min(qtd_int_roda_1_geral, qtd_int_roda_2_geral) >= int_destino){
                        pare_temp();
                        config_trajetoria_curva_direita();
                        trecho = 7;
                    }
                    break;
                case 7:
                    if(angulo_z <= -270){
                        pare_temp();
                        config_trajetoria_reta();
                        int_destino = calc_int_destino(30);
                        trecho = 8;
                    }
                    break;
                case 8:
                    if(min(qtd_int_roda_1_geral, qtd_int_roda_2_geral) >= int_destino){

                        pare_temp();
                        config_trajetoria_curva_esquerda();
                        trecho = 9;
                    }
                    break;
                case 9:
                    if(angulo_z >= -225){
                        pare_temp();
                        config_trajetoria_reta();
                        int_destino = calc_int_destino(42);
                        trecho = 10;
                    }
                    break;
                case 10:
                    if(min(qtd_int_roda_1_geral, qtd_int_roda_2_geral) >= int_destino){
                        pare_temp();
                        pare = 1;
                    }
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }

    if(pare == 1){

        config_trajetoria_parar();

        pwm_roda_1 = 0;
        pwm_roda_2 = 0;
        pwm_roda_3 = 0;
        pwm_roda_4 = 0;

        SetupPCA(pwm_roda_1, pwm_roda_2, pwm_roda_3, pwm_roda_4);

        Semaphore_post(sem_ConfigInit);
    }
}

void PID_CONTROLE(){

    int_roda_4 = qtd_int_roda_4;
    int_roda_1 = qtd_int_roda_1;
    int_roda_3 = qtd_int_roda_3;
    int_roda_2 = qtd_int_roda_2;

    qtd_int_roda_1 = 0;
    qtd_int_roda_2 = 0;
    qtd_int_roda_3 = 0;
    qtd_int_roda_4 = 0;

    erro_roda_1 = QTD_IDEAL_INT_100m - int_roda_1;
    erro_roda_2 = QTD_IDEAL_INT_100m - int_roda_2;
    erro_roda_3 = QTD_IDEAL_INT_100m - int_roda_3;
    erro_roda_4 = QTD_IDEAL_INT_100m - int_roda_4;

    proporcional_1 = erro_roda_1 * Kp;
    proporcional_2 = erro_roda_2 * Kp;
    proporcional_3 = erro_roda_3 * Kp;
    proporcional_4 = erro_roda_4 * Kp;

    integral_1 += erro_roda_1 * Ki;
    integral_2 += erro_roda_2 * Ki;
    integral_3 += erro_roda_3 * Ki;
    integral_4 += erro_roda_4 * Ki;

    derivativo_1 = (int_roda_1 - qtd_int_roda_1_anterior) * Kd;
    derivativo_2 = (int_roda_2 - qtd_int_roda_2_anterior) * Kd;
    derivativo_3 = (int_roda_3 - qtd_int_roda_3_anterior) * Kd;
    derivativo_4 = (int_roda_4 - qtd_int_roda_4_anterior) * Kd;

    PID_roda_1 = proporcional_1 + integral_1 + derivativo_1;
    PID_roda_2 = proporcional_2 + integral_2 + derivativo_2;
    PID_roda_3 = proporcional_3 + integral_3 + derivativo_3;
    PID_roda_4 = proporcional_4 + integral_4 + derivativo_4;

    pwm_roda_1 += PID_roda_1;
    pwm_roda_2 += PID_roda_2;
    pwm_roda_3 += PID_roda_3;
    pwm_roda_4 += PID_roda_4;

    SetupPCA(pwm_roda_1, pwm_roda_2, pwm_roda_3, pwm_roda_4);

    qtd_int_roda_1_anterior = int_roda_1;
    qtd_int_roda_2_anterior = int_roda_2;
    qtd_int_roda_3_anterior = int_roda_3;
    qtd_int_roda_4_anterior = int_roda_4;

}

void calibracao(int qtd_iteracoes){

    float gyro_z_acum = 0;

    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);

    for(int i = 0; i < qtd_iteracoes; i++){

        int gyro_z_l = readSensor(handle, GYRO_ZOUT_L, MPU_6050_SLAVE_ADDR);
        int gyro_z_h = readSensor(handle, GYRO_ZOUT_H, MPU_6050_SLAVE_ADDR);
        int gyro_z = (gyro_z_h << 8) | gyro_z_l;
        float aux_gyro_z = ConvertTwosComplementByteToFloatGyro(gyro_z, MPU6050_SCALE_1000DPS);
        gyro_z_acum += aux_gyro_z;

    }

    I2C_close(handle);

    offset_gyro_z = 0 - (gyro_z_acum / (1.0 * qtd_iteracoes));
}

void readParams(){

    I2C_Params i2cParams;
    I2C_Handle handle = NULL;
    I2C_Params_init(&i2cParams);
    handle = I2C_open(I2C_MPU_6065_INSTANCE, &i2cParams);

    int gyro_z_l = readSensor(handle, GYRO_ZOUT_L, MPU_6050_SLAVE_ADDR);
    int gyro_z_h = readSensor(handle, GYRO_ZOUT_H, MPU_6050_SLAVE_ADDR);
    int gyro_z = (gyro_z_h << 8) | gyro_z_l;
    value_gyro_z = ConvertTwosComplementByteToFloatGyro(gyro_z, MPU6050_SCALE_1000DPS) + offset_gyro_z;

    I2C_close(handle);

    if((absolute(value_gyro_z) < ERRO)){
        angulo_z += 0;
    }else{
        angulo_z += value_gyro_z * 0.01;
    }
}

void isrFunc(){

    if(GPIOPinIntStatus(GPIO_REGS_RODA_1, GPIO_INT_LINE_RODA_1, GPIO_PIN_RODA_1)){
        qtd_int_roda_1_geral++;
        qtd_int_roda_1++;
        GPIOPinIntClear(GPIO_REGS_RODA_1, GPIO_INT_LINE_RODA_1, GPIO_PIN_RODA_1);
    }

    if(GPIOPinIntStatus(GPIO_REGS_RODA_2, GPIO_INT_LINE_RODA_2, GPIO_PIN_RODA_2)){
        qtd_int_roda_2_geral++;
        qtd_int_roda_2++;
        GPIOPinIntClear(GPIO_REGS_RODA_2, GPIO_INT_LINE_RODA_2, GPIO_PIN_RODA_2);
    }

    if(GPIOPinIntStatus(GPIO_REGS_RODA_3, GPIO_INT_LINE_RODA_3, GPIO_PIN_RODA_3)){
        qtd_int_roda_3_geral++;
        qtd_int_roda_3++;
        GPIOPinIntClear(GPIO_REGS_RODA_3, GPIO_INT_LINE_RODA_3, GPIO_PIN_RODA_3);
    }

    if(GPIOPinIntStatus(GPIO_REGS_RODA_4, GPIO_INT_LINE_RODA_4, GPIO_PIN_RODA_4)){
        qtd_int_roda_4_geral++;
        qtd_int_roda_4++;
        GPIOPinIntClear(GPIO_REGS_RODA_4, GPIO_INT_LINE_RODA_4, GPIO_PIN_RODA_4);
    }

    if(GPIOPinIntStatus(GPIO_REGS_CONTROLE, GPIO_LINE_CONTROLE, GPIO_PIN_CONTROLE)){
        flag_botao_int = 1;
        GPIOPinIntClear(GPIO_REGS_CONTROLE, GPIO_LINE_CONTROLE, GPIO_PIN_CONTROLE);
    }
}


void swiFuncPID(){
    if(pare == 0){
        Semaphore_post(sem_PID);
    }
}

void swiFuncEspaco(){
    if(pare == 0){
        Semaphore_post(sem_Espaco);
    }
}

void PID(){

    Semaphore_pend(sem_PID, BIOS_WAIT_FOREVER);

    while(1){
        PID_CONTROLE();
        Semaphore_pend(sem_PID, BIOS_WAIT_FOREVER);
    }

}

void Espaco(){

    Semaphore_pend(sem_Espaco, BIOS_WAIT_FOREVER);

    while(1){
        readParams();
        Espaco_CONTROLE();
        Semaphore_pend(sem_Espaco, BIOS_WAIT_FOREVER);
    }

}

void ConfigInit(){

    UART_printf("Config...");

    UART_printf("Config MPU...\n");

    SetupMPU();

    UART_printf("Config MPU\n");

    config_trajetoria_parar();

    pwm_roda_1 = 0;
    pwm_roda_2 = 0;
    pwm_roda_3 = 0;
    pwm_roda_4 = 0;

    UART_printf("Config PCA...\n");

    SetupPCA(pwm_roda_1, pwm_roda_2, pwm_roda_3, pwm_roda_4);

    UART_printf("Config PCA\n");

    trajetoria = 0;

    while(1){

        GPIOPinWrite(GPIO_REGS_TRAJETORIA_1, GPIO_PIN_TRAJETORIA_1, GPIO_PIN_LOW);
        GPIOPinWrite(GPIO_REGS_TRAJETORIA_2, GPIO_PIN_TRAJETORIA_2, GPIO_PIN_LOW);
        GPIOPinWrite(GPIO_REGS_TRAJETORIA_3, GPIO_PIN_TRAJETORIA_3, GPIO_PIN_LOW);
        GPIOPinWrite(GPIO_REGS_TRAJETORIA_4, GPIO_PIN_TRAJETORIA_4, GPIO_PIN_LOW);

        flag_botao_int = 0;

        UART_printf("Select trajetoria\n");

        do{
            Osal_delay(1000);
        }while(flag_botao_int == 0);

        trajetoria = (trajetoria % 4) + 1;

        UART_printf("Config trajetoria\n");

        switch (trajetoria) {
            case 1:
                GPIOPinWrite(GPIO_REGS_TRAJETORIA_1, GPIO_PIN_TRAJETORIA_1, GPIO_PIN_HIGH);
                int_destino = calc_int_destino(100);
                break;
            case 2:
                GPIOPinWrite(GPIO_REGS_TRAJETORIA_2, GPIO_PIN_TRAJETORIA_2, GPIO_PIN_HIGH);
                int_destino = calc_int_destino(200);
                break;
            case 3:
                GPIOPinWrite(GPIO_REGS_TRAJETORIA_3, GPIO_PIN_TRAJETORIA_3, GPIO_PIN_HIGH);
                angulo = 1;
                trecho = 0;
                int_destino = calc_int_destino(100);
                break;
            case 4:
                trecho = 0;
                GPIOPinWrite(GPIO_REGS_TRAJETORIA_4, GPIO_PIN_TRAJETORIA_4, GPIO_PIN_HIGH);
                int_destino = calc_int_destino(30);
                break;
            default:
                break;
        }

        angulo_z = 0;
        value_gyro_z = 0;

        UART_printf("Calibration...\n");

        Osal_delay(3000);

        calibracao(1000);

        UART_printf("Calibration\n");

        Osal_delay(1000);

        pwm_roda_1 = PWM_init;
        pwm_roda_2 = PWM_init;
        pwm_roda_3 = PWM_init;
        pwm_roda_4 = PWM_init;

        qtd_int_roda_1 = 0;
        qtd_int_roda_2 = 0;
        qtd_int_roda_3 = 0;
        qtd_int_roda_4 = 0;

        qtd_int_roda_1_geral = 0;
        qtd_int_roda_2_geral = 0;
        qtd_int_roda_3_geral = 0;
        qtd_int_roda_4_geral = 0;

        qtd_int_roda_1_anterior = 0;
        qtd_int_roda_2_anterior = 0;
        qtd_int_roda_3_anterior = 0;
        qtd_int_roda_4_anterior = 0;

        SetupPCA(pwm_roda_1, pwm_roda_2, pwm_roda_3, pwm_roda_4);

        config_trajetoria_reta();

        pare = 0;

        Semaphore_pend(sem_ConfigInit, BIOS_WAIT_FOREVER);

    }
}
