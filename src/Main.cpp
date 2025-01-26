#include "mbed.h"                               // MBED
#include "MPU6050.h"                            // Biblioteca do MPU6050

// Namespace
using namespace std::chrono;                    // Namespace para o uso de contadores de tempo

// Constantes
const int DELAY_MS = 1;                         // Delay de 1ms para integrar o giroscópio 
const double PI = 3.14159265359;

// Definições de hardware
#define SDA_PIN PB_9                            // Pino SDA do F767
#define SCL_PIN PB_8                            // Pino SCL do F767
#define FREQUENCY 200000                        // Frequência da comunicação I2C
const int AD0 = 0b1;                            // Seletor do AD0: 0b0 -> 0x67; 0b1 -> 0x68
const int MPU6050_ADDRESS = (0x67 | AD0);       // Endereço do MPU6050: 0x67 ou 0x68 7-bit

// Inicialização do MPU6050
MPU6050 mpu(MPU6050_ADDRESS, AFS_4G, GFS_500DPS, SDA_PIN, SCL_PIN, NC);  // Configuração do MPU6050

int main() {
    // Variáveis de calibração
    const int OFFSET[2] = {0, 0};               // Valores de offset do sensor em relação ao centro do robô (não usado)
    const float GYRO_READ_SCALE = 65.536;       // Escala de leitura do giroscópio para converção em graus por segundo
    const float ACCEL_READ_SCALE = 8192;        // Escala de leitura do acelerômetro para converção em g
    const float ANGULAR_VEL_SCALE = 0;          // Escala de correção da velocidade angular do robô (não usado por falta de testes)
    const float ANGULAR_VEL_OFFSET = 0;         // Componente linear de correção da velocidade angular (não usado por falta de testes)
    const float ERROR_MARGIN = 0.5;             // Margem de erro para a leitura da velocidade angular            

    // Variáveis gerais
    int16_t z_gyro_value = 0;                   // Valor do giroscópio em graus por segundo
    int16_t z_gyro_value_verify = 0;            // Valor do giroscópio em graus por segundo para verificação   
    float angular_velocity = 0;                 // Velocidade angular do robô  
    float angular_velocity_verify = 0;          // Velocidade angular do robô para verificação 
    float angle = 0;                            // Ângulo de rotação do robô
    float corrected_angle = 0;                  // Ângulo corrigido para o intervalo [-PI, PI]
    
    // Inicializa o MPU6050
    mpu.init();                                 // Inicializa o MPU6050
    bool calibrated = false;                    // Verifica se o MPU6050 está calibrado
    for(int i = 0; i < 3 && !calibrated; i++){  // Verifica se o MPU6050 está calibrado até 3 vezes
        calibrated = mpu.selfTestOK();          // Verifica se o MPU6050 está calibrado
        ThisThread::sleep_for(100ms);           // Delay de 100ms para a calibração
    }
    if(!calibrated)                             // Se o MPU6050 não estiver calibrado
        return 1;                               // Encerra o programa

    // Variáveis de posição do sensor
    float offset_radius = sqrt(pow(OFFSET[0], 2) + pow(OFFSET[1], 2));

    // Inicializa o timer
    Timer timer;        // Inicialização do timer para contagem de tempo de execução das linhas de comando
    timer.reset();      // Reseta o timer
    timer.start();      // Inicia o timer

    while(true) {   // Escopo de repetição
        do {
            mpu.gyro();                                                                     // Gera os dados internamente
            angular_velocity = mpu.gyroZ;                                                   // Salva os dados
            mpu.gyro();                                                                     // Gera novos dados internamente
            angular_velocity_verify = mpu.gyroZ;                                            // Salva os dados para precisão
        } while(abs(angular_velocity - angular_velocity_verify) > ERROR_MARGIN);            // Verifica se o valor é consistente  
        auto us = timer.elapsed_time().count();                                 // Conta o tempo em microsegundos
        timer.reset();                                                          // Reseta o timer
        angle += angular_velocity * (DELAY_MS + us * 1000);                     // Integral para o cálculo do ângulo com dt = DELAY_MS (em ms) com correção                                            // Filtra os dados do sensor
        corrected_angle = fmod(angle + PI, 2 * PI) - PI;                        // Corrige o ângulo para o intervalo [-PI, PI] *
        ThisThread::sleep_for(DELAY_MS);                                        // Delay de DELAY_MS para a integral
        // * corrected_angle é o valor principal da posição angular por aproximação
        // * angular_velocity é o valor principal da velocidade angular em rad/s
        // ...
    }
}