#include "mbed.h"                               // MBED

// Namespace
using namespace std::chrono;                    // Namespace para o uso de contadores de tempo

// Constantes
const int DELAY_MS = 1;                         // Delay de 1ms para integrar o giroscópio 
const double PI = 3.14159265359;

// Definições de hardware
#define SDA_PIN PB_9                            // Pino SDA do F767
#define SCL_PIN PB_8                            // Pino SCL do F767
#define FREQUENCY 200000                        // Frequência da comunicação I2C
const int AD0 = 0b0;                            // Seletor do AD0: 0b0 -> 0x67; 0b1 -> 0x68
const int MPU6050_ADDRESS = (0x67 | AD0) << 1;  // Endereço do MPU6050: 0x67 ou 0x68 7-bit

// Definições de registradores
const int MPU6050_GYRO_CONFIG_REG = 0x1B;       // Endereço do registrador de configuração do giroscópio
const int MPU6050_GYRO_SET = 0b00011000;        // Configuração do giroscópio para +-2000 graus por segundo

const int MPU6050_ACCEL_CONFIG_REG = 0x1C;      // Endereço do registrador de configuração do acelerômetro
const int MPU6050_ACCEL_SET = 0b00011000;       // Configuração do acelerômetro para +-16g

const int MPU6050_PWR_MGMT_1_REG = 0x6B;        // Endereço do registrador de gerenciamento de energia
const int MPU6050_PWR_MGMT_1_SET = 0b00000000;  // Configuração do registrador de gerenciamento de energia

const int MPU6050_GYRO_ZOUT_MSB_REG = 0x47;     // Endereço do registrador de leitura do giroscópio 8bits mais significativos
const int MPU6050_GYRO_ZOUT_LSB_REG = 0x48;     // Endereço do registrador de leitura do giroscópio 8bits menos significativos

// Inicialização
I2C i2c(SDA_PIN, SCL_PIN);                      // Inicialização do barramento I2C para o MPU6050

// Funções específicas
void initMPU6050(int freq) {
    i2c.frequency(freq);                        // Configura a frequência do barramento I2C
}

void writeRegister(int deviceAddress, int regAddress, int data) {
    char data_write[2];                         // Buffer de escrita
    data_write[0] = regAddress;                 // Endereço do registrador
    data_write[1] = data;                       // Dados a serem escritos
    i2c.write(deviceAddress, data_write, 2, 0); // Escreve os dados no registrador
}

void readRegister(int deviceAddress, int regAddress, char *data, int length) {
    char data_write[1];                         // Buffer de escrita
    data_write[0] = regAddress;                 // Endereço do registrador
    i2c.write(deviceAddress, data_write, 1, 1); // Escreve o endereço do registrador
    i2c.read(deviceAddress, data, length, 0);   // Lê os dados do registrador
}

int main() {
    // Variáveis de calibração
    const int OFFSET[2] = {0, 0};               // Valores de offset do sensor em relação ao centro do robô (não usado)
    const float GYRO_READ_SCALE = 16.38;        // Escala de leitura do giroscópio para converção em graus por segundo
    const float ACCEL_READ_SCALE = 2048;        // Escala de leitura do acelerômetro para converção em g
    const float ANGULAR_VEL_SCALE = 0;          // Escala de correção da velocidade angular do robô
    const float ANGULAR_VEL_OFFSET = 0;         // Componente linear de correção da velocidade angular
    const float ERROR_MARGIN = 0.5;             // Margem de erro para a leitura da velocidade angular            

    // Variáveis gerais
    char z_gyro[2];                             // Buffer de leitura do giroscópio   
    char z_gyro_verify[2];                      // Buffer de leitura do giroscópio para verificação
    int16_t z_gyro_value = 0;                   // Valor do giroscópio em graus por segundo
    int16_t z_gyro_value_verify = 0;            // Valor do giroscópio em graus por segundo para verificação   
    float angular_velocity = 0;                 // Velocidade angular do robô  
    float angular_velocity_verify = 0;          // Velocidade angular do robô para verificação 
    float angle = 0;                            // Ângulo de rotação do robô
    float corrected_angle = 0;                  // Ângulo corrigido para o intervalo [-PI, PI]

    // Inicializa o MPU6050
    initMPU6050(FREQUENCY);                     // Inicializa o barramento I2C

    // Variáveis de posição do sensor
    float offset_radius = sqrt(pow(OFFSET[0], 2) + pow(OFFSET[1], 2));
    
    // Configura o giroscópio
    writeRegister(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG_REG, MPU6050_GYRO_SET);

    // Configura o acelerômetro
    writeRegister(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG_REG, MPU6050_ACCEL_SET);

    // Configura o registrador de gerenciamento de energia
    writeRegister(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1_REG, MPU6050_PWR_MGMT_1_SET);

    // Inicializa o timer
    Timer timer;        // Inicialização do timer para contagem de tempo de execução das linhas de comando
    timer.start();      // Inicia o timer

    while(true) {   // Escopo de repetição
        auto us_l = timer.elapsed_time().count();                               // Salva o tempo antes da leitura
        do {
            readRegister(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_MSB_REG, z_gyro, 2);            // Lê o valor do giroscópio
            readRegister(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_MSB_REG, z_gyro_verify, 2);     // Lê o valor do giroscópio para verificação
            z_gyro_value = (float)((z_gyro[0] << 8) | z_gyro[1]);                           // Converte o valor do giroscópio para PF
            z_gyro_value_verify = (float)((z_gyro_verify[0] << 8) | z_gyro_verify[1]);      // Converte o valor do giroscópio para PF
            angular_velocity = (z_gyro_value / GYRO_READ_SCALE) * (PI / 180);               // Salva o valor da velocidade angular em rad/s *
            angular_velocity_verify = (z_gyro_value_verify / GYRO_READ_SCALE) * (PI / 180); // Salva o valor da velocidade angular em rad/s para verificação 
        } while(abs(angular_velocity - angular_velocity_verify) > ERROR_MARGIN);            // Verifica se o valor é consistente  
        auto us = timer.elapsed_time().count();                                 // Conta o tempo em microssegundos
        if(us_l > us) {                                                         // Se o contador der overflow e a última leitura for menor que a penúltima:
            us_l = 0;                                                           // desconsidera os valores na integral (adiciona erro, mas muito improvável de acontecer)
            us = 0;
        }
        angle += angular_velocity * (DELAY_MS/1000 + (us - us_l));              // Integral para o cálculo do ângulo com dt = DELAY_MS (em ms) com correção
        corrected_angle = fmod(angle + PI, 2 * PI) - PI;                        // Corrige o ângulo para o intervalo [-PI, PI] *
        ThisThread::sleep_for(DELAY_MS);                                        // Delay de DELAY_MS para a integral
        // * corrected_angle é o valor principal da posição angular por aproximação
        // * angular_velocity é o valor principal da velocidade angular em rad/s
        // ...
    }
}