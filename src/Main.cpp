#include "mbed.h"                           // MBED

// Definições de hardware
#define SDA_PIN PB_9                        // Pino SDA do F767
#define SCL_PIN PB_8                        // Pino SCL do F767
#define FREQUENCY 200000                    // Frequência da comunicação I2C
const int MPU6050_ADDRESS = 0x1101000 << 1; // 0x68 7-bit I2C address

// Definições de registradores
const int MPU6050_GYRO_CONFIG_REG = 0x1B;   // Endereço do registrador de configuração do giroscópio
const int MPU6050_GYRO_SET = 0b00011000;    // Configuração do giroscópio para +-2000 graus por segundo

const int MPU6050_ACCEL_CONFIG_REG = 0x1C;  // Endereço do registrador de configuração do acelerômetro
const int MPU6050_ACCEL_SET = 0b00011000;   // Configuração do acelerômetro para +-16g

const int MPU6050_PWR_MGMT_1_REG = 0x6B;    // Endereço do registrador de gerenciamento de energia
const int MPU6050_PWR_MGMT_1_SET = 0b00000000;  // Configuração do registrador de gerenciamento de energia

const int MPU6050_GYRO_ZOUT_MSB_REG = 0x47; // Endereço do registrador de leitura do giroscópio 8bits mais significativos
const int MPU6050_GYRO_ZOUT_LSB_REG = 0x48; // Endereço do registrador de leitura do giroscópio 8bits menos significativos

// Variáveis de calibração
const int OFFSET[2] = {0, 0};               // Valores de offset do sensor em relação ao centro do robô (não usado)
const float GYRO_READ_SCALE = 16.38;        // Escala de leitura do giroscópio para converção em graus por segundo
const float ACCEL_READ_SCALE = 2048;        // Escala de leitura do acelerômetro para converção em g
const double PI = 3.14159265359;            

// Variáveis gerais
char z_gyro[2];                             // Buffer de leitura do giroscópio           
float angle = 0;                            // Ângulo de rotação do robô

// Inicialização de hardware
I2C i2c(SDA_PIN, SCL_PIN);                  // Inicialização do barramento I2C para o MPU6050

// Funções específicas
void initMPU6050(int freq) {
    i2c.frequency(freq);                    // Configura a frequência do barramento I2C
}

void writeRegister(int deviceAddress, int regAddress, int data) {
    char data_write[2];                     // Buffer de escrita
    data_write[0] = regAddress;             // Endereço do registrador
    data_write[1] = data;                   // Dados a serem escritos
    i2c.write(deviceAddress, data_write, 2, 0); // Escreve os dados no registrador
}

void readRegister(int deviceAddress, int regAddress, char *data, int length) {
    char data_write[1];                     // Buffer de escrita
    data_write[0] = regAddress;             // Endereço do registrador
    i2c.write(deviceAddress, data_write, 1, 1); // Escreve o endereço do registrador
    i2c.read(deviceAddress, data, length, 0);   // Lê os dados do registrador
}

int main() {
    // Inicializa o MPU6050
    initMPU6050(FREQUENCY);                 // Inicializa o barramento I2C
    printf("I2C device address is: 0x%X\n", MPU6050_ADDRESS);

    // Variáveis de posição do sensor
    float offset_radius = sqrt(pow(OFFSET[0], 2) + pow(OFFSET[1], 2));
    
    // Configura o giroscópio
    writeRegister(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG_REG, MPU6050_GYRO_SET);

    // Configura o acelerômetro
    writeRegister(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG_REG, MPU6050_ACCEL_SET);

    // Configura o registrador de gerenciamento de energia
    writeRegister(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1_REG, MPU6050_PWR_MGMT_1_SET);

    while(true) {
        readRegister(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_MSB_REG, z_gyro, 2);    // Lê o valor do giroscópio
        float z_gyro_value = (float)((z_gyro[0] << 8) | z_gyro[1]);             // Converte o valor do giroscópio para PF
        z_gyro_value = (z_gyro_value / GYRO_READ_SCALE) * (PI / 180);           // Redimensiona o valor para radianos/s
        angle += z_gyro_value * 0.01;                                           // Integral discreta para o cálculo do ângulo com dt = 10ms
        ThisThread::sleep_for(10);                                              // Delay de 10ms para a integral
    }
}