#include "mbed.h"

// Definições de hardware
#define SDA_PIN PB_9
#define SCL_PIN PB_8
#define FREQUENCY 200000
const int MPU6050_ADDRESS = 0x1101000 << 1; // 0x68 7-bit I2C address

// Definições de registradores
const int MPU6050_GYRO_CONFIG_REG = 0x1B;
const int MPU6050_GYRO_SET = 0b00011000;

const int MPU6050_ACCEL_CONFIG_REG = 0x1C;
const int MPU6050_ACCEL_SET = 0b00011000;

const int MPU6050_PWR_MGMT_1_REG = 0x6B;
const int MPU6050_PWR_MGMT_1_SET = 0b00000000;

const int MPU6050_GYRO_ZOUT_MSB_REG = 0x47;
const int MPU6050_GYRO_ZOUT_LSB_REG = 0x48;

// Variáveis de calibração
const int OFFSET[2] = {0, 0};
const float GYRO_READ_SCALE = 16.38;
const float ACCEL_READ_SCALE = 2048;
const double PI = 3.14159265359;

// Variáveis gerais
char z_gyro[2];
float angle = 0;

// Inicialização de hardware
I2C i2c(SDA_PIN, SCL_PIN);

// Funções específicas
void initMPU6050(int freq) {
    i2c.frequency(freq);
}

void writeRegister(int deviceAddress, int regAddress, int data) {
    char data_write[2];
    data_write[0] = regAddress;
    data_write[1] = data;
    i2c.write(deviceAddress, data_write, 2, 0);
}

void readRegister(int deviceAddress, int regAddress, char *data, int length) {
    char data_write[1];
    data_write[0] = regAddress;
    i2c.write(deviceAddress, data_write, 1, 1);
    i2c.read(deviceAddress, data, length, 0);
}

int main() {
    // Inicializa o MPU6050
    initMPU6050(FREQUENCY);
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
        readRegister(MPU6050_ADDRESS, MPU6050_GYRO_ZOUT_MSB_REG, z_gyro, 2);
        float z_gyro_value = (float)((z_gyro[0] << 8) | z_gyro[1]);
        z_gyro_value = (z_gyro_value / GYRO_READ_SCALE) * (PI / 180);
        angle += z_gyro_value * 0.01;
        ThisThread::sleep_for(10);
    }
}