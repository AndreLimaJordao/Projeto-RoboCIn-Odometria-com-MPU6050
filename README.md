# Projeto da seletiva 2025 RoboCIn
Por: André Lima Jordão <alj@cin.ufpe.br>

Last release date: 15/01/2025

First release date: 13/01/2025

O presente projeto tem como objetivo realizar medições através do MPU6050, usando um STM32F767 como uC, e calcular a velocidade angular (rad/s) e movimentação angular (rad).

O projeto não faz uso de bibliotecas externas com exceção do "mbed.h".
A implementação usa do protocolo de comunicação I2C com o MPU6050 de endereço salvo como 0x68 (MPU6050_ADDRESS), configurando os registradores de definições do giroscópio (MPU6050_GYRO_CONFIG_REG), com FULL_SCALE_RANGE em +- 2000 °/s, de definições do acelerômetro (MPU6050_ACCEL_CONFIG_REG), com FULL_SCALE_RANGE em +- 16g, e de energia (MPU6050_PWR_MGMT_1_REG).
O projeto faz uso de duas funções criadas para gravação e leitura no MPU6050 e de variáveis de escala para correção dos valores do MPU (offset e scale).
A única leitura de importância para o projeto é o Gyro_Z (valor do giroscópio no eixo z) para avaliar a velocidade angular e a integral em relação ao tempo (de forma discreta em intervalos de tempo de 10ms usando uma pausa de execução).

