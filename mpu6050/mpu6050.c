#include "mpu6050.h"

// Função auxiliar para escrever em um registrador
static inline void mpu6050_write_register(i2c_inst_t *i2c, uint8_t reg, uint8_t data) {
    uint8_t buf[] = {reg, data};
    i2c_write_blocking(i2c, MPU6050_ADDR, buf, 2, false);
}

// Função auxiliar para ler múltiplos bytes a partir de um registrador
static inline void mpu6050_read_registers(i2c_inst_t *i2c, uint8_t start_reg, uint8_t *buf, size_t len) {
    i2c_write_blocking(i2c, MPU6050_ADDR, &start_reg, 1, true);
    i2c_read_blocking(i2c, MPU6050_ADDR, buf, len, false);
}

// Configuração inicial
void mpu6050_set_config(imu_c *config, i2c_inst_t *i2c, int pin_sda, int pin_scl, int acc_scale) {
    config->i2c = i2c;
    config->pin_sda = pin_sda;
    config->pin_scl = pin_scl;
    config->acc_scale = acc_scale;
}

// Inicialização do sensor
int mpu6050_init(imu_c config) {
    i2c_init(config.i2c, 400 * 1000);
    gpio_set_function(config.pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(config.pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(config.pin_sda);
    gpio_pull_up(config.pin_scl);

    return mpu6050_reset(config);
}

// Reset do dispositivo (acorda o sensor do modo sleep)
int mpu6050_reset(imu_c config) {
    mpu6050_write_register(config.i2c, 0x6B, 0x00);
    return 0;
}

// Leitura dos dados do acelerômetro
int mpu6050_read_acc(imu_c config, int16_t accel[3]) {
    uint8_t buf[6];
    mpu6050_read_registers(config.i2c, MPU6050_REG_ACCEL_XOUT_H, buf, 6);

    for (int i = 0; i < 3; i++) {
        accel[i] = ((int16_t)buf[2 * i] << 8) | buf[2 * i + 1];
    }

    return 0;
}

// Leitura dos dados do giroscópio
int mpu6050_read_gyro(imu_c config, int16_t gyro[3]) {
    uint8_t buf[6];
    mpu6050_read_registers(config.i2c, MPU6050_REG_GYRO_XOUT_H, buf, 6);

    for (int i = 0; i < 3; i++) {
        gyro[i] = ((int16_t)buf[2 * i] << 8) | buf[2 * i + 1];
    }

    return 0;
}

// Leitura da temperatura
int mpu6050_read_temp(imu_c config, int16_t *temp) {
    uint8_t buf[2];
    mpu6050_read_registers(config.i2c, MPU6050_REG_TEMP_OUT_H, buf, 2);

    int16_t raw = ((int16_t)buf[0] << 8) | buf[1];
    *temp = (int16_t)(((float)raw) / 340.0 + 36.53);  // conversão conforme datasheet

    return 0;
}
