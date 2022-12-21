void mpu6050_reset();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]);
void mpu_calibrate();
void mpu_read(double accel[3], double gyro[3]);