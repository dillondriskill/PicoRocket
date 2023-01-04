void mpu6050_reset();
void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3]);
void mpu_calibrate();
void mpu_read(double accel[3], double gyro[3]);
uint8_t mpu_read_reg(uint8_t reg);
void mpu_write_c(uint8_t buf[], int len);
void mpu_write(uint8_t buf[], int len);
void update_postion(double pos[3], double angle[3]);
void acc_cal_grav(double acc[3], double angle[3]);
