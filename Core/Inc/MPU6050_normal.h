#ifndef __MPU6050_NORMAL_H
#define __MPU6050_NORMAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ===== REGISTER CONFIG (Same as CanBusTest) =====
#define MPU6050_ADDR            0xD0          // (0x68 << 1)
#define SMPLRT_DIV_REG          0x19
#define CONFIG_REG              0x1A
#define GYRO_CONFIG_REG         0x1B
#define ACCEL_CONFIG_REG        0x1C
#define ACCEL_XOUT_H_REG        0x3B
#define GYRO_XOUT_H_REG         0x43
#define PWR_MGMT_1_REG          0x6B
#define WHO_AM_I_REG            0x75

// ===== STRUCT (Same as CanBusTest) =====
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    float Ax, Ay, Az;
    float Gx, Gy, Gz;

    float Roll, Pitch, Yaw;

    float GyroOffsetX;
    float GyroOffsetY;
    float GyroOffsetZ;

    float gx_t;
    float gy_t;
    float gz_t;
} MPU6050_t;

// ===== MADGWICK =====
typedef struct {
    float q0, q1, q2, q3;   // quaternion
    float beta;             // gain
} Madgwick_t;

extern MPU6050_t mpu;
extern Madgwick_t madgwick;

/* Exported Global Variables for main.c compatibility ------------------------*/
extern volatile float Robot_Yaw;
extern volatile float Robot_Roll;
extern volatile float Robot_Pitch;
extern volatile float Final_Gyro_Z;
extern signed char Is_Calibrated;

extern volatile float Accel_X_Raw_g;
extern volatile float Accel_Y_Raw_g;
extern volatile float Accel_Z_Raw_g;
extern volatile float Accel_Total_Raw_g;

extern volatile float Gyro_X_dps;
extern volatile float Gyro_Y_dps;
extern volatile float Gyro_Z_dps;

// ===== FUNCTIONS (CanBusTest API) =====
void Madgwick_Init(Madgwick_t *mad, float beta);
void Madgwick_Update(Madgwick_t *mad,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt);
void Madgwick_GetEuler(Madgwick_t *mad,
                       float *roll, float *pitch, float *yaw);

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *dat);
void MPU6050_Update(MPU6050_t *dat, float dt);
void MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, MPU6050_t *dat);

// ===== WRAPPER FUNCTIONS (main.c compatibility) =====
uint8_t MPU6050_Init_Direct(void);
uint8_t MPU6050_Read_Accel_Raw(void);
uint8_t MPU6050_Read_Gyro_All(void);
void MPU6050_Calibrate_Gyro(void);
uint8_t MPU6050_Is_Stationary_IMU(void);
void MPU6050_Update_AutoBias(void);
void IMU_Process_Yaw(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_NORMAL_H */