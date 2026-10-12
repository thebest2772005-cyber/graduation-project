#include "MPU6050_normal.h"

extern I2C_HandleTypeDef hi2c1;

// ===== GLOBAL INSTANCES (Same as CanBusTest) =====
MPU6050_t mpu;
Madgwick_t madgwick;

// ===== GLOBAL SHARED VARIABLES (main.c compatibility) =====
volatile float Robot_Yaw = 0.0f;
volatile float Robot_Roll = 0.0f;
volatile float Robot_Pitch = 0.0f;
volatile float Final_Gyro_Z = 0.0f;
signed char Is_Calibrated = 0;

volatile float Accel_X_Raw_g = 0.0f;
volatile float Accel_Y_Raw_g = 0.0f;
volatile float Accel_Z_Raw_g = 0.0f;
volatile float Accel_Total_Raw_g = 1.0f;

volatile float Gyro_X_dps = 0.0f;
volatile float Gyro_Y_dps = 0.0f;
volatile float Gyro_Z_dps = 0.0f;

static uint32_t last_yaw_time = 0;

// ===== MADGWICK =====
void Madgwick_Init(Madgwick_t *mad, float beta)
{
    mad->q0 = 1.0f;
    mad->q1 = 0.0f;
    mad->q2 = 0.0f;
    mad->q3 = 0.0f;
    mad->beta = beta;
}

void Madgwick_Update(Madgwick_t *mad,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float dt)
{
    float q0 = mad->q0, q1 = mad->q1, q2 = mad->q2, q3 = mad->q3;

    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0.0f) return;

    ax /= norm; ay /= norm; az /= norm;

    gx *= M_PI / 180.0f;
    gy *= M_PI / 180.0f;
    gz *= M_PI / 180.0f;

    float f1 = 2.0f*(q1*q3 - q0*q2) - ax;
    float f2 = 2.0f*(q0*q1 + q2*q3) - ay;
    float f3 = 2.0f*(0.5f - q1*q1 - q2*q2) - az;

    float s0 = -2.0f*q2*f1 + 2.0f*q1*f2;
    float s1 =  2.0f*q3*f1 + 2.0f*q0*f2 - 4.0f*q1*f3;
    float s2 = -2.0f*q0*f1 + 2.0f*q3*f2 - 4.0f*q2*f3;
    float s3 =  2.0f*q1*f1 + 2.0f*q2*f2;

    norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (norm == 0.0f) return;

    s0 /= norm; s1 /= norm; s2 /= norm; s3 /= norm;

    float qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz) - mad->beta*s0;
    float qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy) - mad->beta*s1;
    float qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx) - mad->beta*s2;
    float qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx) - mad->beta*s3;

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    mad->q0 = q0 / norm;
    mad->q1 = q1 / norm;
    mad->q2 = q2 / norm;
    mad->q3 = q3 / norm;
}

void Madgwick_GetEuler(Madgwick_t *mad,
                       float *roll, float *pitch, float *yaw)
{
    float q0 = mad->q0, q1 = mad->q1, q2 = mad->q2, q3 = mad->q3;

    *roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * 180.0f / M_PI;
    *pitch = asinf(2.0f*(q0*q2 - q3*q1)) * 180.0f / M_PI;
    *yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * 180.0f / M_PI;
}

// ===== INIT (Same register settings as CanBusTest) =====
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t check = 0;
    uint8_t data = 0;

    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100) != HAL_OK)
        return 1;
    if (check != 0x68)
        return 1;

    data = 0x00; // Wake up (internal 8MHz oscillator)
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);

    data = 0x03; // DLPF 44Hz
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, 100);

    data = 0x07; // 125Hz sample rate
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);

    data = 0x00; // Accel +-2g (16384 LSB/g)
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100);

    data = 0x00; // Gyro +-250 dps (131 LSB/(deg/s))
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100);

    Madgwick_Init(&madgwick, 0.02f);

    return 0;
}

// ===== READ (14 bytes burst read, same scaling as CanBusTest) =====
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_t *dat)
{
    uint8_t data[14];

    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data, 14, 100) == HAL_OK)
    {
        dat->Accel_X_RAW = (int16_t)(data[0] << 8 | data[1]);
        dat->Accel_Y_RAW = (int16_t)(data[2] << 8 | data[3]);
        dat->Accel_Z_RAW = (int16_t)(data[4] << 8 | data[5]);

        dat->Gyro_X_RAW  = (int16_t)(data[8] << 8 | data[9]);
        dat->Gyro_Y_RAW  = (int16_t)(data[10] << 8 | data[11]);
        dat->Gyro_Z_RAW  = (int16_t)(data[12] << 8 | data[13]);

        dat->Ax = dat->Accel_X_RAW / 16384.0f;
        dat->Ay = dat->Accel_Y_RAW / 16384.0f;
        dat->Az = dat->Accel_Z_RAW / 16384.0f;

        dat->Gx = dat->Gyro_X_RAW / 131.0f;
        dat->Gy = dat->Gyro_Y_RAW / 131.0f;
        dat->Gz = dat->Gyro_Z_RAW / 131.0f;

        Accel_X_Raw_g = dat->Ax;
        Accel_Y_Raw_g = dat->Ay;
        Accel_Z_Raw_g = dat->Az;
        Accel_Total_Raw_g = sqrtf(dat->Ax*dat->Ax + dat->Ay*dat->Ay + dat->Az*dat->Az);

        Gyro_X_dps = dat->Gx;
        Gyro_Y_dps = dat->Gy;
        Gyro_Z_dps = dat->Gz;
    }
}

// ===== UPDATE (Same filter, deadzone, auto-bias, and yaw integration as CanBusTest) =====
void MPU6050_Update(MPU6050_t *dat, float dt)
{
    static float gx_f = 0.0f, gy_f = 0.0f, gz_f = 0.0f;
    static float gz_bias = 0.0f;
    static float yaw_cont = 0.0f;   // infinite yaw

    // ===== REMOVE OFFSET =====
    float gx = dat->Gx - dat->GyroOffsetX;
    float gy = dat->Gy - dat->GyroOffsetY;
    float gz = dat->Gz - dat->GyroOffsetZ;

    // ===== LOW PASS FILTER =====
    gx_f = 0.7f * gx_f + 0.3f * gx;
    gy_f = 0.7f * gy_f + 0.3f * gy;
    gz_f = 0.7f * gz_f + 0.3f * gz;

    gx = gx_f;
    gy = gy_f;
    gz = gz_f;

    // ===== DEADZONE =====
    if (fabsf(gx) < 0.15f) gx = 0.0f;
    if (fabsf(gy) < 0.15f) gy = 0.0f;
    if (fabsf(gz) < 0.15f) gz = 0.0f;

    dat->gx_t = gx;
    dat->gy_t = gy;
    dat->gz_t = gz;

    // ===== AUTO BIAS (only when stable) =====
    float acc_norm = sqrtf(dat->Ax*dat->Ax + dat->Ay*dat->Ay + dat->Az*dat->Az);

    if (fabsf(gx) < 0.1f && fabsf(gy) < 0.1f && fabsf(gz) < 0.1f &&
        acc_norm > 0.99f && acc_norm < 1.01f)
    {
        gz_bias = 0.999f * gz_bias + 0.001f * gz;
    }

    gz -= gz_bias;

    // ===== ADAPTIVE BETA =====
    if (acc_norm > 0.98f && acc_norm < 1.02f)
        madgwick.beta = 0.05f;
    else
        madgwick.beta = 0.01f;

    // ===== MADGWICK UPDATE (for roll/pitch stability) =====
    Madgwick_Update(&madgwick, gx, gy, gz, dat->Ax, dat->Ay, dat->Az, dt);

    float roll, pitch, yaw;
    Madgwick_GetEuler(&madgwick, &roll, &pitch, &yaw);

    // ===== INFINITE YAW (PURE GYRO INTEGRATION) =====
    yaw_cont += gz * dt;   // deg/s * s = deg

    // ===== SMOOTH ROLL & PITCH =====
    static float r = 0.0f, p = 0.0f;
    r = 0.9f * r + 0.1f * roll;
    p = 0.9f * p + 0.1f * pitch;

    // ===== OUTPUT =====
    dat->Roll = r;
    dat->Pitch = p;
    dat->Yaw = yaw_cont;

    Robot_Roll = r;
    Robot_Pitch = p;
    Robot_Yaw = yaw_cont;
    Final_Gyro_Z = gz;
}

// ===== CALIB (2000 samples, same as CanBusTest) =====
void MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, MPU6050_t *dat)
{
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    float time = 2000.0f;
    for (int i = 0; i < (int)time; i++)
    {
        MPU6050_Read_All(hi2c, dat);
        sx += dat->Gx;
        sy += dat->Gy;
        sz += dat->Gz;
    }

    dat->GyroOffsetX = sx / time;
    dat->GyroOffsetY = sy / time;
    dat->GyroOffsetZ = sz / time;

    Is_Calibrated = 1;
    last_yaw_time = HAL_GetTick();
}

// ===== WRAPPER FUNCTIONS FOR MAIN.C =====
uint8_t MPU6050_Init_Direct(void)
{
    return MPU6050_Init(&hi2c1);
}

uint8_t MPU6050_Read_Accel_Raw(void)
{
    MPU6050_Read_All(&hi2c1, &mpu);
    return 0;
}

uint8_t MPU6050_Read_Gyro_All(void)
{
    MPU6050_Read_All(&hi2c1, &mpu);
    return 0;
}

void MPU6050_Calibrate_Gyro(void)
{
    MPU6050_CalibrateGyro(&hi2c1, &mpu);
}

uint8_t MPU6050_Is_Stationary_IMU(void)
{
    return (fabsf(mpu.gx_t) < 0.1f && fabsf(mpu.gy_t) < 0.1f && fabsf(mpu.gz_t) < 0.1f &&
            Accel_Total_Raw_g > 0.99f && Accel_Total_Raw_g < 1.01f);
}

void MPU6050_Update_AutoBias(void)
{
    // Auto-bias da duoc xu ly truc tiep ben trong MPU6050_Update() giong CanBusTest
}

void IMU_Process_Yaw(void)
{
    if (!Is_Calibrated) return;

    uint32_t now = HAL_GetTick();
    float dt = (now - last_yaw_time) / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f)
    {
        dt = 0.01f;
    }
    last_yaw_time = now;

    MPU6050_Update(&mpu, dt);
}