#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>

typedef struct {
    /* Tham so c?u hình PID */
    float Kp;
    float Ki;
    float Kd;
    
    /* C?u hình b? l?c thông th?p khâu vi phân (Low-pass filter alpha) */
    float alpha_d;
    
    /* Gi?i h?n ngõ ra và ch?ng bão hòa khâu I */
    float limit_output;
    float limit_integral; // Gi?i h?n riêng cho khâu I (n?u c?n)
    float Kb;             // H? s? Anti-Windup (Back-calculation method)

    /* Bi?n tr?ng thái n?i b? (Không t? ý thay d?i bên ngoài) */
    float setpoint;
    float error_past;
    float integral_past;
    float error_sat_past;
    float derivative_filtered_past;
} PID_Instance_t;

/* Kh?i t?o b? PID v?i các tham s? ban d?u */
void PID_Init(PID_Instance_t *pid, float Kp, float Ki, float Kd, float alpha_d, float limit_out, float limit_i);

/* C?p nh?t nhanh tham s? Kp, Ki, Kd khi d?i ch? d? ch?y (ví d? t? Straight sang Turn) */
void PID_Update_Gains(PID_Instance_t *pid, float Kp, float Ki, float Kd, float alpha_d);

/* Thay d?i Setpoint */
void PID_Set_Setpoint(PID_Instance_t *pid, float setpoint);

/* Tính toán ngõ ra PID - G?i m?i chu k? Ts */
float PID_Compute(PID_Instance_t *pid, float feedback, float dt);

/* Reset toàn b? khâu tích phân và sai s? quá kh? (Dùng khi chuy?n tr?ng thái robot d?t ng?t) */
void PID_Reset(PID_Instance_t *pid);

#endif /* PID_CONTROLLER_H */