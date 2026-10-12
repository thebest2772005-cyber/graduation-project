#include "pid_controller.h"
#include <math.h>

void PID_Init(PID_Instance_t *pid, float Kp, float Ki, float Kd, float alpha_d, float limit_out, float limit_i) 
{
    pid->limit_output = limit_out;
    pid->limit_integral = limit_i;
    
    PID_Update_Gains(pid, Kp, Ki, Kd, alpha_d);
    PID_Reset(pid);
}

void PID_Update_Gains(PID_Instance_t *pid, float Kp, float Ki, float Kd, float alpha_d) 
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->alpha_d = alpha_d;
    
    // T? d?ng tính toán h? s? ch?ng bão hòa khâu I tracking
    if (Kp > 0.0f) {
        pid->Kb = Ki / Kp;
    } else {
        pid->Kb = 0.0f;
    }
}

void PID_Set_Setpoint(PID_Instance_t *pid, float setpoint) 
{
    pid->setpoint = setpoint;
}

void PID_Reset(PID_Instance_t *pid) 
{
    pid->setpoint = 0.0f;
    pid->error_past = 0.0f;
    pid->integral_past = 0.0f;
    pid->error_sat_past = 0.0f;
    pid->derivative_filtered_past = 0.0f;
}

float PID_Compute(PID_Instance_t *pid, float feedback, float dt) 
{
    if (dt <= 0.0f) return 0.0f;

    // 1. Tính toán sai s?
    float error = pid->setpoint - feedback;
    
    // 2. Khâu T? l? (P)
    float up = pid->Kp * error;
    
    // 3. Khâu Tích phân (I) kèm co ch? Anti-Windup d?a trên sai s? bão hòa chu k? tru?c
    float ui = pid->integral_past + dt * (pid->Ki * error + pid->Kb * pid->error_sat_past);
    
    // Gi?i h?n riêng cho khâu I (n?u có c?u hình limit_integral l?n hon 0)
    if (pid->limit_integral > 0.0f) {
        if (ui > pid->limit_integral) ui = pid->limit_integral;
        else if (ui < -pid->limit_integral) ui = -pid->limit_integral;
    }

    // 4. Khâu Vi phân (D) + B? l?c nhi?u s? (Low-Pass Filter)
    float ud_raw = pid->Kd * (error - pid->error_past) / dt;
    float ud_filtered = (pid->alpha_d * ud_raw) + ((1.0f - pid->alpha_d) * pid->derivative_filtered_past);

    // 5. T?ng h?p ngõ ra thô
    float output_raw = up + ui + ud_filtered;
    float output_saturated = output_raw;
    
    // 6. Gi?i h?n ngõ ra t?i da (Saturation) & tính toán sai l?ch bão hòa cho chu k? sau
    if (output_raw > pid->limit_output) {
        output_saturated = pid->limit_output;
        pid->error_sat_past = pid->limit_output - output_raw;
    } 
    else if (output_raw < -pid->limit_output) {
        output_saturated = -pid->limit_output;
        pid->error_sat_past = -pid->limit_output - output_raw;
    } 
    else {
        pid->error_sat_past = 0.0f;
    }

    // 7. Luu l?i tr?ng thái chu k? này ph?c v? chu k? k? ti?p
    pid->error_past = error;
    pid->integral_past = ui;
    pid->derivative_filtered_past = ud_filtered;

    return output_saturated;
}