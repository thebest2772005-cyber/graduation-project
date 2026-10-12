#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx_hal.h" // Thay đổi theo dòng chip bạn dùng (f1xx, f4xx, g4xx,...)

// Định nghĩa các thông số cơ khí cấu hình cố định (hoặc truyền vào lúc Init nếu muốn động)
#define ENCODER_PPR         17.0f       // Số xung vật lý trên 1 vòng motor
#define GEAR_RATIO          50.0f      // Tỷ số truyền hộp số
#define ENCODER_MULTIPLIER  4.0f       // Hệ số nhân (2 nếu dùng TI1/TI2, 4 nếu dùng TI1 and TI2)
#define SAMPLE_TIME_S       0.01f      // Chu kỳ lấy mẫu delta t = 10ms (0.01 giây)
#define WHEEL_RADIUS_M      0.05f     // Bán kính bánh xe (0.05m = 5cm)

// Tổng số xung đếm được thực tế cho 1 vòng quay của trục ra hộp số (Đo thực tế 10 vòng = 8139 xung -> 1 vòng = 813.9 xung)
#define COUNTS_PER_REV      814.0f

typedef struct {
    TIM_TypeDef *Instance;     // Thanh ghi Timer phần cứng (TIM2, TIM3,...)
    int32_t encoder_position;  // Vị trí xung tích lũy (tổng số xung)
    int16_t prev_cnt;          // Giá trị counter lần trước
    
    // Các thông số đầu ra (Đọc trực tiếp sau khi Update)
    float speed_rpm;           // Vận tốc Vòng/Phút (RPM)
    float speed_rad_s;         // Vận tốc góc (rad/s)
    float velocity_m_s;        // Vận tốc dài của bánh xe (m/s)
    float angle_deg;           // Góc hiện tại của bánh xe (0 - 360 độ)
} Encoder_t;

// Khai báo hàm
void Encoder_Init(Encoder_t *enc, TIM_TypeDef *TIMx);
void Encoder_Update(Encoder_t *enc);

#endif /* __ENCODER_H */