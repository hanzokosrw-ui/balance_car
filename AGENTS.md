# AGENTS.md — HAL_BalanceCar 项目规则

## 核心约束

**外设驱动代码已完成，禁止主动修改。**

所有驱动层文件视为 **只读**，除非用户显式要求修改：

### 受保护文件
- `Drivers/eMPL/*` — MPU6050 / DMP / I2C 驱动
- `Drivers/OLED/*` — OLED 显示驱动
- `Drivers/SYSTEM/*` — delay / sys / usart 系统模块
- `Drivers/STM32F1xx_HAL_Driver/*` — HAL 库
- `Users/motor.c` `Users/motor.h` — 电机 PWM 驱动
- `Users/encoder.c` `Users/encoder.h` — 编码器驱动
- `Users/kalman.c` `Users/kalman.h` — 卡尔曼滤波器
- `Users/MPU6050.c` `Users/MPU6050.h` — MPU6050 封装（若在 Drivers 外部）
- `Users/key.c` `Users/key.h` — 按键驱动
- `Users/OLED.c` `Users/OLED.h` — OLED 封装
- `Users/speed_control.c` `Users/speed_control.h` — 速度环（暂未启用）
- `Users/speed_pi.c` `Users/speed_pi.h` — 速度 PI（暂未启用）
- `Projects/MDK-ARM/*` — 项目构建配置
- `Users/stm32f1xx_hal_conf.h` `Users/stm32f1xx_it.*` — HAL 配置 / 中断

### 唯一可修改文件
| 文件 | 用途 | 修改场景 |
|------|------|---------|
| `Users/pid.h` | PID 参数宏定义 | 调参时改 Kp/Kd/Ki 默认值或步长 |
| `Users/pid.c` | PID 算法实现 | 用户主动要求优化算法 |
| `Users/main.c` | 主控制循环 | 用户主动要求改逻辑 / 调参 |

## 调参指导

- **Kp**：角度回复刚度，偏小则"发软"，偏大则高频振荡
- **Kd**：角速度阻尼，抑制振荡（约为 Kp 的 1/10）
- **Ki**：建议保持 0，平衡车用纯 PD 即可
- 按键调参：长按→Kp+, 单击→Kd+, 双击→Ki+
- 死区 55（PWM 最小值），输出限幅由 `BALANCE_PID_OUTPUT_LIMIT` 控制

## 符号约定

- `error = target - measure = 0 - kalman_angle × 100`
- `pid_out > 0` → 车体后倾，需前转补偿 → `motor_set_left(-pid_out)` 得正值
- 右电机机械反装：`motor_set_right(pid_out)`（不取反）
