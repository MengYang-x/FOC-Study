/* 代码功能：开环速度控制，Ud设为0
 * 控制算法：SPWM，正弦波驱动
 * PWM频率20K，PWM精度10位
 */
#include <Arduino.h>

// PWM输出引脚
const int pwmA = 32;
const int pwmB = 33;
const int pwmC = 25;

// 幅值限制函数
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// 变量定义
const int pole_pairs = 7;        // 电机的极对数
float voltage_power_supply = 12; // 电机供电电压
float shaft_angle = 0;           // 机械角度
float open_loop_timestamp = 0;   // 每次循环执行的时间
float Ualpha = 0, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;

// 电角度求解
float _electricalAngle(float shaft_angle)
{
  return shaft_angle * pole_pairs; // 电角度 = 机械角度*极对数
}

// 角度归一化，归一化角度到 [0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2 * PI); // 取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2 * PI);
  // 三目运算符。格式：condition ? expr1 : expr2
  // 其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  // fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  // 例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}

// 驱动无刷电机，设置PWM输出，由计算得到的Ua、Ub、Uc，得到输出的占空比
void setPwm(float Ua, float Ub, float Uc)
{
  // 限制占空比从0到1
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  // 写入PWM到PWM 0 1 2 通道
  ledcWrite(0, dc_a * 1023);
  ledcWrite(1, dc_b * 1023);
  ledcWrite(2, dc_c * 1023);

  // 串口调试，输出三相的占空比
  Serial.printf("%f, %f, %f\n", dc_a, dc_b, dc_c);
}

// Clark逆变换和Park逆变换
void setPhaseVoltage(float Uq, float Ud, float theta)
{
  theta = _normalizeAngle(theta); // 把角度归一化到0~2pi

  // 帕克逆变换
  Ualpha = -Uq * sin(theta);
  Ubeta = Uq * cos(theta);

  // 克拉克逆变换
  Ua = Ualpha + voltage_power_supply / 2;
  Ub = (sqrt(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
  Uc = (-Ualpha - sqrt(3) * Ubeta) / 2 + voltage_power_supply / 2;
  setPwm(Ua, Ub, Uc); // 驱动无刷电机运动
}

// Uq和电角度生成器，输入参数：target_velocity目标速度
float velocityOpenloop(float target_velocity)
{
  unsigned long now_us = micros(); // 获取从开启芯片以来的微秒数，它的精度是 4 微秒。 micros() 返回的是一个无符号长整型（unsigned long）的值

  float Ts = (now_us - open_loop_timestamp) * 1e-6f; // 计算当前每个Loop的运行时间间隔

  // 由于 micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数，在由70分钟跳变到0时，TS会出现异常，因此需要进行修正。如果时间间隔小于等于零或大于 0.5 秒，则将其设置为一个较小的默认值，即 1e-3f
  if (Ts <= 0 || Ts > 0.5f)
    Ts = 1e-3f;

  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
  shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
  // 以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
  // 如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

  // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
  // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
  float Uq = voltage_power_supply / 8;

  setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle));

  open_loop_timestamp = now_us; // 用于计算下一个时间间隔

  return Uq;
}

void setup()
{
  Serial.begin(9600);
  Serial.println("-------------------------");
  Serial.println("SimpleFOC Study!");

  // PWM配置
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);

  ledcSetup(0, 20000, 10); // pwm频道, 频率, 精度
  ledcSetup(1, 20000, 10); // pwm频道, 频率, 精度
  ledcSetup(2, 20000, 10); // pwm频道, 频率, 精度

  ledcAttachPin(pwmA, 0);
  ledcAttachPin(pwmB, 1);
  ledcAttachPin(pwmC, 2);
  Serial.println("完成PWM初始化设置");
  Serial.println("-------------------------");
  delay(3000);
}

void loop()
{
  velocityOpenloop(10);
}
