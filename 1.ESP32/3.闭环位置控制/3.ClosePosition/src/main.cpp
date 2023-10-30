/*程序功能：1.闭环角度控制，让电机转到指定位置
 *2.电压力矩闭环
 */
#include <Arduino.h>
#include "AS5600.h"
// PWM输出引脚
const int pwmA = 32;
const int pwmB = 33;
const int pwmC = 25;
#define _3PI_2 4.71238898038f

// 幅值限制函数
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// 变量定义
const int pole_pairs = 7;        // 电机的极对数
float voltage_power_supply = 12; // 电机供电电压

// float shaft_angle = 0;         // 机械角度
float open_loop_timestamp = 0; // 每次循环执行的时间
float Ualpha = 0, Ubeta = 0, Ua = 0, Ub = 0, Uc = 0, dc_a = 0, dc_b = 0, dc_c = 0;

float zero_electric_angle = 0; // 电角度零位偏差
int DIR = -1;                   // 编码器的方向需要自己测试得到，逆时针(-1)，顺时针(1)

// 角度归一化，归一化角度到 [0,2PI]
float _normalizeAngle(float angle)
{
  float a = fmod(angle, 2 * PI); // 取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2 * PI);
}

// 电角度求解
float _electricalAngle()
{
  return _normalizeAngle((float)(DIR * pole_pairs) * getAngle_Without_track() - zero_electric_angle);
}

// 驱动无刷电机，设置PWM输出，由计算得到的Ua、Ub、Uc，得到输出的占空比
void setPwm(float Ua, float Ub, float Uc)
{
  // 限制占空比从0到1
  dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  // 写入PWM到PWM 0 1 2 通道
  ledcWrite(0, dc_a * 255);
  ledcWrite(1, dc_b * 255);
  ledcWrite(2, dc_c * 255);

  // 串口调试，输出三相的占空比
  // Serial.printf("%f, %f, %f\n", dc_a, dc_b, dc_c);
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

//==============串口接收==============
float motor_target;
int commaPosition;
String serialReceiveUserCommand()
{

  // a string to hold incoming data
  static String received_chars;

  String command = "";

  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;

    // end of user input
    if (inChar == '\n')
    {

      // execute the user command
      command = received_chars;

      commaPosition = command.indexOf('\n'); // 检测字符串中的逗号
      if (commaPosition != -1)               // 如果有逗号存在就向下执行
      {
        motor_target = command.substring(0, commaPosition).toDouble(); // 电机角度
        Serial.println(motor_target);
      }
      // reset the command buffer
      received_chars = "";
    }
  }
  return command;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("-------------------------");
  Serial.println("SimpleFOC Study!");

  // PWM配置
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);

  ledcSetup(0, 30000, 8); // pwm频道, 频率, 精度
  ledcSetup(1, 30000, 8); // pwm频道, 频率, 精度
  ledcSetup(2, 30000, 8); // pwm频道, 频率, 精度

  ledcAttachPin(pwmA, 0);
  ledcAttachPin(pwmB, 1);
  ledcAttachPin(pwmC, 2);
  Serial.println("完成PWM初始化设置");
  Serial.println("-------------------------");

  BeginSensor(); // 配置AS5600编码器的引脚

  // 电角度零位偏差校准
  setPhaseVoltage(3, 0, _3PI_2);
  delay(3000);
  zero_electric_angle = _electricalAngle();
  setPhaseVoltage(0, 0, _3PI_2);

  Serial.print("0电角度：");
  Serial.println(zero_electric_angle);
}



void loop()
{
  Serial.println(getAngle());
  float Sensor_Angle = getAngle();
  float Kp = 0.133;
  setPhaseVoltage(_constrain(Kp * (motor_target - DIR * Sensor_Angle) * 180 / PI, -6, 6), 0, _electricalAngle());
  serialReceiveUserCommand();
}