#include <Arduino.h>

void setup()
{
  Serial.begin(115200);
  Serial.println("Please input serial data.");
}

void loop()
{
  // 检查串口缓存中是否有数据等待读取
  if (Serial.available() > 0)
  {
    // char serialData = Serial.read();  // Serial.read()返回值数据类型为单个char
    // int serialData = Serial.parseInt();     // 接收串口中的整数数据
    float serialData = Serial.parseFloat(); // 接收串口中的浮点数据，只能接收两位有效小数

    Serial.print("RX:");
    Serial.println(serialData);
  }
}