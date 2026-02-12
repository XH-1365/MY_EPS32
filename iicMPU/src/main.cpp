#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// 创建 MPU6050 对象
Adafruit_MPU6050 mpu;

void setup() {
  // 初始化串口
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // 等待串口连接
  }

  Serial.println("=== ESP32-C3 + MPU6050 初始化 ===");

  // 初始化 I2C 总线
  Wire.begin(18, 19); // SDA: GPIO18, SCL: GPIO19

  // 初始化 MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 初始化失败！");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 初始化成功！");

  // 配置传感器范围（可选）
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("传感器配置完成");
}

void loop() {
  // 获取传感器事件
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 打印加速度数据
  Serial.print("加速度 X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.println(a.acceleration.z);

  // 打印角速度数据
  Serial.print("角速度 X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.println(g.gyro.z);

  // 打印温度数据
  Serial.print("温度: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.println("------------------------");

  delay(500); // 每 500ms 打印一次数据
}