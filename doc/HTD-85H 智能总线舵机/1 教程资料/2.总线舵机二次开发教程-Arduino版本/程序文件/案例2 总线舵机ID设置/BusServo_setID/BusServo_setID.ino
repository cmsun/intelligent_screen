#include "LobotSerialServoControl.h" // 导入库文件

// 控制总线舵机速度例程

#define SERVO_SERIAL_RX   35
#define SERVO_SERIAL_TX   12
#define receiveEnablePin  13
#define transmitEnablePin 14
HardwareSerial HardwareSerial(2);
LobotSerialServoControl BusServo(HardwareSerial,receiveEnablePin,transmitEnablePin);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);        // 设置串口波特率
  Serial.println("start...");  // 串口打印"start..."
  BusServo.OnInit();           // 初始化总线舵机库
  HardwareSerial.begin(115200, SERIAL_8N1, SERVO_SERIAL_RX, SERVO_SERIAL_TX);
  delay(500);                  // 延时500毫秒
}

bool start_en = true;

void loop() {
   
   if(start_en){  
    Serial.print("oldID: ");
    Serial.println(BusServo.LobotSerialServoReadID(0xFE)); // 获取舵机ID并通过串口打印
    delay(1000); // 延时
    
    uint8_t oldID =BusServo.LobotSerialServoReadID(0xFE);
    delay(1000); // 延时
    
    uint8_t newID =2;
    BusServo.LobotSerialServoSetID(oldID,newID);
    delay(1000); // 延时
    
    Serial.print("newID: ");
    Serial.println(String(newID)); // 获取舵机位置并通过串口打印
    delay(500); // 延时
  
    start_en = false;
  }
  else{
    delay(500); // 延时500毫秒
  }

}
