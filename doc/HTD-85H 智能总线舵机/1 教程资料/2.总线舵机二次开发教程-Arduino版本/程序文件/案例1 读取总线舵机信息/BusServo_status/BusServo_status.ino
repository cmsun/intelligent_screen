#include "LobotSerialServoControl.h" // 导入库文件

// 读取总线舵机信息

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


void loop() {
   
    
    Serial.print("ID: ");
    Serial.println(BusServo.LobotSerialServoReadID(0xFE)); // 获取舵机ID并通过串口打印
    delay(500);// 延时
    
    int ID =1;
    Serial.print("Position: ");
    Serial.println(BusServo.LobotSerialServoReadPosition(ID)); // 获取舵机位置并通过串口打印
    delay(500); // 延时
    
    Serial.print("Vin: ");
    Serial.print(BusServo.LobotSerialServoReadVin(ID)/1000.0); // 获取舵机电压并通过串口打印
    Serial.println(" V");
    delay(900); // 延时

    Serial.print("Temp: ");
    Serial.println(BusServo.LobotSerialServoReadTemp(ID)); // 获取舵机温度并通过串口打印
    delay(500); // 延时

    Serial.print("Dev: ");
    Serial.println(BusServo.LobotSerialServoReadDev(ID)); // 获取舵机偏差并通过串口打印
    delay(1000); // 延时
 
}
