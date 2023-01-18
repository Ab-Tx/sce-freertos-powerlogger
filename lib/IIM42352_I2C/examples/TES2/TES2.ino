// ================================================================
//           TDK IIM-42352 G-SENSOR for ARDUINO UNO
//                         2021.08.30
//                          台灣-嘉修
// ================================================================
// 可以運行更快速的範例
// 加速規數據用MACRO負責讀取,不調函數浪費clock週期
// 將高低位元組合併後輸出數據
// ================================================================

//=================================================================
#include <Wire.h> // I2C library
#include <IIM42352_I2C.h>
IIM42352 IIM42SENSOR(IIM42352_DEVICE_ADDRESS_68);
byte rc;
byte val[6];  // ACC XYZ MSB:LSB 
short acc[3]; // ACC XYZ -32768~32767
byte count;
//=================================================

void setup() {
  Serial.begin(2000000);//bps:9600 115200 230400 250000 500000 1000000 2000000
  delay(1000);
  while (!Serial);
  Wire.begin();// I2C通訊啟用
  rc = IIM42SENSOR.init();// IIM-42352初始化
  if (rc != 0) {
    Serial.println("IIM-42352晶片初始化失敗");
     delay(60000); 
    Serial.flush();
  }
}
//===========================================================================================================================
void loop() {
    MACRO_IIM42352_ACC_READ(IIM42352_DEVICE_ADDRESS_68,0x1F,6,val,count)

    acc[0] = ((signed short)val[0] << 8) | (val[1]); //X ACC // MSB 8bit 串 LSB 8bit 變成16bit
    acc[1] = ((signed short)val[2] << 8) | (val[3]); //Y ACC
    acc[2] = ((signed short)val[4] << 8) | (val[5]); //Z ACC
    
    Serial.print(acc[0]);   // ACC_X
    Serial.print(',');
    Serial.print(acc[1]);   // ACC_Y
    Serial.print(',');
    Serial.println(acc[2]); // ACC_Z
}
