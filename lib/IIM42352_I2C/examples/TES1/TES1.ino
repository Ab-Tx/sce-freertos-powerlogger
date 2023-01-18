
#include <Wire.h>
#include <IIM42352_I2C.h>

IIM42352 IIM42SENSOR(IIM42352_DEVICE_ADDRESS_68);

void setup() {
  byte rc;

  Serial.begin(9600);
  Serial.println("IIM42352 Test Code");
  Serial.println("initializing ............");
  delay(2000);
  while (!Serial);
  Wire.begin();
  rc = IIM42SENSOR.init();
  if (rc != 0) {
    Serial.println(F("Initialization failed. Check data com"));
     delay(500); 
    Serial.flush();
  }
}

void loop() {
  byte rc;
  float acc[3];
  rc = IIM42SENSOR.get_val(acc);
  if (rc == 0) {
    Serial.write("(X) Value = ");
    Serial.print(acc[0],3);
    Serial.println(" [g]");
    Serial.write("(Y) Value = ");
    Serial.print(acc[1],3);
    Serial.println(" [g]");
    Serial.write("(Z) Value = ");
    Serial.print(acc[2],3);
    Serial.println(" [g]");
    Serial.println();
  }
 
  delay(500);

}
