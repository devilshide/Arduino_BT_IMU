#include <SoftwareSerial.h>
#include <ADXL345.h>  // ADXL345 Accelerometer Library
#include <HMC5883L.h> // HMC5883L Magnetometer Library
#include <ITG3200.h>
#include <Wire.h>

#define DEBUG;

SoftwareSerial BTserial(10, 11); // RX | TX

//const long baudRate = 38400; 
const long baudRate = 9600; 
char c=' ';
String str = "";
boolean NL = true;

ADXL345 acc; //variable adxl is an instance of the ADXL345 library
HMC5883L compass;
ITG3200 gyro = ITG3200();
float  gx,gy,gz;
float goffsetX, goffsetY, goffsetZ;
float  gx_rate, gy_rate, gz_rate;
int error = 0;

void setup() 
{
    Serial.begin(38400);
    #ifdef DEBUG
    Serial.print("Sketch:   ");   Serial.println(__FILE__);
    Serial.print("Uploaded: ");   Serial.println(__DATE__);
    Serial.println(" ");
    #endif
 
    BTserial.begin(baudRate);  

    #ifdef DEBUG
    if (BTserial.available() > 0)
        Serial.println(BTserial.readString());
        
    Serial.print("BTserial started at "); Serial.println(baudRate);
    Serial.println(" ");
    #endif
    
    cmd("AT");
    cmd("AT+VERSION");
    cmd("AT+BAUD4");
    Serial.setTimeout(50);
    BTserial.setTimeout(50);

  acc.powerOn();
  compass = HMC5883L();
  error = compass.SetScale(1.3); // Set the scale to +/- 1.3 Ga of the compass
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  
  // Serial.println("Setting measurement mode to continous");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
    for (int i = 0; i <= 200; i++) {
      gyro.readGyro(&gx,&gy,&gz); 
      if (i == 0) {
        goffsetX = gx;
        goffsetY = gy;
        goffsetZ = gz;
      }
      if (i > 1) {
        goffsetX = (gx + goffsetX) / 2;
        goffsetY = (gy + goffsetY) / 2;
        goffsetZ = (gz + goffsetZ) / 2;
      }      
    }
    delay(1000);
    gyro.init(ITG3200_ADDR_AD0_LOW); 
    Serial.print("zero Calibrating...");
    gyro.zeroCalibrate(2500, 2);
    Serial.println("done.");
}
 
void loop()
{
  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (BTserial.available())
    Serial.println(BTserial.readString());

  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available()) {
    String str = Serial.readString();
    Serial.println(str);
    BTserial.println(str);
  }

  // Code fragment for Gyroscope (roll, pitch, yaw)  
  gyro.readGyro(&gx,&gy,&gz);
  gx_rate = (gx-goffsetX) / 14.375;
  gy_rate = (gy-goffsetY) / 14.375;
  gz_rate = (gz-goffsetZ) / 14.375;
  String g_str = "(gx_rate, gy_rate, gz_rate) = " + String(gx_rate) + ", " + String(gy_rate) + ", " + String(gz_rate);
  Serial.println(g_str);
  String b_g_str =  String((int)gx_rate) + "_" + String((int)gy_rate) + "_" + String((int)gz_rate) + "_0_0_0";
  BTserial.println(b_g_str);  

  delay(10);
}

void cmd(const char *msg) {
  BTserial.write(msg);
  unsigned long stime = millis();
  while((millis()-stime) < 1000) {
    if(BTserial.available()) {
      Serial.write(BTserial.read());
    }
  }
}
