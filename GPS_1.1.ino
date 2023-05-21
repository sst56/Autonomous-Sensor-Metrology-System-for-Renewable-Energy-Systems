
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
/*
   This sample code demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 12, TXPin = 7;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

TinyGPSCustom pdop(gps, "GPGSA", 15); // $GPGSA sentence, 15th element
TinyGPSCustom hdop(gps, "GPGSA", 16); // $GPGSA sentence, 16th element
TinyGPSCustom vdop(gps, "GPGSA", 17); // $GPGSA sentence, 17th element
TinyGPSCustom total_satellites(gps, "GPGSV", 3); 


float latt=0;
float longt=0;
unsigned int n=0;
unsigned int nt=0;
long h=0;
unsigned long g=0;
int i=0;
float gp=0;
float gh=0;
float gv=0;

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  Wire.begin(8);                // join i2c bus with address #8
  Wire.onRequest(requestEvent);

}

void loop() {
  latt=gps.location.lat();
  longt=gps.location.lng();
  n=gps.satellites.value();
  nt=atoi(total_satellites.value());
  gp=atof(pdop.value());
  gh=atof(hdop.value());
  gv=atof(vdop.value());
  
  Serial.println(gps.location.lat(),11);
  Serial.println(gps.location.lng(),11);
  Serial.println(gps.satellites.value());
  Serial.println(total_satellites.value());
  Serial.println(pdop.value()); 
  Serial.println(hdop.value()); 
  Serial.println(vdop.value());
  
  smartDelay(1000);
 
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void requestEvent() {
  h=round(latt*100000);
  for(i; i<4; i++){
  Wire.write(byte(h)); // respond with message of 6 bytes
  //Serial.println(byte(h));
  h=h>>8;
  }
  i=0;
  h=round(longt*100000);
  for(i; i<4; i++){
  Wire.write(byte(h)); // respond with message of 6 bytes
  Serial.println(byte(h));
  h=h>>8;
  }
  i=0;
  Wire.write(byte(n));
  Wire.write(byte(nt));
  
   g=round(gp*100);
  Wire.write(byte(g));
  g=g>>8;
  Wire.write(byte(g)); 
    
    g=round(gh*100);
  Wire.write(byte(g));
  g=g>>8;
  Wire.write(byte(g));

   g=round(gv*100);
  Wire.write(byte(g));
  g=g>>8;
  Wire.write(byte(g));
}
