


#define E32_TTL_100
#define OPERATING_FREQUENCY 862

#include "Arduino.h"
#include "LoRa_E32.h"

//#include <arduino-timer.h>

#include <Wire.h>
#include <SoftwareSerial.h>
//#include <AltSoftSerial.h>


#include <Adafruit_BMP085.h>
Adafruit_BMP085 bmp;

#include "Adafruit_SHT31.h"
Adafruit_SHT31 sht31 = Adafruit_SHT31();

#include "Max44009.h"
Max44009 myLux(0x4A);

#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 10
#define TEMPERATURE_PRECISION 12
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
//DeviceAddress sol1_address=0x284A707997200383;
//DeviceAddress water_address=0x28A30F799720033E;
DeviceAddress sol1_address = { 0x28, 0x4A, 0x70, 0x79, 0x97, 0x20, 0x03, 0x83 };
DeviceAddress water_address = { 0x28, 0xA3, 0x0F, 0x79, 0x97, 0x20, 0x03, 0x3E };
DeviceAddress sol2_address = { 0x28, 0xF0, 0x1F, 0x79, 0x97, 0x20, 0x03, 0x35 };
DeviceAddress bat1_address = { 0x28, 0xB8, 0x0E, 0x79, 0x97, 0x20, 0x03, 0x50 };

SoftwareSerial mySerial(3, 4);  // e32 TX e32 RX
LoRa_E32 e32ttl100(&mySerial, 2, 8, 7);

SoftwareSerial mySerial_2(11, 12); // RX, TX

//auto timer = timer_create_default(); // create a timer with default settings

long c[16];
int i = 0;
signed long f = 0;
signed long g = 0;
unsigned int n = 0;
unsigned int nt = 0;
unsigned long gp = 0;
unsigned long gh = 0;
unsigned long gv = 0;
int k = 0;
int l = 0;
int m = 0;
int w = 0;
float gpf = 0;
float ghf = 0;
float gvf = 0;
float latt = 0;
double longt = 0;
//String data="";
String data1;
String data2;
//String data3="";
String data4 = "";
String data5 = "";
String data6 = "";
String data7 = "";

float mppt = 0;
float p = 0;
float v = 0;
char cc;

float pressure = 0;
float temperature = 0;
float humidity = 0;
float light = 0;
float av_light=0;
float sum_light=0;
int count=0;

float t_sol1 = 0;
float t_water = 0;
float t_sol2 = 0;
float t_bat1 = 0;

int ledPin = 9;
int fadeValue = 0;
bool fade = false;
int light_enable=0;

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D
int16_t x;
int16_t y;
int16_t z;

float X = 0;
float Y = 0;
float Z = 0;

float max_rate = 0;
float prev_rate = 0;
float curr_rate = 0;

unsigned long start_time = 0;
unsigned long stop_time = 0;
float period = 0;
float amplitude = 0;
float max_angle = 0;



void lights() {


  sum_light += myLux.getLux()/127.6;
  ++count;

  if (av_light > 35) {
    analogWrite(ledPin, 0);
    light_enable=0;
  }
  if (av_light < 30) {
    light_enable=1;
    if(fade==true){
      --fadeValue;
    }
    if(fade==false){
      ++fadeValue;
    }
   if(fadeValue==255){
    fade=true;
   }
   if(fadeValue==0){
    fade=false;
   }
  analogWrite(ledPin,fadeValue);
  delay(5);
  }

}

void sensor() {

  mySerial_2.listen();
  while (mySerial_2.available() <= 0) {
  }
  //data="";
  data1 = mySerial_2.readString();

  //data1="";
  //data2="";
  //data3="";
  k = data1.indexOf("SOL");
  l = data1.indexOf("BAT");
  m = data1.indexOf("END");
  //w=data.indexOf("BT3");

  data2 = data1.substring(k, l);
  data1 = data1.substring(l, m-1);
  data1 += '\n';
  //data3=data.substring(z);
  //e32ttl100.sendMessage(data);
  Serial.println("start");
  //Serial.println(data);
  Serial.println(data1);
  Serial.println(data2);

  //Serial.println(data);
  //break;

  mySerial.listen();

  Wire.requestFrom(8, 16);    // request 6 bytes from slave device #8

  i = 0;

  while (Wire.available()) { // slave may send less than requested

    c[i] = Wire.read();
    i++;

  }


  f = c[0] | c[1] << 8 | c[2] << 16 | c[3] << 24;
  g = c[4] | c[5] << 8 | c[6] << 16 | c[7] << 24;
  n = c[8];
  nt = c[9];
  gp = c[10] | c[11] << 8;
  gh = c[12] | c[13] << 8;
  gv = c[14] | c[15] << 8;

  latt = float(f) / 100000;
  longt = double(g) * 0.00001;
  gpf = double(gp) * 0.01;
  ghf = double(gh) * 0.01;
  gvf = double(gv) * 0.01;

  data4 = "";
  data4 += "GPS";
  data4 += '\t' + String(latt, 6) + '\t' + String(longt, 6) + '\t' + String(n) + '\t' + String(nt)
           + '\t' + String(gpf, 2) + '\t' + String(ghf, 2) + '\t' + String(gvf, 2) + '\n';

  //e32ttl100.sendMessage(data_send);
  Serial.println(data4);


  pressure = bmp.readPressure();
  temperature = sht31.readTemperature();
  humidity = sht31.readHumidity();

  av_light=sum_light/count;
  count=0;
  sum_light=0;
  
  data5 = "";
  data5 += "SEN";
  data5 += '\t' + String(pressure, 0) + '\t' + String(pressure / 101.325, 2) +
           '\t' + String(temperature, 2) + '\t' + String(humidity, 2) + '\t' + String(av_light , 2) + 
           '\t'+String(light_enable) + '\n';

  //e32ttl100.sendMessage(data_send);
  Serial.println(data5);

  

  sensors.requestTemperatures();
  t_sol1 = sensors.getTempC(sol1_address);
  t_water = sensors.getTempC(water_address);
  t_sol2 = sensors.getTempC(sol2_address);
  t_bat1 = sensors.getTempC(bat1_address);
  data6 = "";
  data6 += "THR";
  data6 += '\t' + String(t_sol1, 2) + '\t' + String(t_water, 2) + '\t' + String(t_sol2, 2) + '\t' + String(t_bat1, 2) + '\n';
  //e32ttl100.sendMessage(data);
  Serial.println(data6);

  if (period<0.2){
    period=0;
    amplitude=0;
    max_rate=0;
    max_angle=0;
  }
  
  data7 = "";
  data7 += "GEN";
  data7 += '\t' + String(period, 3) + '\t' + String(amplitude , 3) +
           '\t' + String(max_rate, 3) + '\t' + String(max_angle, 3) +  '\n';
  max_rate = 0;
  //e32ttl100.sendMessage(data_send);
  Serial.println(data7);

  e32ttl100.sendMessage(data1);
  e32ttl100.sendMessage(data2);
  //e32ttl100.sendMessage(data3);
  e32ttl100.sendMessage(data4);
  e32ttl100.sendMessage(data5);
  e32ttl100.sendMessage(data6);
  e32ttl100.sendMessage(data7);
  
  mySerial_2.listen();
  //return true; // keep timer active? true
}

void gyro() {
  getGyroValues(); // This will update x, y, and z with new values
  //Serial.print("X:");
  X = float(x) / 250;
  //Serial.println(X,3); //Here you can do some operations befor you use that value
  //For example set it on a surface and substract or add numbers to get 0,0,0 if you want that position to be your reference
  curr_rate = X;
  /*
    Serial.print(" Y:");
    Serial.print(y);

    Serial.print(" Z:");
    Serial.println(z);
  */
  if (abs(X) > max_rate) {
    max_rate = abs(X);
  }
  if (prev_rate * curr_rate < 0) {
    stop_time = micros();
    period = 2*(float(stop_time) - float(start_time)) / 1000000;
    amplitude = pow(period * 0.2019, 2.5615);
    max_angle = max_rate * period;
    /*
     if(period>0.1){
      
     
    Serial.print("period:");
    Serial.print(period, 3);
    Serial.println(" s");
    Serial.print("Amplitude:");
    Serial.print(amplitude, 3);
    Serial.println(" m");
    Serial.print("Max rate:");
    Serial.print(max_rate, 3);
    Serial.println(" deg/sec");
    Serial.print("Max angle:");
    Serial.print(max_angle, 3);
    Serial.println(" deg");
     
     }
    */
    start_time = micros();
    max_rate = 0;
  }
  
  prev_rate = curr_rate;
}



void getGyroValues() {

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
}

void setupL3G4200D(int scale) {
  //From Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00001001);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if (scale == 250) {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  } else if (scale == 500) {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  } else {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device
  Wire.write(address); // send register address
  Wire.write(val); // send value to write
  Wire.endTransmission(); // end transmission
}

int readRegister(int deviceAddress, byte address) {

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while (!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}


void setup() {

  //mySerial.begin(9600);
  mySerial_2.begin(9600);
  Serial.begin(115200);

  mySerial.listen();

  delay(500);

  Wire.begin();

  Serial.println("starting up L3G4200D");
  setupL3G4200D(250); // Configure L3G4200 - 250, 500 or 2000 deg/sec

  
  bmp.begin();

  sht31.begin(0x44);
  sht31.heater(false);

  sensors.begin();
  sensors.setResolution(sol1_address, TEMPERATURE_PRECISION);
  sensors.setResolution(water_address, TEMPERATURE_PRECISION);



  e32ttl100.begin();

  ResponseStructContainer c;
  c = e32ttl100.getConfiguration();
  Configuration configuration = *(Configuration*) c.data;
  configuration.ADDL = 30;
  configuration.ADDH = 0;
  configuration.CHAN = 0x04;
  configuration.OPTION.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;
  configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;

  configuration.OPTION.fec = FEC_1_ON;
  configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
  configuration.OPTION.transmissionPower = POWER_20;

  configuration.SPED.airDataRate = AIR_DATA_RATE_011_48;
  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.uartParity = MODE_00_8N1;
  e32ttl100.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  c.close();
  data1.reserve(90);
  data2.reserve(40);
  data4.reserve(80);
  data5.reserve(50);
  data6.reserve(50);
  data7.reserve(50);

  pinMode(ledPin, OUTPUT);
  mySerial_2.listen();
  // call the toggle_led function every 1000 millis (1 second)
  //timer.every(5000, sensor);
//analogWrite(ledPin, 50);
}

void loop() {
  //timer.tick(); // tick the timer
  
  lights();
  gyro();
  
  if (mySerial_2.available()) {
    sensor();
  }





}
