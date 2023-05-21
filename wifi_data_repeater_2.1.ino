#define E32_TTL_100
#define OPERATING_FREQUENCY 862
#include "Arduino.h"
#include "LoRa_E32.h"

#define BLYNK_PRINT Serial // Comment this out to disable prints and save space
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>


#include <SoftwareSerial.h>
SoftwareSerial mySerial(D2, D3);  // e32 TX e32 RX
LoRa_E32 e32ttl100(&mySerial, D5, D7, D6);

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "6PGJZ1Md0ZQXaXDByA0ZijqGPCBVp2Qk";

// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "peter";
char pass[] = "daki2019@!";


  int c[16];
  int i=0;
  signed long f=0;
  signed long g=0;
  unsigned int n=0;
  unsigned int nt=0;
  unsigned long gp=0;
  unsigned long gh=0;
  unsigned long gv=0;

float gpf=0;
float ghf=0;
float gvf=0;

float mppt=0;
float p=0;
float v=0;

float pressure=0;
float pressure2=0;
float temperature=0;
float humidity=0;
float light=0;

float t_sol1=0;
float t_water=0;
float t_sol2=0;
float t_bat1=0;

float bat1_voltage=0;
float bat1_power=0;
float bat1_percent=0;

float bat2_voltage=0;
float bat2_power=0;
float bat2_percent=0;

float p_load=0; 
//SoftwareSerial mySerial(2, 0); // RX, TX
int x_0;
int x=0;
int y=0;
int load_enable=0;
int light_enable=0;
String temp;
double latt=0;
double longt=0;
String data;
BlynkTimer timer;

WidgetMap myMap(V5);

const int analogInPin = A0;
int sensorValue=0;
float receiver_battery_voltage=0;

float period=0;
float amplitude=0;
float max_rate=0;
float max_angle=0;

long time_now=0;

IPAddress ip;


void setup()
{
 mySerial.begin(9600);
 Serial.begin(115200); // See the connection status in Serial Monitor
 delay(500);

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
 
 Blynk.begin(auth, ssid, pass);
 timer.setInterval(1000L,sensorDataSend);


 // Setup a function to be called every second
 Blynk.run(); // Initiates Blynk
}

void loop(){
  
  Blynk.run();
  //timer.run();
  if (e32ttl100.available()>1) {
    sensorDataSend();
  }
if ((WiFi.status()!=WL_CONNECTED) && (millis() - time_now > 30000)) {
    time_now = millis();
    WiFi.disconnect();
    WiFi.reconnect();

    

  }



}


void sensorDataSend(){

  
  
  if (e32ttl100.available()>1) {
    // read the String message
  ResponseContainer rc = e32ttl100.receiveMessage();
  // Is something goes wrong print error
  if (rc.status.code!=1){
    rc.status.getResponseDescription();
  }else{
    // Print the data received
    Serial.println(rc.data);

    data = rc.data;
  }
  }

    sensorValue = analogRead(analogInPin);
    receiver_battery_voltage=0.0046193*1.002697*float(sensorValue);
    // print the readings in the Serial Monitor
    //Serial.print("Voltage = ");
    Serial.println(receiver_battery_voltage,3);
    //Serial.println(" V");



    if(data.indexOf("GPS")!=-1){
    y=data.indexOf("GPS")+4;
    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    latt=temp.toFloat();
    
    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    longt=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    n=temp.toInt();   

    y=x+1;
    
    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    nt=temp.toInt(); 

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    gpf=temp.toFloat();
    
    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    ghf=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    gvf=temp.toFloat();
    
    x=0;
    y=0;
    
    
    //Serial.print("my string is:");
    Serial.println("Data Received");
    Serial.println(latt,6);
    Serial.println(longt,6);
    Serial.println(n);
    Serial.println(nt);
   
 //Blynk.virtualWrite(V1, mppt); // Humidity for gauge
 //Blynk.virtualWrite(V2, p); // Temperature for gauge
 //Blynk.virtualWrite(V3, v); // Humidity for graph


//delay(1000);

  }

  if(data.indexOf("SOL")!=-1){
    y=data.indexOf("SOL")+4;
    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    mppt=temp.toFloat();
    
    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    p=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    v=temp.toFloat();   

    y=x+1;
  } 

    Serial.println(p,6);
  Serial.println(v,6);
  Serial.println(mppt);

   
   if(data.indexOf("SEN")!=-1){
    y=data.indexOf("SEN")+4;
    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    pressure=temp.toFloat();
    
    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    pressure2=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    temperature=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    humidity=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    light=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    light_enable=temp.toInt();

   }

   if(data.indexOf("THR")!=-1){
    y=data.indexOf("THR")+4;
    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    t_sol1=temp.toFloat();
    
    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    t_water=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    t_sol2=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    t_bat1=temp.toFloat();
   
   
   }
  
  if(data.indexOf("BAT")!=-1){
    y=data.indexOf("BAT")+4;
    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    bat1_voltage=temp.toFloat();
    
    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    bat1_percent=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    bat1_power=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    bat2_voltage=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    bat2_percent=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    bat2_power=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    p_load=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    load_enable=temp.toFloat();
   
   }

   if(data.indexOf("GEN")!=-1){
    y=data.indexOf("GEN")+4;
    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    period=temp.toFloat();
    
    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    amplitude=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    max_rate=temp.toFloat();

    y=x+1;

    x=data.indexOf("\t",y);
    temp=data.substring(y,x);
    max_angle=temp.toFloat();
   
   
   }
 myMap.location( 1,latt, longt, "Prototype 1"); 
 //Blynk.virtualWrite(V5,1, float(f)*0.00001,float(g)/100000); // Humidity for gauge
 Blynk.virtualWrite(V6, n); // Temperature for gauge
 Blynk.virtualWrite(V7, latt);
 Blynk.virtualWrite(V8, longt);
 Blynk.virtualWrite(V9, nt); // Temperature for gauge
 Blynk.virtualWrite(V10, gpf);
 Blynk.virtualWrite(V11, ghf);
 Blynk.virtualWrite(V12, gvf);

 ip=WiFi.localIP();
 Blynk.virtualWrite(V14, WiFi.RSSI()); 
 Blynk.virtualWrite(V15, receiver_battery_voltage); 
 //Blynk.virtualWrite(V16, ip); 
 Serial.println(ip);
 
 Blynk.virtualWrite(V1, mppt);
 Blynk.virtualWrite(V2, p);
 Blynk.virtualWrite(V3, v); 

 Blynk.virtualWrite(V17, pressure);
 Blynk.virtualWrite(V18, pressure2);
 Blynk.virtualWrite(V19, temperature); 
 Blynk.virtualWrite(V20, humidity);
 Blynk.virtualWrite(V21, light);

 Blynk.virtualWrite(V25, t_sol1);
 Blynk.virtualWrite(V26, t_water);
 Blynk.virtualWrite(V27, t_sol2);
 Blynk.virtualWrite(V28, t_bat1);

 Blynk.virtualWrite(V30, bat1_voltage);
 Blynk.virtualWrite(V31, bat1_power);
 Blynk.virtualWrite(V32, bat1_percent);

 Blynk.virtualWrite(V35, bat2_voltage);
 Blynk.virtualWrite(V36, bat2_power);
 Blynk.virtualWrite(V37, bat2_percent);
 
 
 
 Blynk.virtualWrite(V40, p_load);

 Blynk.virtualWrite(V50, light_enable);
 Blynk.virtualWrite(V51, load_enable);

 Blynk.virtualWrite(V60, period);
 Blynk.virtualWrite(V61, amplitude);
 Blynk.virtualWrite(V62, max_rate);
 Blynk.virtualWrite(V63, max_angle);

}
  
