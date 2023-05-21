#include <Wire.h>
#include <INA226.h>

INA226 ina;

#include <Beastdevices_INA3221.h>
#define PRINT_DEC_POINTS  3



#include <Adafruit_MCP4725.h>

Adafruit_MCP4725 dac;

#include <SoftwareSerial.h>

SoftwareSerial mySerial(7, 6); // RX, TX


unsigned long incomingByte = 0;
float prevvoltage = 0;
float prevamp = 0;
float currentamp = 0;
float currentvoltage = 0;


unsigned long mppt = 2490;

Beastdevices_INA3221 ina3221(INA3221_ADDR41_VCC);
//float current[3];
float current_compensated[3];
float voltage[3];
float power[3];
float charge_counter = 3200000000;
float charge_counter2 = 3200;

float maxcounter = 3200;
float mincounter = 0;
float capacity = 0;
float percent = 0;
float max_volt = 14.4;
float min_volt = 11.2;
float charge_counter_2 = 2800000000;
float charge_counter2_2 = 2800;
float maxcounter_2 = 2800;
float mincounter_2 = 0;
float capacity_2 = 0;
float percent_2 = 0;
float prev_current[2] = {0, 0};
float test1 = 0;
float test2 = 0;
float dt = 0;
bool change = true;
int load = 0;

float sum_volt = 0;
float av_volt = 0;

float sum_power[4];
float av_power[4];

float sum_current[2];
float av_current[2];

int count = 0;

unsigned long period = 10000;
unsigned long time_now = 0;

#include <eRCaGuy_Timer2_Counter.h>
unsigned long start_time = 0;
unsigned long stop_time = 0;
unsigned long interval = 0;

void measurements(){
  current_compensated[0] = ina3221.getCurrentCompensated(INA3221_CH1);
  voltage[0] = ina3221.getVoltage(INA3221_CH1);
  sum_power[0] += current_compensated[0] * voltage[0];
  sum_current[0] += current_compensated[0];

  current_compensated[1] = ina3221.getCurrentCompensated(INA3221_CH2);
  voltage[1] = ina3221.getVoltage(INA3221_CH2);
  sum_power[1] += current_compensated[1] * voltage[1];
  sum_current[1] += current_compensated[1];

  current_compensated[2] = ina3221.getCurrentCompensated(INA3221_CH3);
  voltage[2] = ina3221.getVoltage(INA3221_CH3);
  sum_power[2] += current_compensated[2] * voltage[2];
}

void battery() {

/*
  current_compensated[0] = ina3221.getCurrentCompensated(INA3221_CH1);
  voltage[0] = ina3221.getVoltage(INA3221_CH1);
  sum_power[0] += current_compensated[0] * voltage[0];
  sum_current[0] += current_compensated[0];

  current_compensated[1] = ina3221.getCurrentCompensated(INA3221_CH2);
  voltage[1] = ina3221.getVoltage(INA3221_CH2);
  sum_power[1] += current_compensated[1] * voltage[1];
  sum_current[1] += current_compensated[1];

  current_compensated[2] = ina3221.getCurrentCompensated(INA3221_CH3);
  voltage[2] = ina3221.getVoltage(INA3221_CH3);
  sum_power[2] += current_compensated[2] * voltage[2];
*/
  interval = stop_time - start_time;
  stop_time = timer2.get_count();
  timer2.reset();

  start_time = timer2.get_count();

  //interval=stop_time-start_time;

  //Serial.print(dt);
  //Serial.println(" us");
  //dt2=dt/(1000000*3600);
  dt=float(interval);
  test1=dt * (prev_current[0]+current_compensated[0])/7.2;
  charge_counter += dt * (prev_current[0] + av_current[0]) / 7.2;
  charge_counter2 = charge_counter / 1000000;
  if (charge_counter2 > maxcounter || (voltage[0] > max_volt && current_compensated[0] < 0.05)) {
    maxcounter = charge_counter2;
  }

  if (charge_counter2 < mincounter || voltage[0] < min_volt) {
    mincounter = charge_counter2;
  }

  capacity = maxcounter - mincounter;

  percent = (charge_counter2 - mincounter) / capacity * 100;

  test2=dt * (prev_current[1]+current_compensated[1])/7.2;
  charge_counter_2 +=  dt * (prev_current[1] + av_current[1]) / 7.2;
  charge_counter2_2 = charge_counter_2 / 1000000;
  if (charge_counter2_2 > maxcounter_2 || (voltage[1] > max_volt && current_compensated[1] < 0.05)) {
    maxcounter_2 = charge_counter2_2;
  }

  if (charge_counter2_2 < mincounter_2 || voltage[1] < min_volt) {
    mincounter_2 = charge_counter2_2;
  }

  capacity_2 = maxcounter_2 - mincounter_2;

  percent_2 = (charge_counter2_2 - mincounter_2) / capacity_2 * 100;

  prev_current[0] = av_current[0];
  prev_current[1] = av_current[1];


  /*
    Serial.print("charge counter ");
    Serial.print(charge_counter,6);
    Serial.println(" Ah");
    Serial.print("charge counter 2 ");
    Serial.print(charge_counter2,2);
    Serial.println(" mAh");
    Serial.print("Capacity ");
    Serial.print(capacity,2);
    Serial.println(" mAh");
    Serial.print("Percentage ");
    Serial.print(percent,2);
    Serial.println(" %");
  */

  //if(timer.state() == RUNNING) Serial.println("timer running");


}
void print_battery() {

  av_volt = sum_volt / count;
  sum_volt = 0;

  for (int i = 0; i < 4; ++i) {
    av_power[i] = sum_power[i] / count;
    sum_power[i] = 0;
  }
  for (int i = 0; i < 2; ++i) {
    av_current[i] = sum_current[i] / count;
    sum_current[i] = 0;
  }

  count = 0;




  //current[0] = ina3221.getCurrent(INA3221_CH1);
  current_compensated[0] = ina3221.getCurrentCompensated(INA3221_CH1);
  voltage[0] = ina3221.getVoltage(INA3221_CH1);
  power[0] = current_compensated[0] * voltage[0];

  //current[1] = ina3221.getCurrent(INA3221_CH2);
  current_compensated[1] = ina3221.getCurrentCompensated(INA3221_CH2);
  voltage[1] = ina3221.getVoltage(INA3221_CH2);
  power[1] = current_compensated[1] * voltage[1];

  //current[2] = ina3221.getCurrent(INA3221_CH3);
  current_compensated[2] = ina3221.getCurrentCompensated(INA3221_CH3);
  voltage[2] = ina3221.getVoltage(INA3221_CH3);
  power[2] = current_compensated[2] * voltage[2];

  Serial.print("Channel 1: \n Current: ");
  //Serial.print(current[0], PRINT_DEC_POINTS);
  Serial.print("A\n Compensated current 1 : ");
  Serial.print(av_current[0], PRINT_DEC_POINTS);
  Serial.print("A\n Compensated current 2 : ");
  Serial.print(av_current[1], PRINT_DEC_POINTS);
  Serial.print("A\n Compensated current 3 : ");
  Serial.print(av_current[2], PRINT_DEC_POINTS);
  Serial.print("A\n Voltage: ");
  Serial.print(voltage[0], PRINT_DEC_POINTS);
  Serial.print("V\n Power 1: ");
  Serial.print(av_power[0], PRINT_DEC_POINTS);
  Serial.println("W");

  Serial.print(" Power 2: ");
  Serial.print(av_power[1], PRINT_DEC_POINTS);
  Serial.println("W");

  Serial.print(" Power 3: ");
  Serial.print(av_power[2], PRINT_DEC_POINTS);
  Serial.println("W");

  /*
    Serial.print("Channel 2: \n Current: ");
    Serial.print(current[1], PRINT_DEC_POINTS);
    Serial.print("A\n Compensated current: ");
    Serial.print(current_compensated[1], PRINT_DEC_POINTS);
    Serial.print("\n Voltage: ");
    Serial.print(voltage[1], PRINT_DEC_POINTS);
    Serial.println("V");

    Serial.print("Channel 3: \n Current: ");
    Serial.print(current[2], PRINT_DEC_POINTS);
    Serial.print("A\n Compensated current: ");
    Serial.print(current_compensated[2], PRINT_DEC_POINTS);
    Serial.print("\n Voltage: ");
    Serial.print(voltage[2], PRINT_DEC_POINTS);
    Serial.println("V\n");
  */
  Serial.print("time elapsed ms: ");
  Serial.print(interval);
  Serial.println(" us");


  Serial.print("charge counter 1 ");
  Serial.print(charge_counter2, 2);
  Serial.println(" mAh");
  Serial.print("charge counter 2 ");
  Serial.print(charge_counter2_2, 2);
  Serial.println(" mAh");
  Serial.print("charge counter 1 us ");
  Serial.print(test1, 2);
  Serial.println(" mAh");
  Serial.print("charge counter 2 us ");
  Serial.print(test2, 2);
  Serial.println(" mAh");
  Serial.print("Capacity ");
  Serial.print(capacity, 2);
  Serial.println(" mAh");
  Serial.print("Percentage ");
  Serial.print(percent, 2);
  Serial.println(" %");
  Serial.println(maxcounter);
  Serial.println(mincounter);

  if (percent > 80 && percent_2 > 80) {
    digitalWrite(9, HIGH);
    load = 1;
  }
  if (percent < 70 || percent_2 < 70) {
    digitalWrite(9, LOW);
    load = 0;
  }
}

void print_data() {

  Serial.print(mppt);
  Serial.print("\t");
  Serial.print(av_power[3] * 1000, 3);
  Serial.print("\t");
  Serial.println(av_volt * 100, 3);


  mySerial.print("SOL");
  mySerial.print("\t");
  mySerial.print(mppt);
  mySerial.print("\t");
  mySerial.print(av_power[3], 3);
  mySerial.print("\t");
  mySerial.println(av_volt, 3);

  mySerial.print("BAT");
  mySerial.print("\t");
  mySerial.print(voltage[0], 3);
  mySerial.print("\t");
  mySerial.print(percent, 3);
  mySerial.print("\t");
  mySerial.print(av_power[0], 3);
  mySerial.print("\t");
  mySerial.print(voltage[1], 3);
  mySerial.print("\t");
  mySerial.print(percent_2, 3);
  mySerial.print("\t");
  mySerial.print(av_power[1], 3);
  mySerial.print("\t");
  mySerial.print(av_power[2], 3);
  mySerial.print("\t");
  mySerial.print(load);
  mySerial.print("\t");
  mySerial.println("END");
  //mySerial.println("BT3");

}

void mppt_track() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.parseInt();
    if (incomingByte > 0) {
      // say what you got:
      mppt = incomingByte;

      //Serial.print("I received: ");
      //Serial.println(incomingByte,DEC);
    }
  }
  currentvoltage = ina.readBusVoltage();
  currentamp = ina.readShuntCurrent();
  float p = ina.readBusPower();
  float g = currentamp / currentvoltage;
  float di = currentamp - prevamp;
  float dv = currentvoltage - prevvoltage;
  float dg = di / dv;

  if (((dv = 0 & di < 0) || (dg < -g))&p > 0.150 & mppt < 4050) {
    mppt = mppt + 1;
  }

  if ((dv = 0 & di > 0) || (dg > -g)&mppt > 1550) {
    mppt = mppt - 1;
  }
  dac.setVoltage(mppt, false);
  prevvoltage = currentvoltage;
  prevamp = currentamp;

  sum_volt += currentvoltage;
  sum_power[3] += p;
  ++count;
  //Serial.println(mppt,DEC);
  //Serial.println(dv,4);
  //Serial.println(di,4);
  //Serial.println(dg,4);
  //Serial.println(-g,4);
}

void setup()
{
  Serial.begin(115200);
  mySerial.begin(9600);
  dac.begin(0x60);

  Serial.println("Initialize INA226");
  Serial.println("-----------------------------------------------");
  ina.begin(0x44);
  ina.configure(INA226_AVERAGES_4, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
  ina.calibrate(0.01, 4);


  ina3221.begin();
  ina3221.reset();

  // Set shunt resistors to 10 mOhm for all channels
  ina3221.setShuntRes(100, 100, 100);

  // Set series filter resistors to 10 Ohm for all channels.
  // Series filter resistors introduce error to the current measurement.
  // The error can be estimated and depends on the resitor values and the bus voltage.
  ina3221.setFilterRes(10, 10, 10);

  timer2.setup();



}






void loop() {

  mppt_track();
 // battery();
  measurements();


  if (millis() - time_now > period) {
    time_now = millis();


    print_battery();
    battery();
    print_data();

  }
}
