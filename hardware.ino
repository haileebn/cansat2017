#include <DHT.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <math.h>

SFE_BMP180 pressure;

// DHT-22
#define DHTPIN 2 
#define DHTTYPE DHT22 
DHT dht(DHTPIN, DHTTYPE);

//PMS5003
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];
int PM01Value = 0;          
int PM25Value = 0;
int PM10Value = 0;


// CO MQ7
//#define buzzer 10
#define sensor A0
int coValue;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
//  Serial.println("DHTxx test!");
  initPressure();
  dht.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(5000);
  String result = readData();
  Serial.println(result);
}

String readData(){
  String result = "";
  result += "temp: " + String(getTemperature()) + "\n";
  result += "hud: " + String(getHumidity()) + "\n";
  result += "Pressure: " + String(getPressure(getTemperaturePressure())) + "\n";
  result += "Pressure-DHT22: " + String(getPressure(getTemperature())) + "\n";
  
  if(Serial.find(0x42)){    //start to read when detect 0x42
    Serial.readBytes(buf,LENG);
 
    if(buf[0] == 0x4d){
      if(checkValue(buf,LENG)){
        PM01Value = transmitPM01(buf); //count PM1.0 value of the air detector module
        PM25Value = transmitPM25(buf);//count PM2.5 value of the air detector module
        PM10Value = transmitPM10(buf); //count PM10 value of the air detector module 
      }           
    }
  }
  result += "pm1 :"+String(PM01Value) + "\n";
  result += "pm25:"+String(PM25Value)+"\n";
  result += "pm10:"+String(PM10Value)+"\n";
  coValue = (analogRead(sensor)*5*200)/1024;
  result += "CO:"+String(coValue)+"\n";
//  if(coValue>170) digitalWrite(buzzer,1);
  return result;
}


// return 1: success
// return 0: fail
int initPressure(){
//  String result = "";
  
  if (pressure.begin()){
    Serial.println("BMP180 init success");
    return 1;
  }
  else
  {
    Serial.println("BMP180 init fail (disconnected?)");
    return 0;
  } 
//  return result;
}

// return 0 fail
double getTemperature(){
  double t = dht.readTemperature();
  String result = "";
  // Check if any reads failed and exit early (to try again).
  if (isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  return t;
}

// return 0 fail
double getHumidity(){
  
  double h = dht.readHumidity();
  // Check if any reads failed and exit early (to try again).
  if (isnan(h)) {
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  return h;
}

// return 0 fail
double getPressure(double T){
  char status;
  double P;
  status = pressure.startPressure(3);
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);
    // Retrieve the completed pressure measurement:
    // Note that the measurement is stored in the variable P.
    // Use '&P' to provide the address of P.
    // Note also that the function requires the previous temperature measurement (T).
    // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getPressure(P,T);
    if (status != 0)
    {
      return P;
    }
    else return 0;
  }
  else return 0;
}

// return 0 fail
double getTemperaturePressure()
{
  char status;
  double T; //,P,p0,a;


  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
//    Serial.print("Temp of pressure: ");
//    Serial.println(T);
    if (status != 0)
    {
      return T;
    }
    else{
       Serial.println("error retrieving temperature measurement\n");
       return 0;
    }
  }
  else {
    Serial.println("error starting temperature measurement\n");
    return 0;
  }
}

/**
* t,p la nhiet do, ap suat o tren cao
* t0, p0 la nhiet do, ap suat ban dau duoi mat dat
*/
double getAltitudePressure(double t, double t0){
  const double R = 287.05; // J/Kg°K
  const double g = 9.80665; //m/s 2.
  
  if(t == 0 || t0 == 0){
    return 0;
  }
  else {
    double k = 273.15;
    double T = t + k;
    double T0 = t0 + k;
    double p = getPressure(t);
    double p0 = getPressure(t0);
    if(p == 0) return 0;
    else {
      return ((R/g)*((T+ T0)/2)*log10 (p0/p));
    }
  }
}


//Check pms5003 module
char checkValue(unsigned char *thebuf, char leng)
{  
  char receiveflag=0;
  int receiveSum=0;
 
  for(int i=0; i<(leng-2); i++){
  receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0x42;
  
  if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1]))  //check the serial data 
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  return receiveflag;
}

//transmit PM 1.0
int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val = ((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}
 
//transmit PM 2.5
int transmitPM25(unsigned char *thebuf)
{
  int PM25Val;
  PM25Val = ((thebuf[5]<<8) + thebuf[6]); //count PM2.5 value of the air detector module
  return PM25Val;
}
 
//transmit PM 10
int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val = ((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module  
  return PM10Val;
}

