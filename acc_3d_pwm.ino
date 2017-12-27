/*
 * Kontrola jednosmernog DC motora pomocu PWM impulsa koji se generise na  
 * osnovu akvizicionih podataka sa ADXL3xx - akcelerometra koji meri ubrzanja
 * u 3 ose. Kontrolu motora vrsi neki od pulse promenljivih koje kontrolisu osna
 * ubrzanja ili moduo superponiranog vektora ubrzanja.
 *
 */
#include <stdio.h>


const int x_acc = A3;
const int y_acc = A4;
const int z_acc = A5;


const int ENA = 2;
const int DIR = 3;
const int PLSX = 4;
const int PLSY = 5;
const int PLSZ = 6;
const int PLS = 7;

const float v_ref = 3.3;
const float zeroV_g_xy = 1.65;
const float zeroV_g_z = 1.8;
const float sensitivity = 0.33;
byte pulse_x,pulse_y,pulse_z,pulse;

short x_sensor,y_sensor,z_sensor;
float x_volts,y_volts,z_volts;

float rx,ry,rz,r;
float ax,ay,az,a;


byte scale(float x, float in_min, float in_max, float out_min, float out_max){

    int res = (int) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
    return (byte)res;


}

void getPulse(){

  float x_v,y_v,z_v;

   x_v = x_sensor * 3.3 / 1023;
   y_v = y_sensor * 3.3 / 1023;
   z_v = z_sensor * 3.3 / 1023;
      
  
   if (x_v < 0.33)
      x_v = 0.33;
   if (x_v > 2.97)
      x_v = 2.97;
         
   if (y_v < 0.33)
      y_v = 0.33;

    if (y_v > 2.97)
      y_v = 2.97;
      
   if (z_v < 0.33)
      z_v = 0.33;

    if (z_v > 2.97)
      z_v = 2.97;
    

    x_volts = (x_v - zeroV_g_xy);
    y_volts = (y_v - zeroV_g_xy);
    z_volts = (z_v - zeroV_g_z);

    
  
}


int rpm(byte pulse,double volts){

  double rpm = (volts * pulse/255) / 24 * 6200;
  return (int) rpm;

  
}

void getAcceleration(){

     rx = (x_volts / sensitivity);
     ry = (y_volts / sensitivity);
     rz = (z_volts / sensitivity);
     r = sqrt(pow(rx,2) + pow(ry,2) + pow(rz,2));

     
      
     ax = acos(rx / r);
     ay = acos(ry / r);
     az = acos(rz / r);
     a = sqrt(pow(ax,2) + pow(ay,2) + pow(az,2));
     
  
}
void setup() {
  // put your setup code here, to run once:
  pinMode(x_acc,INPUT); 
  pinMode(y_acc,INPUT);
  pinMode(z_acc,INPUT);

  pinMode(ENA,OUTPUT);  
  pinMode(DIR,OUTPUT);
  pinMode(PLSX,OUTPUT);
  pinMode(PLSY,OUTPUT);
  pinMode(PLSZ,OUTPUT);

  
  Serial.begin(9600);
  analogReference(EXTERNAL);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  x_sensor = analogRead(x_acc);
  y_sensor = analogRead(y_acc);
  z_sensor = analogRead(z_acc);
  getPulse();
  getAcceleration();

  pulse_x = scale(rx,-1.5,1.5,0,255);
  pulse_y = scale(ry,-1.5,1.5,0,255);
  pulse_z = scale(rz,-1.5,1.5,0,255);
  pulse = scale(r,0,2.5,0,255);
  

  Serial.print("Z: ");
  Serial.println(rz);

  digitalWrite(ENA,HIGH);
  digitalWrite(DIR,HIGH);
  
  if (ENA){

    analogWrite(PLSX,pulse_x);
    analogWrite(PLSY,pulse_y);
    analogWrite(PLSZ,pulse_z);
    analogWrite(PLS,pulse);

    Serial.print("RPM: ");
    Serial.println(rpm(pulse_z,5.0));

  
    delay(1000);

  }

}
