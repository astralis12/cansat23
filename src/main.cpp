#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>
#include "mpumine.h"
#include "bmp.h"
#include "telemetry.h"
#include "component.h"
#include <TeensyThreads.h>

#define CS 10
#define xbee Serial2

Mpumine mpu(Wire1);
bmp_read bmed;
TinyGPSPlus gps;
File myFile;

unsigned long previousTime = 0;const long interval = 1000;unsigned long currentTime;
extern unsigned long packetCount; extern bool tele_command, tele_calibration, tele_enable, tele_sim;extern float sim_press;
float accelX,accelY,gForce,l_gforce,accelZ,value_roll,value_pitch,c,temp=0,press,altit,last_altit=0,ref,lat=0,lng=0,eprom,voltase=5.0,gps_altitude=0;    //MPU, BME, GPS, EEPROM
int packet[3] = {0,0,0},time[7]={0,0,0,0,0,0,0},gps_satelite=0,timer_mil,paket_xbee=0,error; bool var_sim;    //GPS
int no=0,i,sensor_counter=0; int n ; String ayaya[100]; int k=0,state; String hasil, tele; char tampung; bool lock=false;    //PARSING

void DI(){
  while(1){

    if (gps.location.isValid()) {    
  lat = gps.location.lat();
  lng = gps.location.lng();
  }
  if (gps.date.isValid()) {   
  time[0] = gps.date.month();
  time[1] = gps.date.day();
  time[2] = gps.date.year();
  }
  if (gps.time.isValid()) {   
  time[3] = gps.time.hour();
  time[4] = gps.time.minute();
  time[5] = gps.time.second();
  time[6] = gps.time.centisecond();
  }
  if (gps.altitude.isValid()) {
  gps_altitude = gps.altitude.meters();
  }
  if (gps.satellites.isValid()) {
  gps_satelite = gps.satellites.value();
  }
  threads.yield();
  }
}

void sensor(){
  while(1){
  mpu.update_sens();
  l_gforce = mpu.readGforce();
  while(1){
    bmed.bmp_error(ref);
    temp = bmed.read_temp();
    press = bmed.read_press();
    if (tele_calibration == true){
      ref = bmed.read_press();
      bmed.tele_calibration(ref);
      tele_calibration=false;
    }
    if (tele_sim == true){
      altit = bmed.output_bmp(bmed.read_altitude_sim(sim_press));
    }
    
    else {
      altit = bmed.output_bmp(bmed.read_altitude(ref));
  }
    if (error!=0||(mpu.readacc_x()&&mpu.readacc_y()&&mpu.readacc_z())==0) { 
      mpu.begin();
    }else { 
      accelX = mpu.readacc_x();
      accelY = mpu.readacc_y();
      accelZ = mpu.readacc_z();
      value_roll = mpu.read_tiltx();
      value_pitch = mpu.read_tilty();
      gForce = (mpu.readGforce()+l_gforce)/2;
  }

    telemetry().detect_mode(tele_sim);
    telemetry().detect_state(packetCount,gForce,press,altit,last_altit);
    last_altit = altit;
    l_gforce = gForce;sensor_counter++;
    threads.delay(10);
    }
    threads.yield();        
    }
  }

void parsing(){
  while(1){
  n = hasil.length();     
  char inChar[n+1];   
  strcpy(inChar, hasil.c_str());   
  for ( i=0;i<n;i++) {    
    if (inChar[i]==','){  
      k++; 
    }
    else {
      if(inChar[i] >= 30 && inChar[i] <= 122) {
        ayaya[k]+=inChar[i];  
      }
    }
  };
  
  telemetry().tele_readcomm(ayaya[0], ayaya[1], ayaya[2], ayaya[3]);
  for (i=0;i<n;i++) { 
    ayaya[i] = "";  
  }
  k=0;
  lock=false;

  threads.yield();
  }
}

void sim(){
  while(1){
      while (1) {

  telemetry().distort(altit,temp,press,value_pitch,value_roll,voltase,time[3],time[4],time[5],lat,lng,gps_altitude,gps_satelite);
  tele = telemetry().constructMessage();
  tele.replace(" ", "");

  hasil = "";
  while(Serial2.available()) {
    tampung=(char)Serial2.read();
    hasil+=tampung;
    lock=true;
  }
  if(lock==true) {parsing();}  
    threads.delay(300);
  }
      threads.yield();
  }
}

void eeprom(){
  while(1){
  SD.begin(CS); 
  myFile = SD.open("1088.csv", FILE_WRITE);
  myFile.println("TeamID,Date,Count,Mode,State,Altitude,HS,PC,MAST,Temperature,Pressure,Voltage,DateGPS,AltiGPS,LAT,LNG,Satelite,TILTX,TILTY,ECHO");
  myFile.close();
  while(1) {
  myFile = SD.open("1088.csv", FILE_WRITE);  
  if (myFile) {
  myFile.print(tele);
  myFile.close();   
  }              
  threads.delay(1000);
  }
  threads.yield();
  } 
}

void gps_sys(){
  while(1){
    threads.delay(100);
      Serial.println("laut");
    }
}

void print(){
  while(1){
    while (1) {
    currentTime = millis();

    while(Serial3.available()>0) {    
    if (gps.encode(Serial3.read()))  
        DI();     
        }
    
    if (currentTime - previousTime >= interval) {
      previousTime = currentTime;
      if (tele=="") {;}
      else {
      Serial.println(tele);
      if (tele_command==true) {
        Serial2.print(tele);
      }
      packetCount++;
      }
    }
    threads.delay(10);
    }
    threads.yield();
  }
    Serial.println("texas");
}

void setup(){
  Serial.println("beruang");
  Serial.begin(9600);
  Serial2.begin(115200);
  Serial3.begin(9600);
  mpu.begin();
  bmed.begin();
  temp = bmed.read_temp();

  while (temp<26)
  {
    temp = bmed.read_temp();
    press = bmed.read_press();
    altit = bmed.read_altitude(1023.5);
  }

  ref = bmp.pressure/100;
  Serial.println("mengontol");

  threads.addThread(sensor,1);
  threads.addThread(parsing,1);
  threads.addThread(sim,1);
  threads.addThread(eeprom,1);
  threads.addThread(print,1);

}

void loop(){

}

//check while yg gps 
