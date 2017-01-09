#include <mcp_can.h>
#include <SPI.h>
#include <PID_v1.h>

const int SPI_CS_PIN = 9;
#define pinThrIn A0
#define pinThrOut A1
#define pinBreak 2
#define pinLed 13
#define pinBoton 3
#define timi 500 // Como el arduino hace más de una ejecución por ms, hay que ajustar el valor para que el crucero no se desactive inmediatamente después de encenderlo.

//Variables para el PID
int RPM = 0;
int RPM2 = 60;
float Kp=0;
float Ki=10;
float Kd=0;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
const int sampleRate = 1;
const long serialPing = 500;
unsigned long now = 0;
unsigned long lastMessage = 0;

//Variables CAN
unsigned char flagRecv = 0;
unsigned char len = 0;
unsigned char buff[7];
unsigned long canId;

int Vin;
int Vcruise;
int crucero;
bool cruise = false;
unsigned long lastChange = 0;


MCP_CAN CAN(SPI_CS_PIN);

void MCP2515_ISR(){
  flagRecv=1;
}

void setup() {
  pinMode(pinThrIn,INPUT);
  pinMode(pinThrOut,OUTPUT);
  pinMode(pinBreak,INPUT);
  pinMode(pinLed,OUTPUT);
  pinMode(pinBoton,INPUT);

  Serial.begin(9600);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(sampleRate);
  Serial.println("Begin");
  lastMessage=millis();

START_INIT:

  if (CAN_OK == CAN.begin(CAN_1000KBPS))                
  {
    Serial.println("CAN BUS Shield esta ready papi!");
  }
  else
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println("Init CAN BUS Shield again");
    delay(100);
    goto START_INIT;
  }
  attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt
  // rx buffer clearing
  while (Serial.available() > 0) {
    byte c = Serial.read();
  }
  
}

void loop() {
  Vin=analogRead(pinThrIn);         //Lectura de voltaje acelerador
  crucero = digitalRead(pinBoton);  //
  if(crucero && !cruise && (millis() - lastChange > timi)){       // Cruise deshabilitado en Arduino, boton presionado y el último cambio ocurrido hace mas de 'timi' segundos
    cruise=true;
    lastChange = millis();
  } else if(crucero && cruise && (millis() - lastChange > timi)){ // Cruise habilitado en Arduino, boton presionado y el último cambio ocurrido hace mas de 'timi' segundos
    cruise = false;
    lastChange = millis();
  }
  if (!cruise){                     //Cruise deshabilitado, se repite info del pedal en el pin de salida
    analogWrite(pinThrOut,Vin);
  } else {                          //Cruise habilitado, gestionar mediante PID la aceleracion del Arduino.
    Input=RPM;
    Setpoint=RPM2;
    myPID.Compute();
    analogWrite(pinThrOut,Output);
    now = millis();
    if(now-lastMessage>serialPing){
      Serial.print("Setpoint=");
      Serial.print(Setpoint);
      Serial.print("Input");
      Serial.print(Input);
      Serial.print("Output=");
      Serial.print(Output);
      Serial.print("\n");
      if (Serial.available()>0){
        for (int x=0; x<4; x++){
          switch(x){
            case 0:
              Kp=Serial.parseFloat();
              break;
            case 1:
              Ki=Serial.parseFloat();
              break;
            case 2:
              Kd=Serial.parseFloat();
              break;
             case 3:
             for (int y= Serial.available(); y==0; y--) {
              Serial.read();
              }
              break;
              }
              }
              Serial.print("Kp,Ki,Kd =");
              Serial.print(Kp);
              Serial.print(",");
              Serial.print(Ki);
              Serial.print(",");
              Serial.print(Kd);
              myPID.SetTunings(Kp,Ki,Kd);
              }
              lastMessage=now;
          }
  }
}
