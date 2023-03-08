// Include iBus Library
#include <IBusBM.h>
#define zona_morta 4

// Create iBus Object
IBusBM ibus;

// Channel Values

int rcCH1 = 0;   // Joystick Eixo X, esquerda e direita
int rcCH2 = 0;   // Joystick Eixo Y, pra frente e pra tras
bool rcCH3 = 0;  // Utilizado para ligar e desligar
int rcCH4 = 0;   // Joystick esquedo eixo x, SEM USO
int rcCH5 = 0;   // VRA Pot esquero, Definir potencia do pulverizador
int rcCH6 = 0;   // VRB Pot direito, Definir potencia do adubador

// Motor RIGHT Control Connections
const byte R_PWM_DIR = 7;  //   VERDE
const byte L_PWM_DIR = 6;  //   LARANJA
const byte R_EN_DIR = 5;   //   AMARELO

// Motor LEFT Control Connections
const byte R_PWM_ESQ = 4;
const byte L_PWM_ESQ = 3;
const byte L_EN_ESQ = 2;

//SPRAY PWM output
const byte Spray = 8;

// Sensor de corrente para os motores
const int Sensor_I_M_DIR = A1;
const int Sensor_I_M_ESQ = A2;

// Sensor de Tensão para o Conjunto de bateria;
const int Sensor_V_BAT = A0;

// Motor Speed Values - Start at zero
int M_Speed_Esq = 0;
int M_Speed_Dir = 0;

// Motor Direction Values: 0 = backward, 1 = forward

int M_Dir_Esq = 1;
int M_Dir_Dir = 1;

// SPRAY control - Start at zero
int controlSpray = 0;

int state = 0;

void mControl_Esq(int mspeed, int mdir) {
  switch(mdir){
    case 0:
    digitalWrite(L_PWM_ESQ, LOW);
    digitalWrite(L_EN_ESQ, HIGH);
    analogWrite(R_PWM_ESQ, mspeed);
    break;
    case 1:
    // Motor forward
    digitalWrite(R_PWM_ESQ, LOW);
    digitalWrite(L_EN_ESQ, HIGH);
    analogWrite(L_PWM_ESQ, mspeed);
    break;
    case 2:
    digitalWrite(L_EN_ESQ, LOW);
    break;
  }
}

void mControl_Dir(int mspeed, int mdir) {
  switch(mdir){
    case 0:
    digitalWrite(L_PWM_DIR, LOW);
    digitalWrite(R_EN_DIR, HIGH);
    analogWrite(R_PWM_DIR, mspeed);
    break;
    case 1:
    digitalWrite(R_PWM_DIR, LOW);
    digitalWrite(R_EN_DIR, HIGH);
    analogWrite(L_PWM_DIR, mspeed);
    break;
    case 2:
    digitalWrite(R_EN_DIR, LOW);
    break;    
  }
  /*
  if (mdir == 0) {  // Determine direction
    // Motor backward
    digitalWrite(L_PWM_DIR, LOW);
    //digitalWrite(R_EN_DIR, LOW);
    analogWrite(R_PWM_DIR, mspeed);
  } else {
    // Motor forward
    digitalWrite(R_PWM_DIR, LOW);
    digitalWrite(R_EN_DIR, HIGH);
    analogWrite(L_PWM_DIR, mspeed);
  }*/
}

void cSpray(int control) {
  if (control > 60) {
    analogWrite(Spray, control);
  } else {
    //controlSpray = 0;
    analogWrite(Spray, 0);
  }
}

int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

void setup() {

  pinMode(R_PWM_DIR, OUTPUT);
  pinMode(L_PWM_DIR, OUTPUT);
  pinMode(R_EN_DIR, OUTPUT);

  pinMode(R_PWM_ESQ, OUTPUT);
  pinMode(L_PWM_ESQ, OUTPUT);
  pinMode(L_EN_ESQ, OUTPUT);

  pinMode(Sensor_I_M_DIR, INPUT);
  pinMode(Sensor_I_M_ESQ, INPUT);
  pinMode(Sensor_V_BAT, INPUT);

  // Start serial monitor for debugging
  Serial.begin(115200);
  // Attach iBus object to serial port
  ibus.begin(Serial1);
}

void loop(){
  att_canais();
  //Mostrar();
  controless();
}

void att_canais() {
  rcCH1 = readChannel(0, -255, 255, 0);
  rcCH2 = readChannel(1, -255, 255, 0);
  rcCH3 = readSwitch(2, 0);
  rcCH4 = readChannel(3, -100, 100, 0);
  rcCH5 = readChannel(4, 0, 255, 0);
  rcCH6 = readChannel(5, 0, 255, 0);
}

void controless() {
static byte estado_controless = 0;
switch (estado_controless){
   
    case 0:
    if(x>zm){
      estado = 1;
    }
    if (x<zm){
      estado = 2;
    }

break;
 case 1:
  if ((x<zm)&& (y<zm)){
estado = 0;     
  }

  break;








  
}
  

  if(rcCH1 <= zona_morta && rcCH1 >= -zona_morta && rcCH2 <= zona_morta && rcCH2 >= -zona_morta){
    state = 0;
    M_Dir_Dir = 2;
    M_Dir_Esq = 2;
  }else if((rcCH1 >= zona_morta || rcCH1 <= -zona_morta) && rcCH2 <= zona_morta && rcCH2 >= -zona_morta){
      state = 1;
  }

    

  if (state == 0){
    if(rcCH2 > 0){
      M_Dir_Esq = 1;
      M_Dir_Dir = 1;  
      M_Speed_Esq = abs(rcCH2);
      M_Speed_Dir = abs(rcCH2);
    }else{
      M_Dir_Esq = 0;
      M_Dir_Dir = 0;  
      M_Speed_Esq = abs(rcCH2);
      M_Speed_Dir = abs(rcCH2);
    }
    
    
  } else if(state == 1){
    if(rcCH1 > 0){
      M_Dir_Esq = 0;
      M_Dir_Dir = 1;  
      M_Speed_Esq = abs(rcCH1);
      M_Speed_Dir = abs(rcCH1);
    }else{
      M_Dir_Esq = 1;
      M_Dir_Dir = 0;  
      M_Speed_Esq = abs(rcCH1);
      M_Speed_Dir = abs(rcCH1);
    }

    Serial.print("rcCH1: ");
    Serial.print(M_Speed_Dir);

    Serial.print(" rcCH1: ");
    Serial.println(rcCH1);

  }

  
/*
  Serial.print("CH1: ");
  Serial.print(M_Speed_Esq);

  Serial.print(" CH2: ");
  Serial.println(rcCH2);
*/

  M_Speed_Dir = constrain(M_Speed_Dir, 0, 255);
  M_Speed_Esq = constrain(M_Speed_Esq, 0, 255);
  controlSpray = constrain(controlSpray, 0, 255);

  mControl_Dir(M_Speed_Dir, M_Dir_Dir);
  mControl_Esq(M_Speed_Esq, M_Dir_Esq);
  cSpray(controlSpray);
}

