// Include iBus Library
#include <IBusBM.h>

// Create iBus Object
IBusBM ibus;

// Channel Values

int rcCH1 = 0;   // Joystick Eixo X, esquerda e direita
int rcCH2 = 0;   // Joystick Eixo Y, pra frente e pra tras
bool rcCH3 = 0;  // Utilizado para ligar e desligar
int rcCH4 = 0;  // Joystick esquedo eixo x, SEM USO
int rcCH5 = 0;   // VRA Pot esquero, Definir potencia do pulverizador
int rcCH6 = 0;  // VRB Pot direito, Definir potencia do adubador

// Motor RIGHT Control Connections
const byte R_PWM_DIR = 7; //   VERDE
const byte L_PWM_DIR = 6; //   LARANJA
const byte R_EN_DIR = 5;  //   AMARELO

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

// Control Motor A
void mControl_Esq(int mspeed, int mdir) {  
  if (mdir == 0) {  // Determine direction
    // Motor backward
    digitalWrite(L_PWM_ESQ, LOW); 
    //digitalWrite(L_EN_ESQ, LOW);
    analogWrite(R_PWM_ESQ, mspeed);
  } else {
    // Motor forward
    digitalWrite(R_PWM_ESQ, LOW);
    digitalWrite(L_EN_ESQ, HIGH); 
    analogWrite(L_PWM_ESQ, mspeed); 
  }
}

// Control Motor B
void mControl_Dir(int mspeed, int mdir) {
  if (mdir == 0) {  // Determine direction
    // Motor backward    
    digitalWrite(R_PWM_DIR, LOW);
    //digitalWrite(R_EN_DIR, LOW);
    analogWrite(L_PWM_DIR, mspeed);
  } else {
    // Motor forward
    digitalWrite(L_PWM_DIR, LOW);
    digitalWrite(R_EN_DIR, HIGH);
    analogWrite(R_PWM_DIR, mspeed);
  }
}

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
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

void setup(){
  
pinMode(R_PWM_DIR,OUTPUT);
pinMode(L_PWM_DIR,OUTPUT);
pinMode(R_EN_DIR,OUTPUT);

pinMode(R_PWM_ESQ,OUTPUT);
pinMode(L_PWM_ESQ,OUTPUT);
pinMode(L_EN_ESQ,OUTPUT);

pinMode(Sensor_I_M_DIR,INPUT);
pinMode(Sensor_I_M_ESQ,INPUT);
pinMode(Sensor_V_BAT,INPUT);  
  

  // Start serial monitor for debugging
  Serial.begin(115200);

  // Attach iBus object to serial port
  ibus.begin(Serial1);

  // Set all the motor control pins to outputp
  // Keep motors on standby for two seconds & flash LED
}

void Mostrar() {
  Serial.print("Ch1 = ");
  Serial.print(rcCH1);

  Serial.print(" Ch2 = ");
  Serial.print(rcCH2);

  Serial.print(" Ch3 = ");
  Serial.print(rcCH3);

  Serial.print(" Ch4 = ");
  Serial.print(rcCH4);

  Serial.print(" Ch5 = ");
  Serial.print(rcCH5);

  Serial.print(" Ch6 = ");
  Serial.println(rcCH6);
}

void att_canais() {
  rcCH1 = readChannel(1, -255, 255, 0);
  rcCH2 = readChannel(0, -255, 255, 0);
  rcCH3 = readSwitch(2, 0);
  rcCH4 = readChannel(3, -100, 100, 0);
  rcCH5 = readChannel(4, 0, 255, 0);
  rcCH6 = readChannel(5, 0, 255, 0);
}

void att_sensores() {

}

void loop() {
  att_canais();
  int zona_morta = 4;

  //############################# JOYSTICK FRENTE E TRÁS == PARA FRENTE ################

  if (rcCH2 > 4) { // Se o eio Y estiver positivo vai pra frente tendendo a esquerda ou a direita a depender do eixo x

    M_Dir_Esq = 1; // 1 == FRENTE // 0 == TRAS
    M_Dir_Dir = 1; // 1 == FRENTE // 0 == TRAS

    if (rcCH1 > zona_morta) {

      Serial.println("ROBO EM FRENTE PRA DIREITA");
      M_Speed_Esq = rcCH2;
      M_Speed_Dir = rcCH2 - (rcCH1 / 2);


    } else if ((rcCH1 <= zona_morta) && (rcCH1 >= -zona_morta)) {

      Serial.println("ROBO EM FRENTE");
      M_Speed_Esq = rcCH2;
      M_Speed_Dir = rcCH2;

    } else if (rcCH1 < -zona_morta) {

      Serial.println("ROBO EM FRENTE PRA ESQUERDA");
      M_Speed_Esq = rcCH2 - (abs(rcCH1) / 2);
      M_Speed_Dir = rcCH2;

    }

    //############################# JOYSTICK FRENTE E TRÁS == MEIO ################

  } else if ((rcCH2 <= zona_morta) && (rcCH2 >= -zona_morta)) {

    if (rcCH1 > zona_morta) {
      Serial.println("ROBO PARA A DIREITA");

      M_Dir_Esq = 1; // 1 == FRENTE // 0 == TRAS // MOTOR ESQUERDO
      M_Dir_Dir = 0; // 1 == FRENTE // 0 == TRAS // MOTOR DIREITO

      M_Speed_Esq = rcCH1;
      M_Speed_Dir = rcCH1;

    }
    else if ((rcCH1 <= zona_morta) && (rcCH1 >= -zona_morta)) {
      Serial.println("ROBO PARADO");

      M_Speed_Esq = 0;
      M_Speed_Dir = 0;

    }
    else if (rcCH1 < -zona_morta) {
      Serial.println("ROBO PRA ESQUERDA");

      M_Dir_Esq = 0; // 1 == FRENTE // 0 == TRAS // MOTOR ESQUERDO
      M_Dir_Dir = 1; // 1 == FRENTE // 0 == TRAS // MOTOR DIREITO

      M_Speed_Esq = abs(rcCH1);
      M_Speed_Dir = abs(rcCH1);

    }

  }
  
 // ##########ROBO EM RÉ#########
 
  else if (rcCH2 < -zona_morta) { // Se o eixo X estiver abaixo do meio é o motor para trás

    

    if (rcCH1 > zona_morta) {
      
      Serial.println("ROBO EM RE PRA DIREITA");

      M_Speed_Esq = abs(rcCH2);
      M_Speed_Dir = abs(rcCH2) - (rcCH1 / 2);


    } else if ((rcCH1 <= zona_morta) && (rcCH1 >= -zona_morta)) {
      
      M_Dir_Esq = 0; // 1 == FRENTE // 0 == TRAS
      M_Dir_Dir = 0; // 1 == FRENTE // 0 == TRAS
      
      Serial.println("ROBO EM RE ");
      M_Speed_Esq = abs(rcCH2);
      M_Speed_Dir = abs(rcCH2);

    } else if (rcCH1 < -zona_morta) {

      Serial.println("ROBO EM RE PRA ESQUERDA");
      M_Speed_Esq = abs(rcCH2) - (abs(rcCH1) / 2);
      M_Speed_Dir = abs(rcCH2);

    }
  }
 

  M_Speed_Dir = constrain(M_Speed_Dir, 0, 255);
  M_Speed_Esq = constrain(M_Speed_Esq, 0, 255);
  
  mControl_Dir(M_Speed_Dir, M_Dir_Dir);
  mControl_Esq(M_Speed_Esq, M_Dir_Esq);
  
  Serial.println(" ");
  Serial.print(" Dire_ESQ: ");
  Serial.print(M_Dir_Esq);
  Serial.print(" Speed_ESQ: ");
  Serial.print(M_Speed_Esq);
  Serial.print(" Dire_DIR: ");
  Serial.print(M_Dir_Dir);
  Serial.print(" Speed_Dir: ");
  Serial.println(M_Speed_Dir);
  Serial.println(" ");

}
