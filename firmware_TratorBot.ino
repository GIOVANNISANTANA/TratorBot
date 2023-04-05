// Include iBus Library
#include <IBusBM.h>
#define zona_morta 4
#define soft 5
#define soft_nav 2

const int pinEntrada = 21; // Define o pino de entrada dos pulsos ocm interrupção
volatile unsigned int contPulsos = 0; // Variável que conta os pulsos
unsigned long tempoAnterior = 0; // Variável que armazena o tempo anterior
unsigned int rpm = 0; // Variável que armazena as RPM
long int pulsos = 0;

// Create iBus Object
IBusBM ibus;

// Channel Values

int rcCH1 = 0;   // Joystick Eixo X, esquerda e direita
int rcCH2 = 0;   // Joystick Eixo Y, pra frente e pra tras
int rcCH3 = 0;   // Utilizado para ligar e desligar
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

void contador(){
  pulsos++;     //incrementa contador do PID
}

void mControl_Esq(int mspeed, int mdir) {
  switch (mdir) {
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
  switch (mdir) {
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

  attachInterrupt(digitalPinToInterrupt(pinEntrada), contarPulso, RISING); // Configura a interrupção


}

void loop() {
  att_canais();
  //Mostrar();
  controless();
  rotacoes();
}

void att_canais() {
  rcCH1 = readChannel(0, -255, 255, 0);
  rcCH2 = readChannel(1, -255, 255, 0);
  rcCH3 = readChannel(2, 0, 100, 0);
  rcCH4 = readChannel(3, 0, 100, 0);
  rcCH5 = readChannel(4, 0, 255, 0);
  rcCH6 = readChannel(5, 0, 255, 0);
}

void pulverizar() {
  if (rcCH5 > 10 && state == 1) {
    //Serial.println("Pulverizador acionado");
    controlSpray = rcCH5;
  } else {
      //Serial.println("Pulverizador desligado");
      controlSpray = 0;
  }
}

void controless() {
  static byte estado = 0;
  //  estado = 0 -> robo parado
  //  estado = 1 -> robo para frente
  //  estado = 2 -> robo para tras
  //  estado = 3 -> robo girando
  state = 0;
  switch (estado) {
    case 0:
      if (rcCH2 > zona_morta) {
        estado = 1;
      } else if (rcCH2 < -zona_morta) {
        estado = 2;
      } else if (rcCH1 > zona_morta || rcCH1 < -zona_morta) {
        estado = 3;
      } else {
        M_Dir_Dir = 2;    //desliga os PWM
        M_Dir_Esq = 2;    //
        M_Speed_Esq = 0;  //e desabilita os motores
        M_Speed_Dir = 0;  //
      }
      break;
    case 1:
      if (rcCH1 <= zona_morta && rcCH1 >= -zona_morta && rcCH2 <= zona_morta && rcCH2 >= -zona_morta) {
        estado = 0;
      } else if (rcCH2 > zona_morta) {
        M_Dir_Esq = 1;
        M_Dir_Dir = 1;
        M_Speed_Esq = abs(rcCH2) / soft_nav;
        M_Speed_Dir = abs(rcCH2) / soft_nav;
        //state = 1;
        pulverizar();

        if (rcCH1 > zona_morta) {
          M_Speed_Esq = abs(rcCH2) / soft_nav;
          M_Speed_Dir = (abs(rcCH2) - rcCH1) / soft_nav;
        } else if (rcCH1 < -zona_morta) {
            M_Speed_Esq = (abs(rcCH2) - abs(rcCH1)) / soft_nav;
            M_Speed_Dir = abs(rcCH2) / soft_nav;
        }
      }
      break;

    case 2:
      if (rcCH1 <= zona_morta && rcCH1 >= -zona_morta && rcCH2 <= zona_morta && rcCH2 >= -zona_morta) {
        estado = 0;
      } else if (rcCH2 < zona_morta) {
        M_Dir_Esq = 0;
        M_Dir_Dir = 0;
        M_Speed_Esq = abs(rcCH2) / soft_nav;
        M_Speed_Dir = abs(rcCH2) / soft_nav;
        if (rcCH1 > zona_morta) {
          M_Speed_Esq = abs(rcCH2) / soft_nav;
          M_Speed_Dir = (abs(rcCH2) - rcCH1) / soft_nav;
        } else if (rcCH1 < -zona_morta) {
            M_Speed_Esq = (abs(rcCH2) - abs(rcCH1)) / soft_nav;
            M_Speed_Dir = abs(rcCH2) / soft_nav;
        }
      }
      break;

    case 3:
      if (rcCH1 <= zona_morta && rcCH1 >= -zona_morta && rcCH2 <= zona_morta && rcCH2 >= -zona_morta) {
        estado = 0;
      } else if (rcCH1 > zona_morta) {
        M_Dir_Esq = 1;
        M_Dir_Dir = 0;
        M_Speed_Esq = abs(rcCH1) / soft;
        M_Speed_Dir = abs(rcCH1) / soft;
      } else if (rcCH1 < -zona_morta) {
        M_Dir_Esq = 0;
        M_Dir_Dir = 1;
        M_Speed_Esq = abs(rcCH1) / soft;
        M_Speed_Dir = abs(rcCH1) / soft;
      }
      break;
  }

  pulverizar();

  M_Speed_Dir = M_Speed_Dir + rcCH3;
  M_Speed_Esq = M_Speed_Esq + rcCH3;

  M_Speed_Dir = constrain(M_Speed_Dir, 0, 255);
  M_Speed_Esq = constrain(M_Speed_Esq, 0, 255);
  controlSpray = constrain(controlSpray, 0, 255);
/*
  Serial.print("Vel_LEFT ");
  Serial.print(M_Speed_Esq);
  Serial.print(" Vel_RIGH: ");
  Serial.println(M_Speed_Dir);
*/
  mControl_Dir(M_Speed_Dir, M_Dir_Dir);
  mControl_Esq(M_Speed_Esq, M_Dir_Esq);
  cSpray(controlSpray);
}

void rotacoes(){
  if (millis() - tempoAnterior >= 100) { // Verifica se passou 1 segundo
    detachInterrupt(digitalPinToInterrupt(pinEntrada)); // Desabilita a interrupção
    rpm = contPulsos * 1 / 100; // Calcula as RPM
    Serial.print("RPM: "); // Imprime a mensagem
    Serial.println(rpm); // Imprime as RPM
    Serial.print("pulsos: ");
    Serial.println(pulsos);
    contPulsos = 0; // Zera a variável de contagem de pulsos
    tempoAnterior = millis(); // Atualiza o tempo anterior
    attachInterrupt(digitalPinToInterrupt(pinEntrada), contarPulso, RISING); // Habilita a interrupção novamente
  }
}

void contarPulso() {
  contPulsos++; // Incrementa a variável de contagem de pulsos
  pulsos++;
}
