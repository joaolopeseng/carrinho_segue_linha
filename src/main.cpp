#include <Arduino.h>

// ==================== DEFINIÇÕES ====================
// const int: significa que é uma constante que não tem como variar, só permite operações matemática 
// int sensores[numSensor]: significa que foi criado um vetor dentro usando uma constante vetor = biblioteca
const int numSensores = 8;
int sensores[numSensores] = {A0, A1, A2, A3, A4, A5, 2, 3 };
const int IR_SENSOR = 9;
// ===== TB6612FNG =====

// Motor Esquerdo (A)
// resumo são os locais que serão ligados os pinos  // mais pra frente fica o comando
const int AIN1 = 7;
const int AIN2 = 8;
const int PWMA = 5;

// Motor Direito (B)
const int BIN1 = 12;
const int BIN2 = 13;
const int PWMB = 6;

// STBY vai dizer se o motor pode ligar
const int STBY = 4;  

// ==================== PID ====================

float Kp = 3; //k1 errro // ele corrige de forma proporcional ao erro
float Ki = 0.; // ki * integral // ele significa erro acumulado  exemplo 1+1+1+1 = 4 a cada vez que roda o programa ele corrige de forma mais agressiva
float Kd = 90; // ki * derivada // ele corrige baseado na mudança do erro, o quanto o erro mudou de um momento para o outro

float erro = 0;
float erroAnterior = 0;
float integral = 0;
float derivada = 0;
float PID = 0;

// _-_-_-_-_-_-_-_-_-_-_-_-_-ATENÇÃO-_-_-_-_-_-_-_-_-_-_-_-_-_
// ==================== PWM MÁXIMO 185 ======================
// _-_-_-_-_-_-_-_-_-_-_-_-_-ATENÇÃO-_-_-_-_-_-_-_-_-_-_-_-_-_

int velocidadeBase = 1;

// ==================== SETUP ====================

void setup() {

  Serial.begin(9600);

  for(int i = 0; i < numSensores; i++){
    pinMode(sensores[i], INPUT);
    pinMode(IR_SENSOR, OUTPUT);
    digitalWrite(IR_SENSOR, HIGH);
  }
  
  pinMode(IR_SENSOR, OUTPUT);
  pinMode(AIN1, OUTPUT); // pinMODE é o comando de entrada, manda sinal do arduino para ponte H
  pinMode(AIN2, OUTPUT); // OUTPUT ele vai enviar energia pro motor 
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  pinMode(STBY, OUTPUT);
                        
  digitalWrite(STBY, HIGH);  // HIGH está falando para colocar 5v na placa // digitalWrite manda sinal para ponte H 
}

// ==================== CALCULAR ERRO ====================

int calcularErro(){

  int pesos[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
  int soma = 0;
  int sensoresAtivos = 0;

  for(int i = 0; i < numSensores; i++){

    int leitura = digitalRead(sensores[i]);
    Serial.print(leitura);
    Serial.print("|");

    // Linha BRANCA         // HIGH siginifica que esta recebendo luz / se eu colocar LOW significa que não esta recebendo luz 
    if(leitura == LOW){
      soma += pesos[i];
      sensoresAtivos++;
    }
  }


  // ===== CRUZAMENTO =====
  if(sensoresAtivos >= 5){
    return 0;   // força seguir reto
  }

  // ===== PERDEU A LINHA =====
  if(sensoresAtivos == 0){
    return erroAnterior;    // caso perder a linha ele segue o ultimo comando, por isso return erroAnterior 
  }

  return soma / sensoresAtivos; // calcula a média para o PID 
}

// ==================== CONTROLE DOS MOTORES ====================

void controlarMotores(int velEsq, int velDir){

  // _-_-_-_-_-_-_-_-_-_-_-_-_-ATENÇÃO-_-_-_-_-_-_-_-_-_-_-_-_-_
  // ==================== PWM MÁXIMO 185 =======================
  // _-_-_-_-_-_-_-_-_-_-_-_-_-ATENÇÃO-_-_-_-_-_-_-_-_-_-_-_-_-_

  velEsq = constrain(velEsq, 80, 120);
  velDir = constrain(velDir, 80, 120);
  
  // Motor Esquerdo
  digitalWrite(AIN1, HIGH);  // HIGH = ligado // LOW = desligado
  digitalWrite(AIN2, LOW);   // digital é o sensor
  analogWrite(PWMA, velEsq); // analógico é o motor 

  // Motor Direito
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, velDir);
}

// ==================== LOOP ====================

void loop(){

  erro = calcularErro();

  integral += erro;
  derivada = erro - erroAnterior;

  PID = (Kp * erro) + (Ki * integral) + (Kd * derivada);

  erroAnterior = erro;

  Serial.print("ERRO=");
  Serial.print(erro);
  Serial.print(" ");
  Serial.print("PID=");
  Serial.println(PID);

  
  int velocidadeEsquerda = velocidadeBase - PID;
  int velocidadeDireita = velocidadeBase + PID;

  controlarMotores(velocidadeEsquerda, velocidadeDireita);
}
