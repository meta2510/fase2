int M = 3; //SALIDA DEL MOTOR
int POR;  // PORCENTAJE ACUTAL DEL PWM 
int IN1 = 7;
int IN2 = 8;

// DECLARACION DE VARIABLES PARA PINES
const int pinecho = 10;
const int pintrigger = 11;
const int pinled = 13;
 
// VARIABLES PARA CALCULOS
unsigned int tiempo, distancia;
 
void setup() {
  // PREPARAR LA COMUNICACION SERIAL
  Serial.begin(9600);
  // CONFIGURAR PINES DE ENTRADA Y SALIDA
  pinMode(M, OUTPUT);
  pinMode(pinecho, INPUT);
  pinMode(pintrigger, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
}
 
void loop() {
  // ENVIAR PULSO DE DISPARO EN EL PIN "TRIGGER"
  digitalWrite(pintrigger, LOW);
  delayMicroseconds(2);
  digitalWrite(pintrigger, HIGH);
  // EL PULSO DURA AL MENOS 10 uS EN ESTADO ALTO
  delayMicroseconds(10);
  digitalWrite(pintrigger, LOW);
 
  // MEDIR EL TIEMPO EN ESTADO ALTO DEL PIN "ECHO" EL PULSO ES PROPORCIONAL A LA DISTANCIA MEDIDA
  tiempo = pulseIn(pinecho, HIGH);
 
  // LA VELOCIDAD DEL SONIDO ES DE 340 M/S O 29 MICROSEGUNDOS POR CENTIMETRO
  // DIVIDIMOS EL TIEMPO DEL PULSO ENTRE 58, TIEMPO QUE TARDA RECORRER IDA Y VUELTA UN CENTIMETRO LA ONDA SONORA
  distancia = tiempo / 58;
 
  // ENVIAR EL RESULTADO AL MONITOR SERIAL
  Serial.print("Distancia: ");
  Serial.print(distancia);
  Serial.println(" cm");
 // delay(200);
 
  // ENCENDER EL LED CUANDO SE CUMPLA CON CIERTA DISTANCIA
  if (distancia > 50 && distancia<101) { // Ciclo al 75% y se enciende un led
    digitalWrite(30, HIGH);
    digitalWrite(31, LOW);
    digitalWrite(32, LOW);
    analogWrite (M, 191);
    Serial.print("Ciclo de trabajo: 75%");
  } 
  else if(distancia<51 && distancia>25){ //Ciclo al 50% y se enciende el buzzer
    digitalWrite(30, LOW);
    digitalWrite(31, HIGH);
    digitalWrite(32, LOW);
    analogWrite (M, 127);
    Serial.print("Ciclo de trabajo: 50%");
    }
  else if(distancia<26 && distancia>4){ // Ciclo al 0%
    analogWrite (M, 0);
    digitalWrite(30, LOW);
    digitalWrite(31, LOW);
    digitalWrite(32, LOW);
    Serial.print("Ciclo de trabajo: 0%");
    }  
  else {
    digitalWrite(30, LOW);
    digitalWrite(31, LOW);
    digitalWrite(32, LOW);
  }
}
