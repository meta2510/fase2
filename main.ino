int N = 20;                                              // nùmero de ranuras del encoder
float diametro = 6.8;                                    // diametro de la llanta cm
int contadorTicks = 3;                                  // nùmero de ticks para calculo de velocidad
int tam = 10;                                       // tamaño del vector del calculo de promedio, se debe descomentar la linea que se vaya a usar
int IN1 = 52;
int IN2 = 53;
float errorP = 0;    
float errorI = 0;      
float errorD = 0; 
float errorL = 0;                                  // error variables
float Kp = 0.5;                                          // Contante proporcional control
int PWMr = 0;                                           // PWM de la llanta derecha (señal de control llanta derecha)
int PWMl = 0;                                           // PWM de la llanta izquierda (señal de control llanta izquierda)
double Ki = 0.00002;
double Kd = 0.1;
int PWMmax=255;                                          // PWM màximo 
int PWMmin=0;                                           // PWM mìnimo
float Ve=1;
int MotorL=10;
int MotorR=11;
int T;
volatile unsigned muestreoActual = 0;                     // variables para definiciòn del tiempo de muestreo
volatile unsigned muestreoAnterior = 0;
volatile unsigned deltaMuestreo = 0;
volatile unsigned CT = 0;
volatile unsigned ET = 0;
volatile unsigned PT = 0;
int k = 10; 
///------------------------------- Variables del robot  ---------------------------------------------

float longitud = 13.4;                                   // longitud del robot entre llantas
float V = 0;                                             // Velocidad lineal del carro
float PID = 0;                                             // Velocidad Angular del carro

///------------------------------- Variables de motor derecho---------------------------------------------


volatile unsigned muestreoActualInterrupcionR = 0;        // variables para definiciòn del tiempo de interrupciòn y calculo de la velocidad motor derecho
volatile unsigned muestreoAnteriorInterrupcionR = 0;
double deltaMuestreoInterrupcionR = 0;

int encoderR = 3;   // pin de conexiòn del encoder derecho
int llantaR = 900;      // pin de conexiòn de llanta derecha   (pin de PWM)

double frecuenciaR = 0;                                  // frecuencia de interrupciòn llanta R
double Wr = 0;                                           // Velocidad angular R
double Vr = 0;                                           // velocidad Lineal
int CR = 0;                                             // contador ticks
float vectorR[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};    // vector de almacenamiento de datos para promedio del tiempo de interrupciones

float Rdistancia = 0;                                    // distancia recorrida llanta derecha
int Rtick = 0;                                           // ticks del encoder derecho
int RtickAnt = 0;                                        // ticks del encoder derecho anteriores
int deltaRtick = 0;                                      // diferencia del encoder derecho

//------------------------------  Variables de motor Izquierdo ------------------------------------------------

volatile unsigned muestreoActualInterrupcionL = 0;        // variables para definiciòn del tiempo de interrupciòn y calculo de la velocidad motor Izquierdo
volatile unsigned muestreoAnteriorInterrupcionL = 0;
double deltaMuestreoInterrupcionL = 0;

int encoderL = 2;   // pin de conexiòn del encoder Izquierdo
int llantaL = 900;      // pin de conexiòn de llanta Izquierda   (pin de PWM)

double frecuenciaL = 0;                                  // frecuencia de interrupciòn llanta Izquierda
double Wl = 0;                                           // Velocidad angular L
double Vl = 0;                                           // velocidad Lineal
int CL = 0;                                              // contador Ticks
float vectorL[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};    // vector de almacenamiento de datos para promedio del tiempo de interrupciones

float Ldistancia = 0;                                    // distancia recorrida llanta izquierda
int Ltick = 0;                                           // ticks del encoder izquierdo
int LtickAnt = 0;                                        // ticks del encoder izquier anteriores
int deltaLtick = 0;                                      // diferencia del encoder izquierdo


 
//-------------------------------- Variables del ultrasonico -------------------------------------------------
const int pinecho = 30;
const int pintrigger = 31;
unsigned int tiempo, distancia; //32, 33, 34
const int ledRojo = 32;
const int buzzer = 33;
const int ledVerde = 34;
//------------------------------- Variables del bluetooth --------------------------------------------------
#include <SoftwareSerial.h>   // Incluimos la librería  SoftwareSerial  
SoftwareSerial BT(0,1);    // Definimos los pines RX y TX del Arduino conectados al Bluetooth
const int pinled = 13;
char doi;

void setup() {
  BT.begin(9600);       // Inicializamos el puerto serie BT que hemos creado
  attachInterrupt(digitalPinToInterrupt(encoderR),REncoder,FALLING);                // linea para añadir una interrupciòn a un PIN
  attachInterrupt(digitalPinToInterrupt(encoderL),LEncoder,FALLING);                // linea para añadir una interrupciòn a un PIN
  Serial.begin(9600);                                                               // inicio de la comunicaciòn serial
 // pinMode(llantaR, OUTPUT);
 // pinMode(llantaL, OUTPUT);
 // pinMode(VeL, OUTPUT);
 // pinMode(VeR, OUTPUT);
  pinMode(pinled, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(pinecho, INPUT);
  pinMode(pintrigger, OUTPUT);
  pinMode(ledRojo, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(ledVerde, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(encoderR),REncoder,FALLING);                // linea para añadir una interrupciòn a un PIN
  attachInterrupt(digitalPinToInterrupt(encoderL),LEncoder,FALLING);                // linea para añadir una interrupciòn a un PIN
  Serial.begin(9600);
  pinMode(MotorR, OUTPUT);
  pinMode(MotorL, OUTPUT);
}

void REncoder() {                                                                                    // funciòn de interrupciòn del enconder llanta derecha
      Rtick++;  
      CR++;

      if (CR == contadorTicks){
          float media = 0;
          deltaMuestreoInterrupcionR = muestreoActualInterrupcionR -  muestreoAnteriorInterrupcionR;     // diferencia tiempos de interruciones de ticks del motor     
          
           for(int i=0;i < tam-1;i++){                                                                    // relleno del vector para calculo posterior del promedio
              vectorR[i]=vectorR[i+1];
            }
           vectorR[tam-1]=deltaMuestreoInterrupcionR ;                                                     // ùltimo dato del vector 

           for(int i=0;i<tam;i++){                                                                        // Càlculo de la media del vector
              media = vectorR[i]+ media;
            }
            media = media/tam;
            deltaMuestreoInterrupcionR = media;                                                            // se reemplaza por el valor de su medìa. 
           
            frecuenciaR = (1000)/ deltaMuestreoInterrupcionR;                                       // frecuencia de interrupciòn 
          
          muestreoAnteriorInterrupcionR = muestreoActualInterrupcionR;                                   // se actualiza el tiempo de interrupciòn anterior
          CR = 0;                       //Reinicio de contador
      } 
 } 

void LEncoder() {                                                                                       // funciòn de interrupciòn del enconder llanta izquierda
      Ltick++;                                                                                           // Nùmero de ticks llanta izquierda
      CL++;                                                                                             // incremento del contador de ticks
      if (CL == contadorTicks){                                                                         // si el contador de ticks alcanza el valor de ticks   
          float media = 0;                                                                              // variable creada para cálculo del promedio
          deltaMuestreoInterrupcionL = muestreoActualInterrupcionL -  muestreoAnteriorInterrupcionL;     // diferencia tiempos de interruciones de ticks del motor
          for(int i=0;i < tam-1;i++){                                                                    // relleno del vector para calculo posterior del promedio
              vectorL[i]=vectorL[i+1];
          }
          vectorL[tam-1]=deltaMuestreoInterrupcionL;                                                     // último dato del vector (medida actual) 

          for(int i=0;i<tam;i++){                                                                        // Suma de los valores del vector
              media = vectorL[i]+ media;
          }
          media = media/tam;                                                                             //división por el total de datos del vector
          deltaMuestreoInterrupcionL = media;                                                            // se reemplaza por el valor de su medío.       
          frecuenciaL = (1000)/ deltaMuestreoInterrupcionL;                                              // frecuencia de interrupciòn 
          muestreoAnteriorInterrupcionL = muestreoActualInterrupcionL;                                   // se actualiza el tiempo de interrupciòn anterior
          CL = 0;                                                                                        // Reinicio de contador de ticks
       } 
 } 


void ultrasonico() {
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
}

void loop() { 
    muestreoActualInterrupcionR = millis();                     // se asigna el tiempo de ejecuciòn a el muestreo actual
    muestreoActualInterrupcionL = millis();                     // se asigna el tiempo de ejecuciòn a el muestreo actual
    CT = millis();
    ET = CT - PT;

    ultrasonico();

    Serial.print(distancia);                                         // se muestra el tiempo entre TIC y TIC
    Serial.print(" ");                                         // se muestra el tiempo entre TIC y TIC
    Serial.print(llantaL);
        Serial.print(" ");
    Serial.print(llantaR);
        Serial.print(" ");
    Serial.print(PID);
    
       Serial.print(" ");
    Serial.print(frecuenciaR);
        Serial.print(" ");
    Serial.println(frecuenciaL);


     
     //Serial.print("Distancia: ");
     //Serial.print(distancia);
     //Serial.println(" cm");// se muestra el tiempo entre TIC y TIC

     digitalWrite(IN1,HIGH);
     digitalWrite(IN2,HIGH);
    muestreoActual = millis();                                                                           //Tiempo actual de muestreo

        deltaMuestreoInterrupcionR = muestreoActualInterrupcionR -  muestreoAnteriorInterrupcionR;       // diferencia tiempos de interruciones de ticks del motor     
        deltaMuestreoInterrupcionL = muestreoActualInterrupcionL -  muestreoAnteriorInterrupcionL;       // diferencia tiempos de interruciones de ticks del motor     
          analogWrite(MotorL,llantaL);
          analogWrite(MotorR,llantaR);
        if(frecuenciaR < 7 || frecuenciaL < 7){                                              // Esta es la forma de definir cuando el motor se encuentra quieto. Si deltaMuestreoInterrupcionR es mayor a 40 milisegundos por el preescalado de ticks 
          llantaR = 0;  
          V=0; 

          
    llantaL = 0; 
    Ve = 0;   // 40 mS es el tiempo que màximo se tarda un tick a la menor velocidad del motor
    if(BT.available()){
          doi = (char)BT.read();
          if(doi == 'S'){
              Serial.println(distancia);
              delay(20);
          }
  }
        }else {
        if (distancia > 50 && distancia<101) { // Ciclo al 75% y se enciende un led
    digitalWrite(ledRojo, HIGH);
    digitalWrite(buzzer, LOW);
    digitalWrite(ledVerde, LOW);
    analogWrite(MotorL,llantaR);                                     // PWM de la llanta derecha
    analogWrite(MotorR,llantaL);                                     // PWM de la llanta izquierda
    Ve=1;
  } 
  else if(distancia<51 && distancia>25){ //Ciclo al 50% y se enciende el buzzer
    digitalWrite(ledRojo, LOW);
    digitalWrite(buzzer, HIGH);
    digitalWrite(ledVerde, LOW);
    analogWrite(MotorR,llantaR);                                     // PWM de la llanta derecha
    analogWrite(MotorL,llantaL);                                     // PWM de la llanta izquierda
    Ve=0.75;
    }
  else if(distancia<26 && distancia>4){ // Ciclo al 0%
    digitalWrite(ledRojo, LOW);
    digitalWrite(buzzer, LOW);
    digitalWrite(ledVerde, HIGH);
    //analogWrite(llantaR,0);                                     // PWM de la llanta derecha
    //analogWrite(llantaL,0);                                     // PWM de la llanta izquierda
    analogWrite(MotorL, 0);
    analogWrite(MotorR, 0);
    Ve=0;
    }  
  else {
    digitalWrite(30, LOW);
    digitalWrite(31, LOW);
    digitalWrite(32, LOW);
    analogWrite(MotorR,llantaR);                                     // PWM de la llanta derecha
    analogWrite(MotorL,llantaL);                                     // PWM de la llanta izquierda
    Ve=1;
  }


    Wr = contadorTicks*((2*3.141516)/N)*frecuenciaR;                                                     // frecuencia angular Rad/s
    Vr= Wr*(diametro/2)*Ve;                                                                                 // velocidad lineal cm/s
    Wl = contadorTicks*((2*3.141516)/N)*frecuenciaL;                                                     // frecuencia angular Rad/s
    Vl= Wl*(diametro/2)*Ve;                                                                                 // velocidad lineal cm/s     

     V=(Vr+Vl)/2;                                                                                        // calculo de la velocidad del robot
     //V=40;                                                                                             // velocidad constante para alcanzar el àngulo
                                                                                    // error angular Angulo deseado menos el angulo del robot

if(Vr < Vl){

    errorP = V - Vr;  

}
else if(Vr > Vl){

   errorP = V - Vl;
  
} 
     errorI += errorP;
     errorD = (errorP - errorL); 

        PID = errorP * Kp + errorI * Ki*ET + (errorP - errorD) * Kd/ET; 

     errorL = errorP;
                                                                // Càlculo de la velocidad angular con las variables de control
        llantaR = (V + (PID));                                                                       // Señal de control PWM llanta derecha
        llantaL = (V + (PID));                                                                       // Señal de control PWM llanta izquierda

  PT = CT;
 
       if(llantaR > PWMmax){                                                                               
           llantaR = PWMmax;
        }
        if(llantaR < PWMmin){
          llantaR = PWMmin;
        }
        if(llantaL > PWMmax){
           llantaL = PWMmax;
        }
        if(llantaL < PWMmin){
           llantaL = PWMmin;
        }

  if(BT.available()){
          doi = (char)BT.read();
          if(doi == 'S'){
              Serial.println(distancia);
              delay(20);
          }
  }
         
}
 }
