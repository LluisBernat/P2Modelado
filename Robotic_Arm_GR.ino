///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                LIBRERÍAS                                                  //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "MeMegaPi.h"
#include <Servo.h>
// Para implementar la cinemática directa:
#include "MatrixMath.h"
#include <Math.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                VARIABLES                                                  //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
MePort limitSwitch(PORT_7);
Servo svs[1] = {Servo()};
MeStepperOnBoard steppers[3] = {MeStepperOnBoard(PORT_1),MeStepperOnBoard(PORT_2),MeStepperOnBoard(PORT_3)}; 

// Vectores para guadrar los límites de las articulaciones que calcularemos
float qlimit_0[2] = {0.0,0.0};
float qlimit_1[2] = {0.0,0.0};
float qlimit_2[2] = {0.0,0.0};

struct Vector3{
  double x;
  double y; 
  double z;
};

// Variable para guardar la posición de Home
Vector3 HomeGrad; // Guarda los valores articulares
  // Inicializamos la variable Home
HomeGrad.x = 0.0;
HomeGrad.y = 0.0;
HomeGrad.z = 0.0;

float lastPositions[3] = {0,0,0}; // Vector para meter el target de la posición
const double RADS = PI / 180.0;   // Pasar de radianes a segundos
const double DEGS = 180.0 / PI;   // Pasar de grados a segundos
const int STEPS = 2;              // Microsteps
const int GEAR_1 = 9;             // Relaciones de transmisión (mecanismos engranajes-eslabón) de la base
const int GEAR_2 = 7;             // Relación de transmisiópn (mecanismos engranajes-eslabón) de los otros dos motores

// Longitudes de los eslabones
const double L1 = 150.0; 
const double L2 = 155.0; 
const double L3 = 200.0;
const double Tz = -45.0;
const double Tx = 65.0;

String buffer = "";
bool endMoving = true;
int sensor1, sensor2;

Vector3 vectorToAngles(float x,float y,float z);
void setSpeed(float speed);

int testSpeed = 150;
int testAcceleration = 300;
int currentSpeed = 400;
int maxSpeed = 500;
int currentAcceleration = 1000;
int pin1, pin2;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  SETUP                                                    //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  //Puerto serie
  Serial.begin(115200);

  //Configuración motores paso a paso
  setSpeedConfiguration(currentSpeed,maxSpeed,currentAcceleration);

  // Configuración steppers
  for(int i=0;i<3;i++){
    steppers[i].setMicroStep(STEPS);  // MicroStep <-- 2
    steppers[i].enableOutputs();      // Habilito las salidas
  }

  //Sensores finales de carrera
  pin1 = limitSwitch.pin1();
  pin2 = limitSwitch.pin2();
  pinMode(pin1,INPUT_PULLUP);
  pinMode(pin2,INPUT_PULLUP);
    // pin1,pin2 = 1 o 0 dependiendo si la salida está activada o no
  
  //Pinza. Configuración servo 
  svs[0].attach(A8);  // Asigno el servo a donde está cableado
    // Inicialmente, se abre y se cierra la pinza:
  open_grip();
  delay(1000);
  close_grip();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                  LOOP                                                     //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  //Lectura del puerto serie y filtrado del comando
  if (Serial.available()) {
     char c = Serial.read();  // Lee lo que hay en el puerto serial
     if(c == '\n') {
      parseBuffer();          // Esta función lo filtra
    }else{
      buffer += c;
    }
  }

  //Visualización finales de carrera (lee los sensores)
  sensor1 = digitalRead(pin1);
  Serial.println(sensor1);
  sensor2 = digitalRead(pin2);
  Serial.println(sensor2);


  //Permito corriente a los motores en un movimiento
  long isMoving = 0;
  for(int i=0;i<3;i++){
      isMoving += abs(steppers[i].distanceToGo());  // Mientras "haya distancia a recorrer"
      steppers[i].run();                            // Sigue moviendo el motor
  }
  if(isMoving>0){             // Aún no ha recorrido la distancia
      endMoving = true;       // Tiene que seguir moviéndose
  }else{
      if(endMoving){          // Ya ha recorrido la distancia
         endMoving = false;   // Deja de moverse
         // Guarda la posición actual:
         steppers[0].setCurrentPosition(lastPositions[0]);
         steppers[1].setCurrentPosition(lastPositions[1]);
         steppers[2].setCurrentPosition(lastPositions[2]);
      }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                FUNCIONES                                                  //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
void parseBuffer(){ // Función que filtra lo que hay por el puerto serie
  buffer =" "+buffer+" ";
  buffer.toLowerCase();

  int count = 0;
  int startIndex = 0;
  int endIndex = 0;
  int len = buffer.length();
  if (len < 1) {
    return;
  }
  String tmp;
  String values[6];

  bool openEnable = false;
  bool closeEnable = false;
  
  //Filtrado del mensaje por el puerto serie
  while(true){
    startIndex = buffer.indexOf(" ", endIndex);
    endIndex = buffer.indexOf(" ", startIndex + 1);
    tmp = buffer.substring(startIndex + 1, endIndex);
   
    if(tmp.indexOf("q1",0)>-1){
      values[0] = tmp.substring(2, tmp.length());
      Serial.println(values[0]);
      move_q1(stringToFloat(values[0]));
    }else if(tmp.indexOf("open",0)>-1){
       openEnable = true;
    }else if(tmp.indexOf("close",0)>-1){
      closeEnable = true;
    }
    count++;
 
    if (endIndex == len - 1) break;
  }

  //Acciones a realizar tras el filtrado del puerto serie
  if(closeEnable){
     close_grip();
  }else if(openEnable){
     open_grip();
  }

  Serial.println("OK"); // Indica que el filtrado se ha realizado correctamente
  buffer = "";
}

//Establecer velocidad de los motores
void setSpeedConfiguration(float c_speed, float max_speed, float accel){
    for(int i=0,i<3;i++){
        steppers[i].setSpeed(c_speed);
        steppers[i].setMaxSpeed(max_speed);
        steppers[i].setAcceleration(accel);
    }
}


//Apertura y cierre de la pinza
void runServo(int index,int angle){
  svs[index].write(angle);
}
void close_grip(){  // Cierra la pinza
 runServo(0,120);
 delay(100);
}
void open_grip(){   // Abre la pinza
 runServo(0,0);
 delay(100);
}

//Transformación de String a float
float stringToFloat(String s){
  float f = atof(s.c_str());
  return f;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                      ****** FUNCIONES A IMPLEMENTAR POR EL GRUPO DE ALUMNOS ******                        //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// I. Busqueda de límites del robot (en grados) ///////////////////////////////////////////////////////////////
//Límites eje 1 - establecerlo en -90,90
void reset_stepper0(){
  qlimit_0[0] = 90;
  Serial.println(qlimit_0[0]);
  
  qlimit_0[1] = -90;
  Serial.println(qlimit_0[1]);
  
  steppers[0].setCurrentPosition(0.0);  // Establece la posición actual como el 0 (origen)
}

//Límites eje 2 - ejemplo
void reset_stepper1(){
  int count_steps1 = 0;
  int count_steps2 = 0;
  steppers[1].setSpeed(testSpeed);
  bool exit1=true;
  bool exit2=true;
  
  while(exit1){
    if(digitalRead(pin1)==LOW){
      steppers[1].step();
      delay(10);
      count_steps1++;
    }else{
      qlimit_1[0] = count_steps1*1.8/(GEAR_2*STEPS);
      Serial.println(qlimit_1[0]);
      exit1=false;
    }
  }
  
  steppers[1].setSpeed(-testSpeed);
  delay(2000);
  
  while(exit2){
    if(digitalRead(pin1)==LOW){
      steppers[1].step();
      delay(10);
      count_steps2++;
    }else{
      qlimit_1[1] = -(count_steps2-count_steps1)*1.8/(GEAR_2*STEPS);
      Serial.println(qlimit_1[1]);
      exit2=false;
    }
  }

  steppers[1].setCurrentPosition(-(count_steps2-count_steps1));
  delay(1000);
  move_q2(0.0);
  delay(1000);
}

//Límites eje 3
void reset_stepper2(){
    // q3 depende de q2 --> los límites de ambas deben coincidir ??? Esto no lo acabo de entender
}

// II. Movimiento en q1,q2,q3 (mueven los ejes del robot) /////////////////////////////////////////////////////
void move_q1(float q1){ // q1 se introduce en grados
  // Comprobar q1 está en los límites establecidos
  if( q1 < qlimit_0[0] && q1 > qlimit_0[1] ){
    // Pasar q1 de Grados a Pasos
    float p1 = Step2Grad(q1);
    // Uso de la función steppers[n].moveTo(steps) para mover el eje
    steppers[0].moveTo(steppers[0].currentPosition() + p1);
    // Actualizar currentPosition con los pasos calculados
    steppers[0].setCurrentPosition(steppers[0].currentPosition() + p1);
      // Otra opcición: steppers[0].setCurrentPosition(steppers[0].targetPosition());
  }else{
    Serial.println("q1 fuera lo los límites del robot");
  }
}

void move_q2(float q2){ // q2 se introduce en grados
  // Comprobar q2 está en los límites establecidos
  if( q2 < qlimit_1[0] && q2 > qlimit_1[1] ){
    // Pasar q2 de Grados a Pasos
    float p2 = Step2Grad(q2);
    // Uso de la función steppers[n].moveTo(steps) para mover el eje
    steppers[1].moveTo(steppers[1].currentPosition() + p2);
    // Actualizar currentPosition con los pasos calculados
    steppers[1].setCurrentPosition(steppers[1].currentPosition() + p2);
      // Otra opcición: steppers[1].setCurrentPosition(steppers[1].targetPosition());
  }else{
    Serial.println("q2 fuera lo los límites del robot");
  }
}

void move_q3(float q3){ // q3 se introduce en grados
  // Comprobar q1 está en los límites establecidos
  // Pasar q1 de Grados a Pasos
  // Uso de la función steppers[n].moveTo(steps) para mover el eje
  // Actualizar el vector lastPositions con los pasos calculados
}

// III. Movimiente de q1,q2 y q3 (mueve todos los ejes del robot) /////////////////////////////////////////////
void moveToAngles(float q1, float q2, float q3){ // Los valores de q se introducen en grados
  if(q1 < qlimit_0[0] && q1 > qlimit_0[1] && q2 < qlimit_1[0] && q2 > qlimit_1[1] && q3 < qlimit_2[0] && q3 > qlimit_2[1]){
    move_q1(q1);
    move_q2(q2);
    move_q3(q3);
  }else{
    Serial.println("Los valores articualres no están dentro de los límites del robot");
  }
  
}

// IV. Punto inicial y vuelta a la posición de home ///////////////////////////////////////////////////////////
// Establece/guarda la posición actual como Home (Home se entiende como la posición origen)
void setHome(){
  HomeGrad.x = steppers[0].currentPosition();
  HomeGrad.y = steppers[1].currentPosition();
  HomeGrad.z = steppers[2].currentPosition();
}

// Ir a la posición Home
void goHome(){    
  moveToAngles(HomeGrad.x,HomeGrad.y,HomeGrad,z);
}

// V. Cinemática directa. Movimiento en q1,q2,q3 (mueven los ejes del robot) //////////////////////////////////
// Función que devuelve la matriz de transformación T entre Si-1 y Si
void denavit(float q, float d, float a, float alfa){
  
}

// Función que utiliza la función denavit, para calcular 0T3 (base-extremo)
Vector3 forwardKinematics (float q1, float q2, float q3){
  
}

// VI. Cinemática inversa. Movimiento en x,y,z ////////////////////////////////////////////////////////////////
void moveToPoint(float x,float y,float z){
 
}

Vector3 inverseKinematics(float x,float y,float z){
 
}

// Trayectoria y tarea p&p ////////////////////////////////////////////////////////////////////////////////////
void trajectory (float q1, float q2, float q3, float t){
  
}

void pick_and_place (){

}

// Otras funciones ////////////////////////////////////////////////////////////////////////////////////////////
// Función para pasar de grados a pasos
float Step2Grad(float Grad){
  float Step = Grad;
  return Step;
}
// Función para pasar de pasos a grados
float Grad2Step(float Step){
  float Grad = Step;
  return Grad;
}
