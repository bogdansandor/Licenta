#include <Servo.h>
#include <QTRSensors.h>



/////////////////////////////////
//Ultrasonic
/////////////////////////////////

//Pini ultrasonic fata
int const trigPinFata = 3;
int const echoPinFata = 2;

//Pini ultrasonic spate
int const trigPinSpate = 8;
int const echoPinSpate = 9;

//Pini ultrasonic stanga
int const trigPinStanga = 4;
int const echoPinStanga = 5;

//Pini ultrasonic dreapta
int const trigPinDreapta = 7;
int const echoPinDreapta = 6;

/////////////////////////////////
//Alarma
/////////////////////////////////
//int const buzzPin = 2;

/////////////////////////////////
//Motoare
/////////////////////////////////

// Motor Dreapta
int enA = 9;
int in1Dreapta = 8;
int in2Dreapta = 7;

// Motor Stanga
int enB = 14;
int in3Stanga = 16;
int in4Stanga = 15;

//Motor servo
Servo servo;
int pinServo = 17;


/////////////////////////////////
//Scanare linie
/////////////////////////////////

#define NUM_SENSORS   5     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN 9

QTRSensorsRC qtrrc((unsigned char[]) {9, 10, 11, 12, 13}, NUM_SENSORS, TIMEOUT, EMITTER_PIN);

unsigned int sensorValues[5];





/////////////////////////////////
//Setup
/////////////////////////////////
void setup()
{
  Serial.begin(9600);
  
  //Setare motoare spate
 
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1Dreapta, OUTPUT);
  pinMode(in2Dreapta, OUTPUT);
  pinMode(in3Stanga, OUTPUT);
  pinMode(in4Stanga, OUTPUT);

  //Setare servo
  servo.attach(pinServo);


  //Ultrasonic fata
  pinMode(trigPinFata, OUTPUT); // trig pin will have pulses output
  pinMode(echoPinFata, INPUT); // echo pin should be input to get pulse width

  //Ultrasonic spate
  pinMode(trigPinSpate, OUTPUT); // trig pin will have pulses output
  pinMode(echoPinSpate, INPUT); // echo pin should be input to get pulse width

 
  //Ultrasonic dreapta
  pinMode(trigPinDreapta, OUTPUT); // trig pin will have pulses output
  pinMode(echoPinDreapta, INPUT); // echo pin should be input to get pulse width

   //Ultrasonic stanga
   //pinMode(trigPinStanga, OUTPUT); // trig pin will have pulses output
   //pinMode(echoPinStanga, INPUT); // echo pin should be input to get pulse width
    
  //Alarma
  //pinMode(buzzPin, OUTPUT);


  //Calibrate line
  
  delay(500);
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtrrc.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }

  // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
  
}



/////////////////////////////////
//Scenariu
/////////////////////////////////
void scenariuUnu()
{
    //Scenariu unu
    setareUnghiServo(0);
    delay(1000);
    
    setareUnghiServo(20);
    miscaPunteSpateTimp(1, 1000, 230);
    delay(1000);
    setareUnghiServo(-20);
    miscaPunteSpateTimp(-1, 1000, 150);
}

void ruleazaScenariu()
{
  //setareUnghiServo(0);
}

/////////////////////////////////
//Solutie
/////////////////////////////////

int directieDeplasare = 1; //1-fata, -1 spate
int distantaMinimaDeplasare = 30;

int viteza = 150;
int timp = 100;
float distantaParcursaDreapta = 0;
float distantaParcursaStanga = 0;

float distantaMinimaParcare = 12;
int distantaMinimaSenzoriLaterali = 20;

bool locParcareGasit = false;
bool masinaParcata = false;
int tipParcare = 0; //0-fara loc de parcare, 1-parcare dreapta, 2-parcare stanga

void ruleazaSolutieParcare()
{
    
    if (!locParcareGasit)
    {

      delay(50);
      int distantaFata = citireDistantaUltrasonic(trigPinFata, echoPinFata, "fata");
      delay(50);
      int distantaSpate = citireDistantaUltrasonic(trigPinSpate, echoPinSpate, "spate");

      bool seDeplaseaza = false;
      if (distantaFata >= distantaMinimaDeplasare)
        {
          seDeplaseaza = true;
          miscaPunteSpateTimp(1, timp, viteza);

          if (directieDeplasare == 1)
          {
            Serial.println("SCHIMBAM DIRECTIA");
            distantaParcursaStanga = 0;
            distantaParcursaDreapta = 0;
          }

          directieDeplasare = -1;
        }
      else if(distantaSpate >= distantaMinimaDeplasare)
        {
          seDeplaseaza = true;
          miscaPunteSpateTimp(-1, timp, viteza);

          if (directieDeplasare == -1)
          {
            Serial.println("SCHIMBAM DIRECTIA");
            distantaParcursaStanga = 0;
            distantaParcursaDreapta = 0;
          }

          directieDeplasare = 1;
        }
        
      delay(100);

      //Reseteaza distanta parcursa daca nu se deplaseaza
      if (!seDeplaseaza)
       {
        distantaParcursaDreapta = 0;
        distantaParcursaStanga = 0;
       }

      //Ultrasonic dreapta
      int distantaDreapta = citireDistantaUltrasonic(trigPinDreapta, echoPinDreapta, "dreapta");
      if (distantaDreapta < distantaMinimaSenzoriLaterali)
      {
        Serial.println("OBSTACOL GASIT ! RESETEZ DISTANTA");
        distantaParcursaDreapta = 0;
      }
      else if (seDeplaseaza)
      {
        float dist = (float)timp / 100 * (float)viteza / 100;
        distantaParcursaDreapta += dist;
      }

      Serial.println("Distanta libera :" + String(distantaParcursaDreapta) + " din " + String(distantaMinimaParcare));
      if(distantaParcursaDreapta >= distantaMinimaParcare)
      {
        Serial.println("Loc parcare gasit !");
        locParcareGasit = true;
        tipParcare = 1;
      }
    

      //Ultrasonic stanga
      int distantaStanga = citireDistantaUltrasonic(trigPinStanga, echoPinStanga, "stanga");
      distantaStanga = 0;
      if (distantaStanga < distantaMinimaSenzoriLaterali)
      {
        Serial.println("OBSTACOL GASIT ! RESETEZ DISTANTA");
        distantaParcursaStanga = 0;
      }
      else if (seDeplaseaza)
      {
        float dist = (float)timp / 100 * (float)viteza / 100;
        distantaParcursaStanga += dist;
      }

      Serial.println("Distanta libera :" + String(distantaParcursaStanga) + " din " + String(distantaMinimaParcare));
      if(distantaParcursaStanga >= distantaMinimaParcare)
      {
        Serial.println("Loc parcare gasit !");
        locParcareGasit = true;
        tipParcare = 2;
      }
    }
    

  /*
   *  setareUnghiServo(30);
      delay(2000);
      miscaPunteSpateTimp(1, 1000, 150);
      setareUnghiServo(-30);
      delay(100);
      miscaPunteSpateTimp(1, 1000, 150);
      setareUnghiServo(35);
      delay(100);
      miscaPunteSpateTimp(-1, 600, 150);

      setareUnghiServo(0);
      miscaPunteSpateTimp(-1, 500, 150);
   */
  
  if(locParcareGasit && !masinaParcata)
  {
    setareUnghiServo(0);
    delay(500);
 
    if (tipParcare == 1) //parcare gasita pe dreapta
    {
      setareUnghiServo(30);
      delay(2000);
      miscaPunteSpateTimp(1, 1000, 150);
      setareUnghiServo(-30);
      delay(100);
      miscaPunteSpateTimp(1, 1000, 150);
      setareUnghiServo(35);
      delay(100);
      miscaPunteSpateTimp(-1, 600, 150);

      setareUnghiServo(0);
      miscaPunteSpateTimp(-1, 500, 150);
    }
    else
    if (tipParcare == 2) //parcare gasita pe stanga
    {
      setareUnghiServo(-30);
      delay(2000);
      miscaPunteSpateTimp(1, 1000, 150);
      setareUnghiServo(30);
      delay(100);
      miscaPunteSpateTimp(1, 1000, 150);
      setareUnghiServo(-35);
      delay(100);
      miscaPunteSpateTimp(-1, 600, 150);

      setareUnghiServo(0);
      miscaPunteSpateTimp(-1, 500, 150);
    }

    masinaParcata = true;
  }
     
}



int valoareLinieNedetectata = 2500;
int ultimulSenzorActiv = 2; // luat de la 0 la 4
/* void ruleazaSolutieUrmarireBanda()
{
  qtrrc.read(sensorValues); 

  bool avemCitire = false;
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(String(sensorValues[i]) + " ");

    if (sensorValues[i] != valoareLinieNedetectata)
      avemCitire = true;
  }
  Serial.println("");

  if (!avemCitire)
    sensorValues[ultimulSenzorActiv] = 4; //marcam activa ultima citire
  
  int unghiServo = 0;

  //Senzor mijloc
  if (sensorValues[2] != valoareLinieNedetectata)
  {
    ultimulSenzorActiv = 2;
  }

  //Verificam cei mai apropiati 2 senzori de mijloc
  if (sensorValues[1] != valoareLinieNedetectata)
  {
    unghiServo = -25;
    ultimulSenzorActiv = 1;
  }
  if (sensorValues[3] != valoareLinieNedetectata)
  {
    unghiServo = 25;
    ultimulSenzorActiv = 3;
  }

  //Verificam cei mai departati 2 senzori de mijloc
  if (sensorValues[0] != valoareLinieNedetectata)
  {
    unghiServo = -45;
    ultimulSenzorActiv = 0;
  }
  if (sensorValues[4] != valoareLinieNedetectata)
  {
    unghiServo = 45;
    ultimulSenzorActiv = 4;
  }
  
  setareUnghiServo(unghiServo);
  delay(100);
  miscaPunteSpateTimp(-1, 150, 150);
}
/*/ 

/////////////////////////////////
//Citire ultrasunete
/////////////////////////////////
int citireDistantaUltrasonic(int trigPin, int echoPin, String mesaj)
{
  digitalWrite(trigPin, HIGH); 
  delay(1);
  digitalWrite(trigPin, LOW);
  
  int duration = pulseIn(echoPin, HIGH);
  int distance = (duration/2) / 29.1;

  Serial.println("Citire ultrasonic " + mesaj + " " + String(distance) + " cm");
  return distance;
}

/////////////////////////////////
//Loop
/////////////////////////////////
bool executeOnce = false;
void loop()
{
  if (!executeOnce)
  {
    executeOnce = true;
    ruleazaScenariu();

    
    //Calibrare servo
    
   setareUnghiServo(-20);
    delay(2000);
    setareUnghiServo(20);
    delay(2000);
    setareUnghiServo(0);
    delay(10000);
    
  }

  ruleazaSolutieParcare();
  //ruleazaSolutieUrmarireBanda();
  
 
}

/////////////////////////////////
//Functii ajutatoare
/////////////////////////////////

//directie  1: fata
//         -1: spate
//time  ex: 5000ms 
//speed intre 0-255
void miscaPunteSpateTimp(int directie, int time, int speed)
{
  if (directie == 1)
  {
    // Turn on motor A
    digitalWrite(in1Dreapta, HIGH);
    digitalWrite(in2Dreapta, LOW);
  
    // Turn on motor B
    digitalWrite(in3Stanga, LOW);
    digitalWrite(in4Stanga, HIGH);
  }
  else if (directie == -1)
  {
    // Turn on motor A
    digitalWrite(in1Dreapta, LOW);
    digitalWrite(in2Dreapta, HIGH);
  
    // Turn on motor B
    digitalWrite(in3Stanga, HIGH);
    digitalWrite(in4Stanga, LOW);
  }

  //Set speed for both motors
  analogWrite(enA, speed);
  analogWrite(enB, speed);

  delay(time);

  //Turn off motors
  digitalWrite(in1Dreapta, LOW);
  digitalWrite(in2Dreapta, LOW);  
  digitalWrite(in3Stanga, LOW);
  digitalWrite(in4Stanga, LOW);
}

void miscaPunteSpate(int directie, int speed)
{
  if (directie == 1)
  {
    // Turn on motor A
    digitalWrite(in1Dreapta, HIGH);
    digitalWrite(in2Dreapta, LOW);
  
    // Turn on motor B
    digitalWrite(in3Stanga, LOW);
    digitalWrite(in4Stanga, HIGH);
  }
  else if (directie == -1)
  {
    // Turn on motor A
    digitalWrite(in1Dreapta, LOW);
    digitalWrite(in2Dreapta, HIGH);
  
    // Turn on motor B
    digitalWrite(in3Stanga, HIGH);
    digitalWrite(in4Stanga, LOW);
  }

  //Set speed for both motors
  analogWrite(enA, speed);
  analogWrite(enB, speed);
}

void stopPunteSpate()
{
  //Turn off motors
  digitalWrite(in1Dreapta, LOW);
  digitalWrite(in2Dreapta, LOW);  
  digitalWrite(in3Stanga, LOW);
  digitalWrite(in4Stanga, LOW);
}

void setareUnghiServo(int angle)
{
  int defaultServoAngle = 80;
  servo.write(defaultServoAngle + angle);
}