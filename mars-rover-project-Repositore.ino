//-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/-/MARS ROVER PROJECT-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\-\\

//                                                                                             by Ian S. Molina 
//version: beta 0.1
//last update: 11/08/2025 at 02:19 pm


#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SoftPWM.h>




// These constants should match the photoresistor's "gamma" and "rl10" attributes
const float GAMMA = 0.7;
const float RL10 = 50;

//---------------------------------communication-------------------------------------------------------------------------
SoftwareSerial ESP(0, 1); // RX, TX (wifi module)
const int RST = 42; //(wifi module)


TinyGPS gps;
SoftwareSerial ss(19, 20);

//-----------------------------------------------------------------------------------------------------------------

//-------------------motion motors-------------------------------------------------------------------------------

  Servo servoForMotor1 ;
  Servo servoForMotor2 ;
  Servo servoForMotor5 ;
  Servo servoForMotor6 ;

  //declaration of the pins used to control the rotation speed
  const int Motor1_speed = 36;
  const int Motor2_speed = 37;
  const int Motor3_speed = 38;
  const int Motor4_speed = 39;
  const int Motor5_speed = 40;
  const int Motor6_speed = 41;

  //declaration of the pins used to control the motor direction
  const int Motor1_forward = 24;
  const int Motor2_forward = 25;
  const int Motor3_forward = 26;
  const int Motor4_forward = 27;
  const int Motor5_forward = 28;
  const int Motor6_forward = 29;


  const int Motor1_backward = 30;
  const int Motor2_backward = 31;
  const int Motor3_backward = 32;
  const int Motor4_backward = 33;
  const int Motor5_backward = 34;
  const int Motor6_backward = 35;

//---------------------------------------------------------------------------------------------------------------

//------------for the solar panels---------------------------------------------------------------------------------
  Servo servoForSL1 ;
  Servo servoForSL2 ;
  Servo servoForSL3;
#define LDR1 A0
#define LDR2 A1
#define LDR3  A2
//-----------------------------------------------------------------------------------------------------------------

//-------------------------------for the rover's neck and head-----------------------------------------------------
  Servo neckservo ;
  Servo headservo ;
#define PIN_TRIG 52
#define PIN_ECHO 53
//-----------------------------------------------------------------------------------------------------------------

// ----------------------------- Position and Environmental Sensors ------------------------------------------------------------
// Defining position sensors (gyroscope, accelerometer) and environmental sensors (temperature, humidity, rain)

#define DHTPIN 23
#define DHTTYPE DHT22

  DHT dht(DHTPIN, DHTTYPE);
  Adafruit_MPU6050 mpu;

// ------------------------------------------------------------------------------------------------------------------------------

//------------------------for the robotic arm-----------------------------------------------------------------------------------------
  Servo servoarm1 ;
  Servo servoarm2 ;
  Servo servoarm3;
  Servo servoarm4 ;
//-----------------------------------------------------------------------------------------------------------------

void setup() {
  
 
 
  //----------------- Motor's pin configuration as output-----------------------------------------------------------

    enginesConfiguration();

    //Servosresponsible for changing the direction of the wheels
     servoForMotor1.attach(2);
     servoForMotor2.attach(3);
     servoForMotor5.attach(4);
     servoForMotor6.attach(5);
  
  //---------------------------------------------------------------------------------

  servoForSL1.attach(8);
  servoForSL2.attach(9);
  servoForSL3.attach(10);

  pinMode(LDR1, INPUT );
  pinMode(LDR2, INPUT );
  pinMode(LDR3, INPUT );
  
  servoarm1.attach(13);
  servoarm2.attach(7);
  servoarm3.attach(6);
  servoarm4.attach(44);

  neckservo.attach(11);
  headservo.attach(12);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  dht.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
 
  ss.begin(4800);
  Serial.begin(115200);

  delay(100);

}

void loop(){
 

 //------------------------------ Convert the analog value into lux value (LDR-1)-----------------------------------------------------------------------------------

   int analogValue = analogRead(LDR1);
   float voltage = analogValue / 1024. * 5;
   float resistance = 2000 * voltage / (1 - voltage / 5);
   float lux1 = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));
 //-----------------------------------------------------------------------------------------------------------------------------------------------------------------

 //------------------------------ Convert the analog value into lux value (LDR-2)-----------------------------------------------------------------------------------

   int analogValue1 = analogRead(LDR2);
   float voltage1 = analogValue1 / 1024. * 5;
   float resistance1 = 2000 * voltage1 / (1 - voltage1 / 5);
   float lux2 = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance1, (1 / GAMMA));
 //-----------------------------------------------------------------------------------------------------------------------------------------------------------------

 //------------------------------ Convert the analog value into lux value (LDR-3)-----------------------------------------------------------------------------------
 
   int analogValue2 = analogRead(LDR3);
   float voltage2 = analogValue2 / 1024. * 5;
   float resistance2 = 2000 * voltage2 / (1 - voltage2 / 5);
   float lux3 = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance2, (1 / GAMMA));
  //----------------------------------------------------------------------------------------------------------------------------------------------------------------

 //----------------------------------Serial monitor configuration---------------------------------------------------------------------------------------------------
 Serial.println("................................................................");
   Serial.println("Valor 1: " + String(lux1) + " | Valor 2: " + String(lux2) + " | Valor 3: " + String(lux3) + " | Tipo: lux");
   getObstacleDistance();
   getTemperatureAndHumityValue();
   getChangeInAxisXYZ();
 Serial.println("................................................................");
 //----------------------------------------------------------------------------------------------------------------------------------------------------------------

 delay(10000);

}


void getObstacleDistance(){
 //Start a new measurement:
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    // Read the result:
    int duration = pulseIn(PIN_ECHO, HIGH);
    Serial.print("Distância em CM: ");
    Serial.println(duration / 58);
}

void getTemperatureAndHumityValue(){

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  Serial.println("Temperatura | sensor 1: " + String(temperature) + "°C | sensor 2 : " +String(temp.temperature) + " |");
  Serial.print("Umidade: ");
  Serial.print(humidity);
  Serial.println(" %");
}

void getChangeInAxisXYZ(){

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("aceleração : | x : "+ String(a.acceleration.x) + " |" );
  Serial.print(" y : " +String(a.acceleration.y) + " |");
  Serial.println(" z : " + String(a.acceleration.z) + " |");
  Serial.print("posição : |x : " + String(g.gyro.x) + " |");
  Serial.print(" y : " + String(g.gyro.y)+ " |");
  Serial.println( "  z  : " + String(g.gyro.z)+ " |");
}

void enginesConfiguration(){

  pinMode(Motor1_speed, OUTPUT);
  pinMode(Motor2_speed, OUTPUT);
  pinMode(Motor3_speed, OUTPUT);
  pinMode(Motor4_speed, OUTPUT);
  pinMode(Motor5_speed, OUTPUT);
  pinMode(Motor6_speed, OUTPUT);

  pinMode(Motor1_forward, OUTPUT); 
  pinMode(Motor2_forward, OUTPUT); 
  pinMode(Motor3_forward, OUTPUT); 
  pinMode(Motor4_forward, OUTPUT); 
  pinMode(Motor5_forward, OUTPUT); 
  pinMode(Motor6_forward, OUTPUT); 

  pinMode(Motor1_backward, OUTPUT); 
  pinMode(Motor2_backward, OUTPUT); 
  pinMode(Motor3_backward, OUTPUT); 
  pinMode(Motor4_backward, OUTPUT); 
  pinMode(Motor5_backward, OUTPUT); 
  pinMode(Motor6_backward, OUTPUT); 

   //start the code with the engines stopped
  digitalWrite(Motor1_backward, LOW); 
  digitalWrite(Motor2_backward, LOW); 
  digitalWrite(Motor3_backward, LOW); 
  digitalWrite(Motor4_backward, LOW); 
  digitalWrite(Motor5_backward, LOW); 

  digitalWrite(Motor1_forward, LOW); 
  digitalWrite(Motor2_forward, LOW); 
  digitalWrite(Motor3_forward, LOW); 
  digitalWrite(Motor4_forward, LOW); 
  digitalWrite(Motor5_forward, LOW); 

  digitalWrite(Motor1_speed, LOW); 
  digitalWrite(Motor2_speed, LOW); 
  digitalWrite(Motor3_speed, LOW); 
  digitalWrite(Motor4_speed, LOW); 
  digitalWrite(Motor5_speed, LOW); 
}
