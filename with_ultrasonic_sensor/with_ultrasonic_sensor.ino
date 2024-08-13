#include "VoiceRecognitionV3.h"
#include "BluetoothSerial.h" // Include the BluetoothSerial library

/**        
 *       
  Connection
  ESP32       VoiceRecognitionModule
  GPIO16  ------->     TX (VoiceRecognitionModule)
  GPIO17  ------->     RX (VoiceRecognitionModule)
  Ultrasonic Sensor
  TriggerPin ------->  GPIO12
  EchoPin    ------->  GPIO14
*/
int mrw1 = 4;
int mrw2 = 5;
int mrw3 = 18;
int mrw4 = 19;

VR myVR(16, 17);    // Use GPIO16 as RX, GPIO17 as TX
BluetoothSerial SerialBT; // Create a BluetoothSerial object

uint8_t records[7]; // save record
uint8_t buf[64];

int t = 1500; // Left Right delay
int s = 8000; // Forward and Backward delay

#define Forward   (1)
#define Left      (3)
#define Right     (4)
#define Stop      (5)

#define BT_Forward 'F'
#define BT_Left    'L'
#define BT_Right   'R'
#define BT_Stop    'S'

// Ultrasonic sensor pins
const int trigPin = 12;
const int echoPin = 14;
long duration;
int distance;

void printSignature(uint8_t *buf, int len)
{
  for(int i = 0; i < len; i++){
    if(buf[i] > 0x19 && buf[i] < 0x7F){
      Serial.write(buf[i]);
    }
    else{
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}

void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if(buf[0] == 0xFF){
    Serial.print("NONE");
  }
  else if(buf[0] & 0x80){
    Serial.print("UG ");
    Serial.print(buf[0] & (~0x80), DEC);
  }
  else{
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if(buf[3] > 0){
    printSignature(buf + 4, buf[3]);
  }
  else{
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}

void setup()
{
  /** initialize */
  myVR.begin(9600);  // Use GPIO16 as RX, GPIO17 as TX
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT"); // Start Bluetooth with a name
  Serial.println("Elechouse Voice Recognition V3 Module\r\nControl LED sample");
  
  pinMode(mrw1, OUTPUT);
  pinMode(mrw2, OUTPUT);
  pinMode(mrw3, OUTPUT);
  pinMode(mrw4, OUTPUT);

  digitalWrite(mrw1, LOW);
  digitalWrite(mrw2, LOW);
  digitalWrite(mrw3, LOW);
  digitalWrite(mrw4, LOW);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
    
  if(myVR.clear() == 0){
    Serial.println("Recognizer cleared.");
  }else{
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while(1);
  }

  if(myVR.load((uint8_t)Forward) >= 0){
    Serial.println("Forward");
  }

  if(myVR.load((uint8_t)Left) >= 0){
    Serial.println("Left");
  }

  if(myVR.load((uint8_t)Right) >= 0){
    Serial.println("Right");
  }

  if(myVR.load((uint8_t)Stop) >= 0){
    Serial.println("Stop");
  }
}

void controlMotors(int command)
{
  // Check for obstacles before moving
  measureDistance();
  
  if (command == Forward && distance < 30) {
    Serial.println("Obstacle detected! Stopping...");
    command = Stop;  // Override command to stop if an obstacle is too close
  }

  switch(command){
    case Forward:
      digitalWrite(mrw1, HIGH);
      digitalWrite(mrw2, LOW);
      digitalWrite(mrw3, HIGH);
      digitalWrite(mrw4, LOW);
      delay(s);
      break;

    case Left:
      digitalWrite(mrw1, HIGH);
      digitalWrite(mrw2, LOW);
      digitalWrite(mrw3, LOW);
      digitalWrite(mrw4, LOW);
      delay(t);
      break;

    case Right:
      digitalWrite(mrw1, LOW);
      digitalWrite(mrw2, LOW);
      digitalWrite(mrw3, LOW);
      digitalWrite(mrw4, HIGH); 
      delay(t);
      break;

    case Stop:
      digitalWrite(mrw1, LOW);
      digitalWrite(mrw2, LOW);
      digitalWrite(mrw3, LOW);
      digitalWrite(mrw4, LOW);
      break;
  }
  // Stop all motors after the action
  digitalWrite(mrw1, LOW);
  digitalWrite(mrw2, LOW);
  digitalWrite(mrw3, LOW);
  digitalWrite(mrw4, LOW);
}

void measureDistance() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2;

  // Prints the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
}

void loop()
{
  int ret;
  
  // Check for voice recognition
  ret = myVR.recognize(buf, 50);
  if(ret > 0){
    controlMotors(buf[1]);
    printVR(buf);
  }

  // Check for Bluetooth command
  if(SerialBT.available()){
    char btCommand = SerialBT.read();
    switch(btCommand){
      case BT_Forward:
        controlMotors(Forward);
        Serial.println("Bluetooth: Forward");
        break;

      case BT_Left:
        controlMotors(Left);
        Serial.println("Bluetooth: Left");
        break;

      case BT_Right:
        controlMotors(Right);
        Serial.println("Bluetooth: Right");
        break;

      case BT_Stop:
        controlMotors(Stop);
        Serial.println("Bluetooth: Stop");
        break;
        
      default:
        Serial.println("Unknown Bluetooth command");
        break;
    }
  }
}
