#include <ServoTimer2.h>
#include <TimerOne.h>
#include <ACS712.h>
#include <Encoder.h>
#include <Adafruit_AHT10.h>
#include <RCSwitch.h>

#define RELAY1          8
#define RELAY2          9
#define PIEZO           6

#define CLK             2
#define DT              7        //former D3
#define SW              12

#define CURSENS         A0

#define MODE0           A1       //former A6
#define MODE1           A2       //former A3
#define MODE2           A3       //former A2
#define MODE3           A6       //former A1

#define ONOFF1          13
#define ONOFF2          4

#define RXDATA          3       //former D7     D3 = Interrupt 1

#define SERVO           5

#define CHECKERPERIOD   20000000  // Ambient check period in microseconds 
#define ROTARYSENS      15        // Degrees servo position per encoder click
#define OVERHEAT        60        // Temperature Limit

RCSwitch mySwitch = RCSwitch();
Encoder myEnc(CLK, DT);
ACS712  ACS(CURSENS, 5.0, 1023, 100);
ServoTimer2 servo;

Adafruit_AHT10 aht;
sensors_event_t humidity, temp;
volatile bool checkAmb = false;

int currentMode = 0;
bool haltTriggered = false;

int servoPosition = 2300;

int getActiveMode() {  
  if(analogRead(MODE0) > 512) return 0;
  if(analogRead(MODE1) > 512) return 1;
  if(analogRead(MODE2) > 512) return 2;
  if(analogRead(MODE3) > 512) return 3;

  return 0;  
}

void ISR_checkAmbient(void) {
  checkAmb = true;
}


void setup() {
  Serial.begin(115200);
  
  ACS.autoMidPoint();

  servo.attach(SERVO);

  mySwitch.enableReceive(1);

  Serial.println("Absauganlage v2.0.0 started");
  
  while(!aht.begin());

  Timer1.initialize(CHECKERPERIOD);
  Timer1.attachInterrupt(ISR_checkAmbient);
  
  pinMode(RELAY1, OUTPUT);
  digitalWrite(RELAY1, LOW);

  pinMode(RELAY2, OUTPUT);
  digitalWrite(RELAY2, HIGH);

  pinMode(PIEZO, OUTPUT);
  digitalWrite(PIEZO, LOW);

  pinMode(MODE0, INPUT);
  pinMode(MODE1, INPUT);
  pinMode(MODE2, INPUT);
  pinMode(MODE3, INPUT);

  pinMode(ONOFF1, INPUT);
  pinMode(ONOFF2, INPUT);

  pinMode(RXDATA, INPUT);

}

void loop() {
  long newPosition = myEnc.read();

  bool modechanged = false;
  if(currentMode != getActiveMode()) {
    currentMode = getActiveMode();
    Serial.println(currentMode);
    for(int i = 0; i<currentMode+1; i++){
      digitalWrite(PIEZO, HIGH);
      delay(100);
      digitalWrite(PIEZO, LOW);
      delay(100);
    }
        
    modechanged = true;
    digitalWrite(RELAY1, LOW);
  }

  if(!haltTriggered){
    // Modus 0: Schaltermodus
    if(currentMode==0) {
        if(digitalRead(ONOFF1) || digitalRead(ONOFF2)) {
          digitalWrite(RELAY1, HIGH);
        }
        else digitalWrite(RELAY1, LOW);
    }
    // Modus 1: Automatikmodus
    else if(currentMode==1) {
      Serial.println(ACS.mA_AC());
      if(ACS.mA_AC() >= 1000) {
        digitalWrite(RELAY1, HIGH);
      }
      else digitalWrite(RELAY1, LOW);
    }
    // Modus 2: Funkmodus
    else if(currentMode==2) {
      if (mySwitch.available()) {
      
      Serial.print("Received ");
      Serial.print( mySwitch.getReceivedValue() );
      Serial.print(" / ");
      Serial.print( mySwitch.getReceivedBitlength() );
      Serial.print("bit ");
      Serial.print("Protocol: ");
      Serial.println( mySwitch.getReceivedProtocol() );
  
      mySwitch.resetAvailable();
  }
    }
    // Modus 3: Handmodus
    else {
      digitalWrite(RELAY1, HIGH);
    }
  }
  
  if(checkAmb) {
    checkAmb = false;
    aht.getEvent(&humidity, &temp);
    Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
    Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");
  }

  if(temp.temperature >= OVERHEAT) {
    haltTriggered = true;
    digitalWrite(PIEZO, !digitalRead(PIEZO));
    digitalWrite(RELAY1, LOW);
  }
  else {
    haltTriggered = false;
    digitalWrite(PIEZO, LOW);
  }

  servoPosition = servoPosition-ROTARYSENS*myEnc.read();
  servoPosition = constrain(servoPosition, 750, 2300);
  
  myEnc.write(0);

  servo.write(servoPosition);
//  Serial.println(servoPosition);
  
  delay(500);

}
