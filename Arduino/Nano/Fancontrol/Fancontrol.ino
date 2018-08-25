/*

  Lineare Lüftersteuerung für bis zu 4 12V-Lüfter und bis zu 5 DS18B20 Temperaturfühler
  1 Fühler je Lüfter
  zusätzlich 1 Fühler für die Umgebungstemperatur

  Hier das Original:
  
  Motor Control with a Transistor

  This example shows you how to control a motor's using a transistor.
  When a pushbutton on pin 2 is pressed, the Arduino will control a transistor
  via PWM, which will slowly ramp up the motor's speedpwm1, then slow it down.

  The circuit :
   momentary switch with one end connected to 5V, the other end connected
   to GND through a 10-kilohm resistor, and digital pin 2.
   TIP120 tranistor, with the Base connected to digital pin 9, the Emitter to ground,
   and the Collector to one lead from a 9V DC motor
   a 9V battery, with the ground connected to the Arduino's ground, and the power
   connected to the motor
   1N4001 diode across the motor's leads, with the striped side conneted to the 9V

  The Arduino can only provide 40mA at 5V on its pins. Most motors require
  more current and/or voltage to overcome intertia and run. A transistor
  can act as a digital switch, enabling the Arduino to control loads with
  higher electrical requirements.

  Created on 03 January 2013
  by Scott Fitzgerald

  http://www.arduino.cc/en/Tutorial/TransistorMotorControl

  This example code is in the public domain.
*/

#include <OneWire.h>
#include <DallasTemperature.h>

//const int Fan3 = 6;
//const int Fan4 = 9;

//const int TempSensorAmbient = A0;
const int TempSensorFan = 2;

int Fan[5]; // array Initialisierung
int MinRPM[5];
int StartRPM[5];
int StartDelay[5];
String TempName[5];
float tempC[5];
float SpreadTempC[5];
float MinOverTempC[5];
float lastpwm[10];
const bool serial = false;
OneWire oneWire(TempSensorFan);
DallasTemperature sensors(&oneWire);

float maxStep = 5; // maximale Speed-Änderung um Sprünge zu vermeiden
DeviceAddress TempSensor1 = { 0x28, 0xAA, 0xD4, 0x51, 0x16, 0x13, 0x2, 0x5 };
DeviceAddress TempSensor2 = { 0x28, 0xAA, 0x92, 0x5C, 0x16, 0x13, 0x2, 0xED };
DeviceAddress TempSensor3 = { 0x28, 0xAA, 0xD1, 0xA5, 0x13, 0x13, 0x2, 0x9D };
DeviceAddress TempSensor4 = { 0x28, 0xAA, 0xFD, 0xA8, 0x13, 0x13, 0x2, 0xD9 };
DeviceAddress TempSensorAmbient = { 0x28, 0xB7, 0x39, 0xC4, 0x1B, 0x13, 0x1, 0xB0 };

void setup() {

  Fan[1] = 3;
  MinRPM[1] = 16; // 120er
  StartRPM[1] = 15; // Startdrehzahl
  StartDelay[1] = 1100; // Zeit zum anlaufen in ms
  SpreadTempC[1] = 4; // bei x °C mehr hat Lüfter 100%
  MinOverTempC[1] = 1; // bei x °C mehr als Umbegung = Min-Drehzal
  TempName[1] = "120er";

  Fan[2] = 5;
  MinRPM[2] = 21; // 80er
  StartRPM[2] = 30;
  StartDelay[2] = 600;
  SpreadTempC[2] = 2;
  MinOverTempC[2] = 1;
  TempName[2] = "80er";

  TempName[5] = "Umgebung";

  for ( int i = 1; i <= 5; i++ ) {
    pinMode(Fan[i], OUTPUT);
    lastpwm[i] = 0;
  }

  // http://arduino-info.wikispaces.com/Arduino-PWM-Frequency
  // Arduino Pins 5 and 6: 1kHz
  // Arduino Pins 9, 10, 11, and 3: 500Hz
  //---------------------------------------------- Set PWM frequency for D5 & D6 -------------------------------
  TCCR0B = TCCR0B & B11111000 | B00000001;    // set timer 0 divisor to     1 for PWM frequency of 62500.00 Hz
  //---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  //---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------
  TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz

  sensors.begin();

  if ( serial )  Serial.begin(9600);
  mydelay(1000);
  //  findDevices(TempSensorFan); // DS18B20 Suche
}

void loop() {
  //  goToSerial();
  managefansbytemp();
}

void managefan(int fan, float tempC, float AmbientTempC, float SpreadTempC, float MinOverTempC) {
  if ( serial ) {
    Serial.print("Fan");
    Serial.print(fan);
    Serial.print(", tempC=");
    Serial.print(tempC);
    Serial.print(", AmbientTempC=");
    Serial.print(AmbientTempC);
    Serial.print(", SpreadTempC=");
    Serial.print(SpreadTempC);
    Serial.print(", MinOverTempC=");
    Serial.println(MinOverTempC);
  }
  int pwm;
  if ( ( tempC - MinOverTempC ) < AmbientTempC ) {
    pwm = 0;
  } else {
    pwm = ( 255 / SpreadTempC * ( tempC - AmbientTempC - MinOverTempC ) );

    if ( pwm < MinRPM[fan] ) pwm = MinRPM[fan];
    if ( pwm > 255 ) pwm = 255;
  }
  controlFan(fan, pwm);
}

void managefansbytemp() {
  float AmbientTempC = sensors.getTempC(TempSensorAmbient);
  if ( serial ) {
    Serial.print("Ambient-Temp: ");
    Serial.print(AmbientTempC);
    Serial.println("°C");
  }

  for ( int i = 1; i <= 2; i++ ) {
    switch (i) {
      case 1:
        tempC[i] = sensors.getTempC(TempSensor1);
        break;
      case 2:
        tempC[i] = sensors.getTempC(TempSensor2);
        break;
    }
    if ( serial ) {
      Serial.print(TempName[i]);
      Serial.print(": ");
      Serial.print(tempC[i]);
      Serial.println("°C");
    }
    managefan(i, tempC[i], AmbientTempC, SpreadTempC[i], MinOverTempC[i]);
  }
  //  mydelay(500);
}

void mydelay(long xdelay) { // Um wegen des Timings wieder aus 1 eine 1ms zu machen
  delay( xdelay * 64 );
}

void controlFan(int fan, float speedpwm) {
  sensors.requestTemperatures();
  switch (fan) {
    case 1:
      tempC[fan] = sensors.getTempC(TempSensor1);
      break;
    case 2:
      tempC[fan] = sensors.getTempC(TempSensor2);
      break;
    case 3:
      tempC[fan] = sensors.getTempC(TempSensor3);
      break;
    case 4:
      tempC[fan] = sensors.getTempC(TempSensor4);
      break;
  }

  if ( serial ) {
    Serial.print("Fan");
    Serial.print(fan);
    Serial.print(" (");
    Serial.print(TempName[fan]);
    Serial.print("): (soll) = ");
    Serial.print(speedpwm);
    Serial.print(", (letzte) = ");
    Serial.print(lastpwm[fan]);
    Serial.print(", (MinRPM) = ");
    Serial.print(MinRPM[fan]);
    Serial.print(", (StartRPM) = ");
    Serial.print(StartRPM[fan]);
    Serial.print(", (Temperatur) = ");
    Serial.print(tempC[fan]);
  }

  if ( speedpwm > lastpwm[fan] ) {  // Speed +
    if ( ( speedpwm - lastpwm[fan] ) > maxStep )
      speedpwm = lastpwm[fan] + maxStep;
  } else {                          // Speed -
    if ( ( lastpwm[fan] - speedpwm ) > maxStep )
      speedpwm = lastpwm[fan] - maxStep;
  }
  if ( ( lastpwm[fan] == 0 ) && ( speedpwm > 0 )  && ( speedpwm < StartRPM[fan] ) ) { // Anlasser
    analogWrite(Fan[fan], StartRPM[fan]);
    mydelay(StartDelay[fan]);
  }
  lastpwm[fan] = speedpwm;
  if ( speedpwm != 0 ) {
    if ( speedpwm < MinRPM[fan] ) speedpwm = MinRPM[fan];
    if ( speedpwm > 255 ) speedpwm = 255;
  }
  if ( serial ) {
    Serial.print(", (ist) = ");
    Serial.println(speedpwm);
    analogWrite(Fan[fan], speedpwm);
  }
  analogWrite(Fan[fan], speedpwm);
}

void goToSerial() {
  // https://www.mikrocontroller.net/topic/241837#2482570
  if ( serial ) {
    int i, j;
    char X_buffer[3];       //Max. Zahl "999"
    int pwm = 0;

    while (pwm == 0) {
      if (Serial.available()) {
        Serial.flush();
        mydelay(5);
        for ( i = 0; i <= 4; i++ )
          X_buffer[i] = Serial.read();
        Serial.flush();
        pwm = atoi(X_buffer);
        if ( pwm < 0 ) pwm = 0;
        if ( pwm > 255 ) pwm = 255;
        Serial.print("PWM: ");
        Serial.println(pwm);
        //      for ( j = 1; j <= 3; j++ ) {
        //        controlFan(j, pwm);
        //      }
        controlFan(1, pwm);
        controlFan(2, pwm);
      }
    }
    pwm = 0;
  }
}

uint8_t findDevices(int pin) { // finde Adressen der Temperatursensoren
  if ( serial ) {
    Serial.println("Suche Temperatursensoren...");
    OneWire ow(pin);

    uint8_t address[8];
    uint8_t count = 0;


    if (ow.search(address)) {
      Serial.print("\nuint8_t pin");
      Serial.print(pin, DEC);
      Serial.println("[][8] = {");
      {
        count++;
        Serial.println("  {");
        for (uint8_t i = 0; i < 8; i++) {
          Serial.print("0x");
          if (address[i] < 0x10) Serial.print("0");
          Serial.print(address[i], HEX);
          if (i < 7) Serial.print(", ");
        }
        Serial.print("  },");
        // CHECK CRC
        if (ow.crc8(address, 7) == address[7]) {
          Serial.println("\t\t// CRC OK");
        } else {
          Serial.println("\t\t// CRC FAILED");
        }
      }
      while (ow.search(address));

      Serial.println("};");
      Serial.print("// nr devices found: ");
      Serial.println(count);
    }

    return count;
  }
}

