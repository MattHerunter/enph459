// Must be the same value as in Python script
#define ARDUINO_BAUDRATE 115200

// Sample time in microseconds
#define SAMPLE_TIME_MICROS 1000

// Pin that controls the heating element MOSFET
#define MOSFET_GATE_PIN 2

// Number of seconds to wait after updating the fan duty cycle
#define FAN_ADJUST_TIME 3

// Constant array to hold the pins reading from the thermocouples
const int ANALOG_INS[] = {4, 5};
const int COMP_INS[] = {0, 1};

// Number of pins reading from thermocouples
const int NUM_ANALOG_INS = sizeof(ANALOG_INS) / sizeof(ANALOG_INS[0]);

// Global holding the status of the heater
int HEATER_STATUS = 0;

// ---List of Python Commands---
// Start     - Start streaming data
// Stop      - Stop streaming data
// Set:fdc:w - Send a command to the controller to set the FDC to fdc. Arduino will wait w seconds for the fan to adjust.
// Exit      - Exit program
// Time:dt   - Send the sample time in microseconds dt

// Runs once at beginning
void setup() {
  // Initialization
  Serial.begin(ARDUINO_BAUDRATE);
  sendSampleTime();
  startTest();
}

// Loops forever
void loop() {
  float starting_freq_khz = 2.0f / 1000;
  unsigned long chirp_length_millis = 5000;
  float freq_khz_slope = (0.1f - starting_freq_khz) / (50000);
  
  //chirpSignal(starting_freq_khz, freq_khz_slope, chirp_length_millis);
  
    int high_pulse = 150+random(50);
    int low_pulse = 150+random(50);
    turnHeaterOn();
    streamData(high_pulse);
    turnHeaterOff();
    streamData(low_pulse);
}

// Writes data read from the analog ports to the serial ports in format HEATER_STATUS, TC1, TC2, ..., COMP1, COMP2, ...\n
void streamData(unsigned long time_millis) {
  int num_samples = time_millis * 1000 / SAMPLE_TIME_MICROS;
  for (int ii = 0; ii < num_samples; ii++) {
    delayMicroseconds(SAMPLE_TIME_MICROS);
    Serial.print(HEATER_STATUS);
    Serial.print(",");
    for (int jj = 0; jj < NUM_ANALOG_INS; jj++) {
      Serial.print(analogRead(ANALOG_INS[jj]));
      Serial.print(",");
    }
    for (int jj = 0; jj < NUM_ANALOG_INS; jj++) {
      Serial.print(analogRead(COMP_INS[jj]));
      Serial.print(",");
    }
    Serial.print(analogRead(COMP_INS[NUM_ANALOG_INS - 1]));
    Serial.print("\n");
  }
}

// Changes heater duty cycle if a command is sent from python.
void updateHeater() {
  int heaterStrength = Serial.read();

  if(heaterStrength > -1) {
    analogWrite(MOSFET_GATE_PIN, heaterStrength);
  }
}

// Code for up chirp signal
void chirpSignal(float starting_freq_khz, float freq_khz_slope, unsigned long chirp_length_millis) {
  int time_millis = (int)(1.0 / starting_freq_khz);
  float freq_khz = starting_freq_khz;
  unsigned long start_time_millis = millis();
  while (millis() - start_time_millis < chirp_length_millis) {
    time_millis = (int)(1.0 / freq_khz);
    //freq_khz = exp(log(freq_khz)+time_millis * freq_khz_slope);
    //freq_khz = log(exp(freq_khz)+time_millis * freq_khz_slope);
    freq_khz += time_millis * freq_khz_slope;
    turnHeaterOn();
    streamData(time_millis);

    time_millis = (int)(1.0 / freq_khz);
    //freq_khz = exp(log(freq_khz)+time_millis * freq_khz_slope);
    //freq_khz = log(exp(freq_khz)+time_millis * freq_khz_slope);
    freq_khz += time_millis * freq_khz_slope;
    turnHeaterOff();
    streamData(time_millis);
  }
}

// Turns the heater on and sets the value of HEATER_STATUS to 1
void turnHeaterOn() {
  digitalWrite(MOSFET_GATE_PIN, HIGH);
  HEATER_STATUS = 1;
}

// Turns the heater off and sets the value of HEATER_STATUS to -1
void turnHeaterOff() {
  digitalWrite(MOSFET_GATE_PIN, LOW);
  HEATER_STATUS = -1;
}
//---------------Python Commands Beneath Here---------------
// Called at the start of every test
void startTest() {
  delay(4000);
  Serial.flush();
  Serial.print("Start\n");
  turnHeaterOff();
  streamData(1000);
}

// Called at the end of every test
void stopTest() {
  streamData(4000);
  Serial.print("Stop\n");
}

// Called to tell Python to set the FDC on the controller to fdc
void setFDC(int fdc) {
  String s = (String)"Set:" + (String)fdc + (String)":" + (String)FAN_ADJUST_TIME + (String)"\n";
  Serial.print(s);
  // Wait for fan adjust time
  delay(FAN_ADJUST_TIME * 1000);
}

// Called to tell Python data collection is over, return 0
void exitProgram() {
  Serial.print("Exit\n");
}

// Called to tell Python to the sample time in microseconds
void sendSampleTime() {
  String s = (String)"Time:" + (String)SAMPLE_TIME_MICROS + (String)"\n";
  Serial.print(s);
}
