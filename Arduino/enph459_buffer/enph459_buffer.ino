// Must be the same value as in Python script
#define ARDUINO_BAUDRATE 115200
#define SAMPLE_TIME_MICROS 1000

// Pin that controls the heating element MOSFET
#define MOSFET_GATE_PIN 2

// Number of seconds to wait after updating the fan duty cycle
#define FAN_ADJUST_TIME 3

// Fan Duty Cycle Settings (scaled down by a factor of 100)
#define FDC_START 1200
#define FDC_FINAL 1900
#define NUM_FDCS 20
#define FDC_SQ_DELTA ((sqrt(FDC_FINAL)-sqrt(FDC_START))/NUM_FDCS)

// Constant array to hold the pins reading from the thermocouples
const int ANALOG_INS[] = {0, 1};

// Number of pins reading from thermocouples
const int NUM_ANALOG_INS = sizeof(ANALOG_INS) / sizeof(ANALOG_INS[0]);

// ---List of Python Commands---
// Start     - Start streaming data to file
// Stop      - Stop streaming data to file
// Set:fdc:w - Send a command to the controller to set the FDC to fdc. Arduino will wait w seconds for the fan to adjust.
// Exit      - Done data collection, exit program
// Time:dt   - Send the sample time in microseconds dt

// Runs once at beginning
void setup() {
  // Initialization
  Serial.begin(ARDUINO_BAUDRATE);
  //setFDC(FDC_START);
  sendSampleTime();
  startTest();
}

// Loops forever
void loop() {
  int high_pulse = 200;
  int low_pulse = 200;
  
  digitalWrite(2, HIGH);
  streamData(random(high_pulse));
  digitalWrite(2, LOW);
  streamData(random(low_pulse));
}

// Writes data read from the analog ports to the serial ports
void streamData(unsigned long time_millis) {
  int num_samples = time_millis * 1000 / SAMPLE_TIME_MICROS;
  for (int ii = 0; ii < num_samples; ii++) {
    delayMicroseconds(SAMPLE_TIME_MICROS);
    for (int jj = 0; jj < NUM_ANALOG_INS - 1; jj++) {
      Serial.print(analogRead(ANALOG_INS[jj]));
      Serial.print(",");
    }
    Serial.print(analogRead(ANALOG_INS[NUM_ANALOG_INS - 1]));
    Serial.print("\n");
  }
}

// Called at the start of every test
void startTest() {
  delay(4000);
  Serial.flush();
  Serial.print("Start\n");
  digitalWrite(MOSFET_GATE_PIN, LOW);
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