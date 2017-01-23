// Must be the same value as in Python script
#define ARDUINO_BAUDRATE 115200
#define SAMPLE_TIME_MICROS 4000

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
  sendSampleTime();
}

// Loops forever
void loop() {
  int fdc;
  delay(4000);
  // Tests
  for(int i = 0; i <= NUM_FDCS; i++){
    fdc = (sqrt(FDC_START)+i*FDC_SQ_DELTA)*(sqrt(FDC_START)+i*FDC_SQ_DELTA);
    setFDC(fdc);
    randomTestSet();
  }
  
  /*
  for (int fdc_sq = sqrt(FDC_START); fdc_sq <= FDC_FINAL; fdc_sq += FDC_SQ_DELTA) {
    fdc = 100*sqrt(fdc_sq);
    setFDC(fdc);
    randomTestSet();
  }
  */
  
  // Finished
  exitProgram();

  while(1){}
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

// Stochastic square pulse test
void randomTest(unsigned long total_time, unsigned long max_pulse) {
  startTest();
  unsigned long start = millis();
  while (millis() - start < total_time) {
    digitalWrite(2, HIGH);
    streamData(random(max_pulse));
    digitalWrite(2, LOW);
    streamData(random(max_pulse));
  }
  stopTest();
}

// Stochastic square pulse test
void randomDeltaTest(unsigned long total_time, unsigned long max_spacing, unsigned long pulse_length) {
  startTest();
  unsigned long start = millis();
  while (millis() - start < total_time) {
    digitalWrite(2, HIGH);
    streamData(pulse_length);
    digitalWrite(2, LOW);
    streamData(random(max_spacing));
  }
  stopTest();
}

// Regular square pulse test
void squareWaveTest(int num_pulses, unsigned long pulse_high, unsigned long pulse_low) {
  startTest();
  for (int ii = 0; ii < num_pulses; ii++) {
    digitalWrite(2, HIGH);
    streamData(pulse_high);
    digitalWrite(2, LOW);
    streamData(pulse_low);
  }
  stopTest();
}

void squareWaveTestSet() {
  // put your main code here, to run repeatedly:

  // Test 1 - Long, slow pulses.
  squareWaveTest(3, 3000, 3000);

  // Test 2 - Medium, medium pulses.
  squareWaveTest(9, 1000, 1000);

  // Test 3 - Short, fast pulses.
  squareWaveTest(45, 200, 200);

  // Test 4 - Shorter, faster pulses.
  squareWaveTest(90, 100, 100);

  // Test 5 - Super short, hilariously fast pulses.
  squareWaveTest(225, 40, 40);
}

void randomTestSet() {
  // put your main code here, to run repeatedly:

  // Stochastic signal.
  randomTest(18000, 1000);

  // Stochastic signal.
  randomTest(18000, 500);

  // Faster.
  randomTest(18000, 250);

  // Faster.
  randomTest(18000, 125);
}

void randomDeltaTestSet() {
  // put your main code here, to run repeatedly:

  // Stochastic signal.
  randomDeltaTest(18000, 1000,20);

  // Faster.
  randomDeltaTest(18000, 500,20);
}

