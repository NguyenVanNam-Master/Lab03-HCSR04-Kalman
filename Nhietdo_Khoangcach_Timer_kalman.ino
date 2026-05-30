
#include <Arduino.h>
#include <math.h>
const byte TRIG_PIN = 9; 
const byte ECHO_PIN_ICP1 = 8;

const float TEMP_C = 30.0;        
const int SAMPLE_COUNT = 100; 
const unsigned long SAMPLE_GAP_MS = 60;

const float MIN_VALID_CM = 2.0;
const float MAX_VALID_CM = 400.0;
const float TIMER1_TICK_US = 0.5;
const unsigned long MEASURE_TIMEOUT_MS = 35;

const float RAW_SOUND_SPEED_CM_PER_US = 0.0343; 

volatile bool waitingFallingEdge = false;
volatile bool captureDone = false;
volatile uint16_t echoStartTick = 0;
volatile uint16_t echoWidthTicks = 0;
 
float kalmanX = 0.0;
float kalmanP = 1.0;
const float kalmanQ = 0.03; 
const float kalmanR = 4.00;  
bool kalmanInitialized = false;

float drefArr[SAMPLE_COUNT];
float drawArr[SAMPLE_COUNT];
float dcalArr[SAMPLE_COUNT];
float dkalArr[SAMPLE_COUNT];
bool validArr[SAMPLE_COUNT];

void setupTimer1InputCapture() {
  pinMode(ECHO_PIN_ICP1, INPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11);

  TIFR1 = (1 << ICF1) | (1 << TOV1);

  TIMSK1 &= ~(1 << ICIE1);
}

ISR(TIMER1_CAPT_vect) {
  uint16_t nowTick = ICR1;

  if (!waitingFallingEdge) {
    echoStartTick = nowTick;
    waitingFallingEdge = true;

    TCCR1B &= ~(1 << ICES1);
  } else {
    echoWidthTicks = (uint16_t)(nowTick - echoStartTick);
    captureDone = true;
    waitingFallingEdge = false;

    TIMSK1 &= ~(1 << ICIE1);

    TCCR1B |= (1 << ICES1);
  }
}

bool measureEchoByTimer1(float &durationUs) {
  noInterrupts();

  captureDone = false;
  waitingFallingEdge = false;
  echoStartTick = 0;
  echoWidthTicks = 0;

  TCNT1 = 0;
  TIFR1 = (1 << ICF1) | (1 << TOV1);

  TCCR1B |= (1 << ICES1);

  TIMSK1 |= (1 << ICIE1);

  interrupts();

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long t0 = millis();

  while (!captureDone) {
    if (millis() - t0 > MEASURE_TIMEOUT_MS) {
      noInterrupts();
      TIMSK1 &= ~(1 << ICIE1);
      waitingFallingEdge = false;
      captureDone = false;
      TCCR1B |= (1 << ICES1);
      interrupts();

      durationUs = 0.0;
      return false;
    }
  }

  noInterrupts();
  uint16_t ticks = echoWidthTicks;
  interrupts();

  durationUs = ticks * TIMER1_TICK_US;
  return true;
}

float soundSpeedCmPerUsByTemp(float tempC) {
  return (331.3 + 0.606 * tempC) / 10000.0;
}

float kalmanUpdate(float measurement) {
  if (!kalmanInitialized) {
    kalmanX = measurement;
    kalmanP = 1.0;
    kalmanInitialized = true;
    return kalmanX;
  }

  // Predict
  kalmanP = kalmanP + kalmanQ;

  // Update
  float K = kalmanP / (kalmanP + kalmanR);
  kalmanX = kalmanX + K * (measurement - kalmanX);
  kalmanP = (1.0 - K) * kalmanP;

  return kalmanX;
}

void resetKalman() {
  kalmanInitialized = false;
  kalmanX = 0.0;
  kalmanP = 1.0;
}

// ===================== THỐNG KÊ =====================
int countValid() {
  int n = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (validArr[i]) n++;
  }
  return n;
}

float meanOf(const float arr[]) {
  float sum = 0.0;
  int n = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (validArr[i]) {
      sum += arr[i];
      n++;
    }
  }

  if (n == 0) return NAN;
  return sum / n;
}

float stdDevOf(const float arr[], float meanValue) {
  int n = 0;
  float sumSq = 0.0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (validArr[i]) {
      float e = arr[i] - meanValue;
      sumSq += e * e;
      n++;
    }
  }

  if (n <= 1) return NAN;

  return sqrt(sumSq / (n - 1));
}

void printStatsLine(const char *name, float meanValue, float stdValue, float drefMean) {
  float error = meanValue - drefMean;
  float absError = fabs(error);
  float percentError = (drefMean != 0.0) ? (absError / fabs(drefMean)) * 100.0 : NAN;

  Serial.print(name);
  Serial.print(",");
  Serial.print(meanValue, 3);
  Serial.print(",");
  Serial.print(stdValue, 3);
  Serial.print(",");
  Serial.print(error, 3);
  Serial.print(",");
  Serial.print(absError, 3);
  Serial.print(",");
  Serial.println(percentError, 3);
}

void printSummary() {
  int validN = countValid();

  float meanDref = meanOf(drefArr);
  float meanDraw = meanOf(drawArr);
  float meanDcal = meanOf(dcalArr);
  float meanDkal = meanOf(dkalArr);

  float stdDref = stdDevOf(drefArr, meanDref);
  float stdDraw = stdDevOf(drawArr, meanDraw);
  float stdDcal = stdDevOf(dcalArr, meanDcal);
  float stdDkal = stdDevOf(dkalArr, meanDkal);

  Serial.println();
  Serial.println("===== SUMMARY 100 SAMPLES =====");
  Serial.print("ValidSamples,");
  Serial.println(validN);

  Serial.println("Signal,Mean(cm),StdDev(cm),Error(cm),AbsError(cm),PercentError(%)");

  printStatsLine("dref", meanDref, stdDref, meanDref);
  printStatsLine("draw", meanDraw, stdDraw, meanDref);
  printStatsLine("dcal", meanDcal, stdDcal, meanDref);
  printStatsLine("dkalman", meanDkal, stdDkal, meanDref);

  Serial.println("===== END SUMMARY =====");
  Serial.println();
}

// ===================== THU THẬP 100 MẪU =====================
void collect100Samples(float drefCm) {
  resetKalman();

  Serial.println();
  Serial.print("Start collecting 100 samples at dref = ");
  Serial.print(drefCm, 2);
  Serial.println(" cm");
  Serial.print("Temperature_C,");
  Serial.println(TEMP_C, 2);

  Serial.println("Time(ms),dref(cm),draw(cm),dcal(cm),dkalman(cm),Valid");

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    float durationUs = 0.0;
    bool echoOk = measureEchoByTimer1(durationUs);

    float draw = NAN;
    float dcal = NAN;
    float dkal = NAN;
    bool valid = false;

    if (echoOk) {
      draw = (durationUs * RAW_SOUND_SPEED_CM_PER_US) / 2.0;
      dcal = (durationUs * soundSpeedCmPerUsByTemp(TEMP_C)) / 2.0;
      valid = (dcal >= MIN_VALID_CM && dcal <= MAX_VALID_CM);

      if (valid) {
        dkal = kalmanUpdate(dcal);
      }
    }

    drefArr[i] = drefCm;
    drawArr[i] = draw;
    dcalArr[i] = dcal;
    dkalArr[i] = dkal;
    validArr[i] = valid;

    Serial.print(millis());
    Serial.print(",");
    Serial.print(drefCm, 3);
    Serial.print(",");
    if (isnan(draw)) Serial.print("nan"); else Serial.print(draw, 3);
    Serial.print(",");
    if (isnan(dcal)) Serial.print("nan"); else Serial.print(dcal, 3);
    Serial.print(",");
    if (isnan(dkal)) Serial.print("nan"); else Serial.print(dkal, 3);
    Serial.print(",");
    Serial.println(valid ? 1 : 0);

    delay(SAMPLE_GAP_MS);
  }

  printSummary();
}

// ===================== SERIAL COMMAND =====================
void printGuide() {
  Serial.println("Arduino Uno + HC-SR04 Timer1 Input Capture");
  Serial.println("Nhap moc khoang cach chuan theo thuoc do, vi du:");
  Serial.println("D 10");
  Serial.println("D 15.5");
  Serial.println("D 20");
  Serial.println("Moi lenh se thu thap 100 mau va in bang CSV.");
  Serial.println("Cot du lieu: Time(ms),dref(cm),draw(cm),dcal(cm),dkalman(cm),Valid");
  Serial.println();
}

void handleSerialCommand() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  line.replace(",", ".");

  if (line.length() == 0) return;

  if (line == "?" || line == "help" || line == "HELP") {
    printGuide();
    return;
  }

  float drefCm = NAN;

  if (line.charAt(0) == 'D' || line.charAt(0) == 'd') {
    String valuePart = line.substring(1);
    valuePart.trim();
    drefCm = valuePart.toFloat();
  } else {
    // Cho phép nhập trực tiếp: 10 hoặc 15.5
    drefCm = line.toFloat();
  }

  if (drefCm <= 0.0 || isnan(drefCm)) {
    Serial.println("Lenh khong hop le. Vi du dung: D 20");
    return;
  }

  collect100Samples(drefCm);
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  setupTimer1InputCapture();

  delay(500);
  printGuide();
}

void loop() {
  handleSerialCommand();
}
