
#include <Arduino.h>
#include <math.h>

const byte TRIG_PIN = 9;
const byte ECHO_PIN_ICP1 = 8;

float TEMP_C = 30.0;

const unsigned long SAMPLE_PERIOD_MS = 50;
const unsigned long MEASURE_TIMEOUT_MS = 35;

const float MIN_VALID_CM = 2.0;
const float MAX_VALID_CM = 400.0;

const float TIMER1_TICK_US = 0.5;

const int MEDIAN5_SIZE = 5;
const int MEDIAN9_SIZE = 9;
const int OUTLIER_WINDOW = 10;
const int STATIC_SAMPLE_COUNT = 100;

volatile bool waitingFallingEdge = false;
volatile bool captureDone = false;
volatile uint16_t echoStartTick = 0;
volatile uint16_t echoWidthTicks = 0;

float buf5[MEDIAN5_SIZE];
float buf9[MEDIAN9_SIZE];

int idx5 = 0;
int idx9 = 0;
int count5 = 0;
int count9 = 0;

float outlierBuf[OUTLIER_WINDOW];
int outlierIdx = 0;
int outlierCount = 0;

bool runTracking = false;
unsigned long lastSampleTime = 0;

float prevKalman5Dist = NAN;
float prevKalman9Dist = NAN;
unsigned long prevTrackingTime = 0;

// ======================= KALMAN 1D =======================

class Kalman1D {
  public:
    float q;
    float r;
    float x;
    float p;
    float k;
    bool initialized;

    Kalman1D(float processNoise, float measurementNoise) {
      q = processNoise;
      r = measurementNoise;
      x = 0.0;
      p = 1.0;
      k = 0.0;
      initialized = false;
    }

    void setQR(float processNoise, float measurementNoise) {
      q = processNoise;
      r = measurementNoise;
    }

    void reset() {
      x = 0.0;
      p = 1.0;
      k = 0.0;
      initialized = false;
    }

    float update(float measurement) {
      if (!initialized) {
        x = measurement;
        p = 1.0;
        initialized = true;
        return x;
      }

      p = p + q;
      k = p / (p + r);
      x = x + k * (measurement - x);
      p = (1.0 - k) * p;

      return x;
    }
};

float KALMAN_Q = 0.25;
float KALMAN_R = 0.60;

Kalman1D kalman5(KALMAN_Q, KALMAN_R);
Kalman1D kalman9(KALMAN_Q, KALMAN_R);

// ======================= TIMER1 INPUT CAPTURE =======================

void setupTimer1InputCapture() {
  pinMode(ECHO_PIN_ICP1, INPUT);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // ICNC1: chong nhieu input capture
  // ICES1: bat suon len ban dau
  // CS11 : prescaler 8, tick = 0.5 us voi F_CPU = 16 MHz
  TCCR1B = (1 << ICNC1) | (1 << ICES1) | (1 << CS11);

  TIFR1 = (1 << ICF1) | (1 << TOV1);
  TIMSK1 &= ~(1 << ICIE1);
}

ISR(TIMER1_CAPT_vect) {
  uint16_t nowTick = ICR1;

  if (!waitingFallingEdge) {
    echoStartTick = nowTick;
    waitingFallingEdge = true;
    TCCR1B &= ~(1 << ICES1); // doi sang bat suon xuong
  } else {
    echoWidthTicks = (uint16_t)(nowTick - echoStartTick);
    captureDone = true;
    waitingFallingEdge = false;

    TIMSK1 &= ~(1 << ICIE1);
    TCCR1B |= (1 << ICES1); // reset ve bat suon len
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

      durationUs = NAN;
      return false;
    }
  }

  noInterrupts();
  uint16_t ticks = echoWidthTicks;
  interrupts();

  durationUs = ticks * TIMER1_TICK_US;
  return true;
}

// ======================= DO KHOANG CACH =======================

float soundSpeedCmPerUsByTemp(float tempC) {
  // v = 331.3 + 0.606*T (m/s)
  // doi sang cm/us: chia 10000
  return (331.3 + 0.606 * tempC) / 10000.0;
}

float measureDistanceRawCm(bool &valid) {
  float durationUs = NAN;
  bool echoOk = measureEchoByTimer1(durationUs);

  if (!echoOk || isnan(durationUs)) {
    valid = false;
    return NAN;
  }

  float distanceCm = (durationUs * soundSpeedCmPerUsByTemp(TEMP_C)) / 2.0;

  valid = (distanceCm >= MIN_VALID_CM && distanceCm <= MAX_VALID_CM);
  if (!valid) return NAN;

  return distanceCm;
}

// ======================= FILTER MEDIAN/AVERAGE =======================

void resetFilters() {
  idx5 = 0;
  idx9 = 0;
  count5 = 0;
  count9 = 0;

  outlierIdx = 0;
  outlierCount = 0;

  prevKalman5Dist = NAN;
  prevKalman9Dist = NAN;
  prevTrackingTime = 0;

  kalman5.reset();
  kalman9.reset();
}

void addToBuffers(float x) {
  buf5[idx5] = x;
  idx5 = (idx5 + 1) % MEDIAN5_SIZE;
  if (count5 < MEDIAN5_SIZE) count5++;

  buf9[idx9] = x;
  idx9 = (idx9 + 1) % MEDIAN9_SIZE;
  if (count9 < MEDIAN9_SIZE) count9++;
}

float medianOfArray(float arr[], int n) {
  float sorted[MEDIAN9_SIZE];

  for (int i = 0; i < n; i++) {
    sorted[i] = arr[i];
  }

  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (sorted[j] > sorted[j + 1]) {
        float tmp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = tmp;
      }
    }
  }

  if (n % 2 == 1) return sorted[n / 2];
  return (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;
}

float movingAverageOfArray(float arr[], int n) {
  float sum = 0.0;
  for (int i = 0; i < n; i++) sum += arr[i];
  return sum / n;
}

float meanOfArray(const float arr[], int n) {
  float sum = 0.0;
  for (int i = 0; i < n; i++) sum += arr[i];
  return sum / n;
}

float stdOfArray(const float arr[], int n, float meanValue) {
  if (n <= 1) return NAN;

  float sumSq = 0.0;
  for (int i = 0; i < n; i++) {
    float e = arr[i] - meanValue;
    sumSq += e * e;
  }

  return sqrt(sumSq / (n - 1));
}

// ======================= OUTLIER =======================

bool isOutlierByWindow(float x) {
  if (outlierCount < OUTLIER_WINDOW) {
    outlierBuf[outlierIdx] = x;
    outlierIdx = (outlierIdx + 1) % OUTLIER_WINDOW;
    outlierCount++;
    return false;
  }

  float meanValue = meanOfArray(outlierBuf, OUTLIER_WINDOW);
  float stdValue = stdOfArray(outlierBuf, OUTLIER_WINDOW, meanValue);

  bool outlier = false;

  if (!isnan(stdValue) && stdValue > 0.0001) {
    outlier = fabs(x - meanValue) > 3.0 * stdValue;
  }

  outlierBuf[outlierIdx] = x;
  outlierIdx = (outlierIdx + 1) % OUTLIER_WINDOW;

  return outlier;
}

int countOutliersSliding(const float arr[], int n) {
  int cnt = 0;
  if (n <= OUTLIER_WINDOW) return 0;

  for (int i = OUTLIER_WINDOW; i < n; i++) {
    float win[OUTLIER_WINDOW];

    for (int j = 0; j < OUTLIER_WINDOW; j++) {
      win[j] = arr[i - OUTLIER_WINDOW + j];
    }

    float meanValue = meanOfArray(win, OUTLIER_WINDOW);
    float stdValue = stdOfArray(win, OUTLIER_WINDOW, meanValue);

    if (!isnan(stdValue) && stdValue > 0.0001) {
      if (fabs(arr[i] - meanValue) > 3.0 * stdValue) {
        cnt++;
      }
    }
  }

  return cnt;
}

// ======================= TRACKING =======================

void printTrackingHeader() {
  Serial.println("Time_ms,Raw_cm,Median5_cm,Median9_cm,Kalman5_cm,Kalman9_cm,Avg5_cm,Velocity_Kalman5_cm_s,Velocity_Kalman9_cm_s,Valid,Outlier");
}

void sampleAndPrintTracking() {
  bool valid = false;
  float raw = measureDistanceRawCm(valid);
  unsigned long now = millis();

  if (!valid) {
    Serial.print(now);
    Serial.println(",nan,nan,nan,nan,nan,nan,nan,nan,0,1");
    return;
  }

  addToBuffers(raw);

  float med5 = medianOfArray(buf5, count5);
  float med9 = medianOfArray(buf9, count9);
  float avg5 = movingAverageOfArray(buf5, count5);

  float kal5 = kalman5.update(med5);
  float kal9 = kalman9.update(med9);

  float vel5 = NAN;
  float vel9 = NAN;

  if (!isnan(prevKalman5Dist) && !isnan(prevKalman9Dist) && prevTrackingTime > 0 && now > prevTrackingTime) {
    float dt = (now - prevTrackingTime) / 1000.0;
    vel5 = (kal5 - prevKalman5Dist) / dt;
    vel9 = (kal9 - prevKalman9Dist) / dt;
  }

  prevKalman5Dist = kal5;
  prevKalman9Dist = kal9;
  prevTrackingTime = now;

  bool outlier = isOutlierByWindow(raw);

  Serial.print(now);
  Serial.print(",");
  Serial.print(raw, 2);
  Serial.print(",");
  Serial.print(med5, 2);
  Serial.print(",");
  Serial.print(med9, 2);
  Serial.print(",");
  Serial.print(kal5, 2);
  Serial.print(",");
  Serial.print(kal9, 2);
  Serial.print(",");
  Serial.print(avg5, 2);
  Serial.print(",");

  if (isnan(vel5)) Serial.print("nan");
  else Serial.print(vel5, 1);

  Serial.print(",");

  if (isnan(vel9)) Serial.print("nan");
  else Serial.print(vel9, 1);

  Serial.print(",1,");
  Serial.println(outlier ? 1 : 0);
}

// ======================= STATIC 100 SAMPLES =======================

void collectStatic100Samples() {
  resetFilters();

  float rawArr[STATIC_SAMPLE_COUNT];
  float med5Arr[STATIC_SAMPLE_COUNT];
  float med9Arr[STATIC_SAMPLE_COUNT];
  float kal5Arr[STATIC_SAMPLE_COUNT];
  float kal9Arr[STATIC_SAMPLE_COUNT];

  int n = 0;

  Serial.println();
  Serial.println("START_STATIC_100_SAMPLES");
  Serial.println("Time_ms,Raw_cm,Median5_cm,Median9_cm,Kalman5_cm,Kalman9_cm,Valid");

  while (n < STATIC_SAMPLE_COUNT) {
    bool valid = false;
    float raw = measureDistanceRawCm(valid);
    unsigned long now = millis();

    if (valid) {
      addToBuffers(raw);

      float med5 = medianOfArray(buf5, count5);
      float med9 = medianOfArray(buf9, count9);
      float kal5 = kalman5.update(med5);
      float kal9 = kalman9.update(med9);

      rawArr[n] = raw;
      med5Arr[n] = med5;
      med9Arr[n] = med9;
      kal5Arr[n] = kal5;
      kal9Arr[n] = kal9;

      Serial.print(now);
      Serial.print(",");
      Serial.print(raw, 3);
      Serial.print(",");
      Serial.print(med5, 3);
      Serial.print(",");
      Serial.print(med9, 3);
      Serial.print(",");
      Serial.print(kal5, 3);
      Serial.print(",");
      Serial.print(kal9, 3);
      Serial.println(",1");

      n++;
    } else {
      Serial.print(now);
      Serial.println(",nan,nan,nan,nan,nan,0");
    }

    delay(SAMPLE_PERIOD_MS);
  }

  float meanRaw = meanOfArray(rawArr, n);
  float meanMed5 = meanOfArray(med5Arr, n);
  float meanMed9 = meanOfArray(med9Arr, n);
  float meanKal5 = meanOfArray(kal5Arr, n);
  float meanKal9 = meanOfArray(kal9Arr, n);

  float stdRaw = stdOfArray(rawArr, n, meanRaw);
  float stdMed5 = stdOfArray(med5Arr, n, meanMed5);
  float stdMed9 = stdOfArray(med9Arr, n, meanMed9);
  float stdKal5 = stdOfArray(kal5Arr, n, meanKal5);
  float stdKal9 = stdOfArray(kal9Arr, n, meanKal9);

  int outRaw = countOutliersSliding(rawArr, n);
  int outMed5 = countOutliersSliding(med5Arr, n);
  int outMed9 = countOutliersSliding(med9Arr, n);
  int outKal5 = countOutliersSliding(kal5Arr, n);
  int outKal9 = countOutliersSliding(kal9Arr, n);

  Serial.println("SUMMARY_STATIC_100_SAMPLES");
  Serial.println("Metric,Raw,Median_N5,Median_N9,Median_N5_Kalman,Median_N9_Kalman");

  Serial.print("Mean_cm,");
  Serial.print(meanRaw, 3);
  Serial.print(",");
  Serial.print(meanMed5, 3);
  Serial.print(",");
  Serial.print(meanMed9, 3);
  Serial.print(",");
  Serial.print(meanKal5, 3);
  Serial.print(",");
  Serial.println(meanKal9, 3);

  Serial.print("Noise_RMS_or_Std_cm,");
  Serial.print(stdRaw, 3);
  Serial.print(",");
  Serial.print(stdMed5, 3);
  Serial.print(",");
  Serial.print(stdMed9, 3);
  Serial.print(",");
  Serial.print(stdKal5, 3);
  Serial.print(",");
  Serial.println(stdKal9, 3);

  Serial.print("Outlier_per_100,");
  Serial.print(outRaw);
  Serial.print(",");
  Serial.print(outMed5);
  Serial.print(",");
  Serial.print(outMed9);
  Serial.print(",");
  Serial.print(outKal5);
  Serial.print(",");
  Serial.println(outKal9);

  Serial.println("END_STATIC_100_SAMPLES");
  Serial.println();
}
void handleSerialCommand() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  line.replace(",", ".");

  String upperLine = line;
  upperLine.toUpperCase();

  if (upperLine.length() == 0) return;

  if (upperLine == "HELP" || upperLine == "?") {
    printGuide();
  } else if (upperLine == "RUN") {
    resetFilters();
    runTracking = true;
    lastSampleTime = 0;
    printTrackingHeader();
  } else if (upperLine == "STOP") {
    runTracking = false;
    Serial.println("TRACKING_STOPPED");
  } else if (upperLine == "S100") {
    runTracking = false;
    collectStatic100Samples();
  } else if (upperLine.startsWith("T")) {
    String valuePart = line.substring(1);
    valuePart.trim();

    float t = valuePart.toFloat();

    if (t > -20.0 && t < 80.0) {
      TEMP_C = t;
      Serial.print("TEMP_C_SET,");
      Serial.println(TEMP_C, 2);
    } else {
      Serial.println("TEMP_INVALID");
    }
  } else if (upperLine.startsWith("KQ")) {
    String valuePart = line.substring(2);
    valuePart.trim();

    float q = valuePart.toFloat();

    if (q > 0.0 && q < 20.0) {
      KALMAN_Q = q;
      kalman5.setQR(KALMAN_Q, KALMAN_R);
      kalman9.setQR(KALMAN_Q, KALMAN_R);
      kalman5.reset();
      kalman9.reset();

      Serial.print("KALMAN_Q_SET,");
      Serial.println(KALMAN_Q, 4);
    } else {
      Serial.println("KALMAN_Q_INVALID");
    }
  } else if (upperLine.startsWith("KR")) {
    String valuePart = line.substring(2);
    valuePart.trim();

    float r = valuePart.toFloat();

    if (r > 0.0 && r < 100.0) {
      KALMAN_R = r;
      kalman5.setQR(KALMAN_Q, KALMAN_R);
      kalman9.setQR(KALMAN_Q, KALMAN_R);
      kalman5.reset();
      kalman9.reset();

      Serial.print("KALMAN_R_SET,");
      Serial.println(KALMAN_R, 4);
    } else {
      Serial.println("KALMAN_R_INVALID");
    }
  } else {
    Serial.println("UNKNOWN_COMMAND. Type HELP.");
  }
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

  if (runTracking) {
    unsigned long now = millis();

    if (lastSampleTime == 0 || now - lastSampleTime >= SAMPLE_PERIOD_MS) {
      lastSampleTime = now;
      sampleAndPrintTracking();
    }
  }
}
