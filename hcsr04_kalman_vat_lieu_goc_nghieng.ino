

#include <math.h>

// ===================== CAU HINH CHAN =====================
const byte TRIG_PIN = 9;
const byte ECHO_PIN = 10;

// ===================== CAU HINH THI NGHIEM =====================
const int N_SAMPLES = 30;
const float D_REF_CM = 50.00;
const float TEMP_C = 25.0;

// Gioi han hop le cho thi nghiem dat vat can tai 50 cm
const float MIN_VALID_CM = 2.0;
const float MAX_VALID_CM = 120.0;

// Timeout cho pulseIn.
// 9000 us du cho khoang 150 cm, phu hop bai do 50 cm.
const unsigned long TIMEOUT_US = 9000;
const unsigned long SAMPLE_DELAY_MS = 250;
const int WARMUP_READS = 5;

// ===================== THAM SO KALMAN =====================
// Mo hinh: khoang cach that gan nhu khong doi trong tung lan do vat lieu.
// Q nho vi vat can dung yen.
// R la phuong sai nhieu do uoc luong cua HC-SR04.
const float KALMAN_Q = 0.02;   // process noise, don vi cm^2
const float KALMAN_R = 1.00;   // measurement noise, don vi cm^2
const float KALMAN_P0 = 4.00;  // sai so uoc luong ban dau

// Neu sau loc van muon phan ung nhanh hon: tang Q hoac giam R.
// Neu sau loc van nhieu: giam Q hoac tang R.

// ===================== DANH SACH MAU THI NGHIEM =====================
const int N_CASES = 8;

const char* materialNames[N_CASES] = {
  "Go phang tham chieu",
  "Bia cung A4",
  "Kim loai phang",
  "Xop mem foam",
  "Vai ni khan",
  "Bia nghieng 15 do",
  "Bia nghieng 30 do",
  "Bia nghieng 45 do"
};

// ===================== LOP KALMAN 1 CHIEU =====================
class Kalman1D {
  public:
    float x;       // gia tri uoc luong
    float p;       // sai so uoc luong
    float q;       // nhieu qua trinh
    float r;       // nhieu do
    bool initialized;

    Kalman1D(float q_in, float r_in, float p0_in) {
      q = q_in;
      r = r_in;
      p = p0_in;
      x = 0.0;
      initialized = false;
    }

    void reset() {
      x = 0.0;
      p = KALMAN_P0;
      initialized = false;
    }

    float update(float z) {
      // Khoi tao bang gia tri do hop le dau tien de tranh lech transients
      if (!initialized) {
        x = z;
        p = KALMAN_P0;
        initialized = true;
        return x;
      }

      // 1. Predict
      // Mo hinh vat can dung yen: x_pred = x
      p = p + q;

      // 2. Update
      float k = p / (p + r);   // Kalman gain
      x = x + k * (z - x);
      p = (1.0 - k) * p;

      return x;
    }
};

// ===================== BIEN LUU KET QUA =====================
float rawMean[N_CASES];
float rawSigma[N_CASES];
float rawError[N_CASES];

float kalmanMean[N_CASES];
float kalmanSigma[N_CASES];
float kalmanError[N_CASES];

float validRate[N_CASES];
int validCountArr[N_CASES];

// ===================== HAM DO 1 LAN =====================
float measureDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);

  if (duration == 0) {
    return NAN;
  }

  // v = 331.3 + 0.606*T, don vi m/s
  float speedSound_cm_us = (331.3 + 0.606 * TEMP_C) * 100.0 / 1000000.0;
  float distance = duration * speedSound_cm_us / 2.0;

  if (distance < MIN_VALID_CM || distance > MAX_VALID_CM) {
    return NAN;
  }

  return distance;
}

// ===================== HAM IN SO HOAC N/A =====================
void printFloatOrNA(float value, int digits) {
  if (isnan(value)) {
    Serial.print("N/A");
  } else {
    Serial.print(value, digits);
  }
}

// ===================== HAM DOI ENTER =====================
void waitForEnter() {
  while (Serial.available()) {
    Serial.read();
  }

  while (!Serial.available()) {
    delay(10);
  }

  while (Serial.available()) {
    Serial.read();
  }
}

// ===================== TINH TRUNG BINH VA DO LECH CHUAN =====================
void calcStats(float values[], int count, float &meanOut, float &sigmaOut) {
  meanOut = NAN;
  sigmaOut = NAN;

  if (count <= 0) {
    return;
  }

  float sum = 0.0;
  float sumSq = 0.0;

  for (int i = 0; i < count; i++) {
    sum += values[i];
    sumSq += values[i] * values[i];
  }

  meanOut = sum / count;

  if (count > 1) {
    float variance = (sumSq - (sum * sum / count)) / (count - 1);

    if (variance < 0) {
      variance = 0;
    }

    sigmaOut = sqrt(variance);
  }
}

// ===================== DO 1 VAT LIEU =====================
void runOneCase(int index) {
  
  waitForEnter();

  for (int i = 0; i < WARMUP_READS; i++) {
    measureDistanceCM();
    delay(SAMPLE_DELAY_MS);
  }

  Kalman1D kf(KALMAN_Q, KALMAN_R, KALMAN_P0);
  kf.reset();

  float rawValues[N_SAMPLES];
  float kalmanValues[N_SAMPLES];

  int validCount = 0;
  int kalmanCount = 0;

  Serial.println();
  Serial.println("Lan do,Raw_cm,Kalman_cm,Trang thai");

  for (int i = 0; i < N_SAMPLES; i++) {
    float raw = measureDistanceCM();

    Serial.print(i + 1);
    Serial.print(",");

    if (isnan(raw)) {
      Serial.print("N/A,N/A,INVALID");
      Serial.println();

      // Khong update Kalman bang du lieu INVALID
    } else {
      float kalman = kf.update(raw);

      rawValues[validCount] = raw;
      validCount++;

      kalmanValues[kalmanCount] = kalman;
      kalmanCount++;

      Serial.print(raw, 2);
      Serial.print(",");
      Serial.print(kalman, 2);
      Serial.println(",VALID");
    }

    delay(SAMPLE_DELAY_MS);
  }

  float rMean, rSigma;
  float kMean, kSigma;

  calcStats(rawValues, validCount, rMean, rSigma);
  calcStats(kalmanValues, kalmanCount, kMean, kSigma);

  rawMean[index] = rMean;
  rawSigma[index] = rSigma;
  rawError[index] = isnan(rMean) ? NAN : fabs(rMean - D_REF_CM);

  kalmanMean[index] = kMean;
  kalmanSigma[index] = kSigma;
  kalmanError[index] = isnan(kMean) ? NAN : fabs(kMean - D_REF_CM);

  validRate[index] = 100.0 * validCount / N_SAMPLES;
  validCountArr[index] = validCount;

  Serial.println();
  Serial.println("----- TOM TAT TRUOC/SAU KALMAN -----");
  Serial.print("Vat lieu: ");
  Serial.println(materialNames[index]);

  Serial.print("So lan hop le: ");
  Serial.print(validCount);
  Serial.print("/");
  Serial.println(N_SAMPLES);

  Serial.print("Raw d_bar (cm): ");
  printFloatOrNA(rawMean[index], 2);
  Serial.println();

  Serial.print("Raw sigma_d (cm): ");
  printFloatOrNA(rawSigma[index], 2);
  Serial.println();

  Serial.print("Raw e_d (cm): ");
  printFloatOrNA(rawError[index], 2);
  Serial.println();

  Serial.print("Kalman d_bar (cm): ");
  printFloatOrNA(kalmanMean[index], 2);
  Serial.println();

  Serial.print("Kalman sigma_d (cm): ");
  printFloatOrNA(kalmanSigma[index], 2);
  Serial.println();

  Serial.print("Kalman e_d (cm): ");
  printFloatOrNA(kalmanError[index], 2);
  Serial.println();

  Serial.print("Ti le doc hop le (%): ");
  Serial.print(validRate[index], 1);
  Serial.println("%");
}

// ===================== IN BANG CSV CUOI =====================
void printFinalTable() {
  Serial.println();
  Serial.println("====================================================");
  Serial.println("BANG KET QUA CUOI CUNG - COPY SANG EXCEL");
  Serial.println("====================================================");

  Serial.println("Vat lieu,d_raw_bar (cm),sigma_raw (cm),e_raw (cm),d_kalman_bar (cm),sigma_kalman (cm),e_kalman (cm),Valid (%),So lan hop le,Tong so lan");

  for (int i = 0; i < N_CASES; i++) {
    Serial.print(materialNames[i]);
    Serial.print(",");

    printFloatOrNA(rawMean[i], 2);
    Serial.print(",");

    printFloatOrNA(rawSigma[i], 2);
    Serial.print(",");

    printFloatOrNA(rawError[i], 2);
    Serial.print(",");

    printFloatOrNA(kalmanMean[i], 2);
    Serial.print(",");

    printFloatOrNA(kalmanSigma[i], 2);
    Serial.print(",");

    printFloatOrNA(kalmanError[i], 2);
    Serial.print(",");

    Serial.print(validRate[i], 1);
    Serial.print(",");
    Serial.print(validCountArr[i]);
    Serial.print(",");
    Serial.println(N_SAMPLES);
  }

  Serial.println("====================================================");
  Serial.println("Ghi chu:");
  Serial.println("Kalman chi loc cac mau VALID. Mau INVALID khong duoc bien thanh du lieu do that.");
  Serial.println("Neu Valid = 0% thi d_raw va d_kalman deu la N/A.");
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  for (int i = 0; i < N_CASES; i++) {
    rawMean[i] = NAN;
    rawSigma[i] = NAN;
    rawError[i] = NAN;

    kalmanMean[i] = NAN;
    kalmanSigma[i] = NAN;
    kalmanError[i] = NAN;

    validRate[i] = 0.0;
    validCountArr[i] = 0;
  }

  delay(1000);

  Serial.println();
  Serial.println("==============================================");
  Serial.println("THI NGHIEM HC-SR04 CO BO LOC KALMAN");
  Serial.println("==============================================");
  Serial.println("Serial Monitor:");
  Serial.println("- Baud rate: 115200");
  Serial.println("- Line ending: Newline hoac Both NL & CR");
  Serial.println();
  Serial.print("d_ref = ");
  Serial.print(D_REF_CM, 2);
  Serial.println(" cm");

  Serial.print("Nhiet do bu toc do am = ");
  Serial.print(TEMP_C, 1);
  Serial.println(" do C");

  Serial.print("So lan do moi truong hop = ");
  Serial.println(N_SAMPLES);

  Serial.print("Kalman Q = ");
  Serial.print(KALMAN_Q, 4);
  Serial.print(", R = ");
  Serial.print(KALMAN_R, 4);
  Serial.print(", P0 = ");
  Serial.println(KALMAN_P0, 4);

  Serial.println();
  Serial.println("Nhan ENTER de bat dau.");
  waitForEnter();

  for (int i = 0; i < N_CASES; i++) {
    runOneCase(i);
  }

  printFinalTable();

  Serial.println();
  Serial.println("Hoan thanh thi nghiem.");
}

// ===================== LOOP =====================
void loop() {
  // Khong lam gi sau khi do xong.
}
