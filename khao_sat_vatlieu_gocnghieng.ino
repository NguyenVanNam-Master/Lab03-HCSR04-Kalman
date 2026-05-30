

#include <math.h>

// ======================== CAU HINH CHAN ========================
const byte TRIG_PIN = 9;
const byte ECHO_PIN = 10;

// ======================== CAU HINH THI NGHIEM ========================
const int N_SAMPLES = 30;          // Moi vat lieu do 30 lan
const float D_REF_CM = 50.00;      // Khoang cach tham chieu
const float TEMP_C = 33.0;         // Nhiet do moi truong, co the sua thanh 28, 30...

// Gioi han hop le cho bai do tai d_ref = 50 cm
// Neu doc qua xa, co the la phan xa tu tuong/nen/vat khac nen tinh invalid.
const float MIN_VALID_CM = 2.0;
const float MAX_VALID_CM = 120.0;

// Timeout tinh theo micro giay.
// 9000 us tuong ung khoang 155 cm o 25 do C, du cho bai d_ref = 50 cm.
const unsigned long TIMEOUT_US = 9000;

// Khoang nghi giua hai lan do
const unsigned long SAMPLE_DELAY_MS = 250;

// So lan doc lam nong cam bien, khong tinh vao thong ke
const int WARMUP_READS = 5;

// ======================== DANH SACH VAT LIEU ========================
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

// ======================== BIEN LUU KET QUA ========================
float resultMean[N_CASES];
float resultSigma[N_CASES];
float resultError[N_CASES];
float resultValidRate[N_CASES];
int resultValidCount[N_CASES];

// ======================== HAM DO 1 LAN ========================
float measureDistanceCM() {
  // Tao xung trigger 10 us
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Do do rong xung Echo
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT_US);

  // Khong nhan duoc echo
  if (duration == 0) {
    return NAN;
  }

  // Toc do am thanh phu thuoc nhiet do:
  // v = 331.3 + 0.606*T (m/s)
  // Doi sang cm/us: v_cm_us = v * 100 / 1000000
  float speedSound_cm_us = (331.3 + 0.606 * TEMP_C) * 100.0 / 1000000.0;

  // Khoang cach = thoi gian di va ve * toc do / 2
  float distance = duration * speedSound_cm_us / 2.0;

  // Loc gia tri ngoai vung hop le
  if (distance < MIN_VALID_CM || distance > MAX_VALID_CM) {
    return NAN;
  }

  return distance;
}

// ======================== HAM IN SO HOAC N/A ========================
void printFloatOrNA(float value, int digits) {
  if (isnan(value)) {
    Serial.print("N/A");
  } else {
    Serial.print(value, digits);
  }
}

// ======================== HAM DOI NHAN ENTER ========================
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

// ======================== HAM DO 1 VAT LIEU ========================
void runOneCase(int index) {
  waitForEnter();

  // Doc lam nong, khong tinh
  for (int i = 0; i < WARMUP_READS; i++) {
    measureDistanceCM();
    delay(SAMPLE_DELAY_MS);
  }

  int validCount = 0;
  float sum = 0.0;
  float sumSq = 0.0;

  Serial.println();
  Serial.println("Lan do,Ket qua (cm),Trang thai");

  for (int i = 0; i < N_SAMPLES; i++) {
    float d = measureDistanceCM();

    Serial.print(i + 1);
    Serial.print(",");

    if (isnan(d)) {
      Serial.print("N/A");
      Serial.println(",INVALID");
    } else {
      validCount++;
      sum += d;
      sumSq += d * d;

      Serial.print(d, 2);
      Serial.println(",VALID");
    }

    delay(SAMPLE_DELAY_MS);
  }

  float mean = NAN;
  float sigma = NAN;
  float errorAbs = NAN;
  float validRate = 100.0 * validCount / N_SAMPLES;

  if (validCount > 0) {
    mean = sum / validCount;
    errorAbs = fabs(mean - D_REF_CM);
  }

  // Do lech chuan mau: chia cho n - 1
  if (validCount > 1) {
    float variance = (sumSq - (sum * sum / validCount)) / (validCount - 1);

    // Chong loi so hoc rat nho lam variance am
    if (variance < 0) {
      variance = 0;
    }

    sigma = sqrt(variance);
  }

  resultMean[index] = mean;
  resultSigma[index] = sigma;
  resultError[index] = errorAbs;
  resultValidRate[index] = validRate;
  resultValidCount[index] = validCount;

  Serial.println();
  Serial.println("----- KET QUA TOM TAT -----");
  Serial.print("Vat lieu: ");
  Serial.println(materialNames[index]);

  Serial.print("So lan hop le: ");
  Serial.print(validCount);
  Serial.print("/");
  Serial.println(N_SAMPLES);

  Serial.print("d_bar (cm): ");
  printFloatOrNA(mean, 2);
  Serial.println();

  Serial.print("sigma_d (cm): ");
  printFloatOrNA(sigma, 2);
  Serial.println();

  Serial.print("e_d (cm): ");
  printFloatOrNA(errorAbs, 2);
  Serial.println();

  Serial.print("Ti le doc hop le (%): ");
  Serial.print(validRate, 1);
  Serial.println("%");
}

// ======================== IN BANG CUOI CUNG ========================
void printFinalTable() {
  Serial.println();
  Serial.println();
  Serial.println("====================================================");
  Serial.println("BANG KET QUA CUOI CUNG - COPY SANG EXCEL/WORD");
  Serial.println("====================================================");

  Serial.println("Vat lieu,d_bar (cm),sigma_d (cm),e_d (cm),Ti le doc hop le (%),So lan hop le,Tong so lan do");

  for (int i = 0; i < N_CASES; i++) {
    Serial.print(materialNames[i]);
    Serial.print(",");

    printFloatOrNA(resultMean[i], 2);
    Serial.print(",");

    printFloatOrNA(resultSigma[i], 2);
    Serial.print(",");

    printFloatOrNA(resultError[i], 2);
    Serial.print(",");

    Serial.print(resultValidRate[i], 1);
    Serial.print(",");
    Serial.print(resultValidCount[i]);
    Serial.print(",");
    Serial.println(N_SAMPLES);
  }

  Serial.println("====================================================");
  Serial.println("Ghi chu:");
  Serial.print("d_ref = ");
  Serial.print(D_REF_CM, 2);
  Serial.println(" cm");

  Serial.println("d_bar la gia tri trung binh cua cac lan do hop le.");
  Serial.println("sigma_d la do lech chuan mau cua cac lan do hop le.");
  Serial.println("e_d = |d_bar - d_ref|.");
  Serial.println("Ti le doc hop le = so lan VALID / 30 * 100%.");
}

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  digitalWrite(TRIG_PIN, LOW);

  delay(1000);

  for (int i = 0; i < N_CASES; i++) {
    resultMean[i] = NAN;
    resultSigma[i] = NAN;
    resultError[i] = NAN;
    resultValidRate[i] = 0.0;
    resultValidCount[i] = 0;
  }

  for (int i = 0; i < N_CASES; i++) {
    runOneCase(i);
  }

  printFinalTable();

  Serial.println();
  Serial.println("Da hoan thanh toan bo thi nghiem.");
}

// ======================== LOOP ========================
void loop() {
  // Khong lam gi sau khi do xong
}