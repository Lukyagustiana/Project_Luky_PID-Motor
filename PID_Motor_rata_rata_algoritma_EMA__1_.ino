// Definisikan pin
const int encoderPinA = 2;  // Pin untuk channel A encoder
const int motorPWMPin = 9;   // Pin PWM untuk kontrol kecepatan motor

// Konstanta untuk penghitungan RPM
const int pulsesPerRevolution = 20;  // Ganti dengan jumlah pulsa per rotasi dari encoder
const unsigned long samplingInterval = 100; // Interval pengukuran dalam ms

// Variabel untuk menghitung pulsa encoder
volatile long encoderCount = 0; // Jumlah pulsa dari encoder
unsigned long lastTime = 0;     // Waktu terakhir untuk interval penghitungan

// Variabel untuk pengaturan PID
double setPoint = 0;            // Nilai target RPM awal
double input = 0;               // Nilai RPM aktual
double output = 0;              // Nilai output PID (nilai PWM)

// Parameter PID
double kp = 1.0, ki = 0.1, kd = 0.01;
double previousError = 0, integral = 0;

// Variabel untuk rise time dan overshoot
unsigned long startTime = 0;
unsigned long timeAt10Percent = 0;  // Waktu saat mencapai 10%
unsigned long timeAt90Percent = 0;  // Waktu saat mencapai 90%
bool reached10Percent = false;
bool reached90Percent = false;
double riseTime = 0;
double overshoot = 0; // Overshoot adalah nilai tertinggi yang tercatat - setPoint

// Variabel untuk exponential moving average
double alpha = 0.01;  // Faktor smoothing untuk EMA (antara 0 dan 1)
double emaRPM = 0;    // Variabel untuk menyimpan nilai EMA RPM

void setup() {
  pinMode(encoderPinA, INPUT);
  pinMode(motorPWMPin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), countEncoder, RISING);

  Serial.begin(9600);
  Serial.println("Masukkan nilai setpoint RPM:");
}

void loop() {
  // Periksa apakah ada input baru di Serial Monitor
  if (Serial.available() > 0) {
    String inputString = Serial.readStringUntil('\n');
    
    if (inputString.startsWith("Kp:")) {
      kp = inputString.substring(3).toDouble();
      Serial.print("Kp baru: ");
      Serial.println(kp);
    } else if (inputString.startsWith("Ki:")) {
      ki = inputString.substring(3).toDouble();
      Serial.print("Ki baru: ");
      Serial.println(ki);
    } else if (inputString.startsWith("Kd:")) {
      kd = inputString.substring(3).toDouble();
      Serial.print("Kd baru: ");
      Serial.println(kd);
    } else if (inputString.startsWith("alpha:")) {
      alpha = inputString.substring(6).toDouble();
      if (alpha < 0) alpha = 0;       // Pastikan alpha tidak kurang dari 0
      if (alpha > 1) alpha = 1;       // Pastikan alpha tidak lebih dari 1
      Serial.print("Alpha baru: ");
      Serial.println(alpha);
    } else if (inputString.equals("RESET")) {
      resetArduino();
    } else {
      setPoint = inputString.toDouble();  // Konversi input menjadi angka
      Serial.print("Setpoint RPM baru: ");
      Serial.println(setPoint);
      
      // Reset variabel untuk pengukuran rise time dan overshoot
      reached10Percent = false;
      reached90Percent = false;
      riseTime = 0;
      overshoot = 0; // Reset overshoot saat setpoint berubah
      startTime = 0; // Reset start time
    }
  }

  unsigned long currentTime = millis();
  
  // Hitung RPM setiap 100 ms
  if (currentTime - lastTime >= samplingInterval) { // Interval yang ditentukan
    lastTime = currentTime;
    
    // Hitung RPM
    input = (encoderCount * 60.0 / pulsesPerRevolution) * (1000.0 / samplingInterval); // Konversi ke RPM
    encoderCount = 0; // Reset hitungan

    // Menghitung EMA
    emaRPM = alpha * input + (1 - alpha) * emaRPM;

    // PID Controller
    double error = setPoint - emaRPM;  // Gunakan nilai EMA untuk kontrol
    integral += error * (samplingInterval / 1000.0); // Konversi ms ke detik
    double derivative = (error - previousError) / (samplingInterval / 1000.0);
    
    output = kp * error + ki * integral + kd * derivative;
    previousError = error;

    // Batasan output agar sesuai dengan rentang PWM (0-255)
    if (output > 255) output = 255;
    if (output < 0) output = 0;

    // Atur kecepatan motor menggunakan PWM
    analogWrite(motorPWMPin, output);

    // Perhitungan rise time berdasarkan EMA RPM
    double tenPercent = setPoint * 0.1;  // 10% dari setpoint
    double ninetyPercent = setPoint * 0.9; // 90% dari setpoint

    // Mengecek apakah emaRPM sudah mencapai 10%
    if (!reached10Percent && emaRPM >= tenPercent) {
      timeAt10Percent = millis(); // Menyimpan waktu saat mencapai 10%
      reached10Percent = true; // Menandai bahwa 10% sudah tercapai
      Serial.println(" ");
      Serial.println("RPM mencapai 10% dari setpoint.");
    }
    
    // Jika sudah mencapai 10% tapi belum mencapai 90%, terus update waktu
    if (reached10Percent && !reached90Percent) {
      if (emaRPM >= ninetyPercent) {
        timeAt90Percent = millis(); // Menyimpan waktu saat mencapai 90%
        riseTime = (timeAt90Percent - timeAt10Percent) / 1000.0; // Menghitung rise time dalam detik
        Serial.println(" ");
        Serial.println("RPM mencapai 90% dari setpoint.");
        reached90Percent = true; // Menandai bahwa 90% sudah tercapai
      }
    }
    
    // Perhitungan akumulasi overshoot
    if (emaRPM > setPoint) {
      double currentOvershoot = emaRPM - setPoint; // Hitung overshoot saat ini
      if (currentOvershoot > overshoot) {
        overshoot = currentOvershoot;  // Perbarui overshoot jika lebih tinggi dari sebelumnya
      }
    }

    // Cetak setpoint, nilai RPM, Output PID, rise time, overshoot, Kp, Ki, Kd, dan alpha ke Serial Monitor
    Serial.print("Setpoint RPM: ");
    Serial.print(setPoint);
    Serial.print(" | RPM: ");
    Serial.print(emaRPM);
    Serial.print(" | PID Output: ");
    Serial.print(output);
    Serial.print(" | Rise Time: ");
    Serial.print(riseTime);
    Serial.print(" | Overshoot: ");
    Serial.print(overshoot);
    Serial.print(" | Kp: ");
    Serial.print(kp);
    Serial.print(" | Ki: ");
    Serial.print(ki);
    Serial.print(" | Kd: ");
    Serial.print(kd);
    Serial.print(" | Alpha: ");
    Serial.print(alpha); // Menampilkan nilai alpha di Serial Monitor
    Serial.print(" | Error: ");  // Tambahkan pembacaan error
    Serial.println(error);
  }
}

// ISR untuk menghitung pulsa encoder
void countEncoder() {
  encoderCount++;
}

// Fungsi untuk mereset Arduino
void resetArduino() {
  asm volatile ("  jmp 0");
}
