# Project_Luky_TA
Project_Suhu_MLX90614

#include <Wire.h>
#include <Adafruit_MLX90614.h>

// Inisialisasi sensor MLX90614
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  Serial.begin(9600); // Inisialisasi komunikasi serial
  if (!mlx.begin()) {
    Serial.println("Sensor MLX90614 tidak ditemukan. Cek koneksi!");
    while (1);
  }
  Serial.println("Sensor MLX90614 siap digunakan.");
}

void loop() {
  // Baca suhu objek dan suhu ambien
  float suhuObjek = mlx.readObjectTempC();
  float suhuAmbien = mlx.readAmbientTempC();

  // Cek apakah suhu objek dalam rentang 35-36°C
  if (suhuObjek >= 35.0 && suhuObjek <= 36.0) {
    Serial.print("Suhu Objek: ");
    Serial.print(suhuObjek);
    Serial.println(" °C");
    Serial.print("Suhu Ambien: ");
    Serial.print(suhuAmbien);
    Serial.println(" °C");
    Serial.println("--------------------------");
  } else {
    Serial.println("Suhu di luar rentang (35-36°C).");
  }

  delay(1000); // Tunggu 1 detik sebelum pembacaan berikutnya
}
