#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
	Serial.begin(115200);

	// Try to initialize!
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
		  delay(10);
		}
	}
	Serial.println("MPU6050 Found!");

	// set accelerometer range to +-8G
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

	// set gyro range to +- 500 deg/s
	mpu.setGyroRange(MPU6050_RANGE_500_DEG);

	// set filter bandwidth to 21 Hz
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

	delay(100);
}

void loop() {
	static unsigned long lastTime = 0; // Para armazenar o tempo da última leitura
	unsigned long currentTime = millis(); // Tempo atual em milissegundos
	unsigned long deltaTime = (currentTime - lastTime); // Delta t em segundos
	lastTime = currentTime;

	// Obter novos eventos do sensor
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);

	// Imprimir tudo em uma linha
	Serial.print(deltaTime);
	Serial.println(", ");

	Serial.print(a.acceleration.x);
	Serial.print(",");
	Serial.print(a.acceleration.y);
	Serial.print(",");
	Serial.print(a.acceleration.z);
	Serial.print(",");

	Serial.print(g.gyro.x);
	Serial.print(",");
	Serial.print(g.gyro.y);
	Serial.print(",");
	Serial.print(g.gyro.z);
	Serial.print(",");

	Serial.print(temp.temperature);

	
}
