// Gluvn test-rig logger: YD-ESP32-S3, GY-521 (MPU-6050 at 0x68), GY-271 (QMC5883L clone at 0x0D).
// Prints a 50 Hz CSV line: t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,fsr,mark
// Accel in m/s^2, gyro in rad/s, mag in raw counts. Bias removal and calibration happen offline (../analysis).
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>

Adafruit_MPU6050 mpu;
QMC5883LCompass compass;
bool mpuOK = false;

void startMPU() {
  mpuOK = mpu.begin(0x68, &Wire);
  if (mpuOK) {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  }
}

void setup() {
  Serial0.begin(115200);
  Wire.begin(8, 9);            // SDA = 8, SCL = 9
  Wire.setClock(400000);
  startMPU();
  compass.init();
  Serial0.println(mpuOK ? "# MPU OK" : "# MPU NOT FOUND");
  Serial0.println("t_ms,ax,ay,az,gx,gy,gz,mx,my,mz,fsr,mark");
}

void loop() {
  static uint32_t next = 0;
  if (millis() < next) return;
  next = millis() + 20;        // 50 Hz

  sensors_event_t a, g, t;
  bool ok = mpuOK && mpu.getEvent(&a, &g, &t);
  if (!ok || (a.acceleration.x == 0 && a.acceleration.y == 0 && a.acceleration.z == 0)) {
    startMPU();                // re-init if the IMU dropped out
    return;
  }
  compass.read();
  int fsr = analogRead(5);
  int mark = 0;
  if (Serial0.available() && Serial0.read() == 'm') mark = 1;
  Serial0.printf("%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d\n",
                 millis(), a.acceleration.x, a.acceleration.y, a.acceleration.z,
                 g.gyro.x, g.gyro.y, g.gyro.z,
                 compass.getX(), compass.getY(), compass.getZ(), fsr, mark);
}
