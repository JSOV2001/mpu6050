#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;
float x_gyro, y_gyro, z_gyro;
float x_accel, y_accel, z_accel;
float x_gyro_init, y_gyro_init, z_gyro_init;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float roll_accel, pitch_accel;
float dt;
long dt_prev;
float roll_curr, pitch_curr, yaw_curr;
float roll_prev, pitch_prev, yaw_prev;

void setup(){
  Serial.begin(115200);

   // Initialize MPU6050
  while(!Serial) {}
  Wire.begin();
  mpu.initialize();
  
  // Calibrate MPU6050's gyroscope
  calibrate_gyroscope();

  bool is_mpu_allowed;
  is_mpu_allowed = mpu.testConnection();
  
  if(is_mpu_allowed){
    Serial.println("MPU6050's Initialization Succeeded !");
  }
  else{
    Serial.println("MPU6050's Initialization Failed!");
  }
  digitalWrite(13, HIGH);
}

void loop(){
  read_mpu6050();

  Serial.print("\nCurrent RPY: ");
  Serial.print("\nRoll: "); 
  Serial.print(roll_curr); 
  Serial.print(" °"); 
  Serial.print("\nPitch: "); 
  Serial.print(pitch_curr); 
  Serial.print(" °"); 
  Serial.print("\nYaw: "); 
  Serial.print(yaw_curr); 
  Serial.print(" °\n"); 

  delay(250);
}

void calibrate_gyroscope(){
  Serial.println("\nStart MPU6050's calibration"); 
  Serial.println("Please, keep it resting\n");
  const int samples = 1000; // Número de lecturas para la calibración
  float x_gyro_accu = 0.0, y_gyro_accu = 0.0, z_gyro_accu = 0.0;
  
  for (int i = 0; i < samples; ++i) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 25) {
      // Compute gyroscope
      mpu.getRotation(&gx, &gy, &gz);
      x_gyro = gx * (250.0 / 32768.0);
      y_gyro = gy * (250.0 / 32768.0);
      z_gyro = gz * (250.0 / 32768.0);
      prev_ms = millis();
    }

    //Acumulador datos giroscopio
    x_gyro_accu += x_gyro;
    y_gyro_accu += y_gyro;
    z_gyro_accu += z_gyro;
    
    // Intérvalo de muestreo
    delay(5); 
  }

  //Estimación inicial (Bias o sesgo) - CALIBRACIÓN DEL GIROSCOPIO
  x_gyro_init = x_gyro_accu / samples;
  y_gyro_init = y_gyro_accu / samples;
  z_gyro_init = z_gyro_accu / samples;
  Serial.print("\nOffsets:");
  Serial.println("Gyro in X = "); 
  Serial.print(x_gyro_init);
  Serial.println("Gyro in Y = "); 
  Serial.print(y_gyro_init);
  Serial.println("Gyro in Z = "); 
  Serial.print(z_gyro_init);
}

void read_mpu6050(){
  dt = millis() - dt_prev;
  dt = dt/1000.0;
  dt_prev = millis();

  // Compute gyroscope
  mpu.getRotation(&gx, &gy, &gz);
  x_gyro = gx * (250.0 / 32768.0);
  y_gyro = gy * (250.0 / 32768.0);
  z_gyro = gz * (250.0 / 32768.0);

  // Adjust gyroscope's signals
  x_gyro -= x_gyro_init;
  y_gyro -= y_gyro_init;
  z_gyro -= z_gyro_init;
  
  // Compute accelerometer
  mpu.getAcceleration(&ax, &ay, &az);
  x_accel = ax * (9.81 / 16384.0);
  y_accel = ay * (9.81 / 16384.0);
  z_accel = az * (9.81 / 16384.0);

   // Compute angle with measured accelerations
  roll_accel = atan2(y_accel, sqrt(pow(x_accel,2) + pow(z_accel,2))) * (180.0/3.14);
  pitch_accel = atan2(-x_accel, sqrt(pow(y_accel,2) + pow(z_accel,2))) * (180.0/3.14);

  // Compute angle with gyroscope's signals and add complementary filter
  roll_curr = (0.9 * ((x_gyro*dt) + roll_prev)) + (0.1 * roll_accel);
  pitch_curr = (0.9 * ((y_gyro*dt) + pitch_prev)) + (0.1 * pitch_accel);
  yaw_curr = (z_gyro * dt) + yaw_prev;

  roll_prev = roll_curr;
  pitch_prev = pitch_curr;
  yaw_prev = yaw_curr;
}