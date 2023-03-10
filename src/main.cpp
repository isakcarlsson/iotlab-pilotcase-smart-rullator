#include "M5Atom.h"
#include <math.h>
#include "lorasend.h"
#include "string.h"

#define HALL 32
#define WHEEL_SIZE 62

int distance = 0;
int test_distance = 990;

// sudo chmod a+rw /dev/ttyUSB0

float acc0[] = {0.0f, 0.0f, 0.0f};
float acc[] = {0.0f, 0.0f, 0.0f};
float prevAcc[] = {0.0f, 0.0f, 0.0f};

int accelerations = 0;
int steps = 0;

int hasStarted = 0;
int hasStopped = 0;
unsigned long startTime = 0;
int time_ = 0;

float dotProduct(float A[], float B[], int n) {
  float result = 0.0;
  for (int i = 0; i < n; i++) {
    result += A[i] * B[i];
  }
  return result;
}

float magnitude(float A[], int n) {
  float result = 0.0;
  for (int i = 0; i < n; i++) {
    result += A[i] * A[i];
  }
  return sqrt(result);
}

float angleBetweenVectors(float A[], float B[], int n) {
  float dot = dotProduct(A, B, n);
  float magA = magnitude(A, n);
  float magB = magnitude(B, n);
  float cosTheta = dot / (magA * magB);
  return acos(cosTheta) * 180.0 / PI;
}

void projectVector(float A[], float B[], int n, float C[]) {
  float dot = dotProduct(A, B, n);
  float magB = magnitude(B, n);
  float scale = dot / (magB * magB);
  for (int i = 0; i < n; i++) {
    C[i] = scale * B[i];
  }
}


void send () {
  M5.dis.fillpix(0x00ff00);
  Serial.printf("Sending data: Time: %i Steps: %i\n", time_, steps);

  char steps_bytes[5];
  sprintf(steps_bytes, "%04x", steps);

  char time_bytes[9];
  sprintf(time_bytes, "%08x", time_);

  char distance_bytes[5];
  sprintf(distance_bytes, "%04x", distance);

  String bytes = String(steps_bytes) + String(time_bytes) + String(distance_bytes);
  Serial.print(bytes + "\n");

  sendData(bytes);

  M5.dis.fillpix(0x000000);
  delay(100);
  esp_deep_sleep_start();
}

void step_counter(void * pvParameters) {

   while (distance < test_distance) {
    // Get acceleration data
    M5.IMU.getAccelData(&acc[0], &acc[1], &acc[2]);
    acc[0] = acc[0] - acc0[0];
    acc[1] = acc[1] - acc0[1];
    acc[2] = acc[2] - acc0[2];
    
    // Check if acceleration is above threshold
    if (magnitude(acc, 3) > 0.022) {

      // Check if previous acceleration is zero
      if (magnitude(prevAcc, 3) == 0.0f) {
        prevAcc[0] = acc[0];
        prevAcc[1] = acc[1];
        prevAcc[2] = acc[2];    
      }

      float projection[3] = {0.0f, 0.0f, 0.0f};
      projectVector(prevAcc, acc, 3, projection);

      unsigned int angle = round(angleBetweenVectors(acc, prevAcc, 3));
      float projectionMag = magnitude(projection, 3);
      
      // Check if angle is greater than 90 degrees and projection is above threshold
      if (angle > 90 && projectionMag > 0.012) {
        prevAcc[0] = acc[0];
        prevAcc[1] = acc[1];
        prevAcc[2] = acc[2];   

        accelerations++;

        if (accelerations % 2 == 0) {
          steps = accelerations / 2;
          Serial.printf("%u steps\n", steps);
        }
      
        vTaskDelay(pdMS_TO_TICKS(200));
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  vTaskDelete(NULL);
}


void hall(void * pvParameters) {

  while (distance < test_distance) {
    int value = digitalRead(HALL);

    if (value == 0) {
      if (!hasStarted) {
        hasStarted = 1;
        startTime = millis();
        xTaskCreatePinnedToCore(step_counter, "step_counter", 4096, NULL, 6, NULL, 0);
      } else {
        distance += WHEEL_SIZE;
      }

      while (digitalRead(HALL) == 0) {
          vTaskDelay(pdMS_TO_TICKS(50));
      }
    }
  }

  time_ = millis() - startTime;
  hasStopped = 1;

  vTaskDelete(NULL);
}


void setup() {
  M5.begin(true,false,true);
  delay(100);
  M5.dis.fillpix(0xff0000);
  M5.IMU.Init();
  delay(100);
  M5.IMU.SetAccelFsr(M5.IMU.AFS_16G);
  delay(100);
  pinMode(HALL, INPUT);

  // Get initial acceration to calibrate accelerometer
  M5.IMU.getAccelData(&acc[0], &acc[1], &acc[2]);
  acc0[0] = acc[0];
  acc0[1] = acc[1];
  acc0[2] = acc[2]; 

  M5.dis.fillpix(0xffff00);
  delay(100);

  xTaskCreatePinnedToCore(hall, "hall", 4096, NULL, 5, NULL, 1);

  while (!hasStopped) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  send();
}

void loop() {}

