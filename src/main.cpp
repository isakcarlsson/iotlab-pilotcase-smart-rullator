#include "M5Atom.h"
#include <math.h>

// sudo chmod a+rw /dev/ttyUSB0

float acc0[] = {0.0f, 0.0f, 0.0f};
float acc[] = {0.0f, 0.0f, 0.0f};
float prevAcc[] = {0.0f, 0.0f, 0.0f};

unsigned long lastTime = 0;
int vectorCounter = 0;
float vectors[] = {0.0f, 0.0f, 0.0f};

int accelerations = 0;
int steps = 0;

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

void setup(){
  M5.begin(true, false, true);
  delay(100);
  M5.IMU.Init();
  delay(100);
  M5.IMU.SetAccelFsr(M5.IMU.AFS_16G);
  delay(100);

  M5.IMU.getAccelData(&acc[0], &acc[1], &acc[2]);
  acc0[0] = acc[0];
  acc0[1] = acc[1];
  acc0[2] = acc[2]; 

  lastTime = millis();
}

void loop() {
  M5.IMU.getAccelData(&acc[0], &acc[1], &acc[2]);
  acc[0] = acc[0] - acc0[0];
  acc[1] = acc[1] - acc0[1];
  acc[2] = acc[2] - acc0[2];
  
  if (magnitude(acc, 3) > 0.022) {
    vectors[0] = acc[0];
    vectors[1] = acc[1];
    vectors[2] = acc[2];
    vectorCounter++;

    if (millis() - lastTime > 100) {
      
      vectors[0] = vectors[0] / vectorCounter;
      vectors[1] = vectors[1] / vectorCounter;
      vectors[2] = vectors[2] / vectorCounter;

      if (magnitude(prevAcc, 3) == 0.0f) {
        prevAcc[0] = vectors[0];
        prevAcc[1] = vectors[1];
        prevAcc[2] = vectors[2];    
      }

      float projection[3] = {0.0f, 0.0f, 0.0f};
      projectVector(prevAcc, acc, 3, projection);

      unsigned int angle = round(angleBetweenVectors(acc, prevAcc, 3));
      float projectionMag = magnitude(projection, 3);
      
      if (angle > 90 && projectionMag > 0.014) {
        prevAcc[0] = acc[0];
        prevAcc[1] = acc[1];
        prevAcc[2] = acc[2];   

        accelerations++;
        steps = accelerations / 2;

        if (accelerations % 2 == 0) {
          Serial.printf("%u steps\n", steps);
        }
      
        delay(200);            
      }

      vectorCounter = 0;
      lastTime = millis();
    }
  }
}
