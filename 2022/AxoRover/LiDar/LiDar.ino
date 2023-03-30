#include <Servo.h>
#include <SparkFun_VL53L1X.h>

SFEVL53L1X distanceSensor;

float distance;
int i=1;
String LiDar[46];

Servo LiDarS;


void setup() {
  LiDarS.attach(11);
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Starting Distance Sensor");
  distanceSensor.begin();
  LiDarS.write(45);

}

void loop() {
  for(i=0; i<=45; i++){
    LiDarS.write(map(i,0,45,0,180));
    distanceSensor.startRanging();
    while (!distanceSensor.checkForDataReady()){
      delay(1);
    }
    distance = distanceSensor.getDistance();
    distanceSensor.stopRanging();
    LiDar[i]=("a");
    LiDar[i]+=(45+i*2);
    LiDar[i]+=("A");
    LiDar[i]+=(",");
    LiDar[i]+=("b");
    LiDar[i]+=(distance/1000);
    LiDar[i]+=("B");
    Serial.println(LiDar[i]);
    Serial.println(i);
  }
  
  for(i=45; i>=0; i--){
    LiDarS.write(map(i,45,0,180,0));
    distanceSensor.startRanging();
    while (!distanceSensor.checkForDataReady()){
      delay(1);
    }
    distance = distanceSensor.getDistance();
    distanceSensor.stopRanging();
    LiDar[i]=("a");
    LiDar[i]+=(135-(90-i*2));
    LiDar[i]+=("A");
    LiDar[i]+=(",");
    LiDar[i]+=("b");
    LiDar[i]+=(distance/1000);
    LiDar[i]+=("B");
    Serial.println(LiDar[i]);
    Serial.println(i);
  }
}
