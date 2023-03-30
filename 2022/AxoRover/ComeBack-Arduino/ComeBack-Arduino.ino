#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <IBusBM.h>
#include <TB9051FTGMotorCarrier.h>
#include <HMC5883L.h>
//#include <ADXL345.h>
#include <MPU6050.h>
#include <BMP085.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <DFRobot_INA219.h>
#include <TCA9548A.h>
#include <SparkFun_VL53L1X.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

static constexpr uint8_t pwm1Der{5};
static constexpr uint8_t pwm2Der{6};
static constexpr uint8_t pwm1Izq{7};
static constexpr uint8_t pwm2Izq{8};
static TB9051FTGMotorCarrier driverDer{pwm1Der, pwm2Der}, driverIzq{pwm1Izq, pwm2Izq};

DFRobot_INA219_IIC ina219(&Wire, INA219_I2C_ADDRESS1);
TCA9548A mux;
SFEVL53L1X distanceSensor;
IBusBM IBus;
HMC5883L compass;
MPU6050 accelerometer;
BMP085 barometer;
NMEAGPS  gps;
gps_fix  fix;
Servo parachute;
Adafruit_BME680 bme;
                                                                                                            //19.052210550633667, -98.17420512396775
float xv, yv, zv, heading, headingDegrees, calibrated_values[3], referencePressure, relativeAltitude, latitud=0, longitud=0, movimientoNec=0,
latHome=19.1028264, lonHome=-97.9649276, throttleDer, throttleIzq, giro=0, giroRel=0, dist=0, ina219Reading_mA = 1000, extMeterReading_mA = 1000,
voltaje=0, distance=0;
int pitch, roll, channel1, channel2, channel3, channel5, channel6, throttlePercent=0, giroDer, giroIzq, contador=0, cuadrante=0, bandera=0;
double internalTemp;
long realPressure, tiempo=0, tiempoUno=0, tiempoDos=0;
String datos;

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  mux.begin(Wire);
  IBus.begin(Serial2);
  gpsPort.begin(9600);
  parachute.attach(11);
  parachute.write(0);
  mux.openChannel(0);
  bme.begin();
  mux.closeAll();
  mux.openChannel(1);
  ina219.begin();
  mux.closeAll();
  

  //INICIALIZACION OBJETOS
  Serial.println("Inicializando LoRa");
  LoRa.begin(500E6);
  Serial.println("Inicializando Acelerometro");
  accelerometer.begin();
  accelerometer.setI2CBypassEnabled(true);
  Serial.println("Inicializando Magnetometro");
  compass.begin();
  Serial.println("Inicializando Barometro");
  barometer.begin(BMP085_STANDARD);

  checkSettings();

  //INICIALIZACION MOTORES
  pinMode(22,OUTPUT);
  pinMode(23,OUTPUT);
  pinMode(24,OUTPUT);
  pinMode(25,OUTPUT);
  driverDer.enable();
  driverIzq.enable();
  driverDer.setOutput(throttlePercent);
  driverIzq.setOutput(throttlePercent);
  digitalWrite(23,HIGH);
  digitalWrite(22,LOW);
  digitalWrite(25,HIGH);
  digitalWrite(24,LOW); 

  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void loop() {
  magnetometro();
  acelerometro();
  barometro();
  location();
  objetivo();
  movimiento();
  bateria();
  voc();
  info();
}

void info(){
    datos=(" Head=");
    datos+=(headingDegrees);
    datos+=(" Pitch=");
    datos+=(pitch);
    datos+=(" Roll=");
    datos+=(roll);
    datos+=(" Pres=");
    datos+=(realPressure);
    datos+=(" Alt=");
    datos+=(relativeAltitude);
    datos+=(" T-Ext=");
    datos+=(bme.temperature);
    datos+=(" T-Int=");
    datos+=(internalTemp);
    datos+=(" Giro=");
    datos+=(giro);
    datos+=(" Dist=");
    datos+=(dist);
    datos+=("LaH=\n");
  datos+=(latHome,7);
  datos+=(" LoH=");
  datos+=(lonHome,7);
  datos+=(" ThrotDer=");
  datos+=(throttleDer);
  datos+=(" ThrotIzq=");
  datos+=(throttleIzq);
  datos+=(" Bateria=");
  datos+=(voltaje);
  datos+=(" Mov=");
  datos+=(movimientoNec);
  datos+=(" Humedad=");
  datos+=(bme.humidity);
  
    if(contador==10){
    LoRa.beginPacket();
      LoRa.print("Lat=");
      LoRa.print(latitud,7);
      LoRa.print(" Lon=");
      LoRa.print(longitud,7);
      LoRa.println(datos);
    LoRa.endPacket();
    contador=0;
    }

  Serial.print("Lat=");
  Serial.print(latitud,7);
  Serial.print(" Lon=");
  Serial.print(longitud,7);
  Serial.println(datos);

  //Debug
  
  
  contador++;
}

void magnetometro(){
  getHeading();
  
  float values_from_magnetometer[3];
  values_from_magnetometer[0] = xv;
  values_from_magnetometer[1] = yv;
  values_from_magnetometer[2] = zv;
  transformation(values_from_magnetometer);
  
  vector_length_stabilasation();

  heading = atan2(calibrated_values[0],calibrated_values[1]); 

  float declinationAngle = (0 + (54 / 60.0)) / (180 / M_PI);
  heading += declinationAngle - (PI/2);

  if (heading < 0){
    heading += 2 * PI;
  }

  if (heading > 2 * PI){
    heading -= 2 * PI;
  }

  headingDegrees = heading * 180/M_PI; 
}

void acelerometro(){
  Vector norm = accelerometer.readNormalizeAccel();
  Vector filtered = accelerometer.lowPassFilter(norm, 0.5);
  
  pitch = -(atan2(filtered.XAxis, sqrt(filtered.YAxis*filtered.YAxis + filtered.ZAxis*filtered.ZAxis))*180.0)/M_PI;
  roll  = (atan2(filtered.YAxis, filtered.ZAxis)*180.0)/M_PI;
}

void barometro(){
  internalTemp = barometer.readTemperature();
  realPressure = barometer.readPressure();

  relativeAltitude = barometer.getAltitude(realPressure, referencePressure);
}

void location(){
  while(gps.available(gpsPort)){
    fix=gps.read();
    latitud=fix.latitude();
    longitud=fix.longitude();
  }
}

void objetivo(){
  /*
  if(latitud!=0&&tiempoUno==0){
    tiempoUno=millis();
  }
  if(latitud!=0&&bandera==0){
    tiempoDos=millis();
    tiempo=tiempoDos-tiempoUno;
  }
  if(bandera==0&&tiempo>30000){
    latHome=latitud;
    lonHome=longitud;
    Serial.print("Home: ");
    Serial.print(latitud,7);
    Serial.print(", ");
    Serial.println(longitud,7);
  }
  if(bandera==0&&tiempo>31000){
    bandera=1;
  }
  */
  if(latitud>latHome&&longitud<lonHome){
    cuadrante=1;

  }
  else if(latitud>latHome&&longitud>lonHome){
    cuadrante=2;
  }
  else if(latitud<latHome&&longitud>lonHome){
    cuadrante=3;
  }
  else if(latitud<latHome&&longitud<lonHome){
    cuadrante=4;
  }
  else{
    cuadrante=0;
  }
  float restaLat=latHome-latitud;
  float restaLon=lonHome-longitud;
  giro=(atan2(restaLat,restaLon));
  if (giro < 0){
    giro += 2 * PI;
  }

  if (giro > 2 * PI){
    giro -= 2 * PI;
  }

  giro = giro * 180/M_PI; 
  dist=sqrt(sq(restaLat)+sq(restaLon))*111000;
}

void movimiento(){
  movimientoNec=giro-(headingDegrees);
  //Autonomía
  if(movimientoNec<3&&dist>15){
    throttleDer=1.0;
    throttleIzq=0.8;
  }
  else if(movimientoNec>=3&&dist>15){
    throttleDer=0.8;
    throttleIzq=1.0;
  }
  else{
    throttleDer=0;
    throttleIzq=0;
  }
  
  //Control Remoto
  channel5=IBus.readChannel(4);

  if(channel5>1900){
  channel1=IBus.readChannel(0);
  channel2=IBus.readChannel(1);
  channel3=IBus.readChannel(2);
  channel5=IBus.readChannel(5);
  
  throttlePercent=map(channel3,1000,2000,0,100);

  if(channel1>1600){
  giroDer=map(channel1,1600,2000,0,100);
  giroIzq=0;
  }
  else if(channel1<1400){
  giroIzq=map(channel1,1000,1400,100,0);
  giroDer=0;
  }
  else{
    giroDer=0;
    giroIzq=0;
  }
  
  throttleDer=map(channel2,1000,2000,-throttlePercent,throttlePercent)-giroDer;
  throttleIzq=map(channel2,1000,2000,-throttlePercent,throttlePercent)-giroIzq;
  
  if(throttleDer<0){
    throttleDer=throttleDer/200;
  }
  if(throttleIzq<0){
    throttleIzq=throttleIzq/200;
  }
  }
  
  driverDer.setOutput(throttleDer);
  driverIzq.setOutput(throttleIzq);
  parachute.writeMicroseconds(channel6);
  
}

void bateria(){
  mux.openChannel(1);
    voltaje=ina219.getBusVoltage_V();
  mux.closeChannel(1);
  Serial.println("Bat");
}

void voc(){
  mux.openChannel(0);
    bme.performReading();
    mux.closeChannel(0);
    Serial.println("BME");
}

//ADQUISICION MAGNETOMETRO
void getHeading(){ 
  Vector raw = compass.readRaw();
  
  xv=raw.XAxis;
  yv=raw.YAxis;
  zv=raw.ZAxis;
}

//CALIBRACION MAGNETOMETRO
void transformation(float uncalibrated_values[3]){
  double calibration_matrix[3][3] = 
  {
    {1.33,0.021,-0.243},
    {0.032,1.366,0.017},
    {0.077,-0.081,1.471}  
  };
  
  double bias[3] = 
  {
    -22.891,
    -109.252,
    -248.098
  };  

  for (int i=0; i<3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
  float result[3] = {0, 0, 0};
  for (int i=0; i<3; ++i)
    for (int j=0; j<3; ++j)
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j];
  for (int i=0; i<3; ++i) calibrated_values[i] = result[i];
  }
  
  float scaler;
  boolean scaler_flag = false;
  float normal_vector_length;
  void vector_length_stabilasation(){
    
  if (scaler_flag == false)
  {
    getHeading();
    normal_vector_length = sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);
    scaler_flag = true;
  } 

  scaler = normal_vector_length/sqrt(calibrated_values[0]*calibrated_values[0] + calibrated_values[1]*calibrated_values[1] + calibrated_values[2]*calibrated_values[2]);

  calibrated_values[0] = calibrated_values[0]*scaler;
  calibrated_values[1] = calibrated_values[1]*scaler;
  calibrated_values[2] = calibrated_values[2]*scaler;
}

//CHEQUEO y SELECCION DE AJUSTES
void checkSettings(){
  Serial.println("Checking Settings");
  
  //BAROMETER
  referencePressure = barometer.readPressure();
  Serial.println("Barometer Settings OK");
  
  //ACCELEROMETER
  accelerometer.setRange(MPU6050_RANGE_4G);
  Serial.println("Accelerometer Settings OK");
  
  //COMPASS
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_15HZ);
  compass.setSamples(HMC5883L_SAMPLES_1);
  
  Serial.print("Selected range: ");
  
  switch (compass.getRange())
  {
    case HMC5883L_RANGE_0_88GA: Serial.println("0.88 Ga"); break;
    case HMC5883L_RANGE_1_3GA:  Serial.println("1.3 Ga"); break;
    case HMC5883L_RANGE_1_9GA:  Serial.println("1.9 Ga"); break;
    case HMC5883L_RANGE_2_5GA:  Serial.println("2.5 Ga"); break;
    case HMC5883L_RANGE_4GA:    Serial.println("4 Ga"); break;
    case HMC5883L_RANGE_4_7GA:  Serial.println("4.7 Ga"); break;
    case HMC5883L_RANGE_5_6GA:  Serial.println("5.6 Ga"); break;
    case HMC5883L_RANGE_8_1GA:  Serial.println("8.1 Ga"); break;
    default: Serial.println("Bad range!");
  }
  
  Serial.print("Selected Measurement Mode: ");
  switch (compass.getMeasurementMode())
  {  
    case HMC5883L_IDLE: Serial.println("Idle mode"); break;
    case HMC5883L_SINGLE:  Serial.println("Single-Measurement"); break;
    case HMC5883L_CONTINOUS:  Serial.println("Continuous-Measurement"); break;
    default: Serial.println("Bad mode!");
  }

  Serial.print("Selected Data Rate: ");
  switch (compass.getDataRate())
  {  
    case HMC5883L_DATARATE_0_75_HZ: Serial.println("0.75 Hz"); break;
    case HMC5883L_DATARATE_1_5HZ:  Serial.println("1.5 Hz"); break;
    case HMC5883L_DATARATE_3HZ:  Serial.println("3 Hz"); break;
    case HMC5883L_DATARATE_7_5HZ: Serial.println("7.5 Hz"); break;
    case HMC5883L_DATARATE_15HZ:  Serial.println("15 Hz"); break;
    case HMC5883L_DATARATE_30HZ: Serial.println("30 Hz"); break;
    case HMC5883L_DATARATE_75HZ:  Serial.println("75 Hz"); break;
    default: Serial.println("Bad data rate!");
  }
  
  Serial.print("Selected number of samples: ");
  switch (compass.getSamples())
  {  
    case HMC5883L_SAMPLES_1: Serial.println("1"); break;
    case HMC5883L_SAMPLES_2: Serial.println("2"); break;
    case HMC5883L_SAMPLES_4: Serial.println("4"); break;
    case HMC5883L_SAMPLES_8: Serial.println("8"); break;
    default: Serial.println("Bad number of samples!");
  }
  
   Serial.println("Compass Settings OK");
}