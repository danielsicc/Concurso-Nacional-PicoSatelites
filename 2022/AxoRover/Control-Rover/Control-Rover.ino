#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <IBusBM.h>
#include <TB9051FTGMotorCarrier.h>
#include <HMC5883L.h>
#include <ADXL345.h>
#include <BMP085.h>
#include <NMEAGPS.h>
#include <GPSport.h>
#include <DFRobot_INA219.h>
#include <TCA9548A.h>
#include <SparkFun_VL53L1X.h>
#include <Servo.h>

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
ADXL345 accelerometer;
BMP085 barometer;
NMEAGPS  gps;
gps_fix  fix;
Servo parachute;

float xv, yv, zv, heading, headingDegrees, calibrated_values[3], referencePressure, relativeAltitude, latitud=0, longitud=0, latHome=0, lonHome=0, throttleDer, throttleIzq, giro=0, giroRel=0, dist=0;
float ina219Reading_mA = 1000;
float extMeterReading_mA = 1000;
int pitch, roll, channel1, channel2, channel3, channel5, channel6, throttlePercent=0, giroDer, giroIzq, contador=0, cuadrante=0, bandera=0;
double internalTemp;
long realPressure, tiempo=0, tiempoUno=0, tiempoDos=0;
String datos;

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

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);
  IBus.begin(Serial2);
  gpsPort.begin(9600);
  parachute.attach(11);
  parachute.write(0);

  while(!LoRa.begin(433E6)) 
  {
    Serial.println("Starting LoRa failed!");
    delay(500);
  }
  
  while(!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  while(!accelerometer.begin())
  {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    delay(500);
  }
 
  while(!barometer.begin(BMP085_STANDARD))
  {
    Serial.println("Could not find a valid BMP085 or BMP180 sensor, check wiring!");
    delay(500);
  }

/*
  mux.begin(Wire);
  mux.openChannel(0);
    ina219.begin();
    ina219.linearCalibrate(ina219Reading_mA, extMeterReading_mA);
  mux.closeAll(); 
  
  mux.openChannel(1);
    distanceSensor.begin();
  mux.closeAll(); 
  */
  checkSettings();

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
}

void checkSettings(){
  Serial.println("Checking Settings");
  
  //BAROMETER
  referencePressure = barometer.readPressure();
  Serial.println("Barometer Settings OK");
  
  //ACCELEROMETER
  accelerometer.setRange(ADXL345_RANGE_8G);
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

void loop() {
  magnetometro();
  acelerometro();
  barometro();
  location();
  objetivo();
  movimiento();
  //bateria();
  //distancia();
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
    datos+=(" T-Int=");
    datos+=(internalTemp);
    datos+=(" Ch5=");
    datos+=(channel5);
    datos+=(" Ch1=");
    datos+=(channel1);
    datos+=(" Ch2=");
    datos+=(channel2);
    datos+=(" Ch3=");
    datos+=(channel3);
    datos+=(" Ch6=");
    datos+=(channel6);
    datos+=(" Cuad=");
    datos+=(cuadrante);
    datos+=(" Giro=");
    datos+=(giro);
    datos+=(" GiroRel=");
    datos+=(giroRel);
    datos+=(" Dist=");
    datos+=(dist);
    
    /*
    if(contador==30){
    LoRa.beginPacket();
      LoRa.print("Lat=");
      LoRa.print(latitud,7);
      LoRa.print(" Lon=");
      LoRa.print(longitud,7);
      LoRa.println(datos);
    LoRa.endPacket();
    contador=0;
    }
    */
  
  Serial.print("Lat=");
  Serial.print(latitud,7);
  Serial.print(" Lon=");
  Serial.print(longitud,7);
  Serial.println(datos);

  //Debug
  Serial.print("LaH=");
  Serial.print(latHome,7);
  Serial.print(" LoH=");
  Serial.print(lonHome,7);
  Serial.print(" Band=");
  Serial.println(bandera);
  
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

  heading = atan2(calibrated_values[1],calibrated_values[0]); 

  float declinationAngle = (0 + (54 / 60.0)) / (180 / M_PI);
  heading += declinationAngle - (PI/2);

  if (heading < 0){
    heading += 2 * PI;
  }

  if (heading > 2 * PI){
    heading -= 2 * PI;
  }

  headingDegrees = heading * 180/M_PI; 
  headingDegrees += 90;
}

void getHeading(){ 
  Vector raw = compass.readRaw();
  
  xv=raw.XAxis;
  yv=raw.YAxis;
  zv=raw.ZAxis;
}

void acelerometro(){
  Vector norm = accelerometer.readNormalize();
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
  giro=(atan2(restaLat,restaLon))*57.2958;
  giroRel=giro-180;
  dist=sqrt(sq(restaLat)+sq(restaLon))*111000;
}

void movimiento(){
  
  //Autonomía
  if(giro>giroRel+5||giro<giroRel-5){
    throttleDer=(1-abs(giroRel/giro));
    throttleIzq=-(1-abs(giroRel/giro));
  }
  else{
    throttleDer=0.5;
    throttleIzq=0.5;
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
    throttleDer=throttleDer/2;
  }
  if(throttleIzq<0){
    throttleIzq=throttleIzq/2;
  }
  }
  
  driverDer.setOutput(throttleDer/100);
  driverIzq.setOutput(throttleIzq/100);
  parachute.writeMicroseconds(channel6);
  
}

void bateria(){
  mux.openChannel(0);
    Serial.print("BusVoltage:   ");
    Serial.print(ina219.getBusVoltage_V(), 2);
    Serial.println("V");
    Serial.print("ShuntVoltage: ");
    Serial.print(ina219.getShuntVoltage_mV(), 3);
    Serial.println("mV");
    Serial.print("Current:      ");
    Serial.print(ina219.getCurrent_mA(), 1);
    Serial.println("mA");
    Serial.print("Power:        ");
    Serial.print(ina219.getPower_mW(), 1);
    Serial.println("mW");
    Serial.println("");
    mux.closeAll();
}

void distancia(){
  mux.openChannel(1);
    distanceSensor.startRanging(); //Write configuration bytes to initiate measurement
    while (!distanceSensor.checkForDataReady())
    {
      delay(1);
    }
    float distance = distanceSensor.getDistance(); //Get the result of the measurement from the sensor
    distanceSensor.clearInterrupt();
    distanceSensor.stopRanging();

    Serial.print("Distance(cm): ");
    Serial.print(distance/10);

    float distanceInches = distance * 0.0393701;
    float distanceFeet = distanceInches / 12.0;

    Serial.print("\tDistance(ft): ");
    Serial.print(distanceFeet, 2);

    Serial.println();

    mux.closeAll();
}
