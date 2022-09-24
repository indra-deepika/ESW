#include <Wire.h>
#include "max32664.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TMP117.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "WiFi.h"
#include "ThingSpeak.h"

#define RESET_PIN 18
#define MFIO_PIN 19
#define RAWDATA_BUFFLEN 250
#define RXp2 13
#define TXp2 12
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_TMP117  tmp117;
Adafruit_MPU6050 mpu;
float threshold = 1;
int steps, flag = 0;

char* ssid = "Galaxy M31s1D5C";
char* password = "pallavi999";
int writeChannelID = 1848342;
char* WriteAPIKey = "YL6DCGQYDMTHB0AE";
WiFiClient client;

max32664 MAX32664(RESET_PIN, MFIO_PIN, RAWDATA_BUFFLEN);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void mfioInterruptHndlr(){
  //Serial.println("i");
}

void enableInterruptPin(){

  //pinMode(mfioPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MAX32664.mfioPin), mfioInterruptHndlr, FALLING);

}

void loadAlgomodeParameters(){

  algomodeInitialiser algoParameters;

  algoParameters.calibValSys[0] = 120;
  algoParameters.calibValSys[1] = 122;
  algoParameters.calibValSys[2] = 125;

  algoParameters.calibValDia[0] = 80;
  algoParameters.calibValDia[1] = 81;
  algoParameters.calibValDia[2] = 82;

  algoParameters.spo2CalibCoefA = 1.5958422;
  algoParameters.spo2CalibCoefB = -34.659664;
  algoParameters.spo2CalibCoefC = 112.68987;

  MAX32664.loadAlgorithmParameters(&algoParameters);
}



void setup(){

  Serial.begin(115200);
 
  
  // *************** GYRO SENSOR - STEP COUNT **************** //

  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }
  else Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);




 
  // *************** HEART RATE AND SPO2 **************** //
  
  Wire.begin();

  loadAlgomodeParameters();

  int result = MAX32664.hubBegin();
  if (result == CMD_SUCCESS){
    Serial.println("Sensorhub begin!");
  }
  else{
      Serial.println("Could not communicate with the sensor! please make proper connections");
  }

  bool ret = MAX32664.startBPTcalibration();
  while(!ret){

    delay(10000);
    Serial.println("failed calib, please restart");
    //ret = MAX32664.startBPTcalibration();
  }

  delay(1000);

  //Serial.println("start in estimation mode");
  ret = MAX32664.configAlgoInEstimationMode();
  while(!ret){

    //Serial.println("failed est mode");
    ret = MAX32664.configAlgoInEstimationMode();
    delay(10000);
  }

  //MAX32664.enableInterruptPin();
  Serial.println("Getting the device ready..");
  delay(1000);



  // **************** TEMP SENSOR - BODY TEMPERATURE **************** //
  while (!Serial) delay(10);  
  Serial.println("Adafruit TMP117 test!");
  if (!tmp117.begin()) {
    Serial.println("Failed to find TMP117 chip");
  }
  else Serial.println("TMP117 Found!");



//********* LCD ********//

   
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);



   Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);

   
   WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  Serial.println("Wifi connected successfully");
  ThingSpeak.begin(client);
}


void loop(){

    display.setCursor(0, 10);

  // *************** HEART RATE AND SPO2 **************** //

  delay(100);

  // **************** TEMP SENSOR - BODY TEMPERATURE **************** //

    sensors_event_t temp; 

    delay(100);
  // *************** GYRO SENSOR - STEP COUNT **************** //

  sensors_event_t a, g, temp_in_gyro;
  float totvect[100] = {0};
  float totave[100] = {0};

  for(int i=0; i<100; i++) {
    mpu.getEvent(&a, &g, &temp_in_gyro);
    totvect[i] = sqrt(((a.acceleration.x)*(a.acceleration.x)) + ((a.acceleration.y)*(a.acceleration.y)) + ((a.acceleration.z)*(a.acceleration.z)));
    totave[i] = (totvect[i] + totvect[i - 1]) / 2 ;
    delay(100);
    
    if (totave[i] > threshold && flag == 0)
    {
      steps = steps + 1;
      flag = 1;
    }
    else if (totave[i] > threshold && flag == 1)
    {
      // Don't Count
    }
    if (totave[i] < threshold   && flag == 1)
    {
      flag = 0;
    }
    if (steps < 0) {
      steps = 0;
    }

//    Serial.println('\n');
    Serial.print("step count: ");
    Serial.println(steps);
    
    uint8_t num_samples = MAX32664.readSamples();
    float heart, sp, dia, sys;
    String bp;
    if(num_samples){
    Serial.print("heartrate = ");
     heart = MAX32664.max32664Output.hr;
    Serial.print(heart);
    Serial.print(" spo2 = ");
     sp = MAX32664.max32664Output.spo2;
    Serial.println(sp);
    Serial.print("blood pressure = ");
//     Serial.print(MAX32664.max32664Output.sys); Serial.print("/"); Serial.println(MAX32664.max32664Output.dia);
    sys = MAX32664.max32664Output.sys;
    dia = MAX32664.max32664Output.dia;
//    Serial.print("bp = ");
    bp = String(sys) + "/" + String(dia);
    Serial.println(bp);
}

    tmp117.getEvent(&temp); 
    float tpr = temp.temperature*(9.0/5) + 39;
      Serial.print("Temperature  "); Serial.print(tpr);Serial.println(" F");
//    Serial.print("Temperature  "); Serial.print(temp.temperature);Serial.println(" C");
    Serial.println("");
    Serial.println("Message Received: ");
    String gsr = Serial2.readString();
    Serial.println(gsr);
  
   delay(1500);
    
    ThingSpeak.setField(1, tpr);
    ThingSpeak.setField(2, steps);
    ThingSpeak.setField(3, gsr);
    ThingSpeak.setField(4, heart);
    ThingSpeak.setField(5, sp);
    ThingSpeak.setField(6, sys);
    ThingSpeak.setField(7, dia);
    
    if(tpr >= 90 && (heart >= 50 && heart <= 115) && (sp >= 92) && (sys >= 80 && sys <= 130) && (dia >= 50 && dia <= 90))
    {
      int t = ThingSpeak.writeFields(writeChannelID, WriteAPIKey);
      if(t == 200)
      {
        Serial.println("Update Successful");
      }
      else{
        Serial.println("HTTP error code " + String(t));
      }
    }

    display.setCursor(0, 10);
    display.print(" Steps: ");
    display.println(steps);
    display.print(" Temperature: ");
    display.println(tpr);
    display.print(" Heart Rate: ");
    display.println(MAX32664.max32664Output.hr);
    display.print(" SPO2: ");
    display.println(MAX32664.max32664Output.spo2);
    display.print(" Blood pressure: ");
    display.print(sys); display.print("/"); display.println(dia);
    display.display(); 
    display.clearDisplay();
  }
//  Serial.println("Message Received: ");
//  Serial.println(Serial2.readString());
//  
//   delay(1500);
  Serial.println("");
  delay(500);
  
}
