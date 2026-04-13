#include "Particle.h"
#include "SdFat.h"
#include <Ezo_i2c.h> 
#include <Wire.h>    
#include <sequencer2.h> 
#include <Ezo_i2c_util.h> 

// Initialize I2C addresses for the probes 
Ezo_board RTD = Ezo_board(102, "RTD");
Ezo_board EC = Ezo_board(100, "EC");

void step1(); 
void step2();
int secondsUntilNextEvent();

Sequencer2 Seq(&step1, 1000, &step2, 0);

SYSTEM_MODE(MANUAL);
SYSTEM_THREAD(ENABLED);

// SD SPI Configuration
const int SD_CHIP_SELECT = D5;
SdFat sd;
File myFile;

const unsigned long MAX_TIME_TO_PUBLISH_MS = 20000; 
const unsigned long SECONDS_BETWEEN_MEASUREMENTS = 360; 

enum State { DATALOG_STATE, PUBLISH_STATE, SLEEP_STATE };
State state = DATALOG_STATE;

#define PUBLISHING 0

unsigned long stateTime = 0;
long real_time;
int millis_now;
double temp; 
double cond; 
char data[120];
float rtd;
float ec_float; 
int errorcode; 

SerialLogHandler logHandler;
SystemSleepConfiguration config;
const char *eventName = "cond";
FuelGauge batteryMonitor;

void setup() {
  if (PUBLISHING == 1) {
    Particle.connect();
  } else {
    Cellular.off();
  }
  
  delay(3000);
  Wire.begin();
  Serial.begin(9600);
  Seq.reset();

  // Initialize SD card
  if (!sd.begin(SD_CHIP_SELECT)) {
    Serial.println("SD initialization failed!");
  } else {
    Serial.println("SD initialization done.");
    
    // --- DATA DUMP START ---
    // This reads the existing file and prints it to your Mac's screen via Serial
    Serial.println(">>> START OF STORED DATA <<<");
    if (myFile.open("conductivity.csv", O_READ)) {
      while (myFile.available()) {
        Serial.write(myFile.read());
      }
      myFile.close();
      Serial.println("\n>>> END OF STORED DATA <<<");
    } else {
      Serial.println("No existing conductivity.csv found to read.");
    }
    delay(3000); // Wait 3 seconds so you can see the data before logging starts
    // --- DATA DUMP END ---
  }

  // Open the file for logging (append mode)
  if (!myFile.open("conductivity.csv", O_RDWR | O_CREAT | O_AT_END)) {
    Serial.println("opening conductivity.csv for write failed");
  } else {
    // If file is empty, write headers
    if (myFile.fileSize() == 0) {
      myFile.println("Real_Time,Temp_C,Cond_uScm-1,CellVoltage,StateofCharge,ErrorCode");
    }
    myFile.close(); 
  }
}

void loop() {
  switch (state) {
    case DATALOG_STATE: {
      delay(1000);
      step1(); 
      delay(1000); 
      step2(); 
      delay(300);
      temp = RTD.get_last_received_reading(); 
      cond = EC.get_last_received_reading(); 
    
      real_time = Time.now();
      millis_now = millis();
      float cellVoltage = batteryMonitor.getVCell();
      float stateOfCharge = batteryMonitor.getSoC();
      errorcode = 0; 

      if (!sd.begin(SD_CHIP_SELECT, SPI_FULL_SPEED)) {
        Serial.println("failed to open card");
        errorcode = 1; 
      }
      
      if (!myFile.open("conductivity.csv", O_RDWR | O_CREAT | O_AT_END)) {
        Serial.println("opening conductivity.csv for write failed");
        errorcode = 2; 
      } else {
        snprintf(data, sizeof(data), "%li,%.2f,%.2f,%.2f,%.2f,%i", real_time, temp, cond, cellVoltage, stateOfCharge, errorcode);
        Serial.println(data); // Also prints live data to your Mac
        myFile.println(data);
        myFile.close();
      }

      state = (PUBLISHING == 1) ? PUBLISH_STATE : SLEEP_STATE;
    }
    break;

    case PUBLISH_STATE: {
      stateTime = millis();
      while (millis() - stateTime < MAX_TIME_TO_PUBLISH_MS) {
        if (Particle.connected()) {
          Particle.publish(eventName, data, 60, PRIVATE, WITH_ACK);
          state = SLEEP_STATE;
          break;
        }
        Particle.connect();
        delay(500);
      }
      state = SLEEP_STATE;
    }
    break;
    
    case SLEEP_STATE: {
      Serial.println("going to sleep");
      delay(500);
      int wakeInSeconds = secondsUntilNextEvent(); 
      config.mode(SystemSleepMode::ULTRA_LOW_POWER)
        .duration(wakeInSeconds * 1000L);
      System.sleep(config);
      state = DATALOG_STATE;
    }
    break;
  }                             
}

void step1(){
  RTD.send_read_cmd();    
  EC.send_read_cmd();
}

void step2(){
  receive_and_print_reading(RTD);
  receive_and_print_reading(EC);
}

int secondsUntilNextEvent() {
  int current_seconds = Time.now();
  return SECONDS_BETWEEN_MEASUREMENTS - (current_seconds % SECONDS_BETWEEN_MEASUREMENTS);
}
