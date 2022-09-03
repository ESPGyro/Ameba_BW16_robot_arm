// Author : Mason 2022/09/02, masonchen1003@gmail.com
// FB : https://www.facebook.com/mason.chen.1420
// Hardware : A1 Gyro + MG90 x 5 + 3D parts 

#include "BLEDevice.h"
#include <Adafruit_PWMServoDriver.h>

// SG90 Servo PWM Pulse Traveling
const float PWMRES_Min = 0;       // PWM Resolution 0
const float PWMRES_Max = 180;     // PWM Resolution 180
const float SERVOMIN = 100;       // 500
const float SERVOMAX = 480;      // 2400

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// setting PCA9685 pins
int SERVOS_PIN[5] = {12,11,10,9,8};

#define UART_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define STRING_BUF_SIZE 100

BLEService UartService(UART_SERVICE_UUID);
BLECharacteristic Rx(CHARACTERISTIC_UUID_RX);
BLECharacteristic Tx(CHARACTERISTIC_UUID_TX);
BLEAdvertData advdata;
BLEAdvertData scndata;
bool notify = false;

String receive_data ="";
bool run_status = true; 
int arm_servo = 5;  
int step_angle = 3; 

int s_index = 0;

// servo offset, pleaee update your offset values
int offset[] PROGMEM = {  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};  // default

// Version
String FW_Version = "A1 Gyro Arm V103 (2022/09/02)";

// Servos Matrix
const int ALLMATRIX = 11;        // P2 + P5 + P13 + P10 + P4 + P7 + P12 + P11 + P17 + P16 + Run Time
const int ALLSERVOS = 10;        // P2 + P5 + P13 + P10 + P4 + P7 + P12 + P11 + P17 + P16

// Servo update time
const int Update_time = 20;   // 10ms

// Backup Servo Value
float Running_Servo_POS [ALLSERVOS];

int tune[] PROGMEM = { 0,0,0,0,0,0,0,0,0,0};  
int motion[] PROGMEM = { 90,90,90,90,90,90,90,90,90,90,500};

int Servo_home [][ALLMATRIX] = { 90,  120,  170,  60,  130,  90,  90,  90, 90 , 90, 900 };
int Servo_init [][ALLMATRIX] = { 90,  90,  90,  90,  90,  90,  90,  90, 90 , 90, 500 };
int Servo_arm_home [][ALLMATRIX] = { 90,  120,  170,  60,  50,  90,  90,  90, 90 , 90, 500 };

void Servo_PROGRAM_Run(int iMatrix[][11],  int iSteps) {
  float inter_increment[ALLSERVOS];
  
  for ( int MainLoopIndex = 0; MainLoopIndex < iSteps; MainLoopIndex++)  
    {
    int run_time = iMatrix [ MainLoopIndex ] [ ALLMATRIX - 1 ];
        for (int i = 0; i < arm_servo; i++) 
          {
            inter_increment[i] = (iMatrix[MainLoopIndex][i] - Running_Servo_POS[i]) / (run_time / Update_time);
          }          
    for (int j = 0; j < run_time / Update_time; j++) { 
        for (int i = 0; i < arm_servo; i++) 
          {
           pwm.setPWM( SERVOS_PIN[i], 0, floor(((Running_Servo_POS[i] + inter_increment[i] + tune[i]+offset[i]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN)));  
           Running_Servo_POS[i] = Running_Servo_POS[i] + inter_increment[i];
          }
          delay(Update_time);
    }
  }   // end of main_loop
}

void move_up(int i, int limit_h, int step_a) {
      if (Running_Servo_POS[i] < limit_h) {
         pwm.setPWM( SERVOS_PIN[i], 0, floor(((Running_Servo_POS[i]  +offset[i] +step_a) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN)));  
         Running_Servo_POS[i] = Running_Servo_POS[i] +step_a;
    } else {
         pwm.setPWM( SERVOS_PIN[i], 0, floor(((limit_h  + offset[i]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN)));  
         Running_Servo_POS[i] = limit_h;
    }
}

void move_down(int i, int limit_l, int step_b) { 
  if (Running_Servo_POS[i] > limit_l) {
         pwm.setPWM( SERVOS_PIN[i], 0, floor(((Running_Servo_POS[i]  +offset[i] -step_b) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN)));  
         Running_Servo_POS[i] = Running_Servo_POS[i] - step_b;
    } else {
         pwm.setPWM( SERVOS_PIN[i], 0, floor(((limit_l  + offset[i]) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN)));  
         Running_Servo_POS[i] = limit_l;
   }
 }

void readCB (BLECharacteristic* chr, uint8_t connID) {
    printf("Characteristic %s read by connection %d \n", chr->getUUID().str(), connID);
}

void notifCB (BLECharacteristic* chr, uint8_t connID, uint16_t cccd) {
    if (cccd & GATT_CLIENT_CHAR_CONFIG_NOTIFY) {
        printf("Notifications enabled on Characteristic %s for connection %d \n", chr->getUUID().str(), connID);
        notify = true;
    } else {
        printf("Notifications disabled on Characteristic %s for connection %d \n", chr->getUUID().str(), connID);
        notify = false;
    }
}

void writeCB (BLECharacteristic* chr, uint8_t connID) {
    uint16_t datalen = chr->getDataLen();
    if (datalen > 0) {
        receive_data=chr->readString();
        Serial.println(receive_data);

        if (receive_data.startsWith("A")) {
            run_status = false;
            move_down(0,10,step_angle);
        }
        if (receive_data.startsWith("B")) {
            run_status = false;
            move_up(0,170,step_angle);
        }
        if (receive_data.startsWith("C")) {
            run_status = false;
            move_down(1,40,step_angle);
        }
        if (receive_data.startsWith("D")) {
            run_status = false;
            move_up(1,170,step_angle);
        }
        if (receive_data.startsWith("E")) {
            run_status = false;
            move_down(2,10,step_angle);
        }
        if (receive_data.startsWith("F")) {
            run_status = false;
            move_up(2,170,step_angle);
        }
        if (receive_data.startsWith("G")) {
           run_status = false;
            move_down(3,10,step_angle);
        }
        if (receive_data.startsWith("H")) {
            run_status = false;
            move_up(3,170,step_angle);
        }
        if (receive_data.startsWith("J")) {
            run_status = false;
            move_up(4,90,10);
       }
        if (receive_data.startsWith("I")) {
            run_status = false;
            move_down(4,50,10);
        }
        if (receive_data.startsWith("S")) {
            run_status = false;
            Servo_PROGRAM_Run(Servo_init, 1);
        }
        if (receive_data.startsWith("Y")) {  // Home
            run_status = false;
            Servo_PROGRAM_Run(Servo_arm_home, 1);
        }        
    }
}

void setup() {
    Serial.begin(115200);
    
    pwm.begin();
    pwm.setPWMFreq(50);

    for (int i = 0; i < arm_servo; i++) {
      pwm.setPWM( SERVOS_PIN[i], 0, floor(((90+offset[i] ) / PWMRES_Max * (SERVOMAX - SERVOMIN) + SERVOMIN)));  
      Running_Servo_POS[i] = 90;
     }
    
    advdata.addFlags(GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    advdata.addCompleteName("AMEBA_BLE_DEV");
    scndata.addCompleteServices(BLEUUID(UART_SERVICE_UUID));

    Rx.setWriteNRProperty(true);  // no response
    Rx.setWritePermissions(GATT_PERM_WRITE);
    Rx.setWriteCallback(writeCB);
    Rx.setBufferLen(STRING_BUF_SIZE);
    Tx.setReadProperty(true);
    Tx.setReadPermissions(GATT_PERM_READ);
    Tx.setReadCallback(readCB);
    Tx.setNotifyProperty(true);
    Tx.setCCCDCallback(notifCB);
    Tx.setBufferLen(STRING_BUF_SIZE);

    UartService.addCharacteristic(Rx);
    UartService.addCharacteristic(Tx);

    BLE.init();
    BLE.configAdvert()->setAdvData(advdata);
    BLE.configAdvert()->setScanRspData(scndata);
    BLE.configServer(1);
    BLE.addService(UartService);

    BLE.beginPeripheral();
}

void loop() {
    delay(100);
}
