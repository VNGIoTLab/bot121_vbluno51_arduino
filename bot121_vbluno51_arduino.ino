/******************************************************************************************
 * Project:   Code mẫu cơ bản cho Bot trong cuộc thi VNG Bot Battle 12+1
 * Hardware:  
 *    + Main board: Bo mạch VBLUno51 của VNG IoT Lab với giao tiếp không dây
 *                  Bluetooth Low Energy (BLE)
 *    + Robot: Đây chỉ là cấu hình phần cứng cơ bản. Các đội tùy biến theo nhu cầu
 *        - 2 Động cơ bánh sau: Điều khiển Tiến, Lùi, Quay trái, Quay phải, Dừng
 *        - 4 Relay để điều khiển Bật/Tắt các cơ cấu
 * Software:  Arduino IDE
 * Date:      15/7/2017
 * Author:    VNG IoT Lab
 * License:   Apache License 2.0
 ******************************************************************************************/

#include <BLE_API.h>

/***********************************************************************************
 ****************************DEFINES & BIẾN TOÀN CỤC********************************
 ***********************************************************************************/
 
#define DEBUG_SERIAL                          //Sử dụng khi cần debug qua cổng Serial (USB to Serial)

#ifdef DEBUG_SERIAL
  #define PRINTF                Serial.println
  #define BAUD_RATE             115200
#else
  #define PRINTF
  #define BAUD_RATE
#endif

#define uint                    unsigned int
#define uchar                   unsigned char

#define onLedStatus             digitalWrite(LED, HIGH)
#define offLedStatus            digitalWrite(LED, LOW)

/*
 *  @brief  Các kết nối phần cứng và tham số cơ bản cho điều khiển 2 động cơ bánh
 */
#define M1_A                    D13     //Động cơ bánh trái, sau
#define M1_B                    D12     //Động cơ bánh trái, sau 
#define M2_A                    D11     //Động cơ bánh phải, sau
#define M2_B                    D10     //Động cơ bánh phải, sau
#define M3_A                    D9
#define M3_B                    D8
#define M4_A                    D7
#define M4_B                    D6

#define MOTOR_LEFT_A            M1_A
#define MOTOR_LEFT_B            M1_B
#define MOTOR_RIGHT_A           M2_A
#define MOTOR_RIGHT_B           M2_B
#define DIR_FORWARD             1       //TIEN
#define DIR_BACK                2       //LUI

#define RL_1                    D5    //Relay 1
#define RL_2                    D4    //Relay 2
#define RL_3                    D3    //Relay 3
#define RL_4                    D2    //Relay 4
#define RELAY_ON                0                        
#define RELAY_OFF               1

/*
 *  @brief  Các trạng thái di chuyển của Bot
 */
#define STOP                  0
#define FORWARD               1
#define BACK                  2
#define LEFT                  3
#define RIGHT                 4

/*
 *  @brief  Một vài thông số cho giao tiếp BLE
 */
#define DEVICE_NAME       "VNG_Bot1"
#define TXRX_BUF_LEN      2
// Create ble instance
BLE                       ble;

//Bot services gồm 5 characteristics
static const uint8_t bot_service_uuid[]           = {0x00, 0x00, 0xFF, 0xF0, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};
static const uint8_t bot_motion_char_uuid[]       = {0x00, 0x00, 0xFF, 0xF1, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};  //WRITE-uchar
static const uint8_t bot_relay1_char_uuid[]       = {0x00, 0x00, 0xFF, 0xF2, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};  //WRITE-uchar
static const uint8_t bot_relay2_char_uuid[]       = {0x00, 0x00, 0xFF, 0xF3, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};  //WRITE-uchar
static const uint8_t bot_relay3_char_uuid[]       = {0x00, 0x00, 0xFF, 0xF4, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};  //WRITE-uchar
static const uint8_t bot_relay4_char_uuid[]       = {0x00, 0x00, 0xFF, 0xF5, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB};  //WRITE-uchar

// Used in advertisement
static const uint8_t  vng_bot_uuid[]              = {0xFB, 0x34, 0x9B, 0x5F, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xF0, 0xFF, 0x00, 0x00};

// Initialize value of chars
uint8_t bot_motion_char_value[TXRX_BUF_LEN]       = {0};
uint8_t bot_relay1_char_value[TXRX_BUF_LEN]       = {0};
uint8_t bot_relay2_char_value[TXRX_BUF_LEN]       = {0};
uint8_t bot_relay3_char_value[TXRX_BUF_LEN]       = {0};
uint8_t bot_relay4_char_value[TXRX_BUF_LEN]       = {0};
                        
// Create characteristic
GattCharacteristic  bot_motion_characteristic(bot_motion_char_uuid, 
                                              bot_motion_char_value, 
                                              1, 
                                              TXRX_BUF_LEN, 
                                              GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );

GattCharacteristic  bot_relay1_characteristic(bot_relay1_char_uuid, 
                                              bot_relay1_char_value, 
                                              1, 
                                              TXRX_BUF_LEN, 
                                              GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );

GattCharacteristic  bot_relay2_characteristic(bot_relay2_char_uuid, 
                                              bot_relay2_char_value, 
                                              1, 
                                              TXRX_BUF_LEN, 
                                              GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );

GattCharacteristic  bot_relay3_characteristic(bot_relay3_char_uuid, 
                                              bot_relay3_char_value, 
                                              1, 
                                              TXRX_BUF_LEN, 
                                              GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );

GattCharacteristic  bot_relay4_characteristic(bot_relay4_char_uuid, 
                                              bot_relay4_char_value, 
                                              1, 
                                              TXRX_BUF_LEN, 
                                              GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE );

GattCharacteristic *bot_chars[] = {&bot_motion_characteristic, 
                                   &bot_relay1_characteristic, 
                                   &bot_relay2_characteristic, 
                                   &bot_relay3_characteristic, 
                                   &bot_relay4_characteristic};



//Create service
GattService bot_service(bot_service_uuid, 
                        bot_chars, 
                        sizeof(bot_chars) / sizeof(GattCharacteristic *));
  
/***********************************************************************************
 ****************************CÁC HÀM ĐIỀU KHIỂN ĐỘNG CƠ*****************************
 ***********************************************************************************/
 
/*
 *  @brief  Điều khiển 2 động cơ bánh
 */
void controlMotor(uchar left_dir, uchar right_dir) {
  //motor left
  switch(left_dir) {
    case DIR_FORWARD: {
      digitalWrite(MOTOR_LEFT_A, 1);  //0
      digitalWrite(MOTOR_LEFT_B, 0);  //1
      break;
    }
    case DIR_BACK: {
      digitalWrite(MOTOR_LEFT_A, 0);  //1
      digitalWrite(MOTOR_LEFT_B, 1);  //0
      break;
    }
    default: {
      digitalWrite(MOTOR_LEFT_A, 1);  //0
      digitalWrite(MOTOR_LEFT_B, 1);  //0
      break;
    }
  }
  
  //motor right
  switch(right_dir) {
    case DIR_FORWARD: {
      digitalWrite(MOTOR_RIGHT_A, 1); //0
      digitalWrite(MOTOR_RIGHT_B, 0); //1
      break;
    }
    case DIR_BACK: {
      digitalWrite(MOTOR_RIGHT_A, 0); //1
      digitalWrite(MOTOR_RIGHT_B, 1); //0
      break;
    }
    default: {
      digitalWrite(MOTOR_RIGHT_A, 1); //0
      digitalWrite(MOTOR_RIGHT_B, 1); //0
      break;
    }
  }
}

/*
 *  @brief  Dừng xe
 */
void stopCar(){
  controlMotor(STOP, STOP);
  PRINTF("Stop");
}

/*
 *  @brief  Cho xe quay trái
 */
void rotateLeft(){
  controlMotor(DIR_BACK, DIR_FORWARD);  
  PRINTF("Rotate Left");
}

/*
 *  @brief  Cho xe quay phải
 */
void rotateRight(){
  controlMotor(DIR_FORWARD, DIR_BACK);  
  PRINTF("Rotate Right");
}

/*
 *  @brief  Cho xe chạy thẳng
 */
void goForward(){
  controlMotor(DIR_FORWARD, DIR_FORWARD); 
  PRINTF("Go Forward");
}

/*
 *  @brief  Cho xe chạy lùi
 */
void goBack(){
  controlMotor(DIR_BACK, DIR_BACK);
  PRINTF("Go Back");
}

/***********************************************************************************
 ****************************CÁC HÀM ĐIỀU KHIỂN RELAY*****************************
 ***********************************************************************************/
/*
 *  @brief  Điều khiển Relay1
 */
void relay1(uchar value) {
  digitalWrite(RL_1, value);
  if (value == RELAY_ON)
    PRINTF("Turn relay 1 ON");
  else
    PRINTF("Turn relay 1 OFF");
}

/*
 *  @brief  Điều khiển Relay2
 */
void relay2(uchar value) {
  digitalWrite(RL_2, value);
  if (value == RELAY_ON)
    PRINTF("Turn relay 2 ON");
  else
    PRINTF("Turn relay 2 OFF");
}

/*
 *  @brief  Điều khiển Relay3
 */
void relay3(uchar value) {
  digitalWrite(RL_3, value);
  if (value == RELAY_ON)
    PRINTF("Turn relay 3 ON");
  else
    PRINTF("Turn relay 3 OFF");
}

/*
 *  @brief  Điều khiển Relay4
 */
void relay4(uchar value) {
  digitalWrite(RL_4, value);
  if (value == RELAY_ON)
    PRINTF("Turn relay 4 ON");
  else
    PRINTF("Turn relay 4 OFF");
}

/***********************************************************************************
 *******************CÁC HÀM ĐỂ GIAO TIẾP BLE VỚI SMARTPHONE, TABLET*****************
 ***********************************************************************************/

/** @brief  Hàm callback cho sự kiện disconnect
 *
 *  @param[in] *params   params->handle : connect handle
 */
void disconnectionCallback(const Gap::DisconnectionCallbackParams_t *params) {  
  PRINTF("Disconnected. \r\n"
         "Restart advertising....");
  offLedStatus;            
  ble.startAdvertising();
}

/** @brief  Hàm callback cho sự kiện connect
 *
 *  @param[in] *params   params->handle : The ID for this connection
 *                       params->role : PERIPHERAL  = 0x1, // Peripheral Role
 *                                      CENTRAL     = 0x2, // Central Role.
 */
void connectionCallback( const Gap::ConnectionCallbackParams_t *params ) {
  PRINTF("Connected");
  if(params->role == Gap::PERIPHERAL) {
    PRINTF("Role: Peripheral ");
  }
  onLedStatus;
}

/** @brief  Hàm callback cho sự kiện ghi dữ liệu từ gatt server
 *          Stop, Go Forward, Go Back, Rotate Left, Rotate Right
 *
 *  @param[in] *Handler   Handler->connHandle : The handle of the connection that triggered the event
 *                        Handler->handle : Attribute Handle to which the write operation applies
 *                        Handler->writeOp : OP_INVALID               = 0x00,  // Invalid operation.
 *                                           OP_WRITE_REQ             = 0x01,  // Write request.
 *                                           OP_WRITE_CMD             = 0x02,  // Write command.            ////
 *                                           OP_SIGN_WRITE_CMD        = 0x03,  // Signed write command.
 *                                           OP_PREP_WRITE_REQ        = 0x04,  // Prepare write request.
 *                                           OP_EXEC_WRITE_REQ_CANCEL = 0x05,  // Execute write request: cancel all prepared writes.
 *                                           OP_EXEC_WRITE_REQ_NOW    = 0x06,  // Execute write request: immediately execute all prepared writes.
 *                        Handler->offset : Offset for the write operation
 *                        Handler->len : Length (in bytes) of the data to write
 *                        Handler->data : Pointer to the data to write
 */
void gattserverWriteCallback(const GattWriteCallbackParams *Handler) {
  
  static uint8_t buf[TXRX_BUF_LEN];
  static uint16_t bytes_read=0;
  
  PRINTF("--->Central sent values to device.");

  //bot_motion
  if(Handler->handle == bot_motion_characteristic.getValueAttribute().getHandle()) {
    // Read the value of characteristic
    ble.readCharacteristicValue(bot_motion_characteristic.getValueAttribute().getHandle(), 
                                buf, 
                                &bytes_read);       
    switch(buf[0]) {
      case FORWARD: {
        goForward();
        break;
      }
      case BACK: {
        goBack();
        break;
      }
      case LEFT: {
        rotateLeft();
        break;
      }
      case RIGHT: {
        rotateRight();
        break;
      }
      default: {
        stopCar();        
        break;
      }
    }//sw
  }//if

  //relay1
  else if(Handler->handle == bot_relay1_characteristic.getValueAttribute().getHandle()) {
    // Read the value of characteristic
    ble.readCharacteristicValue(bot_relay1_characteristic.getValueAttribute().getHandle(), 
                                buf, 
                                &bytes_read); 
    relay1(buf[0]);
  }//if
  
  //relay2
  else if(Handler->handle == bot_relay2_characteristic.getValueAttribute().getHandle()) {
    // Read the value of characteristic
    ble.readCharacteristicValue(bot_relay2_characteristic.getValueAttribute().getHandle(), 
                                buf, 
                                &bytes_read); 
    relay2(buf[0]);
  }//if

  //rela3
  else if(Handler->handle == bot_relay3_characteristic.getValueAttribute().getHandle()) {
    // Read the value of characteristic
    ble.readCharacteristicValue(bot_relay3_characteristic.getValueAttribute().getHandle(), 
                                buf, 
                                &bytes_read); 
    relay3(buf[0]);
  }//if
  
  //relay4
  else if(Handler->handle == bot_relay4_characteristic.getValueAttribute().getHandle()) {
    // Read the value of characteristic
    ble.readCharacteristicValue(bot_relay4_characteristic.getValueAttribute().getHandle(), 
                                buf, 
                                &bytes_read); 
    relay4(buf[0]);
  }//if
  
}

/**
 * @brief  Set BLE advertisement
 */
void setAdvertisement(void) {
  ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED | GapAdvertisingData::LE_GENERAL_DISCOVERABLE);
  // Add short name to advertisement
  ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                   (const uint8_t *)"Bot1", 
                                   4);
  
  // Add complete 128bit_uuid to advertisement
  ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                   (const uint8_t *)vng_bot_uuid, 
                                   sizeof(vng_bot_uuid));
   
  // Add complete device name to scan response data
  ble.accumulateScanResponse(GapAdvertisingData::COMPLETE_LOCAL_NAME,
                             (const uint8_t *)DEVICE_NAME, 
                             sizeof(DEVICE_NAME));
}

/*
 * @brief   Cài đặt cho BLE và timer 1s
 */
 void setupBle(){

  // Init ble
  ble.init();
  ble.onConnection(connectionCallback);
  ble.onDisconnection(disconnectionCallback);
  ble.onDataWritten(gattserverWriteCallback);
  // set advertisement
  setAdvertisement();
  // set adv_type(enum from 0)
  ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);       //un
  // add service
  ble.addService(bot_service);
  // set device name
  ble.setDeviceName((const uint8_t *)DEVICE_NAME);
  // set tx power,valid values are -40, -20, -16, -12, -8, -4, 0, 4
  ble.setTxPower(4);
  // set adv_interval, 100ms in multiples of 0.625ms.
  ble.setAdvertisingInterval(200);   
  // set adv_timeout, in seconds
  ble.setAdvertisingTimeout(0);
  // start advertising
  ble.startAdvertising();
 }

/***********************************************************************************
 *****************************CÁC HÀM CƠ BẢN CỦA ARDUINO****************************
 ***********************************************************************************/

 /*
 *  @brief      Hàm cài đặt ban đầu (Chỉ chạy 1 lần khi vừa bật nguồn)
 */
void setup() {

  //init debug via serial
  #ifdef DEBUG_SERIAL
    Serial.begin(BAUD_RATE);     
  #endif
  PRINTF("Start Bot application");
  
  //init motor
  pinMode(M1_A, OUTPUT);
  pinMode(M1_B, OUTPUT);
  pinMode(M2_A, OUTPUT);
  pinMode(M2_B, OUTPUT);
  pinMode(M3_A, OUTPUT);
  pinMode(M3_B, OUTPUT);
  pinMode(M4_A, OUTPUT);
  pinMode(M4_B, OUTPUT);
  stopCar();

  //init relay
  pinMode(RL_1, OUTPUT);
  pinMode(RL_2, OUTPUT);
  pinMode(RL_3, OUTPUT);
  pinMode(RL_4, OUTPUT);
  relay1(RELAY_OFF);
  relay2(RELAY_OFF);
  relay3(RELAY_OFF);
  relay4(RELAY_OFF);

  //init LED Status
  pinMode(LED, OUTPUT);
  offLedStatus;
  
  //init BLE
  setupBle();
  
}

 /*
 *  @brief      Hàm này chạy trong một vòng lặp vô hạn
 */
void loop() {
  //for BLE
  ble.waitForEvent();
}
