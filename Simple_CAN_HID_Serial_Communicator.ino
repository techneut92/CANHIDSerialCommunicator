/****************************************************************************
 * This file is publically maintained and as such use it AT YOUR OWN RISK.
 * The authors of the file are in no way responsible when shit hits the fan.
 * 
 * 
 * This file is tested with the following BRANDS (Again use at your own risk):
 *  - Citroen
 *  
 *  This file is tested with the following CARS (Again use at your own risk):
 *  - Citroen C5 MKII (old radio blaupunkt rd4) Comfort CANBUS 125kbps
 ***************************************************************************/

/****************************************************************************
 * LIBRARY'S
 * 
 * Add the following library's to your project:
 * HID-project: https://www.arduino.cc/reference/en/libraries/hid-project/
 * mcp2515: https://??
 ***************************************************************************/

/****************************************************************************
 * INCLUDES - DO NOT EDIT (adding is fine though)
 ***************************************************************************/
#include <SPI.h>          // Standard Library for using SPI Communication 
#include <mcp2515.h>      // Library for using CAN Communication
#include <HID-Project.h>  //include HID_Project library
#include <HID-Settings.h> 

 /*VVVVVVVVVVVVVVVVVVVVVVVVVVVV DO NOT EDIT VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV*/
enum AMODE{                                
  LISTEN = 0x01,       // Car ==> Serial   
  WRITE = 0x02,        // Car <== Serial   
  LISTEN_RAW = 0x03,   // Car ==> Serial   
  WRITE_RAW = 0x04     // Car <== Serial   
};                                         
/*^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^*/

/****************************************************************************
 * Settings
 ***************************************************************************/
// SERIAL
#define SERIAL_BAUD_RATE    9600
#define SERIAL_MAX_BYTESIZE 100

//MCP SETTINGS
#define CAN_SPEED     CAN_125KBPS // set the canbus speed
#define MCP2515_CLOCK MCP_8MHZ    // set the clockrate usually 8mhz.

MCP2515 mcp2515(10);              // SPI CS Pin 10 
void set_mcp_filter_and_mask(){
  
}

/*****************************************************************************
 *  Below you can declare actions. the structure is:
 *  {
 *    {GPIO pin input, GPIO pin output},
 *    {Serial Identifier, Keyboard key, Mode},
 *    {CAN Message ID},
 *    {CAN Message Byte 1, Byte 2, .., Byte 8}
 *  }
 *  
 *  With each mode the actions behave differently:
 *  AMODE::LISTEN
 *    -
 *  
 *  AMODE::WRITE
 *    - 
 *  
 *  AMODE::LISTEN_RAW
 *    - 
 *  
 *  AMODE::WRITE_RAW
 *    - 
 *  
 *  
 *  Serial message construction
 *  | Identifier byte | Optional Message Byte | Stop bytes 0x00 |
 ****************************************************************************/
 
const __u16 REGISTERED_ACTIONS[][4][8] = {
//{{GPIO Pin INPUT, GPIO Pin OUTPUT}, {Serial, Keyboard, Mode}, {CAN Message ID}, {Can Message Byte 1, Byte 2, .., Byte 8}
  {{3, 0}, {0x01, MEDIA_VOLUME_UP,   AMODE::LISTEN}, {0x21F}, {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // VOLUME UP
  {{4, 0}, {0x02, MEDIA_VOLUME_DOWN, AMODE::LISTEN}, {0x21F}, {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // VOLUME DOWN
  //{{0,0}, {0x03, MEDIA_VOLUME_MUTE, AMODE::LISTEN}, {0x21F}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // MUTE
  //{{0,0}, {0x04, MEDIA_NEXT,        AMODE::LISTEN}, {0x21F}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // NEXT TRACK
  //{{0,0}, {0x05, MEDIA_PREVIOUS,    AMODE::LISTEN}, {0x21F}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // PREVIOUS TRACK
  //{{0,0}, {0x06, MEDIA_PLAY_PAUSE,  AMODE::LISTEN}, {0x21F}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // PLAY PAUSE
  {{0,0}, {0x07, 0,                 AMODE::WRITE},  {0x3E5}, {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // RADIO CLIMATE
  {{0,0}, {0x08, 0,                 AMODE::WRITE},  {0x3E5}, {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // RADIO TEL
  {{0,0}, {0x09, 0,                 AMODE::WRITE},  {0x3E5}, {0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // RADIO TRIP
  {{0,0}, {0x0A, 0,                 AMODE::WRITE},  {0x3E5}, {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // RADIO AUDIO
  {{0,0}, {0x0B, 0,                 AMODE::WRITE},  {0x3E5}, {0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}, // RADIO MODE
  {{0,0}, {0x0C, 0,                 AMODE::WRITE},  {0x3E5}, {0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00}}, // RADIO OK
  {{0,0}, {0x0D, 0,                 AMODE::WRITE},  {0x3E5}, {0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00}}, // RADIO DARK
  {{0,0}, {0x31, 0,                 AMODE::WRITE},  {0x3E5}, {0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00}}, // RADIO ESC
  {{0,0}, {0x0F, 0,                 AMODE::WRITE},  {0x3E5}, {0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00}}, // RADIO UP
  {{0,0}, {0x10, 0,                 AMODE::WRITE},  {0x3E5}, {0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00}}, // RADIO RIGHT
  {{0,0}, {0x11, 0,                 AMODE::WRITE},  {0x3E5}, {0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00}}, // RADIO DOWN
  {{0,0}, {0x12, 0,                 AMODE::WRITE},  {0x3E5}, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00}}, // RADIO LEFT
  {{0,0}, {0x30, 0,                 AMODE::WRITE},  {0x3E5}, {0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}  // RADIO MENU
};

// We once more have to register the CAN and serial IDs here. (Dynamic arrays seemed unstable and ram consuming)
// With listen ids: register the CAN IDS.
__u16 listen_ids[] = {0x21F};
// With write ids: register the serial ID
__u16 write_ids[] = {0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x31, 0x0F, 0x10, 0x11, 0x12, 0x30};
/*****************************************************************************
 *  DO NOT EDIT AFTER THIS POINT
 ****************************************************************************/
int actions_size;
int write_ids_size, listen_ids_size;
__u16 serial_msg[SERIAL_MAX_BYTESIZE];
int lp = 0;

struct can_frame canMsg;

bool interrupt = false; // TODO REMOVE?
void irqHandler() {     // TODO REMOVE?
    interrupt = true;   // TODO REMOVE?
}                       // TODO REMOVE?

void setup(){
  actions_size = sizeof(REGISTERED_ACTIONS)/sizeof(REGISTERED_ACTIONS[0]);
  mcp2515.setBitrate(CAN_SPEED,MCP2515_CLOCK);
  Consumer.begin(); // start HID keyboard
  Serial.begin(SERIAL_BAUD_RATE);
  while(!Serial) {} // TODO REMOVE
  SPI.begin();

  mcp2515.setNormalMode();  //Sets CAN at normal mode
  //mcp2515.setListenOnlyMode();
  //mcp2515.setLoopbackMode();  
  
  //setup_pins();
  write_ids_size = sizeof(write_ids)/sizeof(write_ids[0]);
  listen_ids_size = sizeof(listen_ids)/sizeof(listen_ids[0]);

}

bool test = false;
void do_test(){
  if (!test) {
    canMsg.can_id = 0x3E5;
    canMsg.can_dlc = 8;
    canMsg.data[0] = 0x40;
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x00;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
    test = true;
  }
  else{
    canMsg.can_id = 0x3E5;
    canMsg.can_dlc = 8;
    canMsg.data[0] = 0x00;
    canMsg.data[1] = 0x00;
    canMsg.data[2] = 0x10;
    canMsg.data[3] = 0x00;
    canMsg.data[4] = 0x00;
    canMsg.data[5] = 0x00;
    canMsg.data[6] = 0x00;
    canMsg.data[7] = 0x00;
    test = false;
  }
}

void loop(){
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // check if we got any actions in listen mode
    Serial.print("-");
    if (in_listen_ids(canMsg.can_id)) { trigger_listen(); }
  }      
  else { mcp2515.clearInterrupts(); }
  if (lp % 10000 == 0) { mcp2515.clearInterrupts(); } // TODO FIX. Is it my chip? without i can only receive a few messages before locking up
  
  if (Serial.available() > 0){  
    do_test();  
    // check if we got any actions in write mode
    fill_serial_data_until_stop();
    if (in_write_ids(serial_msg[0])) { trigger_write(); }
  }
  lp++;
  delay(5);
}

/*****************************************
 * MAIN FUNCTIONS
 *****************************************/
void fill_serial_data_until_stop(){ // FIX FOR DATA
  for (int i = 0; i < SERIAL_MAX_BYTESIZE; i++) { serial_msg[i] = 0x00; }
  __u16 incoming_byte;
  int counter = 0;
  incoming_byte = Serial.read();
  if (incoming_byte == 0) { return; }
  serial_msg[counter] = incoming_byte;
  counter++;
}
 
void trigger_listen(){
  for (int z = 0; z < actions_size; z++){
    if (REGISTERED_ACTIONS[z][2][0] == canMsg.can_id && REGISTERED_ACTIONS[z][1][2] == AMODE::LISTEN && validate_can_msg(z)) {       
      // do serial action
      Serial.print(REGISTERED_ACTIONS[z][1][0], HEX);
      // do gpio action
      run_listen_gpio(z);
      // do keyboard action
      if (REGISTERED_ACTIONS[z][1][1] != 0) { Consumer.write(REGISTERED_ACTIONS[z][1][1]); }
    }
  }
}

void trigger_write(){
  for (int z = 0; z < actions_size; z++){
    if (REGISTERED_ACTIONS[z][1][0] == serial_msg[0] && REGISTERED_ACTIONS[z][1][2] == AMODE::WRITE) { 
      // do canbus action
      canMsg.can_id = REGISTERED_ACTIONS[z][2][0];
      canMsg.data[0] = REGISTERED_ACTIONS[z][3][0];
      canMsg.data[1] = REGISTERED_ACTIONS[z][3][1];
      canMsg.data[2] = REGISTERED_ACTIONS[z][3][2];
      canMsg.data[3] = REGISTERED_ACTIONS[z][3][3];
      canMsg.data[4] = REGISTERED_ACTIONS[z][3][4];
      canMsg.data[5] = REGISTERED_ACTIONS[z][3][5];
      canMsg.data[6] = REGISTERED_ACTIONS[z][3][6];
      canMsg.data[7] = REGISTERED_ACTIONS[z][3][7];
      mcp2515.sendMessage(&canMsg);
      //Serial.print(0x00);
      // do gpio action
      //run_write_gpio(z);
      
      // do keyboard action
      if (REGISTERED_ACTIONS[z][1][1] != 0) { Consumer.write(REGISTERED_ACTIONS[z][1][1]); }
    }
  }
}

/******************************************
 * LISTEN FUNCTIONS
 *****************************************/
bool validate_can_msg(int id){
  for(int i = 0; i < 8; i++) { 
    if (REGISTERED_ACTIONS[id][3][i] != 0x00 && REGISTERED_ACTIONS[id][3][i] != canMsg.data[i]) { 
      return false; 
    } 
  }
  return true;
}

void run_listen_gpio(int id){
  
}
/******************************************
 * WRITE FUNCTIONS
 *****************************************/
void run_write_gpio(int id){
  
}

/*****************************************
 * SETUP FUNCTIONS
 *****************************************/
void setup_pins(){
  for (int i = 0; i < actions_size; i++){
    int pin_num_input = REGISTERED_ACTIONS[i][0][0];
    int pin_num_output = REGISTERED_ACTIONS[i][0][1];
    if (pin_num_input != 0) { pinMode(pin_num_input, INPUT); }
    if (pin_num_output != 0){ pinMode(pin_num_input, INPUT); }
  }
}


/*********************************************
 * HELPERS
 ********************************************/
bool in_write_ids(__u16 id){ for(int i = 0; i < write_ids_size; i++){ if (write_ids[i] == id) { return true; } } return false; }
bool in_listen_ids(__u16 id){ for(int i = 0; i < listen_ids_size; i++){ if (listen_ids[i] == id) { return true; } } return false; }
