#include <SoftwareSerial.h>

// Pins we use for MCP2004
const int faultPin = 6;
const int tx = 1;
const int rx = 0;

const int left_btn = 3;
const int right_btn = 4; 
const int menuse_btn = 5;
const int hazard_btn = 6; 
const int horn_btn = 7;
const int highbeam_btn = 8;
const int abs_btn = 9;
const int menuad_btn = 10;
const int cancel_btn=11;

const int spin2 = 2;

boolean read_byte = false;
boolean s2 = false;

byte readbuffer[64];
int i;
int buffer_index = 0;
int buffer_max = 64;
int cksum;
long lastrx;
long lasttx;

int stt=0;
int btnState_L = 0;
int btnState_R = 0;
int btnState_MnS = 0;
int btnState_MnA = 0;
int btnState_Hz = 0;
int btnState_Hn = 0;
int btnState_Hb = 0;
int btnState_ABS = 0;
int btnState_Cl=0;

void setup() {

  pinMode(abs_btn, INPUT);
  pinMode(left_btn, INPUT);
  pinMode(right_btn, INPUT);
  pinMode(menuse_btn, INPUT);
  pinMode(menuad_btn, INPUT);
  pinMode(hazard_btn, INPUT);
  pinMode(horn_btn, INPUT);
  pinMode(highbeam_btn, INPUT);
  pinMode(cancel_btn,INPUT);

  
  // initialize buffer array to 0's
  memset(readbuffer, 0, sizeof(readbuffer));

  // Open serial communications to host (PC) and wait for port to open:
  Serial.begin(115200, SERIAL_8E1);

  // Set high to enable TX on TJA1021
  pinMode(tx, OUTPUT);
  digitalWrite(tx, HIGH);

  lastrx = millis();
  lasttx = millis();
}

void loop() {
  
  btnState_Hz = digitalRead(hazard_btn);
  btnState_L = digitalRead(left_btn);
  btnState_R = digitalRead(right_btn);
  btnState_MnA = digitalRead(menuad_btn);
  btnState_MnS = digitalRead(menuse_btn);
  btnState_Hb = digitalRead(highbeam_btn);
  btnState_Hn = digitalRead(horn_btn);
  btnState_ABS = digitalRead(abs_btn);
  btnState_Cl= digitalRead(cancel_btn);

  if (btnState_Cl==1) stt=9;
  if (btnState_MnA==1) stt=8;
  if (btnState_Hz==1) stt=7;
  if (btnState_L==1) stt=6;
  if (btnState_R==1) stt=5;
  if (btnState_MnS==1) stt=4;
  if (btnState_Hb==1) stt=3;
  if (btnState_Hn==1) stt=2;
  if (btnState_ABS==1) stt=1;

  delayMicroseconds(1000);
  lasttx = millis();

  if ((millis() - lastrx) > 15) {
    memset(readbuffer, 0, sizeof(readbuffer));
    buffer_index = 0;
    read_byte = false;
    buffer_max = 64;
    lastrx = millis();
    return;
  }

  if (Serial.available()) {
    readbuffer[buffer_index] = Serial.read();
    read_byte = true;
  }

  if (read_byte) {
    if (buffer_index == 1) {
      buffer_max = readbuffer[buffer_index] + 2;
      cksum = readbuffer[0] ^ readbuffer[1];
    } else if ((buffer_index > 1 ) && (buffer_index < buffer_max)) {
      cksum = cksum ^ readbuffer[buffer_index];
    }
  }

  if (buffer_index == (buffer_max - 1)) {
    if (cksum == 0) {
      if (s2) {
        digitalWrite(spin2, HIGH);
      }
      else {
        digitalWrite(spin2, LOW);
      }
      s2 = !s2;
      Serial.print("Good message: ");
      //Serial.print(millis());
      Serial.print(" S:");
      Serial.print(readbuffer[0], HEX);
      Serial.print( " L:");
      Serial.print(readbuffer[1], DEC);
      Serial.print( " ");
      for (i = 2; i < buffer_max; i++) {
        Serial.print(readbuffer[i], HEX);
        Serial.print(" ");
      }
    } else {
      Serial.print("Invalid message. cksum: ");
      Serial.println(cksum, HEX);
      for (i = 0; i < buffer_max; i++) {
        Serial.print(readbuffer[i], HEX);
        Serial.print(" ");
      }
    }
    
    handleMessage(stt);
    Serial.println();
    // Empty the buffer
    memset(readbuffer, 0, sizeof(readbuffer));
    buffer_index = 0;
    read_byte = false;
    lastrx = millis();
  }

  // Increment index if we put something into the buffer
  if (read_byte == true) {
    read_byte = false;
    buffer_index++;
    lastrx = millis();
  }

}


void handleMessage(byte messageType) { 
  byte default_code_d7[15] = {0x87,0x08,0x84,0x0b,0x81,0x0e,0x82,0x0d,0x8b,0x04,0x88,0x07,0x8d,0x02,0x8e};
  byte default_code_d8[15] = {0x18,0x96,0x19,0x91,0x1a,0x8c,0x17,0x8b,0x0c,0x92,0x0d,0x8d,0x06,0x90,0x03};

  byte Cl_code_d7[15] ={0xd7,0x58,0xd4,0x5b,0xd1,0x5e,0xd2,0x5d,0xdb,0x54,0xd8,0x57,0xdd,0x52,0xde};
  byte Cl_code_d8[15] ={0x87,0x06,0x88,0x01,0x89,0xfb,0x86,0xfa,0x7b,0x02,0x7c,0xfc,0x75,0x00,0x72};
  
  byte L_code_d7[15] ={0x93,0x1c,0x90,0x1f,0x95,0x1a,0x96,0x19,0x9f,0x10,0x9c,0x13,0x99,0x16,0x9a};
  byte L_code_d8[15] ={0xfb,0x72,0xfc,0x6d,0xf5,0x70,0xf2,0x6f,0xe7,0x76,0xe8,0x71,0xe9,0x6c,0xe6};
  
  byte R_code_d7[15] ={0xaf,0x20,0xac,0x23,0xa9,0x26,0xaa,0x25,0xa3,0x2c,0xa0,0x2f,0xa5,0x2a,0xa6};
  byte R_code_d8[15] ={0xcf,0x5e,0xd0,0x59,0xd1,0x54,0xce,0x53,0xd3,0x4a,0xd4,0x45,0xcd,0x48,0xca};
  
  byte MnS_code_d7[15] ={0x46,0xc9,0x45,0xca,0x40,0xcf,0x43,0xcc,0x4a,0xc5,0x49,0xc6,0x4c,0xc3,0x4f};
  byte MnS_code_d8[15] ={0x39,0xb4,0x38,0xb1,0x3b,0xaa,0x36,0xab,0x2d,0xb0,0x2c,0xad,0x27,0xae,0x22};

 // byte MnA_code_d7[15] ={0x69,0xe6,0x6a,0xe5,0x6f,0xe0,0x6c,0xe3,0x65,0xea,0x66,0xe9,0x63,0xec,0x60};
 // byte MnA_code_d8[15] ={0x26,0xa7,0x23,0xa6,0x1c,0xa9,0x1d,0xa4,0x22,0x9b,0x1f,0x9a,0x20,0x95,0x21};
  
  byte Hz_code_d7[15] ={0x82,0x0d,0x81,0x0e,0x84,0x0b,0x87,0x08,0x8e,0x01,0x8d,0x02,0x88,0x07,0x8b};
  byte Hz_code_d8[15] ={0x19,0x8d,0x18,0x8a,0x13,0x8b,0x0e,0x8c,0x05,0x91,0x04,0x8e,0x07,0x87,0x02};
  
  byte Hb_code_d7[15] ={0x53,0xdc,0x50,0xdf,0x55,0xda,0x56,0xd9,0x5f,0xd0,0x5c,0xd3,0x59,0xd6,0x5a};
  byte Hb_code_d8[15] ={0x4a,0xbf,0x4b,0xba,0x4c,0xbd,0x41,0xbc,0x36,0xc3,0x37,0xbe,0x38,0xb9,0x35};
  
  byte Hn_code_d7[15] ={0x27,0xa8,0x24,0xab,0x21,0xae,0x22,0xad,0x2b,0xa4,0x28,0xa7,0x2d,0xa2,0x2e};
  byte Hn_code_d8[15] ={0xf7,0x75,0xf8,0x70,0xf9,0x6b,0xf6,0x6a,0xeb,0x71,0xec,0x6c,0xe5,0x6f,0xe2};

  byte abs_code_d7[15] = {0x0b,0x84,0x08,0x87,0x0d,0x82,0x0e,0x81,0x07,0x88,0x04,0x8b,0x01,0x8e,0x02};
  byte abs_code_d8[15] = {0x92,0x18,0x93,0x13,0x8c,0x16,0x89,0x15,0x8e,0x0c,0x8f,0x07,0x90,0x02,0x8d};
  
  switch (messageType) {
    case 0: // default
      Serial.println("Sending default code: ");
      sendMessage(0x0,0x0,0x0, default_code_d7, default_code_d8);
      break;
    case 1: //ABS button
      Serial.println("Sending abs status:");
      sendMessage(0x0,0x02,0x0, abs_code_d7, abs_code_d8);
      break;
    case 2: //Horn button
      Serial.println("Sending Horn status:");
      sendMessage(0x0,0x80,0x0, Hn_code_d7,Hn_code_d8);
      break;
    case 3: //High beam button
      Serial.println("Sending High Beam  status:");
      sendMessage(0x0,0x0,0x02, Hb_code_d7,Hb_code_d8);
      break;
    case 4: //Hazard button
      Serial.println("Sending Hazard status:");
      sendMessage(0x00,0x04,0x00, Hz_code_d7,Hz_code_d8);
      break;
    case 5: //Menu_Select button
      Serial.println("Sending Menu select status:");
      sendMessage(0x0,0x0,0x20, MnS_code_d7,MnS_code_d8);
      break;
    case 6: //Right button
      Serial.println("Sending right status:");
      sendMessage(0x0,0x20,0x0, R_code_d7,R_code_d8);
      break;
    case 7: //Left button
      Serial.println("Sending left status:");
      sendMessage(0x0,0x10,0x0, L_code_d7,L_code_d8);
      break;
   // case 8: //Menu_Adjust button
  //    Serial.println("Sending Menu adjust status:");
   //   sendMessage(0x0,0x0,0x10, MnA_code_d7,                      abs_code_d8);
   //   break;
    case 9: //Cancel button
      Serial.println("Sending Cancel status:");
      sendMessage(0x0,0x40,0x0, Cl_code_d7,Cl_code_d8);
      break;
  }
 }

void sendMessage(byte d0,byte d1,byte d2, byte message_data1[], byte message_data2[]) {
  int d4=0x30;
  //byte message_cksum = gen_cksum(message_data);
  for (int k = 0; k < 15 ; k++) {
    Serial.write(d0);
    Serial.write(d1);
    Serial.write(d2);
    Serial.write(0xfc);
    Serial.write(d4);d4++;//m=d4;
    Serial.write(0x00);
    Serial.write(0x1f);
    Serial.write(message_data1[k]);
    Serial.write(message_data2[k]);
    delay(45);
  }
 
}
//Not used
byte gen_cksum(const byte message[]) {
  byte cksum = 0x00;
  for (int i = 1; i <= message[0]; i++) {
    cksum = cksum ^ message[i];
  }
  return cksum;
}
