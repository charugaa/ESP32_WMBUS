#include <Arduino.h>
#include "IM871a.h"
#include "multical403.h"

#define RXD2 16 // Purple
#define TXD2 17 // Grey

// Multical 403 lengths
// 0x09 = Status response from something
// 0x25? = Short
// 0x3F = Medium
// 0x56 = Long
#define long_403 0x56
#define long_302 0x2E
#define short_403 0x3F
#define short_302 0x25
#define status 0x09

void setC1Amode();
void flipMeterID();
int comparison();
int sortTelegram();

void setAES();

bool firstRun = true;
// Find the maximum size of a wmbus frame
uint8_t dataRx[100] = {0};
bool dataReady = false;

// Device IDs
byte meterID_302[4] = {0x67, 0x99, 0x39, 0x61};
byte meterID_302_flipped[4];

byte meterID_403[4] = {0x71, 0x85, 0x58, 0x71};
byte meterID_403_flipped[4];

// AESkeys
byte AES_403[] = {0x0B, 0x5A, 0xE0, 0x50, 0x45, 0x2F, 0xBC, 0xBA, 0x7B, 0xEF, 0xC0, 0x1F, 0xCD, 0xF8, 0x69, 0x31};
byte AES_403_flipped[16] = {};
byte AES_302[] = {0xF5, 0x77, 0xA7, 0x90, 0x6A, 0x40, 0xEF, 0x88, 0xAF, 0xD1, 0x73, 0x2E, 0x9A, 0xD5, 0x6B, 0x9E};
byte AES302_flipped[16] = {};

void setup()
{
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //port.Serial(port=Port, baudrate=57600, bytesize=8, parity=port.PARITY_NONE, stopbits=1, timeout=0)
  // HardwareSerial::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin, bool invert, unsigned long timeout_ms)
  Serial2.begin(57600,
                SERIAL_8N1,
                RXD2,
                TXD2);

  flipMeterID();
  // Setting AES returns an error
  //setAES();
}

void loop()
{
  if (firstRun)
  {
    setC1Amode();
    Serial.println("Setup complete!");
    firstRun = false;
  }
  uint8_t hest;
  int i = 0;
  while (Serial2.available() > 0)
  {
    hest = Serial2.read();
    dataRx[i] = hest;
    dataReady = true;
    i++;
  }
  if (dataReady)
  {
    if (comparison())
    // A function to check length of packet is needed
    // As is, printing with trailing 0s
    {
      for (int i = 0; i < sizeof(dataRx); i++)
      {
        if (dataRx[i] != '\0')
        {
          Serial.print(" 0x");
          if (dataRx[i] <= 0xF){
            Serial.print("0");
          }
          Serial.print(dataRx[i], HEX);
        }
      }

      Serial.println();

      for (int i = 0; i < sizeof(dataRx); i++)
      {
        if (dataRx[i] != '\0')
        {
          //Serial.print(" 0x");
          if (dataRx[i] <= 0xF){
            Serial.print("0");
          }
          Serial.print(dataRx[i], HEX);
        }
      }
    }
    dataReady = false;
    // Reset array to \0's
    memset(dataRx, '\0', sizeof(dataRx));
  }
}

void setC1Amode()
{
  byte C1AMode[] = {IM871A_SERIAL_SOF, DEVMGMT_ID, DEVMGMT_MSG_SET_CONFIG_REQ, 0x03, 0x00, 2, 0x6};
  Serial2.write(C1AMode, sizeof(C1AMode));
}

void flipMeterID()
{
  //uint8_t meterID[] = {0x67, 0x99, 0x39, 0x61};
  //uint8_t meterID_flipped[];
  int y = 4;
  int x = 0;
  while (y)
  {
    meterID_302_flipped[x] = meterID_302[y - 1];
    meterID_403_flipped[x] = meterID_403[y - 1];
    AES302_flipped[x] = AES_302[y - 1];
    AES_403_flipped[x] = AES_403[y - 1];
    x++;
    y--;
  }
  int i = 16;
  int j = 0;
  while (i)
  {
    AES302_flipped[j] = AES_302[i - 1];
    AES_403_flipped[j] = AES_403[i - 1];
    j++;
    i--;
  }
}

// Returns: 2 = long tlg, 1 = short tlg, 0 = status, -1 hest!
int comparison()
{
  bool matchFound = false;
  bool matchFound2 = false;
  // Returns 1 if the message is from multical 302

  // Multical 302
  if (dataRx[7] == meterID_302_flipped[0] &&
      dataRx[8] == meterID_302_flipped[1] &&
      dataRx[9] == meterID_302_flipped[2] &&
      dataRx[10] == meterID_302_flipped[3])
  {
    matchFound = true;
  }
  // Multical 403
  if (dataRx[7] == meterID_403_flipped[0] &&
      dataRx[8] == meterID_403_flipped[1] &&
      dataRx[9] == meterID_403_flipped[2] &&
      dataRx[10] == meterID_403_flipped[3])
  {
    //matchFound2 = true;
  }

  // Implement a non-späghet way

  if (matchFound)
  {
    Serial.println("");
    Serial.println("Multical 302 message received: ");

    // Insert code that checks length of telegram and starts decrypting process
    if (dataRx[3] == long_302)
    {
      Serial.println("Long telegram detected");
    }
    else if (dataRx[3] == short_302)
    {
      Serial.println("Short telegram detected");
    }
    else if (dataRx[3] == status)
    {
      Serial.println("Something went wrong. Probably decryption.");
    }
    matchFound = false;
    return 1;
  }
  else if (matchFound2)
  {
    Serial.println("");
    Serial.println("Multical 403 message received: ");
    // Insert code that checks length of telegram and starts decrypting process
    if (dataRx[3] == long_403)
    {
      Serial.println("Long telegram detected");
    }
    else if (dataRx[3] == short_403)
    {
      Serial.println("Short telegram detected");
    }
    else if (dataRx[3] == status)
    {
      Serial.println("Something went wrong. Probably decryption.");
    }
    matchFound = false;
    return 1;
  }
  else
  {
    return 0;
  }
  return -1;
}

void setAES()
{
  // This returns an error
  byte insert_AES403[] = {DEVMGMT_ID,
                          DEVMGMT_MSG_SET_AES_DECKEY_REQ,
                          0x19, // Length is 25
                          0x00, // Table index for aeskey
                          ManID_403[0],
                          ManID_403[1],
                          meterID_403[0],
                          meterID_403[1],
                          meterID_403[2],
                          meterID_403[3],
                          version_403,
                          type_403,
                          AES_403[0],
                          AES_403[1],
                          AES_403[2],
                          AES_403[3],
                          AES_403[4],
                          AES_403[5],
                          AES_403[6],
                          AES_403[7],
                          AES_403[8],
                          AES_403[9],
                          AES_403[10],
                          AES_403[11],
                          AES_403[12],
                          AES_403[13],
                          AES_403[14],
                          AES_403[15]};
  //byte insert_AES403[] = {};

  // send 302 AES-key
  Serial2.write(insert_AES403, sizeof(insert_AES403));
  delay(100);
}

// Future dev: take arguments short/long and meter-type
// Return ???
int sortTelegram()
{
  // Sorting multical 403 and 302
  if (dataRx[3] == long_403 || dataRx[3] == long_302)
  {
    return 2;
  }
  else if (dataRx[3] == short_403 || dataRx[3] == short_302)
  {
    return 1;
  }
  else if (dataRx[3] == status)
  {
    return 0;
  }
  return -1;
}