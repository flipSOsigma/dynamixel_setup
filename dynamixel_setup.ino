/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <Dynamixel2Arduino.h>

// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif

#define MAX_BAUD  5
const int32_t baud[MAX_BAUD] = {57600, 115200, 1000000, 2000000, 3000000};

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

uint8_t DEFAULT_DXL_ID = 0; // Default ID of the DYNAMIXEL will be auto-filled by scanning
const uint8_t NEW_DXL_ID = 6;     // New ID to be set
const float DXL_PROTOCOL_VERSION = 1.0;

void setup() {
  uint8_t found_id = 0;

  // Use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL);

  DEBUG_SERIAL.println("Starting DYNAMIXEL Setup...");

  // Scan for the first available DYNAMIXEL device
  for (int index = 0; index < MAX_BAUD; index++) {
    dxl.begin(baud[index]);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    for (int id = 0; id < DXL_BROADCAST_ID; id++) {
      if (dxl.ping(id)) {
        found_id = id;
        DEFAULT_DXL_ID = found_id;
        DEBUG_SERIAL.print("Found DYNAMIXEL with ID: ");
        DEBUG_SERIAL.println(found_id);
        break;
      }
    }
    if (found_id > 0) {
      break;
    }
  }

  if (found_id == 0) {
    DEBUG_SERIAL.println("No DYNAMIXEL device found. Check connections and power.");
    while (1);
  }

  // Set a new ID for the found DYNAMIXEL
  dxl.torqueOff(DEFAULT_DXL_ID);
  if (dxl.setID(DEFAULT_DXL_ID, NEW_DXL_ID)) {
    DEBUG_SERIAL.print("ID has been successfully changed from ");
    DEBUG_SERIAL.print(DEFAULT_DXL_ID);
    DEBUG_SERIAL.print(" to ");
    DEBUG_SERIAL.println(NEW_DXL_ID);
    DEFAULT_DXL_ID = NEW_DXL_ID;
  } else {
    DEBUG_SERIAL.print("Failed to change ID from ");
    DEBUG_SERIAL.print(DEFAULT_DXL_ID);
    DEBUG_SERIAL.print(" to ");
    DEBUG_SERIAL.println(NEW_DXL_ID);
  }

  DEBUG_SERIAL.println("Setup complete.");
}

void loop() {
  // Main loop, can be used for additional functionalities if needed
}
