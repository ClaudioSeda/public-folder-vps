#include "JSYMK333.h"

// Constructor
JSYMK333::JSYMK333(HardwareSerial& serial) : serial(serial) {
    address = 1; // Default Modbus address for JSY-MK-333
    
    // Initialize data arrays
    for (int i = 0; i < 3; i++) {
        voltages[i] = 0.0;
        currents[i] = 0.0;
        activePowers[i] = 0.0;
        reactivePowers[i] = 0.0;
        apparentPowers[i] = 0.0;
        powerFactors[i] = 0.0;
        energies[i] = 0.0;
    }
    
    totalActivePower = 0.0;
    totalReactivePower = 0.0;
    totalApparentPower = 0.0;
    totalEnergy = 0.0;
    frequency = 0.0;
}

bool JSYMK333::begin(int baud, uint8_t config, int rxPin, int txPin) {
    // Configuration already handled in setup() of main code
    return true;
}

void JSYMK333::setAddress(uint8_t addr) {
    address = addr;
}

// Main read method to get all data from the device
bool JSYMK333::read() {
    // Buffer for Modbus RTU command
    uint8_t cmd[8];
    uint8_t response[60];
    
    // Prepare read command to get all registers starting from address 0
    cmd[0] = address;     // Device address
    cmd[1] = 0x03;        // Function code (read holding registers)
    cmd[2] = 0x00;        // Starting address high byte
    cmd[3] = 0x00;        // Starting address low byte
    cmd[4] = 0x00;        // Number of registers high byte
    cmd[5] = 0x1A;        // Number of registers low byte (26 registers)
    
    // Calculate CRC and add to command
    uint16_t crc = calculateCRC(cmd, 6);
    cmd[6] = crc & 0xFF;  // CRC low byte
    cmd[7] = crc >> 8;    // CRC high byte
    
    // Send command
    if (!sendCommand(cmd, 8)) {
        return false;
    }
    
    // Receive response (expected 57 bytes: addr + func + len + 52 data bytes + 2 CRC)
    if (!receiveResponse(response, 57)) {
        return false;
    }
    
    // Parse response and update stored values
    return parseResponse(response, 57);
}

// Helper method to send command to device
bool JSYMK333::sendCommand(uint8_t* buffer, uint8_t length) {
    // Clear any existing data in the serial buffer
    while (serial.available()) {
        serial.read();
    }
    
    // Send command
    serial.write(buffer, length);
    serial.flush();
    
    return true;
}

// Helper method to receive and validate response
bool JSYMK333::receiveResponse(uint8_t* buffer, uint8_t expectedLength) {
    unsigned long startTime = millis();
    uint8_t bytesRead = 0;
    
    // Wait for complete response with timeout
    while ((bytesRead < expectedLength) && ((millis() - startTime) < 1000)) {
        if (serial.available()) {
            buffer[bytesRead] = serial.read();
            bytesRead++;
        }
        delay(1);
    }
    
    // Check if we got enough bytes
    if (bytesRead != expectedLength) {
        return false;
    }
    
    // Verify CRC
    uint16_t receivedCrc = (buffer[bytesRead - 1] << 8) | buffer[bytesRead - 2];
    uint16_t calculatedCrc = calculateCRC(buffer, bytesRead - 2);
    
    return receivedCrc == calculatedCrc;
}

// Helper method to calculate Modbus CRC16
uint16_t JSYMK333::calculateCRC(uint8_t* buffer, uint8_t length) {
    uint16_t crc = 0xFFFF;
    
    for (uint8_t pos = 0; pos < length; pos++) {
        crc ^= buffer[pos];
        
        for (uint8_t i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    
    return crc;
}

// Method to parse response data and update stored values
bool JSYMK333::parseResponse(uint8_t* buffer, uint8_t length) {
    if (buffer[0] != address || buffer[1] != 0x03) {
        return false;
    }
    
    // Calculate how many data bytes
    uint8_t dataBytes = buffer[2];
    if (dataBytes != length - 5) {  // addr + func + len + data + 2 CRC
        return false;
    }
    
    // Extract data from the response
    // Starting from buffer[3] (first data byte)
    
    // Phase A
    voltages[0] = ((buffer[3] << 8) | buffer[4]) * 0.1f;  // Voltage scale factor 0.1
    currents[0] = ((buffer[5] << 8) | buffer[6]) * 0.01f; // Current scale factor 0.01
    activePowers[0] = ((buffer[7] << 8) | buffer[8]) * 0.1f; // Power scale factor 0.1
    
    // Phase B
    voltages[1] = ((buffer[9] << 8) | buffer[10]) * 0.1f;
    currents[1] = ((buffer[11] << 8) | buffer[12]) * 0.01f;
    activePowers[1] = ((buffer[13] << 8) | buffer[14]) * 0.1f;
    
    // Phase C
    voltages[2] = ((buffer[15] << 8) | buffer[16]) * 0.1f;
    currents[2] = ((buffer[17] << 8) | buffer[18]) * 0.01f;
    activePowers[2] = ((buffer[19] << 8) | buffer[20]) * 0.1f;
    
    // Total values
    totalActivePower = ((buffer[21] << 8) | buffer[22]) * 0.1f;
    totalReactivePower = ((buffer[23] << 8) | buffer[24]) * 0.1f;
    totalApparentPower = ((buffer[25] << 8) | buffer[26]) * 0.1f;
    
    // Power factors (scale factor 0.001)
    powerFactors[0] = ((buffer[27] << 8) | buffer[28]) * 0.001f;
    powerFactors[1] = ((buffer[29] << 8) | buffer[30]) * 0.001f;
    powerFactors[2] = ((buffer[31] << 8) | buffer[32]) * 0.001f;
    
    // Frequency (scale factor 0.01)
    frequency = ((buffer[33] << 8) | buffer[34]) * 0.01f;
    
    // Energies (kWh with scale factor 0.01)
    energies[0] = ((buffer[35] << 24) | (buffer[36] << 16) | (buffer[37] << 8) | buffer[38]) * 0.01f;
    energies[1] = ((buffer[39] << 24) | (buffer[40] << 16) | (buffer[41] << 8) | buffer[42]) * 0.01f;
    energies[2] = ((buffer[43] << 24) | (buffer[44] << 16) | (buffer[45] << 8) | buffer[46]) * 0.01f;
    
    // Total energy
    totalEnergy = ((buffer[47] << 24) | (buffer[48] << 16) | (buffer[49] << 8) | buffer[50]) * 0.01f;
    
    return true;
}

// Accessor methods for phase measurements
float JSYMK333::getVoltage(uint8_t phase) {
    if (phase < 3) return voltages[phase];
    return 0.0;
}

float JSYMK333::getCurrent(uint8_t phase) {
    if (phase < 3) return currents[phase];
    return 0.0;
}

float JSYMK333::getActivePower(uint8_t phase) {
    if (phase < 3) return activePowers[phase];
    return 0.0;
}

float JSYMK333::getReactivePower(uint8_t phase) {
    if (phase < 3) return reactivePowers[phase];
    return 0.0;
}

float JSYMK333::getApparentPower(uint8_t phase) {
    if (phase < 3) return apparentPowers[phase];
    return 0.0;
}

float JSYMK333::getPowerFactor(uint8_t phase) {
    if (phase < 3) return powerFactors[phase];
    return 0.0;
}

float JSYMK333::getEnergy(uint8_t phase) {
    if (phase < 3) return energies[phase];
    return 0.0;
}

// Accessor methods for total measurements
float JSYMK333::getTotalActivePower() {
    return totalActivePower;
}

float JSYMK333::getTotalReactivePower() {
    return totalReactivePower;
}

float JSYMK333::getTotalApparentPower() {
    return totalApparentPower;
}

float JSYMK333::getTotalEnergy() {
    return totalEnergy;
}

float JSYMK333::getFrequency() {
    return frequency;
}