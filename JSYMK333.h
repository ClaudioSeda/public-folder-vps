#ifndef JSYMK333_H
#define JSYMK333_H

#include <Arduino.h>
#include <HardwareSerial.h>

// Define phase constants
#define JSY_PHASE_A 0
#define JSY_PHASE_B 1
#define JSY_PHASE_C 2

class JSYMK333 {
public:
    JSYMK333(HardwareSerial& serial);
    bool begin(int baud = 9600, uint8_t config = SERIAL_8N1, int rxPin = -1, int txPin = -1);
    
    void setAddress(uint8_t address);
    bool read();

    // Methods to get measurements by phase
    float getVoltage(uint8_t phase);
    float getCurrent(uint8_t phase);
    float getActivePower(uint8_t phase);
    float getReactivePower(uint8_t phase);
    float getApparentPower(uint8_t phase);
    float getPowerFactor(uint8_t phase);
    float getEnergy(uint8_t phase);

    // Methods to get total measurements
    float getTotalActivePower();
    float getTotalReactivePower();
    float getTotalApparentPower();
    float getTotalEnergy();
    float getFrequency();
    
private:
    HardwareSerial& serial;
    uint8_t address;
    
    // Data storage
    float voltages[3];
    float currents[3];
    float activePowers[3];
    float reactivePowers[3];
    float apparentPowers[3];
    float powerFactors[3];
    float energies[3];
    
    float totalActivePower;
    float totalReactivePower;
    float totalApparentPower;
    float totalEnergy;
    float frequency;

    // Helper methods for communication
    bool sendCommand(uint8_t* buffer, uint8_t length);
    bool receiveResponse(uint8_t* buffer, uint8_t expectedLength);
    uint16_t calculateCRC(uint8_t* buffer, uint8_t length);
    bool parseResponse(uint8_t* buffer, uint8_t length);
};

#endif // JSYMK333_H