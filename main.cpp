#include "MicroBit.h"

// Get a serial port running over USB back to the host PC
NRF52Pin    usbTx(ID_PIN_USBTX, MICROBIT_PIN_UART_TX, PIN_CAPABILITY_DIGITAL);
NRF52Pin    usbRx(ID_PIN_USBRX, MICROBIT_PIN_UART_RX, PIN_CAPABILITY_DIGITAL);
NRF52Serial serial(usbTx, usbRx, NRF_UARTE0);

// Create an I2C driver for the micro:bit's internal I2C bus that includes the accelerometer
NRF52Pin    i2cSdaInt(ID_PIN_SDA, MICROBIT_PIN_INT_SDA, PIN_CAPABILITY_DIGITAL);
NRF52Pin    i2cSclInt(ID_PIN_SCL, MICROBIT_PIN_INT_SCL, PIN_CAPABILITY_DIGITAL);
NRF52I2C    i2cInt(i2cSdaInt, i2cSclInt, NRF_TWIM0);

// Create an I2C driver for the micro:bit's external I2C bus exposed on the edge connector
NRF52Pin    i2cSdaExt(ID_PIN_P20, MICROBIT_PIN_EXT_SDA, PIN_CAPABILITY_DIGITAL);
NRF52Pin    i2cSclExt(ID_PIN_P19, MICROBIT_PIN_EXT_SCL, PIN_CAPABILITY_DIGITAL);
NRF52I2C    i2cExt(i2cSdaExt, i2cSclExt, NRF_TWIM1);


// api functions
extern void initOledDisplay(void);
extern void clearOledDisplay(void);
extern void setOledPixel(uint8_t, uint8_t);
extern void clearOledPixel(uint8_t, uint8_t);
extern void drawOledLine(uint8_t, uint8_t, uint8_t, uint8_t);

// // for Subtask 3
extern void graphData(uint8_t);




// Entry point is a menu that allows any subtask to be run
int main() {
    
    graphData(128);

}