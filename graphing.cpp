#include "MicroBit.h"

//
// references to external objects
//
extern NRF52Serial serial;  // serial object for displaying type of sensor and debugging 
extern NRF52I2C    i2cInt;  // internal I2C object for talking to accelerometer
extern NRF52I2C    i2cExt;  // external I2C object for talking to OLED display


//###################################################################################

// the bitmap in memory for the OLED pixels
// you must specify the type and the size, but keep the name the same

// frame buffer for the 128x64 Oled display
uint8_t oledDisplayFrameBuffer[128][64/8] = {0};

// I2C address for the OLED display
const uint8_t OLED_I2C_ADDRESS = 0x3C; 

// modes for communicating with the Oled 
const uint8_t DATA_MODE = 0x40;           
const uint8_t COMMAND_MODE = 0x80;

const uint8_t WIDTH = 128;         // OLED width in pixels
const uint8_t HEIGHT = 64;         // OLED height in pixels
const uint8_t PAGES = HEIGHT / 8;  // OLED has 8 rows of 8 pixels (64 / 8 = 8 pages)



/*
 * Function: clearOledDisplay
 * ---------------------------
 * Clears the OLED display by resetting the frame buffer;
 * the function sets all pixels to 'off' (0) and clears the display.
 *
 * Parameters:
 * - None
 * 
 * Returns:
 * - None
*/
void clearOledDisplay() {    
  memset(oledDisplayFrameBuffer, 0, sizeof(oledDisplayFrameBuffer));

  // set column and page addressing mode to start from (0,0)
  uint8_t addressingCommands[] = {
    0x20, 0x00,             // Set addressing mode to horizontal
    0x21, 0x00, WIDTH - 1,  // Set column address (0 to WIDTH - 1)
    0x22, 0x00, PAGES - 1   // Set page address (0 to PAGES - 1)
  };

  // send initialisation commands to set the addressing mode
  for (size_t i = 0; i < sizeof(addressingCommands); i++) {
    uint8_t cmd[2] = {COMMAND_MODE, addressingCommands[i]};
    i2cExt.write(OLED_I2C_ADDRESS << 1, cmd, 2);
  }
  
  // clear the screen
  for (uint16_t i = 0; i < WIDTH * PAGES; i++) {
    uint8_t data[2] = {DATA_MODE, 0x00}; 
    i2cExt.write(OLED_I2C_ADDRESS << 1, data, 2);
  }

  
}


/*
 * Function: initOledDisplay
 * -------------------------
 * Initialises the OLED display by setting up the required parameters
 *
 * Parameters:
 * - None
 * 
 * Returns:
 * - None
 */
void initOledDisplay() {  
  // set I2C frequency to 400kHz for faster communication
  i2cExt.setFrequency(400000);

  const uint8_t initCommands[] = {
    0xA8, 0x3F, // Set MUX Ratio
    0xD3, 0x00, // Set Display Offset
    0x40,       // Set Display Start Line
    0xA1,       // Set Segment Re-map
    0xC8,       // Set COM Output Scan Direction
    0xDA, 0x12, // Set COM Pins hardware configuration
    0x81, 0x7F, // Set Contrast Control
    0xA4,       // Disable Entire Display On
    0xA6,       // Set Normal Display
    0xD5, 0x80, // Set Osc Frequency
    0x8D, 0x14, // Enable charge pump regulator
    0xAF        // Display On
  };

  // send the initialisation commands to the Oled
  for (size_t i = 0; i < sizeof(initCommands); i++) {
    uint8_t cmd[2] = {COMMAND_MODE, initCommands[i]}; 
    i2cExt.write(OLED_I2C_ADDRESS << 1, cmd, 2);
  }

  // set column and page addressing mode to start from (0,0)
  uint8_t addressingCommands[] = {
    0x20, 0x00,             // Set addressing mode to horizontal
    0x21, 0x00, WIDTH - 1,  // Set column address (0 to WIDTH - 1)
    0x22, 0x00, PAGES - 1   // Set page address (0 to PAGES - 1)
  };

  // send addressing commands to set the addressing mode
  for (size_t i = 0; i < sizeof(addressingCommands); i++) {
    uint8_t cmd[2] = {COMMAND_MODE, addressingCommands[i]};
    i2cExt.write(OLED_I2C_ADDRESS << 1, cmd, 2);
  }
  
  // clear the display after initialisation to ensure it is blank
  clearOledDisplay();
}


/*
 * Function: setOledPixel
 * ----------------------
 * Sets a pixel at the specified coordinates (x, y) on the OLED display.
 * This function updates the framebuffer and sends the appropriate I2C 
 * commands to turn on the specified pixel.
 *
 * Parameters:
 * - x: The x-coordinate (horizontal position) of the pixel.
 * - y: The y-coordinate (vertical position) of the pixel.
 * 
 * Returns:
 * - None
*/
void setOledPixel(uint8_t x, uint8_t y) {
  // check if pixel is within bounds
  if (x >= 128 || y >= 64) {return;}

  // determine page (row of 8 pixels)
  uint8_t page = y / 8;

  // determine bit within the page
  uint8_t bit = y % 8;

  // update frame buffer
  oledDisplayFrameBuffer[x][page] |= (1 << bit);

  // set column and page addressing to the pixel's page and colums
  uint8_t addressingCommands[] = {
    0x20, 0x00,             // Set addressing mode to horizontal
    0x21, x, x,             // Set column address to x
    0x22, page, page        // Set page address to page
  };
  // Send addressing commands to set the addressing mode
  for (size_t i = 0; i < sizeof(addressingCommands); i++) {
    uint8_t cmd[2] = {COMMAND_MODE, addressingCommands[i]};
    i2cExt.write(OLED_I2C_ADDRESS << 1, cmd, 2);
  }

  // send pixel data to the OLED
  uint8_t data[2] = {DATA_MODE, oledDisplayFrameBuffer[x][page]};
  i2cExt.write(OLED_I2C_ADDRESS << 1, data, 2);
}

/*
 * Function: clearOledPixel
 * ------------------------
 * Clears a pixel at the specified coordinates (x, y) on the OLED display.
 * This function updates the framebuffer and sends the appropriate I2C 
 * commands to turn off the specified pixel.
 *
 * Parameters:
 * - x: The x-coordinate (horizontal position) of the pixel.
 * - y: The y-coordinate (vertical position) of the pixel.
 * 
 * Returns:
 * - None
*/
void clearOledPixel(uint8_t x, uint8_t y) {
  // check that pixel is within bound
  if (x >= 128 || y >= 64) { return;}

  // determine page
  uint8_t page = y / 8;

  // dtermine the bit within the page
  uint8_t bit = y % 8;

  // update the frame buffer
  oledDisplayFrameBuffer[x][page] &= ~(1 << bit);

  // set column and page addressing to the pixel's page and colums
  uint8_t addressingCommands[] = {
    0x20, 0x00,             // Set addressing mode to horizontal
    0x21, x, x,             // Set column address to x
    0x22, page, page        // Set page address to page
  };
  // send addressing commands to set the addressing mode
  for (size_t i = 0; i < sizeof(addressingCommands); i++) {
    uint8_t cmd[2] = {COMMAND_MODE, addressingCommands[i]};
    i2cExt.write(OLED_I2C_ADDRESS << 1, cmd, 2);
  }

  // send the updated pixel data to the OLED
  uint8_t data[2] = {DATA_MODE, oledDisplayFrameBuffer[x][page]};
  i2cExt.write(OLED_I2C_ADDRESS << 1, data, 2);
}

/*
 * Function: drawOledLine
 * ----------------------
 * Draws a line on the OLED display using the Bresenham line algorithm.
 * The line is drawn from the starting coordinates (x_start, y_start) 
 * to the ending coordinates (x_end, y_end) by setting the appropriate pixels 
 * on the display.
 *
 * Parameters:
 * - x_start: The starting x-coordinate of the line.
 * - y_start: The starting y-coordinate of the line.
 * - x_end: The ending x-coordinate of the line.
 * - y_end: The ending y-coordinate of the line.
 * 
 * Returns:
 * - None
 */
void drawOledLine(uint8_t x_start, uint8_t y_start, uint8_t x_end, uint8_t y_end) {
  // ignore lines that go out of bounds
  if (x_start >= 128 || y_start >= 64 || x_end >= 128 || y_end >= 64) {return;}

  int dx = abs(x_end - x_start);
  int dy = abs(y_end - y_start);
  int sx = (x_start < x_end) ? 1 : -1;
  int sy = (y_start < y_end) ? 1 : -1;

  int err = dx - dy;  // initial error term

  while (true) {
    // set pixel + check for termination
    setOledPixel(x_start, y_start);
    if (x_start == x_end && y_start == y_end) break;

    // update coordinates based on error term
    int e2 = err << 1;  // e2 = 2 * err
    if (e2 > -dy) {
      err -= dy;
      x_start += sx;
    }
    if (e2 < dx) {
      err += dx;
      y_start += sy;
    }
  }
}




//###################################################################################

// Enum to define the different operating modes
enum Mode {MODE_A, MODE_B};

// gloabl variable to track the current mode
static volatile Mode currentMode;

// variable to track the current column being drawn on the Oled
static uint8_t currentColumn = 0;  // Start drawing at the first column

// varaible represents the middle of the Oled (used for jerk mode)
#define MID_Y HEIGHT/2

/*
 * Function: handleButtonA
 * -----------------------
 * Handles the press event of Button A. This function clears the OLED display, 
 * resets the current column to 0, and switches to Mode A (accelerometer). It 
 * also clears the MicroBit display and sets specific pixels to indicate the mode.
 *
 * Parameters:
 * - None
 *
 * Returns:
 * - None
*/
void handleButtonA(){
  clearOledDisplay();   // clear the oled display
  currentColumn = 0;    // reset the current column to 0

  currentMode = MODE_A; // switch to Mode A (accelerometer)
  
  clearMicroBitDisplay(); // clear the microbit display
  
  // set the corresponding pixels to indicate Mode A
  setMicroBitPixel(0,0);
  setMicroBitPixel(0,1);
  setMicroBitPixel(0,2);
  setMicroBitPixel(0,3);
  setMicroBitPixel(0,4);
  setMicroBitPixel(2,2);
}

/*
 * Function: handleButtonB
 * -----------------------
 * Handles the press event of Button B. This function clears the OLED display, 
 * resets the current column to 0, and switches to Mode B (jerk). It also clears 
 * the MicroBit display and sets specific pixels to indicate the mode.
 *
 * Parameters:
 * - None
 *
 * Returns:
 * - None
 */
void handleButtonB(){
  clearOledDisplay();       // clear the oled display
  currentColumn = 0;        // reset the current column to 0

  currentMode = MODE_B;     // switch to Mode B (jerk)
  
  clearMicroBitDisplay();   // clear the microbit display

  // set the corresponding pixels to indicate Mode B
  setMicroBitPixel(4,0);
  setMicroBitPixel(4,1);
  setMicroBitPixel(4,2);
  setMicroBitPixel(4,3);
  setMicroBitPixel(4,4);
  setMicroBitPixel(2,2);
}

/*
 * Function: GPIOTE_IRQHandler
 * ---------------------------
 * Interrupt handler for GPIO events. This function checks for events from 
 * Button A or Button B and calls the corresponding handler 
 * functions to handle the button presses.
 *
 * Parameters:
 * - None
 *
 * Returns:
 * - None
 */
void GPIOTE_IRQHandler(){
  // handle Button A interrupt
  
  if(NRF_GPIOTE->EVENTS_IN[0]){       // check Button A event flag
    NRF_GPIOTE->EVENTS_IN[0] = 0;     // clear Button A event flag    

    // handle Button A press
    handleButtonA();
  }

  // handle Button B interrupt
  if(NRF_GPIOTE->EVENTS_IN[1]){       // check Button B event flag
    NRF_GPIOTE->EVENTS_IN[1] = 0;     // clear Button B event flag

    // handle Button B press
    handleButtonB();

  }
}

/*
 * Function: configureButtonInterrupts
 * -----------------------------------
 * Configures the GPIO pins for Button A and Button B. 
 * Function also enables interrupts for the buttons.
 *
 * Parameters:
 * - None
 *
 * Returns:
 * - None
 */
void configureButtonInterrupts(){
  // configure button A (connected to P0.14)
  NRF_GPIO->PIN_CNF[14] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |        // Set Button A pin as input
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Connect input
                          (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);      // Enable pull-up resistor

  // configure button B (connected to P0.23)
  NRF_GPIO->PIN_CNF[23] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |        // Set Button B pin as input
                          (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |  // Connect input
                          (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos);      // Enable pull-up resistor


  // configure GPIOTE channel 0 fro Button A (P0.14)
  NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |            // event mode
                          (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |   // detect any change
                          (0 << GPIOTE_CONFIG_PORT_Pos) |
                          (14 << GPIOTE_CONFIG_PSEL_Pos);                                   // select pin p0.14

  // Configure GPIOTE channel for Button B (P0.23)
  NRF_GPIOTE->CONFIG[1] = (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |          // event mode
                          (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) | // detect any change
                          (0 << GPIOTE_CONFIG_PORT_Pos) |
                          (23 << GPIOTE_CONFIG_PSEL_Pos);                                 // select pin P0.23


  NRF_GPIOTE->INTENSET =  (GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos) |
                          (GPIOTE_INTENSET_IN1_Set << GPIOTE_INTENSET_IN1_Pos);

  // enable GPIO interrupts for Buttons A and B
  NVIC_SetVector(GPIOTE_IRQn, (uint32_t)GPIOTE_IRQHandler);     // set ISR  vector
  NVIC_SetPriority(GPIOTE_IRQn, 3);                             // set priority      
  NVIC_EnableIRQ(GPIOTE_IRQn);                                  // enable GPIOTE IRQ



}

/*
 * Function: map
 * --------------
 * Maps an accelerometer reading from the raw range to the height of the OLED display.
 * The input value is scaled to fit the display's pixel range.
 *
 * Parameters:
 * - xValue: The accelerometer reading to be mapped to the OLED display.
 *
 * Returns:
 * - A scaled value that corresponds to a pixel position on the OLED display.
*/
int map(int xValue){
  int old_min = -512;     // minimum value from accelerometer
  int old_max = 511;      // maximum value from accelerometer
  int new_min = 0;        // minimum value for Oled display
  int new_max = HEIGHT -1;// maximum value for Oled display 

  int value = ((xValue - old_min) * (new_max - new_min) / (old_max-old_min)) + new_min;
  return value;
}

/*
 * Function: calculateJerk
 * -----------------------
 * Calculates the jerk (rate of change of acceleration) between two accelerometer readings.
 * The calculated jerk is then scaled.
 *
 * Parameters:
 * - currentReading: The current accelerometer reading.
 * - previousReading: The previous accelerometer reading.
 *
 * Returns:
 * - A scaled jerk value representing the rate of change of acceleration.
*/
int calculateJerk(int currentReading, int previousReading) {
  int jerk = currentReading - previousReading;  // calculate jerk
  int old_min = -1023;                          // minimum possible jerk
  int old_max = 1023;                           // maximum possible jerk
  int new_min = -31;                            // minimum scaled jerk
  int new_max = 31;                             // maximum scaled jerk
  
  int value = ((jerk - old_min) * (new_max - new_min) / (old_max-old_min)) + new_min;
  return value;

}

/*
 * Function: getAccelerometerSample
 * -------------------------------
 * Initialises the accelerometer.
 * Reads the current sample from the accelerometer .
 * Combines the high and low bits of the output to produce a 10-bit value.
 *
 * Parameters:
 * - None
 *
 * Returns:
 * - A 10-bit accelerometer reading representing the X-axis acceleration.
*/
int getAccelerometerSample(){
  // buffers for the higher and lower bits
  uint8_t higherBits;
  uint8_t lowerBits;

  static int initialised = 0;
  if(!initialised){
    // enable accelerometer by setting subadress 0x20 to 0x57, see LSM303AGR datasheet 
    // then reada high part of X value from subaddress 0x29
    // remember these functions need the I2C address left-shifted by one bit
    i2cInt.writeRegister(0x19<<1, 0x20, 0x57);  

    initialised = 1;
  }
              
  
  i2cInt.readRegister(0x19<<1, LSM303_OUT_X_H_A, &higherBits, 1);

  i2cInt.readRegister(0x19<<1, LSM303_OUT_X_L_A, &lowerBits, 1);;

  int16_t xValue = ((higherBits << 8) | lowerBits) >> 6;  // hift right to keep top 10 bits
 
  // twos complement
  if (xValue & 0x0200) {  // if the 10th bit is set to 1
    xValue |= 0xFC00;   
  }

  return xValue;
}

/*
 * Function: updateOledDisplay
 * ---------------------------
 * Updates the OLED display by sending the current framebuffer data via I2C.
 * This function flattens the framebuffer data and writes it to the display.
 *
 * Parameters:
 * - None
 *
 * Returns:
 * - None
*/
void updateOledDisplay() {
  // prepare a single buffer for the entire display
  uint8_t frameData[1 + (WIDTH * PAGES)];
  frameData[0] = DATA_MODE; // first byte is DATA_MODE

  // flatten framebuffer into the transmission buffer
  for (uint8_t page = 0; page < PAGES; page++) {
    for (uint8_t x = 0; x < WIDTH; x++) {
      frameData[1 + (page * WIDTH) + x] = oledDisplayFrameBuffer[x][page];
    }
  }

  // send the complete framebuffer in a single I2C transaction
  i2cExt.write(OLED_I2C_ADDRESS << 1, frameData, sizeof(frameData));

}


/*
 * Function: handleModeAccelerometer
 * ----------------------------------
 * Handles Mode A, where the accelerometer data is used to 
 * map the reading to the Oled's display. 
 *
 * Parameters:
 * - accelerometerReading: The current accelerometer reading 
 *
 * Returns:
 * - None
*/
void handleModeAccelerometer(int accelerometerReading){
  int height = map(accelerometerReading);  // map accelerometer reading to the display height

  // set the pixels for the current column based on the height value
  for (int i = HEIGHT - 1; i > height; i--) {
    uint8_t page = i / 8;       // calulate page
    uint8_t bitPos = i % 8;     // determine pixel position within the page
    oledDisplayFrameBuffer[currentColumn][page] |= (1 << bitPos);  // set pixel on
  }
}

/*
 * Function: handleModeJerk
 * ------------------------
 * Handles Mode B, where the jerk (rate of change of acceleration)
 * is calculated and visualised on the OLED display. The jerk is mapped to a vertical line 
 * that is drawn either upwards or downwards based on whether the jerk value is positive 
 * or negative. The pixels for the current column are updated accordingly.
 *
 * Parameters:
 * - accelerometerReading: The current accelerometer reading used to calculate jerk.
 * - previousReading: The previous accelerometer reading used to compute the jerk.
 *
 * Returns:
 * - None
 */
void handleModeJerk(int accelerometerReading, int previousReading){
  int jerk = calculateJerk(accelerometerReading, previousReading);  // calculate jerk

  int startY = MID_Y;           // start drawing from y=31 (middle of the screen)
  int endY = startY + jerk;     // calculate the end position based on the jerk value

  // draw the line for the current column from startY to endY
  if (endY > startY) {  // positive jerk: draw upwards
    for (int y = startY; y <= endY; y++) {
      uint8_t page = y / 8;          // calculate page number
      uint8_t bitPos = y % 8;       // determine bit position within the page
      oledDisplayFrameBuffer[currentColumn][page] |= (1 << bitPos);  // set pixel
    }
  } 
  else {  // negative jerk: draw downwards
    for (int y = startY; y >= endY; y--) {
      uint8_t page = y / 8;          // calculate page number
      uint8_t bitPos = y % 8;       // determine bit position within the page
      oledDisplayFrameBuffer[currentColumn][page] |= (1 << bitPos);  // set pixel
    }
  }
}

/*
 * Function: graphData
 * -------------------
 * Main function that continuously reads data from the accelerometer and updates
 * the Oleds display based on the current mode (Mode A or Mode B). It uses the 
 * accelerometer readings to either plot a graph of the acceleration (Mode A) 
 * or a graph of the jerk (Mode B). 
 * 
 * Parameters:
 * - refreshRate: The rate at which the display is refreshed, affecting the delay 
 *                between accelerometer readings (in Hz).
 *
 * Returns:
 * - None
 */
void graphData(uint8_t refreshRate){
  initMicroBitDisplay();
  initOledDisplay();
  configureButtonInterrupts();
  handleButtonA();

  float delayMs = 1 / (float)refreshRate;
  
  int xValue;  // get accelerometer reading
  int previousReading = 0;    // previous accelerometer reading for jerk calculation
  
  while(true){
    xValue = getAccelerometerSample();  // get accelerometer reading
    
    // Clear the current column
    memset(&oledDisplayFrameBuffer[currentColumn][0], 0, PAGES);

    if (currentMode == MODE_A) {  // Mode A: Accelerometer 
      handleModeAccelerometer(xValue);
    } 
    else if (currentMode == MODE_B) {  // Mode B: Jerk
      handleModeJerk(xValue, previousReading);
  
      previousReading = xValue;  // update previous reading
    }

    // move to the next column
    currentColumn++;
    if (currentColumn >= WIDTH) {  // if the end was reached loop back to the start
      // shift all columns left by one
      memcpy(&oledDisplayFrameBuffer[0][0], &oledDisplayFrameBuffer[1][0], 1024);
      
      // clear the last column
      memset(&oledDisplayFrameBuffer[WIDTH - 1][0], 0, PAGES);

      currentColumn = WIDTH - 1;
    }


    // update oled witht the new buffer
    updateOledDisplay();
    
    // wait
    NRFX_DELAY_MS(delayMs);
  }
}
