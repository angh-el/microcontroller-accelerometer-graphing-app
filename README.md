# microcontroller-accelerometer-graphing-app

## Introduction

This project is a real-time graphing application that displays data from a micro:bit with a built-in accelerometer on an OLED screen. The application has two modes:
1. **Mode A (Accelerometer Mode)**: Displays the raw X-axis accelerometer readings on the OLED as vertical bars, where the height of the bar corresponds to the magnitude of the reading.
2. **Mode B (Jerk Mode)**: Displays the calculated jerk values, which are the differences between consecutive accelerometer readings. Positive jerk values are displayed as lines moving upwards from the center of the screen, and negative jerk values are drawn as lines moving downwards.

The application allows users to switch between these modes using the buttons on a microcontroller. The data is plotted on the OLED display in real-time. The display scrolls horizontally as new data is plotted, and old data is moved off the screen.

## Demo

### Mode A (Accelerometer Mode):
![IMG_3482 (2)](https://github.com/user-attachments/assets/6e609bdb-9003-435f-b9df-7839644eb7a0)





### Mode B (Jerk Mode):
![IMG_3484](https://github.com/user-attachments/assets/ecb14786-f2ca-4b64-88c0-7ef94dce77d5)



### Video Demonstration:
https://github.com/user-attachments/assets/dc75bc16-01a7-4caf-80b7-561ca93b3e52






## How It Works

### Code Breakdown

- **Button Handling**: The application uses interrupts to handle button presses. Pressing Button A switches the display to Mode A, and pressing Button B switches it to Mode B. When either button is pressed, the OLED screen is cleared, and the appropriate mode is activated.
  
- **Accelerometer Data**: The accelerometer readings are fetched via I2C communication. The readings are mapped to the display, with values ranging from -512 to 511. Positive values result in bars drawn upwards, and negative values result in bars drawn downwards. In Mode B, the jerk is calculated as the difference between consecutive accelerometer readings, with values ranging from -1023 to +1023.

- **Display Management**: The OLED display is updated every frame with new data. The `oledDisplayFrameBuffer` stores the pixel data, which is updated based on the current accelerometer or jerk value. The screen is refreshed by sending the entire frame buffer to the OLED using I2C (which is not the most efficient way).

- **Frame Buffer Management**: The display is 128x64 pixels, organized into 8 pages of 128 columns. The graphing application continuously updates the display by shifting the columns, and when the last column is filled, it scrolls the display left to make space for new data.

### Core Functions:
1. `handleButtonA()`: Clears the screen and switches to Mode A.
2. `handleButtonB()`: Clears the screen and switches to Mode B.
3. `getAccelerometerSample()`: Reads the accelerometer data from the sensor.
4. `handleModeAccelerometer()`: Updates the OLED display with accelerometer data (Mode A).
5. `handleModeJerk()`: Updates the OLED display with jerk data (Mode B).
6. `updateOledDisplay()`: Sends the updated frame buffer to the OLED.
7. `graphData(uint8_t refreshRate)`: Main function that continuously reads accelerometer data and updates the display. It handles both Mode A and Mode B, depending on the button pressed.

### Modes:
- **Mode A (Accelerometer Mode)**: 
  - The accelerometer data is mapped to the OLED's height (0-63 pixels).
  - A bar is drawn starting from the bottom of the display to the corresponding height based on the accelerometer value.
  - A continuous series of accelerometer readings is plotted from left to right. When the screen is full, the graph shifts left, and new data is added to the right.
  
- **Mode B (Jerk Mode)**:
  - Jerk is calculated by finding the difference between the current and the previous accelerometer reading.
  - The jerk value is mapped to the OLED display's vertical axis, with positive jerk values being plotted upwards from the center and negative jerk values plotted downwards.



## Further Improvements

Currently, the frame rate of the application is around 40-50 FPS. In future iterations, this can be improved by optimising the following aspects:
1. **Optimising Display Updates**: Instead of updating the entire OLED frame buffer for each refresh, a more efficient method could be employed to update only the areas of the display that have changed.
2. **Hardware Optimisations**: Tweaking the I2C communication speed and optimising the power consumption of the microcontroller could further boost performance.

By implementing these optimizations, it is possible to increase the frame rate and overall performance of the application.
