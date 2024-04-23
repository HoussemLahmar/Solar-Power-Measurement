# Solar-Power-Measurement

This project focuses on measuring essential parameters from solar panels using an STM32 microcontroller. It's designed for the NUCLEO-L152RE board but can be easily adapted to other STM32 boards due to the use of HAL codes.

The STM32 microcontroller reads analog signals from multiple channels via ADC and transfers them using DMA. A TIMER_2 is utilized as an external interrupt to control the frequency of data acquisition.

Additionally, the project incorporates various sensors like temperature and light sensors to provide comprehensive monitoring of solar panel performance.

## Components:
* NUCLEO-L152RE board
* 16x2 I2C LCD
* Light Dependent Resistor (LDR)
* LM35 Linear Temperature Sensor
* Solar Panel
* Assorted resistors for voltage dividers

## Technical Details:
### Voltage Divider:
To measure voltage from the solar panel, a voltage divider is employed to bring the voltage within the measurable range of the STM32 board's ADC. The selection of resistors for the voltage divider follows a specific formula to ensure accurate voltage measurement.

### Shunt Resistor:
For current measurement, a shunt resistor is used in conjunction with the load resistor. The shunt resistor must meet specific criteria to accurately measure the current passing through it without significantly affecting the circuit.

### Code Customization:
The code provided in the `main.c` file can be adjusted based on the selected resistor values and sensor calibration requirements. Parameters such as the voltage divider resistors, shunt resistor value, and I2C LCD address can be modified to suit the circuit configuration.

Temperature compensation for the LM35 sensor and calibration for the LDR sensor can also be adjusted within the code to improve accuracy.

## Note:
This project is based on research papers included in the repository, which provide detailed insights into solar panel monitoring and measurement techniques. The code and circuit design aim to implement the methodologies outlined in these papers for practical application.
