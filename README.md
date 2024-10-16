ECSE444 Microprocessors - Lab 2: Basic I/O and ADC-based Readout of Temperature and Voltage

Course: Fall 2024 - ECSE444

Overview

In this lab, we will learn how to use the General-Purpose Input/Output (GPIO) pins and Analog-to-Digital Converter (ADC) functionality on the STM32L4+ processor. Specifically, we will:

	1.	Control an LED using a GPIO pin.
	2.	Read core temperature and reference voltage using the ADC.
	3.	Switch between temperature and voltage readouts using a button.

The lab is designed to build on the previous exercise, where we wrote C/assembly code for basic processor functions.

Objectives

	•	Program the STM32L4+ microcontroller to toggle an LED when a button is pressed.
	•	Use the ADC to read the internal reference voltage and core temperature.
	•	Implement code to switch between temperature and voltage readouts based on user input (button press).
	•	Optionally, configure interrupts to handle button presses.

Lab Steps

Step 1: LED Control via GPIO

	•	Write a C program that toggles LED2 (connected to pin PB14) every time the blue button is pressed.
	•	Configure GPIO pins using STM32CubeMX and test the LED functionality in an infinite loop.

Step 2: ADC Readout of Reference Voltage

	•	Configure ADC1 to read the internal reference voltage.
	•	Convert the ADC value to a correct voltage value and monitor it using watched variables.

Step 3: ADC Readout of Core Temperature

	•	Configure ADC1 to read the internal temperature sensor.
	•	Convert the ADC value to the temperature in Celsius.

Step 4: Switch Between Voltage and Temperature Readouts

	•	Modify your program to switch between the voltage and temperature readings when the blue button is pressed.

Step 5: Final Program

	•	Combine the code from the previous steps into a complete program that toggles between voltage and temperature readouts.
	•	The program should visually indicate the current state using LED2.

Step 6 (Optional): Interrupt-based Button Press Handling

	•	Implement interrupt-based functionality to handle button presses for switching between voltage and temperature readings.

Experimental Results to Report

	•	Step 1: Describe the GPIO configuration used to toggle the LED.
	•	Step 2: Describe the ADC1 configuration for reading the reference voltage.
	•	Step 3: Describe the ADC1 configuration for reading the core temperature.
	•	Step 4: Provide code or pseudo-code for switching between voltage and temperature readouts.
	•	Step 5: Test temperature sensor readings by heating/cooling the processor (e.g., using fingers or a hairdryer).
