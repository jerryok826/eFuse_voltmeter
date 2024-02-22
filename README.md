# eFuse Voltmeter
![Alt text](https://github.com/jerryok826/eFuse_voltmeter/blob/main/Pictures/eFuse_Volt.jpeg)

# eFuse Voltmeter in fault
![Alt text](https://github.com/jerryok826/eFuse_voltmeter/blob/main/Pictures/eFuse_volt_fault.jpeg)

## Project Description
eFuse_Voltmeter is a protection circuit for down stream devices. The initial reason was to protect a Ham radio when doing bias adjustments mosfet power transistors.

It can be powered from any DC power source between 5 to 25 volts. Its been test up to 6 amps with a voltage drop of less than 100mv.

Features:
1. Reverse power protection.
2. Adjustable over voltage input protection
3. Adjustable over current protection.
4. Voltage and current monitoring.
5. Output power control.
6. Output power disconnected on fault.
7. Audio alarm on fault.

When an over voltage or an over current event happens the board will disable its output. The fault state is latched until the control buttons is pressed. Apond a fault a audiuo alarm is also actived and a fault LED is lite. The fault value is indicted by its red measurement value and the fault RED led is lite. A fault is cleared by pressing the adjusement button. The output button needs to be press again to reable the boards output.
 
### Project Status
The project is basically complete. Working on improvements.

## Design Files
