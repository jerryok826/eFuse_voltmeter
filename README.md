# eFuse Voltmeter
![Alt text](https://github.com/jerryok826/eFuse_voltmeter/blob/main/Pictures/eFuse_Volt.jpeg)

# eFuse Voltmeter in fault
![Alt text](https://github.com/jerryok826/eFuse_voltmeter/blob/main/Pictures/eFuse_volt_fault.jpeg)

## Project Description
eFuse_Voltmeter is a protection circuit for other devices. The initial reason was to protect a a Ham radio when adjusting its mosfet transistors.

It can be powered from wall wart or a battery. 

Features:
1. Reverse power protection.
2. Adjustable over voltage input protection
3. Adjustable over current protection.
4. Voltage and current monitoring.
5. Output power control.
6. Output power disconnected on fault.
7. Audio alarm on fault.

When an over voltage or an over current event happens the board will disable its output. This fault state is latch until the control buttons is pressed. Apond a fault a audiuo alarm is also actived and a fault LED is lite. The fault is indicted by its measurement value will turn RED and the fault RED led is lite
 
### Project Status
The project is basically complete. Working on improvements.

## Design Files