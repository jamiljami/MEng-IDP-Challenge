#ifndef PIN_H
#define PIN_H
//A pin map so we know which sensors are where
//Also defines which bits are to be checked for emergency stop.
//Speak to Georgina/Suzanne about this.

#define ADDR_FRONT_MICROSWITCH 	0x05 //I2C address of chip with front microswitch 
#define ADDR_FRONT_SENSOR      	0x05 //I2C address of chip with front sensor
#define PIN_FRONT_MICROSWITCH  	0x10 //the pin for the front microswitch
#define PIN_ARM_MICROSWITCH  	0x08 //the pin for the arm microswitch
#define ADDR_ACTUATOR_AND_LED   0x01 //I2C address of chip with actuators and LEDs
#define READ_ACTUATOR_AND_LED   READ_PORT_1
#define WRITE_ACTUATOR_AND_LED  WRITE_PORT_1
#define ACTUATOR_TRIGGER        0x01  //Pin number of actuator ckt
#define READ_LINE_SENSORS      	READ_PORT_5
#define LINE_SENSOR_BITMASK    	0x07
#define READ_FRONT_MICROSWITCH 	READ_PORT_5
#define READ_ARM_MICROSWITCH   	READ_PORT_5

//Pin list: 3 for light sensor output
//          1 for light sensor input
//			1 for analogue sensor output
//			2 for microswitches (front and arm)
//			1 for actuator on mechanism
//			4 for LEDs


 
#endif
