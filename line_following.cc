#include <iostream>
#include <robot_instr.h>
#include <robot_link.h>
#include <stopwatch.h>
#include "line_following.h"
#include "pin.h"
#include <vector>
#include <delay.h>
using namespace std;

//reverse speeds
#define MOTOR1_REVERSE  		185
#define MOTOR4_REVERSE   		57

//15, 30
//small correction is 0.4 *70 
#define SMALL_CORRECTION 		30
//large correction is 0.54*85
#define LARGE_CORRECTION		55	

// delay before turning straight- how long to move past the junction for? SET THIS
#define STRAIGHT_MOVEMENT_DELAY 350
#define LEFT_TURN_TIME  		3000
#define RIGHT_TURN_TIME 		3000
#define LEFT_TURN_SPEED     	40
#define RIGHT_TURN_SPEED    	168

//store the past data in a stack-like object, with most
//recent data at the front.
//bool colour represents a white/black line - default is 1, white.
class historical_data
{
private:
	const static int vec_size = 50;
	vector<int> past_readings;
	vector<int> past_black_line_readings;

public:
	//no need for constructor here

	void update(int val, bool colour = 1)
	{
		if(colour)
		{
			if (past_readings.size() == vec_size)
			{
				past_readings.erase(past_readings.end()-1);
			}
			past_readings.insert(past_readings.begin(),val);
		}
		else //black line
		{
			if (past_black_line_readings.size() == vec_size)
			{
				past_black_line_readings.erase(past_black_line_readings.end()-1);
			}
			past_black_line_readings.insert(past_black_line_readings.begin(),val);
		}
	};

	void erase(bool colour = 1)
	{
		if (colour)
			past_readings.clear();
		else
			past_black_line_readings.clear();
	};

	bool lost(bool colour = 1)
	{
		if(colour)
		{
			for (int i = 0; i < past_readings.size(); ++i)
			{
				if (past_readings[i] == 0)
					continue;
				else
					return false;
			}
			return true;
		}
		else
		{
			for (int i = 0; i < (past_black_line_readings.size()-20); ++i)
			{
				if (past_black_line_readings[i] == 7)
					continue;
				else
					return false;
			}
			return true;
		}
	};

	//checks to see how many of the past 10 readings have been equal to param.
	//Enables the line following algorithm to set the motor speed accordingly.
	int time_away(int param, bool colour = 1)
	{	
		if(colour)
		{
			int count = 0;
			for (int i = 0; i < 30; ++i)
			{
				if (past_readings[i] == param)
				{	++count;
					continue;
				}
				else
					return count;
			}
			return count;
		}
		else //on the black line, check 30 readings
		{
			int count = 0;
			for (int i = 0; i < 30; ++i)
			{
				if (past_black_line_readings[i] == param)
				{	++count;
					continue;
				}
				else
					return count;
			}
			return count;
		}
	};

} line_data; 


//method for following a white line straight
//have assumed that the sensors are ordered (from MSB to LSB): LEFT, MIDDLE, RIGHT
line_type_t white_line_straight(bool key)
{
	//this bit of logic uses the optional argument, the key, to decide the motor speeds.
	//Idea is that if the robot is near a pick up/delivery point, it goes slower to ensure
	//correct alignment. 
	int speed_1 = MOTOR1_OFFSET;
	int speed_4 = MOTOR4_OFFSET;
	if (!key)
	{
		speed_1 = MOTOR1_OFFSET - 30;
		speed_4 = MOTOR4_OFFSET - 30;
	}

	//clear the black line readings! They're invalid now.
	line_data.erase(0);

	//this gives a value between 0 and 7 for the line sensor readings.
	//can now deal with this as we need to
	int value = rlink.request(READ_LINE_SENSORS) & LINE_SENSOR_BITMASK;
	line_data.update(value);

	#ifdef LINE_DEBUG
	//cout << "White line straight: sensor value: " << value << endl;
	#endif

	switch(value)
	{
		//if we've got here 10 times and are still lost, then we have a problem.
		//need to find a line.
		case 0:
		{	
			//in case for some reason we're stationary, start moving.
			if (rlink.request(MOTOR_1) == 0 and rlink.request(MOTOR_4)==0)
			{
				rlink.command(MOTOR_1_GO, speed_1-20);
				rlink.command(MOTOR_4_GO, speed_4-20);
				return LINE_OK;
			}
			//go slowly
			if (line_data.lost())
			{
				rlink.command(MOTOR_1_GO, speed_1-20);
				rlink.command(MOTOR_4_GO, speed_4-20);
				return LINE_LOST;
			}
			else
				return LINE_OK;
		}
		//we've strayed off the line quite significantly (to the left).
		//turn right to correct. Scale depending on how long we've been in this situation.
		case 1:
		{
			int scaling = line_data.time_away(1);
			//calculate speed, some function of the scaling parameter.
			//for now, just multiply by 5. TODO- FIX THIS
			rlink.command(MOTOR_1_GO, speed_1 - LARGE_CORRECTION - 1 * (scaling/3));
			rlink.command(MOTOR_4_GO, speed_4);
			return LINE_OK;
		}
		//we're on the line!
		case 2:
		{
			int scaling = line_data.time_away(2);
			rlink.command(MOTOR_1_GO, speed_1);
			rlink.command(MOTOR_4_GO, speed_4);
			return LINE_OK;
		}
		//we've strayed slightly to the left. Adjust a bit.
		case 3:
		{
			int scaling = line_data.time_away(3);
			//calculate speed, some function of the scaling parameter.
			//for now, just multiply by 5. TODO- FIX THIS
			rlink.command(MOTOR_1_GO, speed_1 - SMALL_CORRECTION - 1* (scaling/3));
			rlink.command(MOTOR_4_GO, speed_4);
			return LINE_OK;
		}
		//we've strayed off the line quite significantly (to the right).
		//turn left to correct. Scale depending on how long we've been in this situation.
		case 4:
		{
			int scaling = line_data.time_away(4);
			//calculate speed, some function of the scaling parameter.
			//for now, just multiply by 10. TODO- FIX THIS
			rlink.command(MOTOR_1_GO,speed_1);
			rlink.command(MOTOR_4_GO, speed_4 - LARGE_CORRECTION - 1* (scaling/3));
			return LINE_OK;
		}
		case 5:
		{
			//maybe we've hit the white ramp?
			rlink.command(MOTOR_1_GO, speed_1);
			rlink.command(MOTOR_4_GO, speed_4);
			return LINE_BLACK;
		}
		//we've strayed slightly to the right. Adjust a bit.
		case 6:
		{
			int scaling = line_data.time_away(6);
			//calculate speed, some function of the scaling parameter.
			//for now, just multiply by 5. TODO- FIX THIS
			rlink.command(MOTOR_1_GO, speed_1);
			rlink.command(MOTOR_4_GO, speed_4 - SMALL_CORRECTION - 1 *(scaling/3));
			return LINE_OK;
		}
		case 7:
		{
			//hit a junction
			//go slowly
			int scaling = line_data.time_away(7);
			rlink.command(MOTOR_1_GO, speed_1-20);
			rlink.command(MOTOR_4_GO, speed_4-20);
			if (scaling > 45)
			{	
				//we've been repeatedly getting all white readings- maybe we're on
				// the white ramp.
				return LINE_BLACK;
			}
			return LINE_JUNCTION;
		}
	}
	return LINE_OK;
}

line_type_t reverse_straight(void)
{
	//need better logic here?
	
	int value = rlink.request(READ_LINE_SENSORS) & LINE_SENSOR_BITMASK;

	#ifdef LINE_DEBUG
	cout << "Reverse straight: sensor value: " << value << endl;
	#endif	

	switch(value)
	{
		case 0:
			return LINE_LOST;
		case 1:
		case 3:
			rlink.command(MOTOR_1_GO, MOTOR1_REVERSE - SMALL_CORRECTION);
			rlink.command(MOTOR_4_GO, MOTOR4_REVERSE);
			return LINE_OK;
		case 4:
		case 6:
			rlink.command(MOTOR_1_GO, MOTOR1_REVERSE);
			rlink.command(MOTOR_4_GO, MOTOR4_REVERSE - SMALL_CORRECTION);
			return LINE_OK;
		case 7:
			return LINE_JUNCTION;
			
		default: //if we're on the line already
			rlink.command(MOTOR_1_GO, MOTOR1_REVERSE);
			rlink.command(MOTOR_4_GO, MOTOR4_REVERSE);
			return LINE_OK;
	}	
	
}	
//do these by timing
void turn_left(void)
{	
	#ifdef LINE_DEBUG
	cout << "Turn left"<< endl;
	#endif
	rlink.command(RAMP_TIME,20);
	rlink.command(MOTOR_1_GO, MOTOR1_OFFSET);
	rlink.command(MOTOR_4_GO, MOTOR4_OFFSET);
	stopwatch watch;
	watch.start();
	while(watch.read() < STRAIGHT_MOVEMENT_DELAY)
	{;}
	cout << "!!!!going to turn!!!!" << endl;
	rlink.command(MOTOR_1_GO, LEFT_TURN_SPEED);
    rlink.command(MOTOR_4_GO, LEFT_TURN_SPEED);
    delay(LEFT_TURN_TIME);
	while (true) 
	{
		int val = (rlink.request(READ_LINE_SENSORS) & LINE_SENSOR_BITMASK);
		if (val == 0x02 or val == 0x07)
			break;
		continue;
	}
	
	//delay(100);
	//rlink.command(MOTOR_1_GO, 0);
	//rlink.command(MOTOR_4_GO, 0);
	return;
}
void turn_right(void)
{
	#ifdef LINE_DEBUG
	cout << "Turn right" << endl;
	#endif
	rlink.command(RAMP_TIME,20);
	rlink.command(MOTOR_1_GO, MOTOR1_OFFSET);
	rlink.command(MOTOR_4_GO, MOTOR4_OFFSET);
	stopwatch watch;
	watch.start();
	while(watch.read() < STRAIGHT_MOVEMENT_DELAY)
	{;}
	rlink.command(MOTOR_1_GO, RIGHT_TURN_SPEED);
    rlink.command(MOTOR_4_GO, RIGHT_TURN_SPEED);
    delay(RIGHT_TURN_TIME);
    while (true) 
	{
		int val = (rlink.request(READ_LINE_SENSORS) & LINE_SENSOR_BITMASK);
		if (val == 0x02 or val == 0x07)
			break;
		continue;
	}
	//delay(100);
	//rlink.command(MOTOR_1_GO, 0);
	//rlink.command(MOTOR_4_GO, 0);
	return;
}

void turn_180(void)
{
	#ifdef LINE_DEBUG
	cout << "Turn 180" << endl;
	#endif
	rlink.command(RAMP_TIME,20);
	rlink.command(MOTOR_1_GO, MOTOR1_OFFSET);
	rlink.command(MOTOR_4_GO, MOTOR4_OFFSET);
	stopwatch watch;
	watch.start();
	while(watch.read() < STRAIGHT_MOVEMENT_DELAY)
	{;}
	rlink.command(MOTOR_1_GO, RIGHT_TURN_SPEED);
    rlink.command(MOTOR_4_GO, RIGHT_TURN_SPEED);
    delay(RIGHT_TURN_TIME * 3);
    while (true) 
	{
		int val = (rlink.request(READ_LINE_SENSORS) & LINE_SENSOR_BITMASK);
		if (val == 0x02 or val == 0x07)
			break;
		continue;
	}
	//delay(100);
		//rlink.command(MOTOR_1_GO, 0);
	//rlink.command(MOTOR_4_GO, 0);
	return;	
}

//for the dodgy ramp
line_type_t black_line_straight(void)
{
	//inverted logic to above
	//write out what different sensor readings will mean in this new truth table
	//this gives a value between 0 and 7 for the line sensor readings.
	//can now deal with this as we need to.

	//clear the white line readings! They're invalid now.
	line_data.erase();

	int value = rlink.request(READ_LINE_SENSORS) & LINE_SENSOR_BITMASK;
	line_data.update(value,0);

	#ifdef LINE_DEBUG
	cout << "Black Line straight: sensor value: " << value << endl;
	#endif

	switch(value)
	{	
		//may now be on the white line (sensor issues)
		case 0:
		{
			int scaling = line_data.time_away(2,0);
			if(scaling >5)
			{
				//rlink.command(MOTOR_1_GO, MOTOR1_OFFSET-20);
				//rlink.command(MOTOR_4_GO, MOTOR4_OFFSET-20);
				return LINE_OK;
			}
			return LINE_BLACK;
		}
		//we've strayed off the line slightly to the right.
		//turn left to correct. Scale depending on how long we've been in this situation.
		case 1:
		{
			int scaling = line_data.time_away(1,0);
			//calculate speed, some function of the scaling parameter.
			//for now, just multiply by 0.5. TODO- FIX THIS
			rlink.command(MOTOR_1_GO, MOTOR1_OFFSET);
			rlink.command(MOTOR_4_GO, MOTOR4_OFFSET - SMALL_CORRECTION - 1/3 * scaling);
			return LINE_BLACK;
		}
		//we might be on the white line. check by calling time_away and checking the return val
		//!!!MAY NEED EDITING!!!
		case 2:
		{
			int scaling = line_data.time_away(2,0);
			if(scaling >10)
			{
				rlink.command(MOTOR_1_GO, MOTOR1_OFFSET-20);
				rlink.command(MOTOR_4_GO, MOTOR4_OFFSET-20);
				return LINE_OK;
			}
			return LINE_BLACK;
		}
		//we've strayed significantly to the right. Adjust far left.
		case 3:
		{
			int scaling = line_data.time_away(3,0);
			//calculate speed, some function of the scaling parameter.
			//for now, just multiply by 5. TODO- FIX THIS
			rlink.command(MOTOR_1_GO, MOTOR1_OFFSET);
			rlink.command(MOTOR_4_GO, MOTOR4_OFFSET - LARGE_CORRECTION - 1/3*scaling);
			return LINE_BLACK;
		}
		//we've strayed off the line slightly to the left.
		//turn right to correct. Scale depending on how long we've been in this situation.
		case 4:
		{
			int scaling = line_data.time_away(4,0);
			//calculate speed, some function of the scaling parameter.
			//for now, just multiply by 10. TODO- FIX THIS
			rlink.command(MOTOR_1_GO, MOTOR1_OFFSET - SMALL_CORRECTION - 1/3 *scaling);
			rlink.command(MOTOR_4_GO, MOTOR4_OFFSET);
			return LINE_BLACK;
		}
		case 5:
		{
			int scaling = line_data.time_away(5,0);
			//all good!
			rlink.command(MOTOR_1_GO, MOTOR1_OFFSET);
			rlink.command(MOTOR_4_GO, MOTOR4_OFFSET);
			return LINE_BLACK;
		}
		//we've strayed far to the left. Adjust to the right.
		case 6:
		{
			int scaling = line_data.time_away(6,0);
			//calculate speed, some function of the scaling parameter.
			//for now, just multiply by 5. TODO- FIX THIS
			rlink.command(MOTOR_1_GO, MOTOR1_OFFSET - LARGE_CORRECTION -  1/3*scaling);
			rlink.command(MOTOR_4_GO, MOTOR4_OFFSET);
			return LINE_BLACK;
		}

		//if we've got here 10 times and are still lost, then we have a problem.
		//need to find a line.
		case 7:
		{
			//in case for some reason we're stationary, start moving.
			if (rlink.request(MOTOR_1) == 0 and rlink.request(MOTOR_4)==0)
			{
				rlink.command(MOTOR_1_GO, MOTOR1_OFFSET-20);
				rlink.command(MOTOR_4_GO, MOTOR4_OFFSET-20);
				return LINE_BLACK;
			}

			if (line_data.lost(0))
				{
					rlink.command(MOTOR_1_GO, MOTOR1_OFFSET-20);
					rlink.command(MOTOR_4_GO, MOTOR4_OFFSET-20);
					return LINE_LOST;
				}
				else
					return LINE_BLACK;
		}
	}
	return LINE_LOST;
}

//turn right to face line; stop when line found or timer exceeds 1000 ms.
//Default is colour 1, ie white line detection.
void find_line_turn_right(bool colour = 1)
{	
	int key = 0x02;
	if (!colour)
		key = 0x05;
	stopwatch s;
	s.start();
	while(s.read() < 1000)
	{	
		rlink.command(MOTOR_1_GO, RIGHT_TURN_SPEED);
		rlink.command(MOTOR_4_GO, RIGHT_TURN_SPEED);
		if ((rlink.request(READ_LINE_SENSORS) & LINE_SENSOR_BITMASK) == key)
		{
			delay(300);
			break;
		}
	}
}

void find_line_turn_left(bool colour = 1)
{
	int key = 0x02;
	if (!colour)
		key = 0x05;
	stopwatch s;
	s.start();
	while(s.read() < 1000)
	{	
		rlink.command(MOTOR_1_GO, LEFT_TURN_SPEED);
		rlink.command(MOTOR_4_GO, LEFT_TURN_SPEED);
		if ((rlink.request(READ_LINE_SENSORS) & LINE_SENSOR_BITMASK) == key)
		{	
			delay(300);
			break;
		}
	}
}
//if lost
line_type_t find_a_line(void)
{
	
	while(true)
	{
		rlink.command(MOTOR_1_GO, MOTOR1_REVERSE);
		rlink.command(MOTOR_4_GO, MOTOR4_REVERSE);
		int val = (rlink.request(READ_LINE_SENSORS) & LINE_SENSOR_BITMASK);
		if (val == 0x00)
			continue;
		else if (val == 0x01 or val == 0x03)
		{
			find_line_turn_right();
			break;
		}
		else if (val == 0x04 or val == 0x06)
		{
			find_line_turn_left();
			break;
		}
		
	}
	return LINE_OK;
}

line_type_t find_a_black_line(void)
{
	
	while(true)
	{
		rlink.command(MOTOR_1_GO, MOTOR1_REVERSE);
		rlink.command(MOTOR_4_GO, MOTOR4_REVERSE);
		int val = (rlink.request(READ_LINE_SENSORS) & LINE_SENSOR_BITMASK);
		if (val == 0x07)
			continue;
		else if (val == 0x01 or val == 0x03)
		{
			find_line_turn_left(0);
			break;
		}
		else if (val == 0x04 or val == 0x06)
		{
			find_line_turn_right(0);
			break;
		}
		
	}
	return LINE_BLACK;
}

