#include <iostream>
using namespace std;
#include <robot_instr.h>
#include <robot_link.h>
#include <stopwatch.h>
#include "line_following.h"
#include "pin.h"
#include <delay.h>
#define ROBOT_NUM  11                         // The id number (see below)
robot_link  rlink;                            // datatype for the robot link
//straight movement offsets
#define MOTOR1_OFFSET   		70
#define MOTOR4_OFFSET   		194
#define LARGE_CORRECTION		33
#define SMALL_CORRECTION 		20

//reverse speeds
#define MOTOR1_REVERSE  		185
#define MOTOR4_REVERSE   		57



//tests white line following
void white_line_straight_test(void)
{
	stopwatch s;
	s.start();
	while(s.read() < 15000)
	{
		cout << white_line_straight() << endl;
	}
}

void black_line_straight_test(void)
{
	stopwatch s;
	s.start();
	while(s.read() < 10000)
	{
		cout << black_line_straight() << endl;
	}
}

void turn_left_test(void)
{
	turn_left();
	stopwatch s;
	s.start();
	while(s.read() < 5000)
	{
		cout << white_line_straight() << endl;
	}
}

void turn_right_test(void)
{
	turn_right();
	stopwatch s;
	s.start();
	while(s.read() < 5000)
	{
		cout << white_line_straight() << endl;
	}
}

//start from just before bottom of white ramp
//goes to top of 
void white_and_black_line_test(void)
{
	rlink.command(RAMP_TIME,0);
	while(true)
	{
		int val = white_line_straight();
		//cout << val << endl;
		if (val == LINE_BLACK)
			break;
	}
	stopwatch watch;
	watch.start();
	while(watch.read() < 7200)
	{
		//cout << "!!!!SWITCHING ALGORITHM!!!!" << endl;
		int val = black_line_straight();
		//cout << val << endl;
		if (val == LINE_OK)
			break;
	}
	cout << "switched back" << endl;
	rlink.command(MOTOR_1_GO,MOTOR1_OFFSET+50);
	rlink.command(MOTOR_4_GO, MOTOR4_OFFSET+50);
	delay(800);
	//rlink.command(MOTOR_1_GO,MOTOR1_OFFSET);
	//rlink.command(MOTOR_4_GO, MOTOR4_OFFSET - SMALL_CORRECTION);
	//delay(1000);
	watch.stop();
	stopwatch s;
	s.start();
	while(s.read() < 6000)
	{
		int val = white_line_straight();
		if (val == LINE_JUNCTION)
		{	
			cout <<"reached junction" << endl;
			break;
			
		}
	}
	//turn_left();
}

//turn left at first junction we see.
void turn_at_junction_test(void)
{
	while(true)
	{
		int val = white_line_straight();
		//cout << val << endl;
		if (val == LINE_JUNCTION)
			break;
	}	
	turn_left();
	stopwatch s;
	s.start();
	while(s.read() < 7000)
	{
		int val = white_line_straight();
		//~ if (val == LINE_JUNCTION)
		//~ {
			//~ cout << "line junction!!!!!!!!!!!!" << endl;
			//~ break;
		//~ }
	}
}

//turn left at second junction we see
void turn_at_second_junction_test(void)
{
	int junct_count = 0;
	stopwatch s;
	while(junct_count < 2)
	{
		int val = white_line_straight();
		cout << val << endl;
		if (val == LINE_JUNCTION)
		{
			junct_count++;
			delay(800);
		}
	}
	turn_left();
	s.stop();
	s.start();
	while(s.read() < 3000)
		white_line_straight();
}

//simple reverse test
void reverse_test(void)
{
	stopwatch watch;
    watch.start();
    while(watch.read() < 5000) //better to have a time bound here.
    {
        line_type_t ret = reverse_straight();
        cout << ret << endl;
        if (ret == LINE_LOST)
        {
			cout << "lost the line!" << endl;
            return;
        }
        else if (ret == LINE_JUNCTION)
            break;
    }
}

//reverse and turn also
void reverse_and_turn_test(void)
{
	stopwatch watch;
    watch.start();
    while(watch.read() < 10000) //better to have a time bound here.
    {
        line_type_t ret = reverse_straight();
        cout << ret << endl;
        if (ret == LINE_LOST)
        {
			cout << "lost the line!" << endl;
            return;
        }
        else if (ret == LINE_JUNCTION)
            break;
    }
    turn_right();
    stopwatch s;
	s.start();
	while(s.read() < 5000)
		white_line_straight();
}

void microswitch_test(void)
{
	int val = rlink.request(READ_ACTUATOR_AND_LED);
	val = val ^ ACTUATOR_TRIGGER;
	rlink.command(WRITE_ACTUATOR_AND_LED, val);
	delay(1000);
    rlink.command(MOTOR_1_GO,MOTOR1_OFFSET);
    rlink.command(MOTOR_4_GO,MOTOR4_OFFSET);
    while (true) 
    {	
		white_line_straight();
		int val = rlink.request(READ_FRONT_MICROSWITCH) & PIN_FRONT_MICROSWITCH;
		if (val == 16)
			break;
	}
	rlink.command(MOTOR_1_GO,0);
    rlink.command(MOTOR_4_GO,0);
    delay(1000);
	val = val ^ ACTUATOR_TRIGGER;
	rlink.command(WRITE_ACTUATOR_AND_LED, val);
    cout << "done" << endl;
}

void actuator_test(void)
{
	int val = rlink.request(READ_ACTUATOR_AND_LED);
	val = val ^ ACTUATOR_TRIGGER;
	rlink.command(WRITE_ACTUATOR_AND_LED, val);
	int a;
	cin >> a ;
	val = val ^ ACTUATOR_TRIGGER;
	rlink.command(WRITE_ACTUATOR_AND_LED, val);
}

void line_lost_test(void)
{
	find_a_black_line();
	black_line_straight_test();
}

int main (int argc, char* argv[])
{
	rlink.command(RAMP_TIME, 255);
	//int val;
	//stopwatch s = stopwatch();
		
	// data from microprocessor
#ifdef __arm__
	if (!rlink.initialise ()) {               // setup for local hardware
#else
	if (!rlink.initialise (ROBOT_NUM)) {      // setup the link
#endif
		cout << "Cannot initialise link" << endl;
		rlink.print_errs("    ");
		return -1;
	}	
		//microswitch_test();
		//rlink.command(WRITE_ACTUATOR_AND_LED, ACTUATOR_IN);
		//turn_at_junction_test();
		//white_and_black_line_test();
		//white_and_black_line_test();
		//turn_at_junction_test();
		//white_line_straight_test();
		line_lost_test();

}
