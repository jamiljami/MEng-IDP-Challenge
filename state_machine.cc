#include <iostream>
using namespace std;
#include <robot_instr.h>
#include <robot_link.h>
#include <stopwatch.h>
#include "line_following.h"
#include "pin.h"
robot_link rlink;

#define INITIAL_SPEED        60 //Some intial speed for the robot motors
#define TIME_FOR_PICKUP      1000 //time to pick up a cracker. NEEDS TESTING
#define TIME_FOR_DROP        3000 //time for dropping the cracker. NEEDS TESTING
#define TIME_FOR_ID          5000 //time to ID a cracker
#define TIME_FOR_RAMP        10000 //max time to climb black ramp
#define TIME_FOR_WHITE_RAMP  7200 //time to climb white ramp
#define TIME_TO_WHITE_RAMP   1500 //time to reach white ramp from D3
#define JUNCTION_DELAY       800  //delay before incrementing junction count

//Thresholds for the cracker id - TODO SET THESE
#define THRESHOLD_1          20
#define THRESHOLD_2          40
#define THRESHOLD_3          60


//LED indicator codes
#define LED_MASK             0xE9
#define CRACKER_1            0x06
#define CRACKER_2            0x04
#define CRACKER_3            0x02
#define CRACKER_4            0x00
#define REPLACE_CRACKER      0x08


// This file defines the main function and the state machine
// for the IDP robot. 

typedef enum 
{
    STATE_START = 0, STATE_S_TO_P, STATE_COLLECT_CRACKER, STATE_ID_CRACKER,
    STATE_P_TO_D1, STATE_P_TO_D2, STATE_P_TO_D3, STATE_P_TO_D4, STATE_DROP_CRACKER, 
    STATE_D1_TO_P, STATE_D2_TO_P, STATE_D3_TO_P, STATE_D4_TO_P,
    STATE_RETURN_TO_START, NUM_STATES
} state_t;

static state_t current_state;
static int cracker_carried;

//light the right LEDs corresponding to each cracker.
//if the cracker can't be identified, flash LEDs on/off a couple of times
//to indicate detection failure
void cracker_led(int a)
{
    int val = rlink.request(READ_ACTUATOR_AND_LED);
    val = val & LED_MASK;
    if (a != -1)
    {
        switch(a)
        {
            case(1):
                val = val | CRACKER_1;
                break;
            case(2):
                val = val | CRACKER_2;
                break;
            case(3):
                val = val | CRACKER_3;
                break;
            case(4):
                val = val | CRACKER_4;
                break;
        }
        rlink.command(WRITE_ACTUATOR_AND_LED, val);
        return;
    }
    //we couldn't find out what cracker we've got, so indicate error
    val = val | CRACKER_1;
    rlink.command(WRITE_ACTUATOR_AND_LED, val);
    delay(1000);
    val = val & LED_MASK;
    val = val | CRACKER_4;
    rlink.command(WRITE_ACTUATOR_AND_LED, val);
    delay(1000);
    val = val | CRACKER_1;
    rlink.command(WRITE_ACTUATOR_AND_LED, val);
    return;
}
//use ADC to find cracker type.
//Read ADC for 5 seconds or something similar, and take an average.
int id_cracker(void)
{
    stopwatch s;
    int adc_val = 0;
    s.start();
    while(s.read() < TIME_FOR_ID)
    {
        adc_val += rlink.request(ADC0);
    }
    //Take the average of the ADC readings over the time period.
    adc_val /= TIME_FOR_ID;
    //Now we need to do stuff with the value.   
    if (adc_val <= THRESHOLD_1)
    {
        cracker_led(1);
        return 1;
    }
    else if (adc_val <= THRESHOLD_2 && adc_val > THRESHOLD_1)
    {
        cracker_led(2);
        return 2;
    }
    else if (adc_val <= THRESHOLD_3 && adc_val > THRESHOLD_2)
    {
        cracker_led(3);
        return 3;
    }
    else if (adc_val > THRESHOLD_3)
    {
        cracker_led(4);
        return 4;
    }
    else
        //test failed
    {
        cracker_led(-1);
        return -1;
    }

}
//typedef for all the state functions we need
typedef state_t state_func_t(void);

state_t run_state(state_t cur_state);

static state_t do_state_start(void);
static state_t do_state_s_to_p(void);
static state_t do_state_collect_cracker(void);
static state_t do_state_id_cracker(void);
static state_t do_state_p_to_d1(void);
static state_t do_state_p_to_d2(void);
static state_t do_state_p_to_d3(void);
static state_t do_state_p_to_d4(void);
static state_t do_state_drop_cracker(void);
static state_t do_state_d1_to_p(void);
static state_t do_state_d2_to_p(void);
static state_t do_state_d3_to_p(void);
static state_t do_state_d4_to_p(void);
static state_t do_state_return_to_start(void);

//Table of the state functions
state_func_t* const state_table[NUM_STATES] =
{
    do_state_start, do_state_s_to_p, do_state_collect_cracker,
	do_state_id_cracker, do_state_p_to_d1, do_state_p_to_d2,
	do_state_p_to_d3, do_state_p_to_d4, do_state_drop_cracker,
	do_state_d1_to_p, do_state_d2_to_p, do_state_d3_to_p,
	do_state_d4_to_p, do_state_return_to_start
};

//function that runs the state machine.
//Calls the next state
state_t run_state(state_t cur_state)
{
    return state_table[cur_state]();
}

static state_t do_state_start(void)
{
	//TODO: When does the machine start?
	//Is it on power on? 
	//if so, doesn't need this state at all.
	//Can go straight to the 'move to P stage'
	return STATE_S_TO_P;
}

//Operation: set motor speed at start, and follow the line out of S
//Use line following algorithm until we've reached P. Repeatedly call this!
//Set a stopwatch? Maybe to just ensure that we haven't messed up. ERROR DETECTION
//Wait for the front microswitch to make contact with the support at P. 
//This happens when the value read from the specific I2C chip goes high
//When it does, reset the emergency stop mode (ie reenable).
//Then move to the next state
static state_t do_state_s_to_p(void)
{
    rlink.command(MOTOR_1_GO, MOTOR1_OFFSET);
	rlink.command(MOTOR_4_GO, MOTOR4_OFFSET);
    int junct_count = 0;
    while((rlink.request(READ_ARM_MICROSWITCH) & PIN_ARM_MICROSWITCH) != PIN_ARM_MICROSWITCH)
    {
        int val = 0;
        if (junct_count == 3)
            //reached the final junction before P. Probably best to go slowly to ensure alignment is ok
            //for cracker pickup.
            //call white_line_straight with argument 0 to ensure we go slow for correct alignment.
            val = white_line_straight(0);
        else
            val = white_line_straight();
        
        if(val == LINE_LOST)
            find_a_line();
        else if (val == LINE_JUNCTION)
        {
            junct_count++;
            delay(JUNCTION_DELAY);
        }            
    }
    //now we've reached P, go to the next state
    rlink.command(MOTOR_1_GO,0);
    rlink.command(MOTOR_4_GO,0);
    return STATE_COLLECT_CRACKER;
}

//Start a stopwatch at the beginning because this will tell us whether
//we've collected the cracker or not.
//Start the motor/mechanism to close the claws
//wait until the timer has elapsed...
//then move to ID cracker.
//Need some error detection here to see if we've actually picked up the cracker or not?
static state_t do_state_collect_cracker(void)
{
    stopwatch s = stopwatch();
    s.start();
    //Assume that the actuator is in right now (mechanism is open)
    int val = rlink.request(READ_ACTUATOR_AND_LED);
    val = val ^ ACTUATOR_TRIGGER;
    rlink.command(WRITE_ACTUATOR_AND_LED, val);
    while(s.read() < TIME_FOR_PICKUP)
    {
        ;
    }
    //We've now successfully picked up the cracker. Can start to id it
    return STATE_ID_CRACKER;
}

//complicated state. Call a id_cracker to find out what cracker we're dealing with
//Having found this, call the relevant function
static state_t do_state_id_cracker(void)
{
    switch (id_cracker())
    {
        case 1:
            cracker_carried = 1;
            return STATE_P_TO_D1;
        case 2:
            cracker_carried = 2;
            return STATE_P_TO_D2;
        case 3:
            cracker_carried = 3;
            return STATE_P_TO_D3;
        case 4:
            cracker_carried = 4;
            return STATE_P_TO_D4;
        default:
            cracker_carried = 1;
            return STATE_P_TO_D1;
    }
}
static state_t do_state_p_to_d1(void)
{
   
    stopwatch watch;
    watch.start();
    while(watch.read() < 5000) //better to have a time bound here.
    {
        line_type_t ret = reverse_straight();
        if (ret == LINE_JUNCTION)
            break;
    }
    turn_180();
    while(true) 
    {
        line_type_t ret = white_line_straight();
        if (ret == LINE_LOST)
            find_a_line();
        else if (ret == LINE_JUNCTION)
            break;
    }
    turn_right();
    while((rlink.request(READ_FRONT_MICROSWITCH) & PIN_FRONT_MICROSWITCH) != PIN_FRONT_MICROSWITCH)
    {
        if(white_line_straight() == LINE_LOST)
        {
            find_a_line();
        }
    }
    //we've made contact with the box at D1. Move to next state.
    rlink.command(MOTOR_1_GO,0);
    rlink.command(MOTOR_4_GO,0);
    return STATE_DROP_CRACKER;

}


static state_t do_state_p_to_d2(void)
{
    //Flash some LEDs here to indicate which cracker we're carrying.

    stopwatch watch;
    watch.start();
    while(watch.read() < 5000) //better to have a time bound here.
    {
        line_type_t ret = reverse_straight();
        if (ret == LINE_JUNCTION)
            break;
    }
    watch.stop();
    turn_180();
    int junct_number = 0;
    while (junct_number < 3) 
    { 
        line_type_t ret = white_line_straight();
        if (ret == LINE_LOST)
            find_a_line();
        else if (ret == LINE_JUNCTION)
        {
            ++junct_number;
            //delay the next call to white_line_straight to ensure we've crossed the junction fully.
            delay(JUNCTION_DELAY);        
        }
    }

    turn_right();
    watch.start();
    while (watch.read() < TIME_FOR_RAMP)
    { 
        line_type_t ret = white_line_straight();
        if (ret == LINE_LOST)
            find_a_line();
        else if (ret == LINE_JUNCTION)
            break;
    }
    turn_left();
    while((rlink.request(READ_FRONT_MICROSWITCH) & PIN_FRONT_MICROSWITCH) != PIN_FRONT_MICROSWITCH)
    {
        if(white_line_straight() == LINE_LOST)
        {
            find_a_line();
        }
    }
    //we've made contact with the box at D2. Move to next state.
    rlink.command(MOTOR_1_GO,0);
    rlink.command(MOTOR_4_GO,0);
    return STATE_DROP_CRACKER;
}


static state_t do_state_p_to_d3(void)
{
    //Flash some LEDs here to indicate which cracker we're carrying.

    stopwatch watch;
    watch.start();
    while(watch.read() < 5000) //better to have a time bound here.
    {
        line_type_t ret = reverse_straight();
        if (ret == LINE_JUNCTION)
            break;
    }
    watch.stop();
    turn_left(); //configured by timing conditions
    //error correction here- this may have gone wrong.

    while (true) //TODO FIX THIS, add some timing!
    { 
        line_type_t ret = white_line_straight();
        if (ret == LINE_LOST)
            find_a_line();
        else if (ret == LINE_BLACK)
            break;
    }
    //we've now reached the white ramp...
    watch.start();
    while(watch.read() < TIME_FOR_WHITE_RAMP)
    {
        line_type_t ret = black_line_straight();
        if (ret == LINE_LOST)
            find_a_black_line();
        else if (ret == LINE_OK)
            break;
    }
    rlink.command(MOTOR_1_GO,MOTOR1_OFFSET+50);
    rlink.command(MOTOR_4_GO, MOTOR4_OFFSET+50);
    delay(800);
    //reached the top of the white ramp now, hopefully. Need ERROR CHECKING HERE!!!!

    while((rlink.request(READ_FRONT_MICROSWITCH) & PIN_FRONT_MICROSWITCH) != PIN_FRONT_MICROSWITCH)
    {
        if(white_line_straight() == LINE_LOST)
        {
            find_a_line();
        }
    }
    //we've made contact with the box at D3. Move to next state.
    rlink.command(MOTOR_1_GO,0);
    rlink.command(MOTOR_4_GO,0);
    return STATE_DROP_CRACKER;
}

static state_t do_state_p_to_d4(void)
{

    stopwatch watch;
    watch.start();
    while(watch.read() < 5000) //better to have a time bound here.
    {
        line_type_t ret = reverse_straight();
        if (ret == LINE_JUNCTION)
            break;
    }
    turn_180();
    while(true) //this will take longer. May need a better system here!
    {
        line_type_t ret = white_line_straight();
        if (ret == LINE_LOST)
            find_a_line();
        else if (ret == LINE_JUNCTION)
            break;
    }
    turn_right();

    while(true) //this will take longer. May need a better system here!
    {
        line_type_t ret = white_line_straight();
        if (ret == LINE_LOST)
            find_a_line();
        else if (ret == LINE_JUNCTION)
            break;
    }

    turn_right();

    while((rlink.request(READ_FRONT_MICROSWITCH) & PIN_FRONT_MICROSWITCH) != PIN_FRONT_MICROSWITCH)
    {
        if(white_line_straight() == LINE_LOST)
        {
            find_a_line();
        }
    }
    rlink.command(MOTOR_1_GO,0);
    rlink.command(MOTOR_4_GO,0);
    return STATE_DROP_CRACKER;
}

//activate actuator (release the cracker)
//activate replace cracker LED, wait for TIME_FOR_DROP, then shut it off.
static state_t do_state_drop_cracker(void)
{
    int val = rlink.request(READ_ACTUATOR_AND_LED);
    val = ((val ^ ACTUATOR_TRIGGER) ^ REPLACE_CRACKER);
    rlink.command(WRITE_ACTUATOR_AND_LED, val);
    delay(TIME_FOR_DROP);
    rlink.command(WRITE_ACTUATOR_AND_LED, (val ^ REPLACE_CRACKER));

    switch(cracker_carried)
    {
        case 1:
            cracker_carried = -1; //to show that the cracker has been dropped.
            return STATE_D1_TO_P;
        case 2:
            cracker_carried = -1; //to show that the cracker has been dropped.
            return STATE_D2_TO_P;
        case 3:
            cracker_carried = -1; //to show that the cracker has been dropped.
            return STATE_D3_TO_P;
        case 4:
            cracker_carried = -1; //to show that the cracker has been dropped.
            return STATE_D4_TO_P;
    }
    cracker_carried = -1;
    return STATE_D1_TO_P;
}

static state_t do_state_d1_to_p(void)
{

    //going to have to be pseudocode for now, until line detection algorithms are done

    stopwatch watch;
    watch.start();
    while(watch.read() < 5000) //better to have a time bound here.
    {
        line_type_t ret = reverse_straight();
        if (ret == LINE_JUNCTION)
            break;
    }

    turn_180();
    while(true) //this will take longer. May need a better system here!
    {
        line_type_t ret = white_line_straight();
        if (ret == LINE_LOST)
            find_a_line();
        else if (ret == LINE_JUNCTION)
            break;
    }

    turn_left();

    while(true) //this will take longer. May need a better system here!
    {
        line_type_t ret = white_line_straight();
        if (ret == LINE_LOST)
            find_a_line();
        else if (ret == LINE_JUNCTION)
            break;
    }

    while((rlink.request(READ_ARM_MICROSWITCH) & PIN_ARM_MICROSWITCH) != PIN_ARM_MICROSWITCH)
    {
        if(white_line_straight(0) == LINE_LOST)
        {
            find_a_line();
        }
    }
    rlink.command(MOTOR_1_GO,0);
    rlink.command(MOTOR_4_GO,0);
    return STATE_COLLECT_CRACKER;
}

static state_t do_state_d2_to_p(void)
{
    
    //going to have to be pseudocode for now, until line detection algorithms are done

    stopwatch watch;
    watch.start();
    while(watch.read() < 5000) //better to have a time bound here.
    {
        line_type_t ret = reverse_straight();
        if (ret == LINE_JUNCTION)
            break;
    }
    watch.stop();
    turn_left(); //configured by timing conditions
    //error correction here- this may have gone wrong.
    int junct_number = 0;
    while(junct_number < 2) //this will take longer. May need a better system here!
    {
        line_type_t ret = white_line_straight();
        if (ret == LINE_LOST)
            find_a_line();
        else if (ret == LINE_JUNCTION)
        {
            ++count;
            watch.start();
            while(watch.read() < JUNCTION_DELAY)
                ;
            watch.stop();
        }
    }

    turn_left();


    while((rlink.request(READ_ARM_MICROSWITCH) & PIN_ARM_MICROSWITCH) != PIN_ARM_MICROSWITCH)
    {
        if(white_line_straight(0) == LINE_LOST)
        {
            find_a_line();
        }
    }
    rlink.command(MOTOR_1_GO,0);
    rlink.command(MOTOR_4_GO,0);
    return STATE_COLLECT_CRACKER;
}

static state_t do_state_d3_to_p(void)
{
    //going to have to be pseudocode for now, until line detection algorithms are done

    stopwatch watch;
    watch.start();
    while(watch.read() < 5000) //better to have a time bound here.
    {
        line_type_t ret = reverse_straight();
        if (ret == LINE_JUNCTION)
            break;
    }

    turn_180();
    watch.stop();
    //error correction here- this may have gone wrong.
    watch.start();
    while (watch.read() < TIME_TO_WHITE_RAMP) //TODO FIX THIS, add some timing!
    { 
        line_type_t ret = white_line_straight();
        if (ret == LINE_LOST)
            find_a_line();
        else if (ret == LINE_BLACK)
            break;
    }
    rlink.command(MOTOR_1_GO,MOTOR1_OFFSET + 30);
    rlink.command(MOTOR_4_GO,MOTOR4_OFFSET + 30);
    //we've now reached the white ramp...
    watch.stop();
    watch.start();
    while(watch.read() < TIME_FOR_WHITE_RAMP)
    {
        line_type_t ret = black_line_straight();
        if (ret == LINE_LOST)
            find_a_black_line();
        else if (ret == LINE_OK)
            break;
    }
    //reached the bottom of the white ramp now, hopefully. Need ERROR CHECKING HERE!!!!

    turn_left();

    while((rlink.request(READ_ARM_MICROSWITCH) & PIN_ARM_MICROSWITCH) != PIN_ARM_MICROSWITCH)
    {
        if(white_line_straight(0) == LINE_LOST)
        {
            find_a_line();
        }
    }
    //we've made contact with P. Move to next state.
    rlink.command(MOTOR_1_GO,0);
    rlink.command(MOTOR_4_GO,0);
    return STATE_COLLECT_CRACKER;
}

static state_t do_state_d4_to_p(void)
{
    //going to have to be pseudocode for now, until line detection algorithms are done

    stopwatch watch;
    watch.start();
    while(watch.read() < 5000) //better to have a time bound here.
    {
        line_type_t ret = reverse_straight();
        if (ret == LINE_JUNCTION)
            break;
    }

    turn_left(); //configured by timing conditions
    //error correction here- this may have gone wrong.

    while(true) //this will take longer. May need a better system here!
    {
        line_type_t ret = white_line_straight();
        if (ret == LINE_LOST)
            find_a_line();
        else if (ret == LINE_JUNCTION)
            break;
    }

    turn_left();

    while((rlink.request(READ_ARM_MICROSWITCH) & PIN_ARM_MICROSWITCH) != PIN_ARM_MICROSWITCH)
    {
        if(white_line_straight() == LINE_LOST)
        {
            find_a_line();
        }
    }
    //we've made contact with P. Move to next state.
    rlink.command(MOTOR_1_GO,0);
    rlink.command(MOTOR_4_GO,0);
    return STATE_COLLECT_CRACKER;
}

static state_t do_state_return_to_start(void)
{
    //sort this one out later
    return STATE_RETURN_TO_START;
}

//initialize the link/robot.
//set ramp speed? Not sure if necessary here. Ask others.
static int init(void)
{
	if (!rlink.initialise (ROBOT_NUM)) 
	{ // setup the link
        cout << "Cannot initialise link" << endl;
        rlink.print_errs("  ");
        return -1;
	}
    else
    {
        return 1;
    }

}

int main(int argc, char* argv[])
{
	if (init() == -1) 
    { // setup the link
	   cout << "Cannot initialise link" << endl;
	   rlink.print_errs("  ");
	   return -1;
	}
    current_state = STATE_START;
    state_t new_state;
    //rlink.command(RAMP_TIME, 50); //for now, a default ramp time.

	while(true)
    {
        new_state = run_state(current_state);
        if (new_state != current_state)
        {
            current_state = new_state;
        }
    }
}
