#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include <robot_instr.h>
#include <robot_link.h>

#define ROBOT_NUM 11   // The id number (see below)
#define LINE_DEBUG 0

//straight movement offsets- previously 198
#define MOTOR1_OFFSET   		70
#define MOTOR4_OFFSET   		194

extern robot_link rlink;      // datatype for the robot link
typedef enum {
LINE_OK = 0, LINE_LOST, LINE_JUNCTION, LINE_BLACK
} line_type_t;
//Functions that enable line following are prototyped in this file.
line_type_t white_line_straight(bool key = 1);
line_type_t reverse_straight(void);
//do these by timing
void turn_left(void);
void turn_right(void);
void turn_180(void);
//for the dodgy ramp
line_type_t black_line_straight(void);
//if lost
line_type_t find_a_line(void);
line_type_t find_a_black_line(void);

#endif 
