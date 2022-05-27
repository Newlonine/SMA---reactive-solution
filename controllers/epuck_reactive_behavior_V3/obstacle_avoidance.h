#ifndef OBSTACLE_AVOIDANCE
#define OBSTACLE_AVOIDACE

#include <stdbool.h>

#define MAX_SPEED 4.71

void init_array_speed(double array_speed[2]);
void update_array_speed(double array_speed[2], double speed);
bool in_a_corner(double lls[2], double lrs[2], double left_speed, double right_speed);

void turn_left(double *left_speed, double *right_speed,double degree);
void turn_right(double *left_speed, double *right_speed, double degree);
void go_forward(double *left_speed, double *right_speed);
void go_backward(double *left_speed, double *right_speed);
void stop_motors(double *left_speed, double *right_speed);

#endif