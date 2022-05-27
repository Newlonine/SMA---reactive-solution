#include "obstacle_avoidance.h"
#include <stdbool.h>

/*
 *
 */
void init_array_speed(double array_speed[2])
{
    array_speed[0] = 0;
    array_speed[1] = -1;
}

/*
 *
 */
void update_array_speed(double array_speed[2], double speed)
{
    array_speed[1] = array_speed[0];
    array_speed[0] = speed;
}

/*
 * Return true if robot speeds have been reversed
 * First we compare the left_speed with the right_speed, 
 * if they’re the same speed, the robot goes forward or backward.
 * Then we check if the speed has been reversed between 2 steps and
 * we compare the speed at t-1 with the speed at t-2, 
 * if the speed is the same for 2 steps in the same motor, it returns false
 */
bool in_a_corner(double lls[2], double lrs[2], double left_speed, double right_speed)
{
 /* if (left_speed == right_speed)
        return false;

    if (lls[0] != -right_speed || lrs[0] != -left_speed)
        return false;

    if (lls[0] == lls[1] || lrs[0] == lrs[1])
        return false; */

    if((left_speed == -right_speed) &&
       (lls[0] == -right_speed && lrs[0] == -left_speed) &&
       (lls[0] == -lls[1] && lrs[0] == -lrs[1]) &&
       (lls[1] == -left_speed && lrs[1] == -right_speed))
        return true;
    return false;
}

/*
 *
 */
void turn_left(double *left_speed, double *right_speed, double degree)
{
    *left_speed = degree * MAX_SPEED;
    *right_speed = MAX_SPEED;
}

void turn_right(double *left_speed, double *right_speed, double degree)
{
    *left_speed = MAX_SPEED;
    *right_speed = degree * MAX_SPEED;
}

void go_forward(double *left_speed, double *right_speed)
{
    *left_speed = MAX_SPEED;
    *right_speed = MAX_SPEED;
}

void go_backward(double *left_speed, double *right_speed)
{
    *left_speed = -MAX_SPEED;
    *right_speed = -MAX_SPEED;
}

void stop_motors(double *left_speed, double *right_speed){
    *left_speed = 0.0;
    *right_speed = 0.0;
}

void set_speed(double *left_speed, double *right_speed, double speed){
    *left_speed = speed * MAX_SPEED;
    *right_speed = speed * MAX_SPEED;
}