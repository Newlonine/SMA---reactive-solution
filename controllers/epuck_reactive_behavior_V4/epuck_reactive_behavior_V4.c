/*
 * File:          epuck_reactive_behavior_V3.c
 * Date: 19/05/2022
 * Description: The supervisor no longer communicate with the robots.
 *              If the robot is in the range of the target, it has a % of change to stop.
 * Author: Nolwenn
 * Modifications: 19/05 : file created (epuck_reacive_behavior_V3.c)
                  23/05 : add a receiver
                  24/05 : remove the last rule which was if there is no oject then rotate to the left
                          remove the receiver
                          this version must be used with supervisor_controller_V3.c
 */

#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/camera_recognition_object.h>
#include <webots/camera.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "obstacle_avoidance.h"

#define TIME_STEP 32
#define RANGE_TARGET 0.25
//#define RANDOM_MOVE

/*
 * return the id of a robot
 * used to initialize random generator
 */
int get_id(const char *name)
{
  if (!strcmp(name, "epuck"))
  {
    return 1;
  }

  char id[1];
  id[0] = name[7];

  return atoi(id) + 1;
}

double random(int min, int max)
{
  int delta;
  double res;

  delta = max - min + 1;
  res = rand() % delta;
  res = (res + min) / 100;

  return res;
}

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

  // initialization of the random number generator
  const char *nom = wb_robot_get_name();
  int id = get_id(nom);
  srand(time(NULL) * id);

  /*--- Enable all sensors and actuators ---*/
  // IR sensors
  int i;
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
      "ps0", "ps1", "ps2", "ps3",
      "ps4", "ps5", "ps6", "ps7"};

  for (i = 0; i < 8; i++)
  {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  // motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // camera
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, 10);
  wb_camera_recognition_enable(camera, 10);

  /* --- ---*/
  // camera stuff
  int nb_objects;
  const WbCameraRecognitionObject *objects_detected;
  char *model_obejcts = "";
  double x_cible = 0.0, x_robot = 0.0;
  double y_cible = 0.0, y_robot = 0.0; // y position

  // movement stuff
  double left_speed = 0, right_speed = 0, last_left_speed = -1, last_right_speed = -1;
  double lls[2], lrs[2]; // used for stocking the left and right speed at time t-1 and t-2
  int countdown = -1;
  int counterMotor = 0;

  // other stuff
  bool finished = false;
  int r; //random

  // lls[0] = speed at t-1; lls[1] = speed at t-2;
  init_array_speed(lls);
  init_array_speed(lrs);

  while (wb_robot_step(TIME_STEP) != -1)
  {
    // refresh speed
    last_left_speed = left_speed;
    last_right_speed = right_speed;
    update_array_speed(lls, last_left_speed);
    update_array_speed(lrs, last_right_speed);

    // refresh camera stuff
    model_obejcts = "";

    // camera and object recognition
    nb_objects = wb_camera_recognition_get_number_of_objects(camera);
    if (nb_objects > 0)
    {
      objects_detected = wb_camera_recognition_get_objects(camera);
      model_obejcts = objects_detected->model;

      if (!strcmp(model_obejcts, "cible"))
      {
        x_cible = objects_detected->position[0];
        y_cible = objects_detected->position[1];
      }
      else
      {
        x_robot = objects_detected->position[0];
        y_robot = objects_detected->position[1];
      }
    }

   // if (id == 1 && !strcmp(model_obejcts, "cible"))
   //   printf("CIBLE : x = %g, y = %g\n", x_cible, y_cible);

    // 1) the robot is in the range of the target, it stops
    if ( (x_cible > 0.0 && x_cible < RANGE_TARGET) || finished)
    {
      r = rand()%100;

      // chance of stopping at the edge oh the range of the target
      if(r <= 2 || finished){
        stop_motors(&left_speed, &right_speed);
        finished = true;
      }else{ // it slows down
        set_speed(&left_speed, &right_speed, 0.5);
      }
    }
    else
    {
      // 2)avoid obstacles
      if (countdown > 0)
      {
        counterMotor = 0;
        countdown--;

        turn_left(&left_speed, &right_speed, -1);
      }
      else
      {
        if (in_a_corner(lls, lrs, left_speed, right_speed))
          counterMotor++;

        if (counterMotor >= 4)
          countdown = 70;

        else
        {
          // distanceSensor data
          double ps_values[8];
          for (i = 0; i < 8; i++)
            ps_values[i] = wb_distance_sensor_get_value(ps[i]);

          go_forward(&left_speed, &right_speed);

          // capteurs gauche
          bool left_rear = ps_values[4] > 80.0;
          bool left_side = ps_values[5] > 80.0;
          bool left_corner = ps_values[6] > 80.0;
          bool left_front = ps_values[7] > 80.0;
          // capteurs droits
          bool right_rear = ps_values[3] > 80.0;
          bool right_side = ps_values[2] > 80.0;
          bool right_corner = ps_values[1] > 80.0;
          bool right_front = ps_values[0] > 80.0;

          bool lcrs = left_corner && right_side;
          bool lsrc = left_side && right_corner;
          bool lrf = left_front && right_front;
          bool lrr = left_rear && right_rear;

          bool left_obstacle = left_rear || left_side || left_corner || left_front;
          bool right_obstacle = right_rear || right_side || right_corner || right_front;

#ifdef RANDOM_MOVE
          /*
           * 5% to turn right, 5% to turn left
           */
          r = rand() % 100;
          double speed = random(-10, 10);
          if (r < 25)
            turn_left(&left_speed, &right_speed, speed);
          else if (r >75)
            turn_right(&left_speed, &right_speed, speed);
#endif

          if ((left_obstacle || right_obstacle) && strcmp(model_obejcts, "cible"))
          {
            if (lcrs && lsrc && lrf && !lrr)
              go_backward(&left_speed, &right_speed);

            else if (lcrs && lsrc)
              go_forward(&left_speed, &right_speed);

            else
            {
              if (left_front)
                turn_right(&left_speed, &right_speed, -1);

              else if (right_front)
                turn_left(&left_speed, &right_speed, -1);

              else if (left_corner)
                turn_right(&left_speed, &right_speed, 0.3);

              else if (right_corner)
                turn_left(&left_speed, &right_speed, 0.3);

              else if (left_side)
                turn_right(&left_speed, &right_speed, 0.8);

              else if (right_side)
                turn_left(&left_speed, &right_speed, 0.8);
            }
          }
          else
          {
            if (!strcmp(model_obejcts, "cible"))
            {
              counterMotor = 0;
              // 3) target detected, the robot slows and goes towards it
              set_speed(&left_speed, &right_speed, 0.6);
              if (y_cible >= 0.05)
                turn_left(&left_speed, &right_speed, 0.3);
              else if (y_cible < -0.05)
                turn_right(&left_speed, &right_speed, 0.3);
            }
            else
            {
              nb_objects = wb_camera_recognition_get_number_of_objects(camera);
              if (nb_objects > 4)
              {
                if (y_robot >= 0.1)
                  turn_left(&left_speed, &right_speed, 0.3);
                else if (y_robot <= -0.1)
                  turn_right(&left_speed, &right_speed, 0.3);
              }
            }
          }
        }
      }
    }

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  };

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}