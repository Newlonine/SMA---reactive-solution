/*
 * File:          epuck_reactive_behavior_V2.c
 * Date: 19/05/2022
 * Description: First version of the reactive behavior of robots using a camera. It also 
 *              uses a receiver to pick up a signal from the supervisor if the robot is within the
 *              target range. 
 * Author: Nolwenn
 * Modifications: 19/05 : file created
                  23/05 : add a receiver
 */

#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/camera_recognition_object.h>
#include <webots/camera.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "obstacle_avoidance.h"

#define TIME_STEP 32
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

  // receiver
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, 10);
  wb_receiver_set_channel(receiver, id); // each robot has its own channel

  /* --- ---*/
  // camera stuff
  int nb_objects;
  const WbCameraRecognitionObject *objects_detected;
  char *model_object = "";
  double y = 0.0; // x and y position

  // movement stuff
  double left_speed = 0, right_speed = 0, last_left_speed = -1, last_right_speed = -1;
  double lls[2], lrs[2]; // used for stocking the left and right speed at time t-1 and t-2
  int countdown = -1;
  int counterMotor = 0;

  // receiver stuff
  const char *message_received = "";
  bool receiver_enabled = true;

  bool finished = false;

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
    y = 0.0;
    model_object = "";

    // camera and object recognition
    nb_objects = wb_camera_recognition_get_number_of_objects(camera);
    if (nb_objects > 0)
    {
      objects_detected = wb_camera_recognition_get_objects(camera);
      model_object = objects_detected->model;
      // x = objects_detected->position[0];
      y = objects_detected->position[1];

     /* printf("model : %s;\n",
              model_object); */
    }

    //
    while (receiver_enabled && wb_receiver_get_queue_length(receiver) > 0)
    {
      message_received = wb_receiver_get_data(receiver);
      printf("Received : %s\n", message_received);
      wb_receiver_next_packet(receiver);
    }
 
    // 1) the target is in front of the robot, it stops
    if (!strcmp(message_received, "stop") || finished)
    {
      left_speed = 0.0;
      right_speed = 0.0;
      receiver_enabled = false;
      finished = true;
      wb_receiver_disable(receiver);
    }
    else
    {
      // 2)avoid obstacles
      if (countdown > 0)
      {
        counterMotor = 0;
        countdown--;

        left_speed = -MAX_SPEED;
        right_speed = MAX_SPEED;
      }
      else
      {
        if (in_a_corner(lls, lrs, left_speed, right_speed))
          counterMotor++;

        if (counterMotor >= 4)
        {
          countdown = 70;
        }
        else
        {
          // distanceSensor data
          double ps_values[8];
          for (i = 0; i < 8; i++)
          {
            ps_values[i] = wb_distance_sensor_get_value(ps[i]);
          }

          left_speed = MAX_SPEED;
          right_speed = MAX_SPEED;

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
          int r;
          r = rand() % 100;
          if (r < 5)
          {
            left_speed = MAX_SPEED / 8;
            right_speed = MAX_SPEED;
          }
          else if (r >= 5 && r < 10)
          {
            left_speed = MAX_SPEED;
            right_speed = MAX_SPEED / 8;
          }
#endif

          if ((left_obstacle || right_obstacle) && strcmp(model_object, "cible"))
          {
            if (lcrs && lsrc && lrf && !lrr)
            {
              left_speed = -MAX_SPEED;
              right_speed = -MAX_SPEED;
            }
            else if (lcrs && lsrc)
            {
              left_speed = MAX_SPEED;
              right_speed = MAX_SPEED;
            }
            else
            {
              if (left_front)
              {
                left_speed = MAX_SPEED;
                right_speed = -MAX_SPEED;
              }
              else if (right_front)
              {
                left_speed = -MAX_SPEED;
                right_speed = MAX_SPEED;
              }
              else if (left_corner)
              {
                left_speed = MAX_SPEED;
                right_speed = MAX_SPEED / 5;
              }
              else if (right_corner)
              {
                left_speed = MAX_SPEED / 5;
                right_speed = MAX_SPEED;
              }
              else if (left_side)
              {
                left_speed = MAX_SPEED;
                right_speed = MAX_SPEED / 8;
              }
              else if (right_side)
              {
                left_speed = MAX_SPEED / 8;
                right_speed = MAX_SPEED;
              }
            }
          }
          else
          {
            if (!strcmp(model_object, "cible"))
            {
              // 3) target detected, the robot goes towards it
              if (y >= 0.05)
              { // target to the left
                left_speed = -MAX_SPEED;
                right_speed = MAX_SPEED;
              }
              else if (y < -0.05)
              { // target to the right
                left_speed = MAX_SPEED;
                right_speed = -MAX_SPEED;
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