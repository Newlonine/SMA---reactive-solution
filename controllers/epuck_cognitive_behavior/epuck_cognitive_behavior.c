/*
 * File:          epuck_cognitive_behavior.c
 * Date: 01/06/2022
 * Description: This controller uses the epuck_reactive_behavior_V4.c 
 *              and a function called "compute_odometry" which allows an 
 *              agent to calculate its position in the environment.
 * Author: Nolwenn
 * Modifications:
 */

#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <webots/motor.h>
#include <webots/camera_recognition_object.h>
#include <webots/camera.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include "obstacle_avoidance.h"

#define TIME_STEP 32
#define RANGE_TARGET 0.20
#define WB_CHANNEL_BROADCAST -1
#define EMITTER_RANGE -1
#define WHEEL_RADIUS 0.02
#define AXLE_LENGTH 0.052
#define RANGE (1024 / 2)

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

void send_msg(WbDeviceTag emitter, char message[128])
{
  wb_emitter_send(emitter, message, strlen(message) + 1);
}

// function based on "e-puck" controller in Webots\projects\robots\gctronic\e-puck\controllers\e-puck
void compute_odometry(WbDeviceTag left_position_sensor, WbDeviceTag right_position_sensor, double robot_position_values[3], double last_ps_values[2])
{
  double l = wb_position_sensor_get_value(left_position_sensor);
  double r = wb_position_sensor_get_value(right_position_sensor);

  double diff_l = l - last_ps_values[0];
  double diff_r = r - last_ps_values[1];

  double dl = (diff_l * WHEEL_RADIUS); // distance left wheel between two steps
  double dr = (diff_r * WHEEL_RADIUS); // distance right wheel betwenn two steps
  double da = (dr - dl) / AXLE_LENGTH; // delta orientation

  last_ps_values[0] = l;
  last_ps_values[1] = r;
  if (da < 0.001 && da > -0.001)
    da = 0;

  printf("-----------------------------------------------\n");
  printf("estimated distance covered by left wheel: %g m.\n", dl);
  printf("estimated distance covered by right wheel: %g m.\n", dr);
  printf("estimated change of orientation: %g rad.\n", da);

  // this doesn't work yet
  double d = (dl + dr) / 2;
  robot_position_values[0] += (d * cos(robot_position_values[2] + da / 2));
  robot_position_values[1] += (d * sin(robot_position_values[2] + da / 2));
  robot_position_values[2] += da;
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
  wb_receiver_set_channel(receiver, WB_CHANNEL_BROADCAST); // receive from all channels

  // emitter
  WbDeviceTag emitter = wb_robot_get_device("emitter");
  wb_emitter_set_range(emitter, EMITTER_RANGE);
  wb_emitter_set_channel(emitter, WB_CHANNEL_BROADCAST); // transmits to all channels

  // position sensor
  WbDeviceTag left_position_sensor = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_position_sensor = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_position_sensor, TIME_STEP);
  wb_position_sensor_enable(right_position_sensor, TIME_STEP);

  /* --- variables ---*/
  // camera stuff
  int nb_objects;
  const WbCameraRecognitionObject *objects_detected;
  char *model_objects = "";
  bool target_detected = false;
  double x_cible = 0.0;
  double y_cible = -1.0, y_robot = 0.0; // y position

  // movement stuff
  double left_speed = 0, right_speed = 0, last_left_speed = -1, last_right_speed = -1;
  double lls[2], lrs[2]; // used for stocking the left and right speed at time t-1 and t-2
  int countdown = -1;
  int counterMotor = 0;

  // other stuff
  bool finished = false;
  int r; // random
  int step = 0;
  double robot_position[3] = {0, 0, 0}; // x, y, theta
  double last_ps_values[2] = {0, 0};

  // receiver and emitter stuff
  const char *messageReceived = "";
  const double *dir;
  char message[128];
  double x_msg = 0.0, y_msg = 0.0;
  double signal, distance;

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
    model_objects = "";
    target_detected = false;

    // camera and object recognition
    nb_objects = wb_camera_recognition_get_number_of_objects(camera);
    if (nb_objects > 0)
    {
      objects_detected = wb_camera_recognition_get_objects(camera);

      for (i = 0; i < nb_objects; i++)
      {
        model_objects = objects_detected[i].model;

        if (!strcmp(model_objects, "cible"))
        {
          target_detected = true;
          x_cible = objects_detected[i].position[0];
          y_cible = objects_detected[i].position[1];
        }
        else
        {
          y_robot = objects_detected[i].position[1];
        }
      }
    }

    while (wb_receiver_get_queue_length(receiver) > 0)
    {
      messageReceived = wb_receiver_get_data(receiver);
      dir = wb_receiver_get_emitter_direction(receiver);
      signal = wb_receiver_get_signal_strength(receiver);
      distance = sqrt(1 / signal);

      x_msg = dir[0];
      y_msg = dir[1];
      wb_receiver_next_packet(receiver);
    }

    // initialization
    if (step < 100 && !strcmp(model_objects, "GCtronic e-puck"))
    {
      turn_left(&left_speed, &right_speed, -1);
    }
    else
    {

      // 1) the robot is in the range of the target, it stops
      if (finished || (x_cible > 0.0 && x_cible < RANGE_TARGET && y_cible > -0.05 && y_cible < 0.05))
      {
        r = rand() % 100;

        // chance of stopping at the edge of the range of the target
        if (r <= 50 || finished)
        {
          stop_motors(&left_speed, &right_speed);
          finished = true;
        }
        else
        { // it slows down
          if (y_cible >= 0.05)
            turn_left(&left_speed, &right_speed, 0.1);
          else if (y_cible < -0.05)
            turn_right(&left_speed, &right_speed, 0.1);
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

            if ((left_obstacle || right_obstacle) && !target_detected)
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
              if (target_detected)
              {
                counterMotor = 0;
                // 3) target detected, the robot slows and goes towards it
                set_speed(&left_speed, &right_speed, 0.6);
                if (y_cible >= 0.1)
                  turn_left(&left_speed, &right_speed, 0.1);
                else if (y_cible < -0.1)
                  turn_right(&left_speed, &right_speed, 0.1);
              }
              else
              {
                nb_objects = wb_camera_recognition_get_number_of_objects(camera);
                if (nb_objects >= 6)
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
    }

    if (step % 50 == 0)
    {
      compute_odometry(left_position_sensor, right_position_sensor, robot_position, last_ps_values);
      printf("(%g, %g)\n", robot_position[0], robot_position[1]);
    }
    step++;
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  };

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}