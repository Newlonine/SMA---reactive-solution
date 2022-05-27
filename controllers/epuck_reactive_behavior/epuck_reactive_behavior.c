/*
 * File: epuck_reactive_behavior.c
 * Date: 15/05/22
 * Description: First version of the reactive behavior, the target and robots communicate
 *              through receiver and emitter. If a robot picks up a signal from the target, 
 *              it goes towards it.
 *              Use a simple obstacle avoidance.
 * Author: Nolwenn 
 * Modifications:
 */
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define TIME_STEP 32
#define MAX_SPEED 6.28
#define WB_CHANNEL_BROADCAST -1
#define WB_CHANNEL_SUPERVISOR 2
#define RANDOM_MOVE

// return true if the msg comes from the target
bool is_cible(const char *message){
  return !strcmp(message,"cible");
}

void send_msg(WbDeviceTag emitter, char message[128]){
  wb_emitter_send(emitter, message, strlen(message) + 1);
}

void last_message(WbDeviceTag emitter, int id){
  wb_emitter_set_range(emitter, -1); //infinite range
  wb_emitter_set_channel(emitter,WB_CHANNEL_SUPERVISOR); //transmits to the supervisor channel

  char messageId[5];
  sprintf(messageId, "%d", id);
  send_msg(emitter, messageId);

  wb_emitter_set_range(emitter, 0.5);
  wb_emitter_set_channel(emitter,WB_CHANNEL_BROADCAST);
}

/*
 * return the id of a robot
 * used to initialize random generator
 */
int get_id(const char* name){
  if(!strcmp(name, "epuck")){
    return 1;
  }
  
  char id[1];
  id[0] = name[7];
  
  return atoi(id)+1;
}

int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  
  //initialization of the random number generator
  const char *nom;
  int id;
  nom = wb_robot_get_name();
  id = get_id(nom);
  srand(time(NULL)*id);
  
  int i;
  double y = 0.0, x = 0.0;
  double left_speed, right_speed; 
  bool termine = false; 
  
  //IR sensors
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };

  for (i = 0; i < 8; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  
  const char *messageReceived = "";
  const double *dir;
  char message[128];
  
  //receiver
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, 10);
  wb_receiver_set_channel(receiver,WB_CHANNEL_BROADCAST);
  
  //emitter
  WbDeviceTag emitter = wb_robot_get_device("emitter");
  wb_emitter_set_range(emitter, 0.2); //20 cm range
  wb_emitter_set_channel(emitter,WB_CHANNEL_BROADCAST); //transmits to all channels

  //motors
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  //main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    //distanceSensor data
    double ps_values[8];
    for (i = 0; i < 8 ; i++){
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    }

    /*--- Behavior ---*/ 

    //1) The target is in front of the robot, it stops
    if(((ps_values[0] > 200 || ps_values[7] > 200) && is_cible(messageReceived)) || termine){
      //last message      
      if(!termine){
        sprintf(message, "termine");
        send_msg(emitter, message);

        last_message(emitter, id);
      }else{
        sprintf(message, "cible");
        send_msg(emitter, message);
      }
      
      wb_emitter_set_range(emitter, 0.5); //50 cm range
      left_speed = 0.0;
      right_speed = 0.0;
      termine = true;
    
    } else {
      //send a message
      sprintf(message, "agent");
      send_msg(emitter, message);

      bool right_obstacle = ps_values[0] > 80.0 ||
                          ps_values[1] > 80.0 ||
                          ps_values[2] > 80.0;
                  
      bool left_obstacle = ps_values[5] > 80.0 ||
                         ps_values[6] > 80.0 ||
                         ps_values[7] > 80.0;
    
      // initialize motor speeds at 50% of MAX_SPEED.
      left_speed  = 0.5 * MAX_SPEED;
      right_speed = 0.5 * MAX_SPEED;
    
      //2) Avoid obstacles
      if (left_obstacle && !is_cible(messageReceived) ) { //if there is an obstacle on the left ad that's not the target
        // turn right
        left_speed  = 0.5 * MAX_SPEED;
        right_speed = -0.5 * MAX_SPEED;
      } else if (right_obstacle && !is_cible(messageReceived) ) { //same but on the right
        // turn left
        left_speed  = -0.5 * MAX_SPEED;
        right_speed = 0.5 * MAX_SPEED;
        
      }else{

        //3)Target detected, the robot goes towards the target
        if(is_cible(messageReceived)){
      
          if(y > 0.5){ //emitter to the left
            left_speed  = -0.5 * MAX_SPEED;
            right_speed = 0.5 * MAX_SPEED;  
          }else if(y < -0.5){ //emitter to the right
            left_speed  = 0.5 * MAX_SPEED;
            right_speed = -0.5 * MAX_SPEED;
          } 
        }
      }
    }

    //reception des messages de la cible
    while (wb_receiver_get_queue_length(receiver) > 0) {
      messageReceived = wb_receiver_get_data(receiver);
      dir = wb_receiver_get_emitter_direction(receiver);
      y = dir[1];
      //double signal = wb_receiver_get_signal_strength(receiver);
      //printf("received: %s (signal=%g, dir=[%g %g %g])\n",messageReceived, signal, dir[0], dir[1], dir[2]);
      wb_receiver_next_packet(receiver);
    }      
      
    /*--- End behavior ---*/
    
#ifdef RANDOM_MOVE
    /*
     * 5% to turn right, 5% to turn left
     */
    int r;
    r = rand()%100;
    if(r < 5 && !termine){
      left_speed  = -MAX_SPEED;
      right_speed = MAX_SPEED;
    }else if(r >= 5 && r < 10 && !termine){
      left_speed  = MAX_SPEED;
      right_speed = -MAX_SPEED;
    }
#endif 
 
    // write actuators inputs
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  };
  
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}