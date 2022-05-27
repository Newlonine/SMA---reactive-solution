/*
 * File:          goal_emitter.c
 * Date: 
 * Description: Fist controller for the target, must be used with epuck_reactive_behavior.c
 *              If the target has been found (aka, if it receives a message from a robot), 
 *              it increses its range.
 * Author:
 * Modifications:
 */

#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <stdio.h>
#include <string.h>

#define TIME_STEP 16
#define MAX_RANGE 1.0

/*
 * retur true if the message received is "termine"
 */
bool is_termine(const char *message){
  return !strcmp(message,"termine");
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  const char *message_received = "";
  double range = 0.5;
  int est_termine = false;
  
  //recepteur
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, 10);
  
  //emetteur
  WbDeviceTag emitter = wb_robot_get_device("emitter");
  wb_emitter_set_range(emitter, range);
  char message[128];

  while (wb_robot_step(TIME_STEP) != -1) {

     //send a message
     sprintf(message, "cible");
     wb_emitter_send(emitter, message, strlen(message) + 1); 
     
     //receiving messages
     while (wb_receiver_get_queue_length(receiver) > 0) {
        message_received = wb_receiver_get_data(receiver);
        //printf("received: %s\n",message_received);
        if(is_termine(message_received))
          est_termine = true;    
    
        wb_receiver_next_packet(receiver);
      }
      
     //if it has been found
     if(est_termine){
       printf("Trouvée\n");
       est_termine = false;
       
       //we increase the range
       if(range < MAX_RANGE){
         range+=0.1;
         wb_emitter_set_range(emitter, range);
         printf("range : %g\n", range);
       }
     }
  };

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
