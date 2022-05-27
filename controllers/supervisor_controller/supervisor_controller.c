#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define TIME_STEP 32
#define NB_EPUCK 5
#define WB_CHANNEL_SUPERVISOR 2

double random(int min, int max){
  int delta;
  double res;
  
  delta = max - min + 1;
  res = rand()%delta;
  res = (res + min)/100;
  
  return res;
}

bool finished(int robots[NB_EPUCK]){
    int i;
    bool res = 1;

    for(i = 0; i<NB_EPUCK; i++)
        res = res && robots[i];

    return res;
}

int main(int argc, char **argv) {
  wb_robot_init();
  srand(time(NULL));
  
  int step = 0, i;
  double y, x, angle;
  char cible[128], robot[128];
  int robots[NB_EPUCK]; //used to know which robot has found the target
  const char *messageReceived;

  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef children_field = wb_supervisor_node_get_field(root_node, "children");
  
  for(i=0; i<NB_EPUCK; i++){
    x = random(-100, 100); //x position
    y = random(-100, 100); //y position
    angle = random(-314, 314); //angle
  
    sprintf(robot, "E-puckCameraRecognition { translation %g %g 0 rotation 0 0 1 %g controller \"epuck_reactive_behavior_V2\"}", x, y, angle);
    wb_supervisor_field_import_mf_node_from_string(children_field, -1, robot);
    robots[i] = 0;
  }
  
  //on spawn la cible
  x = random(-250, 250);
  y = random(-100, 100);
  
  sprintf(cible, "Cible { translation %g %g 0.04 controller \"goal_emitter\"}", x, y);
  wb_supervisor_field_import_mf_node_from_string(children_field, -1, cible);

/*  
  //Receiver
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, 10);
  wb_receiver_set_channel(receiver,WB_CHANNEL_SUPERVISOR);
*/
  while (wb_robot_step(TIME_STEP) != -1) {
    step++;

/*
    while (wb_receiver_get_queue_length(receiver) > 0) {
      messageReceived = wb_receiver_get_data(receiver);
      robots[atoi(messageReceived)] = 1;
      wb_receiver_next_packet(receiver);
    } 
    
    // freeze the whole simulation, all robots have discovered the target
    if (finished(robots)) {
      wb_robot_cleanup();
      exit(0);
    } 
*/
  }

  wb_robot_cleanup();

  return 0;
}