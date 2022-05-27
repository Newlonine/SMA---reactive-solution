/*
 * File:          supervisor_controller_V3.c
 * Date: 24/05
 * Description: Same as suprevisor_controller_V2.c but without the emitter.
 * Author: Nolwenn
 * Modifications: 24/05 : remove the emitter
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/emitter.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TIME_STEP 32
#define NB_EPUCK 10
#define ARENA_SIZE 1
#define RANGE_DETECTION 0.3

WbNodeRef robots[NB_EPUCK];
WbFieldRef trans_field[NB_EPUCK];
WbFieldRef rttn_field[NB_EPUCK];
WbFieldRef controller_field[NB_EPUCK];
WbFieldRef trans_field_cible;

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

/*
 * Initialize robots[] and trans_field
 * function based on static void reset(void); in advanced_particle_swarm_optimization_supervisor.c
 */
void set_robots(void)
{
  // Name of the robot we are linking.
  char rob[7] = "epuck0";
  int i = 0;
  for (; i < NB_EPUCK; i++)
  {
    // Indicate where robot location has to be stored.
    robots[i] = wb_supervisor_node_get_from_def(rob);
    if (robots[i] == NULL)
    {
      fprintf(stderr, "No DEF node found in the current world file\n");
      exit(1);
    }

    trans_field[i] = wb_supervisor_node_get_field(robots[i], "translation");
    rttn_field[i] = wb_supervisor_node_get_field(robots[i], "rotation");
    controller_field[i] = wb_supervisor_node_get_field(robots[i], "controller");

    // Update robot name.
    rob[5]++;
  }

  return;
}

/*
 * Places each robot randomly in the arena
 * function from advanced_particle_swarm_optimization_supervisor.c
 */
void random_position(void)
{
  double p[3] = {0, 0, 0};
  double r[4] = {0, 0, 1, 0};
  int n = 0;
  for (; n < NB_EPUCK; n++)
  {
    p[0] = random(-100, 100); // x coordinate
    p[1] = random(-100, 100); // y coordinate

    r[3] = random(-314, 314);

    wb_supervisor_field_set_sf_vec3f(trans_field[n], p);
    wb_supervisor_field_set_sf_rotation(rttn_field[n], r);
    wb_supervisor_field_set_sf_string(controller_field[n], "epuck_reactive_behavior_V4");
  }

  printf("positioning the target...\n");
  p[0] = random(-200, 200); // x coordinate
  p[1] = random(-150, 150); // y coordonate
  p[2] = 0.055; // z coordinate

  wb_supervisor_field_set_sf_vec3f(trans_field_cible, p);

  printf("Random positioning done.\n");
  return;
}

bool in_the_range(const double *values_robots, const double *values_cible)
{
  bool x_range = values_robots[0] < values_cible[0] + RANGE_DETECTION && values_robots[0] > values_cible[0] - RANGE_DETECTION;
  bool y_range = values_robots[1] < values_cible[1] + RANGE_DETECTION && values_robots[1] > values_cible[1] - RANGE_DETECTION;

  return x_range && y_range;
}

bool simulation_finished(bool robots_stopped[NB_EPUCK]){
  bool res = true;

  for(int i = 0; i < NB_EPUCK; i++)
    res = res && robots_stopped[i];

  return res;
}

int main()
{
  wb_robot_init();
  srand(time(NULL));

  // Target
  WbNodeRef target = wb_supervisor_node_get_from_def("target");
  trans_field_cible = wb_supervisor_node_get_field(target, "translation");

  if (target == NULL)
  {
    fprintf(stderr, "No DEF target node found in the current world file\n");
    exit(1);
  }

  // setup
  set_robots();
  random_position();

  const double *values_robots;
  const double *values_cible = wb_supervisor_field_get_sf_vec3f(trans_field_cible);

  bool finished = false;

  bool robots_stopped[NB_EPUCK];
  for (int i = 0; i < NB_EPUCK; i++)
    robots_stopped[i] = false;

  while (wb_robot_step(TIME_STEP) != -1)
  {
    for (int i = 0; i < NB_EPUCK; i++)
    {
      values_robots = wb_supervisor_field_get_sf_vec3f(trans_field[i]);

      if (in_the_range(values_robots, values_cible) && !robots_stopped[i])
      {
        printf("epuck%d is in the range\n", i);

        robots_stopped[i] = true;
        finished = simulation_finished(robots_stopped);
        printf("supervisor finished : %d\n", finished);
      } 
    }
    
    // all robots have found the target
    if(finished)
      wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
    
    values_cible = wb_supervisor_field_get_sf_vec3f(trans_field_cible);
  }

  wb_robot_cleanup();
  return 0;
}