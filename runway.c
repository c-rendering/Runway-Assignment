/* Copyright (c) 2025 Trevor Bakker
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILTY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/license/>.
*/

#define _GNU_SOURCE

#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <time.h>

/*** Constants that define parameters of the simulation ***/

#define MAX_RUNWAY_CAPACITY 2    /* Number of aircraft that can use runway simultaneously */
#define CONTROLLER_LIMIT 8       /* Number of aircraft the controller can manage before break */
#define MAX_AIRCRAFT 1000        /* Maximum number of aircraft in the simulation */
#define FUEL_MIN 20              /* Minimum fuel reserve in seconds */
#define FUEL_MAX 60              /* Maximum fuel reserve in seconds */
#define EMERGENCY_TIMEOUT 30     /* Max wait time for emergency aircraft in seconds */
#define DIRECTION_SWITCH_TIME 5  /* Time required to switch runway direction */
#define DIRECTION_LIMIT 3        /* Max consecutive aircraft in same direction */

#define COMMERCIAL 0
#define CARGO 1
#define EMERGENCY 2

#define NORTH 0
#define SOUTH 1
#define EAST  2
#define WEST  4

/* TODO */
/* Add your synchronization variables here */

/* basic information about simulation.  they are printed/checked at the end
 * and in assert statements during execution.
 *
 * you are responsible for maintaining the integrity of these variables in the
 * code that you develop.
 */

/* global mutex protecting shared state */
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;

/* condition variables for admission decisions and controller actions */
static pthread_cond_t cv_commercial = PTHREAD_COND_INITIALIZER; /* commercial wakeup */
static pthread_cond_t cv_cargo      = PTHREAD_COND_INITIALIZER; /* cargo wakeup */
static pthread_cond_t cv_emerg      = PTHREAD_COND_INITIALIZER; /* emergency wakeup */
static pthread_cond_t cv_fuel_emerg = PTHREAD_COND_INITIALIZER; /* fuel emergency wakeup */
static pthread_cond_t cv_controller = PTHREAD_COND_INITIALIZER; /* controller wakeup */
static pthread_cond_t cv_any        = PTHREAD_COND_INITIALIZER; /* global wakeup */

/* waiting counts (by priority and type) */
static int waiting_commercial      = 0;
static int waiting_cargo           = 0;
static int waiting_emergency       = 0;
static int waiting_fuel_emerg      = 0;      /* any type that has exceeded its fuel */
static int waiting_fuel_commercial = 0;      /* commercial that promoted to fuel emergency */
static int waiting_fuel_cargo      = 0;      /* cargo that promoted to fuel emergency */

/* fairness tracking (type-level and direction-level) */
static int last_type = -1;                 /* COMMERCIAL/CARGO/EMERGENCY */
static int consecutive_type_count = 0;     /* number of consecutive commercial/cargo aircraft */

/* controller break flag */
static int controller_on_break = 0;

static int aircraft_on_runway = 0;       /* Total number of aircraft currently on runway */
static int commercial_on_runway = 0;     /* Total number of commercial aircraft on runway */
static int cargo_on_runway = 0;          /* Total number of cargo aircraft on runway */
static int emergency_on_runway = 0;      /* Total number of emergency aircraft on runway */
static int aircraft_since_break = 0;     /* Aircraft processed since last controller break */
static int current_direction = NORTH;    /* Current runway direction (NORTH or SOUTH) */
static int consecutive_direction = 0;    /* Consecutive aircraft in current direction */

typedef struct
{
  int arrival_time;         // time between the arrival of this aircraft and the previous aircraft
  int runway_time;          // time the aircraft needs to spend on the runway
  int aircraft_id;
  int aircraft_type;        // COMMERCIAL, CARGO, or EMERGENCY
  int fuel_reserve;         // Randomly assigned fuel reserve (FUEL_MIN to FUEL_MAX seconds)
  time_t arrival_timestamp; // timestamp when aircraft thread was created
} aircraft_info;

static void wake_all(void)
{
  pthread_cond_broadcast(&cv_any);
  pthread_cond_broadcast(&cv_commercial);
  pthread_cond_broadcast(&cv_cargo);
  pthread_cond_broadcast(&cv_emerg);
  pthread_cond_broadcast(&cv_fuel_emerg);
  pthread_cond_broadcast(&cv_controller);
}

/* Called at beginning of simulation.
 * TODO: Create/initialize all synchronization
 * variables and other global variables that you add.
 */
static int initialize(aircraft_info *ai, char *filename)
{
  aircraft_on_runway    = 0;
  commercial_on_runway  = 0;
  cargo_on_runway       = 0;
  emergency_on_runway   = 0;
  aircraft_since_break  = 0;
  current_direction     = NORTH;
  consecutive_direction = 0;

  waiting_commercial = waiting_cargo = waiting_emergency = waiting_fuel_emerg = 0;
  last_type = -1;
  consecutive_type_count = 0;
  controller_on_break = 0;

  /* seed random number generator for fuel reserves */
  srand(time(NULL));

  /* Read in the data file and initialize the aircraft array */
  FILE *fp;

  if((fp=fopen(filename, "r")) == NULL)
  {
    printf("Cannot open input file %s for reading.\n", filename);
    exit(1);
  }

  int i = 0;
  char line[256];
  while (fgets(line, sizeof(line), fp) && i < MAX_AIRCRAFT)
  {
    /* Skip comment lines and empty lines */
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') 
    {
      continue;
    }

    if (sscanf(line, "%d%d%d", &(ai[i].aircraft_type), &(ai[i].arrival_time),
               &(ai[i].runway_time)) == 3) 
               {
                  /* Assign random fuel reserve between FUEL_MIN and FUEL_MAX */
                  ai[i].fuel_reserve = FUEL_MIN + (rand() % (FUEL_MAX - FUEL_MIN + 1));
                  i = i + 1;
    }
  }

  fclose(fp);
  return i;
}

/* Code executed by controller to simulate taking a break
 * You do not need to add anything here.
 */
__attribute__((unused)) static void take_break()
{
  printf("The air traffic controller is taking a break now.\n");
  sleep(5);
  assert( aircraft_on_runway == 0 );
  aircraft_since_break = 0;
}

/* Code executed to switch runway direction
 * You do not need to add anything here.
 */
__attribute__((unused)) static void switch_direction()
{
  printf("Switching runway direction from %s to %s\n",
         current_direction == NORTH ? "NORTH" : "SOUTH",
         current_direction == NORTH ? "SOUTH" : "NORTH");

  assert( aircraft_on_runway == 0 );  // Runway must be empty to switch

  sleep(DIRECTION_SWITCH_TIME);

  current_direction = (current_direction == NORTH) ? SOUTH : NORTH;
  consecutive_direction = 0;

  printf("Runway direction switched to %s\n",
         current_direction == NORTH ? "NORTH" : "SOUTH");
}

/* helper functions for switching runway direction */
static int opposite_waiters_present(void) 
{
  return (current_direction == NORTH) ? (waiting_cargo > 0)
                                      : (waiting_commercial > 0);
}

static int same_dir_waiters_present(void) 
{
  return (current_direction == NORTH) ? (waiting_commercial > 0)
                                      : (waiting_cargo > 0);
}

static int fuel_emerg_opposite_waiters_present(void) 
{
  return (current_direction == NORTH) ? (waiting_fuel_cargo > 0)
                                      : (waiting_fuel_commercial  > 0);
}

static int fuel_emerg_same_dir_waiters_present(void) {
  return (current_direction == NORTH) ? (waiting_fuel_commercial  > 0)
                                      : (waiting_fuel_cargo > 0);
}

/* Code for the air traffic controller thread. This is fully implemented except for
 * synchronization with the aircraft. See the comments within the function for details.
 */
void *controller_thread(void *arg)
{
 (void)arg;

  printf("The air traffic controller arrived and is beginning operations\n");

  pthread_mutex_lock(&mtx);
  /* Loop while waiting for aircraft to arrive. */
  while (1)
  {
    /* TODO */
    /* Add code here to handle aircraft requests, controller breaks,      */
    /* and runway direction switches.                                     */
    /* Currently the body of the loop is empty.  There's no communication */
    /* between controller and aircraft, i.e. all aircraft are admitted    */
    /* without regard for runway capacity, aircraft type, direction,      */
    /* priorities, and whether the controller needs a break.              */
    /* You need to add all of this.                                       */

    /* Take a break as soon as limit reached and runway is empty */
    if ((aircraft_since_break >= CONTROLLER_LIMIT) && aircraft_on_runway == 0)
    {
      controller_on_break = 1;
      pthread_mutex_unlock(&mtx);
      take_break();
      pthread_mutex_lock(&mtx);
      controller_on_break = 0;
      wake_all();
    }
    /* fuel emergency coming from opposite direction */
    else if (aircraft_on_runway == 0 && fuel_emerg_opposite_waiters_present() && !fuel_emerg_same_dir_waiters_present()) 
    {
      switch_direction();
      wake_all();
    }
    /* switch direction when runway is empty and the same direction limit has been reached or
    there are aircraft needing to come from the opposite direction */
    else if (aircraft_on_runway == 0 && (
      (consecutive_direction >= DIRECTION_LIMIT && opposite_waiters_present())
      ||
      (!same_dir_waiters_present() && opposite_waiters_present())
    ))
    {
      switch_direction();
      wake_all();
    }
    else
    {
      /* Sleep until someone arrives/leaves/changes state */
      struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      ts.tv_sec += 1; /* periodic wake to re-check */
      pthread_cond_timedwait(&cv_controller, &mtx, &ts);
    }

    /* Allow thread to be cancelled */
    pthread_testcancel();
  }
  /* not reached */
  pthread_mutex_unlock(&mtx);
  pthread_exit(NULL);
}

/* Code executed by a commercial aircraft to enter the runway.
 * You have to implement this.  Do not delete the assert() statements,
 * but feel free to add your own.
 */
void commercial_enter(aircraft_info *ai)
{
  /* TODO */
  /* Request permission to use the runway. You might also want to add      */
  /* synchronization for the simulation variables below.                   */
  /* Consider: runway capacity, direction (commercial prefer NORTH),       */
  /* controller breaks, fuel levels, emergency priorities, and fairness.   */

  int became_fuel_emerg = 0;

  pthread_mutex_lock(&mtx);
  waiting_commercial++;
  wake_all();

  while (1)
  {
    time_t now = time(NULL);
    int waited = (int)difftime(now, ai->arrival_timestamp);

    /* fuel reserve promotion */
    if (!became_fuel_emerg && waited >= ai->fuel_reserve) 
    {
      waiting_fuel_emerg++;
      waiting_fuel_commercial++;
      became_fuel_emerg = 1;
      wake_all();
    }

    /* wait for controller if on break or at limit of aircraft before break */
    if (controller_on_break || (aircraft_since_break >= CONTROLLER_LIMIT)) 
    {
      pthread_cond_wait(&cv_any, &mtx);
      continue;
    }

    /* priority gates */
    int we_are_top = 1;
    /* fuel emerg outrank us */
    if (!became_fuel_emerg && waiting_fuel_emerg > 0) 
    {
      we_are_top = 0;
    }
    /* emergencies outrank us */
    if (waiting_emergency > 0 && waiting_fuel_emerg == 0) 
    {
      we_are_top = 0;
    }
    /* too many flights of same type */
    if (!became_fuel_emerg && last_type == COMMERCIAL && consecutive_type_count >= 4 && waiting_cargo > 0) 
    {
      we_are_top = 0;
    }


    /* runway constraints */
    /* runway can't be at capacity, can't be any cargo on runway, direction NORTH for commercial */
    int can_enter = (aircraft_on_runway < MAX_RUNWAY_CAPACITY) &&
                    (cargo_on_runway) == 0 &&
                    (current_direction == NORTH);

    if (we_are_top && can_enter) 
    {
      /* move aircraft to runway */
      aircraft_on_runway++;
      commercial_on_runway++;
      aircraft_since_break++;
      consecutive_direction++;
      waiting_commercial--;

      /* fairness streak */
      if (last_type == COMMERCIAL) 
      {
        consecutive_type_count++;
      }
      else 
      {
        last_type = COMMERCIAL;
        consecutive_type_count = 1;
      }

      if (became_fuel_emerg) 
      {
        waiting_fuel_emerg--;
        waiting_fuel_commercial--;
      }

      wake_all();
      pthread_mutex_unlock(&mtx);
      return;
    }

    if (became_fuel_emerg) 
    {
      pthread_cond_wait(&cv_fuel_emerg, &mtx);
    }
    else 
    {
      pthread_cond_wait(&cv_commercial, &mtx);
    }
  }
}

/* Code executed by a cargo aircraft to enter the runway.
 * You have to implement this.  Do not delete the assert() statements,
 * but feel free to add your own.
 */
void cargo_enter(aircraft_info *ai)
{
  /* TODO */
  /* Request permission to use the runway. You might also want to add      */
  /* synchronization for the simulation variables below.                   */
  /* Consider: runway capacity, direction (cargo prefer SOUTH),            */
  /* controller breaks, fuel levels, emergency priorities, and fairness.   */
  int became_fuel_emerg = 0;

  pthread_mutex_lock(&mtx);
  waiting_cargo++;
  wake_all();

  while (1)
  {
    time_t now = time(NULL);
    int waited = (int)difftime(now, ai->arrival_timestamp);

    /* fuel reserve promotion */
    if (!became_fuel_emerg && waited >= ai->fuel_reserve) 
    {
      waiting_fuel_emerg++;
      waiting_fuel_cargo++;
      became_fuel_emerg = 1;
      wake_all();
    }

    /* wait for controller if on break or at limit of aircraft before break */
    if (controller_on_break || (aircraft_since_break >= CONTROLLER_LIMIT)) 
    {
      pthread_cond_wait(&cv_any, &mtx);
      continue;
    }

    /* priority gates */
    int we_are_top = 1;
    /* fuel emerg outrank us */
    if (!became_fuel_emerg && waiting_fuel_emerg > 0) 
    {
      we_are_top = 0;
    }
    /* emergencies outrank us */
    if (waiting_emergency > 0 && waiting_fuel_emerg == 0) 
    {
      we_are_top = 0;
    }
    /* too many flights of same type */
    if (!became_fuel_emerg && last_type == CARGO && 
      consecutive_type_count >= 4 && waiting_commercial > 0) 
    {
      we_are_top = 0;
    }

    /* runway constraints */
    /* runway can't be at capacity, can't be any commercial on runway, direction SOUTH for cargo */
    int can_enter = (aircraft_on_runway < MAX_RUNWAY_CAPACITY) &&
                    (commercial_on_runway) == 0 &&
                    (current_direction == SOUTH);

    if (we_are_top && can_enter) 
    {
      /* move aircraft to runway */
      aircraft_on_runway++;
      cargo_on_runway++;
      aircraft_since_break++;
      consecutive_direction++;
      waiting_cargo--;

      /* fairness streak */
      if (last_type == CARGO) 
      {
        consecutive_type_count++;
      }
      else 
      {
        last_type = CARGO;
        consecutive_type_count = 1;
      }


      if (became_fuel_emerg) 
      {
        waiting_fuel_emerg--;
        waiting_fuel_cargo--;
      }

      wake_all();
      pthread_mutex_unlock(&mtx);
      return;
    }

    if (became_fuel_emerg) 
    {
      pthread_cond_wait(&cv_fuel_emerg, &mtx);
    }
    else 
    {
      pthread_cond_wait(&cv_cargo, &mtx);
    }
  }
}

/* Code executed by an emergency aircraft to enter the runway.
 * You have to implement this.  Do not delete the assert() statements,
 * but feel free to add your own.
 */
void emergency_enter(aircraft_info *ai)
{
  /* TODO */
  /* Request permission to use the runway. You might also want to add      */
  /* synchronization for the simulation variables below.                   */
  /* Emergency aircraft have priority and must be admitted within 30s,     */
  /* but still respect runway capacity and controller breaks.              */
  /* Emergency aircraft can use either direction.                          */
  int became_fuel_emerg = 0;

  pthread_mutex_lock(&mtx);
  waiting_emergency++;
  wake_all();

  while (1)
  {
    time_t now = time(NULL);
    int waited = (int)difftime(now, ai->arrival_timestamp);

    /* For normal emergencies: promote if waited >= 30s OR fuel_reserve, whichever first */
    if (!became_fuel_emerg && (waited >= EMERGENCY_TIMEOUT || waited >= ai->fuel_reserve)) 
    {
      waiting_fuel_emerg++;
      became_fuel_emerg = 1;
      wake_all();
    }

    if (controller_on_break || (aircraft_since_break >= CONTROLLER_LIMIT)) 
    {
      /* use a timed wait to allow us to check for the EMERGENCY_TIMEOUT */
      struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts); ts.tv_sec += 1;
      pthread_cond_timedwait(&cv_any, &mtx, &ts);
      continue;
    }

    int we_are_top = 1;
    /* the only thing that can pre-empt emergency is fuel emergency */
    if (!became_fuel_emerg && waiting_fuel_emerg > 0) 
    {
      we_are_top = 0;
    }

    /* runway constraints */
    /* runway can't be at capacity, it will pre-empt commercial or cargo and can go in any direction */
    int can_enter = (aircraft_on_runway < MAX_RUNWAY_CAPACITY);

    if (we_are_top && can_enter) 
    {
      aircraft_on_runway++;
      emergency_on_runway++;
      aircraft_since_break++;
      /* emergencies do not change fairness streak */
      consecutive_direction++;
      waiting_emergency--;
      if (became_fuel_emerg) 
      {
        waiting_fuel_emerg--;
      }

      wake_all();
      pthread_mutex_unlock(&mtx);
      return;
    }

    /* use a timed wait to allow us to check for the EMERGENCY_TIMEOUT */
    struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts); ts.tv_sec += 1;
    if (became_fuel_emerg) 
    {
      pthread_cond_timedwait(&cv_fuel_emerg, &mtx, &ts);
    }
    else 
    {
      pthread_cond_timedwait(&cv_emerg, &mtx, &ts);
    }
  }
}

/* Code executed by an aircraft to simulate the time spent on the runway
 * You do not need to add anything here.
 */
static void use_runway(int t)
{
  sleep(t);
}


/* Code executed by a commercial aircraft when leaving the runway.
 * You need to implement this.  Do not delete the assert() statements,
 * but feel free to add as many of your own as you like.
 */
static void commercial_leave()
{
  /*
   *  TODO
   *  YOUR CODE HERE.
   */

  pthread_mutex_lock(&mtx);
  aircraft_on_runway--;
  commercial_on_runway--;

  if (aircraft_on_runway == 0) 
  {
    pthread_cond_broadcast(&cv_controller);
  }

  wake_all();
  pthread_mutex_unlock(&mtx);
}

/* Code executed by a cargo aircraft when leaving the runway.
 * You need to implement this.  Do not delete the assert() statements,
 * but feel free to add as many of your own as you like.
 */
static void cargo_leave()
{
  /*
   * TODO
   * YOUR CODE HERE.
   */

   pthread_mutex_lock(&mtx);
   aircraft_on_runway--;
   cargo_on_runway--;

   if (aircraft_on_runway == 0) 
   {
    pthread_cond_broadcast(&cv_controller);
   }

   wake_all();
   pthread_mutex_unlock(&mtx);
}

/* Code executed by an emergency aircraft when leaving the runway.
 * You need to implement this.  Do not delete the assert() statements,
 * but feel free to add as many of your own as you like.
 */
static void emergency_leave()
{
  /*
   * TODO
   * YOUR CODE HERE.
   */
  pthread_mutex_lock(&mtx);
  aircraft_on_runway--;
  emergency_on_runway--;

  if (aircraft_on_runway == 0) pthread_cond_broadcast(&cv_controller);

  wake_all();
  pthread_mutex_unlock(&mtx);
}

/* Main code for commercial aircraft threads.
 * You do not need to change anything here, but you can add
 * debug statements to help you during development/debugging.
 */
void* commercial_aircraft(void *ai_ptr)
{
  aircraft_info *ai = (aircraft_info*)ai_ptr;

  /* Record arrival time for fuel tracking */
  ai->arrival_timestamp = time(NULL);

  /* Request runway access */
  commercial_enter(ai);

  printf("Commercial aircraft %d (fuel: %ds) is now on the runway (direction: %s)\n",
         ai->aircraft_id, ai->fuel_reserve,
         current_direction == NORTH ? "NORTH" : "SOUTH");

  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway == 0 ); // Commercial and cargo cannot mix

  /* Use runway  --- do not make changes to the 3 lines below*/
  printf("Commercial aircraft %d begins runway operations for %d seconds\n",
         ai->aircraft_id, ai->runway_time);
  use_runway(ai->runway_time);
  printf("Commercial aircraft %d completes runway operations and prepares to depart\n",
         ai->aircraft_id);

  /* Leave runway */
  commercial_leave();

  printf("Commercial aircraft %d has cleared the runway\n", ai->aircraft_id);

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) 
  {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", aircraft_on_runway, MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n",
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  pthread_exit(NULL);
}

/* Main code for cargo aircraft threads.
 * You do not need to change anything here, but you can add
 * debug statements to help you during development/debugging.
 */
void* cargo_aircraft(void *ai_ptr)
{
  aircraft_info *ai = (aircraft_info*)ai_ptr;

  /* Record arrival time for fuel tracking */
  ai->arrival_timestamp = time(NULL);

  /* Request runway access */
  cargo_enter(ai);

  printf("Cargo aircraft %d (fuel: %ds) is now on the runway (direction: %s)\n",
         ai->aircraft_id, ai->fuel_reserve,
         current_direction == NORTH ? "NORTH" : "SOUTH");

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) 
  {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", aircraft_on_runway,
            MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n",
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(commercial_on_runway == 0 );

  printf("Cargo aircraft %d begins runway operations for %d seconds\n",
         ai->aircraft_id, ai->runway_time);
  use_runway(ai->runway_time);
  printf("Cargo aircraft %d completes runway operations and prepares to depart\n",
         ai->aircraft_id);

  /* Leave runway */
  cargo_leave();

  printf("Cargo aircraft %d has cleared the runway\n", ai->aircraft_id);

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) 
  {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n",
           aircraft_on_runway, MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n",
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  pthread_exit(NULL);
}

/* Main code for emergency aircraft threads.
 * You do not need to change anything here, but you can add
 * debug statements to help you during development/debugging.
 */
void* emergency_aircraft(void *ai_ptr)
{
  aircraft_info *ai = (aircraft_info*)ai_ptr;

  /* Record arrival time for fuel and emergency timeout tracking */
  ai->arrival_timestamp = time(NULL);

  /* Request runway access */
  emergency_enter(ai);

  printf("EMERGENCY aircraft %d (fuel: %ds) is now on the runway (direction: %s)\n",
         ai->aircraft_id, ai->fuel_reserve,
         current_direction == NORTH ? "NORTH" : "SOUTH");

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) 
  {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n", aircraft_on_runway,
            MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n",
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  printf("EMERGENCY aircraft %d begins runway operations for %d seconds\n",
         ai->aircraft_id, ai->runway_time);
  use_runway(ai->runway_time);
  printf("EMERGENCY aircraft %d completes runway operations and prepares to depart\n",
         ai->aircraft_id);

  /* Leave runway */
  emergency_leave();

  printf("EMERGENCY aircraft %d has cleared the runway\n", ai->aircraft_id);

  if (!(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0)) 
  {
    printf("ASSERT FAILURE: aircraft_on_runway=%d (should be 0-%d)\n",
           aircraft_on_runway, MAX_RUNWAY_CAPACITY);
    printf("Runway state: commercial=%d, cargo=%d, emergency=%d, direction=%s\n",
           commercial_on_runway, cargo_on_runway, emergency_on_runway,
           current_direction == NORTH ? "NORTH" : "SOUTH");
  }
  assert(aircraft_on_runway <= MAX_RUNWAY_CAPACITY && aircraft_on_runway >= 0);
  assert(commercial_on_runway >= 0 && commercial_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(cargo_on_runway >= 0 && cargo_on_runway <= MAX_RUNWAY_CAPACITY);
  assert(emergency_on_runway >= 0 && emergency_on_runway <= MAX_RUNWAY_CAPACITY);

  pthread_exit(NULL);
}

/* Main function sets up simulation and prints report
 * at the end.
 * GUID: 355F4066-DA3E-4F74-9656-EF8097FBC985
 */
int main(int nargs, char **args)
{
  int i;
  int result;
  int num_aircraft;
  void *status;
  pthread_t controller_tid;
  pthread_t aircraft_tid[MAX_AIRCRAFT];
  aircraft_info ai[MAX_AIRCRAFT];

  if (nargs != 2)
  {
    printf("Usage: runway <name of inputfile>\n");
    return EINVAL;
  }

  num_aircraft = initialize(ai, args[1]);
  if (num_aircraft > MAX_AIRCRAFT || num_aircraft <= 0)
  {
    printf("Error:  Bad number of aircraft threads. "
           "Maybe there was a problem with your input file?\n");
    return 1;
  }

  printf("Starting runway simulation with %d aircraft ...\n", num_aircraft);

  result = pthread_create(&controller_tid, NULL, controller_thread, NULL);

  if (result)
  {
    printf("runway:  pthread_create failed for controller: %s\n", strerror(result));
    exit(1);
  }

  for (i=0; i < num_aircraft; i++)
  {
    ai[i].aircraft_id = i;
    sleep(ai[i].arrival_time);

    if (ai[i].aircraft_type == COMMERCIAL)
    {
      result = pthread_create(&aircraft_tid[i], NULL, commercial_aircraft,
                             (void *)&ai[i]);
    }
    else if (ai[i].aircraft_type == CARGO)
    {
      result = pthread_create(&aircraft_tid[i], NULL, cargo_aircraft,
                             (void *)&ai[i]);
    }
    else
    {
      result = pthread_create(&aircraft_tid[i], NULL, emergency_aircraft,
                             (void *)&ai[i]);
    }

    if (result)
    {
      printf("runway: pthread_create failed for aircraft %d: %s\n",
            i, strerror(result));
      exit(1);
    }
  }

  /* wait for all aircraft threads to finish */
  for (i = 0; i < num_aircraft; i++)
  {
    pthread_join(aircraft_tid[i], &status);
  }

  /* tell the controller to finish. */
  pthread_cancel(controller_tid);
  pthread_join(controller_tid, &status);

  printf("Runway simulation done.\n");

  return 0;
}

