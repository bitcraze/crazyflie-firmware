/*
 * wall_follower_multi_ranger_onboard.c
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */

#include "wallfollowing_multiranger_onboard.h"
#include <math.h>
#include "usec_time.h"

// variables
static float ref_distance_from_wall = 0.0f;
static float max_speed = 0.2f;
static float max_rate = 0.5f;
static float direction = 1.0f;
static float first_run = false;
static int state = 1;
float state_start_time;



void wall_follower_init(float new_ref_distance_from_wall, float max_speed_ref, int init_state)
{
  ref_distance_from_wall = new_ref_distance_from_wall;
  max_speed = max_speed_ref;
  first_run = true;
  state = init_state;

}

// Static helper functions
static bool logicIsCloseTo(float real_value, float checked_value, float margin)
{
  if (real_value > checked_value - margin && real_value < checked_value + margin) {
    return true;
  } else {
    return false;
  }
}

static float wraptopi(float number)
{

  if (number > (float)M_PI) {
    return (number - (float)(2 * M_PI));
  } else if (number < (float)(-1 * M_PI)) {
    return (number + (float)(2 * M_PI));
  } else {
    return (number);
  }

}


// Static command functions
static void commandTurn(float *vel_x, float *vel_w, float ref_rate)
{
  *vel_x = 0.0f;
  *vel_w = direction * ref_rate;

}

static void commandAlignCorner(float *vel_y, float *vel_w, float ref_rate, float range,
                               float wanted_distance_from_corner)
{

  if (range > wanted_distance_from_corner + 0.3f) {
    *vel_w = direction * ref_rate;
    *vel_y = 0;

  } else {
    if (range > wanted_distance_from_corner) {
      *vel_y = direction * (-1.0f * max_speed / 3.0f);
    } else {
      *vel_y = direction * (max_speed / 3.9f);
    }
    *vel_w = 0;
  }


}

static void commandHover(float *vel_x, float *vel_y, float *vel_w)
{
  *vel_x = 0.0f;
  *vel_y = 0.0f;
  *vel_w = 0.0f;
}

static void commandForwardAlongWall(float *vel_x, float *vel_y, float range)
{
  *vel_x = max_speed;
  bool check_distance_wall = logicIsCloseTo(ref_distance_from_wall, range, 0.1f);
  *vel_y = 0;
  if (!check_distance_wall) {
    if (range > ref_distance_from_wall) {
      *vel_y = direction * (-1.0f * max_speed / 2.0f);
    } else {
      *vel_y = direction * (max_speed / 2.0f);
    }
  }
}

static void commandTurnAroundCornerAndAdjust(float *vel_x, float *vel_y, float *vel_w, float radius, float range)
{
  *vel_x = max_speed;
  *vel_w = direction * (-1 * (*vel_x) / radius);
  bool check_distance_to_wall = logicIsCloseTo(ref_distance_from_wall, range, 0.1f);
  if (!check_distance_to_wall) {
    if (range > ref_distance_from_wall) {
      *vel_y = direction * (-1.0f * max_speed / 3.0f);

    }

    else {
      *vel_y = direction * (max_speed / 3.0f);

    }

  }
}

static void commandTurnAndAdjust(float *vel_y, float *vel_w, float rate, float range)
{
  *vel_w = direction * rate;
  *vel_y = 0.0f;

}

static int transition(int new_state)
{
  float t =  usecTimestamp() / 1e6;
  state_start_time = t;
  return new_state;
}

void adjustDistanceWall(float distance_wall_new)
{
  ref_distance_from_wall = distance_wall_new;
}

int wall_follower(float *vel_x, float *vel_y, float *vel_w, float front_range, float side_range, float current_heading,
                  int direction_turn)
{


  direction = direction_turn;
  static float previous_heading = 0.0f;
  static float angle = 0.0f;
  static bool around_corner_go_back = false;
  float now = usecTimestamp() / 1e6;

  if (first_run) {
    previous_heading = current_heading;
    //  around_corner_first_turn = false;
    around_corner_go_back = false;
    first_run = false;
  }


  /***********************************************************
   * State definitions
   ***********************************************************/
  // 1 = forward
  // 2 = hover
  // 3 = turn_to_find_wall
  // 4 = turn_to_allign_to_wall
  // 5 = forward along wall
  // 6 = rotate_around_wall
  // 7 = rotate_in_corner
  // 8 = find corner

  /***********************************************************
  * Handle state transitions
  ***********************************************************/

  if (state == 1) {     //FORWARD
    if (front_range < ref_distance_from_wall + 0.2f) {
      state = transition(3);
    }
  } else if (state == 2) {  // HOVER

  } else if (state == 3) { // TURN_TO_FIND_WALL
    // check if wall is found
    bool side_range_check = side_range < ref_distance_from_wall / (float)cos(0.78f) + 0.2f;
    bool front_range_check = front_range < ref_distance_from_wall / (float)cos(0.78f) + 0.2f;
    if (side_range_check && front_range_check) {
      previous_heading = current_heading;
      angle = direction * (1.57f - (float)atan(front_range / side_range) + 0.1f);
      state = transition(4); // go to turn_to_allign_to_wall
    }
    if (side_range < 1.0f && front_range > 2.0f) {
      //  around_corner_first_turn = true;
      around_corner_go_back = false;
      previous_heading = current_heading;
      state = transition(8); // go to rotate_around_wall
    }
  } else if (state == 4) { //TURN_TO_ALLIGN_TO_WALL
    bool allign_wall_check = logicIsCloseTo(wraptopi(current_heading - previous_heading), angle, 0.1f);
    if (allign_wall_check) {
      // prev_side_range = side_range;
      state = transition(5);
    }
  } else if (state == 5) {  //FORWARD_ALONG_WALL

    // If side range is out of reach,
    //    end of the wall is reached
    if (side_range > ref_distance_from_wall + 0.3f) {
      //  around_corner_first_turn = true;
      state = transition(8);
    }
    // If front range is small
    //    then corner is reached
    if (front_range < ref_distance_from_wall + 0.2f) {
      previous_heading = current_heading;
      state = transition(7);
    }

  } else if (state == 6) {  //ROTATE_AROUND_WALL
    if (front_range < ref_distance_from_wall + 0.2f) {
      state = transition(3);
    }


  } else if (state == 7) {   //ROTATE_IN_CORNER
    // Check if heading goes over 0.8 rad
    bool check_heading_corner = logicIsCloseTo(fabs(wraptopi(current_heading - previous_heading)), 0.8f, 0.1f);
    if (check_heading_corner) {
      state = transition(3);
    }

  } else if (state == 8) {   //FIND_CORNER
    if (side_range <= ref_distance_from_wall) {
      state = transition(6);
    }

  }

  else {
  }


  /***********************************************************
   * Handle state actions
   ***********************************************************/

  float temp_vel_x = 0.0f;
  float temp_vel_y = 0.0f;
  float temp_vel_w = 0.0f;

  if (state == 1) {      //FORWARD
    temp_vel_x = max_speed;
    temp_vel_y = 0.0f;
    temp_vel_w = 0.0f;

  } else if (state == 2) {  // HOVER
    commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);


  } else if (state == 3) { // TURN_TO_FIND_WALL
    commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
    temp_vel_y = 0.0f;

  } else if (state == 4) { //TURN_TO_ALLIGN_TO_WALL



    if (now - state_start_time < 1.0f)
    {
      commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
    } else { // then turn again
      commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
      temp_vel_y = 0.0f;
    }

  } else if (state == 5) {  //FORWARD_ALONG_WALL
    commandForwardAlongWall(&temp_vel_x, &temp_vel_y, side_range);
    temp_vel_w = 0.0f;

    //commandForwardAlongWallHeadingSine(&temp_vel_x, &temp_vel_y,&temp_vel_w, side_range);

  } else if (state == 6) {  //ROTATE_AROUND_WALL
    // If first time around corner
    //first try to find the corner again

    // if side range is larger than prefered distance from wall
    if (side_range > ref_distance_from_wall + 0.5f) {

      // check if scanning has already occured
      if (wraptopi(fabs(current_heading - previous_heading)) > 0.8f) {
        around_corner_go_back = true;
      }
      // turn and adjust distnace to corner from that point
      if (around_corner_go_back) {
        // go back if it already went into one direction
        commandTurnAndAdjust(&temp_vel_y, &temp_vel_w, -1 * max_rate, side_range);
        temp_vel_x = 0.0f;
      } else {
        commandTurnAndAdjust(&temp_vel_y, &temp_vel_w, max_rate, side_range);
        temp_vel_x = 0.0f;
      }
    } else {
      // continue to turn around corner
      previous_heading = current_heading;
      around_corner_go_back = false;
      commandTurnAroundCornerAndAdjust(&temp_vel_x, &temp_vel_y, &temp_vel_w, ref_distance_from_wall, side_range);
    }

  } else if (state == 7) {     //ROTATE_IN_CORNER
    commandTurn(&temp_vel_x, &temp_vel_w, max_rate);
    temp_vel_y = 0.0f;

  } else if (state == 8) { //FIND_CORNER
    commandAlignCorner(&temp_vel_y, &temp_vel_w, -1 * max_rate, side_range, ref_distance_from_wall);
    temp_vel_x = 0.0f;
  }

  else {
    //State does not exist so hover!!
    commandHover(&temp_vel_x, &temp_vel_y, &temp_vel_w);
  }


  *vel_x = temp_vel_x;
  *vel_y = temp_vel_y;
  *vel_w = temp_vel_w;

  return state;

}
