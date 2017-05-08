#include <stdint.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "cosmotechs_driver/pcpg23iw.h"
#include "cosmotechs_driver/MultiJointPositionAction.h"
#include "cosmotechs_driver/ResetAngle.h"

// Set true without actual hardware
static bool g_loopback = false;
// Board ID
static int board_id = 0;
// Pulse per round [pulse]
static double g_pulse_per_round;
// Acceleartion time [sec]
static double g_acc_time = 0.2;

typedef actionlib::SimpleActionServer < cosmotechs_driver::MultiJointPositionAction > Server;

void execute (const cosmotechs_driver::MultiJointPositionGoalConstPtr & goal, Server * as)
{
  cosmotechs_driver::MultiJointPositionResult result;
  cosmotechs_driver::MultiJointPositionFeedback feedback;
  feedback.positions.resize (2);
  feedback.velocities.resize (2);
  feedback.errors.resize (2);

  long abs_pulse[2];
  long ref_pulse[2];
  double high_speed[2] = { 0, 0 };
  double low_speed[2] = { 0, 0 };

  // Get current pulse
  for (int i = 0; i < 2; i++)
  {
    Pcpg23iwGetInternalCounter (board_id, i, PDWORD (&abs_pulse[i]));
  }
  // 相対パルスに変換
  for (int i = 0; i < 2; i++)
  {
    ref_pulse[i] = int (goal->positions[i] * g_pulse_per_round / (2 * M_PI)) - int (abs_pulse[i]);
  }
  // 動作時間を実現するための速度の設定
  double all_time = goal->min_duration.toSec ();

  if (all_time > 2 * g_acc_time)
  {
    for (int i = 0; i < 2; i++)
    {
      high_speed[i] = fabs ((ref_pulse[i] - low_speed[i] * g_acc_time) / (all_time - g_acc_time));
    }
  }
  else
  {
    for (int i = 0; i < 2; i++)
    {
      high_speed[i] = (4 * ref_pulse[i] * g_acc_time +
                       (all_time * all_time - all_time * g_acc_time) * low_speed[i]) / (all_time * all_time);
    }
  }
  // 速度パラメータの設定(台形速度)
  for (int i = 0; i < 2; i++)
  {
    Pcpg23iwSpeedParameterWrite (board_id, i, low_speed[i], high_speed[i], short (1000 * g_acc_time), 0.0);
  }
  ROS_DEBUG ("abs_pulse: %ld %ld", abs_pulse[0], abs_pulse[1]);
  ROS_DEBUG ("ref_pulse: %ld %ld", ref_pulse[0], ref_pulse[1]);
  ROS_DEBUG ("all_time: %f", all_time);
  ROS_DEBUG ("acc_time: %f", g_acc_time);
  ROS_DEBUG ("low_speed: %f %f", low_speed[0], low_speed[1]);
  ROS_DEBUG ("high_speed: %lf %lf", high_speed[0], high_speed[1]);

  // p.48 in Software Manual
  for (int axis = 0; axis < 2; axis++)
  {
    if (ref_pulse[axis] > 0)
    {
      if (!g_loopback)
        // Preset Pulse Drive: p.85 in Reference Manual
        Pcpg23iwDataFullWrite (board_id, axis, PCPG23I_PLUS_PRESET_PULSE_DRIVE, fabs (ref_pulse[axis]));
    }
    else
    {
      if (!g_loopback)
        // Preset Pulse Drive: p.85 in Reference Manual
        Pcpg23iwDataFullWrite (board_id, axis, PCPG23I_MINUS_PRESET_PULSE_DRIVE, fabs (ref_pulse[axis]));
    }
  }
  // Software Sync Execute: p.90 in Reference Manual
  if (!g_loopback)
    Pcpg23iwCommandWrite (board_id, 0, PCPG23I_SOFT_SYNC_EXECUTE);
  for (int counter = 0;; counter++)
  {
    unsigned char bSts = 0;
    if (!g_loopback)
    {
      // Check the end of motion: p.42 in Software Manual
      Pcpg23iwGetDriveStatus (board_id, 0, &bSts);
    }
    else
    {
      if (counter > 10)
      {
        bSts = 0;
      }
      else
      {
        bSts = 1;
      }
    }
    // Check if the motion end
    if ((bSts & 1) == 0)
    {
      result.positions = goal->positions;
      as->setSucceeded (result);
      break;
    }
    // check that preempt has not been requested by the client
    if (as->isPreemptRequested () || !ros::ok ())
    {
      // set the action state to preempted
      for (int i = 0; i < 2; i++)
      {
        Pcpg23iwSlowStop (board_id, i);
      }
      as->setPreempted ();
      break;
    }
    // Set feedback
    for (int i = 0; i < 2; i++)
    {
      if (!g_loopback)
      {
        unsigned long pulse;
        unsigned short speed;
        // Internal Counter Read: P.83 in Reference Manual
        Pcpg23iwGetInternalCounter (board_id, i, &pulse);
        feedback.positions[i] = 2 * M_PI * pulse / g_pulse_per_round;
        Pcpg23iwGetNowSpeedData (board_id, i, &speed);
        feedback.velocities[i] = 2 * M_PI * speed / g_pulse_per_round;
      }
      else
      {
        feedback.positions[i] = counter * goal->positions[i] / 10;
        feedback.velocities[i] = 0;
      }
      feedback.errors[i] = 0;
    }
    as->publishFeedback (feedback);
  }
}

/**
 * @brief Reset internal counter(angle) to zero
 */
bool ResetAngle (cosmotechs_driver::ResetAngle::Request & req, cosmotechs_driver::ResetAngle::Response & res)
{
  if (!g_loopback)
  {
    // Set internal counter to zero
    for (int i = 0; i < 2; i++)
    {
      Pcpg23iwSetInternalCounter (board_id, i, 0);
    }
  }
  return true;
}

/**
 * @brief main function of the ROS node
 */
int main (int argc, char **argv)
{
  ros::init (argc, argv, "pcpg23i_node");
  ros::NodeHandle n ("~");

  // Parameters
  n.param ("loopback", g_loopback, false);
  n.param ("board_id", board_id, 0);
  n.param ("pulse_per_round", g_pulse_per_round, 50000.0);
  n.param ("acc_time", g_acc_time, 0.2);

  if (!g_loopback)
  {
    if (Pcpg23iwCreate (board_id) < 0)
    {
      ROS_ERROR ("Cannot initialize PCPG23I(F) board[%d]. Check the board is available.", board_id);
      return -1;
    }
  }
  // Initialize the servo board
  if (!g_loopback)
  {
    for (int axis = 0; axis < 2; axis++)
    {
      // Initial Clear: p.93 in Reference Manual
      Pcpg23iwCommandWrite (board_id, axis, PCPG23I_INITIAL_CLEAR);
      // Mode1: P.63 in Reference Manual
      Pcpg23iwMode1Write (board_id, axis, 0x40);
      // Mode2: P.61 in Reference Manual
      Pcpg23iwMode2Write (board_id, axis, 0x3F);
      // Range Data Write
      Pcpg23iwDataHalfWrite (board_id, axis, PCPG23I_RANGE_WRITE, 1000);
      // Software Sync Mode Write: p.90 in Reference Manual
      Pcpg23iwDataWrite (board_id, axis, PCPG23I_SOFT_SYNC_MODE_WRITE, 1);
    }
  }
  Server server (n, "command", boost::bind (&execute, _1, &server), false);
  ros::ServiceServer s_reset = n.advertiseService ("reset_angle", ResetAngle);

  server.start ();
  ros::spin ();

  // Stop the motors
  if (!g_loopback)
  {
    for (int i = 0; i < 2; i++)
    {
      Pcpg23iwSlowStop (board_id, i);
      //Pcpg23iwEmergencyStop(board_id, i);
    }
    Pcpg23iwClose (board_id);
  }
  return 0;
}
