#include <stdint.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "cosmotechs_driver/pcpg23iw.h"
#include "cosmotechs_driver/MultiJointPositionAction.h"

// Board ID
static bool g_loopback = false;
static int board_id = 0;
static bool sync_flag = false;

typedef actionlib::SimpleActionServer < cosmotechs_driver::MultiJointPositionAction > Server;

void execute (const cosmotechs_driver::MultiJointPositionGoalConstPtr & goal, Server * as)
{
  cosmotechs_driver::MultiJointPositionResult result;
  cosmotechs_driver::MultiJointPositionFeedback feedback;
  feedback.positions.resize(2);
  feedback.velocities.resize(2);
  feedback.errors.resize(2);

  // Range Data Read
  unsigned short wData;
  unsigned long dwData;

  // 以下はAPI可能か
  // p.48 in Software Manual
  // Pcpg23iwStartSignalWrite(board_id, 0x03)
  if (sync_flag)
  {
    // Software Sync Mode Write: p.90 in Reference Manual
    for (int i = 0; i < 2; i++)
    {
      if (!g_loopback)
      {
        Pcpg23iwDataWrite (board_id, i, PCPG23I_SOFT_SYNC_MODE_WRITE, 1);
      }
    }
  }
  else
  {
    // Software Sync Mode Write: p.90 in Reference Manual
    for (int i = 0; i < 2; i++)
    {
      if (!g_loopback)
      {
        Pcpg23iwDataWrite (board_id, i, PCPG23I_SOFT_SYNC_MODE_WRITE, 0);
      }
    }
  }
  // set reference pulse
  // 以下はAPI可能か
  // p.48 in Software Manual
  // Pcpg23iwStartSignalWrite(board_id, 0x03)
  for (int axis = 0; axis < 2; axis++)
  {
    if (!g_loopback)
    {
      // Preset Pulse Drive: p.85 in Reference Manual
      if (goal->positions[axis] > 0) 
	{
	  Pcpg23iwDataFullWrite (board_id, axis,
				 PCPG23I_PLUS_PRESET_PULSE_DRIVE,
				 fabs(goal->positions[axis]));
	}
      else
	{
	  Pcpg23iwDataFullWrite (board_id, axis,
				 PCPG23I_MINUS_PRESET_PULSE_DRIVE,
				 fabs(goal->positions[axis]));
	}
      //Pcpg23iwDataFullWrite (board_id, axis,
      //PCPG23I_PRESET_PULSE_DATA_OVERRIDE,
      //goal->positions[axis]);
    }
  }
  if (sync_flag)
  {
    if (!g_loopback)
    {
      // Software Sync Execute: p.90 in Reference Manual
      Pcpg23iwCommandWrite (board_id, 0, PCPG23I_SOFT_SYNC_EXECUTE);
    }
  }
  for (int counter=0;;counter++)
  {
    // Check the end of motion
    // p.42 in Software Manual
    ROS_INFO("counter: %d\n", counter);
    unsigned char bSts = 0;
    if (!g_loopback)
    {
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
    // Check BUSY
    if ((bSts & 1) == 0)
    {
      ROS_INFO ("pcpg23i: command end");
      result.positions = goal->positions;
      as->setSucceeded (result);
      break;
    }
    // check that preempt has not been requested by the client
    if (as->isPreemptRequested () || !ros::ok ())
    {
      ROS_INFO ("pcpg23i: preempted");
      // set the action state to preempted
      // 通常停止はこれ
      for (int axis=0; axis<2; axis++)
	{
	  Pcpg23iwSlowStop(board_id, axis);
	}
      
      as->setPreempted ();
      break;
    }
    // Set feedback
    for (int i = 0; i < 2; i++)
    {
      if (!g_loopback)
      {
        // Internal Counter Read: P.83 in Reference Manual
        //Pcpg23iwDataFullRead (board_id, i, PCPG23I_INTERNAL_COUNTER_READ, &dwData);
        Pcpg23iwGetInternalCounter(board_id, i, &dwData);
        ROS_INFO ("Internal counter=%d\n", dwData);
        feedback.positions[i] = dwData;
        Pcpg23iwGetNowSpeedData(board_id, i, &wData);
        feedback.velocities[i] = wData;
      }
      else
      {
        feedback.positions[i] = counter * goal->positions[i] / 10;
        feedback.velocities[i] = 0;
      }
      feedback.errors[i] = 0;
    }
  }
}

/**
 * @brief main function of the ROS node
 */
int main (int argc, char **argv)
{
  ros::init (argc, argv, "pcpg23i_node");
  ros::NodeHandle n("~");
  unsigned short wData;
  unsigned long dwData;

  // Parameters
  n.param ("loopback", g_loopback, false);
  n.param ("board_id", board_id, 0);
  n.param ("sync", sync_flag, true);
  
  if (!g_loopback)
  {
    if (Pcpg23iwCreate (board_id) < 0)
    {
      ROS_ERROR ("Create Error\n");
      return -1;
    }
  }
  // ボード情報を表示
  pciresource res;
  Pcpg23iwGetResource(board_id, &res);
  ROS_INFO("bus: %d", res.bus);
  ROS_INFO("dev: %d", res.dev);
  ROS_INFO("func: %d", res.func);
  ROS_INFO("baseclass: %d", res.baseclass);
  ROS_INFO("subclass: %d", res.subclass);
  ROS_INFO("programif: %d", res.programif);
  ROS_INFO("revision: %d", res.revision);
  ROS_INFO("irq: %d", res.irq);
  ROS_INFO("Bsn: %d", res.Bsn);
  ROS_INFO("mem_base: %x", res.Mem_base);
  ROS_INFO("Io_base: %x", res.Io_base);
  // return 0;

  // エラー情報の取得はこれ
  //   Pcpg23iwGetLastError()

  // 通常停止はこれ
  // Pcpg23iwSlowStop()

  // 緊急停止はこれ
  // Pcpg23iwEmergencyStop

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

      Pcpg23iwDataHalfRead (board_id, axis, PCPG23I_RANGE_READ, &wData);
      ROS_INFO ("Range Data=%d\n", wData);

      // このへんはすべてSpeedParameterWriteで置き換えられるはず
      //Start Stop Speed Data (Offset of speed) Write
      Pcpg23iwDataHalfWrite (board_id, axis, PCPG23I_START_STOP_SPEED_DATA_WRITE, 100);
      //Start Stop Speed Data (Offset of speed) Read
      Pcpg23iwDataHalfRead (board_id, axis, PCPG23I_START_STOP_SPEED_DATA_READ, &wData);
      ROS_INFO ("Start Stop Data=%d\n", wData);

      Pcpg23iwDataHalfWrite (board_id, axis, 0x04, 8000); //Object Data Write
      Pcpg23iwDataHalfRead (board_id, axis, 0x05, &wData);
      ROS_INFO ("Object Data=%d\n", wData);

      // Rate: S字加減速比率
      Pcpg23iwDataHalfWrite (board_id, axis, 0x06, 2048); //RATE-1 Data Write
      Pcpg23iwDataHalfRead (board_id, axis, 0x07, &wData);
      ROS_INFO ("RATE-1 Data=%d\n", wData);

      // Internal Counter: 原点合わせ?
      Pcpg23iwDataFullWrite (board_id, axis, PCPG23I_INTERNAL_COUNTER_WRITE, 0);
      Pcpg23iwDataFullRead (board_id, axis, PCPG23I_INTERNAL_COUNTER_READ, &dwData);
      printf ("Internal Counter=%lu\n", dwData);
    }
  }

  Server server (n, "command", boost::bind (&execute, _1, &server), false);
  server.start ();
  ros::spin ();

  if (!g_loopback)
  {
    Pcpg23iwClose(board_id);
  }
  return 0;
}
