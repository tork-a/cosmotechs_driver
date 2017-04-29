#include <stdint.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "cosmotechs_driver/pcpg23i.h"
#include "cosmotechs_driver/MultiJointPositionAction.h"

// Board ID
static int board_id;

typedef actionlib::SimpleActionServer<cosmotechs_driver::MultiJointPositionAction> Server;

void execute(const cosmotechs_driver::MultiJointPositionGoalConstPtr& goal, Server* as)
{
  cosmotechs_driver::MultiJointPositionResult result;

#if 0
  for(axis=0; axis<2; axis++){
    // Initial Clear: p.93 in Reference Manual
    Pcpg23iwCommandWrite(board_id, axis, PCPG23I_INITIAL_CLEAR);

    // Mode1: P.63 in Reference Manual
    Pcpg23iwMode1Write(wBsn,wAxis, 0x40);

    // Mode2: P.61 in Reference Manual
    Pcpg23iwMode2Write(wBsn,wAxis, 0x3F);

    // Range Data Write
    Pcpg23iwDataHalfWrite(wBsn,wAxis,PCPG23I_RANGE_WRITE, 1000);

    // Range Data Read
    Pcpg23iwDataHalfRead(wBsn,wAxis,PCPG23I_RANGE_READ, &wData);
    printf("Range Data=%d\n",wData);

    // このへんはすべてSpeedParameterWriteで置き換えられるはず
    //Start Stop Speed Data (Offset of speed) Write
    Pcpg23iwDataHalfWrite(wBsn, wAxis,PCPG23I_START_STOP_SPEED_DATA_WRITE,100);
    //Start Stop Speed Data (Offset of speed) Read
    Pcpg23iwDataHalfRead(wBsn,wAxis,PCPG23I_START_STOP_SPEED_DATA_READ,&wData);
    printf("Start Stop Data=%d\n",wData);

    Pcpg23iwDataHalfWrite(wBsn,wAxis,0x04,8000); //Object Data Write
    Pcpg23iwDataHalfRead(wBsn,wAxis,0x05,&wData);
    printf("Object Data=%d\n",wData);

    // Rate: S字加減速比率
    Pcpg23iwDataHalfWrite(wBsn,wAxis,0x06,2048); //RATE-1 Data Write
    Pcpg23iwDataHalfRead(wBsn,wAxis,0x07,&wData);
    printf("RATE-1 Data=%d\n",wData);

    // Internal Counter: 原点合わせ?
    Pcpg23iwDataFullWrite(wBsn,wAxis,PCPG23I_INTERNAL_COUNTER_WRITE,0);
    Pcpg23iwDataFullRead(wBsn,wAxis,PCPG23I_INTERNAL_COUNTER_READ,&dwData);
    printf("Internal Counter=%lu\n",dwData);
  }
  // 以下はAPI可能か
  // p.48 in Software Manual
  // Pcpg23iwStartSignalWrite(wBsn, 0x03)
  for(cnt=0;cnt<2;cnt++){
    // Software Sync Mode Write: p.90 in Reference Manual
    Pcpg23iwDataWrite(wBsn, cnt, PCPG23I_SOFT_SYNC_MODE_WRITE, 1);
  }

  // 以下はAPI可能か
  // p.48 in Software Manual
  // Pcpg23iwStartSignalWrite(wBsn, 0x03)
  for(cnt=0;cnt<2;cnt++){
    // Preset Pulse Drive: p.85 in Reference Manual
    Pcpg23iwDataFullWrite(wBsn,0,PCPG23I_PLUS_PRESET_PULSE_DRIVE,10000);
  }

  // Software Sync Execute: p.90 in Reference Manual
  Pcpg23iwCommandWrite(wBsn, 0, PCPG23I_SOFT_SYNC_EXECUTE);
  while(1)
  {
    // p.42 in Software Manual
    Pcpg23iwGetDriveStatus(wBsn, 0, &bSts);
    // Check BUSY
    if((bSts & 1) == 0){
      break;
    }
    // Internal Counter Read: P.83 in Reference Manual
    Pcpg23iwDataFullRead(wBsn,0, PCPG23I_INTERNAL_COUNTER_READ, &dwData);
    // Pcpg23iwGetInternalCounter(wBsn, 0, &dwData);
    printf("Internal Counter=%lu\n",dwData);
 }
  for(cnt=0;cnt<2;cnt++){
    
    Pcpg23iwDataWrite(wBsn, cnt, PCPG23I_SOFT_SYNC_MODE_WRITE, 0);
  }

#endif



  ROS_INFO_STREAM(__func__ << ":" << goal);
  result.positions = goal->positions;
  as->setSucceeded(result);
}

/**
 * @brief main function of the ROS node
 */
int main (int argc, char **argv)
{
  ros::init (argc, argv, "pcio32ha_node");
  ros::NodeHandle n;

  // Parameters
  n.param("board_id", board_id, 0);
  // n.param<uint8_t&>("board_id", board_id, (uint8_t)0);

#if 0
  if(Pcpg23iwCreate(wBsn)==-1){
    printf("Create Err\n");
    return -1;
  }
  // ボード情報を表示
  // Pcpg23iwGetResource()

  // エラー情報の取得はこれ
  //   Pcpg23iwGetLastError()

  // 通常停止はこれ
  // Pcpg23iwSlowStop()

  // 緊急停止はこれ
  // Pcpg23iwEmergencyStop

#endif
  Server server(n, "command", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();

  //Pcpg23iwClose();

  return 0;
}
