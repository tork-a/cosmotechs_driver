#include <stdint.h>
#include <ros/ros.h>
#include "cosmotechs_driver/pcio32ha.h"
#include "cosmotechs_driver/GetPort.h"
#include "cosmotechs_driver/SetPort.h"
#include "cosmotechs_driver/GetPortBit.h"
#include "cosmotechs_driver/SetPortBit.h"

// Board ID
static int board_id;

bool GetPort (cosmotechs_driver::GetPort::Request & req, cosmotechs_driver::GetPort::Response & res)
{
  uint8_t data = 0;
  ROS_INFO ("%s: port_id=%d", __func__, (int) req.port_id);
#if 0
  if (Pcio32hwInPort(board_id, req.port_id, &data) < 0)
  {
  }
#endif
  res.data = data;
  return true;
}

bool SetPort (cosmotechs_driver::SetPort::Request & req, cosmotechs_driver::SetPort::Response & res)
{
  ROS_INFO ("%s: port_id=%d, data=%x", __func__, (int) req.port_id, (int) req.data);
  //Pcio32hwOutPort(board_id, req.port_id, req.data);
  return true;
}

bool GetPortBit (cosmotechs_driver::GetPortBit::Request & req,
                 cosmotechs_driver::GetPortBit::Response & res)
{
  uint8_t data = 0;
  ROS_INFO ("%s: port_id=%d", __func__, (int) req.port_id);
#if 0
  if (Pcio32hwInPort(board_id, req.port_id, &data) < 0)
  {
  }
#endif
  res.data = data;
  return true;
}

bool SetPortBit (cosmotechs_driver::SetPortBit::Request & req,
                 cosmotechs_driver::SetPortBit::Response & res)
{
  ROS_INFO ("%s: port_id=%d, bit=%d, data=%d", __func__, (int) req.port_id, (int) req.bit, (int)req.data);
  //Pcio32hwOutPort(board_id, req.port_id, req.data);
  return true;
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

#if 0
  if (Pcio32hwCreate (board_id) == -1)
  {
    ROS_ERROR("Cannot initialize I/O board: %d\n", board_id);
    return -1;
  }
#endif

  ros::ServiceServer s_get_port = n.advertiseService ("get_port", GetPort);
  ros::ServiceServer s_set_port = n.advertiseService ("set_port", SetPort);
  ros::ServiceServer s_get_port_bit = n.advertiseService ("get_port_bit", GetPortBit);
  ros::ServiceServer s_set_port_bit = n.advertiseService ("set_port_bit", SetPortBit);

  ros::spin ();

  //Pcio32hwClose();

  return 0;
}
