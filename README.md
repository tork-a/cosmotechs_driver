# cosmotechs_driver

This package contains driver nodes for control and mesurement board by
[CosmoTechs](http://www.cosmotechs.co.jp/). 

The supported products are:

- Dual axis motor control board [PCPG-23I(F)](http://www.cosmotechs.co.jp/products/?id=1406335747-890867)
- 32bit Digital I/O board [PCIO-32H/A](http://www.cosmotechs.co.jp/products/?id=1408345224-477949)

# Quick start with the test bed

## pcpg23i_node

Start the launch file as:

```
$ roslaunch cosmotechs_driver pcpg23i.launch
```

Publish command with check.py as:

```
$ rosrun cosmotechs_driver check.py 6.28 10
```

With this command, the motor rotates a round in 10 seconds.
It commands the angle and duration of motion.

## pcio32ha_node

Start the launch file as:

```
$ roslaunch cosmotechs_driver pcio32ha.launch
```

Publish command with rosservice as:

```
$ rosservice call /pcio32ha_node/set_port_bit 0 0 1

```

This set the bit[0] on port[0] to 1.

```
$ rosservice call /pcio32ha_node/set_port_bit 0 0 0

```

This set the bit[0] on port[0] to 0. In the testbed, this port is
connected to a electric relay, so you can hear click noises with this
command.

To get the input port status, 

```
$ rosservice call /pcio32ha_node/get_port_bit 0 0 0
data: 0
```

This get the bit[0] on port[0].  In the test bed, this bit is
connected to the toggle switch, so you can get the status of the
switch.

# ROS Nodes

## pcpg23i_node

pcg23i_node is a driver for PCPG-23I(F). 

### Actions

- command (cosmotechs_driver/MultiJointPosition)

### Services

- reset_angle (void)
  This service reset the motor angle to zero. No arguments and results.

### Parameters

- loopback (bool)
  When this parameter set to true, the node doesn't access actual hardware. Default is false.

- board_id (int)
  This parameter is set to specify the board id to access. Default is 0.

- pulse_per_round (float)
  This parameter is set to specify the number of pulse when the motor move a round. Default is 50000.

- acc_time (float)
  This parameter is set to specify the acceleration/deceleration time in the tapezoidal trajectory. Default is 0.2.

## pcio32ha_node

### Services

- get_port (cosmotechs_driver/GetPort)
- set_port (cosmotechs_driver/SetPort)

### Parameters

- loopback (bool)
  When this parameter set to true, the node doesn't access actual hardware. Default is false.

- board_id (int)
  This parameter is set to specify the board id to access. Default is 0.
