# cosmotechs_driver

This package contains driver nodes for control and mesurement board by
[CosmoTechs](http://www.cosmotechs.co.jp/). 

The supported products are:

- Dual axis motor control board [PCPG-23I(F)](http://www.cosmotechs.co.jp/products/?id=1406335747-890867)
- 32bit Digital I/O board [PCIO-32H/A](http://www.cosmotechs.co.jp/products/?id=1408345224-477949)

# ROS Nodes

## pcpg23i_node

pcg23i_node is a driver for PCPG-23I(F). 

### Actions

- command (cosmotechs_driver/MultiJointPosition)

## pcio32ha_node

### Services

- get_port (cosmotechs_driver/GetPort)
- set_port (cosmotechs_driver/SetPort)

