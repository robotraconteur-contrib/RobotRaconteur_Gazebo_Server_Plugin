# Robot Raconteur Gazebo Server Plugin

A plugin for [Gazebo](http://gazebosim.org) using the [Robot Raconteur](http://robotraconteur.com) communication system to provide a friendly object-oriented interface to the Gazebo API without requiring the modification of SDF world files. It is developed by Wason Technology, LLC.

## Building the plugin

The plugin is built using CMake. Gazebo and the Robot Raconteur SDK must be installed before building. See https://github.com/robotraconteur/robotraconteur for Robot Raconteur installation instructions.

To build, set **RobotRaconteur_DIR** to the directory of the extracted Robot Raconteur SDK.

## Invoking the plugin

First, set the **GAZEBO_PLUGIN_PATH** environmental variable to the folder containing `librobotraconteur_gazebo_system_plugin.so'.

The plugin is invoked using the command line. For the examples, use:

`gazebo --verbose rip_sensors_world.world -s libgazebo_robotraconteur_server_plugin.so --robotraconteur-server-tcp-port=11346`

The plugin will scan the command line to configure plugin options. The options that are recognized are:

`--robotraconteur-server-nodename=`*nodename*

Set the NodeName of the server plugin. Default is "experimental.gazebo.GazeboServer".

`--robotraconteur-server-nodeid=`*nodeid*

Set the NodeID of the plugin. Option is mutually exclusive with `--robotraconteur-server-nodename`

`--robotraconteur-server-tcp-port=`*port*

Set the TCP port to use for the plugin. Set to "sharer" to use the Robot Raconteur Port Sharer service.

`--robotraconteur-server-tcp-announce`

Enable the node discovery announce for Robot Raconteur. Search for service type `experimental.gazebo.Server` to find the service.

`--robotraconteur-server-tcp-loadtls`

Load a Robot Raconteur TLS certificate for the node to enable encryption over TCP. Enables use of `rrs+tcp://` transport schemes.

`--robotraconteur-server-password-file=`*filename*

Load a file containing password information to enable authentication. Uses `PasswordFileUserAuthenticator` to authenticate users.

## Designing clients

Clients using Python, MATLAB, and JavaScript will automatically generate object references to communicate with the server plugin. For the other languages, the file `experimental.gazebo.robdef` contains the full service definition to use with RobotRaconteurGen.

## Examples

The examples directory contains a number of Python scripts demonstrating the usage of the plugin. While the examples are in Python, clients can be written in any language that Robot Raconteur supports.

## Build in ROS workspace

    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone https://github.com/robotraconteur/robotraconteur.git
    git clone https://github.com/johnwason/robotraconteur_standard_robdef_cpp.git
    git clone https://github.com/johnwason/RobotRaconteur_Gazebo_Server_Plugin.git
    cd ..
    catkin_make_isolated -DROBOTRACONTEUR_ROS=1 -DCMAKE_BUILD_TYPE=Release
    source devel_isolated/setup.bash

## License

This plugin is released under the Apache 2 license. Robot Raconteur itself is released under the Robot Raconteur commercial license.



