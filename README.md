# Robot Raconteur Gazebo Server Plugin

A plugin for [Gazebo](http://gazebosim.org) using the [Robot Raconteur](http://robotraconteur.com) communication system to provide a friendly object-oriented interface to the Gazebo API without requiring the modification of SDF world files. It is developed by Wason Technology, LLC.

## Download plugin binary
-------------------------

The Robot Raconteur Gazebo Plugin binaries  can be downloaded from [http://robotraconteur.com/download](http://robotraconetur.com/download). Registration is required but the dowload is free.

## Building the plugin
----------------------

The plugin is built using CMake. Building has only been tested on Ubuntu Trusty and Xenial.  Gazebo and the Robot Raconteur SDK must be installed before building. The Robot Raconteur SDK can be download at [http://robotraconteur.com/download](http://robotraconetur.com/download). Registration is required but the dowload is free. Be sure to download the correct *Ubuntu* SDK version that matches your operating system.

To build, set **RobotRaconteur_DIR** to the directory of the extracted Robot Raconteur SDK.

## Invoking the plugin
----------------------

First, set the **GAZEBO_PLUGIN_PATH** environmental variable to the folder containing `librobotraconteur_gazebo_system_plugin.so'.

The plugin is invoked using the command line. For the examples, use:

`gazebo --verbose rip_sensors_world.world -s librobotraconteur_gazebo_server_plugin.so --robotraconteur-server-tcp-port=11346`

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
--------------------

Clients using Python, MATLAB, and JavaScript will automatically generate object references to communicate with the server plugin. For the other languages, the file `experimental.gazebo.robdef` contains the full service definition to use with RobotRaconteurGen.

## Examples
-----------

The examples directory contains a number of Python scripts demonstrating the usage of the plugin. While the examples are in Python, clients can be written in any language that Robot Raconteur supports.

## License
----------

This plugin is released under the Apache 2 license. Robot Raconteur itself is released under the Robot Raconteur commercial license.



