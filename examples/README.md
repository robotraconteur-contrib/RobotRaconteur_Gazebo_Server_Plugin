# Robot Raconteur Gazebo Server Plugin Examples

This directory contains a number of Python scripts that demonstrate how to use the plugin. The examples include two SDF world files, `rip_world.world` and `rip_sensor_world.world`. These files implement the classic Rotary Inverted Pendulum mechatronics example. `rip_sensor_world.world` contains extra sensors and sensor targets, while `rip_sensor_world.world` is the pendulum and a single camera.

## Starting Gazebo
------------------

Be sure to set **GAZEBO_PLUGIN_PATH** environmental variable to the folder containing `librobotraconteur_gazebo_system_plugin.so'.

To start `rip_world.world`, run:

`gazebo --verbose rip_world.world -s librobotraconteur_gazebo_server_plugin.so --robotraconteur-server-tcp-port=11346`

To start `rip_sensor_world.world`:

`gazebo --verbose rip_sensors_world.world -s librobotraconteur_gazebo_server_plugin.so --robotraconteur-server-tcp-port=11346`

## Running the Python clients
-----------------------------

Running the Python clients requires installing the Python Robot Raconteur module. It can be downloaded from [http://robotraconteur.com/download](http://robotraconetur.com/download). Registration is required but the dowload is free. Be sure to download the Linux x86_64 version of the Python module.