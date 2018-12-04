## This README contains a brief overview of each of my completed tutorial files.

### - `wander.py` - This node controls a simulation of TurtleBot in visualization tool, Gazebo. The control system sends turtlebot forward at .2 m/s for 10 seconds or unless an obstacle is dectected within 1.2 of the front laser scanner. In either scenario, the TurtleBot will spin for 5 seconds and cotninue driving forward.

### - `red_light_green_light.py` - This node also controls a simulation of TurleBot in visualization tool, Gazebo. The control system drive forward for 3 secods at .25 m/s and then stops for 3 seconds.

### - `talker_param.py` - This node demonstrates a use for parameters. If the node is called using a private parameter, it will output "Hello I'm [the private parameter] current time", else it will simply use default name "Anybody".

### - `keys_to_twist_with_ramps.py` - This node offers scaled and ramping velocities and accelerations. In other words, rather than commanding the TurtleBot 100% or 0 command velocity, this code offers smoother ramp-ups. And while that doesn't mean a lot in a simulation, this tweak in a real robot will ensure a longer shelf-life for all the mechanical components.

### - `key_publisher.py` This node monitors the keyboard inputs and publishes them to the topic `keys`.

### - launch file keys_to_twist_with_ramps.launch creates instances of  `key_publisher.py` and `keys_to_twist_with_ramps.py` and also sends private parameters into `keys_to_twist_with_ramps.py` that overwrite the defaults for scaling on velocity and acceleration. Finally, it open a session of Gazebo with TurtleBot to be commanded by the user input into `key_publisher.py`
