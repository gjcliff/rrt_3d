# rrt_3d
A package that demonstrates the Rapidly-Expanding Random Tree algorithm.

# Build
```bash
mkdir -p ws/src/
cd ws/src/
git clone https://github.com/gjcliff/rrt_3d.git
cd ../../
colcon build
ros2 launch rrt3d.launch.py
```
# Arguments
You can pass the following arguments to control aspects of the environment and
algorithm.
```bash
Arguments (pass arguments as '<name>:=<value>'):

    'world_bounds':
        The start position for RRT
        (default: '[10.0, 10.0, 10.0]')

    'start_coord':
        The start position for RRT
        (default: '[0.0]')

    'goal_coord':
        The start position for RRT
        (default: '[0.0]')

    'step_size':
        Default step size along unit vector in RRT for new nodes
        (default: '1.0')

    'use_rviz':
        Whether or not to use rviz
        (default: 'true')
```
