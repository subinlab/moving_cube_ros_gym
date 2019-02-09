# moving_cube_ros_gym

Moving cube example using openai gym in ros gazebo.

This is not directory to real use, just updoaded files and folder for convenience.

Look communications between python scripts. And use urdf, yaml to set model and parameters.

<br>

<br>

<h3>Model</h3>

------

<h4>Robot Model</h4>

`Cube Robot(The Cubli)`

<em>Youtube</em> :  https://www.youtube.com/watch?v=n_6p-1J551Y

<br>

<h4>Training Example</h4>

<em>Youtube</em> : https://www.youtube.com/watch?v=3_afZzjAQbc

<br>

<br>

<h3>Settings</h3>

------

<h4>필요한 파일들 다운로드</h4>

```shell
cd ~/catkin_ws/src

git clone https://bitbucket.org/theconstructcore/moving_cube
git clone https://bitbucket.org/theconstructcore/moving_cube_training
git clone https://bitbucket.org/theconstructcore/moving_cube_ai
```

<br>

<h4>catkin_ws 안에 있는 ros package</h4>

- `moving_cube_description`
- `moving_cube_training_pkg`
- `moving_cube_learning`

<br>

<h4>Files in ros package</h4>

`.py` `.urdf` `.yaml` `.launch`



<br>

<h4>Launch example</h4>

```
roslaunch [ros_package_name] [launch_file_name]

roslaunch moving_cube_description spawn_moving_cube.launch
```

<br>

<h4>Topic list</h4>

```
rostopic list
```

<br>

<h4>ros Odometry msg</h4>

http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html

msg

```
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

example

```
  //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
```





<br>

<br>

<h3>Wrappers</h3>

------



Wrappers are used to transform an environment in a modular way:

```
env = gym.make('Pong-v0')
env = MyWrapper(env)

# Will be supported in future releases
from gym.wrappers import MyWrapper
```

- Don't forget to call super(class_name, self).**init**(env) if you override the wrapper's **init** function
- You can access the inner environment with `self.unwrapped`
- You can access the previous layer using `self.env`
- The variables `metadata`, `action_space`, `observation_space`, `reward_range`, and `spec` are copied to `self` from the previous layer
- Create a wrapped function for at least one of the following: `__init__(self, env)`, `_step`, `_reset`, `_render`, `_close`, or `_seed`
- Your layered function should take its input from the previous layer (`self.env`) and/or the inner layer (`self.unwrapped`)

<br>

<h3>Run</h3>

------

run gazebo empty world

```shell
roslaunch gazebo_ros empty_world.launch
```

cube 모델 gazebo에 넣기

```
roslaunch moving_cube_description spawn_moving_cube.launch
```

cube controller 실행하기

```
roslaunch moving_cube_description moving_cube_control.launch
```

q-learning training

```
rosrun moving_cube_training_pkg cube_rl_utils.py
```

q-learning training using openai gym

```
roslaunch my_moving_cube_training_pkg start_training.launch
```

<br>

<br>

<h3>ROBOTIS</h3>

------

<h4>[ROBOTIS]openmanipulator</h4>

http://emanual.robotis.com/docs/en/platform/openmanipulator/



```
roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch
```

![openmanipulator_chain_gazebo_1](https://user-images.githubusercontent.com/37301677/52516600-912bbc80-2c71-11e9-9f31-fa10e61632ed.png)

Enter `rostopic list`

```
 /clock
  /gazebo/link_states
  /gazebo/model_states
  /gazebo/set_link_state
  /gazebo/set_model_state
  /open_manipulator/gripper/kinematics_pose
  /open_manipulator/gripper_position/command
  /open_manipulator/gripper_sub_position/command
  /open_manipulator/joint1_position/command
  /open_manipulator/joint2_position/command
  /open_manipulator/joint3_position/command
  /open_manipulator/joint4_position/command
  /open_manipulator/joint_states
  /open_manipulator/option
  /open_manipulator/states
  /rosout
  /rosout_agg
```



<br>

<h4>[ROBOTIS]turtlebot3 machine learning</h4>

http://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/



<br>

<br>

<h3><em>Reference</em></h3>

------

- https://bitbucket.org/theconstructcore/moving_cube/src/master/
- https://bitbucket.org/theconstructcore/moving_cube_training/src/master/
- https://bitbucket.org/theconstructcore/moving_cube_ai/src/master/
