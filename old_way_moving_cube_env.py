import gym
import rospy
import time
import numpy as np
import math
import copy
from gym import utils, spaces
import numpy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from nav_msgs.msgs import Odometry
from gazebo_connection import GazeboConnection
from controllers_connection import ControllersConnection

from gym.utils import seeding
from gym.envs.registration import register

reg = register(
    id='OldMovingCube-v0',
    entry_point='old_way_moving_cube_env:OldMovingCubeEnv',
    timestep_limit=1000,
    )

class OldMovingCubeEnv(gym.Env):

    def __init__(self):
        self.publishers_array = []
        self._roll_vel_pub = rospy.Publisher('/moving_cube/inertia_wheel_roll_joint_velocity_controller/command', Float64, queue_size=1)
        self.publishers_array.append(self._roll_vel_pub)

        self.action_space = spaces.Discrete(3) #l,r,nothing
        self._seed()
        
        #get configuration parameters
        self.wait_time = rospy.get_param('/moving_cube/wait_time')
        self.running_step = rospy.get_param('/moving_cube/running_step')
        self.speed_step = rospy.get_param('/moving_cube/speed_step')
        self.init_roll_vel = rospy.get_param('/moving_cube/init_roll_vel')

        self.init_internal_vars(self.init_roll_vel)

        rospy.Subscriber("/moving_cube/joint_states", JointState, self.joints_callback)
        rospy.Subscriber("/moving_cube/odom", Odometry, self.odom_callback)

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()
        self.controllers_object = ControllersConnection(namespace="moving_cube")

    def init_internal_vars(self, init_roll_vel_value):
        self.roll_vel = [init_roll_vel_value]
        self.joints = None

    #always returns the current state of the joints
    def joints_callback(self, data):
        self.joints = data

    def odom_callback(self, data):
        self.odom = data

    def get_clock_time(self):
        self.clock_time = None
        while self.clock_time is None and not rospy.is_shutdown():
            try:
                self.clock_time = rospy.wait_for_message("/clock", Clock, timeout=1.0)
                rospy.logdebug("Current clock_time READY=>" + str(self.clock_time))
            except:
                rospy.logdebug("Current clock_time not ready yet, retrying for getting Current clock_time")
        return self.clock_time
        
    def _seed(self, seed=None): #overriden function
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _step(self, action):#overriden function

        # Take action
        if action == 0: #LEFT
            rospy.logwarn("GO LEFT...")
            self.roll_vel[0] -= self.speed_step
        elif action == 1: #RIGHT
            rospy.logwarn("GO RIGHT...")
            self.roll_vel[0] += self.speed_step
        elif action == 1: #STOP
            rospy.logwarn("STOP...")
            self.roll_vel[0] = 0.0

        rospy.logwarn("MOVING TO SPEED=="+str(self.roll_vel))

        # 1st: unpause simulation
        rospy.logdebug("Unpause SIM...")
        self.gazebo.unpauseSim()

        self.move_joints(self.roll_vel)
        rospy.logdebug("Wait for some time to execute movement, time="+str(self.running_step))
        rospy.sleep(self.running_step) #wait for some time
        rospy.logdebug("DONE Wait for some time to execute movement, time=" + str(self.running_step))

        # 3rd: pause simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()

        # 4th: get the observation
        observation, done, state = self.observation_checks()

        # 5th: get the reward
        if not done:
            step_reward = 0
            obs_reward = self.get_reward_for_observations(state)
            rospy.loginfo("Reward Values: Time="+str(step_reward)+",Obs="+str(obs_reward))
            reward = step_reward + int(obs_reward)
            rospy.loginfo("TOT Reward=" + str(reward))
        else:
            reward = -2000000

        return observation, reward, done, {}

    def _reset(self):

        rospy.logdebug("We UNPause the simulation to start having topic data")
        self.gazebo.unpauseSim()

        rospy.logdebug("CLOCK BEFORE RESET")
        self.get_clock_time()

        rospy.logdebug("SETTING INITIAL POSE TO AVOID")
        self.set_init_pose()
        time.sleep(self.wait_time * 2.0)
        rospy.logdebug("AFTER INITPOSE CHECKING SENSOR DATA")
        self.check_all_systems_ready()

        rospy.logdebug("RESETING SIMULATION")
        self.gazebo.pauseSim()
        self.gazebo.resetSim()
        self.gazebo.unpauseSim()
        rospy.logdebug("CLOCK AFTER RESET")
        self.get_clock_time()

        rospy.logdebug("RESETING CONTROLLERS SO THAT IT DOESNT WAIT FOR THE CLOCK")
        self.controllers_object.reset_cartpole_joint_controllers()
        rospy.logdebug("AFTER RESET CHECKING SENSOR DATA")
        self.check_all_systems_ready()
        rospy.logdebug("CLOCK AFTER SENSORS WORKING AGAIN")
        self.get_clock_time()
        rospy.logdebug("END")

        # 7th: pauses simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()

        # get the last observation got when paused, generated by the callbakc or the check_all_systems_ready
        # Depends on who was last
        observation, _, state = self.observation_checks()

        return observation
        
        
    '''
    UTILITY CODE FOLLOWS HERE
    '''
    
    def observation_checks(self):
        done = False
        # State defined by the speed of the disk and position in the world plane
        self.get_distance_from_point(pstart, p_end)
        state = [round(self.joints.velocity[0],1), round(self.odom.pose.pose.position.x,1), round(self.odom.pose.pose.position.y,1)]
        # TODO: Create Correct Observation

        if (self.min_base_position >= state[0] or state[0] >= self.max_base_position): #check if the base is still within the ranges of (-2, 2)
            rospy.logerr("Base Ouside Limits==>min="+str(self.min_base_position)+",pos="+str(state[0])+",max="+str(self.max_base_position))
            done = True
        if (self.min_pole_angle >= state[2] or state[2] >= self.max_pole_angle): #check if pole has toppled over
            rospy.logerr(
                "Pole Angle Ouside Limits==>min=" + str(self.min_pole_angle) + ",pos=" + str(state[2]) + ",max=" + str(
                    self.max_pole_angle))
            done = True

        observations = [state[2]]

        return observations, done, state

    def get_distance_from_point(self, pstart, p_end):
        """
        Given a Vector3 Object, get distance from current position
        :param p_end:
        :return:
        """
        a = numpy.array((pstart.x, pstart.y, pstart.z))
        b = numpy.array((p_end.x, p_end.y, p_end.z))

        distance = numpy.linalg.norm(a - b)

        return distance

    def get_reward_for_observations(self, state):
        """
        Gives more points for staying upright, gets data from given observations to avoid
        having different data than other previous functions
        :return:reward
        """

        pole_angle = state[2]
        pole_vel = state[3]

        rospy.logwarn("pole_angle for reward==>" + str(pole_angle))
        delta = 0.7 - abs(pole_angle)
        reward_pole_angle = math.exp(delta*10)

        # If we are moving to the left and the pole is falling left is Bad
        rospy.logwarn("pole_vel==>" + str(pole_vel))
        pole_vel_sign = numpy.sign(pole_vel)
        pole_angle_sign = numpy.sign(pole_angle)
        rospy.logwarn("pole_vel sign==>" + str(pole_vel_sign))
        rospy.logwarn("pole_angle sign==>" + str(pole_angle_sign))

        # We want inverted signs for the speeds. We multiply by -1 to make minus positive.
        # global_sign + = GOOD, global_sign - = BAD
        base_reward = 500
        if pole_vel != 0:
            global_sign = pole_angle_sign * pole_vel_sign * -1
            reward_for_efective_movement = base_reward * global_sign
        else:
            # Is a particular case. If it doesnt move then its good also
            reward_for_efective_movement = base_reward

        reward = reward_pole_angle + reward_for_efective_movement

        rospy.logwarn("reward==>" + str(reward)+"= r_pole_angle="+str(reward_pole_angle)+",r_movement= "+str(reward_for_efective_movement))
        return reward


    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self._roll_vel_pub.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.logdebug("No susbribers to _roll_vel_pub yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("_base_pub Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def check_all_systems_ready(self, init=True):
        self.disk_joints_data = None
        while self.disk_joints_data is None and not rospy.is_shutdown():
            try:
                self.disk_joints_data = rospy.wait_for_message("/moving_cube/joint_states", JointState, timeout=1.0)
                rospy.logdebug("Current moving_cube/joint_states READY=>"+str(self.disk_joints_data))

            except:
                rospy.logerr("Current moving_cube/joint_states not ready yet, retrying for getting joint_states")

        self.cube_odom_data = None
        while self.disk_joints_data is None and not rospy.is_shutdown():
            try:
                self.cube_odom_data = rospy.wait_for_message("/moving_cube/odom", Odometry, timeout=1.0)
                rospy.logdebug("Current /moving_cube/odom READY=>" + str(self.cube_odom_data))

            except:
                rospy.logerr("Current /moving_cube/odom not ready yet, retrying for getting odom")
        rospy.logdebug("ALL SYSTEMS READY")


    def move_joints(self, joints_array):
        i = 0
        for joint_value in joints_array:
            joint_value = Float64()
            joint_value.data = joints_array[0]
            rospy.logdebug("Single Disk Joints Velocity>>"+str(joint_value))
            self.publishers_array[i].publish(joint_value)
            i += 0


    def set_init_pose(self):
        """
        Sets Roll Disk to initial vel [0.0]
        :return:
        """
        self.check_publishers_connection()
        # Reset Internal pos variable
        self.init_internal_vars(self.init_roll_vel)
        self.move_joints(self.roll_vel)