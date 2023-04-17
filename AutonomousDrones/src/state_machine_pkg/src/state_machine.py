import rospy
import smach
import smach_ros
import time
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from std_msgs.msg import String
import math

# define state Start
class Start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Starting', 'Finished_Start'])
        self.cur_state_1_z = 2
        self.cur_state_2_z = 2

    def get_cur_state_1(self, cur_state):
        self.cur_state_1_z = cur_state.pose.pose.position.z

    def get_cur_state_2(self, cur_state):
        self.cur_state_2_z = cur_state.pose.pose.position.z

    def execute(self, userdata):
        # Check if the drones are ready
        rospy.Subscriber("/Quadrotor_1/current_state_est", Odometry, self.get_cur_state_1)
        rospy.Subscriber("/Quadrotor_2/current_state_est", Odometry, self.get_cur_state_2)
        pub_1 = rospy.Publisher("/Quadrotor_1/state_trigger", String, queue_size = 1)
        pub_2 = rospy.Publisher("/Quadrotor_2/state_trigger", String, queue_size = 1)
        rospy.loginfo('Start two drones')
        if self.cur_state_1_z > 0.2 or self.cur_state_2_z > 0.2:
            state_trigger = "Starting"
            pub_1.publish(state_trigger)
            pub_2.publish(state_trigger)
            return 'Starting'
        else:
            time.sleep(1)
            return 'Finished_Start'


# define state Take_off
class Take_off_1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Taking_off','Finished_Take_off'])
        self.cur_state_z = 0
        self.des_state_z = 2.0

    def get_cur_state(self, cur_state):
        self.cur_state_z = cur_state.pose.pose.position.z

    def get_des_state(self, des_state):
        self.des_state_z = des_state.transforms[0].translation.z


    def execute(self, userdata):
        rospy.Subscriber("/Quadrotor_1/current_state_est", Odometry, self.get_cur_state)
        rospy.Subscriber("/Quadrotor_1/desired_state", MultiDOFJointTrajectoryPoint, self.get_des_state)
        pub = rospy.Publisher("/Quadrotor_1/state_trigger", String, queue_size = 1)
        rate = rospy.Rate(500)
        if abs(self.cur_state_z - self.des_state_z) > 0.2:
            state_trigger = "Processing Take_off"
            rospy.loginfo(state_trigger)
            pub.publish(state_trigger)
            rate.sleep()
            return 'Taking_off'
        else:
            rate.sleep()
            take_off_trigger = "Processing Scan"
            rospy.loginfo(take_off_trigger)
            pub.publish(take_off_trigger)
            rate.sleep()
            return 'Finished_Take_off'


# define state Scan
class Scan_1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Scanning', 'Finished_Scan'])
        self.stop_flag = False

    def stop_callback(self, msg):
        if msg.data == "Stop":
            self.stop_flag = True

    def execute(self, userdata):
        rospy.loginfo('Executing state SCAN')
        rospy.Subscriber("stop_flag_1", String, self.stop_callback)
        pub = rospy.Publisher("/Quadrotor_1/state_trigger", String, queue_size = 1)
        rate = rospy.Rate(500)
        if not self.stop_flag:
            state_trigger = "Processing Scan"
            rospy.loginfo(state_trigger)
            pub.publish(state_trigger)
            rate.sleep()
            return 'Scanning'
        else:
            time.sleep(1)
            return 'Finished_Scan'


# define state Back_1
class Back_1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Flying_back', 'Arrived'])
        self.arrived_flag = False

    def arrived_callback(self, cur_state):
        if math.sqrt(math.pow(cur_state.pose.pose.position.x - 2, 2) + math.pow(cur_state.pose.pose.position.y + 2, 2)) < 0.5:
            self.arrived_flag = True

    def execute(self, userdata):
        pub = rospy.Publisher("/Quadrotor_1/state_trigger", String, queue_size = 1)
        rospy.Subscriber("/Quadrotor_1/current_state_est", Odometry, self.arrived_callback)
        rate = rospy.Rate(500)
        if not self.arrived_flag:
            pub.publish("Processing Flying_back")
            return 'Flying_back'
        else:
            rate.sleep()
            time.sleep(2)
            return 'Arrived'

# define state Land_1
class Land_1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Landing', 'Landed'])
        self.landed_flag = False

    def arrived_callback(self, cur_state):
        if abs(cur_state.pose.pose.position.z - 0.16) < 0.05:
            self.landed_flag = True

    def execute(self, userdata):
        pub = rospy.Publisher("/Quadrotor_1/state_trigger", String, queue_size = 1)
        rospy.Subscriber("/Quadrotor_1/current_state_est", Odometry, self.arrived_callback)
        rate = rospy.Rate(500)
        if not self.landed_flag:
            pub.publish("Processing Landing")
            return 'Landing'
        else:
            pub.publish("Finished")
            return 'Landed'
        

        
        


# define state Take_off
class Take_off_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Taking_off','Finished_Take_off'])
        self.cur_state_z = 0
        self.des_state_z = 2.0

    def get_cur_state(self, cur_state):
        self.cur_state_z = cur_state.pose.pose.position.z

    def get_des_state(self, des_state):
        self.des_state_z = des_state.transforms[0].translation.z


    def execute(self, userdata):
        rospy.Subscriber("/Quadrotor_2/current_state_est", Odometry, self.get_cur_state)
        rospy.Subscriber("/Quadrotor_2/desired_state", MultiDOFJointTrajectoryPoint, self.get_des_state)
        pub = rospy.Publisher("/Quadrotor_2/state_trigger", String, queue_size = 1)
        rate = rospy.Rate(500)
        if abs(self.cur_state_z - self.des_state_z) > 0.2:
            state_trigger = "Processing Take_off"
            rospy.loginfo(state_trigger)
            pub.publish(state_trigger)
            rate.sleep()
            return 'Taking_off'
        else:
            rate.sleep()
            take_off_trigger = "Processing Scan"
            rospy.loginfo(take_off_trigger)
            pub.publish(take_off_trigger)
            rate.sleep()
            return 'Finished_Take_off'


# define state Scan
class Scan_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Scanning', 'Finished_Scan'])
        self.stop_flag = False

    def stop_callback(self, msg):
        if msg.data == "Stop":
            self.stop_flag = True

    def execute(self, userdata):
        rospy.loginfo('Executing state SCAN')
        rospy.Subscriber("stop_flag_2", String, self.stop_callback)
        pub = rospy.Publisher("/Quadrotor_2/state_trigger", String, queue_size = 1)
        rate = rospy.Rate(500)
        if not self.stop_flag:
            state_trigger = "Processing Scan"
            rospy.loginfo(state_trigger)
            pub.publish(state_trigger)
            rate.sleep()
            return 'Scanning'
        else:
            time.sleep(1)
            return 'Finished_Scan'


# define state Back_2
class Back_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Flying_back', 'Arrived'])
        self.arrived_flag = False

    def arrived_callback(self, cur_state):
        if math.sqrt(math.pow(cur_state.pose.pose.position.x - 2, 2) + math.pow(cur_state.pose.pose.position.y - 2, 2)) < 0.5:
            self.arrived_flag = True

    def execute(self, userdata):
        pub = rospy.Publisher("/Quadrotor_2/state_trigger", String, queue_size = 1)
        rospy.Subscriber("/Quadrotor_2/current_state_est", Odometry, self.arrived_callback)
        rate = rospy.Rate(500)
        if not self.arrived_flag:
            pub.publish("Processing Flying_back")
            return 'Flying_back'
        else:
            rate.sleep()
            time.sleep(2)
            return 'Arrived'

# define state Land_2
class Land_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['Landing', 'Landed'])
        self.landed_flag = False

    def arrived_callback(self, cur_state):
        if abs(cur_state.pose.pose.position.z - 0.16) < 0.05:
            self.landed_flag = True

    def execute(self, userdata):
        pub = rospy.Publisher("/Quadrotor_2/state_trigger", String, queue_size = 1)
        rospy.Subscriber("/Quadrotor_2/current_state_est", Odometry, self.arrived_callback)
        rate = rospy.Rate(500)
        if not self.landed_flag:
            pub.publish("Processing Landing")
            return 'Landing'
        else:
            pub.publish("Finished")
            return 'Landed'


        


def main():
    rospy.init_node('smach_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes = ['Finished'])

    # Open the container sm_top
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('Start', Start(), transitions = {'Starting': 'Start',
                                                                'Finished_Start': 'CON'})

        # Create the sub SMACH state machine
        sm_con = smach.Concurrence(outcomes = ['Working', 'Finished_Work_both'],
                                   default_outcome = 'Working',
                                   outcome_map = {'Finished_Work_both':
                                        { 'Quadrotor_1': 'Finished_Work',
                                          'Quadrotor_2': 'Finished_Work'}})
        with sm_con:

            # Create a container for Quadrotor_1
            sm_quad_1 = smach.StateMachine(outcomes = ['Finished_Work'])

            # Open container sm_quad_1
            with sm_quad_1:

                # Add states to the container
                smach.StateMachine.add('Take_off', Take_off_1(),
                                       transitions = {'Taking_off': 'Take_off',
                                                      'Finished_Take_off': 'Scan'})
                smach.StateMachine.add('Scan', Scan_1(),
                                       transitions = {'Scanning': 'Scan',
                                                      'Finished_Scan': 'Back'})
                smach.StateMachine.add('Back', Back_1(),
                                       transitions = {'Flying_back': 'Back',
                                                      'Arrived': 'Land'})
                smach.StateMachine.add('Land', Land_1(),
                                       transitions = {'Landing': 'Land',
                                                      'Landed': 'Finished_Work'})

            # Add sm_quad to sm_con
            smach.Concurrence.add('Quadrotor_1', sm_quad_1)


            # Create a container for Quadrotor_2
            sm_quad_2 = smach.StateMachine(outcomes = ['Finished_Work'])

            # Open container sm_quad_2
            with sm_quad_2:

                # Add states to the container
                smach.StateMachine.add('Take_off', Take_off_2(),
                                       transitions = {'Taking_off': 'Take_off',
                                                      'Finished_Take_off': 'Scan'})
                smach.StateMachine.add('Scan', Scan_2(),
                                       transitions = {'Scanning': 'Scan',
                                                      'Finished_Scan': 'Back'})
                smach.StateMachine.add('Back', Back_2(),
                                       transitions = {'Flying_back': 'Back',
                                                      'Arrived': 'Land'})
                smach.StateMachine.add('Land', Land_2(),
                                       transitions = {'Landing': 'Land',
                                                      'Landed': 'Finished_Work'})

            # Add sm_quad to sm_con
            smach.Concurrence.add('Quadrotor_2', sm_quad_2)

        smach.StateMachine.add('CON', sm_con,
                               transitions = {'Working': 'CON',
                                              'Finished_Work_both': 'Finished'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_smach', sm_top, '/SM_ROOT')
    sis.start()


    # Execute SMACH plan
    outcome = sm_top.execute()


    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()