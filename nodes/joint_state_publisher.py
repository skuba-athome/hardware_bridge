#! /usr/bin/python
import rospy
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import JointState as SensorJointState
from include.urdf_parser_py.urdf import URDF


class JointStatesPublisher:
    def __init__(self):
        rospy.init_node('joint_states_publisher')

        motors_list = rospy.get_param('~motors_list')
        joint_state_publisher_topic = rospy.get_param('~joint_state_topic', '/dynamixel/joint_states')
        urdf_file_path = rospy.get_param('~urdf_file_path', '')

        self.joint_state = {}
        if len(urdf_file_path) == 0:
            robot = URDF.from_parameter_server()
        else:
            robot = URDF.from_xml_string(open(urdf_file_path,'r').read())

        for joint in robot.joints:
            joint_state = JointState()
            joint_state.name = joint.name
            joint_state.current_pos = 0
            self.joint_state[joint.name] = joint_state

        for motor in motors_list:
            joint_state_topic = '/dynamixel/' + motor + '/state'
            rospy.Subscriber(joint_state_topic, JointState, self.state_callback)

        self.publisher = rospy.Publisher(joint_state_publisher_topic, SensorJointState, queue_size=1)
        self.fix_publisher = rospy.Publisher('/joint_states', SensorJointState, queue_size=1)
    
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.publish_joint_states()
            rate.sleep()

    def state_callback(self, joint_state):
        self.joint_state[joint_state.name] = joint_state

    def publish_joint_states(self):
        joint_states = SensorJointState()
        joint_states.header.stamp = rospy.Time.now()

        for joint in self.joint_state.values():
            joint_states.name.append(joint.name)
            joint_states.position.append(joint.current_pos)
            #joint_states.velocity.append(joint.velocity)
            #joint_states.effort.append(joint.load)

        self.publisher.publish(joint_states)
        self.fix_publisher.publish(joint_states)
        
if __name__ == '__main__':
    JointStatesPublisher()
