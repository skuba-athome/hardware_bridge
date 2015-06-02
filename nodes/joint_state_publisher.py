#! /usr/bin/python
import rospy
from dynamixel_msgs.msg import JointState


class JointStatePublisher:
    def __init__(self, state_subcribe_topic, state_publisher_topic):
        self.publisher = rospy.Publisher(state_publisher_topic, JointState)
        rospy.Subscriber(state_subcribe_topic, JointState, self.state_callback)

    def state_callback(self, joint_state):
        self.publisher.publish(joint_state)


class LumyaiJointPublisher:
    def __init__(self):
        rospy.init_node('joint_states_publisher')

        motors_list = rospy.get_param('~motors_list')
        joint_state_publisher_topic = rospy.get_param('joint_state_topic', '/dynamixel/joint_states')

        publisher_list = []
        for motor in motors_list:
            joint_state_topic = '/dynamixel/' + motor + '/state'
            publisher = JointStatePublisher(joint_state_topic, joint_state_publisher_topic)
            publisher_list.append(publisher)
        rospy.spin()
        
if __name__ == '__main__':
    LumyaiJointPublisher()
