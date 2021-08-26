import sys
import copy
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandGoal
import actionlib




duration=5.0
name= "gripper_controller"
joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
client = actionlib.SimpleActionClient("%s/gripper_action" % name,
                                                   GripperCommandAction )
rospy.loginfo("Waiting for %s..." % name)
client.wait_for_server()

gripper_goal = GripperCommandGoal()
print(gripper_goal)
gripper_goal.command.position = 0.01
gripper_goal.command.max_effort = 0.5
#gripper_goal.stalled = False
#gripper_goal.reached_goal= False
client.send_goal(gripper_goal)
client.wait_for_result()
#for position in range(positions):

# trajectory = JointTrajectory()
# trajectory.joint_names = joints
# trajectory.points.append(JointTrajectoryPoint())
# trajectory.points[0].positions = pose
# trajectory.points[0].velocities = [0.0 for _ in pose]
# trajectory.points[0].accelerations = [0.0 for _ in pose]
# trajectory.points[0].time_from_start = rospy.Duration(duration)
# follow_goal = FollowJointTrajectoryGoal()
# follow_goal.trajectory = trajectory
# client.send_goal(follow_goal)
# client.wait_for_result()




