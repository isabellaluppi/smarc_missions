#!/usr/bin/env python

import rospy
import actionlib
from smarc_msgs.msg import SMTask, StringArray
from smarc_msgs.srv import AddTask, AddTaskRequest, AddTaskResponse
from smarc_msgs import sm_task_utils
from geometry_msgs.msg import PoseStamped
import csv
import sys
import os

def mission_tasks(mission_file):
    """ 
    Create an example of a task which we'll copy for other tasks later.
    This is a good example of creating a task with a variety of arguments.
    """


    if not os.path.isfile(mission_file):
        rospy.logerr("No such file: %s", mission_file)
        sys.exit(-1)

    tasks = []
    with open(mission_file) as csvfile:
        spamreader = csv.reader(csvfile, delimiter=' ', quotechar='"')
        for row in spamreader:
            rospy.loginfo("Got entry: %s", " ".join(row))
            pose = PoseStamped()
            pose.header.frame_id = "/world"
            pose.pose.position.x = float(row[3])
            pose.pose.position.y = float(row[4])
            pose.pose.position.z = -float(row[2])
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
    
            master_task = SMTask(action_topic=row[6])
            #sm_task_utils.add_pose_stamped_argument(master_task, pose_stamped)
            sm_task_utils.add_duration_argument(master_task, float(row[7]))
            master_task.x = float(row[3])
            master_task.y = float(row[4])
            master_task.depth = float(row[2])
            master_task.altitude = float(row[1])
            master_task.task_id = int(row[0])
            master_task.theta = float(row[5])
            master_task.action_arguments.append(StringArray())
            for arg in row[8][1:-1].split(','):
                master_task.action_arguments[0].string_array.append(arg.strip()) 

            tasks.append(master_task)
            print master_task

    return tasks

if __name__ == '__main__':

    rospy.init_node("task_routine", log_level=rospy.INFO)
 
    mission_file = rospy.get_param('~mission_file', "mission.csv")
    rospy.wait_for_service('/task_executor/add_state')

    tasks = mission_tasks(mission_file)
    for task in tasks:
        try:
            add_task = rospy.ServiceProxy('/task_executor/add_state', AddTask)
            add_task_res = add_task(task)        
            print "response %s" %add_task_res.task_id
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    rospy.loginfo("Successfully added %d tasks, exiting...", len(tasks))
