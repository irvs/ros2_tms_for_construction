#!/usr/bin/env python
# -*- coding:utf-8 -*-

# doubleをwhs1(心拍センサ)でコントロールする

import rospy
from tms_msg_db.srv import TmsdbGetData, TmsdbGetDataRequest
from geometry_msgs.msg import PoseStamped

import pymongo
import json

TMSDB_ID = 3021  # database request "id" for whs1
TMSDB_SENSOR = 3021  # database request "sensor" for whs1

def main():
    print "Double Whs1 Control"
    pub = rospy.Publisher("/tms_rc_double/room957/move_base_simple/goal",PoseStamped,queue_size=10)
    rospy.init_node('double_whs1_control')
    rospy.wait_for_service('/tms_db_reader')

    goal_pose_stamped = setGoalPoseStamped()
    is_published = False

    r = rospy.Rate(1)  # 1Hz
    while not rospy.is_shutdown():
        rate = getWhs1HeartRate()
        
        if is_published:
            print "[Warning]",
        print "Whs1 Heart Rate : " + str(rate)  # debug

        if (not is_published) and (rate > 140):
            pub.publish(goal_pose_stamped)
            print "\n[Warning] robot start.\n"
            is_published = True

        r.sleep()

def setGoalPoseStamped():
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "map"
    pose_stamped.pose.position.x = 3.1917
    pose_stamped.pose.position.y = 1.3222
    pose_stamped.pose.position.z = 0.0
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0

    return pose_stamped

def getWhs1HeartRate():
    """Whs1の心拍数を取得する

    tms_ss_whs1/src/main.cppは、tmsdbのnoteにjsonとして各種データを保存しているので、
    noteをjson.loadsで辞書型に変更して読みだす。
    """
    rate = -1  # Error Number
    db_req = TmsdbGetDataRequest()
    db_req.tmsdb.id = TMSDB_ID
    db_req.tmsdb.sensor = TMSDB_SENSOR
    try:
        srv_client = rospy.ServiceProxy("/tms_db_reader", TmsdbGetData)

        res = srv_client(db_req)
        if len(res.tmsdb) == 0:
            return rate
        note = res.tmsdb[0].note

        whs1_params = json.loads(note)
        rate = whs1_params["rate"]

    except rospy.ServiceException as e:
        print "Service call failed: %s" %e
    
    return rate

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass