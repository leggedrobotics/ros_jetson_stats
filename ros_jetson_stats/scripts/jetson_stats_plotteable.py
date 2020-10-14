#!/usr/bin/env python

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from ros_jetson_stats.srv import nvpmodel, nvpmodelResponse, jetson_clocks, jetson_clocksResponse, fan, fanResponse
from datetime import timedelta
import jtop
from jetson_stats_msgs.msg import JetsonStats


class JetsonStatsLite:

    def jetson_diagnostics_callback(self, data):

        # New jetson stats message
        jetson_stats = JetsonStats()

        # Pass on the time stamp
        jetson_stats.header.stamp = data.header.stamp

        # Legend: 
        # data.status[0] -> /jtop/board
        # data.status[1] -> /jtop/board/config
        # data.status[2] -> /jtop/board/disk
        # data.status[3] -> /jtop/board/fan
        # data.status[4] -> /jtop/board/status
        # data.status[5] -> /jtop/CPU
        # data.status[6] -> /jtop/CPU/CPU1
        # data.status[7] -> /jtop/CPU/CPU2
        # data.status[8] -> /jtop/CPU/CPU3
        # data.status[9] -> /jtop/CPU/CPU4
        # data.status[10] -> /jtop/CPU/CPU5
        # data.status[11] -> /jtop/CPU/CPU6
        # data.status[12] -> /jtop/CPU/CPU7
        # data.status[13] -> /jtop/CPU/CPU8
        # data.status[14] -> /jtop/GPU
        # data.status[15] -> /jtop/GPU/gpu
        # data.status[16] ->
        # data.status[17] ->
        # data.status[18] ->


  
        # Read out the GPU load
        if (len(data.status) > 15):
            print (data.status[14].name)
            print (data.status[14].values[0].value)

            jetson_stats.gpu_load = 0.01 * float(data.status[14].values[0].value.rstrip("%"))
            self.pub_jetson_stats_lite.publish(jetson_stats)

    def __init__(self):
        print ("was inited")
        self.sub_jetson_diagnostics = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.jetson_diagnostics_callback)

        # Initialization jetson stats ros publisher (for plotteable output)
        self.pub_jetson_stats_lite = rospy.Publisher('/jetson_stats', JetsonStats, queue_size=1)

def jetson_stats_lite():
    # Init ros node
    rospy.init_node('jetson_stats_lite_node')

    # Init object
    jetsonStatsLite = JetsonStatsLite()

    # Spin ros
    rospy.spin()

if __name__ == '__main__':
    jetson_stats_lite()
# EOF
