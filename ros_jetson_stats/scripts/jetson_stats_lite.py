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
        # data.status[16] -> /jtop/Memory
        # data.status[17] -> /jtop/Memory/emc
        # data.status[18] -> /jtop/Memory/ram
        # data.status[19] -> /jtop/Memory/swap
        # data.status[20] -> /jtop/Power
        # data.status[21] -> /jtop/Power/power
        # data.status[22] -> /jtop/Temperatures
        # data.status[23] -> /jtop/Temperatures/temp
        # data.status[24] -> /jtop

        # Read out the GPU load
        if (len(data.status) > 23):            
            jetson_stats.ram_used_gb = 0.000001 * float(data.status[18].values[0].value)
            jetson_stats.ram_shared_gb = 0.0000001 * float(data.status[18].values[1].value)
            jetson_stats.ram_total_gb = 0.0000001 * float(data.status[18].values[2].value)
            jetson_stats.swap_used_gb = 0.001 * float(data.status[19].values[0].value)
            jetson_stats.swap_total_gb = 0.001 * float(data.status[19].values[1].value)
            jetson_stats.cpu_temp_celsius = float(data.status[23].values[6].value.rstrip("C"))
            jetson_stats.gpu_load_percent = 0.01 * float(data.status[14].values[0].value.rstrip("%"))
            jetson_stats.gpu_temp_celsius = float(data.status[23].values[4].value.rstrip("C"))
            self.pub_jetson_stats_lite.publish(jetson_stats)

    def __init__(self):
        print ("was inited")
        self.sub_jetson_diagnostics = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.jetson_diagnostics_callback)

        # Initialization jetson stats ros publisher (for plotteable output)
        self.pub_jetson_stats_lite = rospy.Publisher('/jetson_stats_lite', JetsonStats, queue_size=1)

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
