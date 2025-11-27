#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, os

from std_msgs.msg import Float32MultiArray
from morai_msgs.msg import GPSMessage



class GPS_to_SIM:
    def __init__(self):
        rospy.init_node('GPS_to_SIM', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.gps_callback)

        self.utm_msg = Float32MultiArray()
        self.is_gps_data = False

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            os.system('clear')
            #if not self.is_gps_data:
            #    print("[1] can't subscribe '/gps' topic... \n    please check your GPS sensor connection")

            self.is_gps_data = False
            rate.sleep()

    def convert_gps_to_sim(self, lat, lon):
        x = (1.74366642 * lat) + (111705.27046915 * lon) + (2.74602510)
        y = (110234.58048352 * lat) + (-23.11916045 * lon) + (-4.08750102)
        
        return x, y


    def gps_callback(self, gps_msg):
        self.is_gps_data = True
        latitude = gps_msg.latitude
        longitude = gps_msg.longitude
        altitude = gps_msg.altitude
        sim_x, sim_y = self.convert_gps_to_sim(latitude, longitude)

        map_x = sim_x
        map_y = sim_y
        
        os.system('clear')
        print(f''' 
        ----------------[ GPS data ]----------------
            latitude    : {latitude}
            longitude   : {longitude}
            altitude    : {altitude}

                             |
                             | apply Projection
                             V

        ------------------[ SIM ]-------------------
              sim_x     : {sim_x}
              sim_y     : {sim_y}

                             |
                             |
                             V
              
        ------------------[ MAP ]-------------------
              map_x     : {map_x}
              map_y     : {map_y}
        ''')



if __name__ == '__main__':
    try:
        GPS_to_UTM = GPS_to_SIM()
    except rospy.ROSInterruptException:
        pass