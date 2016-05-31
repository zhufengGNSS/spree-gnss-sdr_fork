#!/bin/bash
 sed -i '/Tracking_GPS.implementation=GPS_L1_CA_DLL_PLL_Tracking/c\Tracking_GPS.implementation=GPS_L1_CA_DLL_PLL_EC_Tracking' loc*.conf
