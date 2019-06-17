#!/bin/bash
 sed -i '/Spoofing.ap_detection = true/c\Spoofing.ap_detection = false' loc*.conf
 sed -i '/Channels.in_acquisition = 10/c\Channels.in_acquisition = 5' loc*.conf
 sed -i '/Channels_GPS.count = 20/c\Channels_GPS.count = 12' loc*.conf
 sed -i '/Acquisition_GPS.implementation=GPS_L1_CA_PCPS_SD_Acquisition/c\Acquisition_GPS.implementation=GPS_L1_CA_PCPS_Acquisition' loc*.conf
 sed -i '/Tracking_GPS.implementation=GPS_L1_CA_DLL_PLL_EC_Tracking/c\Tracking_GPS.implementation=GPS_L1_CA_DLL_PLL_Tracking' loc*.conf
 sed -i '/PVT.implementation=GPS_L1_CA_SD_PVT/c\PVT.implementation=GPS_L1_CA_PVT' loc*.conf
 sed -i '/TelemetryDecoder_GPS.implementation=GPS_L1_CA_Telemetry_Decoder/c\TelemetryDecoder_GPS.implementation=GPS_L1_CA_SD_Telemetry_Decoder' loc*.conf
 sed -i '/Spoofing.inter_satellite_detection = true/c\Spoofing.inter_satellite_detection = false' loc*.conf

