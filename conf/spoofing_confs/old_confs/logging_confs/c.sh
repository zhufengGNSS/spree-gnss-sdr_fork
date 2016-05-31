#!/bin/bash
 sed -i '/Acquisition_GPS.implementation=GPS_L1_CA_PCPS_Acquisition/c\Acquisition_GPS.implementation=GPS_L1_CA_PCPS_SD_Acquisition' loc*.conf
 sed -i '/PVT.implementation=GPS_L1_CA_PVT/c\PVT.implementation=GPS_L1_CA_SD_PVT' loc*.conf
 sed -i '/TelemetryDecoder_GPS.implementation=GPS_L1_CA_Telemetry_Decoder/c\TelemetryDecoder_GPS.implementation=GPS_L1_CA_SD_Telemetry_Decoder' loc*.conf
