#!/bin/bash
sed -i '/Channels.in_acquisition = [0-9]/c\Channels.in_acquisition = 5' texbat_*.conf
sed -i '/Channels.in_acquisition = 12/c\Channels_GPS.count = 12' texbat_*.conf
