#!/bin/bash
 sed -i '/Channels.in_acquisition = 2/c\Channels.in_acquisition = 5' texbat_ds4.conf
 sed -i '/Channels_GPS.count = 2/c\Channels_GPS.count = 5' texbat_ds4.conf
