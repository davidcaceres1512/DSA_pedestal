#!/bin/bash

azimuth_speed=1.0
elevation_pos=0.0
base_url=http://10.10.10.27/
speed_url=${base_url}speed?params=
position_url=${base_url}position?params=

temp_json={"axis":"azimuth","speed":0.0}
temp_base64=`echo -n $temp_json | base64`
curl ${speed_url}${temp_base64}
sleep 1
temp_json={"axis":"elevation","speed":0.0}
temp_base64=`echo -n $temp_json | base64`
curl ${speed_url}${temp_base64}
sleep 1
temp_json={"axis":"azimuth","position":0.0}
temp_base64=`echo -n $temp_json | base64`
curl ${position_url}${temp_base64}
sleep 1
temp_json={"axis":"elevation","position":${elevation_pos}}
temp_base64=`echo -n $temp_json | base64`
curl ${position_url}${temp_base64}
sleep 1
temp_json={"axis":"azimuth","speed":${azimuth_speed}}
temp_base64=`echo -n $temp_json | base64`
curl ${speed_url}${temp_base64}
sleep 1
