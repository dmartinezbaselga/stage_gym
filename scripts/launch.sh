#!/bin/bash

# [ -n "$1" ] && [ "$1" -eq "$1" ] 2>/dev/null
# if [ $? -ne 0 ]; then
#    echo "Usage: $0 number_episodes [-p n_pasives n_actives | -s scenario | -l | -qTable <file> | -t | -a <alpha> | -g <gamma> | -e <epsilon> | -d <epsilon_discount> ]"
#    exit -1
# fi

g++ start_agents.cpp -o start_agents -Wall -O2 -Werror -pedantic -Werror  -std=c++14

main_agent_name=active_agent_multi
pidDQN=0
./start_agents $@

launch_file=stage.launch
rm -rf /home/diego/.ros/log
roslaunch stage_ros_dovs $launch_file 
# for i in $(seq $1)