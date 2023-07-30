# ./topic_command_to_csv.sh
# ./topic_speed_to_csv.sh
# ./topic_time_to_csv.sh
./bag_it.sh
timeout 5.0 rostopic pub -r 100 /left_wheel/command/set  std_msgs/Float64 "data : 2.0"
timeout 5.0 rostopic pub -r 100 /left_wheel/command/set  std_msgs/Float64 "data : -2.0"
timeout 5.0 rostopic pub -r 100 /left_wheel/command/set  std_msgs/Float64 "data : 4.0"
timeout 5.0 rostopic pub -r 100 /left_wheel/command/set  std_msgs/Float64 "data : -4.0"
timeout 5.0 rostopic pub -r 100 /left_wheel/command/set  std_msgs/Float64 "data : 0.0"

