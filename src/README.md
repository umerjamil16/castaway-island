goal_service_server.py launches a service called /get_goal (run this file before proceeding, also set the pos of robot_0 using tele_op keyboard package first)

main.py publishes a tf GF_01, based on data recieve from /get_goal service 

roslaunch ca_main goal_gen_tf_br.launch launches the above two files i.e main.py and goal_service_server.py

robot_1_send.py used t0 send 2D pose goal to robot_1

q_e.py for debugging goal_service_server.py, copied file