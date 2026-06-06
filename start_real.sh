#!/bin/bash

SESSION="px4_sim"

# Kill old session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# Create session
tmux new-session -d -s $SESSION

# Create 4 panes
tmux split-window -h -t $SESSION:0
tmux split-window -v -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.1

# Pane 0: Offboard control
tmux send-keys -t $SESSION:0.0 \
"ssh firedrone@10.42.0.1 'source ~/ws_offboard_control/install/setup.bash && \ 
ros2 launch px4_ros_com offboard_control_drone.launch.py'" C-m

sleep 10

# Pane 1: Keyboard control
tmux send-keys -t $SESSION:0.1 \
"source ~/ws_offboard_control/install/setup.bash && \
ros2 run px4_ros_com control_keyboard.py" C-m

# Pane 2: SLAM
tmux send-keys -t $SESSION:0.2 \
"source ~/ws_offboard_control/install/setup.bash && \
ros2 launch px4_ros_com offboard_control_slam.launch.py use_sim:=False" C-m

# Pane 3: GCS
tmux send-keys -t $SESSION:0.3 \
"source ~/ws_offboard_control/install/setup.bash && \
ros2 launch px4_ros_com offboard_control_gcs.launch.py" C-m

# Arrange nicely
tmux select-layout -t $SESSION tiled

# Attach
tmux attach -t $SESSION