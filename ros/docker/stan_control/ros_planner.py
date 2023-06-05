from stan_path_planner import path_generation
import cuber as cb
import std_msgs.msg as std_msgs



ros = cb.ros()
make_path = path_generation()

# Define a callback function
def callback(msg):
    path_info = make_path.left_turn(msg)
    ros.publish({path_info})

    

ros.subscribe({rostopic_name}, {rostopic_type}, callback)



ros.init_publisher({'/path_info'}, {std_msgs.Float32MultiArray}, queue_size = 2)
# Publish the message

ros.publish({message})
