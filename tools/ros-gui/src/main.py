import PySimpleGUI as sg
import subprocess
import shlex

layout = [
    [sg.Text("Command")], 
    [sg.Input()], 
    [sg.Button('Send')]
]

# Create the window
window = sg.Window('Window Title', layout)      # Part 3 - Window Defintion

# Display and interact with the Window
event, values = window.read()                   # Part 4 - Event loop or Window.read call

cmd = "docker exec zed bash -c 'source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash && {}'".format(values[0]).split()
cmd = shlex.split("docker exec zed bash -c 'source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash && {}'".format(values[0]))
print(cmd)

p = subprocess.run(cmd, capture_output=True)

# p = subprocess.Popen(cmd, stdout=subprocess.PIPE)
# out, err = p.communicate()
print(p.stdout)


# Finish up by removing from the screen
window.close()                                  # Part 5 - Close the Window