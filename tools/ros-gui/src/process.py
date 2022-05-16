import subprocess
import shlex

def execute(cmd):
    shlex.split("docker exec zed bash -c 'source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash && {}'".format(cmd))
    popen = subprocess.Popen(
        shlex.split("docker exec zed bash -c 'source /opt/ros/noetic/setup.bash && source ros_ws/devel/setup.bash && {}'".format(cmd)), 
        stdout=subprocess.PIPE,
        universal_newlines=True
    )
    for stdout_line in iter(popen.stdout.readline, ""):
        yield stdout_line
    popen.stdout.close()
    return_code = popen.wait()
    if return_code:
        raise subprocess.CalledProcessError(return_code, cmd)

for path in execute("roslaunch zed_launch zed2i.launch"):
    print(path, end="")