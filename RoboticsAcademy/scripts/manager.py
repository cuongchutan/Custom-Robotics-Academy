#!/usr/bin/python3
import sys
import asyncio
import websockets
import os
import threading
import time
import json

print(sys.path)
GAZEBO_RESOURCE_PATH = "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo11:$GAZEBO_RESOURCE_PATH:"

instructions = {
    "follow_line": {
        "gazebo_path": "~/jderobot_ws/src/RoboticsAcademy/exercises/follow_line/web-template/launch",
        "instructions_ros": ["/opt/ros/noetic/bin/roslaunch ~/jderobot_ws/src/RoboticsAcademy/exercises/follow_line/web-template/launch/simple_line_follower_ros_headless.launch"],
        "instructions_host": "python3 ~/jderobot_ws/src/RoboticsAcademy/exercises/follow_line/web-template/host.py 0.0.0.0"
    },
    "obstacle_avoidance": {
        "gazebo_path": "~/jderobot_ws/src/RoboticsAcademy/exercises/obstacle_avoidance/web-template/launch",
        "instructions_ros": ["/opt/ros/noetic/bin/roslaunch ~/jderobot_ws/src/RoboticsAcademy/exercises/obstacle_avoidance/web-template/launch/obstacle_avoidance_f1_headless.launch"],
        "instructions_host": "python3 ~/jderobot_ws/src/RoboticsAcademy/exercises/obstacle_avoidance/web-template/host.py 0.0.0.0"
    },
    "vacuum_cleaner": {
        "gazebo_path": "~/jderobot_ws/src/RoboticsAcademy/exercises/vacuum_cleaner/web-template/launch",
        "instructions_ros": ["/opt/ros/noetic/bin/roslaunch ~/jderobot_ws/src/RoboticsAcademy/exercises/vacuum_cleaner/web-template/launch/vacuum_cleaner_headless.launch"],
        "instructions_host": "python3 ~/jderobot_ws/src/RoboticsAcademy/exercises/vacuum_cleaner/web-template/host.py 0.0.0.0"
    },
    "vacuum_cleaner_loc": {
        "gazebo_path": "~/jderobot_ws/src/RoboticsAcademy/exercises/vacuum_cleaner_loc/web-template/launch",
        "instructions_ros": ["/opt/ros/noetic/bin/roslaunch ~/jderobot_ws/src/RoboticsAcademy/exercises/vacuum_cleaner_loc/web-template/launch/vacuum_cleaner_headless.launch"],
        "instructions_host": "python3 ~/jderobot_ws/src/RoboticsAcademy/exercises/vacuum_cleaner_loc/web-template/host.py 0.0.0.0"
    },
}


def export_gazebo(exercise):
    gazebo_path = GAZEBO_RESOURCE_PATH + instructions[exercise]["gazebo_path"] + ";"
    return gazebo_path


def ros_instructions(exercise):
    roslaunch_cmd = '/bin/sh -c "export PWD="/"; \
                    chmod +rwx /; \
                    export DISPLAY=:0; \
                    export OLDPWD=/etc/ros/rosdep; \
                    cd /; \
                    export LD_LIBRARY_PATH=/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins; \
                    export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH; \
                    export GAZEBO_MODEL_DATABASE_URI=http://gazebosim.org/models; \
                    export ROS_DISTRO=noetic; \
                    export PKG_CONFIG_PATH=/opt/ros/noetic/lib/pkgconfig; \
                    export OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.9.0; \
                    export SHLVL=1; \
                    export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:; \
                    export TERM=xterm; \
                    export ROS_VERSION=1; \
                    export GAZEBO_MASTER_URI=http://localhost:11345;ROS_ETC_DIR=/opt/ros/noetic/etc/ros; \
                    export CMAKE_PREFIX_PATH=/opt/ros/noetic; \
                    export ROS_PACKAGE_PATH=/opt/ros/noetic/share; \
                    chmod +x /opt/ros/noetic/bin/rosmaster; \
                    export PYTHONPATH=/opt/ros/noetic/lib/python3.8/dist-packages; \
                    chmod +x /opt/ros/noetic/bin/roslaunch; \
                    cd; \
                    export ROS_ROOT=/opt/ros/noetic/share/ros; \
                    export GAZEBO_RESOURCE_PATH=/usr/share/gazebo11:$GAZEBO_RESOURCE_PATH; \
                    export ROS_MASTER_URI=http://localhost:11311; \
                    export PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin;'
    roslaunch_cmd = roslaunch_cmd + export_gazebo(exercise)
    for instruction in instructions[exercise]["instructions_ros"]:
        roslaunch_cmd = roslaunch_cmd + instruction + ";"
    roslaunch_cmd = roslaunch_cmd + '"'
    return roslaunch_cmd


async def kill_simulation():
    cmd_gzweb = "pkill -9 -f host.py"
    os.popen(cmd_gzweb)
    cmd_host = "pkill -9 -f node"
    os.popen(cmd_host)
    cmd_host = "pkill -9 -f gzserver"
    os.popen(cmd_host)
    cmd_ros = "pkill -9 -f roslaunch"
    os.popen(cmd_ros)
    cmd_rosout = "pkill -9 -f rosout"
    os.popen(cmd_rosout)
    cmd_mel = "pkill -9 -f melodroot"
    os.popen(cmd_mel)
    cmd_rosout = "pkill -9 -f rosout"
    os.popen(cmd_rosout)
    """cmd_py = "pkill -9 -f python"
    os.popen(cmd_py)"""


class DockerThread(threading.Thread):
    def __init__(self, cmd):
        threading.Thread.__init__(self)
        self.cmd = cmd

    def run(self):
        stream = os.popen(self.cmd)
        out = stream.read()
        print(out)

    
async def hello(websocket, path):
    # name = await websocket.recv()
    print(websocket)
    async for name in websocket:
        print(name)
        data = json.loads(name)
        command = data["command"]
        if command == "open":
            print("> Starting simulation")
            xvfb_cmd = "/usr/bin/Xorg -noreset +extension GLX +extension RANDR +extension RENDER -logfile ./xdummy.log -config ./xorg.conf :0"
            xvfb_thread = DockerThread(xvfb_cmd)
            xvfb_thread.start()
            roslaunch_cmd = ros_instructions(data["exercise"])
            roslaunch_thread = DockerThread(roslaunch_cmd)
            roslaunch_thread.start()
            time.sleep(5)
            gzweb_cmd = 'cd /gzweb; npm start -p 8080'
            gzweb_thread = DockerThread(gzweb_cmd)
            gzweb_thread.start()
            host_cmd = instructions[data["exercise"]]["instructions_host"]
            host_thread = DockerThread(host_cmd)
            host_thread.start()
        elif command == "resume":
            print("RESUME SIMULATIOn")
            cmd = "/opt/ros/noetic/bin/rosservice call gazebo/unpause_physics"
            rosservice_thread = DockerThread(cmd)
            rosservice_thread.start()
        elif command == "stop":
            print("STOP SIMULATIOn")
            cmd = "/opt/ros/noetic/bin/rosservice call gazebo/pause_physics"
            rosservice_thread = DockerThread(cmd)
            rosservice_thread.start()
        elif command == "start":
            cmd = "/opt/ros/noetic/bin/rosservice call gazebo/unpause_physics"
            rosservice_thread = DockerThread(cmd)
            rosservice_thread.start()
        elif command == "reset":
            cmd = "/opt/ros/noetic/bin/rosservice call gazebo/reset_simulation"
            rosservice_thread = DockerThread(cmd)
            rosservice_thread.start()
        else:
            print("ALL KILED")
            await kill_simulation()

        greeting = f"Hello {name}!"

        await websocket.send("Done")
        # print(f"> {greeting}")

start_server = websockets.serve(hello, "0.0.0.0", 8765)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()