from flask import Flask, Response, jsonify, request
import subprocess
import time
import os
import signal


app = Flask(__name__)
process = None
state = "Idle"

arwain_env = {
    "PYTHONPATH":"/opt/intel/openvino/python/python3.7:/opt/intel/openvino/python/python3:/opt/intel/openvino/deployment_tools/model_optimizer:/opt/ros/melodic/lib/python2.7/dist-packages:/opt/intel/openvino/python/python3.7:/opt/intel/openvino/python/python3:/opt/intel/openvino/deployment_tools/model_optimizer",
    "InferenceEngine_DIR":"/opt/intel/openvino/deployment_tools/inference_engine/share",
    "INTEL_OPENVINO_DIR":"/opt/intel/openvino",
    "OpenCV_DIR":"/opt/intel/openvino/opencv/cmake",
    "LD_LIBRARY_PATH":"/opt/intel/openvino/opencv/lib:/opt/intel/openvino/deployment_tools/ngraph/lib:/opt/intel/opencl:/opt/intel/openvino/deployment_tools/inference_engine/external/hddl/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/gna/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/mkltiny_lnx/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/tbb/lib:/opt/intel/openvino/deployment_tools/inference_engine/lib/armv7l:/opt/ros/melodic/lib:/opt/intel/openvino/opencv/lib:/opt/intel/openvino/deployment_tools/ngraph/lib:/opt/intel/opencl:/opt/intel/openvino/deployment_tools/inference_engine/external/hddl/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/gna/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/mkltiny_lnx/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/tbb/lib:/opt/intel/openvino/deployment_tools/inference_engine/lib/armv7l",
    "PATH":"/opt/intel/openvino/deployment_tools/model_optimizer:/opt/ros/melodic/bin:/home/pi/.vscode-server/bin/ccbaa2d27e38e5afa3e5c21c1c7bef4657064247/bin:/home/pi/.local/bin:/opt/intel/openvino/deployment_tools/model_optimizer:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games",
}

env = {
    "SHELL":"/bin/bash",
    "ROS_VERSION":"1",
    "COLORTERM":"truecolor",
    "TERM_PROGRAM_VERSION":"1.62.3",
    "PKG_CONFIG_PATH":"/opt/ros/melodic/lib/pkgconfig",
    "ROS_PYTHON_VERSION":"2",
    "NO_AT_BRIDGE":"1",
    "ROS_PACKAGE_PATH":"/opt/ros/melodic/share",
    "ROSLISP_PACKAGE_DIRECTORIES":"",
    "PWD":"/home/pi/arwain_inference_core",
    "LOGNAME":"pi",
    "XDG_SESSION_TYPE":"tty",
    "InferenceEngine_DIR":"/opt/intel/openvino/deployment_tools/inference_engine/share",
    "HOME":"/home/pi",
    "LANG":"en_GB.UTF-8",
    "ROS_ETC_DIR":"/opt/ros/melodic/etc/ros",
    "INTEL_OPENVINO_DIR":"/opt/intel/openvino",
    "CMAKE_PREFIX_PATH":"/opt/ros/melodic",
    "OpenCV_DIR":"/opt/intel/openvino/opencv/cmake",
    "XDG_SESSION_CLASS":"user",
    "PYTHONPATH":"/opt/intel/openvino/python/python3.7:/opt/intel/openvino/python/python3:/opt/intel/openvino/deployment_tools/model_optimizer:/opt/ros/melodic/lib/python2.7/dist-packages:/opt/intel/openvino/python/python3.7:/opt/intel/openvino/python/python3:/opt/intel/openvino/deployment_tools/model_optimizer",
    "TERM":"xterm-256color",
    "USER":"pi",
    "SHLVL":"1",
    "ROS_MASTER_URI":"http://localhost:11311",
    "XDG_SESSION_ID":"c2",
    "LD_LIBRARY_PATH":"/opt/intel/openvino/opencv/lib:/opt/intel/openvino/deployment_tools/ngraph/lib:/opt/intel/opencl:/opt/intel/openvino/deployment_tools/inference_engine/external/hddl/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/gna/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/mkltiny_lnx/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/tbb/lib:/opt/intel/openvino/deployment_tools/inference_engine/lib/armv7l:/opt/ros/melodic/lib:/opt/intel/openvino/opencv/lib:/opt/intel/openvino/deployment_tools/ngraph/lib:/opt/intel/opencl:/opt/intel/openvino/deployment_tools/inference_engine/external/hddl/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/gna/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/mkltiny_lnx/lib:/opt/intel/openvino/deployment_tools/inference_engine/external/tbb/lib:/opt/intel/openvino/deployment_tools/inference_engine/lib/armv7l",
    "XDG_RUNTIME_DIR":"/run/user/1000",
    "SSH_CLIENT":"192.168.2.101 52633 22",
    "PATH":"/opt/intel/openvino/deployment_tools/model_optimizer:/opt/ros/melodic/bin:/home/pi/.vscode-server/bin/ccbaa2d27e38e5afa3e5c21c1c7bef4657064247/bin:/home/pi/.local/bin:/opt/intel/openvino/deployment_tools/model_optimizer:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/local/games:/usr/games",
    "DBUS_SESSION_BUS_ADDRESS":"unix:path=/run/user/1000/bus",
    "MAIL":"/var/mail/pi",
    "HDDL_INSTALL_DIR":"/opt/intel/openvino/deployment_tools/inference_engine/external/hddl",
    "ROS_ROOT":"/opt/ros/melodic/share/ros",
    "ROS_DISTRO":"melodic",
    "INTEL_CVSDK_DIR":"/opt/intel/openvino",
    "TEXTDOMAIN":"Linux-PAM",
    "_":"/usr/bin/env",
}


@app.route("/info")
def info():
    global process, state
    if process is None or process.poll() is not None:
        state = "Idle"
        process = None
    response = jsonify({'action': "info", "result": state})

    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/calibrate_gyroscopes")
def calibrate_gyroscopes():
    global process, state
    response_dict = {"action": "start"}
    process = subprocess.Popen(["./build/arwain", "-conf", "arwain.conf", "-calibg"], shell=False, env=arwain_env)
    state = "Calibrating gyroscopes"
    response_dict["result"] = "Started calibration"
    response = jsonify(response_dict)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/start", methods=["GET"])
def start():
    global process, state
    response_dict = {"action": "start"}

    ## Are we already running?
    if process is not None:
        response_dict["result"] = "Failed, already running"

    else:
        cmd = ["./build/arwain", "-lfile", "-conf", "arwain.conf"]
        fname = request.args.get("fname")
        if fname is not None:
            cmd.append("-name")
            cmd.append(fname)
        print(cmd)
        state = "Performing inference"
        process = subprocess.Popen(cmd, shell=False, env=arwain_env)
        
        response_dict["result"] = f"Started with pid {process.pid}"

    response = jsonify(response_dict)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/stop")
def stop():
    global process

    response_dict = {"action":"stop", "result": "Stopped"}

    if process is not None:
        ## Try to stop
        os.kill(process.pid, signal.SIGINT)

        for _ in range(5):
            if process.poll() is None:
                time.sleep(1.0)

        subprocess.Popen(["pkill", "-f", "arwain"])
        subprocess.Popen(["pkill", "-f", "ncs2_interface.py"])

    process = None

    response = jsonify(response_dict)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/shutdown")
def shutdown():
    subprocess.Popen(["sudo", "shutdown", "now"])
    response_dict = dict()
    response = jsonify(response_dict)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/kill")
def kill():
    subprocess.Popen(["pkill", "-f", "arwain"])
    subprocess.Popen(["pkill", "-f", "ncs2"])
    response = jsonify({'action': f"kill", "result": "killed"})
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8051, debug=False)
