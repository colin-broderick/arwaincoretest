from flask import Flask, Response, jsonify, request, send_from_directory, render_template
from systemd import journal
import subprocess
import socket
import time


app = Flask(__name__, static_url_path="/home/pi/arwain_inference_core/python_utils")
arwain_service_stdin = "/tmp/arwain.stdin"

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


def get_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0)
    s.connect(("10.255.255.255", 1))
    IP = s.getsockname()[0]
    return IP


def write_to_service(s):
    with open(arwain_service_stdin, "w") as h:
        h.write(s + "\n")


@app.route("/")
def index():
    return render_template("controller.html", ip=IP)
    return send_from_directory("static", "controller.html")


@app.route("/static/<path:path>")
def send_static(path):
    return send_from_directory("static", path)


@app.route("/info")
def info():
    j.get_next()
    response_dict = {"action":"info", "result":list()}
    while True:
        a = j.get_next().get("MESSAGE", "null")
        if a == "null":
            j.get_previous()
            break
        else:
            response_dict["result"].append(a)
    response = jsonify(response_dict)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/calibrate_gyroscopes")
def calibrate_gyroscopes():
    write_to_service("calibg")
    response_dict = {"action": "calibg"}
    response = jsonify(response_dict)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/calibrate_magnetometer")
def calibrate_magnetometer():
    write_to_service("calibm")
    response_dict = {"action": "calibm"}
    response = jsonify(response_dict)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/mode")
def check_mode():
    response_dict = {"action":"mode"}
    write_to_service("mode")
    response = jsonify(response_dict)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/start", methods=["GET"])
def start():
    response_dict = {"action": "start"}

    fname = request.args.get("fname")
    if fname is not None:
        write_to_service(f"name {fname}")
        time.sleep(0.1)
        write_to_service("infer")
    else:
        write_to_service("name nullname")
        time.sleep(0.1)
        write_to_service("infer")        

    response = jsonify(response_dict)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/stop")
def stop():
    response_dict = {"action":"stop", "result": "Stopped"}
    write_to_service("idle")
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


@app.route("/reboot")
def reboot():
    subprocess.Popen(["sudo", "reboot", "now"])
    response_dict = dict()
    response = jsonify(response_dict)
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response
    


@app.route("/restart_service")
def restart():
    subprocess.Popen(["sudo", "systemctl", "stop", "arwain"])
    time.sleep(1)
    subprocess.Popen(["sudo", "systemctl", "enable", "--now", "arwain"])
    response = jsonify({'action': f"kill", "result": "restarted_service"})
    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


j = journal.Reader()
j.this_boot()
j.add_match(_SYSTEMD_UNIT="arwain.service")
j.seek_tail()
j.get_previous()


IP = get_ip()


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8051, debug=False)
