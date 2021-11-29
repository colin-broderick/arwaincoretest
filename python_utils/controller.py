from flask import Flask, Response, jsonify, request
import subprocess
import time
import os
import signal


app = Flask(__name__)
process = None


@app.route("/info")
def info():
    if process is None:
        response = jsonify({'action': "info", "result": "Not running"})
    else:
        response = jsonify({'action': "info", "result": "Running"})

    response.headers.add('Access-Control-Allow-Origin', '*')
    return response


@app.route("/start", methods=["GET"])
def start():
    global process
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
        
        process = subprocess.Popen(cmd, shell=False)

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
        subprocess.Popen(["pkill", "-f", "ncs2"])

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
    app.run(host="0.0.0.0", port=1081, debug=False)
