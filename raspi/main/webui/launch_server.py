from flask import Flask, jsonify, send_from_directory, Response
import subprocess
import os

app = Flask(__name__)

# Paths to scripts
SCRIPT_PATHS = {
    "Setup": "/home/walle/webui/run_talker.sh",
    "Controller": "/home/walle/webui/controller_setup.sh",
    "Scan": "/home/walle/webui/start_scan.sh",
    "Vision": "/home/walle/webui/start_vision.sh"
}

# Dictionary to track running processes
processes = {}

def execute_script(script_path):
    """Utility function to execute a bash script and return the result."""
    try:
        if not os.path.exists(script_path):
            return {"error": f"Script {script_path} not found!"}, 404

        process = subprocess.Popen(
            ["bash", script_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # Create a new process group
        )
        return process
    except Exception as e:
        return {"error": str(e)}, 500

@app.route('/')
def home():
    # Serve the main HTML file
    return send_from_directory('.', 'test_index.html')

@app.route('/start/<script_name>', methods=['POST'])
def start_script(script_name):
    script_path = SCRIPT_PATHS.get(script_name)
    if not script_path:
        return jsonify({"error": f"Script {script_name} not found!"}), 404

    if script_name in processes and processes[script_name].poll() is None:
        return jsonify({"error": f"{script_name} is already running"}), 400

    process = execute_script(script_path)
    if isinstance(process, dict):  # Error occurred
        return jsonify(process[0]), process[1]

    processes[script_name] = process
    return jsonify({"status": f"{script_name} started"}), 200

@app.route('/stop/<script_name>', methods=['POST'])
def stop_script(script_name):
    process = processes.get(script_name)
    if process and process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), subprocess.signal.SIGTERM)
            process.wait()
            processes.pop(script_name, None)
            return jsonify({"status": f"{script_name} stopped"}), 200
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    return jsonify({"error": f"{script_name} is not running"}), 400

@app.route('/status/<script_name>', methods=['GET'])
def status_script(script_name):
    process = processes.get(script_name)
    if process and process.poll() is None:
        return jsonify({"status": "running"}), 200

    return jsonify({"status": "stopped"}), 200

@app.route('/video_feed')
def video_feed():
    """Stream RTP video."""
    def generate():
        process = subprocess.Popen(
            [
                "gst-launch-1.0",
                "udpsrc", "address=10.0.0.177", "port=5000",  # RTP source
                "!", "application/x-rtp",
                "!", "rtph264depay",
                "!", "avdec_h264",
                "!", "videoconvert",
                "!", "jpegenc",
                "!", "multipartmux", "boundary=frame",
                "!", "filesink", "location=/dev/stdout"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        try:
            while True:
                frame = process.stdout.read(1024)
                if not frame:
                    break
                yield frame
        except GeneratorExit:
            process.kill()

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
from flask import Flask, jsonify, send_from_directory, Response
import subprocess
import os

app = Flask(__name__)

# Paths to scripts
SCRIPT_PATHS = {
    "Setup": "/home/walle/webui/run_talker.sh",
    "Controller": "/home/walle/webui/controller_setup.sh",
    "Scan": "/home/walle/webui/start_scan.sh",
    "Vision": "/home/walle/webui/start_vision.sh"
}

# Dictionary to track running processes
processes = {}

def execute_script(script_path):
    """Utility function to execute a bash script and return the result."""
    try:
        if not os.path.exists(script_path):
            return {"error": f"Script {script_path} not found!"}, 404

        process = subprocess.Popen(
            ["bash", script_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # Create a new process group
        )
        return process
    except Exception as e:
        return {"error": str(e)}, 500

@app.route('/')
def home():
    # Serve the main HTML file
    return send_from_directory('.', 'test_index.html')

@app.route('/start/<script_name>', methods=['POST'])
def start_script(script_name):
    script_path = SCRIPT_PATHS.get(script_name)
    if not script_path:
        return jsonify({"error": f"Script {script_name} not found!"}), 404

    if script_name in processes and processes[script_name].poll() is None:
        return jsonify({"error": f"{script_name} is already running"}), 400

    process = execute_script(script_path)
    if isinstance(process, dict):  # Error occurred
        return jsonify(process[0]), process[1]

    processes[script_name] = process
    return jsonify({"status": f"{script_name} started"}), 200

@app.route('/stop/<script_name>', methods=['POST'])
def stop_script(script_name):
    process = processes.get(script_name)
    if process and process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), subprocess.signal.SIGTERM)
            process.wait()
            processes.pop(script_name, None)
            return jsonify({"status": f"{script_name} stopped"}), 200
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    return jsonify({"error": f"{script_name} is not running"}), 400

@app.route('/status/<script_name>', methods=['GET'])
def status_script(script_name):
    process = processes.get(script_name)
    if process and process.poll() is None:
        return jsonify({"status": "running"}), 200

    return jsonify({"status": "stopped"}), 200

@app.route('/video_feed')
def video_feed():
    """Stream RTP video."""
    def generate():
        process = subprocess.Popen(
            [
                "gst-launch-1.0",
                "udpsrc", "address=10.0.0.177", "port=5000",  # RTP source
                "!", "application/x-rtp",
                "!", "rtph264depay",
                "!", "avdec_h264",
                "!", "videoconvert",
                "!", "jpegenc",
                "!", "multipartmux", "boundary=frame",
                "!", "filesink", "location=/dev/stdout"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        try:
            while True:
                frame = process.stdout.read(1024)
                if not frame:
                    break
                yield frame
        except GeneratorExit:
            process.kill()

    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

