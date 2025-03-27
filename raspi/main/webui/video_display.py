from flask import Flask, Response
import subprocess

app = Flask(__name__)

@app.route('/')
def index():
    # Serve the HTML page to display the video
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>RTP Video Stream</title>
    </head>
    <body>
        <h1>Live Video Feed</h1>
        <img src="/video_feed" alt="RTP Stream" style="max-width: 100%; height: auto;" />
    </body>
    </html>
    '''

@app.route('/video_feed')
def video_feed():
    # Use GStreamer to decode the RTP stream and serve it as MJPEG
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
    app.run(host='0.0.0.0', port=5000)  # Serve the Flask app

