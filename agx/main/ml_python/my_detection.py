from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput

net = detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = videoSource('/dev/video0')      # '/dev/video0' for V4L2 default:videoSource("csi://0")
display = videoOutput('rtp://192.168.1.180:1234') # 'my_video.mp4' for file or 'rtp://192.168.1.180:1234' to stream

while display.IsStreaming():
    img = camera.Capture()

    if img is None: # capture timeout
        continue

    detections = net.Detect(img)    
    for detection in detections:
        x,y=detection.Center # Print only the center coordinates
        print('X: {}, Y: {}'.format(x,y))

    
    display.Render(img)
    #display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))

