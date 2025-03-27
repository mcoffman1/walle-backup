import cv2 as cv

def canny_webcam():

    # Check if OpenCV is compiled with CUDA support
    if not cv.cuda.getCudaEnabledDeviceCount():
        print("OpenCV is not compiled with CUDA support, falling back to CPU.")
        return

    # Create Canny detector with CUDA
    detector = cv.cuda.createCannyEdgeDetector(75, 100)

    # Start VideoStream
    cap = cv.VideoCapture(0)
    edgeshow = -1

    while True:
        if cv.waitKey(20) == ord(' '):
            edgeshow = edgeshow * -1

        ret, frame = cap.read()
        if not ret:
            print("Could not read frame from camera. Exiting...")
            break

        # Resize the frame to 720p
        frame = cv.resize(frame, (1280, 720))

        # Upload frame to GPU
        gpu_frame = cv.cuda_GpuMat()
        gpu_frame.upload(frame)

        # Convert to grayscale on GPU
        gpu_gray = cv.cuda.cvtColor(gpu_frame, cv.COLOR_BGR2GRAY)

        # Apply Canny detector on GPU
        gpu_edge = detector.detect(gpu_gray)

        # Download result back to CPU
        edge = gpu_edge.download()

        if edgeshow == 1:
            cv.imshow('Canny Edge', edge)
        else:
            cv.imshow('Normal', frame)

        if cv.waitKey(20) == ord('q'):
            cap.release()
            cv.destroyAllWindows()
            break

canny_webcam()

