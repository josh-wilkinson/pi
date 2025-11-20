import cv2
import numpy as np
from flask import Flask, Response

app = Flask(__name__)

# Initialize camera
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Restore auto exposure mode
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)   # 0.75 = Auto mode on V4L2

# Lower brightness if the image is washed out
cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.3)

# Reduce gain (auto-gain is usually the problem)
cap.set(cv2.CAP_PROP_GAIN, 0)               # Try 0–10

# Optional: stabilize colors
cap.set(cv2.CAP_PROP_AUTO_WB, 1)            # enable auto white balance


# ArUco setup
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)


def generate_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # fix upside-down image here
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        corners, ids, _ = detector.detectMarkers(gray)
        print("Detected:", ids)

        # Draw markers if found
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()

        # MJPEG frame format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return """
    <html>
        <head><title>ArUco Stream</title></head>
        <body>
            <h1>Raspberry Pi ArUco Camera Stream</h1>
            <img src="/video_feed">
        </body>
    </html>
    """


@app.route('/video_feed')
def video_feed():
    return Response(
        generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


if __name__ == '__main__':
    print("Starting server... Open http://<pi-ip>:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)
