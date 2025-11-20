import cv2
import numpy as np

def load_camera_calibration(path="camera_intrinsics.yml"):
    """Load camera matrix and distortion coefficients from a .yml file."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise IOError(f"Failed to open calibration file: {path}")

    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("distortion_coefficients").mat()
    fs.release()

    if camera_matrix is None or dist_coeffs is None:
        raise ValueError("camera_matrix or distortion_coefficients not found in YAML file.")

    return camera_matrix, dist_coeffs


def detect_aruco_markers():
    # Load camera calibration
    try:
        camera_matrix, dist_coeffs = load_camera_calibration("cv-course-robot-viewer-main/robot-code/camera_intrinsics.yml")
        print("Camera calibration loaded.")
    except Exception as e:
        print("Failed to load camera calibration:", e)
        return

    # Initialize camera
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # ArUco setup
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    print("Press 'q' to quit, 's' to save current frame")

    # Adjust this to your marker size in meters
    marker_length = 0.05  # 5 cm

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            print(f"Detected {len(ids)} markers: {ids.flatten()}")

            # Pose estimation for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, marker_length, camera_matrix, dist_coeffs
            )

            for i, corner in enumerate(corners):
                marker_id = ids[i][0]
                center = corner[0].mean(axis=0).astype(int)

                # Draw marker ID text
                cv2.putText(frame, f"ID: {marker_id}",
                            (center[0] - 20, center[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # Draw center point
                cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)

                # Draw 3D axis for pose visualization
                cv2.drawFrameAxes(
                    frame, camera_matrix, dist_coeffs,
                    rvecs[i], tvecs[i], marker_length * 0.5
                )

                # Print pose
                print(f"Marker {marker_id}:")
                print(" rvec:", rvecs[i].flatten())
                print(" tvec:", tvecs[i].flatten())

        cv2.imshow('ArUco Marker Detection', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            cv2.imwrite('aruco_detection_snapshot.jpg', frame)
            print("Frame saved as 'aruco_detection_snapshot.jpg'")

    cap.release()
    cv2.destroyAllWindows()


def main():
    print("ArUco Marker Detection with Calibration")
    print("OpenCV version:", cv2.__version__)

    try:
        detect_aruco_markers()
    except Exception as e:
        print("Error:", e)

if __name__ == "__main__":
    main()
