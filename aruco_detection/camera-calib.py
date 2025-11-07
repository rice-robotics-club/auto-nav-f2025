"""
Python equivalent of C++ ChArUco camera calibration code.

This is the Python conversion of the C++ OpenCV ChArUco calibration code.
Key differences from C++:
- Uses cv2.aruco module instead of cv::aruco
- Python lists instead of C++ vectors
- numpy arrays for matrix operations
- Different API for some functions (e.g., board.getChessboardCorners() vs board.matchImagePoints())
"""

import cv2
import numpy as np
from cv2 import aruco
import argparse

try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False


class RealSenseCapture:
    """
    Wrapper for Intel RealSense camera that mimics cv2.VideoCapture interface.
    Ensures we get the color stream (RGB), not infrared.
    """
    def __init__(self, width=640, height=480, fps=30):
        if not REALSENSE_AVAILABLE:
            raise ImportError("pyrealsense2 is not installed. Install with: pip install pyrealsense2")

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure color stream
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # Start pipeline
        try:
            self.pipeline.start(self.config)
            self._is_opened = True
            print(f"RealSense camera opened: {width}x{height} @ {fps}fps (Color stream)")
        except Exception as e:
            print(f"Failed to start RealSense pipeline: {e}")
            self._is_opened = False

    def isOpened(self):
        """Check if camera is opened"""
        return self._is_opened

    def read(self):
        """Read a frame from the camera. Returns (ret, frame) like cv2.VideoCapture"""
        if not self._is_opened:
            return False, None

        try:
            # Wait for frames
            frames = self.pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()

            if not color_frame:
                return False, None

            # Convert to numpy array (already in BGR8 format)
            frame = np.asanyarray(color_frame.get_data())
            return True, frame

        except Exception as e:
            print(f"Error reading frame: {e}")
            return False, None

    def release(self):
        """Release the camera"""
        if self._is_opened:
            self.pipeline.stop()
            self._is_opened = False
            print("RealSense camera released")


def calibrate_camera_charuco(capture, squares_x, squares_y, square_length, marker_length,
                           dictionary_id=aruco.DICT_4X4_50, calibration_flags=0, aspect_ratio=1.0):
    """
    Calibrate camera using ChArUco board detection
    
    Args:
        capture: cv2.VideoCapture or RealSenseCapture object (already opened)
        squares_x: Number of squares in X direction
        squares_y: Number of squares in Y direction
        square_length: Length of square side (in meters or preferred unit)
        marker_length: Length of marker side (in same unit as square_length)
        dictionary_id: ArUco dictionary to use
        calibration_flags: Calibration flags for cv2.calibrateCamera
        aspect_ratio: Aspect ratio if CALIB_FIX_ASPECT_RATIO flag is used
    
    Returns:
        camera_matrix: Camera intrinsic matrix
        dist_coeffs: Distortion coefficients
        rep_error: Reprojection error
        all_images: List of captured images used for calibration
    """
    
    # Create ArUco dictionary and ChArUco board
    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    board = aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length, dictionary)
    
    # Create detector with default parameters
    detector_params = aruco.DetectorParameters()
    charuco_params = aruco.CharucoParameters()
    detector = aruco.CharucoDetector(board, charuco_params, detector_params)

    # Check if capture is opened
    if not capture.isOpened():
        print(f"Error: Could not open capture device")
        return None, None, None, None

    print(f"Capture device opened successfully")

    # Storage for calibration data
    all_charuco_corners = []
    all_charuco_ids = []
    all_image_points = []
    all_object_points = []
    all_images = []
    image_size = None
    
    print("Auto mode: capturing every frame with a detected board. Press 'q' to stop (webcam) or let the video finish.")
    
    frame_count = 0
    while True:
        ret, image = capture.read()
        if not ret:
            print(f"Failed to read frame or end of video reached (frame {frame_count})")
            break
        
        frame_count += 1
        if frame_count % 30 == 0:  # Print every 30 frames
            print(f"Processing frame {frame_count}...")
            
        image_copy = image.copy()
        
        # Detect ChArUco board
        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)
        
        # Draw detected corners and markers for visualization
        if marker_ids is not None:
            aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)
        
        if charuco_corners is not None and len(charuco_corners) >= 6:
            aruco.drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids)
            # Add status text when board is detected
            cv2.putText(image_copy, f"Board detected (Auto capture)  Captured: {len(all_images)}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(image_copy, f"No board detected  Captured: {len(all_images)}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Display image
        cv2.imshow('ChArUco Detection', image_copy)
        key = cv2.waitKey(1) & 0xFF  # keep UI responsive

        # Auto-capture whenever a valid board is detected (need at least 6 corners for DLT algorithm)
        if charuco_corners is not None and len(charuco_corners) >= 6 and charuco_ids is not None:
            try:
                chessboard_corners = board.getChessboardCorners()

                # Extract object points - handle potential shape issues
                # chessboard_corners may be (N, 3) or (N, 1, 3)
                obj_pts = []
                for id in charuco_ids.flatten():
                    pt = chessboard_corners[int(id)]
                    # Flatten to ensure it's a 1D array of 3 elements
                    pt_flat = pt.flatten()
                    if len(pt_flat) == 3:
                        obj_pts.append(pt_flat)

                if len(obj_pts) < 6:
                    continue

                object_points = np.array(obj_pts, dtype=np.float32)
                image_points = charuco_corners.reshape(-1, 2).astype(np.float32)

                # Validate shapes
                if object_points.shape != (len(charuco_ids), 3):
                    continue
                if image_points.shape != (len(charuco_ids), 2):
                    continue

                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                all_image_points.append(image_points)
                all_object_points.append(object_points)
                all_images.append(image.copy())

                if image_size is None:
                    image_size = (image.shape[1], image.shape[0])  # (width, height)

            except Exception as e:
                # Skip frame on error with debug info for first few frames
                if frame_count <= 30:
                    print(f"Warning: Skipping frame {frame_count}: {e}")
                continue

        # Allow user to stop early (useful for webcam)
        if key == ord('q'):
            break
        
    capture.release()
    cv2.destroyAllWindows()

    if len(all_charuco_corners) == 0 or image_size is None:
        print("No frames captured for calibration or image size not determined!")
        return None, None, None, None

    print(f"Calibrating camera with {len(all_charuco_corners)} frames...")

    # point matching and geometry done internally
    rep_error, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_charuco_corners,
        charucoIds=all_charuco_ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
        flags=calibration_flags
    )

    print(f"Calibration completed!")
    print(f"Reprojection error: {rep_error}")
    print(f"Camera matrix:\n{camera_matrix}")
    print(f"Distortion coefficients: {dist_coeffs.ravel()}")

    return camera_matrix, dist_coeffs, rep_error, all_images


def main():
    # CLI arguments
    parser = argparse.ArgumentParser(description="ChArUco camera calibration")
    parser.add_argument("--video", type=str, default='', help="Path to input video file. If omitted, uses webcam.")
    parser.add_argument("--camera", type=int, default=0, help="Webcam index to use when --video is not provided.")
    parser.add_argument("--realsense", action='store_true', help="Use Intel RealSense camera (color stream).")
    parser.add_argument("--squares-x", type=int, default=11, help="Number of ChArUco squares in X direction.")
    parser.add_argument("--squares-y", type=int, default=8, help="Number of ChArUco squares in Y direction.")
    parser.add_argument("--square-length", type=float, default=0.023, help="Square side length in meters.")
    parser.add_argument("--marker-length", type=float, default=0.017, help="Marker side length in meters.")
    args = parser.parse_args()

    # Configure ChArUco board parameters
    squares_x = args.squares_x
    squares_y = args.squares_y
    square_length = args.square_length
    marker_length = args.marker_length

    # Calibration parameters
    calibration_flags = 0  # Default flags
    # calibration_flags = cv2.CALIB_FIX_ASPECT_RATIO  # Example flag

    # Create capture device based on flags
    if args.realsense:
        if not REALSENSE_AVAILABLE:
            print("Error: pyrealsense2 is not installed. Install with: pip install pyrealsense2")
            return
        print("Using Intel RealSense camera (color stream)")
        capture = RealSenseCapture()
    else:
        # Select source (video file or webcam)
        input_source = args.video if args.video else args.camera
        print(f"Using input source: {input_source}")
        capture = cv2.VideoCapture(input_source)

    # Perform calibration
    camera_matrix, dist_coeffs, rep_error, images = calibrate_camera_charuco(
        capture, squares_x, squares_y, square_length, marker_length,
        calibration_flags=calibration_flags
    )
    
    if camera_matrix is not None and dist_coeffs is not None:
        # Save calibration results
        np.savez('camera_calibration.npz', 
                camera_matrix=camera_matrix, 
                dist_coeffs=dist_coeffs,
                reprojection_error=rep_error)
        print("Calibration saved to camera_calibration.npz")
    else:
        print("Calibration failed!")


if __name__ == "__main__":
    main()
