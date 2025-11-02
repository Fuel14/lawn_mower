import numpy as np 
import cv2
from tqdm import tqdm
import os

# Create directories to save captured images
pathL = "Camera_Calibration/data/stereoL/"
pathR = "Camera_Calibration/data/stereoR/"

# Create directories if they don't exist
os.makedirs(pathL, exist_ok=True)
os.makedirs(pathR, exist_ok=True)

print("Live camera calibration - Press 'c' to capture, 'q' to quit")

# Termination criteria for refining the detected corners
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((9*6,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Initialize camera captures
capL = cv2.VideoCapture(0)  # Left camera
capR = cv2.VideoCapture(1)  # Right camera

# Set camera properties (optional)
capL.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capL.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
capR.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capR.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Check if cameras opened successfully
if not capL.isOpened() or not capR.isOpened():
    print("Error: Could not open cameras")
    exit()

img_count = 0
img_ptsL = []
img_ptsR = []
obj_pts = []

print("Press 'c' to capture images when chessboard is visible in both cameras")
print("Press 'q' to quit and perform calibration")

while True:
    retL, frameL = capL.read()
    retR, frameR = capR.read()
    
    if not retL or not retR:
        print("Error: Could not read frames")
        break
    
    # Convert to grayscale for chessboard detection
    grayL = cv2.cvtColor(frameL, cv2.COLOR_BGR2GRAY)
    grayR = cv2.cvtColor(frameR, cv2.COLOR_BGR2GRAY)
    
    # Detect chessboard corners
    ret_cornersL, cornersL = cv2.findChessboardCorners(grayL, (9,6), None)
    ret_cornersR, cornersR = cv2.findChessboardCorners(grayR, (9,6), None)
    
    # Draw chessboard corners if found
    displayL = frameL.copy()
    displayR = frameR.copy()
    
    if ret_cornersL:
        cv2.drawChessboardCorners(displayL, (9,6), cornersL, ret_cornersL)
    
    if ret_cornersR:
        cv2.drawChessboardCorners(displayR, (9,6), cornersR, ret_cornersR)
    
    # Add instruction text
    cv2.putText(displayL, "Press 'c' to capture, 'q' to quit", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(displayL, f"Captured: {img_count} images", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    cv2.putText(displayR, "Press 'c' to capture, 'q' to quit", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(displayR, f"Captured: {img_count} images", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Display the frames
    cv2.imshow('Left Camera', displayL)
    cv2.imshow('Right Camera', displayR)
    
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('c'):
        # Capture image only if chessboard is found in both cameras
        if ret_cornersL and ret_cornersR:
            img_count += 1
            
            # Refine corner positions
            cornersL_refined = cv2.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
            cornersR_refined = cv2.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
            
            # Store points for calibration
            img_ptsL.append(cornersL_refined)
            img_ptsR.append(cornersR_refined)
            obj_pts.append(objp)
            
            # Save images
            cv2.imwrite(f"{pathL}img{img_count}.png", frameL)
            cv2.imwrite(f"{pathR}img{img_count}.png", frameR)
            
            print(f"Captured image pair {img_count}")
            
            # Show confirmation
            confirmation = np.zeros((100, 600, 3), dtype=np.uint8)
            cv2.putText(confirmation, f"Captured image pair {img_count} successfully!", 
                       (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow('Confirmation', confirmation)
            cv2.waitKey(500)  # Show confirmation for 500ms
            cv2.destroyWindow('Confirmation')
        else:
            print("Chessboard not found in both cameras! Try different position.")
    
    elif key == ord('q'):
        print("Quitting capture...")
        break

# Release cameras
capL.release()
capR.release()
cv2.destroyAllWindows()

# Perform calibration if we have enough images
if len(obj_pts) > 0:
    print(f"\nStarting calibration with {len(obj_pts)} image pairs...")
    
    # Use the first captured images to get dimensions
    imgL = cv2.imread(f"{pathL}img1.png", 0)
    imgR = cv2.imread(f"{pathR}img1.png", 0)
    
    print("Calculating left camera parameters ... ")
    # Calibrating left camera
    retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(obj_pts, img_ptsL, imgL.shape[::-1], None, None)
    hL, wL = imgL.shape[:2]
    new_mtxL, roiL = cv2.getOptimalNewCameraMatrix(mtxL, distL, (wL, hL), 1, (wL, hL))

    print("Calculating right camera parameters ... ")
    # Calibrating right camera
    retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(obj_pts, img_ptsR, imgR.shape[::-1], None, None)
    hR, wR = imgR.shape[:2]
    new_mtxR, roiR = cv2.getOptimalNewCameraMatrix(mtxR, distR, (wR, hR), 1, (wR, hR))

    print("Stereo calibration .....")
    flags = 0
    flags |= cv2.CALIB_FIX_INTRINSIC
    criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Stereo calibration
    retS, new_mtxL, distL, new_mtxR, distR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(obj_pts,
                                                              img_ptsL,
                                                              img_ptsR,
                                                              new_mtxL,
                                                              distL,
                                                              new_mtxR,
                                                              distR,
                                                              imgL.shape[::-1],
                                                              criteria_stereo,
                                                              flags)

    # Stereo rectification
    rectify_scale = 1
    rect_l, rect_r, proj_mat_l, proj_mat_r, Q, roiL, roiR = cv2.stereoRectify(new_mtxL, distL, new_mtxR, distR,
                                                     imgL.shape[::-1], Rot, Trns,
                                                     rectify_scale, (0,0))

    # Compute rectification maps
    Left_Stereo_Map = cv2.initUndistortRectifyMap(new_mtxL, distL, rect_l, proj_mat_l,
                                                 imgL.shape[::-1], cv2.CV_16SC2)
    Right_Stereo_Map = cv2.initUndistortRectifyMap(new_mtxR, distR, rect_r, proj_mat_r,
                                                  imgR.shape[::-1], cv2.CV_16SC2)

    print("Saving parameters ......")
    cv_file = cv2.FileStorage("data/params_py.xml", cv2.FILE_STORAGE_WRITE)
    cv_file.write("Left_Stereo_Map_x", Left_Stereo_Map[0])
    cv_file.write("Left_Stereo_Map_y", Left_Stereo_Map[1])
    cv_file.write("Right_Stereo_Map_x", Right_Stereo_Map[0])
    cv_file.write("Right_Stereo_Map_y", Right_Stereo_Map[1])
    cv_file.write("Q", Q)
    cv_file.release()
    
    print("Calibration completed successfully!")
    print(f"Calibration parameters saved to data/params_py.xml")
else:
    print("No images captured for calibration!")