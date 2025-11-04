import numpy as np
import cv2
import glob
import os
import time
from tqdm import tqdm

def capture_stereo_images(left_cam_id=0, right_cam_id=1, num_pairs=20, save_dir="calibration_images"):
    """
    Capture stereo image pairs from two webcams
    """
    os.makedirs(f"{save_dir}/left", exist_ok=True)
    os.makedirs(f"{save_dir}/right", exist_ok=True)
    
    cap_left = cv2.VideoCapture(left_cam_id)
    cap_right = cv2.VideoCapture(right_cam_id)
    
    cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    if not cap_left.isOpened() or not cap_right.isOpened():
        print("Error: Could not open cameras")
        return False
    
    print(f"Press 's' to capture stereo pair, 'q' to quit")
    print(f"Will capture {num_pairs} image pairs")
    print("Show chessboard pattern in different positions and orientations")
    
    pair_count = 0
    while pair_count < num_pairs:
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()
        
        if not ret_left or not ret_right:
            print("Error: Could not read frames from cameras")
            break
        
        combined = np.hstack((frame_left, frame_right))
        cv2.putText(combined, f"Pair {pair_count}/{num_pairs} - Press 's' to capture, 'q' to quit", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow('Stereo Cameras - Left | Right', combined)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            left_filename = f"{save_dir}/left/left_{pair_count:02d}.png"
            right_filename = f"{save_dir}/right/right_{pair_count:02d}.png"
            
            cv2.imwrite(left_filename, frame_left)
            cv2.imwrite(right_filename, frame_right)
            
            print(f"Saved pair {pair_count}: {left_filename}, {right_filename}")
            pair_count += 1
            
            cv2.putText(combined, "CAPTURED!", (300, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            cv2.imshow('Stereo Cameras - Left | Right', combined)
            cv2.waitKey(500)
            
        elif key == ord('q'):
            break
    
    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()
    
    print(f"Captured {pair_count} stereo pairs in '{save_dir}' directory")
    return True

def stereo_calibrate_with_rectification(left_images_path, right_images_path, chessboard_size, square_size):
    """
    Complete stereo calibration with rectification (FIXED VERSION)
    """
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp = objp * square_size
    
    objpoints = []
    img_ptsL = []
    img_ptsR = []
    
    left_images = sorted(glob.glob(left_images_path))
    right_images = sorted(glob.glob(right_images_path))
    
    print(f"Found {len(left_images)} left images and {len(right_images)} right images")
    print("Extracting image coordinates of respective 3D pattern...")
    
    # Get image size from first image
    img_sample = cv2.imread(left_images[0])
    if img_sample is None:
        raise ValueError("Could not read sample image to determine size")
    image_size = img_sample.shape[1], img_sample.shape[0]  # (width, height)
    
    successful_pairs = 0
    for i in tqdm(range(len(left_images))):
        left_img = cv2.imread(left_images[i])
        right_img = cv2.imread(right_images[i])
        
        if left_img is None or right_img is None:
            continue
            
        left_gray = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        
        retL, cornersL = cv2.findChessboardCorners(left_gray, chessboard_size, None)
        retR, cornersR = cv2.findChessboardCorners(right_gray, chessboard_size, None)
        
        if retL and retR:
            objpoints.append(objp)
            
            cornersL_refined = cv2.cornerSubPix(left_gray, cornersL, (11, 11), (-1, -1), criteria)
            cornersR_refined = cv2.cornerSubPix(right_gray, cornersR, (11, 11), (-1, -1), criteria)
            
            img_ptsL.append(cornersL_refined)
            img_ptsR.append(cornersR_refined)
            
            successful_pairs += 1
    
    print(f"\nSuccessfully processed {successful_pairs} stereo pairs")
    
    if successful_pairs < 3:
        raise ValueError(f"Not enough valid stereo pairs for calibration. Only found {successful_pairs}")
    
    # Individual camera calibration
    print("Calculating left camera parameters...")
    retL, mtxL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objpoints, img_ptsL, image_size, None, None)
    
    print("Calculating right camera parameters...")
    retR, mtxR, distR, rvecsR, tvecsR = cv2.calibrateCamera(objpoints, img_ptsR, image_size, None, None)
    
    # FIX: Ensure distortion coefficients have correct shape
    # distL and distR should be 1x5 or 1x4 arrays
    if distL.shape != (1, 5) and len(distL) == 5:
        distL = distL.reshape(1, 5)
    if distR.shape != (1, 5) and len(distR) == 5:
        distR = distR.reshape(1, 5)
    
    print("Left camera distortion coefficients shape:", distL.shape)
    print("Right camera distortion coefficients shape:", distR.shape)
    
    # Get optimal camera matrices
    new_mtxL, roiL = cv2.getOptimalNewCameraMatrix(mtxL, distL, image_size, 1, image_size)
    new_mtxR, roiR = cv2.getOptimalNewCameraMatrix(mtxR, distR, image_size, 1, image_size)
    
    # Stereo calibration
    print("Stereo calibration...")
    criteria_stereo = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    flags = cv2.CALIB_FIX_INTRINSIC
    
    # FIX: Use the original camera matrices for stereo calibration
    retS, mtxL, distL, mtxR, distR, Rot, Trns, Emat, Fmat = cv2.stereoCalibrate(
        objpoints, img_ptsL, img_ptsR, mtxL, distL, mtxR, distR,
        image_size, criteria_stereo, flags
    )
    
    print(f"Stereo calibration RMS error: {retS:.6f}")
    
    # Stereo rectification - use optimal matrices for rectification
    print("Performing stereo rectification...")
    rectify_scale = 1  # 0=cropped, 1=uncropped
    rect_l, rect_r, proj_mat_l, proj_mat_r, Q, roiL, roiR = cv2.stereoRectify(
        new_mtxL, distL, new_mtxR, distR, image_size, Rot, Trns, 
        rectify_scale, (0, 0)
    )
    
    # Create rectification maps
    print("Creating rectification maps...")
    Left_Stereo_Map = cv2.initUndistortRectifyMap(new_mtxL, distL, rect_l, proj_mat_l, 
                                                 image_size, cv2.CV_16SC2)
    Right_Stereo_Map = cv2.initUndistortRectifyMap(new_mtxR, distR, rect_r, proj_mat_r, 
                                                  image_size, cv2.CV_16SC2)
    
    return {
        'intrinsics': {
            'K1': mtxL, 'D1': distL,  # Original intrinsics
            'K2': mtxR, 'D2': distR,
            'new_K1': new_mtxL, 'new_K2': new_mtxR  # Optimal intrinsics
        },
        'extrinsics': {
            'R': Rot, 'T': Trns,
            'E': Emat, 'F': Fmat
        },
        'rectification': {
            'R1': rect_l, 'R2': rect_r,
            'P1': proj_mat_l, 'P2': proj_mat_r,
            'Q': Q
        },
        'maps': {
            'left_map': Left_Stereo_Map,
            'right_map': Right_Stereo_Map
        },
        'roi': (roiL, roiR)
    }

def format_calibration_output(calibration_data):
    """
    Format calibration parameters in copy-paste ready format (FIXED)
    """
    K1 = calibration_data['intrinsics']['K1']
    D1 = calibration_data['intrinsics']['D1']
    K2 = calibration_data['intrinsics']['K2']
    D2 = calibration_data['intrinsics']['D2']
    R = calibration_data['extrinsics']['R']
    T = calibration_data['extrinsics']['T']
    
    # FIX: Handle distortion coefficients shape properly
    if D1.shape[0] == 1:
        D1_flat = D1[0]
    else:
        D1_flat = D1.flatten()
        
    if D2.shape[0] == 1:
        D2_flat = D2[0]
    else:
        D2_flat = D2.flatten()
    
    # FIX: Handle translation vector shape properly
    # T can be (3,1) or (3,) - make sure we access it correctly
    if T.shape == (3, 1):
        T0, T1, T2 = T[0,0], T[1,0], T[2,0]
    else:
        T0, T1, T2 = T[0], T[1], T[2]
    
    output = f"""# Intrinsic parameters - Left Camera
self.K1 = np.array([
    [{K1[0,0]:.8f}, {K1[0,1]:.1f}, {K1[0,2]:.8f}],
    [{K1[1,0]:.1f}, {K1[1,1]:.8f}, {K1[1,2]:.8f}],
    [{K1[2,0]:.1f}, {K1[2,1]:.1f}, {K1[2,2]:.1f}]
])
        
# Intrinsic parameters - Right Camera  
self.K2 = np.array([
    [{K2[0,0]:.8f}, {K2[0,1]:.1f}, {K2[0,2]:.8f}],
    [{K2[1,0]:.1f}, {K2[1,1]:.8f}, {K2[1,2]:.8f}],
    [{K2[2,0]:.1f}, {K2[2,1]:.1f}, {K2[2,2]:.1f}]
])

# Distortion Coefficients - Left Camera
self.D1 = np.array([[{D1_flat[0]:.8f}], [{D1_flat[1]:.8f}], [{D1_flat[2]:.8f}], [{D1_flat[3]:.8f}], [{D1_flat[4]:.8f}]])

# Distortion Coefficients - Right Camera
self.D2 = np.array([[{D2_flat[0]:.8f}], [{D2_flat[1]:.8f}], [{D2_flat[2]:.8f}], [{D2_flat[3]:.8f}], [{D2_flat[4]:.8f}]])

# Extrinsics: Rotation between cameras
self.R = np.array([
    [{R[0,0]:.8f}, {R[0,1]:.8f}, {R[0,2]:.8f}],
    [{R[1,0]:.8f}, {R[1,1]:.8f}, {R[1,2]:.8f}],
    [{R[2,0]:.8f}, {R[2,1]:.8f}, {R[2,2]:.8f}]
])

# Translation vector between cameras (in mm)
self.T = np.array([[{T0:.8f}],[{T1:.8f}],[{T2:.8f}]])"""
    
    return output


def save_calibration_data(calibration_data, output_file="stereo_calibration_data.xml"):
    """
    Save complete calibration data to XML file
    """
    cv_file = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
    
    # Save intrinsics
    cv_file.write("K1", calibration_data['intrinsics']['K1'])
    cv_file.write("D1", calibration_data['intrinsics']['D1'])
    cv_file.write("K2", calibration_data['intrinsics']['K2'])
    cv_file.write("D2", calibration_data['intrinsics']['D2'])
    
    # Save extrinsics
    cv_file.write("R", calibration_data['extrinsics']['R'])
    cv_file.write("T", calibration_data['extrinsics']['T'])
    cv_file.write("E", calibration_data['extrinsics']['E'])
    cv_file.write("F", calibration_data['extrinsics']['F'])
    
    # Save rectification data
    cv_file.write("R1", calibration_data['rectification']['R1'])
    cv_file.write("R2", calibration_data['rectification']['R2'])
    cv_file.write("P1", calibration_data['rectification']['P1'])
    cv_file.write("P2", calibration_data['rectification']['P2'])
    cv_file.write("Q", calibration_data['rectification']['Q'])
    
    # Save maps
    left_map_x, left_map_y = calibration_data['maps']['left_map']
    right_map_x, right_map_y = calibration_data['maps']['right_map']
    
    cv_file.write("Left_Stereo_Map_x", left_map_x)
    cv_file.write("Left_Stereo_Map_y", left_map_y)
    cv_file.write("Right_Stereo_Map_x", right_map_x)
    cv_file.write("Right_Stereo_Map_y", right_map_y)
    
    cv_file.release()
    print(f"Complete calibration data saved to {output_file}")

def test_stereo_cameras(left_cam_id=0, right_cam_id=1):
    """Test if stereo cameras are working"""
    cap_left = cv2.VideoCapture(left_cam_id)
    cap_right = cv2.VideoCapture(right_cam_id)
    
    cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    if not cap_left.isOpened() or not cap_right.isOpened():
        return False
    
    print("Testing cameras... Press 'q' to exit")
    for i in range(50):
        ret_left, frame_left = cap_left.read()
        ret_right, frame_right = cap_right.read()
        
        if not ret_left or not ret_right:
            break
            
        combined = np.hstack((frame_left, frame_right))
        cv2.imshow('Camera Test', combined)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap_left.release()
    cap_right.release()
    cv2.destroyAllWindows()
    return True

# Main execution
if __name__ == "__main__":
    # Configuration
    LEFT_CAM_ID = 0
    RIGHT_CAM_ID = 1
    NUM_PAIRS = 15  # Reduced for testing
    SAVE_DIR = "calibration_images"
    CHESSBOARD_SIZE = (9, 6)  # Inner corners (columns, rows)
    SQUARE_SIZE = 23.5        # mm
    
    print("Complete Stereo Camera Calibration Pipeline")
    print("=" * 50)
    
    # Step 1: Test cameras
    print("Step 1: Testing cameras...")
    if not test_stereo_cameras(LEFT_CAM_ID, RIGHT_CAM_ID):
        print("Camera test failed. Please check camera IDs.")
        exit(1)
    
    # Step 2: Capture images
    print("\nStep 2: Capturing stereo images...")
    input("Press Enter to start capturing images...")
    
    if not capture_stereo_images(LEFT_CAM_ID, RIGHT_CAM_ID, NUM_PAIRS, SAVE_DIR):
        print("Image capture failed")
        exit(1)
    
    # Step 3: Perform complete calibration
    print("\nStep 3: Performing complete stereo calibration...")
    try:
        left_images_path = f"{SAVE_DIR}/left/*.png"
        right_images_path = f"{SAVE_DIR}/right/*.png"
        
        calibration_data = stereo_calibrate_with_rectification(
            left_images_path, right_images_path, CHESSBOARD_SIZE, SQUARE_SIZE
        )
        
        # Output 1: Copy-paste format
        output_text = format_calibration_output(calibration_data)
        print("\n" + "="*80)
        print("CALIBRATION PARAMETERS (copy-paste ready):")
        print("="*80)
        print(output_text)
        print("="*80)
        
        # Save to Python file
        with open("stereo_calibration_params.py", "w") as f:
            f.write("import numpy as np\n\n")
            f.write(output_text)
        
        # Output 2: Complete XML file
        save_calibration_data(calibration_data, "complete_stereo_calibration.xml")
        
        print(f"\nCalibration completed successfully!")
        print(f"- Copy-paste parameters saved to: stereo_calibration_params.py")
        print(f"- Complete calibration data saved to: complete_stereo_calibration.xml")
        print(f"- Calibration images saved in: {SAVE_DIR}/")
        
    except Exception as e:
        print(f"Error during calibration: {e}")
        import traceback
        traceback.print_exc()