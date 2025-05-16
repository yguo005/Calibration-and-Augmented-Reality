# Project 4: Calibration and Augmented Reality

## Development Environment

- **Key Libraries:**
    Built-in laptop camera
- **Target Used:**  "ARuco marker 

---

## Project Overview
This project implements a camera calibration and augmented reality (AR) system. The core tasks involve:
1.  Detecting and extracting corners from a target (e.g., checkerboard or ARuco marker).
2.  Allowing the user to select multiple views of the target for calibration.
3.  Calibrating the camera using these views to obtain intrinsic parameters (camera matrix, distortion coefficients) and extrinsic parameters for each view.
4.  Calculating the current pose (rotation and translation) of the camera relative to the target in real-time.
5.  Projecting 3D world points (like axes or virtual object vertices) onto the 2D image plane based on the estimated pose.
6.  Creating and displaying a 3D virtual object that appears anchored to the target and moves correctly with camera/target motion.
The project also explores robust feature detection as a potential basis for AR.

## Files Submitted
- `main_calibration.cpp` 
- `main_ar.cpp` 
- `robust_features.cpp` (for Task 7)
- `calibration_utils.cpp` 
- `calibration_utils.h`
- `ar_utils.cpp` 
- `ar_utils.h`
- `Makefile` 
- `readme.md`
- `calibration_params.yml` 


## Compilation and Execution



### Running the Executable(s)

**1. Camera Calibration Program (`./calibration` or similar):**
   `./calibration`
   - The program will attempt to open the default webcam.
   - **Key Bindings for Calibration:**
     - **'s'**: Save the current frame's detected target corners for calibration. The system will indicate how many views have been saved. (Needs at least 5 views).
     - **'c'**: Calibrate the camera using the saved views. Prints camera matrix, distortion coefficients, and re-projection error.
     - **'w'**: Write the current calibration parameters (camera matrix, distortion coefficients) to a file (e.g., `calibration_params.yml`).
     - **'q'**: Quit the program.
    
**2. Augmented Reality Program (`./ar` or similar):**
   `./ar calibration_params.yml` (or path to your saved calibration file)
   - Loads calibration parameters from the specified file.
   - Attempts to open the default webcam.
   - Detects the target and displays the virtual object and/or 3D axes.
   - **Key Bindings for AR:**
     - **'a'**: Toggle display of 3D axes on the target (Task 5).
     - **'v'**: Toggle display of the virtual object (Task 6).
     - **'p'**: Print current rotation and translation vectors to console (Task 4).
     - **'q'**: Quit the program.
     
**3. Robust Feature Detection Program (`./robust_features` or similar - Task 7):**
   `./robust_features`
   - Attempts to open the default webcam.
   - Displays detected robust features (e.g., Harris corners, SURF) on the live video stream and/or a chosen pattern.
   - **Key Bindings for Robust Features:**
     - **'h'**: Toggle Harris corner detection.
     - **'u'**: Toggle SURF feature detection (if implemented).
     - **'q'**: Quit the program.
    
**Calibration File:**
- The calibration program saves parameters to `calibration_params.yml` .
- The AR program loads parameters from this file.

