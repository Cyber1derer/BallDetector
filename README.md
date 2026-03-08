# BallDetector

BallDetector estimates the **3D position of a sphere from a single camera** by processing each video frame and fitting a geometric model to the detected ball contour.

## What the project does

Given:
- a calibrated camera,
- a known ball radius,
- and a reference image of the ball color,

the program detects the ball in 2D, reconstructs viewing rays, and computes the ball center in 3D coordinates.

## How it works

The pipeline in `BallDetector.cpp` and `ColorFilter.cpp` is:

1. **Learn a color model** from a sample image (`ColorFilter`).
2. **Segment the ball** in each frame using that model and extract its contour.
3. **Undistort contour points** using camera intrinsics.
4. Convert points to normalized 3D direction vectors.
5. **Fit a plane with a RANSAC-like loop** to vectors associated with the visible sphere edge.
6. Estimate the cone angle and recover the **sphere center position** from the known radius.
7. Reproject the estimated center to pixels to update the ROI for the next frame.

## Build

### Dependencies
- CMake >= 3.8
- OpenCV
- nlohmann_json (>= 3.2.0)


## Input and output

### Input
- Video file with a visible ball.
- Color reference image used to build the color filter.
- Camera calibration values and sphere radius configured in code.

### Output
- Per-frame estimated 3D ball coordinates printed to console.
- Saved coordinate log file (currently written to a hardcoded path).

## Notes

- This repository is currently a research/prototype implementation.

