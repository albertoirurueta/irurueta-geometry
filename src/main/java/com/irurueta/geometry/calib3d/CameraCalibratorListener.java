/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.CameraCalibratorListener
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 7, 2015
 */
package com.irurueta.geometry.calib3d;

import com.irurueta.geometry.PinholeCameraIntrinsicParameters;

/**
 * Listener to be notified when calibration starts, finishes or any progress
 * changes
 */
public interface CameraCalibratorListener {
    /**
     * Called when a calibrator starts the camera calibration process.
     * @param calibrator reference to a camera calibrator
     */
    public void onCalibrateStart(CameraCalibrator calibrator);
    
    /**
     * Called when a calibrator ends the camera calibration process.
     * @param calibrator reference to a camera calibrator
     */
    public void onCalibrateEnd(CameraCalibrator calibrator);
    
    /**
     * Called to notify changes in the camera calibration progress.
     * @param calibrator reference to a camera calibrator
     * @param progress current percentage of progress expressed as a value
     * between 0.0f and 1.0f
     */
    public void onCalibrateProgressChange(CameraCalibrator calibrator, 
            float progress);
    
    /**
     * Called when a calibrator starts the estimation of intrinsic parameters
     * of a pinhole camera.
     * Depending on the calibrator implementation, this method might be called
     * multiple times while the distortion parameters are being refined
     * @param calibrator reference to a camera calibrator
     */
    public void onIntrinsicParametersEstimationStarts(
            CameraCalibrator calibrator);
    
    /**
     * Called when a calibrator finishes the estimation of intrinsic parameters
     * of a pinhole camera.
     * Depending on the calibrator implementation, this method might be called
     * multiple times while the distortion parameters are being refined
     * @param calibrator reference to a camera calibrator
     * @param intrinsicParameters intrinsic parameters that have been estimated
     * so far
     */
    public void onIntrinsicParametersEstimationEnds(CameraCalibrator calibrator,
            PinholeCameraIntrinsicParameters intrinsicParameters);
    
    /**
     * Called when a calibrator starts the estimation of radial distortion of
     * the camera lens.
     * Depending on the calibrator implementation, this method might be called
     * multiple times while the distortion parameters are being refined
     * @param calibrator reference to a camera calibrator
     */
    public void onRadialDistortionEstimationStarts(CameraCalibrator calibrator);
    
    /**
     * Called when a calibrator finishes the estimation of radial distortion of
     * the camera lens.
     * Depending on the calibrator implementation, this method might be called
     * multiple times while the distortion parameters are being refined
     * @param calibrator reference to a camera calibrator
     * @param distortion radial distortion that has been estimated so far
     */
    public void onRadialDistortionEstimationEnds(CameraCalibrator calibrator,
            RadialDistortion distortion);
}
