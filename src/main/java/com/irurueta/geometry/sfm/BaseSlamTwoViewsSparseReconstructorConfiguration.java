/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.BaseSlamTwoViewsSparseReconstructorConfiguration
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 15, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.slam.BaseCalibrationData;

import java.io.Serializable;

/**
 * Contains base configuration for a two view sparse reconstructor using SLAM 
 * (Simultaneous Location And Mapping) to determine the scale of the scene
 * (i.e. the baseline or separation between cameras) by fusing both camera data
 * and data from sensors like an accelerometer or gyroscope.
 * @param <C> type defining calibration data.
 * @param <T> an actual implementation of a configuration class.
 */
public abstract class BaseSlamTwoViewsSparseReconstructorConfiguration<
        C extends BaseCalibrationData, 
        T extends BaseSlamTwoViewsSparseReconstructorConfiguration> extends 
        BaseTwoViewsSparseReconstructorConfiguration<T> implements Serializable {
    
    /**
     * Calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If this is null, no calibration data will be used.
     */
    private C mCalibrationData;
    
    /**
     * Constructor.
     */
    public BaseSlamTwoViewsSparseReconstructorConfiguration() { }
    
    /**
     * Gets calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If this is null, no calibration data will be used.
     * @return calibration data or null.
     */
    public C getCalibrationData() {
        return mCalibrationData;
    }
    
    /**
     * Specifies calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If set to null, no calibration data will be used.
     * @param calibrationData calibration data or null.
     * @return this instance so that method can be easily chained.
     */
    public T setCalibrationData(C calibrationData) {
        mCalibrationData = calibrationData;
        return (T)this;
    }
}
