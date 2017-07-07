/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.slam.SlamCalibrationData
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 12, 2016.
 */
package com.irurueta.geometry.slam;

import java.io.Serializable;

/**
 * Contains control calibration data for a SLAM estimator during
 * Kalman filtering prediction stage.
 */
public class SlamCalibrationData extends BaseCalibrationData 
        implements Serializable {
    
    /**
     * Constructor.
     */
    public SlamCalibrationData() {
        super(SlamEstimator.CONTROL_LENGTH, SlamEstimator.STATE_LENGTH);
    }
}
