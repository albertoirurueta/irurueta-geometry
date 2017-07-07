/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.slam.ConstantVelocityModelSlamCalibrationData
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 12, 2016.
 */
package com.irurueta.geometry.slam;

import java.io.Serializable;

/**
 * Contains control calibration data for constant velocity model SLAM estimator
 * during Kalman filtering prediction stage.
 */
public class ConstantVelocityModelSlamCalibrationData 
        extends BaseCalibrationData implements Serializable {
    
    /**
     * Constructor.
     */
    public ConstantVelocityModelSlamCalibrationData() {
        super(ConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                ConstantVelocityModelSlamEstimator.STATE_LENGTH);
    }
    
}
