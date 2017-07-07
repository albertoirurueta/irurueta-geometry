/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.slam.AbsoluteOrientationConstantVelocityModelSlamCalibrationData
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 12, 2016.
 */
package com.irurueta.geometry.slam;

import java.io.Serializable;

/**
 * Contains control calibration data for an absolute orientation constant 
 * velocity model SLAM estimator during Kalman filtering prediction stage.
 */
public class AbsoluteOrientationConstantVelocityModelSlamCalibrationData 
        extends BaseCalibrationData implements Serializable {
    
    /**
     * Constructor.
     */
    public AbsoluteOrientationConstantVelocityModelSlamCalibrationData() {
        super(AbsoluteOrientationConstantVelocityModelSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationConstantVelocityModelSlamEstimator.STATE_LENGTH);
    }
}
