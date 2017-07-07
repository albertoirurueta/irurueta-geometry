/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.slam.AbsoluteOrientationSlamCalibrationData
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 12, 2016.
 */
package com.irurueta.geometry.slam;

import java.io.Serializable;

/**
 * Contains control calibration data for an absolute orientation SLAM estimator 
 * during Kalman filtering prediction stage.
 */
public class AbsoluteOrientationSlamCalibrationData extends BaseCalibrationData 
        implements Serializable {
    
    /**
     * Constructor.
     */
    public AbsoluteOrientationSlamCalibrationData() {
        super(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH,
                AbsoluteOrientationSlamEstimator.STATE_LENGTH);
    }
}
