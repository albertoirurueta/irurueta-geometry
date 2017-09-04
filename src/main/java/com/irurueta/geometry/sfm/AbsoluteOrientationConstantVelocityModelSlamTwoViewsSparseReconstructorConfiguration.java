/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 15, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.slam.AbsoluteOrientationConstantVelocityModelSlamCalibrationData;

import java.io.Serializable;

/**
 * Contains configuration for a two view sparse reconstructor using SLAM 
 * (Simultaneous Location And Mapping) to determine the scale of the scene
 * (i.e. the baseline or separation between cameras) by fusing both camera data
 * and data from sensors like an accelerometer or gyroscope.
 * This configuration assumes a constant velocity model and that an absolute 
 * orientation will be considered on each view.
 */
public class AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration 
        extends BaseSlamTwoViewsSparseReconstructorConfiguration<
        AbsoluteOrientationConstantVelocityModelSlamCalibrationData, 
        AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration> implements Serializable {
    
    /**
     * Creates an instance of a two views sparse reconstructor configuration with
     * constant velocity model in slam estimation and absolute orientation.
     * @return configuration instance.
     */
    public static AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration make() {
        return new AbsoluteOrientationConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();
    }
}
