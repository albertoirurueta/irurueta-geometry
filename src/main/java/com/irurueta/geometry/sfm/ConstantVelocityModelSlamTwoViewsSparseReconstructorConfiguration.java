/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.ConstantVelocitySlamTwoViewsSparseReconstructorConfiguration
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 15, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.slam.ConstantVelocityModelSlamCalibrationData;

/**
 * Contains configuration for a two view sparse reconstructor using SLAM
 * (Simultaneous Location And Mapping) to determine the scale of the scene
 * (i.e. the baseline or separation between cameras) by fusing both camera data
 * and data from sensors like an accelerometer or gyroscope.
 * This configuration assumes a constant velocity model and that an orientation
 * relative to the first view will be estimated.
 */
public class ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration 
        extends BaseSlamTwoViewsSparseReconstructorConfiguration<
        ConstantVelocityModelSlamCalibrationData, 
        ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration> {
    
    /**
     * Creates an instance of a two views sparse reconstructor configuration with
     * constant velocity model in slam estimation.
     * @return configuration instance.
     */
    public static ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration make() {
        return new ConstantVelocityModelSlamTwoViewsSparseReconstructorConfiguration();
    }
}
