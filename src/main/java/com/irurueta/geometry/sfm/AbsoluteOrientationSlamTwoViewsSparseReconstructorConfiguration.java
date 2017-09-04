/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 15, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.slam.AbsoluteOrientationSlamCalibrationData;

import java.io.Serializable;

/**
 * Contains configuration for a two view sparse reconstructor using SLAM
 * (Simultaneous Location And Mapping) to determine the scale of the scene
 * (i.e. the baseline or separation between cameras) by fusing both camera data
 * and data from sensors like an accelerometer or gyroscope.
 * This configuration assumes that an absolute orientation will be considered
 * on each view.
 */
public class AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration 
        extends BaseSlamTwoViewsSparseReconstructorConfiguration<
        AbsoluteOrientationSlamCalibrationData, 
        AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration> implements Serializable {

    /**
     * Creates an instance of a two views sparse reconstructor configuration with
     * slam estimation and absolute orientation.
     * @return configuration instance.
     */
    public static AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration make() {
        return new AbsoluteOrientationSlamTwoViewsSparseReconstructorConfiguration();
    }
}
