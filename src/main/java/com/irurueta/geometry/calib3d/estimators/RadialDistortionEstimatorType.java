/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.RadialDistortionEstimatorType
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 16, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

/**
 * Defines types of radial distortion estimators depending on their 
 * implementation
 */
public enum RadialDistortionEstimatorType {
    /**
     * Radial distortion estimator using LMSE (Least Mean Square Error) 
     * solutions
     */
    LMSE_RADIAL_DISTORTION_ESTIMATOR,
    
    /**
     * Radial distortion estimator using weighted samples
     */
    WEIGHTED_RADIAL_DISTORTION_ESTIMATOR
}
