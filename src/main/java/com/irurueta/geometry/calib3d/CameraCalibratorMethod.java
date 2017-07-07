/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.CameraCalibratorMethod
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 7, 2015
 */
package com.irurueta.geometry.calib3d;

/**
 * Contains camera calibrator methods
 */
public enum CameraCalibratorMethod {
    /**
     * Alternates the estimation of the intrinsic camera parameters with
     * radial distortion estimation until the solution converges.
     * This is usually slower than error optimization method but typically is
     * more numerically stable
     */
    ALTERNATING_CALIBRATOR,
    
    /**
     * Optimizes radial distortion and intrinsic parameters after an initial
     * guess is found until the solution converges.
     * This typically converges faster than the alternating method
     */
    ERROR_OPTIMIZATION,
}
