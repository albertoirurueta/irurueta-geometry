/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.estimators.FundamentalMatrixEstimatorMethod
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 12, 2015
 */
package com.irurueta.geometry.epipolar.estimators;

/**
 * Indicates method of non-robust fundamental matrix estimator.
 */
public enum FundamentalMatrixEstimatorMethod {
    /**
     * Fundamental matrix estimation method based on 7 matched 2D points.
     */
    SEVEN_POINTS_ALGORITHM,
    
    /**
     * Fundamental matrix estimation method based on 8 matched 2D points.
     */
    EIGHT_POINTS_ALGORITHM,
    
    /**
     * Algorhtm used for Affine geometry. This method should only be used
     * on specific cases where cameras ore geometry are known to be affine.
     */
    AFFINE_ALGORITHM
}
