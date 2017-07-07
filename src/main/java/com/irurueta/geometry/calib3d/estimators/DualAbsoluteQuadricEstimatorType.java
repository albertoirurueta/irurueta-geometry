/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.DualAbsoluteQuadricEstimatorType
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 22, 2016.
 */
package com.irurueta.geometry.calib3d.estimators;

/**
 * Defines types of Dual Absolute Quadric estimators depending their algorithm 
 * of implementation.
 */
public enum DualAbsoluteQuadricEstimatorType {
    /**
     * Dual Absolute Quadric estimator using an LMSE solution.
     */
    LMSE_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR,
    
    /**
     * Dual Absolute Quadric estimator 
     */
    WEIGHTED_DUAL_ABSOLUTE_QUADRIC_ESTIMATOR
}
