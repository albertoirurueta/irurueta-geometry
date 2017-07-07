/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.ImageOfAbsoluteConicEstimatorType
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 1, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

/**
 * Image of Absolute Conic (IAC) estimator types.
 */
public enum ImageOfAbsoluteConicEstimatorType {
    /**
     * Image of Absolute Conic (IAC) estimator using LMSE (Least Mean Square 
     * Error) solutions.
     */
    LMSE_IAC_ESTIMATOR,
    
    /**
     * Image of Absolute Conic (IAC) estimator using weighted samples.
     */
    WEIGHTED_IAC_ESTIMATOR
}
