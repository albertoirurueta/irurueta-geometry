/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.DualAbsoluteQuadricEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 22, 2016.
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.geometry.estimators.GeometryEstimatorException;

/**
 * Thrown when DAQ estimation fails.
 */
public class DualAbsoluteQuadricEstimatorException extends 
        GeometryEstimatorException {
    
    /**
     * Constructor.
     */
    public DualAbsoluteQuadricEstimatorException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public DualAbsoluteQuadricEstimatorException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public DualAbsoluteQuadricEstimatorException(String message, 
            Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public DualAbsoluteQuadricEstimatorException(Throwable cause) {
        super(cause);
    }
}
