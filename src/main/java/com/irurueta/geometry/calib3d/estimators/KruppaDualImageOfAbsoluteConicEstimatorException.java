/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.KruppaDualImageOfAbsoluteConicEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 19, 2016.
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.geometry.estimators.GeometryEstimatorException;

/**
 * Thrown when DIAC estimation fails.
 */
public class KruppaDualImageOfAbsoluteConicEstimatorException extends 
        GeometryEstimatorException {
    
    /**
     * Constructor.
     */
    public KruppaDualImageOfAbsoluteConicEstimatorException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public KruppaDualImageOfAbsoluteConicEstimatorException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public KruppaDualImageOfAbsoluteConicEstimatorException(String message, 
            Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public KruppaDualImageOfAbsoluteConicEstimatorException(Throwable cause) {
        super(cause);
    }
}
