/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.ImageOfAbsoluteConicEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 1, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.geometry.estimators.GeometryEstimatorException;

/**
 * Thrown when IAC estimation fails.
 */
public class ImageOfAbsoluteConicEstimatorException extends 
        GeometryEstimatorException {
    
    /**
     * Constructor.
     */
    public ImageOfAbsoluteConicEstimatorException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message Message indicating the cause of the exception.
     */
    public ImageOfAbsoluteConicEstimatorException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message Message describing the cause of the exception.
     * @param cause Instance containing the cause of the exception.
     */
    public ImageOfAbsoluteConicEstimatorException(String message, 
            Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause Instance containing the cause of the exception.
     */
    public ImageOfAbsoluteConicEstimatorException(Throwable cause) {
        super(cause);
    }       
}
