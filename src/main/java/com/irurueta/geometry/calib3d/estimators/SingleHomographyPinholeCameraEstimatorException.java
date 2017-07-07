/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.SingleHomographyPinholeCameraEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 23, 2017.
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.geometry.estimators.GeometryEstimatorException;

/**
 * Thrown when camera estimation fails.
 */
public class SingleHomographyPinholeCameraEstimatorException extends 
        GeometryEstimatorException {
    
    /**
     * Constructor.
     */
    public SingleHomographyPinholeCameraEstimatorException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message messagen indicating the cause of the exception.
     */
    public SingleHomographyPinholeCameraEstimatorException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public SingleHomographyPinholeCameraEstimatorException(String message,
            Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public SingleHomographyPinholeCameraEstimatorException(Throwable cause) {
        super(cause);
    }
}
