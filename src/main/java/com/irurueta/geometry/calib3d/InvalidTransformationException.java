/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.InvalidTransformationException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 28, 2016.
 */
package com.irurueta.geometry.calib3d;

/**
 * Exception produced when provided transformation is numerically unstable.
 */
public class InvalidTransformationException extends CalibrationException {
    
    /**
     * Constructor.
     */
    public InvalidTransformationException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public InvalidTransformationException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public InvalidTransformationException(String message, Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public InvalidTransformationException(Throwable cause) {
        super(cause);
    }
}
