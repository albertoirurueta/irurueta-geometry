/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.InvalidPlaneAtInfinityException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 22, 2016.
 */
package com.irurueta.geometry.calib3d;

/**
 * Exception produced when plane at infinity cannot be determined.
 */
public class InvalidPlaneAtInfinityException extends CalibrationException {
    
    /**
     * Constructor.
     */
    public InvalidPlaneAtInfinityException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public InvalidPlaneAtInfinityException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public InvalidPlaneAtInfinityException(String message, Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public InvalidPlaneAtInfinityException(Throwable cause) {
        super(cause);
    }
}
