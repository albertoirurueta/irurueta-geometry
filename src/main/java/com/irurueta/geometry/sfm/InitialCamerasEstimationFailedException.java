/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.InitialCamerasEstimationFailedException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 3, 2016.
 */
package com.irurueta.geometry.sfm;

/**
 * Exception raised if initial cameras estimation fails.
 */
public class InitialCamerasEstimationFailedException extends 
        StructureFromMotionException {
    
    /**
     * Constructor.
     */
    public InitialCamerasEstimationFailedException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public InitialCamerasEstimationFailedException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public InitialCamerasEstimationFailedException(String message, 
            Throwable cause) {
        super(message, cause);        
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public InitialCamerasEstimationFailedException(Throwable cause) {
        super(cause);
    }
}
