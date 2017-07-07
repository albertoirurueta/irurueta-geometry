/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.ReconstructionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 28, 2016.
 */
package com.irurueta.geometry.sfm;

/**
 * Exception raised if a reconstructor fails or is cancelled.
 */
public class ReconstructionException extends StructureFromMotionException {
    
    /**
     * Constructor.
     */
    public ReconstructionException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public ReconstructionException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public ReconstructionException(String message, Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public ReconstructionException(Throwable cause) {
        super(cause);
    }
}
