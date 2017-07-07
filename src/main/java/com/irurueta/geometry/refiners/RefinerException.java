/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.refiners.RefinerException.
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 28, 2017.
 */
package com.irurueta.geometry.refiners;

import com.irurueta.geometry.GeometryException;

/**
 * Exception raised when refinement fails.
 */
public class RefinerException extends GeometryException {
    
    /**
     * Constructor.
     */
    public RefinerException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public RefinerException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public RefinerException(String message, Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public RefinerException(Throwable cause) {
        super(cause);
    }
}
