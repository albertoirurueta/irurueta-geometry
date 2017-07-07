/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.slam.GeometryException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 6, 2016.
 */
package com.irurueta.geometry.slam;

import com.irurueta.geometry.GeometryException;

/**
 * Base exception for all exceptions related to SLAM.
 */
public class SlamException extends GeometryException {
    
    /**
     * Constructor.
     */
    public SlamException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public SlamException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause
     */
    public SlamException(String message, Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public SlamException(Throwable cause) {
        super(cause);
    }
}
