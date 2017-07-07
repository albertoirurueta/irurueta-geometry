/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.StructureFromMotionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 29, 2015
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.GeometryException;

/**
 * Base exception for all exceptions in the com.irurueta.geometry.sfm package.
 */
public class StructureFromMotionException extends GeometryException {
    
    /**
     * Constructor.
     */
    public StructureFromMotionException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public StructureFromMotionException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public StructureFromMotionException(String message, Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public StructureFromMotionException(Throwable cause) {
        super(cause);
    }
}
