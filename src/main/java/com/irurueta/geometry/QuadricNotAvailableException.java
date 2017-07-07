/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.QuadricNotAvailableException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date August 14, 2012
 */
package com.irurueta.geometry;

/**
 * Raised when a quadric cannot be computed because the inverse matrix of a
 * dual quadric does not exist
 */
public class QuadricNotAvailableException extends GeometryException {
   
    /**
     * Constructor
     */
    public QuadricNotAvailableException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public QuadricNotAvailableException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public QuadricNotAvailableException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public QuadricNotAvailableException(Throwable cause){
        super(cause);
    }
}
