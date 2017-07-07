/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.UndefinedPointException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 14, 2012
 */
package com.irurueta.geometry;

/**
 * Raised when a point cannot be defined (because solution is indeterminate) or
 * there are infinite points, etc.
 */
public class UndefinedPointException extends GeometryException{
    /**
     * Constructor
     */
    public UndefinedPointException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public UndefinedPointException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public UndefinedPointException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public UndefinedPointException(Throwable cause){
        super(cause);
    }    
}
