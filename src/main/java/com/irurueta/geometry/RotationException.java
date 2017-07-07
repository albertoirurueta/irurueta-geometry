/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.RotationException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 9, 2012
 */
package com.irurueta.geometry;

/**
 * Exception raised when doing operations on rotations
 */
public class RotationException extends GeometryException{
    /**
     * Constructor
     */
    public RotationException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public RotationException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public RotationException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public RotationException(Throwable cause){
        super(cause);
    }    
}
