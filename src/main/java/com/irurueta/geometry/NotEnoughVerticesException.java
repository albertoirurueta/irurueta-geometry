/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.NotEnoughVerticesException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 17, 2012
 */
package com.irurueta.geometry;

/**
 * Raised when not enough vertices are provided
 */
public class NotEnoughVerticesException extends GeometryException{
    
    /**
     * Constructor
     */
    public NotEnoughVerticesException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public NotEnoughVerticesException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public NotEnoughVerticesException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public NotEnoughVerticesException(Throwable cause){
        super(cause);
    }    
}
