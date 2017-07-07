/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.NotSupportedException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 16, 2015
 */
package com.irurueta.geometry;

/**
 * Raised when a given feature is not supported
 */
public class NotSupportedException extends GeometryException{
    
    /**
     * Constructor
     */
    public NotSupportedException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public NotSupportedException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public NotSupportedException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public NotSupportedException(Throwable cause){
        super(cause);
    }    
}
