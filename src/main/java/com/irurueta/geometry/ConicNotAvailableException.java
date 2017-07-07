/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.ConicNotAvailableException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 4, 2012
 */
package com.irurueta.geometry;

/**
 * Exception raised when a conic cannot be computed
 */
public class ConicNotAvailableException extends GeometryException {
    /**
     * Constructor
     */
    public ConicNotAvailableException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public ConicNotAvailableException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public ConicNotAvailableException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public ConicNotAvailableException(Throwable cause){
        super(cause);
    }
}
