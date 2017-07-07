/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.CoincidentPointsException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 3, 2012
 */
package com.irurueta.geometry;

/**
 * Raised when providing points which are assumed to be equal
 */
public class CoincidentPointsException extends GeometryException {
    
    /**
     * Constructor
     */
    public CoincidentPointsException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public CoincidentPointsException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public CoincidentPointsException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public CoincidentPointsException(Throwable cause){
        super(cause);
    }
}
