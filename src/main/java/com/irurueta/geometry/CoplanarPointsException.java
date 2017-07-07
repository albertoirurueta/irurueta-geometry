/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.CoplanarPointsException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 26, 2015
 */
package com.irurueta.geometry;

/**
 * Raised when provided points are coplanar (lay on a single plane)
 */
public class CoplanarPointsException extends GeometryException{
    /**
     * Constructor
     */
    public CoplanarPointsException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public CoplanarPointsException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public CoplanarPointsException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public CoplanarPointsException(Throwable cause){
        super(cause);
    }
}
