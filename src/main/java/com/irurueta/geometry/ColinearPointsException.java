/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.ColinearPointsException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 17, 2012
 */
package com.irurueta.geometry;

/**
 * Raised when provided points are colinear (lay on a single line)
 */
public class ColinearPointsException extends GeometryException {
    /**
     * Constructor
     */
    public ColinearPointsException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public ColinearPointsException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public ColinearPointsException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public ColinearPointsException(Throwable cause){
        super(cause);
    }    
}
