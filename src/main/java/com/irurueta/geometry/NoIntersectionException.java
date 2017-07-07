/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.NoIntersectionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 13, 2012
 */
package com.irurueta.geometry;

/**
 *  Raised when lines or planes are parallel and there is no intersection
 */
public class NoIntersectionException extends GeometryException {
    
    /**
     * Constructor
     */
    public NoIntersectionException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public NoIntersectionException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public NoIntersectionException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public NoIntersectionException(Throwable cause){
        super(cause);
    }
}
