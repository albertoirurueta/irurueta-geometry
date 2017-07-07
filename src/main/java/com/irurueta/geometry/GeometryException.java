/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.GeometryException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 1, 2012
 */
package com.irurueta.geometry;

/**
 * Base exception for all exceptions in the com.irurueta.geometry package.
 */
public class GeometryException extends Exception{
    
    /**
     * Constructor.
     */
    public GeometryException(){
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message Message indicating the cause of the exception.
     */
    public GeometryException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message Message describing the cause of the exception.
     * @param cause Instance containing the cause of the exception.
     */
    public GeometryException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause Instance containing the cause of the exception.
     */
    public GeometryException(Throwable cause){
        super(cause);
    }
}
