/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.CameraException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date November 7, 2012
 */
package com.irurueta.geometry;

/**
 * Raised when an error occurs while using a camera
 */
public class CameraException extends GeometryException{
    /**
     * Constructor
     */
    public CameraException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public CameraException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public CameraException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public CameraException(Throwable cause){
        super(cause);
    }    
}
