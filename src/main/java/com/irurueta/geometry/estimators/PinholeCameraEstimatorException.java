/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimator.PinholeCameraEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 18, 2013
 */
package com.irurueta.geometry.estimators;

/**
 * Thrown when pinhole camera estimation fails
 */
public class PinholeCameraEstimatorException extends GeometryEstimatorException{
    
    /**
     * Constructor
     */
    public PinholeCameraEstimatorException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public PinholeCameraEstimatorException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public PinholeCameraEstimatorException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public PinholeCameraEstimatorException(Throwable cause){
        super(cause);
    }
}
