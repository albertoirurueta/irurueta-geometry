/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.GeometryEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 18, 2013
 */
package com.irurueta.geometry.estimators;

/**
 * Raised when an estimator is not ready
 */
public class NotReadyException extends GeometryEstimatorException{
    
    /**
     * Constructor
     */
    public NotReadyException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public NotReadyException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public NotReadyException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public NotReadyException(Throwable cause){
        super(cause);
    }
}
