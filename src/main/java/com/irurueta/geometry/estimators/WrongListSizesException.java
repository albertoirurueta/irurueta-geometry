/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimator.WrongListSizesException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 18, 2013
 */
package com.irurueta.geometry.estimators;

/**
 * Thrown when a pair of lists don't have equal size
 */
public class WrongListSizesException extends GeometryEstimatorException{
    
    /**
     * Constructor
     */
    public WrongListSizesException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public WrongListSizesException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public WrongListSizesException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public WrongListSizesException(Throwable cause){
        super(cause);
    }
}
