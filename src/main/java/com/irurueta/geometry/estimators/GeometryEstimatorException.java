/**
 * This file contains implementation of
 * com.irurueta.geometry.estimators.GeometryException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 18, 2013
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.GeometryException;

/**
 * This is the base exception class of this package
 */
public class GeometryEstimatorException extends GeometryException{
    
    /**
     * Constructor
     */
    public GeometryEstimatorException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public GeometryEstimatorException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public GeometryEstimatorException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public GeometryEstimatorException(Throwable cause){
        super(cause);
    }
}
