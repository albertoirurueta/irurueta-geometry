/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.NormalizerException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 9, 2015
 */
package com.irurueta.geometry.estimators;

/**
 * Raised when normalizer cannot normalize points. Usually this happens when
 * only a point is provided or all provided points are located too close to each
 * other, and hence normalization scale becomes infinite creating a numerical
 * degeneracy
 */
public class NormalizerException extends GeometryEstimatorException{
    /**
     * Constructor
     */
    public NormalizerException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message message indicating the cause of the exception
     */
    public NormalizerException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message message describing the cause of the exception
     * @param cause instance containing the cause of the exception
     */
    public NormalizerException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause instance containing the cause of the exception
     */
    public NormalizerException(Throwable cause){
        super(cause);
    }
}
