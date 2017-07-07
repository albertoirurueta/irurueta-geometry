/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.RadialDistortionEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 16, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.geometry.estimators.GeometryEstimatorException;

/**
 * Thrown when radial distortion estimation fails
 */
public class RadialDistortionEstimatorException extends GeometryEstimatorException{
    
    /**
     * Constructor
     */
    public RadialDistortionEstimatorException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public RadialDistortionEstimatorException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public RadialDistortionEstimatorException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public RadialDistortionEstimatorException(Throwable cause){
        super(cause);
    }    
}
