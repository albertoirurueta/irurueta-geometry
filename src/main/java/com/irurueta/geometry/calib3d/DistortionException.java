/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.DistortionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 30, 2015
 */
package com.irurueta.geometry.calib3d;

import com.irurueta.geometry.GeometryException;

/**
 * Raised when an error occurs while using a Distortion
 * @author albertoirurueta
 */
public class DistortionException extends GeometryException{
    /**
     * Constructor
     */
    public DistortionException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public DistortionException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public DistortionException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public DistortionException(Throwable cause){
        super(cause);
    }            
}
