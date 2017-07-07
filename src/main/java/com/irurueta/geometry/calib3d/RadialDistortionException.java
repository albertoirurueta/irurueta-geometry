/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.RadialDistortionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 23, 2015
 */
package com.irurueta.geometry.calib3d;

/**
 * Raised when an error occurs while using a RadialDistortion
 */
public class RadialDistortionException extends DistortionException{
    /**
     * Constructor
     */
    public RadialDistortionException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public RadialDistortionException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public RadialDistortionException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public RadialDistortionException(Throwable cause){
        super(cause);
    }        
}
