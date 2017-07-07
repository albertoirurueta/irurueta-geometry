/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.CalibrationException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 6, 2015
 */
package com.irurueta.geometry.calib3d;

import com.irurueta.geometry.GeometryException;

/**
 * Base exception class for calibration package.
 */
public class CalibrationException extends GeometryException{
    /**
     * Constructor
     */
    public CalibrationException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public CalibrationException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public CalibrationException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public CalibrationException(Throwable cause){
        super(cause);
    }                
}
