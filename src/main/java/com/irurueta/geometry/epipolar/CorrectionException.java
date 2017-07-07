/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.CorrectionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 27, 2015
 */
package com.irurueta.geometry.epipolar;

/**
 * Exception raised when point correction fails when trying to be fit into a 
 * given epipolar geometry
 */
public class CorrectionException extends EpipolarException{
    
    /**
     * Constructor
     */
    public CorrectionException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message message indicating the cause of the exception
     */
    public CorrectionException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message message describing the cause of the exception
     * @param cause instance containing the cause of the exception
     */
    public CorrectionException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause instance containing the cause of the exception
     */
    public CorrectionException(Throwable cause){
        super(cause);
    }
}
