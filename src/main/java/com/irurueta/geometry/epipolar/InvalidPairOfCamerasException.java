/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.InvalidPairOfCamerasException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 12, 2015
 */
package com.irurueta.geometry.epipolar;

/**
 * Raised if a given pair of cameras cannot span a valid epipolar geometry,
 * typically because they are set in a degenerate configuration
 */
public class InvalidPairOfCamerasException extends EpipolarException{
    
    /**
     * Constructor
     */
    public InvalidPairOfCamerasException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message message indicating the cause of the exception
     */
    public InvalidPairOfCamerasException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message message describing the cause of the exception
     * @param cause instance containing the cause of the exception
     */
    public InvalidPairOfCamerasException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause instance containing the cause of the exception
     */
    public InvalidPairOfCamerasException(Throwable cause){
        super(cause);
    }
}
