/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.InvalidPairOfIntrinsicParametersException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 25, 2015
 */
package com.irurueta.geometry.epipolar;

/**
 * Raised when providing an invalid pair of intrinsic parameters to define an
 * essential matrix
 */
public class InvalidPairOfIntrinsicParametersException extends 
        EpipolarException{
    
    /**
     * Constructor
     */
    public InvalidPairOfIntrinsicParametersException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message message indicating the cause of the exception
     */
    public InvalidPairOfIntrinsicParametersException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message message describing the cause of the exception
     * @param cause instance containing the cause of the exception
     */
    public InvalidPairOfIntrinsicParametersException(String message, 
            Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause instance containing the cause of the exception
     */
    public InvalidPairOfIntrinsicParametersException(Throwable cause){
        super(cause);
    }
}
