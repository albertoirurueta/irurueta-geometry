/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.InvalidEssentialMatrixException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 25, 2015
 */
package com.irurueta.geometry.epipolar;

/**
 * Raised when an essential matrix is not well defined
 */
public class InvalidEssentialMatrixException extends 
        InvalidFundamentalMatrixException{
    
    /**
     * Constructor
     */
    public InvalidEssentialMatrixException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message message indicating the cause of the exception
     */
    public InvalidEssentialMatrixException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message message describing the cause of the exception
     * @param cause instance containing the cause of the exception
     */
    public InvalidEssentialMatrixException(String message, 
            Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause instance containing the cause of the exception
     */
    public InvalidEssentialMatrixException(Throwable cause){
        super(cause);
    }    
}
