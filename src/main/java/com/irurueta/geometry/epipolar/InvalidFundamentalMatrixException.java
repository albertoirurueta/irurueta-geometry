/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.InvalidFundamentalMatrixException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 12, 2015
 */
package com.irurueta.geometry.epipolar;

/**
 * Raised if a given matrix is not a valid fundamental matrix (i.e. 3x3 matrix
 * having rank 2)
 */
public class InvalidFundamentalMatrixException extends EpipolarException{
    
    /**
     * Constructor
     */
    public InvalidFundamentalMatrixException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message message indicating the cause of the exception
     */
    public InvalidFundamentalMatrixException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message message describing the cause of the exception
     * @param cause instance containing the cause of the exception
     */
    public InvalidFundamentalMatrixException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause instance containing the cause of the exception
     */
    public InvalidFundamentalMatrixException(Throwable cause){
        super(cause);
    }
}
