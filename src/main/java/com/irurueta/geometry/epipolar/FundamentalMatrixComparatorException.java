/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.FundamentalMatrixComparatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 28, 2015
 */
package com.irurueta.geometry.epipolar;

/**
 * Raised when fundamental matrices comparison fails
 */
public class FundamentalMatrixComparatorException extends EpipolarException{
    
    /**
     * Constructor
     */
    public FundamentalMatrixComparatorException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message message indicating the cause of the exception
     */
    public FundamentalMatrixComparatorException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message message describing the cause of the exception
     * @param cause instance containing the cause of the exception
     */
    public FundamentalMatrixComparatorException(String message, 
            Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause instance containing the cause of the exception
     */
    public FundamentalMatrixComparatorException(Throwable cause){
        super(cause);
    }
}
