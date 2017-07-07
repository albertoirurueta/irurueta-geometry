/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.NonSymmetricMatrixException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 1, 2012
 */
package com.irurueta.geometry;

/**
 *  Exception raised when a non-symmetric matrix is used
 */
public class NonSymmetricMatrixException extends GeometryException {
    
    /**
     * Constructor
     */
    public NonSymmetricMatrixException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public NonSymmetricMatrixException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public NonSymmetricMatrixException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public NonSymmetricMatrixException(Throwable cause){
        super(cause);
    }
}
