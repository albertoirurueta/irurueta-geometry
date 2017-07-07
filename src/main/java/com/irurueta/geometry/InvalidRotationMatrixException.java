/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.InvalidRotationMatrixException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date August 15, 2012
 */
package com.irurueta.geometry;

/**
 *  Exception raised if provided rotation matrix is not orthonormal
 */
public class InvalidRotationMatrixException extends GeometryException {
    /**
     * Constructor
     */
    public InvalidRotationMatrixException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public InvalidRotationMatrixException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public InvalidRotationMatrixException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public InvalidRotationMatrixException(Throwable cause){
        super(cause);
    }    
}
