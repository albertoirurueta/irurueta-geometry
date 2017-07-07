/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.ParallelVectorsException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 17, 2012
 */
package com.irurueta.geometry;

/**
 * Raised when provided vectors are parallel
 */
public class ParallelVectorsException extends GeometryException {
    /**
     * Constructor
     */
    public ParallelVectorsException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public ParallelVectorsException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public ParallelVectorsException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public ParallelVectorsException(Throwable cause){
        super(cause);
    }        
}
