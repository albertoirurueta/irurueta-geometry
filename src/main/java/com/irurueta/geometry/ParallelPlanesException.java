/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.ParallelPlanesException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 15, 2012
 */
package com.irurueta.geometry;

/**
 *  Exception raised when providing parallel planes
 */
public class ParallelPlanesException extends GeometryException{
    /**
     * Constructor
     */
    public ParallelPlanesException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public ParallelPlanesException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public ParallelPlanesException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public ParallelPlanesException(Throwable cause){
        super(cause);
    }        
}
