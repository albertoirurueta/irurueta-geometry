/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.CoincidentPlanesException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 9, 2012
 */
package com.irurueta.geometry;

/**
 *  Raised when providing planes which are assumed to be equal
 */
public class CoincidentPlanesException extends GeometryException {
    /**
     * Constructor
     */
    public CoincidentPlanesException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public CoincidentPlanesException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public CoincidentPlanesException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public CoincidentPlanesException(Throwable cause){
        super(cause);
    }    
}
