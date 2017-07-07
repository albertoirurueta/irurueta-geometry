/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.DualConicNotAvailableException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 2, 2012
 */
package com.irurueta.geometry;

/**
 * Exception raised when a dual-conic cannot be computed
 */
public class DualConicNotAvailableException extends GeometryException {
    /**
     * Constructor
     */
    public DualConicNotAvailableException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public DualConicNotAvailableException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public DualConicNotAvailableException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public DualConicNotAvailableException(Throwable cause){
        super(cause);
    }
}
