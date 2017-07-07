/**
 * @file
 * This file contains implementation of
 * ocm.irurueta.geometry.DualQuadricNotAvailableException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date August 13, 2012
 */
package com.irurueta.geometry;

/**
 *  Exception raised when a dual-quadric cannot be computed
 */
public class DualQuadricNotAvailableException extends GeometryException {
    
    /**
     * Constructor
     */
    public DualQuadricNotAvailableException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public DualQuadricNotAvailableException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public DualQuadricNotAvailableException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public DualQuadricNotAvailableException(Throwable cause){
        super(cause);
    }
}
