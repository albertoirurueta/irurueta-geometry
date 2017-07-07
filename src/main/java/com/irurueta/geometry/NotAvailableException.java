/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.NotAvailableException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date November 5, 2012
 */
package com.irurueta.geometry;

/**
 * Thrown when something cannot be retrieved because it is not yet available
 */
public class NotAvailableException extends GeometryException{
    /**
     * Constructor
     */
    public NotAvailableException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public NotAvailableException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public NotAvailableException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public NotAvailableException(Throwable cause){
        super(cause);
    }    
}
