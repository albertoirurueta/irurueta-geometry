/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.NotLocusException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 8, 2012
 */
package com.irurueta.geometry;

/**
 * Raised when using objects (e.g. points) that are not locus of another 
 * geometric structure
 */
class NotLocusException extends GeometryException {
    
    /**
     * Constructor
     */
    public NotLocusException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public NotLocusException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public NotLocusException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public NotLocusException(Throwable cause){
        super(cause);
    }    
}
