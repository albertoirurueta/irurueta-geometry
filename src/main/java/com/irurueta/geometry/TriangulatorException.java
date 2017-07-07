/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.TriangulatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 17, 2012
 */
package com.irurueta.geometry;

/**
 * Raised when a polygon cannot be triangulated
 */
public class TriangulatorException extends GeometryException{

    /**
     * Constructor
     */
    public TriangulatorException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public TriangulatorException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public TriangulatorException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public TriangulatorException(Throwable cause){
        super(cause);
    }        
}
