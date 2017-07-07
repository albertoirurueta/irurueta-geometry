/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.CoincidentLinesException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 9, 2012
 */
package com.irurueta.geometry;

/**
 *  Raised when providing lines which are assumed to be equal
 */
public class CoincidentLinesException extends GeometryException {
    
    /**
     * Constructor
     */
    public CoincidentLinesException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public CoincidentLinesException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public CoincidentLinesException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public CoincidentLinesException(Throwable cause){
        super(cause);
    }
}
