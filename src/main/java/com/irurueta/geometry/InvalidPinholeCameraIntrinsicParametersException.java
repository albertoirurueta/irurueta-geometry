/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.InvalidPinholeCameraIntrinsicParametersException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date November 5, 2012
 */
package com.irurueta.geometry;

/**
 * Raised when provided pinhole camera intrinsic parameters matrix is not valid
 * @author albertoirurueta
 */
public class InvalidPinholeCameraIntrinsicParametersException 
    extends GeometryException {
    
    /**
     * Constructor
     */
    public InvalidPinholeCameraIntrinsicParametersException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message Message indicating the cause of the exception
     */
    public InvalidPinholeCameraIntrinsicParametersException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message Message describing the cause of the exception
     * @param cause Instance containing the cause of the exception
     */
    public InvalidPinholeCameraIntrinsicParametersException(String message, 
            Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause Instance containing the cause of the exception
     */
    public InvalidPinholeCameraIntrinsicParametersException(Throwable cause){
        super(cause);
    }        
}
