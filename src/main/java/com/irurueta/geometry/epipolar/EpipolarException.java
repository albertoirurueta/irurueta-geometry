/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.EpipolarException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 12, 2015
 */
package com.irurueta.geometry.epipolar;

import com.irurueta.geometry.GeometryException;

/**
 * Base exception for all exceptions in the com.irurueta.geometry.epipolar
 * package
 */
public class EpipolarException extends GeometryException{
    
    /**
     * Constructor
     */
    public EpipolarException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message message indicating the cause of the exception
     */
    public EpipolarException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message message describing the cause of the exception
     * @param cause instance containing the cause of the exception
     */
    public EpipolarException(String message, Throwable cause){
        super(message, cause);
    }
    
    /**
     * Constructor with cause
     * @param cause instance containing the cause of the exception
     */
    public EpipolarException(Throwable cause){
        super(cause);
    }
}
