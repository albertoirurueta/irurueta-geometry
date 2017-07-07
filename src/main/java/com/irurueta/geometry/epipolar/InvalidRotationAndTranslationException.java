/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.InvalidRotationAndTranslationException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 25, 2015
 */
package com.irurueta.geometry.epipolar;

/**
 * Raised if given rotation or translation are not valid to define an essential
 * matrix
 */
public class InvalidRotationAndTranslationException extends EpipolarException{
    
    /**
     * Constructor
     */
    public InvalidRotationAndTranslationException(){
        super();
    }
    
    /**
     * Constructor with String containing message
     * @param message message indicating the cause of the exception
     */
    public InvalidRotationAndTranslationException(String message){
        super(message);
    }
    
    /**
     * Constructor with message and cause
     * @param message message describing the cause of the exception
     * @param cause instance containing the cause of the exception
     */
    public InvalidRotationAndTranslationException(String message, 
            Throwable cause){
        super(message, cause);
    }
        
    /**
     * Constructor with cause
     * @param cause instance containing the cause of the exception
     */
    public InvalidRotationAndTranslationException(Throwable cause){
        super(cause);
    }
}
