/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.estimators.HomographyDecomposerException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date May 24, 2017.
 */
package com.irurueta.geometry.epipolar.estimators;

import com.irurueta.geometry.epipolar.EpipolarException;

/**
 * Exception raised if homography decomposition fails.
 */
public class HomographyDecomposerException extends EpipolarException {
    
    /**
     * Constructor.
     */
    public HomographyDecomposerException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public HomographyDecomposerException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause instance containing the cause of the exception.
     */
    public HomographyDecomposerException(String message, Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public HomographyDecomposerException(Throwable cause) {
        super(cause);
    }
}
