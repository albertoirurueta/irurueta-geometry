/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.epipolar.estimators.FundamentalMatrixEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 12, 2015
 */
package com.irurueta.geometry.epipolar.estimators;

import com.irurueta.geometry.epipolar.EpipolarException;

/**
 * Exception raised if fundamental matrix estimation fails.
 */
public class FundamentalMatrixEstimatorException extends EpipolarException {
    
    /**
     * Constructor.
     */
    public FundamentalMatrixEstimatorException() {
        super();
    }
    
    /**
     * Constructor with String containing message.
     * @param message message indicating the cause of the exception.
     */
    public FundamentalMatrixEstimatorException(String message) {
        super(message);
    }
    
    /**
     * Constructor with message and cause.
     * @param message message describing the cause of the exception.
     * @param cause  instance containing the cause of the exception.
     */
    public FundamentalMatrixEstimatorException(String message, 
            Throwable cause) {
        super(message, cause);
    }
    
    /**
     * Constructor with cause.
     * @param cause instance containing the cause of the exception.
     */
    public FundamentalMatrixEstimatorException(Throwable cause) {
        super(cause);
    }
}
