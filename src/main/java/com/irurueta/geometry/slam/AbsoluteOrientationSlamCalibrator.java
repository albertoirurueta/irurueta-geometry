/**
 * @file
 * This file contains implementation of
 * com.irurueta.geomtry.slam.AbsoluteOrientationSlamCalibrator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 8, 2016.
 */
package com.irurueta.geometry.slam;

import com.irurueta.geometry.Quaternion;
import java.io.Serializable;

/**
 * Processes data to estimate calibration for absolute orientation SLAM 
 * estimator.
 * This class must be used while gathering data for a system being kept constant
 * (under no motion).
 */
public class AbsoluteOrientationSlamCalibrator extends 
        AbsoluteOrientationBaseSlamCalibrator<AbsoluteOrientationSlamCalibrationData> 
        implements Serializable {
    
    /**
     * Last sample of linear acceleration along x-axis.
     */
    private double mLastAccelerationX;
    
    /**
     * Last sample of linear acceleration along y-axis.
     */
    private double mLastAccelerationY;
    
    /**
     * Last sample of linear acceleration along z-axis.
     */
    private double mLastAccelerationZ;
    
    /**
     * Last sample of angular speed along x-axis.
     */
    private double mLastAngularSpeedX;
    
    /**
     * Last sample of angular speed along y-axis.
     */
    private double mLastAngularSpeedY;
    
    /**
     * Last sample of angular speed along z-axis.
     */
    private double mLastAngularSpeedZ;
    
    /**
     * Last timestamp of a full sample expressed in nanoseconds since the epoch 
     * time..
     */
    private long mLastTimestampNanos = -1;
    
    /**
     * Last sample of absolute orientation.
     */
    private Quaternion mLastOrientation;
    
    /**
     * Variation of orientation respect to last sample.
     */
    private Quaternion mDeltaOrientation;
    
    /**
     * Constructor.
     */
    public AbsoluteOrientationSlamCalibrator() {
        super(AbsoluteOrientationSlamEstimator.CONTROL_LENGTH);
        mLastOrientation = new Quaternion();
        mDeltaOrientation = new Quaternion();   
    }
    
    /**
     * Resets calibrator.
     */
    @Override
    public void reset() {
        super.reset();
        mLastOrientation = new Quaternion();
        mLastAccelerationX = mLastAccelerationY = mLastAccelerationZ =
                mLastAngularSpeedX = mLastAngularSpeedY = mLastAngularSpeedZ = 
                0.0;
        mLastTimestampNanos = -1;
    }    
    
    /**
     * Obtains the number of state parameters in associated SLAM estimator.
     * @return number of state parameters.
     */
    @Override
    protected int getEstimatorStateLength() {
        return AbsoluteOrientationSlamEstimator.STATE_LENGTH;
    }   
    
    /**
     * Gets a new instance containing calibration data estimated by this
     * calibrator.
     * @return a new calibration data instance.
     */        
    @Override
    public AbsoluteOrientationSlamCalibrationData getCalibrationData() {
        AbsoluteOrientationSlamCalibrationData result = 
                new AbsoluteOrientationSlamCalibrationData();
        getCalibrationData(result);
        return result;
    }    
    
    /**
     * Processes a full sample of accelerometer and gyroscope data to compute
     * statistics such as mean and covariance of variations.
     */
    @Override
    protected void processFullSample() {
        if(mListener != null) {
            mListener.onFullSampleReceived(this);
        }
        
        long timestamp = getMostRecentTimestampNanos();
        if(mLastTimestampNanos < 0) {                        
            //first time receiving control data we cannot determine its 
            //variation
            mLastOrientation.fromQuaternion(mAccumulatedOrientation);
            
            mLastAccelerationX = mAccumulatedAccelerationSampleX;
            mLastAccelerationY = mAccumulatedAccelerationSampleY;
            mLastAccelerationZ = mAccumulatedAccelerationSampleZ;
            
            mLastAngularSpeedX = mAccumulatedAngularSpeedSampleX;
            mLastAngularSpeedY = mAccumulatedAngularSpeedSampleY;
            mLastAngularSpeedZ = mAccumulatedAngularSpeedSampleZ;
            
            mLastTimestampNanos = timestamp;
            
            if(mListener != null) {
                mListener.onFullSampleProcessed(this);
            }
            
            return;
        }
        
        mAccumulatedOrientation.normalize();
        
        mLastOrientation.inverse(mDeltaOrientation);
        mDeltaOrientation.combine(mAccumulatedOrientation);
        mDeltaOrientation.normalize();        
        
        double deltaAccelerationX = mAccumulatedAccelerationSampleX - 
                mLastAccelerationX;
        double deltaAccelerationY = mAccumulatedAccelerationSampleY -
                mLastAccelerationY;
        double deltaAccelerationZ = mAccumulatedAccelerationSampleZ -
                mLastAccelerationZ;
        double deltaAngularSpeedX = mAccumulatedAngularSpeedSampleX -
                mLastAngularSpeedX;
        double deltaAngularSpeedY = mAccumulatedAngularSpeedSampleY -
                mLastAngularSpeedY;
        double deltaAngularSpeedZ = mAccumulatedAngularSpeedSampleZ -
                mLastAngularSpeedZ;
        
        mSample[0] = mDeltaOrientation.getA();
        mSample[1] = mDeltaOrientation.getB();
        mSample[2] = mDeltaOrientation.getC();
        mSample[3] = mDeltaOrientation.getD();        
        mSample[4] = mSample[5] = mSample[6] = 0.0;
        
        mSample[7] = deltaAccelerationX;
        mSample[8] = deltaAccelerationY;
        mSample[9] = deltaAccelerationZ;
        
        mSample[10] = deltaAngularSpeedX;
        mSample[11] = deltaAngularSpeedY;
        mSample[12] = deltaAngularSpeedZ;
        updateSample();
        
        mLastOrientation.combine(mDeltaOrientation);
        mLastAccelerationX = mAccumulatedAccelerationSampleX;
        mLastAccelerationY = mAccumulatedAccelerationSampleY;
        mLastAccelerationZ = mAccumulatedAccelerationSampleZ;
        
        mLastAngularSpeedX = mAccumulatedAngularSpeedSampleX;
        mLastAngularSpeedY = mAccumulatedAngularSpeedSampleY;
        mLastAngularSpeedZ = mAccumulatedAngularSpeedSampleZ;        

        mLastTimestampNanos = timestamp;
        
        if(mListener != null) {
            mListener.onFullSampleProcessed(this);
        }
    }    
}
