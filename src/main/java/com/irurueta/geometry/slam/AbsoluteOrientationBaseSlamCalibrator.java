/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.slam.AbsoluteOrientationBaseSlamCalibrator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 8, 016.
 */
package com.irurueta.geometry.slam;

import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import java.io.Serializable;

/**
 * Base class for estimating mean and covariance of noise in control values
 * when the system state is held constant (only noise is provided as control
 * input).
 * This subclass of BaseSlamCalibrator is used for slam estimator taking into
 * account absolute orientation.
 * @param <D> type of calibration data.
 */
public abstract class AbsoluteOrientationBaseSlamCalibrator<D extends BaseCalibrationData> extends 
        BaseSlamCalibrator<D> implements Serializable {
    
    /**
     * Timestamp expressed in nanoseconds since the epoch time of the last
     * sample containing absolute orientation.
     */
    protected long mOrientationTimestampNanos = -1;
    
    /**
     * Number of orientation samples accumulated since last full sample.
     */
    protected int mAccumulatedOrientationSamples = 0;

    /**
     * Average orientation accumulated since last full sample.
     */
    protected Quaternion mAccumulatedOrientation;
        
    /**
     * termporary quaternion. For memory reuse.
     */
    private Quaternion mTempQ;
    
    /**
     * Constructor.
     * @param sampleLength sample length of control values used during 
     * prediction stage in SLAM estimator.
     * @throws IllegalArgumentException if sample length is less than 1.
     */
    public AbsoluteOrientationBaseSlamCalibrator(int sampleLength) 
            throws IllegalArgumentException {
        super(sampleLength);
        mAccumulatedOrientation = new Quaternion();
    }
    
    /**
     * Gets timestamp expressed in nanoseconds since the epoch time of the last
     * orientation sample, or -1 if no sample has been set yet.
     * @return timestamp expressed in nanoseconds since the epoch time of the
     * last orientation sample, or -1.
     */
    public long getOrientationTimestampNanos() {
        return mOrientationTimestampNanos;
    }
    
    /**
     * Gets average orientation accumulated since last full sample.
     * @return orientation accumulated since last full sample.
     */
    public Rotation3D getAccumulatedOrientation() {
        return mAccumulatedOrientation.toQuaternion();
    }
    
    /**
     * Gets average orientation accumulated since last full sample.
     * @param result instance where orientation accumulated since last full 
     * sample will be stored.
     */
    public void getAccumulatedOrientation(Rotation3D result) {
        result.fromRotation(mAccumulatedOrientation);
    }
        
    /**
     * Gets number of orientation samples accumulated since last full sample.
     * @return number of orientation samples accumulated since last full sample.
     */
    public int getAccumulatedOrientationSamples() {
        return mAccumulatedOrientationSamples;
    }
    
    /**
     * Indicates whether the orientation sample has been received since last
     * full sample (accelerometer + gyroscope + orientation).
     * @return true if orientation sample has been received, false otherwise.
     */
    public boolean isOrientationSampleReceived() {
        return mAccumulatedOrientationSamples > 0;
    }
        
    /**
     * Indicates whether a full sample (accelerometer + gyroscope + 
     * magnetic field) has been received or not.
     * @return true if full sample has been received, false otherwise.
     */
    @Override
    public boolean isFullSampleAvailable() {
        return super.isFullSampleAvailable() && isOrientationSampleReceived();
    }
 
    /**
     * Provides a new orientation sample.
     * If accumulation is enabled, samples are averaged until a full sample is 
     * received.
     * When a full sample (acceleromteter + gyroscope + orientation) is 
     * received, internal state gets also updated.
     * @param timestamp timestamp of accelerometer sample since epoch time and
     * expressed in nanoseconds.
     * @param orientation new orientation.
     */
    public void updateOrientationSample(long timestamp, 
            Rotation3D orientation) {
        if (!isFullSampleAvailable()) {
            mOrientationTimestampNanos = timestamp;
            if (isAccumulationEnabled() && isOrientationSampleReceived()) {
                //accumulation enabled
                int nextSamples = mAccumulatedOrientationSamples + 1;
                
                double accumA = mAccumulatedOrientation.getA();
                double accumB = mAccumulatedOrientation.getB();
                double accumC = mAccumulatedOrientation.getC();
                double accumD = mAccumulatedOrientation.getD();
                
                if(mTempQ == null) {
                    mTempQ = new Quaternion();
                }                
                mTempQ.fromRotation(orientation);
                mTempQ.normalize();
                double a = mTempQ.getA();
                double b = mTempQ.getB();
                double c = mTempQ.getC();
                double d = mTempQ.getD();
                
                accumA = (accumA * mAccumulatedOrientationSamples + a) / 
                        nextSamples;
                accumB = (accumB * mAccumulatedOrientationSamples + b) / 
                        nextSamples;
                accumC = (accumC * mAccumulatedOrientationSamples + c) / 
                        nextSamples;
                accumD = (accumD * mAccumulatedOrientationSamples + d) /
                        nextSamples;
                
                mAccumulatedOrientation.setA(accumA);
                mAccumulatedOrientation.setB(accumB);
                mAccumulatedOrientation.setC(accumC);
                mAccumulatedOrientation.setD(accumD);
                mAccumulatedOrientationSamples = nextSamples;                
            } else {
                //accumulation disabled
                mAccumulatedOrientation.fromRotation(orientation);
                mAccumulatedOrientationSamples++;
            }            
            notifyFullSampleAndResetSampleReceive();
        }
    }
    
    /**
     * Gets most recent timestamp of received partial samples (accelerometer,
     * gyroscope or magnetic field).
     * @return most recent timestamp of received partial sample.
     */
    @Override
    public long getMostRecentTimestampNanos() {
        long mostRecent = super.getMostRecentTimestampNanos();
        return mostRecent > mOrientationTimestampNanos ? mostRecent :
                mOrientationTimestampNanos;
    }
    
    /**
     * Notifies that a full sample has been received and resets flags indicating
     * whether partial samples have been received.
     */
    @Override
    protected void notifyFullSampleAndResetSampleReceive() {
        if(isFullSampleAvailable()) {
            processFullSample();
            mAccumulatedAccelerometerSamples = mAccumulatedGyroscopeSamples =
                    mAccumulatedOrientationSamples = 0;
        }
    }    
}
