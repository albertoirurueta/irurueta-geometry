/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.BaseSlamTwoViewsSparseReconstructor
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 17, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.slam.BaseCalibrationData;
import com.irurueta.geometry.slam.BaseSlamEstimator;
import java.util.ArrayList;
import java.util.List;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from
 * sparse image point correspondences in two views and also in charge of 
 * estimating overall scene scale by means of SLAM (Simultaneous Location And
 * Mapping) using data obtained from sensors like accelerometers or gyroscopes.
 * @param <C> type of configuration.
 * @param <R> type of reconstructor.
 * @param <S> type of SLAM estimator.
 */
public class BaseSlamTwoViewsSparseReconstructor<
        C extends BaseSlamTwoViewsSparseReconstructorConfiguration,
        R extends BaseSlamTwoViewsSparseReconstructor,
        S extends BaseSlamEstimator> extends 
        BaseTwoViewsSparseReconstructor<C, R> {
    
    /**
     * Slam estimator to estimate position, speed, orientation using 
     * accelerometer and gyroscope data.
     */
    protected S mSlamEstimator;
    
    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not 
     * provided.
     */
    public BaseSlamTwoViewsSparseReconstructor(C configuration, 
            BaseTwoViewsSparseReconstructorListener<R> listener) 
            throws NullPointerException {
        super(configuration, listener);
    }
    
    /**
     * Provides a new accelerometer sample to update SLAM estimation.
     * This method must be called whenever the accelerometer sensor receives new 
     * data.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of accelerometer sample since epoch time and 
     * expressed in nanoseconds.
     * @param accelerationX linear acceleration along x-axis expressed in meters
     * per squared second (m/s^2).
     * @param accelerationY linear acceleration along y-axis expressed in meters
     * per squared second (m/s^2).
     * @param accelerationZ linear acceleration along z-axis expressed in meters
     * per squared second (m/s^2).
     */
    public void updateAccelerometerSample(long timestamp, float accelerationX,
            float accelerationY, float accelerationZ) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateAccelerometerSample(timestamp, accelerationX, 
                    accelerationY, accelerationZ);
        }
    }
    
    /**
     * Provides a new accelerometer sample to update SLAM estimation.
     * This method must be called whenever the accelerometer sensor receives new
     * data.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of accelerometer sample since epoch time and
     * expressed in nanoseconds.
     * @param data array containing x,y,z components of linear acceleration
     * expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void updateAccelerometerSample(long timestamp, float[] data) 
            throws IllegalArgumentException {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateAccelerometerSample(timestamp, data);
        }
    }
    
    /**
     * Provides a new gyroscope sample to update SLAM estimation.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of gyroscope sample since epoch time and
     * expressed in nanoseconds.
     * @param angularSpeedX angular speed of rotation along x-axis expressed in
     * radians per second (rad/s).
     * @param angularSpeedY angular speed of rotation along y-axis expressed in
     * radians per second (rad/s).
     * @param angularSpeedZ angular speed of rotation along z-axis expressed in
     * radians per second (rad/s).
     */
    public void updateGyroscopeSample(long timestamp, float angularSpeedX, 
            float angularSpeedY, float angularSpeedZ) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateGyroscopeSample(timestamp, angularSpeedX, 
                    angularSpeedY, angularSpeedZ);
        }
    }
    
    /**
     * Provies a new gyroscope sample to update SLAM estimation.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of gyroscope sample since epoch time and 
     * expressed in nanoseconds.
     * @param data angular speed of rotation along x,y,z axes expressed in 
     * radians per second (rad/s).
     * @throws IllegalArgumentException if provided array does not have length 
     * 3.
     */
    public void updateGyroscopeSample(long timestamp, float[] data)
            throws IllegalArgumentException {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateGyroscopeSample(timestamp, data);
        }        
    }
    
    /**
     * Set ups calibration data on SLAM estimator if available.
     */
    protected void setUpCalibrationData() {
        BaseCalibrationData calibrationData = 
                mConfiguration.getCalibrationData();
        if (calibrationData != null) {
            mSlamEstimator.setCalibrationData(calibrationData);
        }
    }
    
    /**
     * Update scene scale using SLAM data.
     * @throws CancelledReconstructionException if reconstruction is cancelled.
     */
    protected void updateScale() throws CancelledReconstructionException {
        if (!isRunning() && !isCancelled() && !hasFailed()) {
            //obtain baseline (camera separation from slam estimator data
            double posX = mSlamEstimator.getStatePositionX();
            double posY = mSlamEstimator.getStatePositionY();
            double posZ = mSlamEstimator.getStatePositionZ();
            
            //to estimate baseline, we assume that first camera is placed at 
            //world origin
            double baseline = Math.sqrt(posX*posX + posY*posY + posZ*posZ);
            
            try{
                PinholeCamera camera1 = mEstimatedCamera1.getCamera();
                PinholeCamera camera2 = mEstimatedCamera2.getCamera();
            
                camera1.decompose();
                camera2.decompose();
            
                Point3D center1 = camera1.getCameraCenter();
                Point3D center2 = camera2.getCameraCenter();
            
                double estimatedBaseline = center1.distanceTo(center2);
            
                double scale = baseline / estimatedBaseline;
            
                MetricTransformation3D scaleTransformation = 
                        new MetricTransformation3D(scale);
            
                //update scale of cameras
                scaleTransformation.transform(camera1);
                scaleTransformation.transform(camera2);
            
                mEstimatedCamera1.setCamera(camera1);
                mEstimatedCamera2.setCamera(camera2);
            
                //update scale of reconstructed points
                int numPoints = mReconstructedPoints.size();
                List<Point3D> reconstructedPoints3D = new ArrayList<Point3D>();
                for (int i = 0; i < numPoints; i++) {
                    reconstructedPoints3D.add(mReconstructedPoints.get(i).
                            getPoint());
                }
            
                scaleTransformation.transformAndOverwritePoints(
                        reconstructedPoints3D);
            
                //set scaled points into result
                for (int i = 0; i < numPoints; i++) {
                    mReconstructedPoints.get(i).setPoint(
                            reconstructedPoints3D.get(i));
                }
            } catch (Exception e) {
                throw new CancelledReconstructionException(e);
            }
        }        
    }    
}
