/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.BaseAbsoluteOrientationSlamTwoViewsSparseReconstructor
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 19, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.slam.AbsoluteOrientationBaseSlamEstimator;
import java.util.ArrayList;
import java.util.List;

/**
 * Base classin charge of estimating cameras and 3D reconstructed points from
 * sparse image point correspondences in two views and also in charge of 
 * estimating overall scene scale and absolute orientation by means of SLAM 
 * (Simultaneous Location And Mapping) using data obtained from sensors like 
 * accelerometers or gyroscopes.
 * NOTE: absolute orientation slam estimators are not very accurate during 
 * estimation of the orientation state, for that reason we take into account
 * the initial orientation
 * @param <C> type of configuration.
 * @param <R> type of reconstructor.
 * @param <S> type of SLAM estimator.
 */
public abstract class BaseAbsoluteOrientationSlamTwoViewsSparseReconstructor<
        C extends BaseSlamTwoViewsSparseReconstructorConfiguration,
        R extends BaseSlamTwoViewsSparseReconstructor,
        S extends AbsoluteOrientationBaseSlamEstimator> extends 
        BaseSlamTwoViewsSparseReconstructor<C, R, S> {
    
    /**
     * First sample of orientation received.
     */
    protected Rotation3D mFirstOrientation;
    
    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not 
     * provided.
     */
    public BaseAbsoluteOrientationSlamTwoViewsSparseReconstructor(
            C configuration, 
            BaseTwoViewsSparseReconstructorListener<R> listener) 
            throws NullPointerException {
        super(configuration, listener);
    }    
    
    /**
     * Provides a new orientation sample to update SLAM estimator.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of accelerometer sample since epoch time and
     * expressed in nanoseconds.
     * @param orientation new orientation.
     */    
    public void updateOrientationSample(long timestamp,
            Rotation3D orientation) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateOrientationSample(timestamp, orientation);
        }
        if (mFirstOrientation == null) {
            //make a copy of orientation
            mFirstOrientation = orientation.toQuaternion();
        }
    }
    
    /**
     * Updates scene scale and orientation using SLAM data.
     * @return true if scale was successfully updated, false otherwise.
     */
    protected boolean updateScaleAndOrientation() {
            
        //obtain baseline (camera separation from slam estimator data
        double posX = mSlamEstimator.getStatePositionX();
        double posY = mSlamEstimator.getStatePositionY();
        double posZ = mSlamEstimator.getStatePositionZ();
            
        //to estimate baseline, we assume that first camera is placed at
        //world origin
        double baseline = Math.sqrt(posX*posX + posY*posY + posZ*posZ);
            
        try {
            PinholeCamera camera1 = mEstimatedCamera1.getCamera();
            PinholeCamera camera2 = mEstimatedCamera2.getCamera();
            
            camera1.decompose();
            camera2.decompose();
            
            Point3D center1 = camera1.getCameraCenter();
            Point3D center2 = camera2.getCameraCenter();
                
            //R1' = R1*Rdiff
            //Rdiff = R1^T*R1'
                
            //where R1' is the desired orientation (obtained by sampling a
            //sensor)
            //and R1 is always the identity for the 1st camera.
            //Hence R1' = Rdiff
                
            //t1' is the desired translation which is zero for the 1st
            //camera.
                
            //We want: P1' = K*[R1' t1'] = K*[R1' 0]
            //And we have P1 = K[I 0]
                
            //We need a transformation T so that:
            //P1' = P1*T^-1 = K[I 0][R1' 0]
            //                      [0   1]
                
            //Hence: T^-1 = [R1' 0]
            //              [0   1]
                
            //or T = [R1'^T 0]
            //       [0     1]
                
            //because we are also applying a transformation of scale s,
            //the combination of both transformations is
            //T = [s*R1'^T 0]
            //    [0       1]
                
            Rotation3D r = mFirstOrientation.inverseRotationAndReturnNew();
            
            double estimatedBaseline = center1.distanceTo(center2);
            
            double scale = baseline / estimatedBaseline;
            
            MetricTransformation3D scaleAndOrientationTransformation =
                    new MetricTransformation3D(scale);
            scaleAndOrientationTransformation.setRotation(r);
            
            //update scale of cameras
            scaleAndOrientationTransformation.transform(camera1);
            scaleAndOrientationTransformation.transform(camera2);
            
            mEstimatedCamera1.setCamera(camera1);
            mEstimatedCamera2.setCamera(camera2);
            
            //update scale of reconstructed points
            int numPoints = mReconstructedPoints.size();
            List<Point3D> reconstructedPoints3D = new ArrayList<Point3D>();
            for (int i = 0; i < numPoints; i++) {
                reconstructedPoints3D.add(mReconstructedPoints.get(i).
                        getPoint());
            }
            
            scaleAndOrientationTransformation.transformAndOverwritePoints(
                    reconstructedPoints3D);
            
            //set scaled points into result
            for (int i = 0; i < numPoints; i++) {
                mReconstructedPoints.get(i).setPoint(
                        reconstructedPoints3D.get(i));
            }

            return true;
        } catch (Exception e) {
            mFailed = true;
            mListener.onFail((R)this);

            return false;
        }
    }    
}
