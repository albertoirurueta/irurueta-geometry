/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.sfm.DualAbsoluteQuadricInitialCamerasEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 4, 2017.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.ProjectiveTransformation3D;
import com.irurueta.geometry.calib3d.DualAbsoluteQuadric;
import com.irurueta.geometry.calib3d.estimators.DualAbsoluteQuadricEstimator;
import com.irurueta.geometry.calib3d.estimators.LMSEDualAbsoluteQuadricEstimator;
import com.irurueta.geometry.epipolar.FundamentalMatrix;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import java.util.ArrayList;
import java.util.List;

/**
 * Estimates an initial pair of cameras in the metric stratum (up to an 
 * arbitrary scale) using a given fundamental matrix and assuming zero skewness
 * and principal point at the origin for the intrinsic parameters of estimated 
 * cameras.
 * Aspect ratio can be configured but by default it is assumed to be 1.0.
 */
public class DualAbsoluteQuadricInitialCamerasEstimator extends 
        InitialCamerasEstimator {
    
    /**
     * Aspect ratio of intrinsic parameters of cameras. 
     * Typically this value is 1.0 if vertical coordinates increase upwards,
     * or -1.0 if it is the opposite.
     */
    private double mAspectRatio = DualAbsoluteQuadricEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO;
    
    /**
     * Constructor.
     */
    public DualAbsoluteQuadricInitialCamerasEstimator() {
        super();
    }
    
    /**
     * Constructor.
     * @param fundamentalMatrix fundamental matrix relating two views.
     */
    public DualAbsoluteQuadricInitialCamerasEstimator(
            FundamentalMatrix fundamentalMatrix) {
        super(fundamentalMatrix);
    }

    /**
     * Constructor.
     * @param listener listener to handle events raised by this instance.
     */
    public DualAbsoluteQuadricInitialCamerasEstimator(
            InitialCamerasEstimatorListener listener) {
        super(listener);
    }
    
    /**
     * Constructor.
     * @param fundamentalMatrix fundamental matrix relating two views.
     * @param listener listener to handle events raised by this instance.
     */
    public DualAbsoluteQuadricInitialCamerasEstimator(
            FundamentalMatrix fundamentalMatrix, 
            InitialCamerasEstimatorListener listener) {
        super(fundamentalMatrix, listener);
    }
    
    /**
     * Returns method used by this estimator.
     * @return method used by this estimator.
     */
    @Override
    public InitialCamerasEstimatorMethod getMethod() {
        return InitialCamerasEstimatorMethod.DUAL_ABSOLUTE_QUADRIC;
    }    
    
    /**
     * Indicates if estimator is ready.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return mFundamentalMatrix != null;
    }   
    
    /**
     * Estimates cameras.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if estimator is not ready.
     * @throws InitialCamerasEstimationFailedException if estimation of cameras
     * fails for some reason, typically due to numerical unstabilities.
     */
    @Override
    public void estimate() throws LockedException, NotReadyException, 
            InitialCamerasEstimationFailedException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        try {
            mLocked = true;
            
            if (mListener != null) {
                mListener.onStart(this);
            }
            
            if (mEstimatedLeftCamera == null) {
                mEstimatedLeftCamera = new PinholeCamera();
            }
            if (mEstimatedRightCamera == null) {
                mEstimatedRightCamera = new PinholeCamera();
            }            
            
            generateInitialMetricCamerasUsingDAQ(mFundamentalMatrix, 
                    mAspectRatio, mEstimatedLeftCamera, mEstimatedRightCamera);
            
            if (mListener != null) {
                mListener.onFinish(this, mEstimatedLeftCamera, 
                        mEstimatedRightCamera);
            }
        } catch (InitialCamerasEstimationFailedException e) {
            if (mListener != null) {
                mListener.onFail(this, e);
            }
            throw e;            
        } finally {
            mLocked = false;
        }
    }    
    
    /**
     * Gets aspect ratio of intrinsic parameters of cameras.
     * Typically this value is 1.0 if vertical coordinates increase upwards,
     * or -1.0 if it is the opposite.
     * @return aspect ratio of intrinsic parameters of cameras.
     */
    public double getAspectRatio() {
        return mAspectRatio;
    }
    
    /**
     * Sets aspect ratio of intrinsic parameters of cameras.
     * Typically this value is 1.0 if vertical coordinates increase upwards,
     * or -1.0 if it is the opposite.
     * @param aspectRatio aspect ratio of intrinsic parameters of cameras.
     * @throws LockedException if estimator is locked.
     */
    public void setAspectRatio(double aspectRatio) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mAspectRatio = aspectRatio;
    }
    
    /**
     * Generates initial cameras in metric stratum (with arbitrary scale) using
     * provided fundamental matrix by means of estimation of the Dual Absolute
     * Quadric and the required projective to metric transformation.
     * @param fundamentalMatrix input fundamental matrix to estimate cameras.
     * @param leftCamera instance where left camera will be stored.
     * @param rightCamera instance where right camera will be stored.
     * @throws InitialCamerasEstimationFailedException if estimation fails.
     */
    public static void generateInitialMetricCamerasUsingDAQ(
            FundamentalMatrix fundamentalMatrix, PinholeCamera leftCamera,
            PinholeCamera rightCamera) 
            throws InitialCamerasEstimationFailedException {
        generateInitialMetricCamerasUsingDAQ(fundamentalMatrix, 
                DualAbsoluteQuadricEstimator.
                DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO, leftCamera, rightCamera);
    }
    
    /**
     * Generates initial cameras in metric stratum (with arbitrary scale) using
     * provided fundamental matrix by means of estimation of the Dual Absolute 
     * Quadric and the required projective to metric transformation.
     * @param fundamentalMatrix input fundamental matrix to estimate cameras.
     * @param aspectRatio aspect ratio of intrinsic parameters of cameras. 
     * Typically this value is 1.0 if vertical coordinates increase upwards or
     * -1.0 if it is the opposite.
     * @param leftCamera instance where left camera will be stored.
     * @param rightCamera instance where right camera will be stored.
     * @throws InitialCamerasEstimationFailedException if estimation fails.
     */
    public static void generateInitialMetricCamerasUsingDAQ(
            FundamentalMatrix fundamentalMatrix, double aspectRatio, 
            PinholeCamera leftCamera, PinholeCamera rightCamera) 
            throws InitialCamerasEstimationFailedException {
        
        try {
            //generate arbitrary projective cameras
            fundamentalMatrix.generateCamerasInArbitraryProjectiveSpace(
                    leftCamera, rightCamera);
                        
            List<PinholeCamera> cameras = new ArrayList<PinholeCamera>();
            cameras.add(leftCamera);
            cameras.add(rightCamera);
            
            //estimate dual absolute quadric
            LMSEDualAbsoluteQuadricEstimator daqEstimator =
                    new LMSEDualAbsoluteQuadricEstimator(cameras);
            daqEstimator.setLMSESolutionAllowed(false);
            daqEstimator.setZeroSkewness(true);
            daqEstimator.setPrincipalPointAtOrigin(true);
            daqEstimator.setFocalDistanceAspectRatioKnown(true);
            daqEstimator.setFocalDistanceAspectRatio(
                    DualAbsoluteQuadricEstimator.
                            DEFAULT_FOCAL_DISTANCE_ASPECT_RATIO);
            daqEstimator.setSingularityEnforced(true);
            
            DualAbsoluteQuadric daq = daqEstimator.estimate();
            ProjectiveTransformation3D transformation = 
                    daq.getMetricToProjectiveTransformation();
            //inverse transformation to upgrade cameras from projective to
            //metric stratum
            transformation.inverse();
            
            //transform cameras
            transformation.transform(leftCamera);
            transformation.transform(rightCamera);
        } catch (Exception e) {
            throw new InitialCamerasEstimationFailedException(e);
        }
    }
}
