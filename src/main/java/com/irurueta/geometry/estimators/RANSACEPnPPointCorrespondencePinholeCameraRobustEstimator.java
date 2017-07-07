/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 26, 2017.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.RANSACRobustEstimator;
import com.irurueta.numerical.robust.RANSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best pinhole camera for provided collections of matched 2D/3D 
 * points using RANSAC + EPnP algorithms.
 */
public class RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator extends 
        EPnPPointCorrespondencePinholeCameraRobustEstimator{

    /**
     * Constant defining default threshold to determine whether points are 
     * inliers or not.
     * By defaul 1.0 is considered a good value for cases where measures are
     * done on pixels, since typically the minimum resolution is 1 pixel.
     */
    public static final double DEFAULT_THRESHOLD = 1.0;
        
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Indicates that by default inliers will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_INLIERS = false;
    
    /**
     * Indicates that by default residuals will only be computed but not kept.
     */
    public static final boolean DEFAULT_COMPUTE_AND_KEEP_RESIDUALS = false;    
    
    /**
     * Threshold to determine whether points are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a possible 
     * solution has on a matched pair of points.
     */
    private double mThreshold;         
    
    /**
     * Indicates whether inliers must be computed and kept.
     */
    private boolean mComputeAndKeepInliers;
    
    /**
     * Indicates whether residuals must be computed and kept.
     */
    private boolean mComputeAndKeepResiduals;    
    
    /**
     * Constructor.
     */
    public RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor with lists of points to be used to estimate a pinhole camera.
     * Points in the lists located at the same position are cnsidered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     * estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size or their size is smaller than required minimum size (6 
     * correspondences).
     */
    public RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
            List<Point3D> points3D, List<Point2D> points2D) 
            throws IllegalArgumentException {
        super(points3D, points2D);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
        
    /**
     * Constructor with listener and lists of points to be used to estimate a
     * pinhole camera.
     * Points in the lists located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     * estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size (6
     * correspondences).
     */
    public RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener, 
            List<Point3D> points3D, List<Point2D> points2D)
            throws IllegalArgumentException {
        super(listener, points3D, points2D);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor with intrinsic parameters.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     */
    public RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraIntrinsicParameters intrinsic) {
        super(intrinsic);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor with intrinsic parameters and listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     */
    public RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener,
            PinholeCameraIntrinsicParameters intrinsic) {
        super(listener, intrinsic);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor with lists of points to be used to estimate a pinhole camera
     * and intrinsic parameters.
     * Points in the lists located at the same position are cnsidered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     * estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size or their size is smaller than required minimum size (6 
     * correspondences).
     */
    public RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraIntrinsicParameters intrinsic, List<Point3D> points3D, 
            List<Point2D> points2D) throws IllegalArgumentException {
        super(intrinsic, points3D, points2D);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
        
    /**
     * Constructor with listener and lists of points to be used to estimate a
     * pinhole camera and intrinsic parameters.
     * Points in the lists located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MIN_NUMBER_OF_POINT_CORRESPONDENCES (6 points).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of 3D points used to estimate a pinhole camera.
     * @param points2D list of corresponding projected 2D points used to
     * estimate a pinhole camera.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than required minimum size (6
     * correspondences).
     */
    public RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener, 
            PinholeCameraIntrinsicParameters intrinsic, List<Point3D> points3D, 
            List<Point2D> points2D) throws IllegalArgumentException {
        super(listener, intrinsic, points3D, points2D);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Returns threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error (i.e. euclidean distance) a 
     * possible solution has on projected 2D points.
     * @return threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     */
    public double getThreshold() {
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     * Thre threshold refers to the amount of error (i.e. euclidean distance) a
     * possible solution has on projected 2D points.
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than 
     * zero.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     */
    public void setThreshold(double threshold) throws IllegalArgumentException, 
            LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        mThreshold = threshold;
    }    
    
    /**
     * Indicates whether inliers must be computed and kept.
     * @return true if inliers must be computed and kept, false if inliers
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepInliersEnabled() {
        return mComputeAndKeepInliers;
    }
    
    /**
     * Specifies whether inliers must be computed and kept.
     * @param computeAndKeepInliers true if inliers must be computed and kept,
     * false if inliers only need to be computed but not kept.
     * @throws LockedException if estimator is locked.
     */
    public void setComputeAndKeepInliersEnabled(boolean computeAndKeepInliers) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mComputeAndKeepInliers = computeAndKeepInliers;
    }
    
    /**
     * Indicates whether residuals must be computed and kept.
     * @return true if residuals must be computed and kept, false if residuals
     * only need to be computed but not kept.
     */
    public boolean isComputeAndKeepResidualsEnabled() {
        return mComputeAndKeepResiduals;
    }
    
    /**
     * Specifies whether residuals must be computed and kept.
     * @param computeAndKeepResiduals true if residuals must be computed and
     * kept, false if residuals only need to be computed but not kept.
     * @throws LockedException if estimator is locked.
     */
    public void setComputeAndKeepResidualsEnabled(
            boolean computeAndKeepResiduals) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mComputeAndKeepResiduals = computeAndKeepResiduals;
    }    
    
    /**
     * Estimates a pinhole camera using a robust estimator and
     * the best set of matched 2D/3D point correspondences or 2D line/3D plane 
     * correspondences found using the robust estimator.
     * @return a pinhole camera.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */    
    @Override
    public PinholeCamera estimate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        //pinhole camera estimator using EPnP (Efficient Perspective-n-Point)
        //algorithm
        final EPnPPointCorrespondencePinholeCameraEstimator nonRobustEstimator =
                new EPnPPointCorrespondencePinholeCameraEstimator(mIntrinsic);
                
        nonRobustEstimator.setPlanarConfigurationAllowed(
                mPlanarConfigurationAllowed);
        nonRobustEstimator.setNullspaceDimension2Allowed(
                mNullspaceDimension2Allowed);
        nonRobustEstimator.setNullspaceDimension3Allowed(
                mNullspaceDimension3Allowed);
        nonRobustEstimator.setPlanarThreshold(mPlanarThreshold);
        
        //suggestions
        nonRobustEstimator.setSuggestSkewnessValueEnabled(
                isSuggestSkewnessValueEnabled());
        nonRobustEstimator.setSuggestedSkewnessValue(
                getSuggestedSkewnessValue());
        nonRobustEstimator.setSuggestHorizontalFocalLengthEnabled(
                isSuggestHorizontalFocalLengthEnabled());
        nonRobustEstimator.setSuggestedHorizontalFocalLengthValue(
                getSuggestedHorizontalFocalLengthValue());
        nonRobustEstimator.setSuggestVerticalFocalLengthEnabled(
                isSuggestVerticalFocalLengthEnabled());
        nonRobustEstimator.setSuggestedVerticalFocalLengthValue(
                getSuggestedVerticalFocalLengthValue());
        nonRobustEstimator.setSuggestAspectRatioEnabled(
                isSuggestAspectRatioEnabled());
        nonRobustEstimator.setSuggestedAspectRatioValue(
                getSuggestedAspectRatioValue());
        nonRobustEstimator.setSuggestPrincipalPointEnabled(
                isSuggestPrincipalPointEnabled());
        nonRobustEstimator.setSuggestedPrincipalPointValue(
                getSuggestedPrincipalPointValue());
        nonRobustEstimator.setSuggestRotationEnabled(
                isSuggestRotationEnabled());
        nonRobustEstimator.setSuggestedRotationValue(
                getSuggestedRotationValue());
        nonRobustEstimator.setSuggestCenterEnabled(isSuggestCenterEnabled());
        nonRobustEstimator.setSuggestedCenterValue(
                getSuggestedCenterValue());                    
        
        
        RANSACRobustEstimator<PinholeCamera> innerEstimator =
                new RANSACRobustEstimator<PinholeCamera>(
                new RANSACRobustEstimatorListener<PinholeCamera>() {

            //point to be reused when computing residuals
            private Point2D mTestPoint = Point2D.create(
                    CoordinatesType.HOMOGENEOUS_COORDINATES); 
                        
            //3D points for a subset of samples
            private List<Point3D> mSubset3D = new ArrayList<Point3D>();
            
            //2D points for a subset of samples
            private List<Point2D> mSubset2D = new ArrayList<Point2D>();
                    
            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mPoints3D.size();
            }

            @Override
            public int getSubsetSize() {
                return EPnPPointCorrespondencePinholeCameraEstimator.
                        MIN_NUMBER_OF_POINT_CORRESPONDENCES;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<PinholeCamera> solutions) {
                mSubset3D.clear();
                mSubset3D.add(mPoints3D.get(samplesIndices[0]));
                mSubset3D.add(mPoints3D.get(samplesIndices[1]));
                mSubset3D.add(mPoints3D.get(samplesIndices[2]));
                mSubset3D.add(mPoints3D.get(samplesIndices[3]));
                mSubset3D.add(mPoints3D.get(samplesIndices[4]));
                mSubset3D.add(mPoints3D.get(samplesIndices[5]));

                mSubset2D.clear();
                mSubset2D.add(mPoints2D.get(samplesIndices[0]));
                mSubset2D.add(mPoints2D.get(samplesIndices[1]));
                mSubset2D.add(mPoints2D.get(samplesIndices[2]));
                mSubset2D.add(mPoints2D.get(samplesIndices[3]));
                mSubset2D.add(mPoints2D.get(samplesIndices[4]));
                mSubset2D.add(mPoints2D.get(samplesIndices[5]));
                
                try {
                    nonRobustEstimator.setLists(mSubset3D, mSubset2D);
                                    
                    PinholeCamera cam = nonRobustEstimator.estimate();
                    solutions.add(cam);
                } catch (Exception e) {
                    //if points configuration is degenerate, no solution is
                    //added
                }
            }

            @Override
            public double computeResidual(PinholeCamera currentEstimation, 
                    int i) {
                //pick i-th points
                Point3D point3D = mPoints3D.get(i);
                Point2D point2D = mPoints2D.get(i);
                
                //project point3D into test point
                currentEstimation.project(point3D, mTestPoint);
                
                //compare test point and 2D point
                return mTestPoint.distanceTo(point2D);
            }

            @Override
            public boolean isReady() {
                return RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.
                        this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<PinholeCamera> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<PinholeCamera> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<PinholeCamera> estimator, int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<PinholeCamera> estimator, float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            RANSACEPnPPointCorrespondencePinholeCameraRobustEstimator.this, 
                            progress);
                }
            }
        });
        
        try {
            mLocked = true;
            mInliersData = null;
            innerEstimator.setComputeAndKeepInliersEnabled(
                    mComputeAndKeepInliers || mRefineResult);
            innerEstimator.setComputeAndKeepResidualsEnabled(
                    mComputeAndKeepResiduals || mRefineResult);            
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            PinholeCamera result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(result, 
                    nonRobustEstimator.getMaxSuggestionWeight());
        } catch (com.irurueta.numerical.LockedException e){
            throw new LockedException(e);
        } catch (com.irurueta.numerical.NotReadyException e){
            throw new NotReadyException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns method being used for robust estimation.
     * @return method being used for robust estimation.
     */            
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.RANSAC;
    }

    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during 
     * refinement.
     * Returned value gives an indication of how much variance each residual
     * has.
     * Typically this value is related to the threshold used on each robust 
     * estimation, since residuals of found inliers are within the range of 
     * such threshold.
     * @return standard deviation used for refinement.
     */    
    @Override
    protected double getRefinementStandardDeviation() {
        return mThreshold;
    }
}
