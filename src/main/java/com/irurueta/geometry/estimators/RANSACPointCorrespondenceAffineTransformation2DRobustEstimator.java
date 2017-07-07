/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.RANSACPointCorrespondenceAffineTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 9, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.AffineTransformation2D;
import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.RANSACRobustEstimator;
import com.irurueta.numerical.robust.RANSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best affine 2D transformation for provided collections of matched
 * 2D points using RANSAC algorithm.
 */
public class RANSACPointCorrespondenceAffineTransformation2DRobustEstimator 
        extends PointCorrespondenceAffineTransformation2DRobustEstimator {
    
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
    public RANSACPointCorrespondenceAffineTransformation2DRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;
    }
    
    /**
     * Constructor with lists of points to be used to estimate an affine 2D 
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 2D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public RANSACPointCorrespondenceAffineTransformation2DRobustEstimator(
            List<Point2D> inputPoints, List<Point2D> outputPoints) 
            throws IllegalArgumentException {
        super(inputPoints, outputPoints);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public RANSACPointCorrespondenceAffineTransformation2DRobustEstimator(
            AffineTransformation2DRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor with listener and lists of points to be used to estimate an
     * affine 2D transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 2D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public RANSACPointCorrespondenceAffineTransformation2DRobustEstimator(
            AffineTransformation2DRobustEstimatorListener listener,
            List<Point2D> inputPoints, List<Point2D> outputPoints) 
            throws IllegalArgumentException {
        super(listener, inputPoints, outputPoints);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Returns threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error (i.e. euclidean distance) a 
     * possible solution has on a matched pair of points.
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
     * possible solution has on a matched pair of points.
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
     * @return true if inliers must be computed and kept, false if inliers only
     * need to be computed but not kept.
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
     * Estimates an affine 2D transformation using a robust estimator and
     * the best set of matched 2D point correspondences found using the robust
     * estimator.
     * @return an affine 2D transformation.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */    
    @Override
    public AffineTransformation2D estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        RANSACRobustEstimator<AffineTransformation2D> innerEstimator = 
                new RANSACRobustEstimator<AffineTransformation2D>(
                    new RANSACRobustEstimatorListener<AffineTransformation2D>() {
                    
            //point to be reused when computing residuals
            private Point2D mTestPoint = Point2D.create(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);
            
            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mInputPoints.size();
            }

            @Override
            public int getSubsetSize() {
                return AffineTransformation2DRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<AffineTransformation2D> solutions) {
                Point2D inputPoint1 = mInputPoints.get(samplesIndices[0]);                
                Point2D inputPoint2 = mInputPoints.get(samplesIndices[1]);
                Point2D inputPoint3 = mInputPoints.get(samplesIndices[2]);
                
                Point2D outputPoint1 = mOutputPoints.get(samplesIndices[0]);                
                Point2D outputPoint2 = mOutputPoints.get(samplesIndices[1]);
                Point2D outputPoint3 = mOutputPoints.get(samplesIndices[2]);

                try {
                    AffineTransformation2D transformation = 
                        new AffineTransformation2D(inputPoint1, inputPoint2, 
                        inputPoint3, outputPoint1, outputPoint2, outputPoint3);
                    solutions.add(transformation);
                } catch (CoincidentPointsException e) {
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(
                    AffineTransformation2D currentEstimation, int i) {
                Point2D inputPoint = mInputPoints.get(i);
                Point2D outputPoint = mOutputPoints.get(i);
                
                //transform input point and store result in mTestPoint
                currentEstimation.transform(inputPoint, mTestPoint);
                
                return outputPoint.distanceTo(mTestPoint);
            }

            @Override
            public boolean isReady() {
                return RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.
                        this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<AffineTransformation2D> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<AffineTransformation2D> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<AffineTransformation2D> estimator, 
                    int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<AffineTransformation2D> estimator, 
                    float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            RANSACPointCorrespondenceAffineTransformation2DRobustEstimator.this, 
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
            AffineTransformation2D transformation = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(transformation);
        } catch (com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (com.irurueta.numerical.NotReadyException e) {
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
