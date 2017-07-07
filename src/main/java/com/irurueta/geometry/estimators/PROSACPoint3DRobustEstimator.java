/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.PROSACPoint3DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 2, 2015
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.NoIntersectionException;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.PROSACRobustEstimator;
import com.irurueta.numerical.robust.PROSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.List;

/**
 * Finds the best 3D point for provided collection of 3D planes using PROSAC
 * algorithm
 */
public class PROSACPoint3DRobustEstimator extends Point3DRobustEstimator {
    
    /**
     * Constant defining default threshold to determine whether points are 
     * inliers or not.
     * Because typical resolution for points is 1 voxel, then default threshold 
     * is defined as 1.
     */
    public static final double DEFAULT_THRESHOLD = 1;
        
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
     * Threshold to determine whether planes are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a possible 
     * solution has on a sampled plane.
     */
    private double mThreshold;     
    
    /**
     * Quality scores corresponding to each provided point.
     * The larger the score value the betther the quality of the sample.
     */
    private double[] mQualityScores;   
    
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
    public PROSACPoint3DRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }

    /**
     * Constructor with planes.
     * @param planes 3D planes to estimate a 3D point.
     * @throws IllegalArgumentException if provided list of planes doesn't have 
     * a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACPoint3DRobustEstimator(List<Plane> planes) 
            throws IllegalArgumentException {
        super(planes);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public PROSACPoint3DRobustEstimator(
            Point3DRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes 3D planes to estimate a 3D point.
     * @throws IllegalArgumentException if provided list of planes doesn't have 
     * a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACPoint3DRobustEstimator(Point3DRobustEstimatorListener listener,
            List<Plane> planes) throws IllegalArgumentException {
        super(listener, planes);
        mThreshold = DEFAULT_THRESHOLD;
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 2 planes).
     */
    public PROSACPoint3DRobustEstimator(double[] qualityScores) 
            throws IllegalArgumentException {
        super();
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }

    /**
     * Constructor with planes.
     * @param planes 3D planes to estimate a 3D point.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @throws IllegalArgumentException if provided list of planes doesn't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACPoint3DRobustEstimator(List<Plane> planes,
            double[] qualityScores) throws IllegalArgumentException {
        super(planes);
        
        if (qualityScores.length != planes.size()) {
            throw new IllegalArgumentException();
        }
        
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 3 planes).
     */
    public PROSACPoint3DRobustEstimator(Point3DRobustEstimatorListener listener,
            double[] qualityScores) throws IllegalArgumentException {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes 3D planes to estimate a 3D point.
     * @param qualityScores quality scores corresponding to each provided plane.
     * @throws IllegalArgumentException if provided list of planes doesn't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACPoint3DRobustEstimator(Point3DRobustEstimatorListener listener,
            List<Plane> planes, double[] qualityScores) 
            throws IllegalArgumentException {
        super(listener, planes);
        
        if (qualityScores.length != planes.size()) {
            throw new IllegalArgumentException();
        }
        
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
        mComputeAndKeepInliers = DEFAULT_COMPUTE_AND_KEEP_INLIERS;
        mComputeAndKeepResiduals = DEFAULT_COMPUTE_AND_KEEP_RESIDUALS;        
    }
    
    /**
     * Returns threshold to determine whether planes are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a 
     * given plane.
     * @return threshold to determine whether planes are inliers or not when 
     * testing possible estimation solutions.
     */
    public double getThreshold() {
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether planes are inliers or not when 
     * testing possible estimation solutions.
     * Thre threshold refers to the amount of error a possible solution has on 
     * a given plane.
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
     * Returns quality scores corresponding to each provided plane.
     * The larger the score value the betther the quality of the sampled plane.
     * @return quality scores corresponding to each plane.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }
    
    /**
     * Sets quality scores corresponding to each provided plane.
     * The larger the score value the better the quality of the sampled plane.
     * @param qualityScores quality scores corresponding to each plane.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 23 samples).
     */
    @Override
    public void setQualityScores(double[] qualityScores) throws LockedException,
            IllegalArgumentException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Indicates if eatimator is ready to start the 3D point estimation.
     * This is true when input data (i.e. 3D planes and quality scores) are 
     * provided and a minimum of MINIMUM_SIZE points are available.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null && 
                mQualityScores.length == mPlanes.size();
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
     * Estimates a 3D point using a robust estimator and the best set of 3D 
     * planes that intersect into the estimated 3D point.
     * @return a 3D point.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public Point3D estimate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        PROSACRobustEstimator<Point3D> innerEstimator =
                new PROSACRobustEstimator<Point3D>(
                        new PROSACRobustEstimatorListener<Point3D>() {

            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mPlanes.size();
            }

            @Override
            public int getSubsetSize() {
                return Point3DRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<Point3D> solutions) {
                Plane plane1 = mPlanes.get(samplesIndices[0]);
                Plane plane2 = mPlanes.get(samplesIndices[1]);
                Plane plane3 = mPlanes.get(samplesIndices[2]);
                
                try {
                    Point3D point = plane1.getIntersection(plane2, plane3);
                    solutions.add(point);
                } catch (NoIntersectionException e) {
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(Point3D currentEstimation, int i) {
                return residual(currentEstimation, mPlanes.get(i));
            }

            @Override
            public boolean isReady() {
                return PROSACPoint3DRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<Point3D> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            PROSACPoint3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<Point3D> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(PROSACPoint3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<Point3D> estimator, int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            PROSACPoint3DRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<Point3D> estimator, float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            PROSACPoint3DRobustEstimator.this, progress);
                }
            }
            
            @Override
            public double[] getQualityScores() {
                return mQualityScores;
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
            Point3D result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(result);
        } catch (com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (com.irurueta.numerical.NotReadyException e) {
            throw new NotReadyException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns method being used for robust estimation
     * @return method being used for robust estimation
     */    
    @Override
    public RobustEstimatorMethod getMethod() {
        return RobustEstimatorMethod.PROSAC;
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
    
    /**
     * Sets quality scores corresponding to each provided line.
     * This method is used internally and does not check whether instance is
     * locked or not
     * @param qualityScores quality scores to be set
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE
     */
    private void internalSetQualityScores(double[] qualityScores) 
            throws IllegalArgumentException {
        if (qualityScores.length < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        
        mQualityScores = qualityScores;        
    }        
}
