/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.MSACEuclideanTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 25, 2017.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.EuclideanTransformation2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.MSACRobustEstimator;
import com.irurueta.numerical.robust.MSACRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best euclidean 2D transformation for provided collections of 
 * matched 2D points using MSAC algorithm.
 */
public class MSACEuclideanTransformation2DRobustEstimator extends 
        EuclideanTransformation2DRobustEstimator {
    
    /**
     * Constant defining default threshold to determine whether points are 
     * inliers or not.
     * By default 1.0 is considered a good value for cases where measures are
     * done on pixels, since typically the minimum resolution is 1 pixel.
     */
    public static final double DEFAULT_THRESHOLD = 1.0;
    
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Threshold to determine whether points are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance) a possible
     * solution has on a matched pair of points.
     */
    private double mThreshold;
    
    /**
     * Constructor.
     */
    public MSACEuclideanTransformation2DRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor with lists of points to be used to estimate an euclidean 2D
     * transformation.
     * Points in the list located at the same position are considered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param inputPoints list of input points ot be used to estimate an 
     * euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     * euclidean 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public MSACEuclideanTransformation2DRobustEstimator(
            List<Point2D> inputPoints, List<Point2D> outputPoints)
            throws IllegalArgumentException {
        super(inputPoints, outputPoints);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public MSACEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor with listener and lists of points to be used to estimate an
     * euclidean 2D transformation.
     * Points in thelist located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size msut
     * be greater or equal than MINIMUM_SIZE.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an
     * euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     * euclidean 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public MSACEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener,
            List<Point2D> inputPoints, List<Point2D> outputPoints)
            throws IllegalArgumentException {
        super(listener, inputPoints, outputPoints);
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     */
    public MSACEuclideanTransformation2DRobustEstimator(
            boolean weakMinimumSizeAllowed) {
        super(weakMinimumSizeAllowed);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor with lists of points to be used to estimate an euclidean 2D
     * transformation.
     * Points in the list located at the same position are considered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param inputPoints list of input points ot be used to estimate an 
     * euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     * euclidean 2D transformation.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public MSACEuclideanTransformation2DRobustEstimator(
            List<Point2D> inputPoints, List<Point2D> outputPoints,
            boolean weakMinimumSizeAllowed) throws IllegalArgumentException {
        super(inputPoints, outputPoints, weakMinimumSizeAllowed);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     */
    public MSACEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener,
            boolean weakMinimumSizeAllowed) {
        super(listener, weakMinimumSizeAllowed);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor with listener and lists of points to be used to estimate an
     * euclidean 2D transformation.
     * Points in thelist located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size msut
     * be greater or equal than MINIMUM_SIZE.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an
     * euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     * euclidean 2D transformation.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public MSACEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener,
            List<Point2D> inputPoints, List<Point2D> outputPoints,
            boolean weakMinimumSizeAllowed) throws IllegalArgumentException {
        super(listener, inputPoints, outputPoints, weakMinimumSizeAllowed);
        mThreshold = DEFAULT_THRESHOLD;
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
     * The threshold refers to the amount of error (i.e. euclidean distance) a
     * possible solution has on a matched pair of points.
     * @param threshold threshold to determine whether points are inliers or 
     * not.
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
     * Estimates an euclidean 2D transformation using a robust estimator and
     * the best set of matched 2D point correspondences found using the robust
     * estimator.
     * @return an euclidean 2D transformation.
     * @throws LockedException if robust estimator is locked because an
     * estimation is already in progress.
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation.
     * @throws RobustEstimatorException if estimatoin fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */
    @Override
    public EuclideanTransformation2D estimate() throws LockedException,
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        MSACRobustEstimator<EuclideanTransformation2D> innerEstimator = 
                new MSACRobustEstimator<EuclideanTransformation2D>(
                    new MSACRobustEstimatorListener<EuclideanTransformation2D>() {
                    
            //point to be reused when computing residuals
            private Point2D mTestPoint = Point2D.create(
                    CoordinatesType.HOMOGENEOUS_COORDINATES);
            
            private EuclideanTransformation2DEstimator mNonRobustEstimator = 
                    new EuclideanTransformation2DEstimator(
                            isWeakMinimumSizeAllowed());
            
            private List<Point2D> mSubsetInputPoints = 
                    new ArrayList<Point2D>();
            private List<Point2D> mSubsetOutputPoints = 
                    new ArrayList<Point2D>();
            
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
                return mNonRobustEstimator.getMinimumPoints();
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<EuclideanTransformation2D> solutions) {
                mSubsetInputPoints.clear();
                mSubsetOutputPoints.clear();
                for(int i = 0; i < samplesIndices.length; i++) {
                    mSubsetInputPoints.add(mInputPoints.get(samplesIndices[i]));
                    mSubsetOutputPoints.add(mOutputPoints.get(
                            samplesIndices[i]));
                }

                try{
                    mNonRobustEstimator.setPoints(mSubsetInputPoints, 
                            mSubsetOutputPoints);
                    solutions.add(mNonRobustEstimator.estimate());
                }catch(Exception e){
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(
                    EuclideanTransformation2D currentEstimation, int i) {
                Point2D inputPoint = mInputPoints.get(i);
                Point2D outputPoint = mOutputPoints.get(i);
                
                //transform input point and store result in mTestPoint
                currentEstimation.transform(inputPoint, mTestPoint);
                
                return outputPoint.distanceTo(mTestPoint);
            }

            @Override
            public boolean isReady() {
                return MSACEuclideanTransformation2DRobustEstimator.this.
                        isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<EuclideanTransformation2D> estimator) {
                if(mListener != null){
                    mListener.onEstimateStart(
                            MSACEuclideanTransformation2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<EuclideanTransformation2D> estimator) {
                if(mListener != null){
                    mListener.onEstimateEnd(
                            MSACEuclideanTransformation2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<EuclideanTransformation2D> estimator, 
                    int iteration) {
                if(mListener != null){
                    mListener.onEstimateNextIteration(
                            MSACEuclideanTransformation2DRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<EuclideanTransformation2D> estimator, 
                    float progress) {
                if(mListener != null){
                    mListener.onEstimateProgressChange(
                            MSACEuclideanTransformation2DRobustEstimator.this, 
                            progress);
                }
            }
        });
        
        try {
            mLocked = true;
            mInliersData = null;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            EuclideanTransformation2D transformation = 
                    innerEstimator.estimate();
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
        return RobustEstimatorMethod.MSAC;
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
