/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.PROMedSEuclideanTransformation2DRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 25, 2017.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CoordinatesType;
import com.irurueta.geometry.EuclideanTransformation2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.PROMedSRobustEstimator;
import com.irurueta.numerical.robust.PROMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best euclidean 2D transformation for provided collections of 
 * matched 2D point susing PROMedS algorithm.
 */
public class PROMedSEuclideanTransformation2DRobustEstimator extends 
        EuclideanTransformation2DRobustEstimator {
    
    /**
     * Default value to be used for stop threshold. Stop threshold can be used 
     * to keep the algorithm iterating in case that best estimated threshold 
     * using median of residuals is not small enough. Once a solution is found 
     * that generates a threshold below this value, the algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1.0;
    
    /**
     * Minimum allowed stop threshold value.
     */
    public static final double MIN_STOP_THRESHOLD = 0.0;

    /**
     * Threshold to be used to keep the algorithm iterating in case that best 
     * estimated threshold using median of residuals is not small enough. Once 
     * a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     */
    private double mStopThreshold;
        
    /**
     * Quality scores corresponding to each pair of matched points.
     * The larger the score value the betther the quality of the matching.
     */
    private double[] mQualityScores;
    
   /**
     * Constructor.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor with lists of points to be used to estimate an euclidean 2D 
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param inputPoints list of input points to be used to estimate an 
     * euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * euclidean 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            List<Point2D> inputPoints, List<Point2D> outputPoints) 
            throws IllegalArgumentException {
        super(inputPoints, outputPoints);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor with listener and lists of points to be used to estimate an
     * euclidean 2D transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param listener listener to be notified of events such as when estimation
     * stars, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an 
     * euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * euclidean 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener,
            List<Point2D> inputPoints, List<Point2D> outputPoints) 
            throws IllegalArgumentException {
        super(listener, inputPoints, outputPoints);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            double[] qualityScores) throws IllegalArgumentException {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor with lists of points to be used to estimate an euclidean 2D 
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param inputPoints list of input points to be used to estimate an 
     * euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * euclidean 2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @throws IllegalArgumentException if provided lists of points and array
     * of quality scores don't have the same size or their size is smaller than 
     * MINIMUM_SIZE.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            List<Point2D> inputPoints, List<Point2D> outputPoints,
            double[] qualityScores) throws IllegalArgumentException {
        super(inputPoints, outputPoints);

        if (qualityScores.length != inputPoints.size()) {
            throw new IllegalArgumentException();
        }
        
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener,
            double[] qualityScores) throws IllegalArgumentException {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor with listener and lists of points to be used to estimate an
     * euclidean 2D transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param listener listener to be notified of events such as when estimation
     * stars, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 2D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener,
            List<Point2D> inputPoints, List<Point2D> outputPoints,
            double[] qualityScores) throws IllegalArgumentException {
        super(listener, inputPoints, outputPoints);
        
        if (qualityScores.length != inputPoints.size()) {
            throw new IllegalArgumentException();
        }
        
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
   /**
     * Constructor.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            boolean weakMinimumSizeAllowed) {
        super(weakMinimumSizeAllowed);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor with lists of points to be used to estimate an euclidean 2D 
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param inputPoints list of input points to be used to estimate an 
     * euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * euclidean 2D transformation.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            List<Point2D> inputPoints, List<Point2D> outputPoints,
            boolean weakMinimumSizeAllowed) 
            throws IllegalArgumentException {
        super(inputPoints, outputPoints, weakMinimumSizeAllowed);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener,
            boolean weakMinimumSizeAllowed) {
        super(listener, weakMinimumSizeAllowed);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor with listener and lists of points to be used to estimate an
     * euclidean 2D transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param listener listener to be notified of events such as when estimation
     * stars, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an 
     * euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * euclidean 2D transformation.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener,
            List<Point2D> inputPoints, List<Point2D> outputPoints,
            boolean weakMinimumSizeAllowed) throws IllegalArgumentException {
        super(listener, inputPoints, outputPoints, weakMinimumSizeAllowed);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            double[] qualityScores, boolean weakMinimumSizeAllowed) 
            throws IllegalArgumentException {
        super(weakMinimumSizeAllowed);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor with lists of points to be used to estimate an euclidean 2D 
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param inputPoints list of input points to be used to estimate an 
     * euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * euclidean 2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @throws IllegalArgumentException if provided lists of points and array
     * of quality scores don't have the same size or their size is smaller than 
     * MINIMUM_SIZE.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            List<Point2D> inputPoints, List<Point2D> outputPoints,
            double[] qualityScores, boolean weakMinimumSizeAllowed) 
            throws IllegalArgumentException {
        super(inputPoints, outputPoints, weakMinimumSizeAllowed);

        if (qualityScores.length != inputPoints.size()) {
            throw new IllegalArgumentException();
        }
        
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples).
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener,
            double[] qualityScores, boolean weakMinimumSizeAllowed) 
            throws IllegalArgumentException {
        super(listener, weakMinimumSizeAllowed);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor with listener and lists of points to be used to estimate an
     * euclidean 2D transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param listener listener to be notified of events such as when estimation
     * stars, ends or its progress significantly changes.
     * @param inputPoints list of input points to be used to estimate an 
     * affine 2D transformation.
     * @param outputPoints list of output points to be used to estimate an 
     * affine 2D transformation.
     * @param qualityScores quality scores corresponding to each pair of matched 
     * points.
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public PROMedSEuclideanTransformation2DRobustEstimator(
            EuclideanTransformation2DRobustEstimatorListener listener,
            List<Point2D> inputPoints, List<Point2D> outputPoints,
            double[] qualityScores, boolean weakMinimumSizeAllowed) 
            throws IllegalArgumentException {
        super(listener, inputPoints, outputPoints, weakMinimumSizeAllowed);
        
        if (qualityScores.length != inputPoints.size()) {
            throw new IllegalArgumentException();
        }
        
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Returns threshold to be used to keep the algorithm iterating in case that 
     * best estimated threshold using median of residuals is not small enough. 
     * Once a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * As in LMedS, the stop threshold can be used to prevent the PROMedS 
     * algorithm iterating too many times in cases where samples have a very 
     * similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     * @return stop threshold to stop the algorithm prematurely when a certain
     * accuracy has been reached.
     */
    public double getStopThreshold() {
        return mStopThreshold;
    }
    
    /**
     * Sets threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough. 
     * Once a solution is found that generates a threshold below this value, the 
     * algorithm will stop.
     * As in LMedS, the stop threshold can be used to prevent the PROMedS 
     * algorithm iterating too many times in cases where samples have a very 
     * similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close 
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would 
     * iterate for a long time trying to find the best solution when indeed 
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     * @param stopThreshold stop threshold to stop the algorithm prematurely 
     * when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     */
    public void setStopThreshold(double stopThreshold) 
            throws IllegalArgumentException, LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        
        mStopThreshold = stopThreshold;
    }
    
    /**
     * Returns quality scores corresponding to each pair of matched points.
     * The larger the score value the betther the quality of the matching.
     * @return quality scores corresponding to each pair of matched points.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }
    
    /**
     * Sets quality scores corresponding to each pair of matched points.
     * The larger the score value the better the quality of the matching.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 3 samples).
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
     * Indicates if eatimator is ready to start the euclidean 2D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched points and quality
     * scores) are provided and a minimum of MINIMUM_SIZE points are available.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null && 
                mQualityScores.length == mInputPoints.size();
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
     * @throws RobustEstimatorException if estimation fails for any reason
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
        
        PROMedSRobustEstimator<EuclideanTransformation2D> innerEstimator = 
                new PROMedSRobustEstimator<EuclideanTransformation2D>(
                    new PROMedSRobustEstimatorListener<EuclideanTransformation2D>() {
                    
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
                return mStopThreshold;
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
                return PROMedSEuclideanTransformation2DRobustEstimator.this.
                        isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<EuclideanTransformation2D> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            PROMedSEuclideanTransformation2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<EuclideanTransformation2D> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            PROMedSEuclideanTransformation2DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<EuclideanTransformation2D> estimator, 
                    int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            PROMedSEuclideanTransformation2DRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<EuclideanTransformation2D> estimator, 
                    float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            PROMedSEuclideanTransformation2DRobustEstimator.this, 
                            progress);
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
        return RobustEstimatorMethod.PROMedS;
    }    
    
    /**
     * Gets standard deviation used for Levenberg-Marquardt fitting during
     * refinement.
     * Returned value gives an indication of how much variance each residual 
     * has.
     * Typically this value is related to the threshold used on each robust
     * estimation, since residuals of found inliers are within the range of such
     * threshold.
     * @return standard deviation used for refinement.
     */
    @Override
    protected double getRefinementStandardDeviation() {
        PROMedSRobustEstimator.PROMedSInliersData inliersData =
                (PROMedSRobustEstimator.PROMedSInliersData)getInliersData();
        return inliersData.getEstimatedThreshold();
    }
    
    /**
     * Sets quality scores corresponding to each pair of matched points.
     * This method is used internally and does not check whether instance is
     * locked or not.
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE.
     */
    private void internalSetQualityScores(double[] qualityScores) 
            throws IllegalArgumentException {
        if (qualityScores.length < getMinimumPoints()) {
            throw new IllegalArgumentException();
        }
        
        mQualityScores = qualityScores;        
    }            
}