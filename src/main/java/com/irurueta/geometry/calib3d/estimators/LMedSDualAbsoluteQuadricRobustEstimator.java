/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.calib3d.estimators.LMedSDualAbsoluteQuadricRobustEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 9, 2016.
 */
package com.irurueta.geometry.calib3d.estimators;

import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.calib3d.DualAbsoluteQuadric;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;
import com.irurueta.numerical.robust.LMedSRobustEstimator;
import com.irurueta.numerical.robust.LMedSRobustEstimatorListener;
import com.irurueta.numerical.robust.RobustEstimator;
import com.irurueta.numerical.robust.RobustEstimatorException;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best Dual Absolute Quadric (DAQ) for provided collection of cameras
 * using LMedS algorithm.
 */
public class LMedSDualAbsoluteQuadricRobustEstimator extends 
        DualAbsoluteQuadricRobustEstimator {
    
    /**
     * Default value to be used for stop threshold. Stop threshold can be used
     * to keep the algorithm iterating in case that best estimated threshold 
     * using median of residuals is not small enough. Once a solution is found
     * that generates a threshold below this value, the algorithm will stop.
     * Threshold is defined by the equations used to estimate the DAQ depending
     * on the required settings (zero skewness, principal point at origin, and 
     * known aspect ratio).
     */
    public static final double DEFAULT_STOP_THRESHOLD = 1e-6;
    
    /**
     * Minimum value that can be set as stop threshold.
     * Threshold must be strictly greater than 0.0.
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
     * lower than one typically used in RANSAC, and yet the algorithm could 
     * still produce even smaller thresholds in estimated results.
     */
    private double mStopThreshold;
    
    /**
     * Constructor.
     */
    public LMedSDualAbsoluteQuadricRobustEstimator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public LMedSDualAbsoluteQuadricRobustEstimator(
            DualAbsoluteQuadricRobustEstimatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the Dual Absolute Quadric
     * (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @throws IllegalArgumentException if not enough cameras are provided
     * for default settings. Hence, at least 2 cameras must be provided.
     */
    public LMedSDualAbsoluteQuadricRobustEstimator(
            List<PinholeCamera> cameras) throws IllegalArgumentException {
        super(cameras);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param cameras list of cameras used to estimate the Dual AbsoluteQuadric
     * (DAQ), which can be used to obtain pinhole camera intrinsic parameters.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @throws IllegalArgumentException if not enough cameras are provided
     * for default settings. Hence, at least 2 cameras must be provided.
     */
    public LMedSDualAbsoluteQuadricRobustEstimator(List<PinholeCamera> cameras,
            DualAbsoluteQuadricRobustEstimatorListener listener) 
            throws IllegalArgumentException {
        super(cameras, listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Returns threshold to be used to keep the algorithm iterating in case that
     * best estimated threshold using median of residuals is not small enough.
     * Once a solution is found that generates a threshold below this value, the
     * algorithm will stop.
     * The stop threshold can be used to prevent the LMedS algorithm iterating 
     * too many times in cases where samples have a very similar accuracy.
     * For instanc,e in cases where proportion of outliers is very small (close 
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
     * The stop threshold can be used to prevent the LMedS algorithm iterating
     * too many times in cases where samples have a very similar accuracy.
     * For instance, in cases where proportion of outliers is very small (close
     * to 0%), and samples are very accurate (i.e. 1e-6), the algorithm would
     * iterate for a long time trying to find the best solution when indeed
     * there is no need to do tat if a reasonable threshold has already been 
     * reached.
     * Because of this behaviour the stop threshold can be set to a value
     * lower than the one typically used in RANSAC, and yet the algorithm still
     * produce even smaller thresholds in estimated results.
     * @param stopThreshold stop threshold to stop the algorithm prematurely
     * when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
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
     * Estimates the Dual Absolute Quadric using provided cameras.
     * @return estimated Dual Absolute Quadric (DAQ).
     * @throws LockedException if robust estimator is locked.
     * @throws NotReadyException if no valid input data has already been 
     * provided.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */    
    @Override
    public DualAbsoluteQuadric estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        LMedSRobustEstimator<DualAbsoluteQuadric> innerEstimator =
                new LMedSRobustEstimator<DualAbsoluteQuadric>(
                new LMedSRobustEstimatorListener<DualAbsoluteQuadric>() {
                            
            //subset of cameras picked on each iteration
            private List<PinholeCamera> mSubsetCameras =
                    new ArrayList<PinholeCamera>();
                            
            @Override
            public int getTotalSamples() {
                return mCameras.size();
            }

            @Override
            public int getSubsetSize() {
                return mDAQEstimator.getMinNumberOfRequiredCameras();
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<DualAbsoluteQuadric> solutions) {
                mSubsetCameras.clear();
                for (int i = 0; i < samplesIndices.length; i++) {
                    mSubsetCameras.add(mCameras.get(samplesIndices[i]));
                }
                
                try {
                    mDAQEstimator.setLMSESolutionAllowed(false);
                    mDAQEstimator.setCameras(mSubsetCameras);
                    
                    DualAbsoluteQuadric daq = mDAQEstimator.estimate();
                    solutions.add(daq);
                } catch (Exception e) {
                    //if anything fails, no solution is added
                }
            }

            @Override
            public double computeResidual(DualAbsoluteQuadric currentEstimation, 
                    int i) {
                return residual(currentEstimation, mCameras.get(i));
            }

            @Override
            public boolean isReady() {
                return LMedSDualAbsoluteQuadricRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<DualAbsoluteQuadric> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            LMedSDualAbsoluteQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<DualAbsoluteQuadric> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            LMedSDualAbsoluteQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<DualAbsoluteQuadric> estimator, 
                    int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            LMedSDualAbsoluteQuadricRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<DualAbsoluteQuadric> estimator, 
                    float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            LMedSDualAbsoluteQuadricRobustEstimator.this, 
                            progress);
                }
            }
        });
        
        try {
            mLocked = true;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
            innerEstimator.setStopThreshold(mStopThreshold);
            return innerEstimator.estimate();
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
        return RobustEstimatorMethod.LMedS;
    }    
}
