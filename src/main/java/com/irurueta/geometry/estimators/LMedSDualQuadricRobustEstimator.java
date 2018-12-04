/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CoincidentPlanesException;
import com.irurueta.geometry.DualQuadric;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.*;

import java.util.List;

/**
 * Finds the best dual quadric for provided collection of 3D planes using LMedS
 * algorithm.
 */
public class LMedSDualQuadricRobustEstimator extends DualQuadricRobustEstimator{
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
    public static final double DEFAULT_STOP_THRESHOLD = 1e-9;
    
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
     * Constructor.
     */
    public LMedSDualQuadricRobustEstimator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }

    /**
     * Constructor with points.
     * @param planes 3D planes to estimate a dual quadric.
     * @throws IllegalArgumentException if provided list of planes don't have 
     * a size greater or equal than MINIMUM_SIZE.
     */
    public LMedSDualQuadricRobustEstimator(List<Plane> planes) {
        super(planes);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public LMedSDualQuadricRobustEstimator(
            DualQuadricRobustEstimatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes 3D planes to estimate a dual quadric.
     * @throws IllegalArgumentException if provided list of planes don't have 
     * a size greater or equal than MINIMUM_SIZE.
     */
    public LMedSDualQuadricRobustEstimator(
            DualQuadricRobustEstimatorListener listener,
            List<Plane> planes) {
        super(listener, planes);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Returns threshold to be used to keep the algorithm iterating in case that 
     * best estimated threshold using median of residuals is not small enough. 
     * Once a solution is found that generates a threshold below this value, the 
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
     * there is no need to do that if a reasonable threshold has already been
     * reached.
     * Because of this behaviour the stop threshold can be set to a value much
     * lower than the one typically used in RANSAC, and yet the algorithm could
     * still produce even smaller thresholds in estimated results.
     * @param stopThreshold stop threshold to stop the algorithm prematurely 
     * when a certain accuracy has been reached.
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     */
    public void setStopThreshold(double stopThreshold) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        
        mStopThreshold = stopThreshold;
    }  
    
            
    /**
     * Estimates a dual quadric using a robust estimator and the best set of 3D 
     * planes that fit into the locus of the estimated dual quadric found using 
     * the robust estimator.
     * @return a dual quadric.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */    
    @Override
    public DualQuadric estimate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        LMedSRobustEstimator<DualQuadric> innerEstimator =
                new LMedSRobustEstimator<>(
                        new LMedSRobustEstimatorListener<DualQuadric>() {

            @Override
            public int getTotalSamples() {
                return mPlanes.size();
            }

            @Override
            public int getSubsetSize() {
                return DualQuadricRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<DualQuadric> solutions) {
                Plane plane1 = mPlanes.get(samplesIndices[0]);
                Plane plane2 = mPlanes.get(samplesIndices[1]);
                Plane plane3 = mPlanes.get(samplesIndices[2]);
                Plane plane4 = mPlanes.get(samplesIndices[3]);
                Plane plane5 = mPlanes.get(samplesIndices[4]);
                Plane plane6 = mPlanes.get(samplesIndices[5]);
                Plane plane7 = mPlanes.get(samplesIndices[6]);
                Plane plane8 = mPlanes.get(samplesIndices[7]);
                Plane plane9 = mPlanes.get(samplesIndices[8]);
                
                try {
                    DualQuadric dualQuadric = new DualQuadric(plane1, plane2, 
                            plane3, plane4, plane5, plane6, plane7, plane8, 
                            plane9);
                    solutions.add(dualQuadric);
                } catch (CoincidentPlanesException e) {
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(DualQuadric currentEstimation, 
                    int i) {
                return residual(currentEstimation, mPlanes.get(i));
            }

            @Override
            public boolean isReady() {
                return LMedSDualQuadricRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<DualQuadric> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            LMedSDualQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<DualQuadric> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            LMedSDualQuadricRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<DualQuadric> estimator, int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            LMedSDualQuadricRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<DualQuadric> estimator, float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            LMedSDualQuadricRobustEstimator.this, progress);
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
