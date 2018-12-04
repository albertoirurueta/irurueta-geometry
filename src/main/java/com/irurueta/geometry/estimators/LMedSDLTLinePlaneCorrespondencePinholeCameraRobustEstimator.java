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

import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds the best pinhole camera for provided collections of matched lines and 
 * planes using LMedS algorithm.
 */
public class LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator extends 
        DLTLinePlaneCorrespondencePinholeCameraRobustEstimator {

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
     * Constructor.
     */
    public LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator() {
        super();
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor with lists of matched planes and 2D lines to estimate a 
     * pinhole camera.
     * Points and lines in the lists located at the same position are considered
     * to be matched. Hence, both lists must have the same size, and their size
     * must be greater or equal than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES
     * (4 matches).
     * @param planes list of planes used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @throws IllegalArgumentException if provided lists don't have the same
     * size or their size is smaller than required minimum size (4 matches).
     */
    public LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            List<Plane> planes, List<Line2D> lines) {
        super(planes, lines);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener) {
        super(listener);
        mStopThreshold = DEFAULT_STOP_THRESHOLD;
    }
    
    /**
     * Constructor with listener and lists of matched planes and 2D lines to
     * estimate a pinhole camera.
     * Points and lines in the lists located at the same position are considered
     * to be matched. Hence, both lists must have the same size, and their size
     * must be greater or equal than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES
     * (4 matches).
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes list of planes used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @throws IllegalArgumentException if provided lists don't have the same
     * size or their size is smaller than required minimum size (4 matches).
     */
    public LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener,
            List<Plane> planes, List<Line2D> lines) {
        super(listener, planes, lines);
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
    public void setStopThreshold(double stopThreshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (stopThreshold <= MIN_STOP_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        
        mStopThreshold = stopThreshold;
    }
    
    /**
     * Estimates a pinhole camera using a robust estimator and
     * the best set of matched 2D line/3D plane correspondences found using the 
     * robust estimator.
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
        
        //pinhole camera estimator using DLT (Direct Linear Transform) algorithm
        final DLTLinePlaneCorrespondencePinholeCameraEstimator nonRobustEstimator =
                new DLTLinePlaneCorrespondencePinholeCameraEstimator();
        
        nonRobustEstimator.setLMSESolutionAllowed(false);        
                
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
        nonRobustEstimator.setSuggestCenterEnabled(
                isSuggestCenterEnabled());
        nonRobustEstimator.setSuggestedCenterValue(
                getSuggestedCenterValue());
        
        LMedSRobustEstimator<PinholeCamera> innerEstimator =
                new LMedSRobustEstimator<>(
                new LMedSRobustEstimatorListener<PinholeCamera>() {
            
            //3D planes for a subset of samples
            private List<Plane> mSubsetPlanes = new ArrayList<>();
            
            //2D lines for a subset of samples
            private List<Line2D> mSubsetLines = new ArrayList<>();
                    
            @Override
            public int getTotalSamples() {
                return mPlanes.size();
            }

            @Override
            public int getSubsetSize() {
                return LinePlaneCorrespondencePinholeCameraEstimator.
                        MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<PinholeCamera> solutions) {
                mSubsetPlanes.clear();
                mSubsetPlanes.add(mPlanes.get(samplesIndices[0]));
                mSubsetPlanes.add(mPlanes.get(samplesIndices[1]));
                mSubsetPlanes.add(mPlanes.get(samplesIndices[2]));
                mSubsetPlanes.add(mPlanes.get(samplesIndices[3]));
                
                mSubsetLines.clear();
                mSubsetLines.add(mLines.get(samplesIndices[0]));
                mSubsetLines.add(mLines.get(samplesIndices[1]));
                mSubsetLines.add(mLines.get(samplesIndices[2]));
                mSubsetLines.add(mLines.get(samplesIndices[3]));
                
                try {
                    nonRobustEstimator.setLists(mSubsetPlanes, mSubsetLines);
                                        
                    PinholeCamera cam = nonRobustEstimator.estimate();
                    solutions.add(cam);
                } catch (Exception e) {
                    //if lines/planes configuration is degenerate, no solution 
                    //is added
                }
            }

            @Override
            public double computeResidual(PinholeCamera currentEstimation, 
                    int i) {
                Line2D inputLine = mLines.get(i);
                Plane inputPlane = mPlanes.get(i);
                
                return singleBackprojectionResidual(currentEstimation, 
                        inputLine, inputPlane);
            }

            @Override
            public boolean isReady() {
                return LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.
                        this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<PinholeCamera> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<PinholeCamera> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<PinholeCamera> estimator, int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.this,
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<PinholeCamera> estimator, float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            LMedSDLTLinePlaneCorrespondencePinholeCameraRobustEstimator.this,
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
            innerEstimator.setStopThreshold(mStopThreshold);
            PinholeCamera result = innerEstimator.estimate();
            mInliersData = innerEstimator.getInliersData();
            return attemptRefine(result, 
                    nonRobustEstimator.getMaxSuggestionWeight());
            
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
        LMedSRobustEstimator.LMedSInliersData inliersData = 
                (LMedSRobustEstimator.LMedSInliersData)getInliersData();

        //avoid setting a threshold too strict
        double threshold = inliersData.getEstimatedThreshold();
        return threshold > mStopThreshold ? threshold : mStopThreshold;
    }    
}
