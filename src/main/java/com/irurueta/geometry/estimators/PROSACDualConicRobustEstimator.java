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

import com.irurueta.geometry.CoincidentLinesException;
import com.irurueta.geometry.DualConic;
import com.irurueta.geometry.Line2D;
import com.irurueta.numerical.robust.*;

import java.util.List;

/**
 * Finds the best dual conic for provided collection of 2D lines using PROSAC
 * algorithm.
 */
public class PROSACDualConicRobustEstimator extends DualConicRobustEstimator {
    
    /**
     * Constant defining default threshold to determine whether lines are 
     * inliers or not.
     * Threshold is defined by the equation abs(trans(l) * dc * l) &lt; t, where
     * trans is the transposition, l is a line, dc is a dual conic and t is a
     * threshold.
     * This equation determines the lines l belonging to the locus of a dual
     * conic dC up to a certain threshold.
     */
    public static final double DEFAULT_THRESHOLD = 1e-7;
    
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Threshold to determine whether lines are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of algebraic error a possible 
     * solution has on a given line.
     */
    private double mThreshold;   
    
    /**
     * Quality scores corresponding to each line.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;    
    
    /**
     * Constructor.
     */
    public PROSACDualConicRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points.
     * @param lines 2D lines to estimate a dual conic.
     * @throws IllegalArgumentException if provided list of lines don't have 
     * a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACDualConicRobustEstimator(List<Line2D> lines) 
            throws IllegalArgumentException {
        super(lines);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public PROSACDualConicRobustEstimator(
            DualConicRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param lines 2D lines to estimate a dual conic.
     * @throws IllegalArgumentException if provided list of lines don't have 
     * a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACDualConicRobustEstimator(
            DualConicRobustEstimatorListener listener,
            List<Line2D> lines) throws IllegalArgumentException {
        super(listener, lines);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor with quality scores.
     * @param qualityScores quality scores corresponding to each provided line.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 5 lines).
     */
    public PROSACDualConicRobustEstimator(double[] qualityScores)
            throws IllegalArgumentException {
        super();
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with lines and quality scores.
     * @param lines 2D lines to estimate a dual conic.
     * @param qualityScores quality scores corresponding to each provided line.
     * @throws IllegalArgumentException if provided list of lines don't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACDualConicRobustEstimator(List<Line2D> lines,
            double[] qualityScores) throws IllegalArgumentException {
        super(lines);
        
        if (qualityScores.length != lines.size()) {
            throw new IllegalArgumentException();
        }
        
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided line.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 5 lines).
     */
    public PROSACDualConicRobustEstimator(
            DualConicRobustEstimatorListener listener, double[] qualityScores)
            throws IllegalArgumentException {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param lines 2D lines to estimate a dual conic.
     * @param qualityScores quality scores corresponding to each provided line.
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACDualConicRobustEstimator(
            DualConicRobustEstimatorListener listener,
            List<Line2D> lines, double[] qualityScores) 
            throws IllegalArgumentException {
        super(listener, lines);
        
        if (qualityScores.length != lines.size()) {
            throw new IllegalArgumentException();
        }
        
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }    
    
    /**
     * Returns threshold to determine whether lines are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a 
     * given line.
     * @return threshold to determine whether lines are inliers or not when 
     * testing possible estimation solutions.
     */
    public double getThreshold() {
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether lines are inliers or not when 
     * testing possible estimation solutions.
     * Thre threshold refers to the amount of algebrauc error a possible 
     * solution has on a given line.
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
     * Returns quality scores corresponding to each provided line.
     * The larger the score value the better the quality of the sampled line.
     * @return quality scores corresponding to each point.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }
    
    /**
     * Sets quality scores corresponding to each provided line.
     * The larger the score value the better the quality of the sampled line.
     * @param qualityScores quality scores corresponding to each line.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 5 samples).
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
     * Indicates if estimator is ready to start the conic estimation.
     * This is true when input data (i.e. 2D lines and quality scores) are 
     * provided and a minimum of MINIMUM_SIZE lines are available.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null && 
                mQualityScores.length == mLines.size();
    }        
            
    /**
     * Estimates a dual conic using a robust estimator and the best set of 2D 
     * lines that fit into the locus of the estimated dual conic found using the
     * robust estimator.
     * @return a dual conic.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */    
    @Override
    public DualConic estimate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        PROSACRobustEstimator<DualConic> innerEstimator =
                new PROSACRobustEstimator<>(
                        new PROSACRobustEstimatorListener<DualConic>() {

            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mLines.size();
            }

            @Override
            public int getSubsetSize() {
                return DualConicRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<DualConic> solutions) {
                Line2D line1 = mLines.get(samplesIndices[0]);
                Line2D line2 = mLines.get(samplesIndices[1]);
                Line2D line3 = mLines.get(samplesIndices[2]);
                Line2D line4 = mLines.get(samplesIndices[3]);
                Line2D line5 = mLines.get(samplesIndices[4]);
                
                try {
                    DualConic dualConic = new DualConic(line1, line2, line3, 
                            line4, line5);
                    solutions.add(dualConic);
                } catch (CoincidentLinesException e) {
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(DualConic currentEstimation, int i) {
                return residual(currentEstimation, mLines.get(i));
            }

            @Override
            public boolean isReady() {
                return PROSACDualConicRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<DualConic> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            PROSACDualConicRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<DualConic> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            PROSACDualConicRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<DualConic> estimator, int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            PROSACDualConicRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<DualConic> estimator, float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            PROSACDualConicRobustEstimator.this, progress);
                }
            }
            
            @Override
            public double[] getQualityScores() {
                return mQualityScores;
            }              
        });
        
        try {
            mLocked = true;
            innerEstimator.setConfidence(mConfidence);
            innerEstimator.setMaxIterations(mMaxIterations);
            innerEstimator.setProgressDelta(mProgressDelta);
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
        return RobustEstimatorMethod.PROSAC;
    }    
    
    /**
     * Sets quality scores corresponding to each provided line.
     * This method is used internally and does not check whether instance is
     * locked or not.
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE.
     */
    private void internalSetQualityScores(double[] qualityScores) 
            throws IllegalArgumentException {
        if (qualityScores.length < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        
        mQualityScores = qualityScores;        
    }        
}
