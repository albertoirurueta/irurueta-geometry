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

import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.Conic;
import com.irurueta.geometry.Point2D;
import com.irurueta.numerical.robust.*;

import java.util.List;

/**
 * Finds the best conic for provided collection of 2D points using PROSAC
 * algorithm.
 */
public class PROSACConicRobustEstimator extends ConicRobustEstimator{
    /**
     * Constant defining default threshold to determine whether points are 
     * inliers or not.
     * Threshold is defined by the equation abs(trans(x) * C * x) &lt; t, where trans
     * is the transposition, x i a point and C is a conic and t is a threshold.
     * This equation determines the points x belonging to the locus of a conic 
     * C up to a certain threshold.
     */
    public static final double DEFAULT_THRESHOLD = 1e-6;
        
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
     * Quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sample.
     */
    private double[] mQualityScores;
    
    /**
     * Constructor.
     */
    public PROSACConicRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with points.
     * @param points 2D points to estimate a conic.
     * @throws IllegalArgumentException if provided list of points don't have 
     * a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACConicRobustEstimator(List<Point2D> points) {
        super(points);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public PROSACConicRobustEstimator(ConicRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points 2D points to estimate a conic.
     * @throws IllegalArgumentException if provided list of points don't have 
     * a size greater or equal than MINIMUM_SIZE.
     */
    public PROSACConicRobustEstimator(ConicRobustEstimatorListener listener,
            List<Point2D> points) {
        super(listener, points);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 5 points).
     */
    public PROSACConicRobustEstimator(double[] qualityScores) {
        super();
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }

    /**
     * Constructor with points.
     * @param points 2D points to estimate a conic.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACConicRobustEstimator(List<Point2D> points,
            double[] qualityScores) {
        super(points);
        
        if (qualityScores.length != points.size()) {
            throw new IllegalArgumentException();
        }
        
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE (i.e. 5 points).
     */
    public PROSACConicRobustEstimator(ConicRobustEstimatorListener listener,
            double[] qualityScores) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param points 2D points to estimate a conic.
     * @param qualityScores quality scores corresponding to each provided point.
     * @throws IllegalArgumentException if provided list of points don't have 
     * the same size as the list of provided quality scores, or it their size 
     * is not greater or equal than MINIMUM_SIZE.
     */
    public PROSACConicRobustEstimator(ConicRobustEstimatorListener listener,
            List<Point2D> points, double[] qualityScores) {
        super(listener, points);
        
        if (qualityScores.length != points.size()) {
            throw new IllegalArgumentException();
        }
        
        mThreshold = DEFAULT_THRESHOLD;
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Returns threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     * The threshold refers to the amount of error a possible solution has on a 
     * given point.
     * @return threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     */
    public double getThreshold() {
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether points are inliers or not when 
     * testing possible estimation solutions.
     * Thre threshold refers to the amount of error a possible solution has on 
     * a given point.
     * @param threshold threshold to be set.
     * @throws IllegalArgumentException if provided value is equal or less than 
     * zero.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     */
    public void setThreshold(double threshold) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (threshold <= MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
        mThreshold = threshold;
    }
    
    /**
     * Returns quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sampled point
     * @return quality scores corresponding to each point.
     */
    @Override
    public double[] getQualityScores() {
        return mQualityScores;
    }
    
    /**
     * Sets quality scores corresponding to each provided point.
     * The larger the score value the better the quality of the sampled point.
     * @param qualityScores quality scores corresponding to each point.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is 
     * smaller than MINIMUM_SIZE (i.e. 5 samples).
     */
    @Override
    public void setQualityScores(double[] qualityScores) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetQualityScores(qualityScores);
    }
    
    /**
     * Indicates if estimator is ready to start the conic estimation.
     * This is true when input data (i.e. 2D points and quality scores) are 
     * provided and a minimum of MINIMUM_SIZE points are available.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mQualityScores != null && 
                mQualityScores.length == mPoints.size();
    }    
    
    /**
     * Estimates a conic using a robust estimator and the best set of 2D points 
     * that fit into the locus of the estimated conic found using the robust 
     * estimator.
     * @return a conic.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws NotReadyException if provided input data is not enough to start
     * the estimation.
     * @throws RobustEstimatorException if estimation fails for any reason
     * (i.e. numerical instability, no solution available, etc).
     */    
    @Override
    public Conic estimate() throws LockedException, NotReadyException, 
            RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        PROSACRobustEstimator<Conic> innerEstimator =
                new PROSACRobustEstimator<>(new PROSACRobustEstimatorListener<Conic>() {

            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mPoints.size();
            }

            @Override
            public int getSubsetSize() {
                return ConicRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<Conic> solutions) {
                Point2D point1 = mPoints.get(samplesIndices[0]);
                Point2D point2 = mPoints.get(samplesIndices[1]);
                Point2D point3 = mPoints.get(samplesIndices[2]);
                Point2D point4 = mPoints.get(samplesIndices[3]);
                Point2D point5 = mPoints.get(samplesIndices[4]);
                
                try {
                    Conic conic = new Conic(point1, point2, point3, point4, 
                            point5);
                    solutions.add(conic);
                } catch (CoincidentPointsException e) {
                    //if points are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(Conic currentEstimation, int i) {
                return residual(currentEstimation, mPoints.get(i));
            }

            @Override
            public boolean isReady() {
                return PROSACConicRobustEstimator.this.isReady();
            }

            @Override
            public void onEstimateStart(RobustEstimator<Conic> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(PROSACConicRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(RobustEstimator<Conic> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(PROSACConicRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<Conic> estimator, int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            PROSACConicRobustEstimator.this, iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<Conic> estimator, float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            PROSACConicRobustEstimator.this, progress);
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
     * Sets quality scores corresponding to each provided point.
     * This method is used internally and does not check whether instance is
     * locked or not.
     * @param qualityScores quality scores to be set.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MINIMUM_SIZE.
     */
    private void internalSetQualityScores(double[] qualityScores) {
        if (qualityScores.length < MINIMUM_SIZE) {
            throw new IllegalArgumentException();
        }
        
        mQualityScores = qualityScores;        
    }     
}
