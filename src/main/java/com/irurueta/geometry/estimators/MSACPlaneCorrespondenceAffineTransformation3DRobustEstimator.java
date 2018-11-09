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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.geometry.AffineTransformation3D;
import com.irurueta.geometry.CoincidentPlanesException;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.*;

import java.util.List;

/**
 * Finds the best affine 3D transformation for provided collections of matched
 * planes using MSAC algorithm.
 */
public class MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator 
        extends PlaneCorrespondenceAffineTransformation3DRobustEstimator{
    
    /**
     * Constant defining default threshold to determine whether planes are
     * inliers or not. 
     * Residuals to determine whether planes are inliers or not are computed by
     * comparing two planes algebraically (e.g. doing the dot product of their 
     * parameters).
     * A residual of 0 indicates that dot product was 1 or -1 and planes were 
     * equal.
     * A residual of 1 indicates that dot product was 0 and planes were 
     * orthogonal.
     * If dot product between planes is -1, then although their director vectors 
     * are opposed, planes are considered equal, since sign changes are not 
     * taken into account and their residuals will be 0.
     */
    public static final double DEFAULT_THRESHOLD = 1e-6;
    
    /**
     * Minimum value that can be set as threshold.
     * Threshold must be strictly greater than 0.0.
     */
    public static final double MIN_THRESHOLD = 0.0;
        
    /**
     * Threshold to determine whether planes are inliers or not when testing
     * possible estimation solutions.
     * The threshold refers to the amount of error (i.e. distance and director
     * vector angle difference) a possible solution has on a matched pair of 
     * planes.
     */
    private double mThreshold;   
    
    /**
     * Constructor.
     */
    public MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator() {
        super();
        mThreshold = DEFAULT_THRESHOLD;
    }

    /**
     * Constructor with lists of planes to be used to estimate an affine 2D
     * transformation.
     * Planes in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param inputPlanes list of input planes to be used to estimate an affine
     * 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate an affine
     * 3D transformation.
     * @throws IllegalArgumentException if provided lists of planes don't have 
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
            List<Plane> inputPlanes, List<Plane> outputPlanes) 
            throws IllegalArgumentException {
        super(inputPlanes, outputPlanes);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
            AffineTransformation3DRobustEstimatorListener listener) {
        super(listener);
        mThreshold = DEFAULT_THRESHOLD;
    }
    
    /**
     * Constructor with listener and lists of planes to be used to estimate an
     * affine 3D transformation.
     * Planes in the list located at the same position are considered to be 
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param inputPlanes list of input planes to be used to estimate an affine
     * 3D transformation.
     * @param outputPlanes list of output planes to be used to estimate an 
     * affine 3D transformation.
     * @throws IllegalArgumentException if provided lists of planes don't have
     * the same size or their size is smaller than MINIMUM_SIZE.
     */
    public MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator(
            AffineTransformation3DRobustEstimatorListener listener,
            List<Plane> inputPlanes, List<Plane> outputPlanes) 
            throws IllegalArgumentException {
        super(listener, inputPlanes, outputPlanes);
        mThreshold = DEFAULT_THRESHOLD;     
    }
    
    /**
     * Returns threshold to determine whether planes are inliers or not when 
     * testing possible estimation solutions.
     * Residuals to determine whether planes are inliers or not are computed by
     * comparing two planes algebraically (e.g. doing the dot product of their 
     * parameters).
     * A residual of 0 indicates that dot product was 1 or -1 and planes were 
     * equal.
     * A residual of 1 indicates that dot product was 0 and planes were 
     * orthogonal.
     * If dot product between lines is -1, then although their director vectors 
     * are opposed, planes are considered equal, since sign changes are not 
     * taken into account and their residuals will be 0.
     * @return threshold to determine whether matched planes are inliers or not.
     */
    public double getThreshold() {
        return mThreshold;
    }
    
    /**
     * Sets threshold to determine whether planes are inliers or not when
     * testing possible estimation solutions.
     * Residuals to determine whether planes are inliers or not are computed by
     * comparing two planes algebraically (e.g. doing the dot product of their 
     * parameters).
     * A residual of 0 indicates that dot product was 1 or -1 and planes were 
     * equal.
     * A residual of 1 indicates that dot product was 0 and planes were 
     * orthogonal.
     * If dot product between planes is -1, then although their director vectors 
     * are opposed, planes are considered equal, since sign changes are not 
     * taken into account and their residuals will be 0.
     * @param threshold threshold to determine whether matched planes are 
     * inliers or not.
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
     * Estimates an affine 2D transformation using a robust estimator and
     * the best set of matched 2D planes correspondences found using the robust
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
    public AffineTransformation3D estimate() throws LockedException, 
            NotReadyException, RobustEstimatorException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        MSACRobustEstimator<AffineTransformation3D> innerEstimator =
                new MSACRobustEstimator<>(
                new MSACRobustEstimatorListener<AffineTransformation3D>() {
                    
            //plane to be reused when computing residuals
            private Plane mTestPlane = new Plane();

            @Override
            public double getThreshold() {
                return mThreshold;
            }

            @Override
            public int getTotalSamples() {
                return mInputPlanes.size();
            }

            @Override
            public int getSubsetSize() {
                return AffineTransformation3DRobustEstimator.MINIMUM_SIZE;
            }

            @Override
            public void estimatePreliminarSolutions(int[] samplesIndices, 
                    List<AffineTransformation3D> solutions) {
                Plane inputPlane1 = mInputPlanes.get(samplesIndices[0]);
                Plane inputPlane2 = mInputPlanes.get(samplesIndices[1]);
                Plane inputPlane3 = mInputPlanes.get(samplesIndices[2]);
                Plane inputPlane4 = mInputPlanes.get(samplesIndices[3]);
                
                Plane outputPlane1 = mOutputPlanes.get(samplesIndices[0]);
                Plane outputPlane2 = mOutputPlanes.get(samplesIndices[1]);
                Plane outputPlane3 = mOutputPlanes.get(samplesIndices[2]);
                Plane outputPlane4 = mOutputPlanes.get(samplesIndices[3]);
                
                try {
                    AffineTransformation3D transformation =
                            new AffineTransformation3D(inputPlane1, inputPlane2, 
                            inputPlane3, inputPlane4, outputPlane1, 
                            outputPlane2, outputPlane3, outputPlane4);
                    solutions.add(transformation);
                } catch (CoincidentPlanesException e) {
                    //if lines are coincident, no solution is added
                }
            }

            @Override
            public double computeResidual(
                    AffineTransformation3D currentEstimation, int i) {
                Plane inputPlane = mInputPlanes.get(i);
                Plane outputPlane = mOutputPlanes.get(i);
                
                //transform input line and store result in mTestLine
                try {
                    currentEstimation.transform(inputPlane, mTestPlane);
                    
                    return getResidual(outputPlane, mTestPlane);
                } catch (AlgebraException e) {
                    //this happens when internal matrix of affine transformation
                    //cannot be reverse (i.e. transformation is not well defined,
                    //numerical instabilities, etc)
                    return Double.MAX_VALUE;
                }
            }

            @Override
            public boolean isReady() {
                return MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.
                        this.isReady();
            }

            @Override
            public void onEstimateStart(
                    RobustEstimator<AffineTransformation3D> estimator) {
                if (mListener != null) {
                    mListener.onEstimateStart(
                            MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateEnd(
                    RobustEstimator<AffineTransformation3D> estimator) {
                if (mListener != null) {
                    mListener.onEstimateEnd(
                            MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.this);
                }
            }

            @Override
            public void onEstimateNextIteration(
                    RobustEstimator<AffineTransformation3D> estimator, 
                    int iteration) {
                if (mListener != null) {
                    mListener.onEstimateNextIteration(
                            MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.this, 
                            iteration);
                }
            }

            @Override
            public void onEstimateProgressChange(
                    RobustEstimator<AffineTransformation3D> estimator, 
                    float progress) {
                if (mListener != null) {
                    mListener.onEstimateProgressChange(
                            MSACPlaneCorrespondenceAffineTransformation3DRobustEstimator.this, 
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
            AffineTransformation3D transformation = innerEstimator.estimate();
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
