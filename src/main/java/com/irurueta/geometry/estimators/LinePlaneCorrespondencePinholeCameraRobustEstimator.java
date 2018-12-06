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
import com.irurueta.geometry.refiners.DecomposedLinePlaneCorrespondencePinholeCameraRefiner;
import com.irurueta.geometry.refiners.NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner;
import com.irurueta.numerical.robust.InliersData;
import com.irurueta.numerical.robust.RobustEstimatorMethod;

import java.util.List;

/**
 * This is an abstract class for algorithms to robustly find the best pinhole
 * camera for collections of matched lines and planes.
 * Implementations of this class should be able to detect and discard outliers
 * in order to find the best solution.
 */
public abstract class LinePlaneCorrespondencePinholeCameraRobustEstimator 
        extends PinholeCameraRobustEstimator {
    
    /**
     * Minimum number of required line/plane correspondences to estimate a
     * camera.
     */
    public static final int MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES = 4;

    /**
     * Default robust estimator method when none is provided.
     */
    public static final RobustEstimatorMethod DEFAULT_ROBUST_METHOD =
            RobustEstimatorMethod.PROMedS;
    
    /**
     * List of matched planes.
     */
    protected List<Plane> mPlanes;
    
    /**
     * List of matched lines.
     */
    protected List<Line2D> mLines;
    
    /**
     * Plane to be reused when computing residuals.
     */
    private Plane mResidualTestPlane = new Plane();
    
    /**
     * Constructor.
     */
    public LinePlaneCorrespondencePinholeCameraRobustEstimator() {
        super();        
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
    public LinePlaneCorrespondencePinholeCameraRobustEstimator(
            List<Plane> planes, List<Line2D> lines) {
        super();
        internalSetLinesAndPlanes(planes, lines);
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     */
    public LinePlaneCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener) {
        super(listener);
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
    public LinePlaneCorrespondencePinholeCameraRobustEstimator(
            PinholeCameraRobustEstimatorListener listener,
            List<Plane> planes, List<Line2D> lines) {
        super(listener);
        internalSetLinesAndPlanes(planes, lines);
    }
    
    /**
     * Returns list of 3D planes to be used to estimate a pinhole camera.
     * Each plane in the list of 3D planes must be matched with a corresponding
     * 2D line resulting from the projection of the 3D plane through the camera
     * to be estimated. Matched lines and planes must be located on the same
     * position in the lists, hence, both lists must have the same size, and
     * their size must be greater or equal than 
     * MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES.
     * @return list of 3D planes to be used to estimate a pinhole camera.
     */
    public List<Plane> getPlanes() {
        return mPlanes;
    }
    
    /**
     * Returns list of 2D lines to be used to estimate a pinhole camera.
     * Each line in the list is the result of the projection of a matched.
     * @return list of 2D lines to be used to estimate a pinhole camera.
     */
    public List<Line2D> getLines() {
        return mLines;
    }
       
    /**
     * Sets list of matched 3D planes and 2D lines to be used to estimate a 
     * pinhole camera.
     * Each plane in the list of 3D planes must be matched with a corresponding
     * 2D line resulting from the projection of the 3D plane through the camera
     * to be estimated. Matched lines and planes must be located on the same
     * position in the lists, hence, both lists must have the same size, and
     * their size must be greater or equal than 
     * MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES.
     * @param planes 3D planes to be used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @throws IllegalArgumentException if provided lists don't have the same
     * size or their size is smaller than required minimum size (4 matches).
     * @throws LockedException if estimator is locked because a computation is
     * already in progress.
     */
    public final void setLinesAndPlanes(List<Plane> planes, List<Line2D> lines)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetLinesAndPlanes(planes, lines);
    }
    
    /**
     * Indicates if estimator is ready to start the pinhole camera estimation.
     * This is true when input data (i.e. lists of matched planes and lines) are
     * provided an a minimum of MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES are 
     * available.
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mPlanes != null && mLines != null &&
                mPlanes.size() == mLines.size() &&
                mPlanes.size() >= MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES;
    }
    
    /**
     * Returns quality scores corresponding to each pair of matched samples 
     * (plane and line).
     * The larger the score value the better the quality of the matching.
     * This implementation always returns null.
     * Subclasses using quality scores must implement proper behaviour.
     * @return quality scores corresponding to each pair of matched points.
     */
    public double[] getQualityScores() {
        return null;
    }
    
    /**
     * Sets quality scores corresponding to each pair of matched points.
     * The larger the score value the better the quality of the matching.
     * This implementation makes no action.
     * Subclasses using quality scores must implement proper behaviour.
     * @param qualityScores quality scores corresponding to each pair of matched
     * points.
     * @throws LockedException if robust estimator is locked because an 
     * estimation is already in progress.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES (i.e. 4 samples).
     */
    public void setQualityScores(double[] qualityScores) throws LockedException { }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided robust estimator method.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator
            create(RobustEstimatorMethod method) {
        return DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                method);
    }
            
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided planes and lines and provided robust
     * estimator method.
     * @param planes list of 3D planes to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines
     * don't have the same size or their size is smaller than required minimum
     * size (4 correspondences).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            List<Plane> planes, List<Line2D> lines, 
            RobustEstimatorMethod method) {
        return DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided listener and robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener, 
            RobustEstimatorMethod method) {
        return DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided listener, planes and lines, and robust
     * estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes list of 3D planes to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines
     * don't have the same size or their size is smaller than required minimum
     * size (4 correspondences).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            List<Plane> planes, List<Line2D> lines,
            RobustEstimatorMethod method) {
        return DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided quality scores and robust estimator 
     * method.
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes/lines.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than required minimum size (4 samples).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            double[] qualityScores, RobustEstimatorMethod method) {
        return DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                qualityScores, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided planes and lines, quality scores and 
     * provided robust estimator method.
     * @param planes list of 3D planes to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes/lines.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines or
     * quality scores don't have the same size or their size is smaller than 
     * required minimum size (4 correspondences).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            List<Plane> planes, List<Line2D> lines, double[] qualityScores,
            RobustEstimatorMethod method) {
        return DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                planes, lines, qualityScores, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided listener, quality scores and robust 
     * estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes/lines.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have 
     * the required minimum size (4 samples).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener, 
            double[] qualityScores, RobustEstimatorMethod method) {
        return DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, qualityScores, method);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided listener, planes and lines, and robust
     * estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes list of 3D planes to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes/lines.
     * @param method method of a robust estimator algorithm to estimate best
     * pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines or
     * quality scores don't have the same size or their size is smaller than 
     * required minimum size (4 correspondences).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            List<Plane> planes, List<Line2D> lines, double[] qualityScores,
            RobustEstimatorMethod method) {
        return DLTLinePlaneCorrespondencePinholeCameraRobustEstimator.create(
                listener, planes, lines, qualityScores, method);
    }    
    
   /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using default robust estimator method.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create() {
        return create(DEFAULT_ROBUST_METHOD);
    }
            
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided planes and lines and default robust
     * estimator method.
     * @param planes list of 3D planes to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines
     * don't have the same size or their size is smaller than required minimum
     * size (4 correspondences).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            List<Plane> planes, List<Line2D> lines) {
        return create(planes, lines, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided listener and default estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @return an instance of a pinhole camera robust estimator.
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener) {
        return create(listener, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided listener, planes and lines, and 
     * default robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes list of 3D planes to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines
     * don't have the same size or their size is smaller than required minimum
     * size (4 correspondences).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            List<Plane> planes, List<Line2D> lines) {
        return create(listener, planes, lines, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided quality scores and default robust 
     * estimator method.
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes/lines.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores length is
     * smaller than required minimum size (4 samples).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            double[] qualityScores) {
        return create(qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided planes and lines, quality scores and 
     * default robust estimator method.
     * @param planes list of 3D planes to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes/lines.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines or
     * quality scores don't have the same size or their size is smaller than 
     * required minimum size (4 correspondences).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            List<Plane> planes, List<Line2D> lines, double[] qualityScores) {
        return create(planes, lines, qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided listener, quality scores and default 
     * robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes/lines.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided quality scores don't have 
     * the required minimum size (4 samples).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener, 
            double[] qualityScores) {
        return create(listener, qualityScores, DEFAULT_ROBUST_METHOD);
    }
    
    /**
     * Creates a pinhole camera robust estimator based on plane/line 
     * correspondences and using provided listener, planes and lines, and 
     * default robust estimator method.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or its progress significantly changes.
     * @param planes list of 3D planes to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @param qualityScores quality scores corresponding to each pair of matched
     * planes/lines.
     * @return an instance of a pinhole camera robust estimator.
     * @throws IllegalArgumentException if provided lists of planes and lines or
     * quality scores don't have the same size or their size is smaller than 
     * required minimum size (4 correspondences).
     */
    public static LinePlaneCorrespondencePinholeCameraRobustEstimator create(
            PinholeCameraRobustEstimatorListener listener,
            List<Plane> planes, List<Line2D> lines, double[] qualityScores) {
        return create(listener, planes, lines, qualityScores, 
                DEFAULT_ROBUST_METHOD);
    }    
    
    /**
     * Attempt sto refine provided camera.
     * @param pinholeCamera camera to be refined.
     * @param weight weight for suggestion residual.
     * @return refined camera of provided camera if anything fails.
     */
    protected PinholeCamera attemptRefine(PinholeCamera pinholeCamera,
            double weight) {
        if (mUseFastRefinement) {
            return attemptFastRefine(pinholeCamera, weight);
        } else {
            return attemptSlowRefine(pinholeCamera, weight);
        }
    }
    
    /**
     * Backprojection residual/error for a single line using provided camera.
     * @param pinholeCamera camera ot be checked.
     * @param line line to be backprojected.
     * @param plane plane to check against.
     * @return dot product distance between backprojected line and plane.
     */
    protected double singleBackprojectionResidual(PinholeCamera pinholeCamera, 
            Line2D line, Plane plane) {
        //backproject line into test plane
        pinholeCamera.backProject(line, mResidualTestPlane);
        mResidualTestPlane.normalize();
        
        double dotProduct = Math.abs(plane.getA() * mResidualTestPlane.getA() +
                plane.getB() * mResidualTestPlane.getB() +
                plane.getC() * mResidualTestPlane.getC() + 
                plane.getD() * mResidualTestPlane.getD());
        return 1.0 - dotProduct;
    }    
    
    /**
     * Internal method to set lists of planes and lines to be used to estimate a
     * pinhole camera.
     * This method does not check whether estimator is locked or not.
     * @param planes list of 3D planes used to estimate a pinhole camera.
     * @param lines list of corresponding projected 2D lines used to estimate
     * a pinhole camera.
     * @throws IllegalArgumentException if provided lists of planes/lines don't 
     * have the same size or their size is smaller than required minimum size 
     * (4 samples).
     */
    private void internalSetLinesAndPlanes(List<Plane> planes, 
            List<Line2D> lines) {
        if (planes.size() < MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES) {
            throw new IllegalArgumentException();
        }
        if (lines.size() != planes.size()) {
            throw new IllegalArgumentException();
        }
        mPlanes = planes;
        mLines = lines;
    }
    
    /**
     * Attempts to refine provided camera using a slow but more accurate and
     * stable algorithm by first doing a Powell optimization and then obtaining
     * covariance using LEvenberg/Marquardt if needed.
     * @param pinholeCamera camera to be refined.
     * @param weight weight for suggestion residual.
     * @return refined camera or provided camera if anything fails.
     */
    private PinholeCamera attemptSlowRefine(PinholeCamera pinholeCamera,
            double weight) {
        InliersData inliersData = getInliersData();
        if ((mRefineResult || mKeepCovariance) && inliersData != null) {
            DecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                    new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(
                    pinholeCamera, mKeepCovariance, inliersData, mPlanes,
                    mLines, getRefinementStandardDeviation());
            try {
                if (mRefineResult) {
                    refiner.setMinSuggestionWeight(weight);
                    refiner.setMaxSuggestionWeight(weight);
                    
                    refiner.setSuggestSkewnessValueEnabled(
                            mSuggestSkewnessValueEnabled);
                    refiner.setSuggestedSkewnessValue(mSuggestedSkewnessValue);
                    refiner.setSuggestHorizontalFocalLengthEnabled(
                            mSuggestHorizontalFocalLengthEnabled);
                    refiner.setSuggestedHorizontalFocalLengthValue(
                            mSuggestedHorizontalFocalLengthValue);
                    refiner.setSuggestVerticalFocalLengthEnabled(
                            mSuggestVerticalFocalLengthEnabled);
                    refiner.setSuggestedVerticalFocalLengthValue(
                            mSuggestedVerticalFocalLengthValue);
                    refiner.setSuggestAspectRatioEnabled(
                            mSuggestAspectRatioEnabled);
                    refiner.setSuggestedAspectRatioValue(
                            mSuggestedAspectRatioValue);
                    refiner.setSuggestPrincipalPointEnabled(
                            mSuggestPrincipalPointEnabled);
                    refiner.setSuggestedPrincipalPointValue(
                            mSuggestedPrincipalPointValue);
                    refiner.setSuggestRotationEnabled(mSuggestRotationEnabled);
                    refiner.setSuggestedRotationValue(mSuggestedRotationValue);
                    refiner.setSuggestCenterEnabled(mSuggestCenterEnabled);
                    refiner.setSuggestedCenterValue(mSuggestedCenterValue);                
                }
                
                PinholeCamera result = new PinholeCamera();
                boolean improved = refiner.refine(result);
            
                if (mKeepCovariance) {
                    //keep covariance
                    mCovariance = refiner.getCovariance();
                }
                
                return improved ? result : pinholeCamera;
                 
            } catch (Exception e) {
                return pinholeCamera;   
            }            
        } else {
            mCovariance = null;
            return pinholeCamera;
        }
    }
    
    /**
     * Attempts to refine provided camera using a fast algorithm based on
     * Levenberg/Marquardt.
     * @param pinholeCamera camera to be refined.
     * @param weight weight for suggestion residual.
     * @return refined camera or provided camera if anything fails.
     */
    private PinholeCamera attemptFastRefine(PinholeCamera pinholeCamera, 
            final double weight) {
        InliersData inliersData = getInliersData();
        if (mRefineResult && inliersData != null) {
            NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                    new NonDecomposedLinePlaneCorrespondencePinholeCameraRefiner(
                    pinholeCamera, mKeepCovariance, inliersData, mPlanes,
                    mLines, getRefinementStandardDeviation());
            
            try {
                refiner.setSuggestionErrorWeight(weight);
            
                refiner.setSuggestSkewnessValueEnabled(
                        mSuggestSkewnessValueEnabled);
                refiner.setSuggestedSkewnessValue(mSuggestedSkewnessValue);
                refiner.setSuggestHorizontalFocalLengthEnabled(
                        mSuggestHorizontalFocalLengthEnabled);
                refiner.setSuggestedHorizontalFocalLengthValue(
                        mSuggestedHorizontalFocalLengthValue);
                refiner.setSuggestVerticalFocalLengthEnabled(
                        mSuggestVerticalFocalLengthEnabled);
                refiner.setSuggestedVerticalFocalLengthValue(
                        mSuggestedVerticalFocalLengthValue);
                refiner.setSuggestAspectRatioEnabled(
                        mSuggestAspectRatioEnabled);
                refiner.setSuggestedAspectRatioValue(
                        mSuggestedAspectRatioValue);
                refiner.setSuggestPrincipalPointEnabled(
                        mSuggestPrincipalPointEnabled);
                refiner.setSuggestedPrincipalPointValue(
                        mSuggestedPrincipalPointValue);
                refiner.setSuggestRotationEnabled(mSuggestRotationEnabled);
                refiner.setSuggestedRotationValue(mSuggestedRotationValue);
                refiner.setSuggestCenterEnabled(mSuggestCenterEnabled);
                refiner.setSuggestedCenterValue(mSuggestedCenterValue);                
                
                PinholeCamera result = new PinholeCamera();
                boolean improved = refiner.refine(result);
                
                if (mKeepCovariance) {
                    //keep covariance
                    mCovariance = refiner.getCovariance();
                }
                
                return improved ? result : pinholeCamera;                
            } catch (Exception e) {
                //refinement failed, so we return input value
                return pinholeCamera;
            }                                   
        } else {
            return pinholeCamera;
        }        
    }            
}
