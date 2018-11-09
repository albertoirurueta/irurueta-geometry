/*
 * Copyright (C) 2013 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.refiners.DecomposedLinePlaneCorrespondencePinholeCameraRefiner;
import java.util.BitSet;
import java.util.Collections;
import java.util.List;

/**
 * This file contains abstract implementation for pinhole camera estimators
 * based on line/plane correspondences.
 */
public abstract class LinePlaneCorrespondencePinholeCameraEstimator extends
        PinholeCameraEstimator {
    
    /**
     * List of corresponding 3D planes.
     */
    protected List<Plane> mPlanes;
    
    /**
     * List of corresponding 2D lines.
     */
    protected List<Line2D> mLines2D;
    
    /**
     * Minimum number of required line/plane correspondences to estimate a
     * camera.
     */
    public static final int MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES = 4;
    
    /**
     * Constructor.
     */
    public LinePlaneCorrespondencePinholeCameraEstimator() {
        super();
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public LinePlaneCorrespondencePinholeCameraEstimator(
            PinholeCameraEstimatorListener listener) {
        super(listener);
    }
    
    /**
     * Constructor.
     * @param planes list of corresponding 3D planes.
     * @param lines2D list of corresponding 2D lines.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of points don't have
     * the same size and enough correspondences.
     */
    public LinePlaneCorrespondencePinholeCameraEstimator(List<Plane> planes,
            List<Line2D> lines2D) throws IllegalArgumentException,
            WrongListSizesException {
        super();
        internalSetLists(planes, lines2D);
    }

    /**
     * Constructor.
     * @param planes list of corresponding 3D planes.
     * @param lines2D list of corresponding 2D lines.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of points don't have
     * the same size and enough correspondences.
     */
    public LinePlaneCorrespondencePinholeCameraEstimator(List<Plane> planes,
            List<Line2D> lines2D, PinholeCameraEstimatorListener listener) 
            throws IllegalArgumentException, WrongListSizesException {
        super(listener);
        internalSetLists(planes, lines2D);
    }
    
    /**
     * Internal method to set list of corresponding lines/planes (it does not
     * check if estimator is locked).
     * @param planes list of corresponding 3D planes.
     * @param lines2D list of corresponding 2D lines.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of points don't have
     * the same size and enough correspondences.
     */
    private void internalSetLists(List<Plane> planes, List<Line2D> lines2D)
            throws IllegalArgumentException, WrongListSizesException {
        
        if (planes == null || lines2D == null) {
            throw new IllegalArgumentException();
        }
        
        if (!areValidLists(planes, lines2D)) {
            throw new WrongListSizesException();
        }
        
        mPlanes = planes;
        mLines2D = lines2D;
    }
    
    /**
     * Set list of corresponding lines/planes.
     * @param planes list of corresponding 3D planes.
     * @param lines2D list of corresponding 2D lines.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of points don't have
     * the same size and enough correspondences.
     */
    public void setLists(List<Plane> planes, List<Line2D> lines2D)
            throws LockedException, IllegalArgumentException,
            WrongListSizesException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        internalSetLists(planes, lines2D);
    }
    
    /**
     * Returns list of corresponding lines/planes.
     * Notice that this method returns an unmodifiable list of planes to avoid
     * undesired modifications.
     * @return list of corresponding 3D planes.
     * @throws NotAvailableException if list of planes is not yet available.
     */
    public List<Plane> getPlanes() throws NotAvailableException {
        if (mPlanes == null) {
            throw new NotAvailableException();
        }
        
        //to avoid undesired modifications
        return Collections.unmodifiableList(mPlanes);
    }
    
    /**
     * Returns list of corresponding 2D lines.
     * Notice that this method returns an unmodifiable list of 2D lines to avoid
     * undesired modifications.
     * @return list of corresponding 2D lines.
     * @throws NotAvailableException if list of 2D lines is not yet available.
     */
    public List<Line2D> getLines2D() throws NotAvailableException {
        if (mLines2D == null) {
            throw new NotAvailableException();
        }
        
        //to avoid undesired modifications
        return Collections.unmodifiableList(mLines2D);
    }
    
    /**
     * Indicates if lists of corresponding 2D lines and planes are valid.
     * Lists are considered valid if they have the same number of points and
     * both have more than the required minimum of correspondences (which is 4).
     * @param planes list of corresponding 3D planes.
     * @param lines2D list of corresponding 2D lines.
     * @return true if corresponding 2D lines and planes are valid, false 
     * otherwise.
     */
    public static boolean areValidLists(List<Plane> planes, 
            List<Line2D> lines2D) {
        if (planes == null || lines2D == null) {
            return false;
        }
        return planes.size() == lines2D.size() &&
                lines2D.size() >= MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES;
    }
    
    /**
     * Indicates if lists have already been provided and are available for
     * retrieval.
     * @return true if available, false otherwise.
     */
    public boolean areListsAvailable() {
        return mPlanes != null && mLines2D != null;
    }
    
    /**
     * Attempts to refine provided camera using requested suggestions.
     * If no suggestions are requested or if refinement fails, provided
     * camera is returned instead.
     * @param pinholeCamera camera to be refined.
     * @return refined camera.
     */
    @Override
    protected PinholeCamera attemptRefine(PinholeCamera pinholeCamera) {
        if (hasSuggestions()) {
            int numPoints = mPlanes.size();
            BitSet inliers = new BitSet(numPoints);
            inliers.set(0, numPoints, true);
            double[] residuals = new double[numPoints];
            
            DecomposedLinePlaneCorrespondencePinholeCameraRefiner refiner =
                    new DecomposedLinePlaneCorrespondencePinholeCameraRefiner(
                    pinholeCamera, false, inliers, residuals, numPoints, 
                    mPlanes, mLines2D, 0.0);
            try {
                refiner.setMinSuggestionWeight(mMinSuggestionWeight);
                refiner.setMaxSuggestionWeight(mMaxSuggestionWeight);
                refiner.setSuggestionWeightStep(mSuggestionWeightStep);
                    
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
            
                return improved ? result : pinholeCamera;
                 
            } catch (Exception e) {
                return pinholeCamera;   
            }            
        } else {
            return pinholeCamera;
        }
    }        
}
