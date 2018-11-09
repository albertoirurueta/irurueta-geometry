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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.*;
import com.irurueta.geometry.refiners.DecomposedPointCorrespondencePinholeCameraRefiner;

import java.util.ArrayList;
import java.util.BitSet;
import java.util.Collections;
import java.util.List;

/**
 * This file contains abstract implementation for pinhole camera estimators 
 * based on point correspondences.
 */
@SuppressWarnings("WeakerAccess")
public abstract class PointCorrespondencePinholeCameraEstimator extends 
        PinholeCameraEstimator {
    
    /**
     * Minimum number of required point correspondences to estimate a pinhole
     * camera.
     */
    public static final int MIN_NUMBER_OF_POINT_CORRESPONDENCES = 6;
    
    /**
     * Indicates if by default provided point correspondences are normalized to
     * increase the accuracy of the estimation.
     */
    public static final boolean DEFAULT_NORMALIZE_POINT_CORRESPONDENCES = true;
    
    /**
     * Defines tiny value considered as machine precision.
     */
    public static final double EPS = 1e-8;    
    
    /**
     * List of corresponding 3D points.
     */
    protected List<Point3D> mPoints3D;
    
    /**
     * List of corresponding 2D points.
     */
    protected List<Point2D> mPoints2D;
    
    /**
     * Indicates if provided point correspondences are normalized to increase
     * the accuracy of the estimation.
     */
    protected boolean mNormalizePointCorrespondences;    
        
    /**
     * Constructor.
     */
    public PointCorrespondencePinholeCameraEstimator() {
        super();
        mNormalizePointCorrespondences = 
                DEFAULT_NORMALIZE_POINT_CORRESPONDENCES;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public PointCorrespondencePinholeCameraEstimator(
            PinholeCameraEstimatorListener listener) {
        super(listener);
        mNormalizePointCorrespondences = 
                DEFAULT_NORMALIZE_POINT_CORRESPONDENCES;
    }
    
    /**
     * Constructor.
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of points don't have
     * the same size and enough points.
     */
    public PointCorrespondencePinholeCameraEstimator(List<Point3D> points3D,
            List<Point2D> points2D) throws IllegalArgumentException,
            WrongListSizesException {
        super();
        internalSetLists(points3D, points2D);
        mNormalizePointCorrespondences = 
                DEFAULT_NORMALIZE_POINT_CORRESPONDENCES;        
    }
    
    /**
     * Constructor.
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of points don't have
     * the same size and enough points.
     */
    public PointCorrespondencePinholeCameraEstimator(List<Point3D> points3D,
            List<Point2D> points2D, PinholeCameraEstimatorListener listener) 
            throws IllegalArgumentException, WrongListSizesException {
        super(listener);
        internalSetLists(points3D, points2D);
        mNormalizePointCorrespondences = 
                DEFAULT_NORMALIZE_POINT_CORRESPONDENCES;        
    }
    
    /**
     * Internal method to set list of corresponding points (it does not check
     * if estimator is locked).
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of points don't have
     * the same size and enough points.
     */
    private void internalSetLists(List<Point3D> points3D, 
            List<Point2D> points2D) throws IllegalArgumentException,
            WrongListSizesException {
        
        if (points3D == null || points2D == null) {
            throw new IllegalArgumentException();
        }
        
        if (!areValidLists(points3D, points2D)) {
            throw new WrongListSizesException();
        }
        
        mPoints3D = points3D;
        mPoints2D = points2D;        
    }
    
    /**
     * Sets list of corresponding points.
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @throws LockedException if estimator is locked.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of points don't have
     * the same size and enough points.
     */
    public void setLists(List<Point3D> points3D, List<Point2D> points2D)
            throws LockedException, IllegalArgumentException, 
            WrongListSizesException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        internalSetLists(points3D, points2D);
    }
    
    /**
     * Returns list of corresponding 3D points.
     * Notice that this method returns an unmodifiable list of points to avoid
     * undesired modifications.
     * @return list of corresponding 3D points.
     * @throws NotAvailableException if list of points is not yet available.
     */
    public List<Point3D> getPoints3D() throws NotAvailableException {
        if (mPoints3D == null) {
            throw new NotAvailableException();
        }
        
        //to avoid undesired modifications
        return Collections.unmodifiableList(mPoints3D);
    }
    
    /**
     * Returns list of corresponding 2D points.
     * Notice that this method returns an unmodifiable list of points to avoid
     * undesired modifications.
     * @return list of corresponding 2D points.
     * @throws NotAvailableException if list of points is not yet available.
     */
    public List<Point2D> getPoints2D() throws NotAvailableException {
        if (mPoints2D == null) {
            throw new NotAvailableException();
        }
        
        //to avoid undesired modifications
        return Collections.unmodifiableList(mPoints2D);
    }
    
    /**
     * Indicates if lists of corresponding 2D/3D points are valid.
     * Lists are considered valid if they have the same number of points and 
     * both have more than the required minimum of correspondences (which is 6).
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @return true if corresponding 2D/3D points are valid, false otherwise.
     */
    public static boolean areValidLists(List<Point3D> points3D, 
            List<Point2D> points2D) {
        if (points3D == null || points2D == null) {
            return false;
        }
        return points3D.size() == points2D.size() && 
                points3D.size() >= MIN_NUMBER_OF_POINT_CORRESPONDENCES;
    }
    
    /**
     * Indicates if lists have already been provided and are available for
     * retrieval.
     * @return true if available, false otherwise.
     */
    public boolean areListsAvailable() {
        return mPoints3D != null && mPoints2D != null;
    }
    
    /**
     * Indicates if provided point correspondences are normalized to increase
     * the accuracy of the estimation.
     * @return true if input point correspondences will be normalized, false
     * otherwise.
     */
    public boolean arePointCorrespondencesNormalized() {
        return mNormalizePointCorrespondences;
    }
    
    /**
     * Specifies whether provided point correspondences are normalized to 
     * increase the accuracy of the estimation.
     * @param normalize true if input point correspondences will be normalized,
     * false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setPointCorrespondencesNormalized(boolean normalize)
        throws LockedException {
        
        if (isLocked()) {
            throw new LockedException();
        }
        mNormalizePointCorrespondences = normalize;
    }
    
    /**
     * Transforms 2D points so that they have zero mean and unitary standard
     * deviation. Provided transformation will be updated containing the
     * transformation used for input points.
     * @param list list of input 2D points.
     * @param inverseTransformation inverseTransformation used on input points. 
     * This is an output variable.
     * @return transformed 2D points.
     * @throws PinholeCameraEstimatorException if transformation cannot be
     * computed because point configuration might be degenerate.
     */
    protected List<Point2D> transformPoints2D(List<Point2D> list, 
            ProjectiveTransformation2D inverseTransformation) 
            throws PinholeCameraEstimatorException {
        
        //compute image point coordinates limits
        double x, y;
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        for (Point2D point : list) {
            point.normalize();
            x = point.getInhomX();
            y = point.getInhomY();
            
            if(x < minX) minX = x;
            if(x > maxX) maxX = x;
            if(y < minY) minY = y;
            if(y > maxY) maxY = y;
        }
        
        //compute size of image plane (based on 2D coordinates limits), scale 
        //and centroid
        double width = maxX - minX;
        double height = maxY - minY;
        
        double norm = Math.sqrt(width * width + height * height);
        //points are too close to each other (norm is too small).
        //This is a degenerate configuration and results will be meaningless        
        if (norm < EPS) {
            throw new PinholeCameraEstimatorException();
        }
        
        double scale = 1.0 / norm;
        double centroidX = (minX + maxX) / 2.0;
        double centroidY = (minY + maxY) / 2.0;
        
        //set inverse transformation matrix
        try {
            Matrix inverseTransformationMatrix=
                new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH,
                Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
        
            inverseTransformationMatrix.setElementAt(0, 0, norm);
            inverseTransformationMatrix.setElementAt(1, 1, norm); //1.0 / scale
            inverseTransformationMatrix.setElementAt(2, 2, 1.0);
            inverseTransformationMatrix.setElementAt(0, 2, centroidX);
            inverseTransformationMatrix.setElementAt(1, 2, centroidY);
        
            inverseTransformation.setT(inverseTransformationMatrix);
        } catch (WrongSizeException ignore) { }
        
        inverseTransformation.normalize();
        
        
        //transform list of image points
        List<Point2D> transformedPoints = new ArrayList<>(list.size());
        HomogeneousPoint2D homPoint;
        double homX, homY, homW;
        for (Point2D point : list) {
            point.normalize();
            homX = point.getHomX();
            homY = point.getHomY();
            homW = point.getHomW();
            
            homPoint = new HomogeneousPoint2D(
                    scale * (homX - centroidX * homW),
                    scale * (homY - centroidY * homW), homW);
            //normalize point to increase accuracy
            homPoint.normalize();
            transformedPoints.add(homPoint);
        }
            
        return transformedPoints;
    }
    
    /**
     * Transforms 3D points so that they have zero mean and unitary standard
     * deviation. Provided transformation will be updated containing the
     * transformation used for input points.
     * @param list list of input 3D points.
     * @param transformation transformation used on input points. This is an
     * output variable.
     * @return transformed 3D points.
     * @throws PinholeCameraEstimatorException if transformation cannot be
     * computed because point configuration might be degenerate.
     */
    protected List<Point3D> transformPoints3D(List<Point3D> list,
            ProjectiveTransformation3D transformation) 
            throws PinholeCameraEstimatorException {
        
        //compute image point coordinates limits
        double x, y, z;
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        double minZ = Double.MAX_VALUE;
        double maxZ = -Double.MAX_VALUE;        
        for (Point3D point : list) {
            point.normalize();
            x = point.getInhomX();
            y = point.getInhomY();
            z = point.getInhomZ();
            
            if(x < minX) minX = x;
            if(x > maxX) maxX = x;
            if(y < minY) minY = y;
            if(y > maxY) maxY = y;
            if(z < minZ) minZ = z;
            if(z > maxZ) maxZ = z;
        }
        
        //compute size of image plane (based on 2D coordinates limits), scale 
        //and centroid
        double width = maxX - minX;
        double height = maxY - minY;
        double depth = maxZ - minZ;
        
        double norm = Math.sqrt(width * width + height * height + 
                depth * depth);
        //points are too close to each other (norm is too small).
        //This is a degenerate configuration and results will be meaningless        
        if (norm < EPS) {
            throw new PinholeCameraEstimatorException();
        }
        
        double scale = 1.0 / norm;
        double centroidX = (minX + maxX) / 2.0;
        double centroidY = (minY + maxY) / 2.0;
        double centroidZ = (minZ + maxZ) / 2.0;
        
        //set transformation matrix
        try {
            Matrix transformMatrix = new Matrix(
                Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH,
                Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH);
            transformMatrix.setElementAt(0, 0, scale);
            transformMatrix.setElementAt(1, 1, scale);
            transformMatrix.setElementAt(2, 2, scale);
            transformMatrix.setElementAt(3, 3, 1.0);
            transformMatrix.setElementAt(0, 3, -scale * centroidX);
            transformMatrix.setElementAt(1, 3, -scale * centroidY);
            transformMatrix.setElementAt(2, 3, -scale * centroidZ);
        
            //normalize transformation to increase accuracy
            transformation.setT(transformMatrix);
        } catch (WrongSizeException ignore) { }
        transformation.normalize();
                    
        //transform list of world points
        List<Point3D> transformedPoints = new ArrayList<>(list.size());
        for (Point3D point : list) {
            transformedPoints.add(transformation.transformAndReturnNew(point));
        }
            
        return transformedPoints;
    }    
    
    /**
     * Estimates a pinhole camera.
     * @return estimated pinhole camera.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if input has not yet been provided.
     * @throws PinholeCameraEstimatorException if an error occurs during 
     * estimation, usually because input data is not valid.
     */
    @Override
    public PinholeCamera estimate() throws LockedException, NotReadyException, 
        PinholeCameraEstimatorException {
        
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }
        
        try {
            mLocked = true;
            if (mListener != null) {
                mListener.onEstimateStart(this);
            }

            ProjectiveTransformation2D inverseTransformation2D = 
                    new ProjectiveTransformation2D();
            ProjectiveTransformation3D transformation3D = 
                    new ProjectiveTransformation3D();
            List<Point2D> inputPoints2D;
            List<Point3D> inputPoints3D;

            if (mNormalizePointCorrespondences) {
                //normalize 2D and 3D points and update transformations
                inputPoints2D = transformPoints2D(mPoints2D, inverseTransformation2D);
                inputPoints3D = transformPoints3D(mPoints3D, transformation3D);
            } else {
                inputPoints2D = mPoints2D;
                inputPoints3D = mPoints3D;
            }
            
            Matrix pinholeCameraMatrix = internalEstimate(inputPoints3D, 
                    inputPoints2D);
            
            if (mNormalizePointCorrespondences) {
                inverseTransformation2D.normalize();
                transformation3D.normalize();
                
                //denormalize pinhole camera                
                Matrix invTrans2DMatrix = inverseTransformation2D.asMatrix();
                Matrix trans3DMatrix = transformation3D.asMatrix();
                                
                invTrans2DMatrix.multiply(pinholeCameraMatrix);
                invTrans2DMatrix.multiply(trans3DMatrix);
                
                pinholeCameraMatrix = invTrans2DMatrix;
                                
                //normalize by frobenius norm to increase accuracy after point
                //denormalization
                double norm = Utils.normF(pinholeCameraMatrix);
                pinholeCameraMatrix.multiplyByScalar(1.0 / norm);
            }
            
            PinholeCamera camera = new PinholeCamera(pinholeCameraMatrix);
                        
            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
            
            return attemptRefine(camera);
            
        } catch (PinholeCameraEstimatorException e) {
            throw e;
        } catch (Exception e) {
            throw new PinholeCameraEstimatorException(e);
        } finally {
            mLocked = false;
        }
    }  
        
    /**
     * Internal method that actually computes the normalized pinhole camera
     * internal matrix.
     * Returned matrix must have norm equal to one and might be estimated using
     * any convenient algorithm (i.e. DLT or weighted DLT).
     * @param points3D list of 3D points. Points might or might not be 
     * normalized.
     * @param points2D list of 2D points. Points might or might not be 
     * normalized.
     * @return matrix of estimated pinhole camera.
     * @throws PinholeCameraEstimatorException if estimation fails for some
     * reason (i.e. numerical instability or geometric degeneracy).
     */
    protected abstract Matrix internalEstimate(List<Point3D> points3D,
            List<Point2D> points2D) throws PinholeCameraEstimatorException;
        
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
            int numPoints = mPoints3D.size();
            BitSet inliers = new BitSet(numPoints);
            inliers.set(0, numPoints, true);
            double[] residuals = new double[numPoints];
            
            DecomposedPointCorrespondencePinholeCameraRefiner refiner =
                    new DecomposedPointCorrespondencePinholeCameraRefiner(
                    pinholeCamera, false, inliers, residuals, numPoints, 
                    mPoints3D, mPoints2D, 0.0);
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
