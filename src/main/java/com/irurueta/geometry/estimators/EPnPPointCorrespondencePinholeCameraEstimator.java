/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.*;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.*;

import java.util.ArrayList;
import java.util.List;

/**
 * EPnP (Efficient Perspective-n-Point) implementation to estimate pinhole 
 * cameras from 2D/3D point correspondences.
 * This class is an implementation following the one proposed by Vincent Lepetit
 * on "EPnP: An Accurate O(n) Solution to the PnP Problem" with some minor 
 * changes and improvements.
 * Paper and source code can be found at:
 * http://cvlabwww.epfl.ch/~lepetit/papers/lepetit_ijcv08.pdf
 * http://cvlab.epfl.ch/EPnP/index.php
 */
public class EPnPPointCorrespondencePinholeCameraEstimator extends
        PointCorrespondencePinholeCameraEstimator {
    
    /**
     * Indicates that by default planar configuration is checked to determine
     * whether point correspondences are in such configuration and find a 
     * specific solution for such case.
     */
    public static final boolean DEFAULT_PLANAR_CONFIGURATION_ALLOWED = true;

    /**
     * Indicates that by default a dimension 2 nullspace is allowed.
     */
    public static final boolean DEFAULT_NULLSPACE_DIMENSION2_ALLOWED = true;
    
    /**
     * Indicates that by default a dimension 3 nullspace is allowed.
     */
    public static final boolean DEFAULT_NULLSPACE_DIMENSION3_ALLOWED = true;    
            
    /**
     * Default threshold to determine whether 3D matched points are in a
     * planar configuration.
     * Points are considered to be laying in a plane when the smallest singular
     * value of their covariance matrix has a value much smaller than the 
     * second smallest as many times as this value. 
     */
    public static final double DEFAULT_PLANAR_THRESHOLD = 1e13;
        
    /**
     * Number of control points used in a general configuration.
     */
    private static final int GENERAL_NUM_CONTROL_POINTS = 4;
    
    /**
     * Number of control points used in a planar configuration.
     */
    private static final int PLANAR_NUM_CONTROL_POINTS = 3;
    
    /**
     * Indicates whether planar configuration is checked to determine whether
     * point correspondences are in such configuration and find a specific
     * solution for such case.
     */
    private boolean mPlanarConfigurationAllowed = 
            DEFAULT_PLANAR_CONFIGURATION_ALLOWED;
    
    /**
     * Inicates whether the case where a dimension 2 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok.
     */
    private boolean mNullspaceDimension2Allowed = 
            DEFAULT_NULLSPACE_DIMENSION2_ALLOWED;
    
    /**
     * Indicates whether the case where a dimension 3 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok although less precise than
     * when a nullspace of dimension 2 is used.
     */
    private boolean mNullspaceDimension3Allowed = 
            DEFAULT_NULLSPACE_DIMENSION3_ALLOWED;    
    
    /**
     * Threshold to determine whether 3D matched points are in a planar 
     * configuration.
     * Points are considered to be laying in a plane when the smallest singular
     * value of their covariance matrix has a value much smaller than the 
     * largest one as many times as this value. 
     */
    private double mPlanarThreshold = DEFAULT_PLANAR_THRESHOLD;
    
    /**
     * Intrinsic parameters of camera to be estimated.
     */
    private PinholeCameraIntrinsicParameters mIntrinsic;
    
    /**
     * Indicates whether provided correspondences were found to be laying in a
     * planar configuration during the estimation.
     */
    private boolean mIsPlanar;
    
    /**
     * Computed control points in world coordinates.
     */
    private List<Point3D> mControlWorldPoints;    
    
    /**
     * Contains barycentric coordinates to express 3D world point in terms of
     * control points.
     * For general configuration, each row contains 4 coordinates and alphas
     * has size nx4, where n is the number of provided 3D world points.
     * For planar configuration, each row contains 3 coordinates and alphas
     * has size nx3, where n is the number of provided 3D world points.
     * both reference frames are centered in the centroid), alphas can be used
     * in both world and camera coordinates.
     */
    private Matrix mAlphas;
    
    /**
     * M matrix to find control points in camera coordinates.
     * M has size 2*n x 12 (general configuration) or 2*n x 9
     * (planar configuration), where n is the number of provided 2D observed 
     * points.
     */
    private Matrix mM;
    
    /**
     * List containing columns of nullspace of M. Linear combinations of these
     * columns contain possible solutions for control points coordinates in 
     * camera reference (up to scale).
     * First item of the list contains last column of v, which corresponds to
     * the smallest singular value.
     * Last item of the list contains (column - number of control points) column
     * of v.
     */
    private List<double[]> mNullspace;
    
    /**
     * Possible solutions for the estimation.
     */
    private List<Solution> mSolutions;
    
    /**
     * Constructor.
     */
    public EPnPPointCorrespondencePinholeCameraEstimator() {
        super();
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public EPnPPointCorrespondencePinholeCameraEstimator(
            PinholeCameraEstimatorListener listener) {
        super(listener);
    }
    
    /**
     * Constructor.
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of points don't have 
     * the same size and enough points.
     */
    public EPnPPointCorrespondencePinholeCameraEstimator(List<Point3D> points3D, 
            List<Point2D> points2D) throws WrongListSizesException {
        super();
        internalSetLists(points3D, points2D);
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
    public EPnPPointCorrespondencePinholeCameraEstimator(List<Point3D> points3D, 
            List<Point2D> points2D, PinholeCameraEstimatorListener listener)
            throws WrongListSizesException {
        super(listener);
        internalSetLists(points3D, points2D);
    }

    /**
     * Constructor.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     */
    public EPnPPointCorrespondencePinholeCameraEstimator(
            PinholeCameraIntrinsicParameters intrinsic) {
        this();
        mIntrinsic = intrinsic;
    }
    
    /**
     * Constructor with listener.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if absolute values of focal lengths are 
     * too small.
     */
    public EPnPPointCorrespondencePinholeCameraEstimator(
            PinholeCameraIntrinsicParameters intrinsic,
            PinholeCameraEstimatorListener listener) {
        this(listener);
        mIntrinsic = intrinsic;
    }
    
    /**
     * Constructor.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @throws IllegalArgumentException if any of the lists are null or if 
     * absolute values of focal lengths are too small.
     * @throws WrongListSizesException if provided lists of points don't have 
     * the same size and enough points.
     */
    public EPnPPointCorrespondencePinholeCameraEstimator(
            PinholeCameraIntrinsicParameters intrinsic,
            List<Point3D> points3D, List<Point2D> points2D) 
            throws WrongListSizesException {
        this(points3D, points2D);
        mIntrinsic = intrinsic;
    }
    
    /**
     * Constructor.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if any of the lists are null or if 
     * absolute values of focal lengths are too small.
     * @throws WrongListSizesException if provided lists of points don't have 
     * the same size and enough points.
     */
    public EPnPPointCorrespondencePinholeCameraEstimator(
            PinholeCameraIntrinsicParameters intrinsic,
            List<Point3D> points3D, List<Point2D> points2D, 
            PinholeCameraEstimatorListener listener) 
            throws WrongListSizesException {
        this(points3D, points2D, listener);
        mIntrinsic = intrinsic;
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
    @Override
    public void setLists(List<Point3D> points3D, List<Point2D> points2D)
            throws LockedException, WrongListSizesException {
        if (isLocked()) {
            throw new LockedException();
        }
        
        internalSetLists(points3D, points2D);
    }
            
    /**
     * Indicates whether planar configuration is checked to determine whether
     * point correspondences are in such configuration and find a specific
     * solution for such case.
     * @return true to allow specific solutions for planar configurations,
     * false to always find a solution assuming the general case.
     */
    public boolean isPlanarConfigurationAllowed() {
        return mPlanarConfigurationAllowed;
    }
    
    /**
     * Specifies whether planar configuration is checked to determine whether
     * point correspondences are in such configuration and find a specific
     * solution for such case.
     * @param planarConfigurationAllowed true to allow specific solutions for
     * planar configurations, false to always find a solution assuming the 
     * general case.
     * @throws LockedException if estimator is locked.
     */
    public void setPlanarConfigurationAllowed(
            boolean planarConfigurationAllowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mPlanarConfigurationAllowed = planarConfigurationAllowed;
    }
    
    /**
     * Indicates whether the case where a dimension 2 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok.
     * @return true to allow 2 dimensional nullspace, false otherwise.
     */
    public boolean isNullspaceDimension2Allowed() {
        return mNullspaceDimension2Allowed;
    }
    
    /**
     * Specifies whether the case where a dimension 2 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok.
     * @param nullspaceDimension2Allowed true to allow 2 dimensional nullspace,
     * false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setNullspaceDimension2Allowed(
            boolean nullspaceDimension2Allowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mNullspaceDimension2Allowed = nullspaceDimension2Allowed;
    }
        
    /**
     * Indicates whether the case where a dimension 3 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure 
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok although less precise than
     * when a nullspace of dimension 2 is used.
     * @return true to allow 3 dimensional nullspace, false otherwise.
     */
    public boolean isNullspaceDimension3Allowed() {
        return mNullspaceDimension3Allowed;
    }
    
    /**
     * Specifies whether the case where a dimension 3 nullspace is allowed.
     * When allowed, additional constraints are taken into account to ensure
     * equality of scales so that less point correspondences are required.
     * Enabling this parameter is usually ok although less precise than
     * when a nullspace of dimension 2 is used.
     * @param nullspaceDimension3Allowed true to allow 3 dimensional nullspace,
     * false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setNullspaceDimension3Allowed(
            boolean nullspaceDimension3Allowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mNullspaceDimension3Allowed = nullspaceDimension3Allowed;
    }
    
    /**
     * Gets threshold to determine whether 3D matched points are in a planar
     * configuration.
     * Points are considered to be laying in a plane when the smallest singular
     * value of their covariance matrix has a value much smaller than the
     * largest one as many times as this value.
     * @return threshold to determine whether 3D matched points are in a planar
     * configuration.
     */
    public double getPlanarThreshold() {
        return mPlanarThreshold;
    }
    
    /**
     * Sets threshold to determine whether 3D matched points are in a planar
     * configuration.
     * Points are considered to be laying in a plane when the smallest singular
     * value of their covariance matrix has a value much smaller than the
     * largest one as many times as this value.
     * @param planarThreshold threshold to determine whether 3D matched points 
     * are in a planar configuration.
     * @throws IllegalArgumentException if provided threshold is negative.
     * @throws LockedException if estimator is locked.
     */
    public void setPlanarThreshold(double planarThreshold) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (planarThreshold < 0.0) {
            throw new IllegalArgumentException();
        }
        mPlanarThreshold = planarThreshold;
    }
    
    /**
     * Gets intrinsic parameters of camera to be estimated.
     * @return intrinsic parameters of camera to be estimated.
     */
    public PinholeCameraIntrinsicParameters getIntrinsic() {
        return mIntrinsic;
    }
    
    /**
     * Sets intrinsic parameters of camera to be estimated.
     * @param intrinsic intrinsic parameters of camera to be estimated.
     * @throws LockedException if estimator is locked.
     */
    public void setIntrinsic(PinholeCameraIntrinsicParameters intrinsic)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mIntrinsic = intrinsic;
    }
    
    /**
     * Indicates if this estimator is ready to start the estimation.
     * @return true if estimator is ready, false otherwise.
     */    
    @Override
    public boolean isReady() {
        return areListsAvailable() && areValidLists(mPoints3D, mPoints2D) &&
                mIntrinsic != null;
    }

    /**
     * Returns type of pinhole camera estimator.
     * @return type of pinhole camera estimator.
     */    
    @Override
    public PinholeCameraEstimatorType getType() {
        return PinholeCameraEstimatorType.EPnP_PINHOLE_CAMERA_ESTIMATOR;
    }
    
    /**
     * Indicates if provided point correspondences are normalized to increase
     * the accuracy of the estimation.
     * @return true if input point correspondences will be normalized, false
     * otherwise.
     */
    @Override
    public boolean arePointCorrespondencesNormalized() {
        return false;
    }
    
    /**
     * Specifies whether provided point correspondences are normalized to 
     * increase the accuracy of the estimation.
     * @param normalize true if input point correspondences will be normalized,
     * false otherwise.
     * @throws LockedException if estimator is locked.
     */
    @Override
    public void setPointCorrespondencesNormalized(boolean normalize)
        throws LockedException {
        
        if (isLocked()) {
            throw new LockedException();
        }
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
            
            computeWorldControlPointsAndPointConfiguration();
            computeBarycentricCoordinates();
            buildM();
            solveNullspace();
        } catch (AlgebraException e) {
            mLocked = false;
            throw new PinholeCameraEstimatorException(e);
        }

            
        mSolutions = new ArrayList<>();
            
        //general case
        try {
            generalSolution1();
        } catch (GeometryException ignore) {
            //continue attempting 2nd solution if 1st one fails
        }
        if (mNullspaceDimension2Allowed) {
            try {
                generalSolution2();
            } catch (GeometryException | AlgebraException ignore) {
                //continue attempting 3rd solution if 2nd one fails
            }
        }
        if (mNullspaceDimension3Allowed && !mIsPlanar) {
            try {
                generalSolution3();
            } catch (GeometryException | AlgebraException ignore) {
                //3rd solution could not be found
            }
        }
            
        //pick best solution
        Solution bestSolution = pickBestSolution();
            
        if (mListener != null) {
            mListener.onEstimateEnd(this);
        }
            
        if (bestSolution == null) {
            throw new PinholeCameraEstimatorException();
        }
        mLocked = false;
        return attemptRefine(bestSolution.camera);
    }
        
 
    /**
     * Indicates whether provided correspondences were found to be laying in a
     * planar configuration during the estimation.
     * @return true if point correspondences are in a planar configuration, 
     * false otherwise.
     */
    public boolean isPlanar() {
        return mIsPlanar;
    }                
    
    /**
     * Internal method that actually computes the normalized pinhole camera
     * internal matrix.
     * This implementation makes no action.
     * @param points3D list of 3D points. Points might or might not be 
     * normalized.
     * @param points2D list of 2D points. Points might or might not be 
     * normalized.
     * @return matrix of estimated pinhole camera.
     */
    @Override
    protected Matrix internalEstimate(List<Point3D> points3D, 
            List<Point2D> points2D) {
        return null;
    }    
        
    /**
     * Internal method to set list of corresponding points (it does not check
     * if estimator is locked).
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @throws IllegalArgumentException if any of the lists are null
     * @throws WrongListSizesException if provided lists of points don't have
     * the same size and enough points.
     */
    private void internalSetLists(List<Point3D> points3D, 
            List<Point2D> points2D) throws WrongListSizesException {
        
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
     * Picks best solution (the one having the smallest reprojection error).
     * @return best solution.
     */
    private Solution pickBestSolution() {
        Solution bestSolution = null;
        double bestError = Double.MAX_VALUE;
        for(Solution s : mSolutions) {
            if(s.reprojectionError < bestError) {
                bestError = s.reprojectionError;
                bestSolution = s;
            }
        }
        
        return bestSolution;
    }    
    
    /**
     * Tests solution 3 for general point configuration.
     * Because solution is up to scale, 8 different solutions for different 
     * beta1, beta2 and beta3 signs are tried.
     * @throws AlgebraException if a numerical degeneracy occurs.
     * @throws LockedException never happens.
     * @throws NotReadyException never happens.
     * @throws CoincidentPointsException if a point degeneracy has occurred.
     */
    private void generalSolution3() throws AlgebraException, 
            LockedException, NotReadyException, CoincidentPointsException {
        if (mIsPlanar) {
            return;
        }
        
        //we have the distance constraints between control world points (c) and
        //control camera points (v):
        //||(beta1*vai + beta2*vbi + beta3*vci) - (beta1*vaj + beta2*vbj + beta3*vcj)||^2 = ||ci - cj||^2,	i,j 1...4

        //((beta1*vaix + beta2*vbix + beta3*vcix) - (beta1*vajx + beta2*vbjx + beta3*vcjx))^2 + 
        //((beta1*vaiy + beta2*vbiy + beta3*vciy) - (beta1*vajy + beta2*vbjy + beta3*vcjy))^2 +
        //((beta1*vaiz + beta2*vbiz + beta3*vciz) - (beta1*vajz + beta2*vbjz + beta3*vcjz))^2 = 
        //(cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2,		i,j 1...4

        //(beta1*(vaix - vajx) + beta2*(vbix - vbjx) + beta3*(vcix - vcjx))^2 +
        //(beta1*(vaiy - vajy) + beta2*(vbiy - vbjy) + beta3*(vciy - vcjy))^2 +
        //(beta1*(vaiz - vajz) + beta2*(vbiz - vbjz) + beta3*(vciz - vcjz))^2 =
        //(cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2,		i,j 1...4

        //beta1^2*(vaix - vajx)^2 + 2*beta1*(vaix - vajx)*(beta2*(vbix - vbjx) + beta3*(vcix - vcjx)) + (beta2*(vbix - vbjx) + beta3*(vcix - vcjx))^2 +
        //beta1^2*(vaiy - vajy)^2 + 2*beta1*(vaiy - vajy)*(beta2*(vbiy - vbjy) + beta3*(vciy - vcjy)) + (beta2*(vbiy - vbjy) + beta3*(vciy - vcjy))^2 +
        //beta1^2*(vaiz - vajz)^2 + 2*beta1*(vaiz - vajz)*(beta2*(vbiz - vbjz) + beta3*(vciz - vcjz)) + (beta2*(vbiz - vbjz) + beta3*(vciz - vcjz))^2 =
        //(cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2,		i,j 1...4

        //beta1^2*(vaix - vajx)^2 + beta1*beta2*2*(vaix - vajx)*(vbix - vbjx) + beta1*beta3*2*(vaix - vajx)*(vcix - vcjx) + beta2^2*(vbix - vbjx)^2 + beta2*beta3*2*(vbix - vbjx)*(vcix - vcjx) + beta3^2*(vcix - vcjx)^2 +
        //beta1^2*(vaiy - vajy)^2 + beta1*beta2*2*(vaiy - vajy)*(vbiy - vbjy) + beta1*beta3*2*(vaiy - vajy)*(vciy - vcjy) + beta2^2*(vbiy - vbjy)^2 + beta2*beta3*2*(vbiy - vbjy)*(vciy - vcjy) + beta3^2*(vciy - vcjy)^2 +
        //beta1^2*(vaiz - vajz)^2 + beta1*beta2*2*(vaiz - vajz)*(vbiz - vbjz) + beta1*beta3*2*(vaiz - vajz)*(vciz - vcjz) + beta2^2*(vbiz - vbjz)^2 + beta2*beta3*2*(vbiz - vbjz)*(vciz - vcjz) + beta3^2*(vciz - vcjz)^2 =
        //(cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2,		i,j 1...4

        //We linearize the equation assuming:
        //alpha1 = beta1^2
        //alpha2 = beta1*beta2
        //alpha3 = beta1*beta3
        //alpha4 = beta2^2
        //alpha5 = beta2*beta3
        //alpha6 = beta3^2

        //alpha1*(vaix - vajx)^2 + alpha2*2*(vaix - vajx)*(vbix - vbjx) + alpha3*2*(vaix - vajx)*(vcix - vcjx) + alpha4*(vbix - vbjx)^2 + alpha5*2*(vbix - vbjx)*(vcix - vcjx) + alpha6*(vcix - vcjx)^2 +
        //alpha1*(vaiy - vajy)^2 + alpha2*2*(vaiy - vajy)*(vbiy - vbjy) + alpha3*2*(vaiy - vajy)*(vciy - vcjy) + alpha4*(vbiy - vbjy)^2 + alpha5*2*(vbiy - vbjy)*(vciy - vcjy) + alpha6*(vciy - vcjy)^2 +
        //alpha1*(vaiz - vajz)^2 + alpha2*2*(vaiz - vajz)*(vbiz - vbjz) + alpha3*2*(vaiz - vajz)*(vciz - vcjz) + alpha4*(vbiz - vbjz)^2 + alpha5*2*(vbiz - vbjz)*(vciz - vcjz) + alpha6*(vciz - vcjz)^2 =
        //(cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2,		i,j 1...4

        //Reorder
        //alpha1*((vaix - vajx)^2 + (vaiy - vajy)^2 + (vaiz - vajz)^2) +
        //alpha2*2*((vaix - vajx)*(vbix - vbjx) + (vaiy - vajy)*(vbiy - vbjy) + (vaiz - vajz)*(vbiz - vbjz)) +
        //alpha3*2*((vaix - vajx)*(vcix - vcjx) + (vaiy - vajy)*(vciy - vcjy) + (vaiz - vajz)*(vciz - vcjz)) +
        //alpha4*((vbix - vbjx)^2 + (vbiy - vbjy)^2 + (vbiz - vbjz)^2) +
        //alpha5*2*((vbix - vbjx)*(vcix - vcjx) + (vbiy - vbjy)*(vciy - vcjy) + (vbiz - vbjz)*(vciz - vcjz)) +
        //alpha6*((vcix - vcjx)^2 + (vciy - vcjy)^2 + (vciz - vcjz)^2) =
        //(cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2,		i,j 1...4

        //this also produces 6 equations as in case 2.
        
        double[] va = mNullspace.get(0);
        double[] vb = mNullspace.get(1);
        double[] vc = mNullspace.get(2);
        
        List<Point3D> controlCameraPointsA = controlPointsFromV(va);
        List<Point3D> controlCameraPointsB = controlPointsFromV(vb);
        List<Point3D> controlCameraPointsC = controlPointsFromV(vc);
        
        Matrix c = constraintMatrixSolution3(controlCameraPointsA, 
                controlCameraPointsB, controlCameraPointsC);
        double[] rhos = rhos(mControlWorldPoints);
        
        double[] a = Utils.solve(c, rhos);
        
        double beta1;
        double beta2;
        double beta3;
        if (a[0] < 0.0) {
            beta1 = Math.sqrt(-a[0]);
            beta2 = a[3] < 0.0 ? Math.sqrt(-a[3]) : 0.0;
        } else {
            beta1 = Math.sqrt(a[0]);
            beta2 = a[3] > 0.0 ? Math.sqrt(a[3]) : 0.0;
        }
        
        //fix sign of betas
        if (a[1] < 0.0) {
            beta1 = -beta1;
        }
        
        beta3 = a[2] / beta1;     
        
        //We linearize the equation assuming:
        //alpha1 = beta1^2
        //alpha2 = beta1*beta2
        //alpha3 = beta1*beta3
        //alpha4 = beta2^2
        //alpha5 = beta*beta3
        //alpha6 = beta3^2
        
        double initialBeta1 = beta1;
        double initialBeta2 = beta2;
        double initialBeta3 = beta3;
        
        //compute linear combination of va and vb as 
        //v = beta1*va + beta2*vb + beta3*vc
        double[] tmp1 = ArrayUtils.multiplyByScalarAndReturnNew(va, beta1);
        double[] tmp2 = ArrayUtils.multiplyByScalarAndReturnNew(vb, beta2);
        double[] tmp3 = ArrayUtils.multiplyByScalarAndReturnNew(vc, beta3);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        ArrayUtils.sum(tmp1, tmp3, tmp1);
        double[] v = tmp1;
        List<Point3D> controlCameraPoints = controlPointsFromV(v);
        
        Solution solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);    
        
        //because solutions are square roots, beta1, beta2 and beta3 can have 
        //different signs, so we add solutions for each combination so that the 
        //one with the smallest reprojection error will be picked
        beta1 = -initialBeta1;
        //no need to set: beta2 = initialBeta2 and beta3 = initialBeta3 because they already have
        //those values
        
        ArrayUtils.multiplyByScalar(va, beta1, tmp1);
        ArrayUtils.multiplyByScalar(vb, beta2, tmp2);
        ArrayUtils.multiplyByScalar(vc, beta3, tmp3);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        ArrayUtils.sum(tmp1, tmp3, tmp1);
        //no need to set v = tmp1, because it already has this value
        controlCameraPoints = controlPointsFromV(v);
        
        solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);    


        beta1 = initialBeta1;
        beta2 = -initialBeta2;
        //no need to set: beta3 = initialBeta3 as it already has that value
        
        ArrayUtils.multiplyByScalar(va, beta1, tmp1);
        ArrayUtils.multiplyByScalar(vb, beta2, tmp2);
        ArrayUtils.multiplyByScalar(vc, beta3, tmp3);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        ArrayUtils.sum(tmp1, tmp3, tmp1);
        //no need to set v = tmp1, because it already has this value
        controlCameraPoints = controlPointsFromV(v);
        
        solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);    


        beta1 = -initialBeta1;
        beta2 = -initialBeta2;
        //no need to set beta3 = initialBeta3, as it already has that value
        
        ArrayUtils.multiplyByScalar(va, beta1, tmp1);
        ArrayUtils.multiplyByScalar(vb, beta2, tmp2);
        ArrayUtils.multiplyByScalar(vc, beta3, tmp3);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        ArrayUtils.sum(tmp1, tmp3, tmp1);
        //no need to set v = tmp1, because it already has this value
        controlCameraPoints = controlPointsFromV(v);
        
        solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);    
        
        
        beta1 = initialBeta1;
        beta2 = initialBeta2;
        beta3 = -initialBeta3;
        
        ArrayUtils.multiplyByScalar(va, beta1, tmp1);
        ArrayUtils.multiplyByScalar(vb, beta2, tmp2);
        ArrayUtils.multiplyByScalar(vc, beta3, tmp3);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        ArrayUtils.sum(tmp1, tmp3, tmp1);
        //no need to set v = tmp1, because it already has this value
        controlCameraPoints = controlPointsFromV(v);
        
        solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);    
        
        
        beta1 = -initialBeta1;
        //no need to set beta2 = initialBeta2, because it already has this value
        beta3 = -initialBeta3;
        
        ArrayUtils.multiplyByScalar(va, beta1, tmp1);
        ArrayUtils.multiplyByScalar(vb, beta2, tmp2);
        ArrayUtils.multiplyByScalar(vc, beta3, tmp3);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        ArrayUtils.sum(tmp1, tmp3, tmp1);
        //no need to set v = tmp1, because it already has this value
        controlCameraPoints = controlPointsFromV(v);
        
        solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);      
        
        
        beta1 = initialBeta1;
        beta2 = -initialBeta2;
        beta3 = -initialBeta3;
        
        ArrayUtils.multiplyByScalar(va, beta1, tmp1);
        ArrayUtils.multiplyByScalar(vb, beta2, tmp2);
        ArrayUtils.multiplyByScalar(vc, beta3, tmp3);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        ArrayUtils.sum(tmp1, tmp3, tmp1);
        //no need to set v = tmp1, because it already has this value
        controlCameraPoints = controlPointsFromV(v);
        
        solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);
        
        
        beta1 = -initialBeta1;
        beta2 = -initialBeta2;
        beta3 = -initialBeta3;
        
        ArrayUtils.multiplyByScalar(va, beta1, tmp1);
        ArrayUtils.multiplyByScalar(vb, beta2, tmp2);
        ArrayUtils.multiplyByScalar(vc, beta3, tmp3);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        ArrayUtils.sum(tmp1, tmp3, tmp1);
        //no need to set v = tmp1, because it already has this value
        controlCameraPoints = controlPointsFromV(v);
        
        solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);        
    }

    /**
     * Tests solution 2 for general point configuration.
     * Because solution is up to scale, 4 different solutions for different 
     * beta1 and beta2 signs are tried.
     * @throws AlgebraException if a numerical degeneracy occurs.
     * @throws LockedException never happens.
     * @throws NotReadyException never happens.
     * @throws CoincidentPointsException if a point degeneracy has occurred.
     */
    private void generalSolution2() throws AlgebraException, 
            LockedException, NotReadyException, CoincidentPointsException {
        //we have the distance constraints between control world points (c) and
        //control camera points (v):
        //||beta*vi - beta*vj||^2 = ||ci - cj||^2, i,j 1...4
        //we need to find beta to scale control camera points
        //in the case we pick 2 columns of the nullspace v, then v is a linear
        //combination v = beta1*vA + beta2*vB and previous equation becomes
        //||(beta1*vAi + beta2*vBi) - (beta1*vAj + beta2*vBj)||^2 = ||ci - cj||^2, i,j 1...4
        //This equation can be expanded in x,y,z coordinates as follows:
        //((beta1*vAix + beta2*vBix) - (beta1*vAjx + beta2*vBjx))^2 + ((beta1*vAiy + beta2*vBiy) - (beta1*vAjy + beta2*vBjy))^2 + ((beta1*vAiz + beta2*vBiz) - (beta1*vAjz + beta2*vBjz))^2 = ((cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2),		i,j 1...4
        //(beta1*(vAix - vAjx) + beta2*(vBix - vBjx))^2 + (beta1*(vAiy - vAjy) + beta2*(vBiy - vBjy))^2 + (beta1*(vAiz - vAjz) + beta2*(vBiz - vBjz))^2 = ((cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2),		i,j 1...4
        //beta1^2*(vAix - vAjx)^2 + beta1*beta2*2*(vAix - vAjx)*(vBix - vBjx) + beta2^2*(vBix - vBjx)^2 + beta1^2*(vAiy - vAjy)^2 + beta1*beta2*2*(vAiy - vAjy)*(vBiy - vBjy) + beta2^2*(vBiy - vBjy)^2 + beta1^2*(vAiz - vAjz)^2 + beta1*beta2*2*(vAiz - vAjz)*(vBiz - vBjz) + beta2^2*(vBiz - vBjz)^2 = ((cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2),		i,j 1...4

        //Since beta1 and beta2 are the unknowns, we can reorganize equation as:
        //beta1^2*((vAix - vAjx)^2 + (vAiy - vAjy)^2 + (vAiz - vAjz)^2) + 
        //beta1*beta2*2*((vAix - vAjx)*(vBix - vBjx) + (vAiy - vAjy)*(vBiy - vBjy) + (vAiz - vAjz)*(vBiz - vBjz)) + 
        //beta2^2*((vBix - vBjx)^2 + (vBiy - vBjy)^2 + (vBiz - vBjz)^2) = 
        //((cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2),		i,j 1...4

        //We linearize the equation assuming:
        //alpha1 = beta1^2
        //alpha2 = beta1*beta2
        //alpha3 = beta2^2

        //alpha1*((vAix - vAjx)^2 + (vAiy - vAjy)^2 + (vAiz - vAjz)^2) + alpha2*2*((vAix - vAjx)*(vBix - vBjx) + (vAiy - vAjy)*(vBiy - vBjy) + (vAiz - vAjz)*(vBiz - vBjz)) + alpha3*((vBix - vBjx)^2 + (vBiy - vBjy)^2 + (vBiz - vBjz)^2) = ((cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2),		i,j 1...4
        
        //finally we evaluate the equation for all 6 possible combinations of 
        //i,j 1...4 when we have 4 control points
        //Obtaining the following equations:
        //alpha1*((vA1x - vA2x)^2 + (vA1y - vA2y)^2 + (vA1z - vA2z)^2) + alpha2*2*((vA1x - vA2x)*(vB1x - vB2x) + (vA1y - vA2y)*(vB1y - vB2y) + (vA1z - vA2z)*(vB1z - vB2z)) + alpha3*((vB1x - vB2x)^2 + (vB1y - vB2y)^2 + (vB1z - vB2z)^2) = ((cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2)
        //alpha1*((vA1x - vA3x)^2 + (vA1y - vA3y)^2 + (vA1z - vA3z)^2) + alpha2*2*((vA1x - vA3x)*(vB1x - vB3x) + (vA1y - vA3y)*(vB1y - vB3y) + (vA1z - vA3z)*(vB1z - vB3z)) + alpha3*((vB1x - vB3x)^2 + (vB1y - vB3y)^2 + (vB1z - vB3z)^2) = ((cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2)
        //alpha1*((vA1x - vA4x)^2 + (vA1y - vA4y)^2 + (vA1z - vA4z)^2) + alpha2*2*((vA1x - vA4x)*(vB1x - vB4x) + (vA1y - vA4y)*(vB1y - vB4y) + (vA1z - vA4z)*(vB1z - vB4z)) + alpha3*((vB1x - vB4x)^2 + (vB1y - vB4y)^2 + (vB1z - vB4z)^2) = ((cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2)
        //alpha1*((vA2x - vA3x)^2 + (vA2y - vA3y)^2 + (vA2z - vA3z)^2) + alpha2*2*((vA2x - vA3x)*(vB2x - vB3x) + (vA2y - vA3y)*(vB2y - vB3y) + (vA2z - vA3z)*(vB2z - vB3z)) + alpha3*((vB2x - vB3x)^2 + (vB2y - vB3y)^2 + (vB2z - vB3z)^2) = ((cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2)
        //alpha1*((vA2x - vA4x)^2 + (vA2y - vA4y)^2 + (vA2z - vA4z)^2) + alpha2*2*((vA2x - vA4x)*(vB2x - vB4x) + (vA2y - vA4y)*(vB2y - vB4y) + (vA2z - vA4z)*(vB2z - vB4z)) + alpha3*((vB2x - vB4x)^2 + (vB2y - vB4y)^2 + (vB2z - vB4z)^2) = ((cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2)
        //alpha1*((vA3x - vA4x)^2 + (vA3y - vA4y)^2 + (vA3z - vA4z)^2) + alpha2*2*((vA3x - vA4x)*(vB3x - vB4x) + (vA3y - vA4y)*(vB3y - vB4y) + (vA3z - vA4z)*(vB3z - vB4z)) + alpha3*((vB3x - vB4x)^2 + (vB3y - vB4y)^2 + (vB3z - vB4z)^2) = ((cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2)        
        
        //where alpha1, alpha2 and alpha3 are the unknowns of linear system of 
        //equations whose matrix C has size 6,3 (as seen below), and the right
        //terms of the equation can be built by calling method 
        //#rhos(List<Point3D>) in this class
        
        double[] va = mNullspace.get(0);
        double[] vb = mNullspace.get(1);
        
        List<Point3D> controlCameraPointsA = controlPointsFromV(va);
        List<Point3D> controlCameraPointsB = controlPointsFromV(vb);
        
        Matrix c = constraintMatrixSolution2(controlCameraPointsA, 
                controlCameraPointsB);
        double[] rhos = rhos(mControlWorldPoints);
        
        double[] a = Utils.solve(c, rhos);
        
        //obtained a values are related to betas with the following expressions
        //due to linearization:
        //alpha1 = beta1^2
        //alpha2 = beta1*beta2
        //alpha3 = beta2^2
        
        double beta1;
        double beta2;
        if (a[0] < 0.0) {
            beta1 = Math.sqrt(-a[0]);
            beta2 = a[2] < 0.0 ? Math.sqrt(-a[2]) : 0.0;
        } else {
            beta1 = Math.sqrt(a[0]);
            beta2 = a[2] > 0.0 ? Math.sqrt(a[2]) : 0.0;
        }
       
        //fix sign of betas
        if (a[1] < 0.0) {
            beta1 = -beta1;
        }
        
        double initialBeta1 = beta1;
        double initialBeta2 = beta2;
         
        
        //compute linear combination of va and vb as v = beta1*va + beta2*vb
        double[] tmp1 = ArrayUtils.multiplyByScalarAndReturnNew(va, beta1);
        double[] tmp2 = ArrayUtils.multiplyByScalarAndReturnNew(vb, beta2);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        double[] v = tmp1;
        List<Point3D> controlCameraPoints = controlPointsFromV(v);
        
        Solution solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution); 
        
        //because solutions are square roots, beta1 and beta2 can have different
        //signs, so we add solutions for each combination so that the one with
        //the smallest reprojection error will be picked
        beta1 = -initialBeta1;
        beta2 = -initialBeta2;
        
        ArrayUtils.multiplyByScalar(va, beta1, tmp1);
        ArrayUtils.multiplyByScalar(vb, beta2, tmp2);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        //no need to set v = tmp1, as it already has this value
        controlCameraPoints = controlPointsFromV(v);
        
        solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);
        
        
        beta1 = initialBeta1;
        beta2 = -initialBeta2;
        
        ArrayUtils.multiplyByScalar(va, beta1, tmp1);
        ArrayUtils.multiplyByScalar(vb, beta2, tmp2);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        //no need to set v = tmp1, as it already has this value
        controlCameraPoints = controlPointsFromV(v);
        
        solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);

        
        beta1 = -initialBeta1;
        beta2 = initialBeta2;
        
        ArrayUtils.multiplyByScalar(va, beta1, tmp1);
        ArrayUtils.multiplyByScalar(vb, beta2, tmp2);
        ArrayUtils.sum(tmp1, tmp2, tmp1);
        //no need to set v = tmp1, as it already has this value
        controlCameraPoints = controlPointsFromV(v);
        
        solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution); 
    }
    
    /**
     * Fills constraint matrix to solve betas when using control points from the
     * last 3 columns of v (the nullspace).
     * The solution will be the linear combination of control points from the
     * last 3 columns using estimated betas. This solution will be control 
     * points in camera coordinates.
     * @param controlCameraPointsA control points of last column of v.
     * @param controlCameraPointsB control points of second last column of v.
     * @param controlCameraPointsC control points of third last column of v.
     * @return constraint matrix to solve a linear system of equations.
     * @throws AlgebraException never happens.
     */
    private static Matrix constraintMatrixSolution3(            
            List<Point3D> controlCameraPointsA,
            List<Point3D> controlCameraPointsB,
            List<Point3D> controlCameraPointsC) throws AlgebraException {
        
        int numControl = controlCameraPointsA.size();        
        int numEquations = numEquations(numControl);                
        
        Matrix c = new Matrix(numEquations, 6);
        int row = 0;
        for (int i = 0; i < numControl; i++) {
            Point3D vai = controlCameraPointsA.get(i);
            Point3D vbi = controlCameraPointsB.get(i);
            Point3D vci = controlCameraPointsC.get(i);
            
            for (int j = i + 1; j < numControl; j++) {
                Point3D vaj = controlCameraPointsA.get(j);
                Point3D vbj = controlCameraPointsB.get(j);
                Point3D vcj = controlCameraPointsC.get(j);
                
                fillRowConstraintMatrixSolution3(row, c, vai, vaj, vbi, vbj,
                        vci, vcj);
                row++;
            }
        }
        
        return c;        
    }
    
    /**
     * Fills constraint matrix to solve betas when using control points from the
     * last 2 columns of v (the nullspace).
     * The solution will be the linear combination of control points from the
     * last 2 columns using estimated betas. This solution will be control
     * points in camera coordinates.
     * @param controlCameraPointsA control points of last column of v.
     * @param controlCameraPointsB control points of second last column of v.
     * @return constraint matrix to solve a linear system of equations.
     * @throws AlgebraException never happens.
     */
    private static Matrix constraintMatrixSolution2(
            List<Point3D> controlCameraPointsA,
            List<Point3D> controlCameraPointsB) throws AlgebraException {
        
        int numControl = controlCameraPointsA.size();        
        int numEquations = numEquations(numControl);                
        
        Matrix c = new Matrix(numEquations, 3);
        int row = 0;
        for (int i = 0; i < numControl; i++) {
            Point3D vai = controlCameraPointsA.get(i);
            Point3D vbi = controlCameraPointsB.get(i);
            
            for (int j = i + 1; j < numControl; j++) {
                Point3D vaj = controlCameraPointsA.get(j);
                Point3D vbj = controlCameraPointsB.get(j);
                
                fillRowConstraintMatrixSolution2(row, c, vai, vaj, vbi, vbj);
                row++;
            }
        }
        
        return c;
    }
    
    /**
     * Fills a row of constraint matrix for solution 3.
     * @param row row to be filled.
     * @param c matrix to be filled.
     * @param vai i-th control point in camera coordinates of last column of v 
     * (i.e. the nullspace).
     * @param vaj j-th control point in camera coordinates of last column of v
     * (i.e. the nullspace).
     * @param vbi i-th control point in camera coordinates of second last column
     * of v (i.e. the nullspace).
     * @param vbj j-th control point in camera coordinates of second last column
     * of v (i.e. the nullspace).
     * @param vci i-th control point in camera coordinates of third last column
     * of v (i.e. the nullspace).
     * @param vcj j-th control point in camera coordinates of third last column
     * of v (i.e. the nullspace).
     */
    private static void fillRowConstraintMatrixSolution3(int row, Matrix c,
            Point3D vai, Point3D vaj, Point3D vbi, Point3D vbj, Point3D vci,
            Point3D vcj) {
        
        double vaix = vai.getInhomX();
        double vaiy = vai.getInhomY();
        double vaiz = vai.getInhomZ();
        
        double vajx = vaj.getInhomX();
        double vajy = vaj.getInhomY();
        double vajz = vaj.getInhomZ();
        
        double vbix = vbi.getInhomX();
        double vbiy = vbi.getInhomY();
        double vbiz = vbi.getInhomZ();
        
        double vbjx = vbj.getInhomX();
        double vbjy = vbj.getInhomY();
        double vbjz = vbj.getInhomZ();
        
        double vcix = vci.getInhomX();
        double vciy = vci.getInhomY();
        double vciz = vci.getInhomZ();
        
        double vcjx = vcj.getInhomX();
        double vcjy = vcj.getInhomY();
        double vcjz = vcj.getInhomZ();
        
        //1st column
        c.setElementAt(row, 0, Math.pow(vaix - vajx, 2.0) + 
                Math.pow(vaiy - vajy, 2.0) + Math.pow(vaiz - vajz, 2.0));
        
        //2nd column
        c.setElementAt(row, 1, 2.0 * ((vaix - vajx) * (vbix - vbjx) + 
                (vaiy - vajy) * (vbiy - vbjy) + (vaiz - vajz) * (vbiz - vbjz)));
        
        //3rd column
        c.setElementAt(row, 2, 2.0 * ((vaix - vajx) * (vcix - vcjx) +
                (vaiy - vajy) * (vciy - vcjy) + (vaiz - vajz) * (vciz - vcjz)));
        
        //4th column
        c.setElementAt(row, 3, Math.pow(vbix - vbjx, 2.0) + 
                Math.pow(vbiy - vbjy, 2.0) + Math.pow(vbiz - vbjz, 2.0));
        
        //5th column
        c.setElementAt(row, 4, 2.0 * ((vbix - vbjx) * (vcix - vcjx) + 
                (vbiy - vbjy) * (vciy - vcjy) + (vbiz - vbjz) * (vciz - vcjz)));
        
        //6th column
        c.setElementAt(row, 5, Math.pow(vcix - vcjx, 2.0) + 
                Math.pow(vciy - vcjy, 2.0) + Math.pow(vciz - vcjz, 2.0));
    }
    
    /**
     * Fills a row of constraint matrix for solution 2.
     * @param row row to be filled.
     * @param c matrix to be filled.
     * @param vai i-th control point in camera coordinates of last column of v 
     * (i.e. the nullspace).
     * @param vaj j-th control point in camera coordinates of last column of v 
     * (i.e. the nullspace).
     * @param vbi i-th control point in camera coordinates of second last column 
     * of v (i.e. the nullspace).
     * @param vbj j-th control point in camera coordinates of second last column
     * of v (i.e. the nullspace).
     */
    private static void fillRowConstraintMatrixSolution2(int row, Matrix c, 
            Point3D vai, Point3D vaj, Point3D vbi, Point3D vbj) {
        
        double vaix = vai.getInhomX();
        double vaiy = vai.getInhomY();
        double vaiz = vai.getInhomZ();
        
        double vajx = vaj.getInhomX();
        double vajy = vaj.getInhomY();
        double vajz = vaj.getInhomZ();
        
        double vbix = vbi.getInhomX();
        double vbiy = vbi.getInhomY();
        double vbiz = vbi.getInhomZ();
        
        double vbjx = vbj.getInhomX();
        double vbjy = vbj.getInhomY();
        double vbjz = vbj.getInhomZ();
        
        //1st column
        c.setElementAt(row, 0, Math.pow(vaix - vajx, 2.0) + 
                Math.pow(vaiy - vajy, 2.0) + Math.pow(vaiz - vajz, 2.0));
        
        //2nd column
        c.setElementAt(row, 1, 2.0 * ((vaix - vajx) * (vbix - vbjx) + 
                (vaiy - vajy) * (vbiy - vbjy) + (vaiz - vajz) * (vbiz - vbjz)));
        
        //3rd column
        c.setElementAt(row, 2, Math.pow(vbix - vbjx, 2.0) + 
                Math.pow(vbiy - vbjy, 2.0) + Math.pow(vbiz - vbjz, 2.0));        
    }
                
    /**
     * Tests solution 1 for general point configuration.
     * Because solution is up to scale. Two possible solutions must be evaluated
     * (positive or negative scale). The one with the smallest reprojection
     * error will be picked.
     * @throws LockedException never happens.
     * @throws NotReadyException never happens.
     * @throws CoincidentPointsException if a point degeneracy has occurred.
     */
    private void generalSolution1() throws LockedException, 
            NotReadyException, CoincidentPointsException {
        //pick last column of nullspace, contains control points in camera
        //coordinates up to scale (including sign change)        
        double[] v = mNullspace.get(0);    
        List<Point3D> controlCameraPoints = controlPointsFromV(v);
                
        //similarly to solution2 and solution3, we could find the scale by
        //imposing the restriction: ||beta*vi - beta*vj||^2 = ||ci - cj||^2, i,j 1...4
        //This results in a linear system of 6 equations (when we have 4 control
        //points)
        //The previous constraint can be expanded as follows:
        //(beta*vi - beta*vj)^2 = (ci - cj)^2
        //(beta*vix - beta*vjx)^2 + (beta*viy - beta*vjy)^2 + (beta*viz - beta*vjz)^2 = (cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2,		i,j 1...4
        //beta^2*(vix - vjx)^2 + beta^2*(viy - vjy)^2 + beta^2*(viz - vjz)^2 = (cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2,		i,j 1...4
        //beta^2*((vix - vjx)^2 + (viy - vjy)^2 + (viz - vjz)^2) = (cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2,		i,j 1...4
        //
        //And the system is linearized by assuming
        //alpha = beta^2
        //
        //alpha * ((vix - vjx)^2 + (viy - vjy)^2 + (viz - vjz)^2) = (cix - cjx)^2 + (ciy - cjy)^2 + (ciz - cjz)^2,		i,j 1...4
        
        //Hoewever, in order to find a solution a MetricTransformation3D estimator
        //is used, which is capable to determine the scale relating input and
        //output points, and thus, solving the linear system of equations is not
        //required in this case.
        Solution solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);
        
        //because v is a solution up to scale, we provide a solution with 
        //opposite sign
        v = ArrayUtils.multiplyByScalarAndReturnNew(v, -1.0);
        controlCameraPoints = controlPointsFromV(v);
        
        solution = computePossibleSolutionWithPoseAndReprojectionError(
                controlCameraPoints);        
        mSolutions.add(solution);        
    }
        
    /**
     * Computes a possible solution with camera, transformation, reprojection 
     * error and control points in camera coordinates.
     * @param controlCameraPoints control points in camera coordinates.
     * @return a possible solution.
     * @throws LockedException never happens.
     * @throws NotReadyException never happens.
     * @throws CoincidentPointsException if a point degeneracy has occurred.
     */
    private Solution computePossibleSolutionWithPoseAndReprojectionError(
            List<Point3D> controlCameraPoints) throws LockedException, 
            NotReadyException, CoincidentPointsException {
                
        MetricTransformation3D worldToCameraTransformation = 
                    worldToCameraTransformationMetric(controlCameraPoints);
        
        Rotation3D rotation = worldToCameraTransformation.getRotation();
        double[] t = worldToCameraTransformation.getTranslation();
        double scale = worldToCameraTransformation.getScale();  
        
        //Camera center is C = -1/s*R'*t
        InhomogeneousPoint3D center = new InhomogeneousPoint3D(
                -t[0] / scale, -t[1] / scale, -t[2] / scale);
        Rotation3D invRotation = rotation.inverseRotationAndReturnNew();
        invRotation.rotate(center, center);
        
        PinholeCamera camera = new PinholeCamera(mIntrinsic, rotation, center);
        
        Solution solution = new Solution();
        solution.controlCameraPoints = controlCameraPoints;
        solution.worldToCameraTransformation = worldToCameraTransformation;
        solution.camera = camera;
        
        //compute projection error
        solution.reprojectionError = reprojectionError(camera);

        return solution;
    }        
        
    /**
     * Estimates world to camera transformation using estimated control points
     * in world and camera coordinates as a metric transformation.
     * @param controlCameraPoints control points in camera coordinates.
     * @return metric transformation relating control points from world to 
     * camera coordinates.
     * @throws LockedException never happens.
     * @throws NotReadyException never happens.
     * @throws CoincidentPointsException if a point degeneracy has occurred.
     */
    private MetricTransformation3D worldToCameraTransformationMetric(
            List<Point3D> controlCameraPoints) throws LockedException, 
            NotReadyException, CoincidentPointsException {
        MetricTransformation3DEstimator estimator = 
                new MetricTransformation3DEstimator(mControlWorldPoints, 
                        controlCameraPoints, mIsPlanar);                
        return estimator.estimate();        
    }
    
    /**
     * Number of equations required to solve constraints for case 1 to 4.
     * @param numControl number of control points.
     * @return number of constraint equations.
     */
    private static int numEquations(int numControl) {
        int numEquations = 0;
        for (int i = 1; i < numControl; i++) {
            numEquations += i;
        }
        return numEquations;        
    }
    
    /**
     * Right term of linearized system of equations to solve betas.
     * @param controlWorldPoints control points in world coordinates.
     * @return right term.
     */
    private static double[] rhos(List<Point3D> controlWorldPoints) {
        int numControl = controlWorldPoints.size();        
        int numEquations = numEquations(numControl);        
        double[] rhos = new double[numEquations];
        
        //squared distance from control world i to control world j
        double dcijSqr;
        int pos = 0;
        for (int i = 0; i < numControl; i++) {
            Point3D ci = controlWorldPoints.get(i);
            
            for(int j = i + 1; j < numControl; j++) {
                Point3D cj = controlWorldPoints.get(j);
                
                dcijSqr = Math.pow(ci.distanceTo(cj), 2.0);
                rhos[pos] = dcijSqr;
                pos++;
            }
        }
        
        return rhos;
    }
        
    /**
     * Total reprojection error for provided camera.
     * @param camera camera to estimate reprojection error.
     * @return reprojection error.
     */
    private double reprojectionError(PinholeCamera camera) {
        int n = mPoints2D.size();
        
        Point3D point3D;
        Point2D projected = Point2D.create();
        Point2D point2D;
        double error = 0.0;
        for (int i = 0; i < n; i++) {
            point3D = mPoints3D.get(i);
            point2D = mPoints2D.get(i);
            camera.project(point3D, projected);
            error += projected.distanceTo(point2D);
        }
        return error;
    }
    
    /**
     * Computes list of control points from provided array containing one column
     * of the nullspace of M or a linear combination of columns of the 
     * nullspace.
     * @param v one column of the nullspace of M or a linear combination of
     * columns of the nullspace.
     * @return control points.
     */
    private List<Point3D> controlPointsFromV(double[] v) {
        int numControl = mControlWorldPoints.size();
        List<Point3D> points = new ArrayList<>();
        
        InhomogeneousPoint3D p;
        for (int j = 0; j < numControl; j++) {
            int k = j * 3;
            p = new InhomogeneousPoint3D(v[k], v[k + 1], v[k + 2]);
            points.add(p);
        }
        
        return points;
    }
    
    /**
     * Solves nullspace of matrix M containing possible solutions of camera 
     * coordinates of control points.
     * @throws AlgebraException if something fails due to numerica 
     * unstabilities.
     */
    private void solveNullspace() throws AlgebraException {        
        int rows = mM.getRows();
        int cols = mM.getColumns();
        int numControl = cols / 
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
        
        //normalize rows of m to increase numerical accuracy
        for (int i = 0; i < rows; i++) {
            normalizeRow(mM, i);
        }
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(mM);
        decomposer.decompose();
        
        //Singular values are always in descending order, hence null space is in 
        //the last columns of v.
        //V is 12x12 (general configuration) or 9x9 (planar configuration).
        //Each column of v contains coordinates of control points in camera 
        //coordinates.
        //A solution for the linear system M*x = 0 is obtained as a linear
        //combination of the columns of v forming the nullspace.
        Matrix v = decomposer.getV();               
        
        //alghouth nullity of M could be determined after SVD, it is assumed
        //instead that nullspace could be located in any of the latter columns 
        //of v up to the number of control points.
        //Hence, for general configuration we pick the last 4 columns of v and
        //for planar configuration we pick the last 3.
     
        //extract null points from the null space
        mNullspace = new ArrayList<>();
        int colsMinusOne = cols - 1;
        double[] vCol;
        for (int i = 0; i < numControl; i++) {
            int column = colsMinusOne - i;

            //each picked column of v contains a possible solution
            vCol = v.getSubmatrixAsArray(0, column, colsMinusOne, column);
            mNullspace.add(vCol);
        }
    }
    
    /**
     * Normalizes provided row of m.
     * @param m matrix to be normalized.
     * @param row row to be normalized.
     */
    private static void normalizeRow(Matrix m, int row) {
        int cols = m.getColumns();
        
        double norm = 0.0;
        for (int i = 0; i < cols; i++) {
            norm += Math.pow(m.getElementAt(row, i), 2.0);
        }
        norm = Math.sqrt(norm);
        
        for (int i = 0; i < cols; i++) {
            m.setElementAt(row, i, m.getElementAt(row, i) / norm);
        }
    }
        
    /**
     * In order to find control points in camera coordinates, an homogeneous 
     * linear system of equations must be solved having the form M*x = 0, where
     * x contains the coordinates of all control points in the form [x1, y1, z1,
     * x2, y2, z2, ... ].
     * For general configuration there are 4 control points, hence x has length
     * 12 (3 coordinates * 4 control points).
     * For a planar configuration there are 3 control points, hence x has length
     * 9 (3 coordinates * 3 control points).
     * This method builds M matrix required to solve such linear system of 
     * equations, where M has size 2*n x 12 (general configuration) or 2*n x 9
     * (planar configuration), where n is the number of provided 2D observed 
     * points.
     * @throws AlgebraException if numerical unstabilities occur.
     */
    private void buildM() throws AlgebraException {
        int n = mPoints2D.size();
        int numControlPoints = mAlphas.getColumns();
        
        mM = new Matrix(2 * n, 3 * numControlPoints);
        
        int row;
        int col;
        double alpha;
        
        double horizontalFocalLength = mIntrinsic.getHorizontalFocalLength();
        double verticalFocalLength = mIntrinsic.getVerticalFocalLength();
        double skewness = mIntrinsic.getSkewness();
        double horizontalPrincipalPoint = 
                mIntrinsic.getHorizontalPrincipalPoint();
        double verticalPrincipalPoint = mIntrinsic.getVerticalPrincipalPoint();
        
        Point2D p;
        double pX;
        double pY;
        for (int i = 0; i < n; i++) {
            p = mPoints2D.get(i);
            pX = p.getInhomX();
            pY = p.getInhomY();
            
            row = i * 2;
            
            for (int j = 0; j < numControlPoints; j++) {
                col = j * 3;
                
                alpha = mAlphas.getElementAt(i, j);
                
                mM.setElementAt(row, col, alpha * horizontalFocalLength);
                mM.setElementAt(row, col + 1, alpha * skewness);
                mM.setElementAt(row, col + 2, 
                        alpha * (horizontalPrincipalPoint - pX));
                
                mM.setElementAt(row + 1, col, 0.0);
                mM.setElementAt(row + 1, col + 1, alpha * verticalFocalLength);
                mM.setElementAt(row + 1, col + 2, 
                        alpha * (verticalPrincipalPoint - pY));                
            }
        }                
    }
            
    /**
     * Computes the coordinates of each provided world point in terms of 
     * estimated control points in world coordinates.
     * Such coordinates (i.e. barycentric coordinates) are stored in alphas
     * matrix, where each row contains the coordinates of each world point in
     * terms of control points.
     * For general configuration, each row contains 4 coordinates and alphas
     * has size nx4, where n is the number of provided 3D world points.
     * For planar configuration, each row contains 3 coordinates and alphas
     * has size nx3, where n is the number of provided 3D world points.
     * Because world and camera coordinates are related by a rotation (since
     * both reference frames are centered in the centroid), alphas can be used
     * in both world and camera coordinates.
     * @throws AlgebraException if there are numerical unstabilities.
     */
    private void computeBarycentricCoordinates() throws AlgebraException {
        //we need to express world points in terms of control points in world
        //coordinates
        
        //In the general configuration case:
        //For a point p1 in world inhomogeneous coordinates
        //p1 =  alpha1 + c1 + alpha2 * c2 + alpha3 * c3 + alpha4 * c4
        //where alpha1, alpha2, alpha3, alpha4 are scalars and
        //c1, c2, c3 are the control points in the principal axes and
        //centroid is the last control point c4, all 4 expressed in world
        //inhomogeneous coordinates as 3-column vectors.
        
        //Assuming a matrix form:
        //[p1] = [c1 c2 c3 c4]*[alpha1] 
        //                     [alpha2]
        //                     [alpha3]
        //                     [alpha4]

        //or in simpler for p = C * alpha, where p is a 3-column vector, C is a 
        //3x4 matrix and alpha is a 4-1 vector.
        //This can be repeated for each i-th point so that:
        //pi = C * alphai --> alphai = inv(C)*pi
        //However, in this form C is not invertible because it is rank deficient
        //To avoid this deficiency we add the constraint that the sum of alphas
        //for a point must be 1, so we can use the reduced form:
        //[p1 - c4] = [(c1 - c4) (c2 - c4) (c3 - c4)]*[alpha1]
        //                                            [alpha2]
        //                                            [alpha3]
        //and set alpha4 = 1 - alpha1 - alpha2 - alpha3
                
        //This way the equation still holds:
        //p1 - c4 = (c1 - c4) * alpha1 + (c2 - c4) * alpha2 + (c3 - c4) * alpha3 =
        //        = c1 * alpha1 + c2 * alpha2 + c3 * alpha3 - c4 * (alpha1 + alpha2 + alpha3)
        //p1 = c1 * alpha1 + c2 * alpha2 * c3 * alpha3 + c4 * (1 - alpha1 - alpha2 - alpha3)

        //This way, we create reduced matrix C as having 3 rows (one for each 
        //inhomogeneous coordinate) and 3 columns in the general case.
        
        //In the planar case we have only 3 control points, and the last one
        //(c3) is the centroid.
        
        int numControl = mControlWorldPoints.size();
        int numDimensions = numControl - 1;
        int numControlMinusTwo = numControl - 2;
        Matrix c = new Matrix(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 
                numDimensions);
        
        //the last control point is the centroid (or mean point)
        Point3D mean = mControlWorldPoints.get(numDimensions);
        double meanX = mean.getInhomX();
        double meanY = mean.getInhomY();
        double meanZ = mean.getInhomZ();
        
        Point3D controlPoint;
        for (int i = 0; i < numDimensions; i++) {
            controlPoint = mControlWorldPoints.get(i);
            c.setElementAt(0, i, controlPoint.getInhomX() - meanX);
            c.setElementAt(1, i, controlPoint.getInhomY() - meanY);
            c.setElementAt(2, i, controlPoint.getInhomZ() - meanZ);            
        }
        
        //to find  reduced alphas, we need to inverse the reduced C matrix and
        //multiply it by [p - centroid], where centroid can be c4 or c3 in 
        //planar case. 
        
        Matrix invC = Utils.inverse(c);
        
        //x is p - centroid, where p is each 3D world point
        int n = mPoints3D.size();
        Matrix reducedPoint = new Matrix(
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,1);
        Matrix reducedAlpha = new Matrix(numDimensions, 1);
        double[] buffer;
        Point3D worldPoint;
        mAlphas = new Matrix(n, numControl);
        for (int i = 0; i < n; i++) {
            worldPoint = mPoints3D.get(i);
            reducedPoint.setElementAtIndex(0, worldPoint.getInhomX() - meanX);
            reducedPoint.setElementAtIndex(1, worldPoint.getInhomY() - meanY);
            reducedPoint.setElementAtIndex(2, worldPoint.getInhomZ() - meanZ);
            
            invC.multiply(reducedPoint, reducedAlpha);
            buffer = reducedAlpha.getBuffer();
            
            //copy reducedAlpha into the former components of i-th row of alphas
            mAlphas.setSubmatrix(i, 0, i, numControlMinusTwo, buffer);
            
            //The last component of each alpha for each point is computed so 
            //that their sum is equal to one            
            if (numControl == GENERAL_NUM_CONTROL_POINTS) {
                //general configuration
                mAlphas.setElementAt(i, numDimensions, 
                        1.0 - buffer[0] - buffer[1] - buffer[2]);
            } else {
                //planar configuration
                mAlphas.setElementAt(i, numDimensions, 
                        1.0 - buffer[0] - buffer[1]);
            }
        }
    }
    
    
    /**
     * Computes control points in world coordinates and determines whether
     * they are located in a planar configuration or not.
     * This method computes the centroid of provided 3D points and their 
     * covariance.
     * Uses PCA by means of SVD decomposition of their covariance matrix in 
     * order to find the principal directions of the cloud formed by the
     * collection of points and sets control points as the computed centroid
     * and points along the principal axes so that they form a basis that
     * can be used to express any 3D points into.
     * If the smallest singular value is close to zero in comparison to the
     * largest one, then it is assumed that 3D points are in a planar 
     * configuration.
     * If a planar configuration is allowed, then only 3 control points are
     * computed along the plane using the centroid and two points on the
     * principal directions of such plane.
     * Otherwise in general configuration, 4 control points are computed as
     * the centroid and 3 points along the principal axes of the cloud of 3D
     * points.
     * @throws AlgebraException if something fails because of numerical 
     * unstabilities.
     */
    private void computeWorldControlPointsAndPointConfiguration() 
            throws AlgebraException {        
        Point3D centroid = Point3D.centroid(mPoints3D);
        
        //covariance matrix elements, summed up here for speed
        double c11 = 0.0;
        double c12 = 0.0;
        double c13 = 0.0;
        double c22 = 0.0;
        double c23 = 0.0;
        double c33 = 0.0;
        double dx;
        double dy;
        double dz;
        int n = mPoints3D.size();
        for (Point3D point : mPoints3D) {
            dx = point.getInhomX() - centroid.getInhomX();
            dy = point.getInhomY() - centroid.getInhomY();
            dz = point.getInhomZ() - centroid.getInhomZ();
            
            c11 += dx * dx;
            c12 += dx * dy;
            c13 += dx * dz;
            
            c22 += dy * dy;
            c23 += dy * dz;
            
            c33 += dz * dz;
        }
        c11 /= n;
        c12 /= n;
        c13 /= n;
        c22 /= n;
        c23 /= n;
        c33 /= n;
        
        Matrix covar = new Matrix(3, 3);
        covar.setElementAt(0, 0, c11);
        covar.setElementAt(1, 0, c12);
        covar.setElementAt(2, 0, c13);
        
        covar.setElementAt(0, 1, c12);
        covar.setElementAt(1, 1, c22);
        covar.setElementAt(2, 1, c23);
        
        covar.setElementAt(0, 2, c13);
        covar.setElementAt(1, 2, c23);
        covar.setElementAt(2, 2, c33);
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(covar);
        decomposer.decompose();
        
        double[] singularValues = decomposer.getSingularValues();
        Matrix v = decomposer.getV();

        //planar check
        int numControl;
        if (!mPlanarConfigurationAllowed ||
                Math.abs(singularValues[0]) < 
                Math.abs(singularValues[2]) * mPlanarThreshold) {
            //general onfiguration
            numControl = GENERAL_NUM_CONTROL_POINTS;
            mIsPlanar = false;
        } else {
            //planar configuration (only if allowed)
            numControl = PLANAR_NUM_CONTROL_POINTS;
            mIsPlanar = true;
        }
        
        mControlWorldPoints = new ArrayList<>();
                
        double centroidX = centroid.getInhomX();
        double centroidY = centroid.getInhomY();
        double centroidZ = centroid.getInhomZ();
        
        int numDimensions = numControl - 1;
        double k = Math.sqrt(singularValues[0] / n);
        double vx;
        double vy;
        double vz;
        for (int i = 0; i < numDimensions; i++) {
            vx = v.getElementAt(0, i) * k;
            vy = v.getElementAt(1, i) * k;
            vz = v.getElementAt(2, i) * k;
            
            mControlWorldPoints.add(new InhomogeneousPoint3D(centroidX + vx, 
                    centroidY + vy, centroidZ + vz));
        }
                
        //add centroid (it will be used for the metric transformation 
        //estimation)
        mControlWorldPoints.add(centroid);          
    }    
    
    /**
     * A possible solution.
     */
    private static class Solution {
        /**
         * Control points in camera coordinates.
         */
        List<Point3D> controlCameraPoints;
        
        /**
         * Transformation from world to camera coordinates.
         * Point projection is expressed by x = P * Xw, where P is a pinhole
         * camera and Xw is a point in world coordinates.
         * Points in camera coordinates are expressed as:
         * Xc = Tw-&lt;c * Xw, where Tw-&lt;c is the transformation from world to
         * camera.
         * The euclidean transformation Tw-&lt;c is expressed as:
         * Tw-&lt;c = [R  t]
         *         [0' 1]
         * Projection of a point in camera coordinates can also be expressed
         * as x = Pc * Xc = K * [I 0] * Xc
         * where Pc is a camera and has the form Pc = K *[I 0], so that
         * x = Pc * Xc = K * [I 0] * Xc = K * [I 0] * Tw-&lt;c * Xw
         * x = K * [I 0] * [R  t] * Xw = K * [I*R + 0, I*t + 0] * Xw = 
         *                 [0' 1]
         * x = K * [R t] * Xw = K * [R - R*C] * Xw = x = P * Xw,
         * where R is a rotation and C is the camera center in world 
         * coordinates.
         * Assuminc that control points are obtained up to scale, then instead
         * of an euclidean transformation we will assume that Tw-&lt;c is a metric
         * transformation, hence:
         * Tw-&lt;c = [s*R t2]
         *            [0'  1 ]
         * To obtain the previous equation, then point in camera coordinates
         * must be 1/s*Xc so that:
         * x = Pc * 1/s * Xc = K * [I 0] * 1/s * Xc 
         * x = K * [I 0] * 1/s * Tw-&lt;c * Xw
         * x = K * [I 0] * 1/s *[s*R t2] * Xw = K * 1/s * [I*s*R + 0, I*t2 + 0] 
         *                      [0'  1 ]
         * x = K * 1 / s * [s*R t2] * Xw = K * [R 1/s*t2] * Xw
         * where t = 1/s*t2 = -R*C and so again
         * x = K * [R t] * Xw
         * and camera center is C = -1/s*R'*t2
         */
        MetricTransformation3D worldToCameraTransformation;
        
        /**
         * Pinhole camera using provided intrinsic parameters and estimated
         * transformation for this solution.
         */
        PinholeCamera camera;
        
        /**
         * Reprojection error.
         */
        double reprojectionError;
    }
}
