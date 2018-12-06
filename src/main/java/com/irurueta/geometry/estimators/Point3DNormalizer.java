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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.ProjectiveTransformation3D;

import java.util.List;

/**
 * This class takes a collection of points and computes its average 
 * inhomogeneous coordinates and their scale so that a metric transformation is
 * computed to transform points and normalize them.
 * Normalized points are useful in many algorithms.
 */
public class Point3DNormalizer {
    /**
     * Minimum amount of points required to perform normalization.
     */
    public static final int MIN_POINTS = 2;
    
    /**
     * Collection of points used to compute normalization.
     */
    private List<Point3D> mPoints;
    
    /**
     * Flag indicating that this instance is locked because computation is in 
     * progress.
     */
    private boolean mLocked;
    
    /**
     * Minimum x inhomogeneous coordinate found in provided points.
     */
    private double mMinInhomX;
    
    /**
     * Minimum y inhomogeneous coordinate found in provided points.
     */
    private double mMinInhomY;
    
    /**
     * Minimum z inhomogeneous coordinate found in provided points.
     */
    private double mMinInhomZ;
    
    /**
     * Maximum x inhomogeneous coordinate found in provided points.
     */
    private double mMaxInhomX;
    
    /**
     * Maximum y inhomogeneous coordinate found in provided points.
     */
    private double mMaxInhomY;
    
    /**
     * Maximum z inhomogeneous coordinate found in provided points.
     */
    private double mMaxInhomZ;
    
    /**
     * Computed scale on x coordinates to normalize points.
     */
    private double mScaleX;
    
    /**
     * Computed scale on y coordinates to normalize points.
     */
    private double mScaleY;
    
    /**
     * Computed scale on z coordinates to normalize points.
     */
    private double mScaleZ;
    
    /**
     * Computed x coordinate of centroid of points.
     */
    private double mCentroidX;
    
    /**
     * Computed y coordinate of centroid of points.
     */
    private double mCentroidY;
    
    /**
     * Computed z coordinate of centroid of points.
     */
    private double mCentroidZ;
    
    /**
     * Transformation to normalize points.
     */
    private ProjectiveTransformation3D mTransformation;
    
    /**
     * Transformation to denormalize points, which corresponds to the
     * inverse transformation.
     */
    private ProjectiveTransformation3D mInverseTransformation;
    
    /**
     * Constructor.
     * @param points collection of points to be used to compute normalization.
     * @throws IllegalArgumentException if provided collection of points does
     * not contain enough points, which is MIN_POINTS.
     */
    public Point3DNormalizer(List<Point3D> points) {
        internalSetPoints(points);
        reset();
    }
    
    /**
     * Returns collection of points used to compute normalization.
     * @return collection of points used to compute normalization.
     */
    public List<Point3D> getPoints() {
        return mPoints;
    }
    
    /**
     * Sets collection of points used to compute normalization.
     * @param points collection of points used to compute normalization.
     * @throws LockedException if instance is locked because another computation
     * is already in progress.
     * @throws IllegalArgumentException if provided collection of points does
     * not contain enough points, which is MIN_POINTS.
     */
    public void setPoints(List<Point3D> points) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPoints(points);
        reset();
    }
    
    /**
     * Indicates whether this instance is ready (i.e. has enough data) to
     * start the computation.
     * @return true if this instance is ready, false otherwise.
     */
    public boolean isReady() {
        return mPoints != null && mPoints.size() >= MIN_POINTS;
    }
    
    /**
     * Indicates whether this instance is locked because computation is
     * in progress.
     * While an instance is in progress, no parameter can be modified and
     * no further computations can be done until instance becomes unlocked.
     * @return true if instance is locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }
    
    /**
     * Returns minimum x inhomogeneous coordinate found in provided points.
     * @return minimum x inhomogeneous coordinate found in provided points.
     */
    public double getMinInhomX() {
        return mMinInhomX;
    }
    
    /**
     * Returns minimum y inhomogeneous coordinate found in provided points.
     * @return minimum y inhomogeneous coordinate found in provided points.
     */
    public double getMinInhomY() {
        return mMinInhomY;
    }
    
    /**
     * Returns minimum z inhomogeneous coordinate found in provided points.
     * @return minimum z inhomogeneous coordinate found in provided points.
     */
    public double getMinInhomZ() {
        return mMinInhomZ;
    }
    
    /**
     * Returns maximum x inhomogeneous coordinate found in provided points.
     * @return maximum x inhomogeneous coordinate found in provided points.
     */
    public double getMaxInhomX() {
        return mMaxInhomX;
    }
    
    /**
     * Returns maximum y inhomogeneous coordinate found in provided points.
     * @return maximum y inhomogeneous coordinate found in provided points.
     */
    public double getMaxInhomY() {
        return mMaxInhomY;
    }
    
    /**
     * Returns maximum z inhomogeneous coordinate found in provided points.
     * @return maximum z inhomogeneous coordinate found in provided points.
     */
    public double getMaxInhomZ() {
        return mMaxInhomZ;
    }
    
    /**
     * Returns computed scale to normalize points on x coordinate.
     * @return computed scale to normalize points on x coordinate.
     */
    public double getScaleX() {
        return mScaleX;
    }
    
    /**
     * Returns computed scale to normalize points on y coordinate.
     * @return computed scale to normalize points on y coordinate.
     */
    public double getScaleY() {
        return mScaleY;
    }
    
    /**
     * Returns computed scale to normalize points on z coordinate.
     * @return computed scale to normalize points on z coordinate.
     */
    public double getScaleZ() {
        return mScaleZ;
    }
    
    /**
     * Returns computed x coordinate of centroid of points.
     * @return computed x coordinate of centroid of points.
     */
    public double getCentroidX() {
        return mCentroidX;
    }
    
    /**
     * Returns computed y coordinate of centroid of points.
     * @return computed y coordinate of centroid of points.
     */
    public double getCentroidY() {
        return mCentroidY;
    }
    
    /**
     * Returns computed z coordinate of centroid of points.
     * @return computed z coordinate of centroid of points.
     */
    public double getCentroidZ() {
        return mCentroidZ;
    }
    
    /**
     * Returns transformation to normalize points.
     * @return transformation to normalize points.
     */
    public ProjectiveTransformation3D getTransformation() {
        return mTransformation;
    }
    
    /**
     * Returns transformation to denormalize points, which corresponds to the
     * inverse transformation.
     * @return transformation to denormalize points.
     */
    public ProjectiveTransformation3D getInverseTransformation() {
        return mInverseTransformation;
    }
    
    /**
     * Indicates whether result (i.e. transformation and inverse transformation)
     * are available or not.
     * @return true if result is available, false otherwise.
     */
    public boolean isResultAvailable() {
        return mTransformation != null;
    }
    
    /**
     * Computes normalization and denormalization transformations
     * @throws NotReadyException if not enough data has been provided to
     * compute normalization.
     * @throws LockedException if instance is locked because another computation
     * is already in progress.
     * @throws NormalizerException if normalization failed due to numerical 
     * degeneracy. This usually happens when all provided points are located too
     * close to each other, which results in a singularity when computing proper
     * normalization scale.
     */
    public void compute() throws NotReadyException, LockedException,
            NormalizerException {
        if (!isReady()) {
            throw new NotReadyException();
        }
        if (isLocked()) {
            throw new LockedException();
        }
        try {
            mLocked = true;
            
            reset();
            computeLimits();
            
            //compute scale and centroids
            double width = mMaxInhomX - mMinInhomX;
            double height = mMaxInhomY - mMinInhomY;
            double depth = mMaxInhomZ - mMinInhomZ;
            
            if (width < Double.MIN_VALUE || height < Double.MIN_VALUE ||
                    depth < Double.MIN_VALUE) {
                //numerical degeneracy
                throw new NormalizerException();
            }
            
            mScaleX = 1.0 / width;
            mScaleY = 1.0 / height;
            mScaleZ = 1.0 / depth;
            
            //centroids of points
            mCentroidX = (mMinInhomX + mMaxInhomX) / 2.0;
            mCentroidY = (mMinInhomY + mMaxInhomY) / 2.0;
            mCentroidZ = (mMinInhomZ + mMaxInhomZ) / 2.0;
            
            //transformation to normalize points            
            Matrix t = new Matrix(
                    ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS);

            //X' = s * X + s * t -->
            //s * X = X' - s * t -->
            //X = 1/s*X' - t            
            t.setElementAt(0, 0, mScaleX);
            t.setElementAt(1, 1, mScaleY);
            t.setElementAt(2, 2, mScaleZ);
            t.setElementAt(0, 3, -mScaleX * mCentroidX);
            t.setElementAt(1, 3, -mScaleY * mCentroidY);
            t.setElementAt(2, 3, -mScaleZ * mCentroidZ);
            t.setElementAt(3, 3, 1.0);
            
            mTransformation = new ProjectiveTransformation3D(t);
            mTransformation.normalize();
            
            //transformation to denormalize points
            Matrix invT = new Matrix(
                    ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS);
            
            invT.setElementAt(0, 0, width);
            invT.setElementAt(1, 1, height);
            invT.setElementAt(2, 2, depth);
            invT.setElementAt(0, 3, mCentroidX);
            invT.setElementAt(1, 3, mCentroidY);
            invT.setElementAt(2, 3, mCentroidZ);
            invT.setElementAt(3, 3, 1.0);
            
            mInverseTransformation = new ProjectiveTransformation3D(invT);
            mInverseTransformation.normalize();
        } catch (Exception e) {
            throw new NormalizerException(e);
        } finally {
            mLocked = false;
        }
    }
    
    /**
     * Computes minimum and maximum inhomogeneous point coordinates from the
     * list of provided 2D points.
     */
    private void computeLimits() {
        for (Point3D point : mPoints) {
            double inhomX = point.getInhomX();
            double inhomY = point.getInhomY();
            double inhomZ = point.getInhomZ();
            if (inhomX < mMinInhomX) {
                mMinInhomX = inhomX;
            }
            if (inhomY < mMinInhomY) {
                mMinInhomY = inhomY;
            }
            if (inhomZ < mMinInhomZ) {
                mMinInhomZ = inhomZ;
            }
            
            if (inhomX > mMaxInhomX) {
                mMaxInhomX = inhomX;
            }
            if (inhomY > mMaxInhomY) {
                mMaxInhomY = inhomY;
            }
            if (inhomZ > mMaxInhomZ) {
                mMaxInhomZ = inhomZ;
            }
        }
    }
    
    /**
     * Sets list of points.
     * @param points list of points to be set.
     * @throws IllegalArgumentException if not enough points are provided, which
     * is MIN_POINTS.
     */
    private void internalSetPoints(List<Point3D> points) {
        if (points.size() < MIN_POINTS) {
            throw new IllegalArgumentException();
        }
        mPoints = points;   
    }
    
    /**
     * Resets internal values.
     */
    private void reset() {
        //reset result
        mTransformation = mInverseTransformation = null;
        //reset limits
        mMinInhomX = mMinInhomY = mMinInhomZ = Double.MAX_VALUE;
        mMaxInhomX = mMaxInhomY = mMaxInhomZ = -Double.MAX_VALUE;
        mScaleX = mScaleY = mScaleZ = 1.0;
        mCentroidX = mCentroidY = mCentroidZ = 0.0;
    }
    
}
