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
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;

import java.util.Iterator;
import java.util.List;

/**
 * This class implements an algorithm to estimate pinhole cameras using the DLT
 * algorithm and point correspondences.
 */
@SuppressWarnings("WeakerAccess")
public class DLTPointCorrespondencePinholeCameraEstimator extends 
        PointCorrespondencePinholeCameraEstimator {
    
    /**
     * Minimum number of required equations to estimate a pinhole camera.
     */
    public static final int MIN_NUMBER_OF_EQUATIONS = 11;
    
    /**
     * Indicates if by default an LMSE (Least Mean Square Error) solution is 
     * allowed if more correspondences than the minimum are provided.
     */
    public static final boolean DEFAULT_ALLOW_LMSE_SOLUTION = false;
        
    /**
     * Indicates if an LMSE (Least Mean Square Error) solution is allowed if 
     * more correspondences than the minimum are provided. If false, the 
     * exceeding correspondences will be ignored and only the 6 first 
     * correspondences will be used.
     */
    private boolean mAllowLMSESolution;
        
    /**
     * Constructor.
     */
    public DLTPointCorrespondencePinholeCameraEstimator() {
        super();
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public DLTPointCorrespondencePinholeCameraEstimator(
            PinholeCameraEstimatorListener listener) {
        super(listener);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor.
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of points don't have
     * the same size and enough points.
     */
    public DLTPointCorrespondencePinholeCameraEstimator(List<Point3D> points3D,
            List<Point2D> points2D) throws WrongListSizesException {
        super(points3D, points2D);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
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
    public DLTPointCorrespondencePinholeCameraEstimator(List<Point3D> points3D,
            List<Point2D> points2D, PinholeCameraEstimatorListener listener) 
            throws WrongListSizesException {
        super(points3D, points2D, listener);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Indicates if an LMSE (Least Mean Square Error) solution is allowed if 
     * more correspondences than the minimum are provided. If false, the 
     * exceeding correspondences will be ignored and only the 6 first 
     * correspondences will be used.
     * @return true if LMSE solution is allowed, false otherwise.
     */
    public boolean isLMSESolutionAllowed() {
        return mAllowLMSESolution;
    }
    
    /**
     * Specifies if an LMSE (Least Mean Square Error) solution is allowed if
     * more correspondences than the minimum are provided. If false, the
     * exceeding correspondences will be ignored and only the 6 first 
     * correspondences will be used.
     * @param allowed true if LMSE solution is allowed, false otherwise.
     * @throws LockedException if estimator is locked.
     */
    public void setLMSESolutionAllowed(boolean allowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mAllowLMSESolution = allowed;
    }
        
    /**
     * Indicates if this estimator is ready to start the estimation.
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return areListsAvailable() && areValidLists(mPoints3D, mPoints2D);
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
    @Override
    protected Matrix internalEstimate(List<Point3D> points3D,
            List<Point2D> points2D) throws PinholeCameraEstimatorException {
        
        try {
            int nPoints = points2D.size();

            Matrix a;
            if (isLMSESolutionAllowed()) {
                //initialize new matrix to zero when LMSE is enabled
                a = new Matrix(2 * nPoints, 12);
            } else {
                //When LMSE is disabled, initialize new matrix to zero only with
                //11 equations
                a = new Matrix(MIN_NUMBER_OF_EQUATIONS, 12);
            }

            Iterator<Point2D> iterator2D = points2D.iterator();
            Iterator<Point3D> iterator3D = points3D.iterator();

            Point2D point2D;
            Point3D point3D;
            int counter = 0;
            double homImageX;
            double homImageY;
            double homImageW;
            double homWorldX;
            double homWorldY;
            double homWorldZ;
            double homWorldW;
            double rowNorm;
            while (iterator2D.hasNext() && iterator3D.hasNext()) {
                point2D = iterator2D.next();
                point3D = iterator3D.next();

                //normalize points to increase accuracy
                point2D.normalize();
                point3D.normalize();

                homImageX = point2D.getHomX();
                homImageY = point2D.getHomY();
                homImageW = point2D.getHomW();

                homWorldX = point3D.getHomX();
                homWorldY = point3D.getHomY();
                homWorldZ = point3D.getHomZ();
                homWorldW = point3D.getHomW();

                //first row (even)
                a.setElementAt(counter, 0, homImageW * homWorldX);
                a.setElementAt(counter, 1, homImageW * homWorldY);
                a.setElementAt(counter, 2, homImageW * homWorldZ);
                a.setElementAt(counter, 3, homImageW * homWorldW);

                //columns 4, 5, 6, 7 are left with zero values

                a.setElementAt(counter, 8, -homImageX * homWorldX);
                a.setElementAt(counter, 9, -homImageX * homWorldY);
                a.setElementAt(counter, 10, -homImageX * homWorldZ);
                a.setElementAt(counter, 11, -homImageX * homWorldW);

                //normalize row
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0) +
                        Math.pow(a.getElementAt(counter, 3), 2.0) +
                        Math.pow(a.getElementAt(counter, 8), 2.0) +
                        Math.pow(a.getElementAt(counter, 9), 2.0) +
                        Math.pow(a.getElementAt(counter, 10), 2.0) +
                        Math.pow(a.getElementAt(counter, 11), 2.0));

                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);
                a.setElementAt(counter, 3, a.getElementAt(counter, 3) / 
                        rowNorm);
                a.setElementAt(counter, 8, a.getElementAt(counter, 8) / 
                        rowNorm);
                a.setElementAt(counter, 9, a.getElementAt(counter, 9) / 
                        rowNorm);
                a.setElementAt(counter, 10, a.getElementAt(counter, 10) / 
                        rowNorm);
                a.setElementAt(counter, 11, a.getElementAt(counter, 11) / 
                        rowNorm);
                counter++;

                //in case we want an exact solution (up to scale) when LMSE is
                //disabled, we stop after 11 equations
                if (!isLMSESolutionAllowed() && (counter >= MIN_NUMBER_OF_EQUATIONS)) {
                    break;
                }

                //second row (odd)

                //columns 0, 1, 2, 3 are left with zero values                

                a.setElementAt(counter, 4, homImageW * homWorldX);
                a.setElementAt(counter, 5, homImageW * homWorldY);
                a.setElementAt(counter, 6, homImageW * homWorldZ);
                a.setElementAt(counter, 7, homImageW * homWorldW);                                

                a.setElementAt(counter, 8, -homImageY * homWorldX);
                a.setElementAt(counter, 9, -homImageY * homWorldY);
                a.setElementAt(counter, 10, -homImageY * homWorldZ);
                a.setElementAt(counter, 11, -homImageY * homWorldW);

                //normalize row
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 4), 2.0) +
                        Math.pow(a.getElementAt(counter, 5), 2.0) +
                        Math.pow(a.getElementAt(counter, 6), 2.0) +
                        Math.pow(a.getElementAt(counter, 7), 2.0) +
                        Math.pow(a.getElementAt(counter, 8), 2.0) +
                        Math.pow(a.getElementAt(counter, 9), 2.0) +
                        Math.pow(a.getElementAt(counter, 10), 2.0) +
                        Math.pow(a.getElementAt(counter, 11), 2.0));

                a.setElementAt(counter, 4, a.getElementAt(counter, 4) / 
                        rowNorm);
                a.setElementAt(counter, 5, a.getElementAt(counter, 5) / 
                        rowNorm);
                a.setElementAt(counter, 6, a.getElementAt(counter, 6) / 
                        rowNorm);
                a.setElementAt(counter, 7, a.getElementAt(counter, 7) / 
                        rowNorm);
                a.setElementAt(counter, 8, a.getElementAt(counter, 8) / 
                        rowNorm);
                a.setElementAt(counter, 9, a.getElementAt(counter, 9) / 
                        rowNorm);
                a.setElementAt(counter, 10, a.getElementAt(counter, 10) / 
                        rowNorm);
                a.setElementAt(counter, 11, a.getElementAt(counter, 11) / 
                        rowNorm);
                counter++;                
            }

            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();

            if (decomposer.getNullity() > 1) {
                //point configuration is degenerate and exists a linear 
                //combination of possible pinhole cameras (i.e. solution is not
                //unique up to scale)
                throw new PinholeCameraEstimatorException();
            }

            Matrix v = decomposer.getV();

            //use last column of V as pinhole camera vector

            //the last column of V contains pinhole camera matrix ordered by 
            //rows as: P11, P12, P13, P14, P21, P22, P23, P24, P31, P32, P33, 
            //P34, hence we reorder p
            Matrix pinholeCameraMatrix = new Matrix(
                    PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS,
                    PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);

            pinholeCameraMatrix.setElementAt(0, 0, v.getElementAt(0, 11));
            pinholeCameraMatrix.setElementAt(0, 1, v.getElementAt(1, 11));
            pinholeCameraMatrix.setElementAt(0, 2, v.getElementAt(2, 11));
            pinholeCameraMatrix.setElementAt(0, 3, v.getElementAt(3, 11));

            pinholeCameraMatrix.setElementAt(1, 0, v.getElementAt(4, 11));
            pinholeCameraMatrix.setElementAt(1, 1, v.getElementAt(5, 11));
            pinholeCameraMatrix.setElementAt(1, 2, v.getElementAt(6, 11));
            pinholeCameraMatrix.setElementAt(1, 3, v.getElementAt(7, 11));

            pinholeCameraMatrix.setElementAt(2, 0, v.getElementAt(8, 11));
            pinholeCameraMatrix.setElementAt(2, 1, v.getElementAt(9, 11));
            pinholeCameraMatrix.setElementAt(2, 2, v.getElementAt(10, 11));
            pinholeCameraMatrix.setElementAt(2, 3, v.getElementAt(11, 11));

            //because pinholeCameraMatrix has been obtained as the last column 
            //of V, then its frobenius norm will be 1 because SVD already 
            //returns normalized singular vector     

            return pinholeCameraMatrix;
            
        } catch (PinholeCameraEstimatorException e) {
            throw e;
        } catch (Exception e) {
            throw new PinholeCameraEstimatorException(e);
        }
    }
    
    /**
     * Returns type of pinhole camera estimator.
     * @return type of pinhole camera estimator.
     */    
    @Override
    public PinholeCameraEstimatorType getType() {
        return PinholeCameraEstimatorType.DLT_POINT_PINHOLE_CAMERA_ESTIMATOR;
    }    
}
