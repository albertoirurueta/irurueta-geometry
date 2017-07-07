/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.DLTLinePlaneCorrespondencePinholeCameraEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 24, 2013
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
import java.util.Iterator;
import java.util.List;

/**
 * This class implements an algorithm to estimate pinhole camera using the DLT
 * algorithm and point correspondences.
 */
public class DLTLinePlaneCorrespondencePinholeCameraEstimator extends
        LinePlaneCorrespondencePinholeCameraEstimator {
    
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
     * Defines tiny value considered as machine precision.
     */
    public static final double EPS = 1e-8;
    
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
    public DLTLinePlaneCorrespondencePinholeCameraEstimator() {
        super();
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor with listener.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     */
    public DLTLinePlaneCorrespondencePinholeCameraEstimator(
            PinholeCameraEstimatorListener listener) {
        super(listener);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }
    
    /**
     * Constructor.
     * @param planes list of corresponding 3D planes.
     * @param lines2D list of corresponding 2D lines.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of correspondences 
     * don't have the same size and enough correspondences.
     */
    public DLTLinePlaneCorrespondencePinholeCameraEstimator(List<Plane> planes,
            List<Line2D> lines2D) throws IllegalArgumentException,
            WrongListSizesException {
        super(planes, lines2D);
        mAllowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }

    /**
     * Constructor.
     * @param planes list of corresponding 3D planes.
     * @param lines2D list of corresponding 2D lines.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException if provided lists of correspondences
     * don't have the same size and enough correspondences.
     */
    public DLTLinePlaneCorrespondencePinholeCameraEstimator(List<Plane> planes,
            List<Line2D> lines2D, PinholeCameraEstimatorListener listener) 
            throws IllegalArgumentException, WrongListSizesException {
        super(planes, lines2D, listener);
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
     * @param allowed true if LMSE solution is allowed, false otherwise
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
        return areListsAvailable();
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
            int nLines = mLines2D.size();
            
            mLocked = true;
            if(mListener != null) mListener.onEstimateStart(this);
            
            Matrix a;
            if (isLMSESolutionAllowed()) {
                //initialize new matrix to zero when LMSE is enabled
                a = new Matrix(3 * nLines, 12);
            } else {
                //When LMSE is disabled, initialize new matrix to zero only with
                //11 equations
                a = new Matrix(MIN_NUMBER_OF_EQUATIONS, 12);
            }
            
            Iterator<Line2D> iterator2D = mLines2D.iterator();
            Iterator<Plane> iterator3D = mPlanes.iterator();
            
            Line2D line2D;
            Plane plane;
            int counter = 0;
            double la, lb, lc;
            double A, B, C, D;
            double rowNorm;
            while (iterator2D.hasNext() && iterator3D.hasNext()) {
                line2D = iterator2D.next();
                plane = iterator3D.next();
                
                //normalize lines and planes to increase accuracy
                line2D.normalize();
                plane.normalize();
                
                la = line2D.getA();
                lb = line2D.getB();
                lc = line2D.getC();
                
                A = plane.getA();
                B = plane.getB();
                C = plane.getC();
                D = plane.getD();
                
                //first row
                a.setElementAt(counter, 0, -D * la);
                a.setElementAt(counter, 1, -D * lb);
                a.setElementAt(counter, 2, -D * lc);
                
                //columns 3, 4, 5, 6, 7, 8 are left with zero values
                
                a.setElementAt(counter, 9, A * la);
                a.setElementAt(counter, 10, A * lb);
                a.setElementAt(counter, 11, A * lc);
                
                //normalize row
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 0), 2.0) +
                        Math.pow(a.getElementAt(counter, 1), 2.0) +
                        Math.pow(a.getElementAt(counter, 2), 2.0) +
                        Math.pow(a.getElementAt(counter, 9), 2.0) +
                        Math.pow(a.getElementAt(counter, 10), 2.0) +
                        Math.pow(a.getElementAt(counter, 11), 2.0));
                
                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / 
                        rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / 
                        rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / 
                        rowNorm);
                a.setElementAt(counter, 9, a.getElementAt(counter, 9) / 
                        rowNorm);
                a.setElementAt(counter, 10, a.getElementAt(counter, 10) / 
                        rowNorm);
                a.setElementAt(counter, 11, a.getElementAt(counter, 11) / 
                        rowNorm);
                counter++;                

                //second row
                
                //columns 0, 1, 2 are left with zero values
                
                a.setElementAt(counter, 3, -D * la);
                a.setElementAt(counter, 4, -D * lb);
                a.setElementAt(counter, 5, -D * lc);
                
                //columns 6, 7, 8 are left with zero values
                
                a.setElementAt(counter, 9, B * la);
                a.setElementAt(counter, 10, B * lb);
                a.setElementAt(counter, 11, B * lc);
                
                //normalize row
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 3), 2.0) +
                        Math.pow(a.getElementAt(counter, 4), 2.0) +
                        Math.pow(a.getElementAt(counter, 5), 2.0) +
                        Math.pow(a.getElementAt(counter, 9), 2.0) +
                        Math.pow(a.getElementAt(counter, 10), 2.0) +
                        Math.pow(a.getElementAt(counter, 11), 2.0));
                
                a.setElementAt(counter, 3, a.getElementAt(counter, 3) / 
                        rowNorm);
                a.setElementAt(counter, 4, a.getElementAt(counter, 4) / 
                        rowNorm);
                a.setElementAt(counter, 5, a.getElementAt(counter, 5) / 
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
                if(!isLMSESolutionAllowed() && (counter >= MIN_NUMBER_OF_EQUATIONS))
                    break;
                
                //third row
                
                //columns 0, 1, 2, 3, 4, 5 are left with zero values
                
                a.setElementAt(counter, 6, -D * la);
                a.setElementAt(counter, 7, -D * lb);
                a.setElementAt(counter, 8, -D * lc);                
                
                a.setElementAt(counter, 9, C * la);
                a.setElementAt(counter, 10, C * lb);
                a.setElementAt(counter, 11, C * lc);
                
                //normalize row
                rowNorm = Math.sqrt(
                        Math.pow(a.getElementAt(counter, 6), 2.0) +
                        Math.pow(a.getElementAt(counter, 7), 2.0) +
                        Math.pow(a.getElementAt(counter, 8), 2.0) +
                        Math.pow(a.getElementAt(counter, 9), 2.0) +
                        Math.pow(a.getElementAt(counter, 10), 2.0) +
                        Math.pow(a.getElementAt(counter, 11), 2.0));
                
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
                //line/plane configuration is degenerate and exists a linear 
                //combination of possible pinhole cameras (i.e. solution is not
                //unique up to scale)
                throw new PinholeCameraEstimatorException();
            }
            
            Matrix v = decomposer.getV();
            
            //use last column of V as pinhole camera vector
            
            //the last column of V contains pinhole camera matrix ordered by
            //columns as: P11, P21, P31, P12, P22, P32, P13, P23, P33, P14, P24, 
            //P34, hence we reorder p
            Matrix pinholeCameraMatrix = new Matrix(
                    PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS,
                    PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);
            
            pinholeCameraMatrix.setElementAt(0, 0, v.getElementAt(0, 11));
            pinholeCameraMatrix.setElementAt(1, 0, v.getElementAt(1, 11));
            pinholeCameraMatrix.setElementAt(2, 0, v.getElementAt(2, 11));

            pinholeCameraMatrix.setElementAt(0, 1, v.getElementAt(3, 11));
            pinholeCameraMatrix.setElementAt(1, 1, v.getElementAt(4, 11));
            pinholeCameraMatrix.setElementAt(2, 1, v.getElementAt(5, 11));

            pinholeCameraMatrix.setElementAt(0, 2, v.getElementAt(6, 11));
            pinholeCameraMatrix.setElementAt(1, 2, v.getElementAt(7, 11));
            pinholeCameraMatrix.setElementAt(2, 2, v.getElementAt(8, 11));

            pinholeCameraMatrix.setElementAt(0, 3, v.getElementAt(9, 11));
            pinholeCameraMatrix.setElementAt(1, 3, v.getElementAt(10, 11));
            pinholeCameraMatrix.setElementAt(2, 3, v.getElementAt(11, 11));
            
            //because pinholeCameraMatrix has been obtained as the last column 
            //of V, then its frobenius norm will be 1 because SVD already 
            //returns normalized singular vector
            
            PinholeCamera camera = new PinholeCamera(pinholeCameraMatrix);
            
            if(mListener != null) mListener.onEstimateEnd(this);
            
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
     * Returns type of pinhole camera estimator.
     * @return type of pinhole camera estimator.
     */        
    @Override
    public PinholeCameraEstimatorType getType() {
        return PinholeCameraEstimatorType.
                DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR;
    }    
}
