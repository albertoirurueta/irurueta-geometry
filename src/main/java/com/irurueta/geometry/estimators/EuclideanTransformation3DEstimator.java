/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.EuclideanTransformation3DEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date 22 January, 2017.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.EuclideanTransformation3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.Point3D;
import java.util.List;

/**
 * Estimator of a 3D euclidean transformation based on point correspondences.
 * This estimator uses Kabsch alhorithm.
 * A minimum of 4 non-coincident matched 3D input/output points is required for 
 * estimation.
 * For some point configurations 3 points are enough to find a valid solution.
 * If more points are provided an LMSE (Least Mean Squared Error) solution will
 * be found.
 * Based on: 
 * https://en.wikipedia.org/wiki/Kabsch_algorithm
 * http://nghiaho.com/?page_id=671
 */
public class EuclideanTransformation3DEstimator {
    
    /**
     * Minimum required number of matched points.
     */
    public static final int MINIMUM_SIZE = 4;
    
    /**
     * For some point configurations a solution can be found with only 3 points.
     */
    public static final int WEAK_MINIMUM_SIZE = 3;    
    
    /**
     * 3D input points.
     */
    private List<Point3D> mInputPoints;
    
    /**
     * 3D output points.
     */
    private List<Point3D> mOutputPoints; 
    
    /**
     * Listener to be notified of events such as when estimation starts or ends.
     */
    private EuclideanTransformation3DEstimatorListener mListener;
    
    /**
     * Indicates whether estimation can start with only 3 points or not.
     * True allows 3 points, false requires 4.
     */
    private boolean mWeakMinimumSizeAllowed;    
    
    /**
     * Indicates if this estimator is locked because an estimation is being 
     * computed.
     */
    private boolean mLocked;
    
    /**
     * Constructor.
     */
    public EuclideanTransformation3DEstimator() { }
    
    /**
     * Constructor.
     * @param inputPoints 3D input points.
     * @param outputPoints 3D output points.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than 4.
     */
    public EuclideanTransformation3DEstimator(List<Point3D> inputPoints,
            List<Point3D> outputPoints) throws IllegalArgumentException {
        internalSetPoints(inputPoints, outputPoints);
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts or ends.
     */
    public EuclideanTransformation3DEstimator(
            EuclideanTransformation3DEstimatorListener listener) {
        mListener = listener;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts or ends.
     * @param inputPoints 3D input points.
     * @param outputPoints 3D output points.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size or their size is smaller than 4.
     */
    public EuclideanTransformation3DEstimator(
            EuclideanTransformation3DEstimatorListener listener, 
            List<Point3D> inputPoints, List<Point3D> outputPoints) 
            throws IllegalArgumentException {
        mListener = listener;
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Constructor.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     */
    public EuclideanTransformation3DEstimator(boolean weakMinimumSizeAllowed) {
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
    }
    
    /**
     * Constructor.
     * @param inputPoints 3D input points.
     * @param outputPoints 3D output points.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than 4.
     */
    public EuclideanTransformation3DEstimator(List<Point3D> inputPoints,
            List<Point3D> outputPoints, boolean weakMinimumSizeAllowed) 
            throws IllegalArgumentException {
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
        internalSetPoints(inputPoints, outputPoints);
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts or ends.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     */
    public EuclideanTransformation3DEstimator(
            EuclideanTransformation3DEstimatorListener listener,
            boolean weakMinimumSizeAllowed) {
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
        mListener = listener;
    }
    
    /**
     * Constructor.
     * @param listener listener to be notified of events such as when estimation
     * starts or ends.
     * @param inputPoints 3D input points.
     * @param outputPoints 3D output points.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     * @throws IllegalArgumentException if provided lists of points don't have 
     * the same size or their size is smaller than 4.
     */
    public EuclideanTransformation3DEstimator(
            EuclideanTransformation3DEstimatorListener listener, 
            List<Point3D> inputPoints, List<Point3D> outputPoints,
            boolean weakMinimumSizeAllowed) 
            throws IllegalArgumentException {
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
        mListener = listener;
        internalSetPoints(inputPoints, outputPoints);
    }
    
    /**
     * Returns list of input points to be used to estimate an euclidean 3D
     * transformation.
     * Each point in the list of input points must be matched with the 
     * corresponding point in the list of output points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than #getMinimumPoints.
     * @return list of input points to be used to estimate an euclidean 3D 
     * transformation.
     */
    public List<Point3D> getInputPoints() {
        return mInputPoints;
    }
    
    /**
     * Returns list of output points ot be used to estimate an euclidean 3D
     * transformation.
     * Each point in the list of output points must be matched with the 
     * corresponding point in the list of input points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than #getMinimumPoints.
     * @return list of output points to be used to estimate an euclidean 3D 
     * transformation.
     */
    public List<Point3D> getOutputPoints() {
        return mOutputPoints;
    }
    
    /**
     * Sets list of points to be used to estimate an euclidean 3D 
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than #getMinimumPoints.
     * @param inputPoints list of input points to be used to estimate an 
     * euclidean 3D transformation.
     * @param outputPoints list of output points to be used to estimate an
     * euclidean 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than #getMinimumPoints.
     * @throws LockedException if estimator is locked because a computation is 
     * already in progress.
     */
    public void setPoints(List<Point3D> inputPoints, 
            List<Point3D> outputPoints) throws IllegalArgumentException, 
            LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPoints(inputPoints, outputPoints);
    }
    
    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts or ends.
     * @return listener to be notified of events.
     */
    public EuclideanTransformation3DEstimatorListener getListener() {
        return mListener;
    }
    
    /**
     * Sets listener to be notified of events such as when estimation starts or
     * ends.
     * @param listener listener to be notified of events.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(EuclideanTransformation3DEstimatorListener listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }
    
    /**
     * Indicates whether estimation can start with only 3 points or not.
     * @return true allows 3 points, false requires 4.
     */
    public boolean isWeakMinimumSizeAllowed() {
        return mWeakMinimumSizeAllowed;
    }
    
    /**
     * Specifies whether estimation can start with only 3 points or not.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     * @throws LockedException if estimator is locked.
     */
    public void setWeakMinimumSizeAllowed(boolean weakMinimumSizeAllowed) 
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
    }
    
    /**
     * Required minimum number of point correspondences to start the estimation.
     * Can be either 3 or 4.
     * @return minimum number of point correspondences.
     */
    public int getMinimumPoints() {
        return mWeakMinimumSizeAllowed ? WEAK_MINIMUM_SIZE : MINIMUM_SIZE;
    }    
    
    /**
     * Indicates whether listener has been provided and is available for 
     * retrieval.
     * @return true if available, false otherwise.
     */
    public boolean isListenerAvailable() {
        return mListener != null;
    }
    
    /**
     * Indicates if this instance is locked because estimation is being 
     * computed.
     * @return true if locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }
    
    /**
     * Indicates if estimator is ready to start the euclidean 3D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched points) are provided
     * and a minimum of MINIMUM_SIZE points are available.
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mInputPoints != null && mOutputPoints != null &&
                mInputPoints.size() == mOutputPoints.size() &&
                mInputPoints.size() >= getMinimumPoints();
    }
    
    /**
     * Estimates an euclidean 3D transformation using the list of matched input
     * and output 3D points.
     * A minimum of 4 matched non-coincident points is required. If more points 
     * are provided an LMSE (Least Mean Squared Error) solution will be found.
     * @return estimated euclidean 3D transformation.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if not enough data has been provided.
     * @throws CoincidentPointsException raised if transformation cannot be
     * estimated for some reason (point configuration degeneracy, duplicate 
     * points or numerical instabilities).
     */
    public EuclideanTransformation3D estimate() throws LockedException,
            NotReadyException, CoincidentPointsException {
        EuclideanTransformation3D result = new EuclideanTransformation3D();
        estimate(result);
        return result;
    }
    
    /**
     * Estimates an euclidean 3D transformation using the list of matched input
     * and output 3D points.
     * A minimum of 4 matched non-coincident points is required. If more points 
     * are provided an LMSE (Least Mean Squared Error) solution will be found.
     * @param result instance where result will be stored.
     * @throws LockedException if estimator is locked.
     * @throws NotReadyException if not enough data has been provided.
     * @throws CoincidentPointsException raised if transformation cannot be
     * estimated for some reason (point configuration degeneracy, duplicate 
     * points or numerical instabilities).
     */
    public void estimate(EuclideanTransformation3D result) 
            throws LockedException, NotReadyException, 
            CoincidentPointsException {
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
            
            Matrix inCentroid = computeCentroid(mInputPoints);        
            Matrix outCentroid = computeCentroid(mOutputPoints);
        
            Matrix m = new Matrix(
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        
            int n = mInputPoints.size();
            Point3D inputPoint, outputPoint;
            Matrix col = new Matrix(
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 1);
            Matrix row = new Matrix(1, 
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            Matrix tmp = new Matrix(
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            for (int i = 0; i < n; i++) {
                inputPoint = mInputPoints.get(i);
                outputPoint = mOutputPoints.get(i);
                
                col.setElementAtIndex(0, inputPoint.getInhomX() - 
                        inCentroid.getElementAtIndex(0));
                col.setElementAtIndex(1, inputPoint.getInhomY() - 
                        inCentroid.getElementAtIndex(1));
                col.setElementAtIndex(2, inputPoint.getInhomZ() - 
                        inCentroid.getElementAtIndex(2));
                
                row.setElementAtIndex(0, outputPoint.getInhomX() - 
                        outCentroid.getElementAtIndex(0));
                row.setElementAtIndex(1, outputPoint.getInhomY() - 
                        outCentroid.getElementAtIndex(1));
                row.setElementAtIndex(2, outputPoint.getInhomZ() - 
                        outCentroid.getElementAtIndex(2));
                
                col.multiply(row, tmp);
                m.add(tmp);
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
            
            if (!mWeakMinimumSizeAllowed && decomposer.getNullity() > 0) {
                throw new CoincidentPointsException();
            }            
            
            Matrix u = decomposer.getU();
            Matrix v = decomposer.getV();
                        
            //rotation R = V*U^T            
            u.transpose();
            v.multiply(u);
            Matrix r = v;
            
            if(Utils.det(r) < 0.0) {
                //multiply 3rd column of R by -1
                r.setElementAt(0, 2, -r.getElementAt(0, 2));
                r.setElementAt(1, 2, -r.getElementAt(1, 2));
                r.setElementAt(2, 2, -r.getElementAt(2, 2));
            }
            
            MatrixRotation3D rotation = new MatrixRotation3D(r);
            
            //translation
            Matrix t = r.multiplyAndReturnNew(inCentroid);
            t.multiplyByScalar(-1.0);
            t.add(outCentroid);
            
            result.setRotation(rotation);
            result.setTranslation(t.getBuffer());
            
            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
            
        } catch (CoincidentPointsException e) {
            throw e;
        } catch (AlgebraException e) {
            throw new CoincidentPointsException(e);
        } catch (InvalidRotationMatrixException e) {
            throw new CoincidentPointsException(e);
        } finally {
            mLocked = false;
        }
    }
    
    /**
     * Computes centroid of provided list of points using inhomogeneous
     * coordinates.
     * @param points list of points to compute centroid.
     * @return centroid.
     * @throws AlgebraException never thrown.
     */
    private static Matrix computeCentroid(List<Point3D> points) 
            throws AlgebraException {        
        double x = 0.0, y = 0.0, z = 0.0;
        double n = points.size();
        for (Point3D p : points) {
            x += p.getInhomX() / n;
            y += p.getInhomY() / n;
            z += p.getInhomZ() / n;
        }
        
        Matrix result = new Matrix(
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 1);
        result.setElementAtIndex(0, x);
        result.setElementAtIndex(1, y);
        result.setElementAtIndex(2, z);
        return result;
    }
    
    /**
     * Internal method to set lists of points to be used to estimate an 
     * euclidean 3D transformation.
     * This method does not check whether estimator is locked or not.
     * @param inputPoints list of input points to be used to estimate an 
     * euclidean 3D transformation.
     * @param outputPoints list of output points to be used to estimate an
     * euclidean 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     * the same size or their size is smaller than #getMinimumPoints.
     */
    private void internalSetPoints(List<Point3D> inputPoints,
            List<Point3D> outputPoints) throws IllegalArgumentException {
        if (inputPoints.size() < getMinimumPoints()) {
            throw new IllegalArgumentException();
        }
        if (inputPoints.size() != outputPoints.size()) {
            throw new IllegalArgumentException();
        }
        mInputPoints = inputPoints;
        mOutputPoints = outputPoints;        
    }
}
