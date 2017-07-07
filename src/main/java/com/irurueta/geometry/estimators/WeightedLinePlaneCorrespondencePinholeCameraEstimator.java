/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.estimators.WeightedLinePlaneCorrespondencePinholeCameraEstimator
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;
import com.irurueta.numerical.robust.WeightSelection;
import java.util.Iterator;
import java.util.List;

/**
 * This class implements pinhole camera estimator using a weighted algorithm and
 * point correspondences
 */
public class WeightedLinePlaneCorrespondencePinholeCameraEstimator extends
        LinePlaneCorrespondencePinholeCameraEstimator{

    /**
     * Minimum number of required equations to estimate a pinhole camera
     */
    public static final int MIN_NUMBER_OF_EQUATIONS = 11;
    
    /**
     * Default number of correspondences to be weighted and taken into account
     */
    public static final int DEFAULT_MAX_CORRESPONDENCES = 50;
    
    /**
     * Indicates if weights are sorted by default so that largest weighted
     * correspondences are used first
     */
    public static final boolean DEFAULT_SORT_WEIGHTS = true;
    
    /**
     * Defines tiny value considered as machine precision
     */
    public static final double EPS = 1e-8;
    
    /**
     * Maximum number of  correspondences to be weighted and taken into account
     */
    private int mMaxCorrespondences;
    
    /**
     * Indicates if weights are sorted by default so that largest weighted
     * correspondences are used first
     */
    private boolean mSortWeights;
    
    /**
     * Array containing weights for all point correspondences
     */
    private double[] mWeights;
    
    /**
     * Constructor
     */
    public WeightedLinePlaneCorrespondencePinholeCameraEstimator(){
        super();       
        mMaxCorrespondences = DEFAULT_MAX_CORRESPONDENCES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
        mWeights = null;
    }
    
    /**
     * Constructor with listener
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes
     */
    public WeightedLinePlaneCorrespondencePinholeCameraEstimator(
            PinholeCameraEstimatorListener listener){
        super(listener);
        mMaxCorrespondences = DEFAULT_MAX_CORRESPONDENCES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
        mWeights = null;        
    }
    
    /**
     * Constructor
     * @param planes list of corresponding 3D planes
     * @param lines2D list of corresponding 2D lines
     * @throws IllegalArgumentException if any of the lists are null
     * @throws WrongListSizesException if provided lists of correspondences 
     * don't have the same size and enough correspondences
     */
    public WeightedLinePlaneCorrespondencePinholeCameraEstimator(
            List<Plane> planes, List<Line2D> lines2D) 
            throws IllegalArgumentException, WrongListSizesException{
        super(planes, lines2D);
        mMaxCorrespondences = DEFAULT_MAX_CORRESPONDENCES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
        mWeights = null;        
    }
    
    /**
     * Constructor
     * @param planes list of corresponding 3D planes
     * @param lines2D list of corresponding 2D lines
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes
     * @throws IllegalArgumentException if any of the lists are null
     * @throws WrongListSizesException if provided lists of correspondences 
     * don't have the same size and enough correspondences
     */
    public WeightedLinePlaneCorrespondencePinholeCameraEstimator(
            List<Plane> planes, List<Line2D> lines2D, 
            PinholeCameraEstimatorListener listener) 
            throws IllegalArgumentException, WrongListSizesException{
        super(planes, lines2D, listener);
        mMaxCorrespondences = DEFAULT_MAX_CORRESPONDENCES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
        mWeights = null;        
    }    
    
    /**
     * Constructor
     * @param planes list of corresponding 3D planes
     * @param lines2D list of corresponding 2D lines
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @throws IllegalArgumentException if any of the lists are null
     * @throws WrongListSizesException if provided lists of correspondences 
     * don't have the same size and enough correspondences
     */
    public WeightedLinePlaneCorrespondencePinholeCameraEstimator(
            List<Plane> planes, List<Line2D> lines2D, double[] weights) 
            throws IllegalArgumentException, WrongListSizesException{
        super();
        mMaxCorrespondences = DEFAULT_MAX_CORRESPONDENCES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
        mWeights = null;         
        internalSetListsAndWeights(planes, lines2D, weights);       
    }
    
    /**
     * Constructor
     * @param planes list of corresponding 3D planes
     * @param lines2D list of corresponding 2D lines
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @param listener listener to be notified of events such as when estimation
     * starts, ends or estimation progress changes
     * @throws IllegalArgumentException if any of the lists are null
     * @throws WrongListSizesException if provided lists of correspondences 
     * don't have the same size and enough correspondences
     */
    public WeightedLinePlaneCorrespondencePinholeCameraEstimator(
            List<Plane> planes, List<Line2D> lines2D, double[] weights,
            PinholeCameraEstimatorListener listener) 
            throws IllegalArgumentException, WrongListSizesException{
        super(listener);
        mMaxCorrespondences = DEFAULT_MAX_CORRESPONDENCES;
        mSortWeights = DEFAULT_SORT_WEIGHTS;
        mWeights = null;                
        internalSetListsAndWeights(planes, lines2D, weights);
    }
    
    /**
     * Internal method to set list of corresponding points (it does not check
     * if estimator is locked)
     * @param planes list of corresponding 3D planes
     * @param lines2D list of corresponding 2D lines
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @throws IllegalArgumentException if any of the lists or arrays are null
     * @throws WrongListSizesException if provided lists of correspondences 
     * don't have the same size and enough correspondences
     */
    private void internalSetListsAndWeights(List<Plane> planes, 
            List<Line2D> lines2D, double[] weights) 
            throws IllegalArgumentException,
            WrongListSizesException{
        
        if(planes == null || lines2D == null || weights == null) 
            throw new IllegalArgumentException();
        
        if(!areValidListsAndWeights(planes, lines2D, weights))
            throw new WrongListSizesException();
        
        mPlanes = planes;
        mLines2D = lines2D;
        mWeights = weights;
    }    
    
    /**
     * Sets list of corresponding planes and lines
     * @param planes list of corresponding 3D planes
     * @param lines2D list of corresponding 2D lines
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @throws LockedException if estimator is locked
     * @throws IllegalArgumentException if any of the lists are null
     * @throws WrongListSizesException if provided lists of correspondences 
     * don't have the same size and enough correspondences
     */
    public void setListsAndWeights(List<Plane> planes, 
            List<Line2D> lines2D, double[] weights)
            throws LockedException, IllegalArgumentException, 
            WrongListSizesException{
        if(isLocked()) throw new LockedException();
        
        internalSetListsAndWeights(planes, lines2D, weights);
    }    
    
    /**
     * Indicates if lists of corresponding planes and lines are valid.
     * Lists are considered valid if they have the same number of 
     * correspondences and both have more than the required minimum of 
     * correspondences (which is 4)
     * @param planes list of corresponding 3D planes
     * @param lines2D list of corresponding 2D lines
     * @param weights array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @return true if corresponding planes and lines are valid, false otherwise
     */
    public static boolean areValidListsAndWeights(List<Plane> planes, 
            List<Line2D> lines2D, double[] weights){
        if(planes == null || lines2D == null || weights == null) 
            return false;
        return planes.size() == lines2D.size() && 
                lines2D.size() == weights.length &&
                planes.size() >= MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES;
    }

    /**
     * Returns array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     * @return array containing weights for each correspondence.
     * @throws NotAvailableException if weights are not available
     */
    public double[] getWeights() throws NotAvailableException{
        if(!areWeightsAvailable()) throw new NotAvailableException();
        return mWeights;
    }
    
    /**
     * Returns boolean indicating whether weights have been provided and are
     * available for retrieval
     * @return true if weights are available, false otherwise.
     */
    public boolean areWeightsAvailable(){
        return mWeights != null;
    }    
    
 /**
     * Returns maximum number of correspondences to be weighted and taken into 
     * account
     * @return maximum number of points to be weighted
     */
    public int getMaxCorrespondences(){
        return mMaxCorrespondences;
    }
    
    /**
     * Sets maximum number of correspondences to be weighted and taken into 
     * account
     * @param maxCorrespondences maximum number of correspondences to be 
     * weighted
     * @throws IllegalArgumentException if provided value is less than the 
     * minimum allowed number of point correspondences
     * @throws LockedException if this instance is locked
     */
    public void setMaxCorrespondences(int maxCorrespondences) 
            throws IllegalArgumentException, LockedException{
        if(isLocked()) throw new LockedException();
        if(maxCorrespondences < MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES)
            throw new IllegalArgumentException();
        
        mMaxCorrespondences = maxCorrespondences;
    }
    
    /**
     * Indicates if weights are sorted by so that largest weighted
     * correspondences are used first
     * @return true if weights are sorted, false otherwise
     */
    public boolean isSortWeightsEnabled(){
        return mSortWeights;
    }
    
    /**
     * Specifies whether weights are sorted by so that largest weighted
     * correspondences are used first
     * @param sortWeights true if weights are sorted, false otherwise
     * @throws LockedException if this instance is locked
     */
    public void setSortWeightsEnabled(boolean sortWeights) 
            throws LockedException{
        if(isLocked()) throw new LockedException();
        
        mSortWeights = sortWeights;
    }    
    
    /**
     * Indicates if this estimator is ready to start the estimation.
     * Estimator will be ready once both lists and weights are available
     * @return true if estimator is ready, false otherwise
     */    
    @Override
    public boolean isReady() {
        return areListsAvailable() && areWeightsAvailable();
    }

    /**
     * Estimates a pinhole camera
     * @return estimated pinhole camera
     * @throws LockedException if estimator is locked
     * @throws NotReadyException if input has not yet been provided
     * @throws PinholeCameraEstimatorException if an error occurs during 
     * estimation, usually because input data is not valid
     */        
    @Override
    public PinholeCamera estimate() throws LockedException, NotReadyException, 
        PinholeCameraEstimatorException {
        
        if(isLocked()) throw new LockedException();
        if(!isReady()) throw new NotReadyException();
        
        try{
            mLocked = true;
            if(mListener != null) mListener.onEstimateStart(this);
            
            WeightSelection selection = WeightSelection.selectWeights(mWeights, 
                    mSortWeights, mMaxCorrespondences);
            boolean[] selected = selection.getSelected();
            
            Matrix a = new Matrix(12, 12);
            Matrix row = new Matrix(3, 12);
            Matrix transRow = new Matrix(12, 3);
            Matrix tmp = new Matrix(12, 12);
            
            Iterator<Line2D> iterator2D = mLines2D.iterator();
            Iterator<Plane> iterator3D = mPlanes.iterator();
            
            Line2D line2D;
            Plane plane;
            int index = 0;
            int nMatches = 0;
            double la, lb, lc;
            double A, B, C, D;
            double weight;
            double previousNorm = 1.0;
            double rowNorm;
            while(iterator2D.hasNext() && iterator3D.hasNext()){
                line2D = iterator2D.next();
                plane = iterator3D.next();
                
                if(selected[index]){
                    weight = mWeights[index];
                    
                    if(Math.abs(weight) < EPS){
                        //skip, because weight is too small
                        index++;
                        continue;
                    }
                    
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
                    row.setElementAt(0, 0, -D * la * weight);
                    row.setElementAt(0, 1, -D * lb * weight);
                    row.setElementAt(0, 2, -D * lc * weight);
                
                    //columns 3, 4, 5, 6, 7, 8 are left with zero values
                
                    row.setElementAt(0, 9, A * la * weight);
                    row.setElementAt(0, 10, A * lb * weight);
                    row.setElementAt(0, 11, A * lc * weight);
                    
                    //normalize row
                    rowNorm = Math.sqrt(
                            Math.pow(row.getElementAt(0, 0), 2.0) +
                            Math.pow(row.getElementAt(0, 1), 2.0) +
                            Math.pow(row.getElementAt(0, 2), 2.0) +
                            Math.pow(row.getElementAt(0, 9), 2.0) +
                            Math.pow(row.getElementAt(0, 10), 2.0) +
                            Math.pow(row.getElementAt(0, 11), 2.0));
                    
                    row.setElementAt(0, 0, row.getElementAt(0, 0) / rowNorm);
                    row.setElementAt(0, 1, row.getElementAt(0, 1) / rowNorm);
                    row.setElementAt(0, 2, row.getElementAt(0, 2) / rowNorm);
                    row.setElementAt(0, 9, row.getElementAt(0, 9) / rowNorm);
                    row.setElementAt(0, 10, row.getElementAt(0, 10) / rowNorm);
                    row.setElementAt(0, 11, row.getElementAt(0, 11) / rowNorm);
                
                    //second row
                
                    //columns 0, 1, 2 are left with zero values
                
                    row.setElementAt(1, 3, -D * la * weight);
                    row.setElementAt(1, 4, -D * lb * weight);
                    row.setElementAt(1, 5, -D * lc * weight);
                
                    //columns 6, 7, 8 are left with zero values
                
                    row.setElementAt(1, 9, B * la * weight);
                    row.setElementAt(1, 10, B * lb * weight);
                    row.setElementAt(1, 11, B * lc * weight);
                    
                    //normalize row
                    rowNorm = Math.sqrt(
                            Math.pow(row.getElementAt(1, 3), 2.0) +
                            Math.pow(row.getElementAt(1, 4), 2.0) +
                            Math.pow(row.getElementAt(1, 5), 2.0) +
                            Math.pow(row.getElementAt(1, 9), 2.0) +
                            Math.pow(row.getElementAt(1, 10), 2.0) +
                            Math.pow(row.getElementAt(1, 11), 2.0));
                    
                    row.setElementAt(1, 3, row.getElementAt(1, 3) / rowNorm);
                    row.setElementAt(1, 4, row.getElementAt(1, 4) / rowNorm);
                    row.setElementAt(1, 5, row.getElementAt(1, 5) / rowNorm);
                    row.setElementAt(1, 9, row.getElementAt(1, 9) / rowNorm);
                    row.setElementAt(1, 10, row.getElementAt(1, 10) / rowNorm);
                    row.setElementAt(1, 11, row.getElementAt(1, 11) / rowNorm);
                    
                                
                    //third row
                
                    //columns 0, 1, 2, 3, 4, 5 are left with zero values
                
                    row.setElementAt(2, 6, -D * la * weight);
                    row.setElementAt(2, 7, -D * lb * weight);
                    row.setElementAt(2, 8, -D * lc * weight);
                
                    row.setElementAt(2, 9, C * la * weight);
                    row.setElementAt(2, 10, C * lb * weight);
                    row.setElementAt(2, 11, C * lc * weight);
                    
                    //normalize row
                    rowNorm = Math.sqrt(
                            Math.pow(row.getElementAt(2, 6), 2.0) +
                            Math.pow(row.getElementAt(2, 7), 2.0) +
                            Math.pow(row.getElementAt(2, 8), 2.0) +
                            Math.pow(row.getElementAt(2, 9), 2.0) +
                            Math.pow(row.getElementAt(2, 10), 2.0) +
                            Math.pow(row.getElementAt(2, 11), 2.0));
                    
                    row.setElementAt(2, 6, row.getElementAt(2, 6) / rowNorm);
                    row.setElementAt(2, 7, row.getElementAt(2, 7) / rowNorm);
                    row.setElementAt(2, 8, row.getElementAt(2, 8) / rowNorm);
                    row.setElementAt(2, 9, row.getElementAt(2, 9) / rowNorm);
                    row.setElementAt(2, 10, row.getElementAt(2, 10) / rowNorm);
                    row.setElementAt(2, 11, row.getElementAt(2, 11) / rowNorm);
                    
                    row.transpose(transRow); //transRow = row'
                    transRow.multiply(row, tmp); //tmp = row' * row
                    
                    tmp.multiplyByScalar(1.0 / previousNorm);
                    
                    //a += 1.0 / previousNorm * tmp
                    a.add(tmp);
                    //normalize
                    previousNorm = Utils.normF(a);
                    a.multiplyByScalar(1.0 / previousNorm);
                    
                    nMatches++;
                }
                index++;
            }
            
            if(nMatches < MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES){
                throw new PinholeCameraEstimatorException();
            }
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();
            
            if(decomposer.getNullity() > 1){
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
            mLocked = false; //ensure it is no longer locked
            throw e;
        } catch (Exception e) {
            mLocked = false; //ensure it is no longer locked
            throw new PinholeCameraEstimatorException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Returns type of pinhole camera estimator
     * @return type of pinhole camera estimator
     */    
    @Override
    public PinholeCameraEstimatorType getType() {
        return PinholeCameraEstimatorType.
                WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR;
    }
    
}
