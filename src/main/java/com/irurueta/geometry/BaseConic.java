/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.BaseConic
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 1, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import java.io.Serializable;

/**
 *
 * Class defining the base interface of any possible conic.
 * Appropriate subclasses should be used for each conic type: pure conics and
 * dual conics.
 */
public abstract class BaseConic implements Serializable{
    /**
     * Number of rows of one matrix that contains conic parameters.
     */
    public static final int BASECONIC_MATRIX_ROW_SIZE = 3;
    
    /**
     * Number of columns of one matrix that contains conic parameters.
     */
    public static final int BASECONIC_MATRIX_COLUMN_SIZE = 3;
    
    /**
     * Number of parameters on a conic or dual conic.
     */
    public static final int N_PARAMS = 6;    
    
    /**
     * Threshold above zero used to determine whether a point lies inside
     * (is locus of) the given conic or not.
     */
    public static final double DEFAULT_LOCUS_THRESHOLD = 1e-12;
    
    /**
     * Threshold above zero used to determine whether two points of a conic
     * are perpendicular or not.
     */
    public static final double DEFAULT_PERPENDICULAR_THRESHOLD = 1e-12;
    
    /**
     * Threshold above zero to determine whether one matrix is symmetric or not.
     */
    public static final double DEFAULT_SYMMETRIC_THRESHOLD = 1e-12;
    
    /**
     * Minimum allowed threshold
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Machine precision
     */
    private static final double PRECISION = 1e-12;    

    
    /**
     * A element of the matrix defining a conic
     */
    protected double mA;
    
    /**
     * B element of the matrix defining a conic
     */    
    protected double mB;
    
    /**
     * C element of the matrix defining a conic
     */    
    protected double mC;
    
    /**
     * D element of the matrix defining a conic
     */    
    protected double mD;
    
    /**
     * E element of the matrix defining a conic
     */    
    protected double mE;
    
    /**
     * F element of the matrix defining a conic
     */    
    protected double mF;
    
    /**
     * Determines whether this instance is already mNormalized
     */
    private boolean mNormalized;
    
    /**
     * Constructor of this class.
     */
    public BaseConic(){
        mA = mB = mC = mD = mE = mF = 0.0;
        mNormalized = false;
    }
    
    /**
     * Constructor of this class. This constructor accepts every parameter
     * describing a base conic (parameters a, b, c, d, e, f)
     * @param a Parameter A of the base conic
     * @param b Parameter B of the base conic.
     * @param c Parameter C of the base conic.
     * @param d Parameter D of the base conic.
     * @param e Parameter E of the base conic.
     * @param f Parameter F of the base conic.
     */
    public BaseConic(double a, double b, double c, double d, double e, 
            double f){
        setParameters(a, b, c, d, e, f);
    }
    
    /**
     * Constructor. This constructor accepts a Matrix describing a base conic.
     * @param m 3x3 matrix describing a base conic
     * @param symmetricThreshold Grade of tolerance to determine whether a 
     * matrix is symmetric or not. It is used because due to the precision of 
     * the CPU, the values may not be exactly equal. (by default: 
     * DEFAULT_SYMMETRIC_THRESHOLD is used if none is provided)
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not
     * symmetric.
     * @throws IllegalArgumentException Raised when the size of the matrix is 
     * not 3x3.
     */
    public BaseConic(Matrix m, double symmetricThreshold)
            throws NonSymmetricMatrixException, IllegalArgumentException{
        setParameters(m, symmetricThreshold);
    }
    
    /**
     * Constructor. This constructor accepts a Matrix describing a base conic.
     * @param m 3x3 matrix describing a base conic
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not
     * symmetric.
     * @throws IllegalArgumentException Raised when the size of the matrix is 
     * not 3x3.
     * @see #BaseConic(Matrix, double)
     */
    public BaseConic(Matrix m) throws NonSymmetricMatrixException, 
            IllegalArgumentException{
        setParameters(m);
    }
    
    /**
     * Returns parameter A of the given base conic.
     * @return Parameter A of a matrix describing a base conic.
     */
    public double getA(){
        return mA;
    }
    
    /**
     * Returns parameter B of the given base conic.
     * @return Parameter B of a matrix describing a base conic.
     */
    public double getB(){
        return mB;
    }

    /**
     * Returns parameter C of the given base conic.
     * @return Parameter C of a matrix describing a base conic.
     */
    public double getC(){
        return mC;
    }

    /**
     * Returns parameter D of the given base conic.
     * @return Parameter D of a matrix describing a base conic.
     */
    public double getD(){
        return mD;
    }

    /**
     * Returns parameter E of the given base conic.
     * @return Parameter E of a matrix describing a base conic.
     */
    public double getE(){
        return mE;
    }

    /**
     * Returns parameter F of the given base conic.
     * @return Parameter F of a matrix describing a base conic.
     */
    public double getF(){
        return mF;
    }
    
    /**
     * This method accepts every parameter describing a base conic (parameters
     * a, b, c, d, e, f)
     * @param a Parameter A of the base conic.
     * @param b Parameter B of the base conic.
     * @param c Parameter C of the base conic.
     * @param d Parameter D of the base conic.
     * @param e Parameter E of the base conic.
     * @param f Parameter F of the base conic.
     */
    public final void setParameters(double a, double b, double c, double d,
            double e, double f){
        mA = a;
        mB = b;
        mC = c;
        mD = d;
        mE = e;
        mF = f;
        mNormalized = false;
    }
    
    /**
     * This method sets the matrix used for describing a base conic.
     * This matrix must be 3x3 and symmetric.
     * @param m 3x3 Matrix describing a base conic.
     * @param symmetricThreshold Grade of tolerance to determine whether a 
     * matrix is symmetric or not. It is used because due the precision of the 
     * CPU, the values may not be exactly equal. (by default: 
     * DEFAULT_SYMMETRIC_THRESHOLD is used)
     * @throws IllegalArgumentException Raised when the size of the matrix is 
     * not 3x3
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not 
     * symmetric.
     */
    public final void setParameters(Matrix m, double symmetricThreshold) 
            throws IllegalArgumentException, NonSymmetricMatrixException{
        if(m.getRows() != BASECONIC_MATRIX_ROW_SIZE ||
                m.getColumns() != BASECONIC_MATRIX_COLUMN_SIZE){
            throw new IllegalArgumentException();
        }else{
            if(!Utils.isSymmetric(m, symmetricThreshold)){
                throw new NonSymmetricMatrixException();
            }else{
                mA = m.getElementAt(0, 0);
                mB = m.getElementAt(0, 1);
                mC = m.getElementAt(1, 1);
                mD = m.getElementAt(0, 2);
                mE = m.getElementAt(1, 2);
                mF = m.getElementAt(2, 2);
                mNormalized = false;
            }
        }
    }
    
    /**
     * This method sets the matrix used for describing a base conic.
     * This matrix must be 3x3 and symmetric.
     * @param m 3x3 Matrix describing a base conic.
     * @throws IllegalArgumentException Raised when the size of the matrix is 
     * not 3x3
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not 
     * symmetric.
     */    
    public final void setParameters(Matrix m) throws IllegalArgumentException,
            NonSymmetricMatrixException{
        setParameters(m, DEFAULT_SYMMETRIC_THRESHOLD);
    }
    
    /**
     * This method sets the A parameter of a base conic.
     * @param a Parameter A of the given base conic.
     */
    public void setA(double a){
        mA = a;
        mNormalized = false;
    }
    
    /**
     * This method sets the B parameter of a base conic.
     * @param b Parameter B of the given base conic.
     */
    public void setB(double b){
        mB = b;
        mNormalized = false;
    }

    /**
     * This method sets the C parameter of a base conic.
     * @param c Parameter C of the given base conic.
     */
    public void setC(double c){
        mC = c;
        mNormalized = false;
    }

    /**
     * This method sets the D parameter of a base conic.
     * @param d Parameter D of the given base conic.
     */
    public void setD(double d){
        mD = d;
        mNormalized = false;
    }

    /**
     * This method sets the E parameter of a base conic.
     * @param e Parameter E of the given base conic.
     */
    public void setE(double e){
        mE = e;
        mNormalized = false;
    }

    /**
     * This method sets the F parameter of a base conic.
     * @param f Parameter F of the given base conic.
     */
    public void setF(double f){
        mF = f;
        mNormalized = false;
    }

    /**
     * Returns the matrix that describes this base conic.
     * @return 3x3 matrix describing this base conic.
     */
    public Matrix asMatrix(){
        Matrix out = null;
        try{
            out = new Matrix(BASECONIC_MATRIX_ROW_SIZE, 
                    BASECONIC_MATRIX_COLUMN_SIZE);
        }catch(WrongSizeException ignore){}
        asMatrix(out);        
        return out;
    }
    
    /**
     * Sets the values in provided matrix corresponding to this base conic.
     * @param m Provided matrix where values will be stored
     * @throws IllegalArgumentException Raised if provided matrix is not 3x3
     */
    public void asMatrix(Matrix m) throws IllegalArgumentException{
        
        if(m.getRows() != BASECONIC_MATRIX_ROW_SIZE ||
                m.getColumns() != BASECONIC_MATRIX_ROW_SIZE)
            throw new IllegalArgumentException();
        
        m.setElementAt(0, 0, mA);
        m.setElementAt(0, 1, mB);
        m.setElementAt(0, 2, mD);
        m.setElementAt(1, 0, mB);
        m.setElementAt(1, 1, mC);
        m.setElementAt(1, 2, mE);
        m.setElementAt(2, 0, mD);
        m.setElementAt(2, 1, mE);
        m.setElementAt(2, 2, mF);        
    }
    
    /**
     * Normalizes the Conic params using its norm
     */
    public void normalize(){
        if(!mNormalized){
            Matrix m = asMatrix();
            double norm = Utils.normF(m);
        
            if(norm > PRECISION && !Double.isNaN(norm)){
                mA /= norm;
                mB /= norm;
                mC /= norm;
                mD /= norm;
                mE /= norm;
                mF /= norm;
                mNormalized = true;
            }
        }
    }
    
    /**
     * Returns boolean indicating whether this base quadric has already been 
     * normalized
     * @return True if normalized, false otherwise
     */
    public boolean isNormalized(){
        return mNormalized;
    }
}