/*
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.DualConic
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 3, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.*;
import java.io.Serializable;

/**
 * This class contains implementation of a dual conic
 */
public class DualConic extends BaseConic implements Serializable {
    
    /**
     * Constructor
     */
    public DualConic(){
        super();
    }
    
    /**
     * Constructor of this class. This constructor accepts every parameter
     * describing a dual conic (parameters a, b, c, d, e, f)
     * @param a Parameter A of the conic.
     * @param b Parameter B of the conic.
     * @param c Parameter C of the conic.
     * @param d Parameter D of the conic.
     * @param e Parameter E of the conic.
     * @param f Parameter F of the conic.
     */
    public DualConic(double a, double b, double c, double d, double e, 
            double f){
        super(a, b, c, d, e, f);
    }
    
    /**
     * This method sets the matrix used to describe a dual conic.
     * This matrix must be 3x3 and symmetric.
     * @param m 3x3 Matrix describing the conic.
     * @throws IllegalArgumentException Raised when the size of the matrix is 
     * not 3x3
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not 
     * symmetric
     */
    public DualConic(Matrix m) throws IllegalArgumentException, 
            NonSymmetricMatrixException {
        super(m);
    }
    
    /**
     * Instantiates a dual conic where provided lines belong to its locus
     * @param line1 1st line
     * @param line2 2nd line
     * @param line3 3rd line
     * @param line4 4th line
     * @param line5 5th line
     * @throws CoincidentLinesException Raised if provided lines are coincident
     * (more than one line is equal) or produce a degenerate configuration.
     */
    public DualConic(Line2D line1, Line2D line2, Line2D line3, Line2D line4, Line2D line5)
            throws CoincidentLinesException{
        setParametersFromLines(line1, line2, line3, line4, line5);
    }
    
    /**
     * Checks if provided line is locus of this dual conic, or in other words,
     * checks whether provided line lies within this conic, or whether provided
     * line is tangent to the conic corresponding to this dual conic.
     * @param line Line2D to be tested
     * @param threshold Threshold of tolerance to determine whether the line is
     * locus or not. This is needed because of limited machine precision. If 
     * threshold is not provided, then DEFAULT_LOCUS_THRESHOLD is used instead.
     * @return True if provided line is locus of this dual conic, false 
     * otherwise
     * @throws IllegalArgumentException Raised if provided threshold is negative
     */
    public boolean isLocus(Line2D line, double threshold) 
            throws IllegalArgumentException {
        
        if(threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }
                
        try {
            normalize();
            Matrix dualC = asMatrix();
            Matrix homLine = 
                    new Matrix(Line2D.LINE_NUMBER_PARAMS, 1);
            line.normalize();
            homLine.setElementAt(0, 0, line.getA());
            homLine.setElementAt(1, 0, line.getB());
            homLine.setElementAt(2, 0, line.getC());
            Matrix locusMatrix = homLine.transposeAndReturnNew();
            locusMatrix.multiply(dualC);
            locusMatrix.multiply(homLine);
                
            return Math.abs(locusMatrix.getElementAt(0, 0)) < threshold;
        } catch (WrongSizeException ignore) {
            return false;
        }
    }
    
    /**
     * Checks if provided line is locus of this dual conic, or in other words,
     * checks whether provided line lies within this conic, or whether provided
     * line is tangent to the conic corresponding to this dual conic.
     * @param line Line2D to be tested
     * @return True if provided line is locus of this dual conic, false 
     * otherwise
     * @see #isLocus(Line2D, double)
     */    
    public boolean isLocus(Line2D line) {
        return isLocus(line, DEFAULT_LOCUS_THRESHOLD);
    }
    
    /**
     * Computes the angle between two lines in radians
     * @param lineA First line to be tested
     * @param lineB Second line to be tested
     * @return Angle between the two provided lines in radians.
     */
    public double angleBetweenLines(Line2D lineA, Line2D lineB) {
        try {
            //retrieve conic as matrix
            normalize();
            Matrix dualC = asMatrix();
            Matrix transHomLineA = new Matrix(1, Line2D.LINE_NUMBER_PARAMS);
            lineA.normalize();
            transHomLineA.setElementAt(0, 0, lineA.getA());
            transHomLineA.setElementAt(0, 1, lineA.getB());
            transHomLineA.setElementAt(0, 2, lineA.getC());
        
        
            Matrix tmp = transHomLineA.multiplyAndReturnNew(dualC);
            tmp.multiply(transHomLineA.transposeAndReturnNew()); //This is 
                                                //homLineA' * dualC * homLineA
        
            double normA = tmp.getElementAt(0, 0);
        
            Matrix homLineB = new Matrix(Line2D.LINE_NUMBER_PARAMS, 1);
            lineB.normalize();
            homLineB.setElementAt(0, 0, lineB.getA());
            homLineB.setElementAt(1, 0, lineB.getB());
            homLineB.setElementAt(2, 0, lineB.getC());
        
            homLineB.transpose(tmp);
            tmp.multiply(dualC);
            tmp.multiply(homLineB);
        
            double normB = tmp.getElementAt(0, 0);
        
            transHomLineA.multiply(dualC);
            transHomLineA.multiply(homLineB); 
                //This is homLineA' * dualC * homLineB
                        
            double angleNumerator = transHomLineA.getElementAt(0, 0);

            double cosTheta = angleNumerator / Math.sqrt(normA * normB);
            return Math.acos(cosTheta);
        } catch (WrongSizeException ignore) {
            return 0.0; //This will never happen
        }
    }
    
    /**
     * Checks if two lines are perpendicular attending to the geometry defined 
     * by this dual conic, or in other words, if lA' * dualC* * lB is zero
     * @param lineA First line to be checked
     * @param lineB Second line to be checked
     * @param threshold Threshold of tolerance to determine whether the lines
     * are perpendicular or not. This is needed because of limited machine 
     * precision. If threshold is not provided, then 
     * DEFAULT_PERPENDICULAR_THRESHOLD is used instead.
     * @return True if provided lines are perpendicular, false otherwise
     * @throws IllegalArgumentException Raised if provided threshold is negative
     */
    public boolean arePerpendicularLines(Line2D lineA, Line2D lineB, 
            double threshold) throws IllegalArgumentException {
        try {
            //retrieve conic as matrix
            Matrix transHomLineA = new Matrix(1, Line2D.LINE_NUMBER_PARAMS);
            lineA.normalize();
            transHomLineA.setElementAt(0, 0, lineA.getA());
            transHomLineA.setElementAt(0, 1, lineA.getB());
            transHomLineA.setElementAt(0, 2, lineA.getC());
                        
            Matrix homLineB =
                    new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 
                    1);
            lineB.normalize();
            homLineB.setElementAt(0, 0, lineB.getA());
            homLineB.setElementAt(1, 0, lineB.getB());
            homLineB.setElementAt(2, 0, lineB.getC());
            
            normalize();
            Matrix dualC = asMatrix();
            transHomLineA.multiply(dualC);
            transHomLineA.multiply(homLineB); 
                //This is homLineA' * dualC * homLineB
            
            double perpend = transHomLineA.getElementAt(0, 0);
        
            return Math.abs(perpend) < threshold;
        } catch (WrongSizeException ignore) {
            return false; //This will never happen
        }  
    }

    /**
     * Checks if two lines are perpendicular attending to the geometry defined 
     * by this dual conic, or in other words, if lA' * dualC* * lB is zero
     * @param lineA First line to be checked
     * @param lineB Second line to be checked
     * @return True if provided lines are perpendicular, false otherwise
     */    
    public boolean arePerpendicularLines(Line2D lineA, Line2D lineB) {
        return arePerpendicularLines(lineA, lineB, 
                DEFAULT_PERPENDICULAR_THRESHOLD);
    }
    
    /**
     * Computes the conic corresponding to this dual conic.
     * @return A new conic instance of this dual conic.
     * @throws ConicNotAvailableException Raised if the rank of the dual conic 
     * matrix is not complete due to wrong parameters or numerical instability.
     */
    public Conic getConic() throws ConicNotAvailableException {
        Conic c = new Conic();
        conic(c);
        return c;
    }
    
    /**
     * Computes the conic corresponding to this dual conic and stores the result
     * in provided instance
     * @param conic Conic where result is stored.
     * @throws ConicNotAvailableException Raised if the rank of the dual conic
     * matrix is not complete due to wrong parameters or numerical instability.
     */
    public void conic(Conic conic) throws ConicNotAvailableException {
        
        Matrix dualConicMatrix = asMatrix();
        try {
            Matrix invMatrix = com.irurueta.algebra.Utils.inverse(
                    dualConicMatrix);
            
            //ensure that resulting matrix after inversion is symmetric
            //by computing the mean of off-diagonal elements
            double a = invMatrix.getElementAt(0, 0);
            double b = 0.5 * (invMatrix.getElementAt(0, 1) + 
                    invMatrix.getElementAt(1, 0));
            double c = invMatrix.getElementAt(1, 1);
            double d = 0.5 * (invMatrix.getElementAt(0, 2) + 
                    invMatrix.getElementAt(2, 0));
            double e = 0.5 * (invMatrix.getElementAt(1, 2) + 
                    invMatrix.getElementAt(2, 1));
            double f = invMatrix.getElementAt(2, 2);
            conic.setParameters(a, b, c, d, e, f);       
        } catch (AlgebraException e) {
            throw new ConicNotAvailableException(e);
        }
    }
    
    /**
     * Sets parameters of this dual conic so that provided lines lie within it 
     * (are locus)
     * @param line1 1st line
     * @param line2 2nd line
     * @param line3 3rd line
     * @param line4 4th line
     * @param line5 5th line
     * @throws CoincidentLinesException Raised if lines are coincident or 
     * produce a degenerated configuration
     */
    public final void setParametersFromLines(Line2D line1, Line2D line2, Line2D line3,
            Line2D line4, Line2D line5) throws CoincidentLinesException {
        
        try {
            line1.normalize();
            line2.normalize();
            line3.normalize();
            line4.normalize();
            line5.normalize();
                
        
            //estimate dual conic that lines inside of provided 5 lines
            Matrix m = new Matrix(5, 6);
                    
            double l1 = line1.getA();
            double l2 = line1.getB();
            double l3 = line1.getC();
            m.setElementAt(0, 0, l1 * l1);
            m.setElementAt(0, 1, 2.0 * l1 * l2);
            m.setElementAt(0, 2, l2 * l2);
            m.setElementAt(0, 3, 2.0 * l1 * l3);
            m.setElementAt(0, 4, 2.0 * l2 * l3);
            m.setElementAt(0, 5, l3 * l3);
        
            l1 = line2.getA();
            l2 = line2.getB();
            l3 = line2.getC();
            m.setElementAt(1, 0, l1 * l1);
            m.setElementAt(1, 1, 2.0 * l1 * l2);
            m.setElementAt(1, 2, l2 * l2);
            m.setElementAt(1, 3, 2.0 * l1 * l3);
            m.setElementAt(1, 4, 2.0 * l2 * l3);
            m.setElementAt(1, 5, l3 * l3);

            l1 = line3.getA();
            l2 = line3.getB();
            l3 = line3.getC();
            m.setElementAt(2, 0, l1 * l1);
            m.setElementAt(2, 1, 2.0 * l1 * l2);
            m.setElementAt(2, 2, l2 * l2);
            m.setElementAt(2, 3, 2.0 * l1 * l3);
            m.setElementAt(2, 4, 2.0 * l2 * l3);
            m.setElementAt(2, 5, l3 * l3);

            l1 = line4.getA();
            l2 = line4.getB();
            l3 = line4.getC();
            m.setElementAt(3, 0, l1 * l1);
            m.setElementAt(3, 1, 2.0 * l1 * l2);
            m.setElementAt(3, 2, l2 * l2);
            m.setElementAt(3, 3, 2.0 * l1 * l3);
            m.setElementAt(3, 4, 2.0 * l2 * l3);
            m.setElementAt(3, 5, l3 * l3);       

            l1 = line5.getA();
            l2 = line5.getB();
            l3 = line5.getC();
            m.setElementAt(4, 0, l1 * l1);
            m.setElementAt(4, 1, 2.0 * l1 * l2);
            m.setElementAt(4, 2, l2 * l2);
            m.setElementAt(4, 3, 2.0 * l1 * l3);
            m.setElementAt(4, 4, 2.0 * l2 * l3);
            m.setElementAt(4, 5, l3 * l3);
            
            //normalize each row to increase accuracy
            double[] row = new double[6];
            double rowNorm;
            
            for(int j = 0; j < 5; j++) {
                m.getSubmatrixAsArray(j, 0, j, 5, row);
                rowNorm = com.irurueta.algebra.Utils.normF(row);
                for(int i = 0; i < 6; i++) {
                    m.setElementAt(j, i, m.getElementAt(j, i) / rowNorm);
                }
            }            
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
            
            if(decomposer.getRank() < 5) {
                throw new CoincidentLinesException();
            }
            
            //the right null-space of m contains the parameters a, b, c, d, e ,f
            //of the conic
            Matrix V = decomposer.getV();
            
            //l1^ + 2*l1*l2 + l2^2 + 2*l1*l3 + 2*l2*l3 + l3^2 = 0            
            double a = V.getElementAt(0, 5);
            double b = V.getElementAt(1, 5);
            double c = V.getElementAt(2, 5);
            double d = V.getElementAt(3, 5);
            double e = V.getElementAt(4, 5);
            double f = V.getElementAt(5, 5);
                            
            setParameters(a, b, c, d, e, f);
        }catch(AlgebraException ex) {
            throw new CoincidentLinesException(ex);
        }
    }
    
    /**
     * Creates a canonical instance of the dual absolute conic in the metric
     * stratum.
     * The intersection of the plane at infinity with the set of planes tangent
     * to the dual absolute quadric produce the dual absolute conic.
     * In other words, The dual absolute conic in the metric stratum is the set 
     * of lines tangent to the absolute conic that also lie in the plane at 
     * infinity.
     * Both the absolute conic and the dual absolute conic define orthogonality
     * in the metric stratum, and in a purely metric stratum (i.e. when camera 
     * is correctly calibrated), their canonical value is equal to the identity
     * @return a canonical instance of the dual absolute conic
     */
    public static DualConic createCanonicalDualAbsoluteConic() {
        return new DualConic(1.0, 0.0, 1.0, 0.0, 0.0, 1.0);
    }    
}
