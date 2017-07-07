/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.Quadric
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date August 13, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.*;
import java.io.Serializable;

/**
 * This class contains the implementation of a quadric
 */
public class Quadric extends BaseQuadric implements Serializable{
    
    /**
     * Constructor
     */
    public Quadric(){
        super();
    }
    
    /**
     * Constructor of this class. This constructor accepts every parameter
     * describing a quadric (parameters a, b, c, d, e, f, g, h, i, j
     * @param a Parameter A of the quadric
     * @param b Parameter B of the quadric
     * @param c Parameter C of the quadric
     * @param d Parameter D of the quadric
     * @param e Parameter E of the quadric
     * @param f Parameter F of the quadric
     * @param g Parameter G of the quadric
     * @param h Parameter H of the quadric
     * @param i Parameter I of the quadric
     * @param j Parameter J of the quadric
     */
    public Quadric(double a, double b, double c, double d, double e, double f,
            double g, double h, double i, double j){
        super(a, b, c, d, e, f, g, h, i, j);
    }
    
    /**
     * Constructor of this class. This constructor accepts a Matrix describing
     * a quadric
     * @param m Matrix describing a quadric 4x4 Matrix describing the quadric
     * @throws IllegalArgumentException Raised when the size of the matrix is 
     * not 4x4
     * @throws NonSymmetricMatrixException Raised when the quadric matrix is not
     * symmetric
     */
    public Quadric(Matrix m) throws IllegalArgumentException,
            NonSymmetricMatrixException{
        super(m);
    }
    
    /**
     * Creates quadric where provided points are contained (are locus)
     * @param point1 1st point
     * @param point2 2nd point
     * @param point3 3rd point
     * @param point4 4th point
     * @param point5 5th point
     * @param point6 6th point
     * @param point7 7th point
     * @param point8 8th point
     * @param point9 9th point
     * @throws CoincidentPointsException Raised if points are coincident or
     * produce a degenerated configuration
     */
    public Quadric(Point3D point1, Point3D point2, Point3D point3, 
            Point3D point4, Point3D point5, Point3D point6, Point3D point7,
            Point3D point8, Point3D point9) throws CoincidentPointsException{
        setParametersFromPoints(point1, point2, point3, point4, point5, point6,
                point7, point8, point9);
    }
    
    /**
     * Checks if the given point is locus (lies within) this quadric.
     * @param point Point to be checked.
     * @param threshold Threshold of distance to determine whether the point
     * is locus of the quadric or not. Threshold might be needed because of 
     * machine precision. If not provided DEFAULT_LOCUS_THRESHOLD will be used
     * instead.
     * @return True if the point lies within this quadric, false otherwise
     * @throws IllegalArgumentException Raised if threshold is negative
     */
    public boolean isLocus(Point3D point, double threshold)
            throws IllegalArgumentException{
        
        if(threshold < MIN_THRESHOLD) throw new IllegalArgumentException();
        
        try{
            normalize();
            Matrix Q = asMatrix();
            Matrix homPoint = new Matrix(
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            point.normalize();
            homPoint.setElementAt(0, 0, point.getHomX());
            homPoint.setElementAt(1, 0, point.getHomY());
            homPoint.setElementAt(2, 0, point.getHomZ());
            homPoint.setElementAt(3, 0, point.getHomW());
            Matrix locusMatrix = homPoint.transposeAndReturnNew();
            locusMatrix.multiply(Q);
            locusMatrix.multiply(homPoint);
            
            return Math.abs(locusMatrix.getElementAt(0, 0)) < threshold;
        }catch(WrongSizeException ignore){
            return false;
        }
    }
    
    /**
     * Checks if the given point is locus (lies within) this quadric.
     * @param point Point to be checked.
     * @return True if the point lies within this conic, false otherwise
     * @see #isLocus(Point3D, double)
     */
    public boolean isLocus(Point3D point){
        return isLocus(point, DEFAULT_LOCUS_THRESHOLD);
    }
    
    /**
     * Computes the angle between two 3D points using this quadric as a geometry
     * base.
     * @param pointA First point
     * @param pointB Second point
     * @return Angle between provided points given in radians.
     */
    public double angleBetweenPoints(Point3D pointA, Point3D pointB){
        try{
            //retrieve quadric as matrix
            Matrix Q = asMatrix();
            Matrix transHomPointA = new Matrix(1, 
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH);
            pointA.normalize();
            transHomPointA.setElementAt(0, 0, pointA.getHomX());
            transHomPointA.setElementAt(0, 1, pointA.getHomY());
            transHomPointA.setElementAt(0, 2, pointA.getHomZ());
            transHomPointA.setElementAt(0, 3, pointA.getHomW());
            
            Matrix tmp = transHomPointA.multiplyAndReturnNew(Q);
            tmp.multiply(transHomPointA.transposeAndReturnNew()); //This is
                                                //homPointA' * Q * homPointA
            
            double normA = tmp.getElementAt(0, 0);
            
            Matrix homPointB = new Matrix(
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            pointB.normalize();
            homPointB.setElementAt(0, 0, pointB.getHomX());
            homPointB.setElementAt(1, 0, pointB.getHomY());
            homPointB.setElementAt(2, 0, pointB.getHomZ());
            homPointB.setElementAt(3, 0, pointB.getHomW());
            
            homPointB.transpose(tmp);
            tmp.multiply(Q);
            tmp.multiply(homPointB);
            
            double normB = tmp.getElementAt(0, 0);
            
            transHomPointA.multiply(Q);
            transHomPointA.multiply(homPointB);
            //This is homPointA' * Q * homPointB
            
            double angleNumerator = transHomPointA.getElementAt(0, 0);
            
            double cosTheta = angleNumerator / Math.sqrt(normA * normB);
            return Math.acos(cosTheta);
        }catch(WrongSizeException ignore){
            return 0.0; //This will never happen
        }
    }
    
    /**
     * Checks if two points are perpendicular in the geometry base generated by
     * this quadric.
     * @param pointA First point
     * @param pointB Second point
     * @param threshold Threshold to determine whether the points are 
     * perpendicular or not. If the dot product between provided points and this
     * quadric is greater than provided threshold, then points won't be assumed
     * to be perpendicular. Threshold is provided because of machine precision
     * limits, if not provided DEFAULT_PERPENDICULAR_THRESHOLD will be used
     * instead.
     * @return True if points are perpendicular, false otherwise.
     * @throws IllegalArgumentException Raised if threshold is negative.
     */
    public boolean arePerpendicularPoints(Point3D pointA, Point3D pointB,
            double threshold) throws IllegalArgumentException{
        
        try{
            //retrieve quadric as matrix
            Matrix transHomPointA = new Matrix(1, 
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH);
            pointA.normalize();
            transHomPointA.setElementAt(0, 0, pointA.getHomX());
            transHomPointA.setElementAt(0, 1, pointA.getHomY());
            transHomPointA.setElementAt(0, 2, pointA.getHomZ());
            transHomPointA.setElementAt(0, 3, pointA.getHomW());
            
            Matrix homPointB = new Matrix(
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            pointB.normalize();
            homPointB.setElementAt(0, 0, pointB.getHomX());
            homPointB.setElementAt(1, 0, pointB.getHomY());
            homPointB.setElementAt(2, 0, pointB.getHomZ());
            homPointB.setElementAt(3, 0, pointB.getHomW());
            
            normalize();
            Matrix Q = asMatrix();
            transHomPointA.multiply(Q);
            transHomPointA.multiply(homPointB);
                //This is homPointA' * Q * homPointB
            
            double perpend = transHomPointA.getElementAt(0, 0);
            
            return Math.abs(perpend) < threshold;
        }catch(WrongSizeException ignore){
            return false; //This will never happen
        }
    }
    
    /**
     * Checks if two points are perpendicular in the geometry base generated by
     * this quadric.
     * @param pointA First point
     * @param pointB Second point
     * @return True if points are perpendicular, false otherwise.
     * @see #arePerpendicularPoints(Point3D, Point3D, double)
     */
    public boolean arePerpendicularPoints(Point3D pointA, Point3D pointB){
        return arePerpendicularPoints(pointA, pointB, 
                DEFAULT_PERPENDICULAR_THRESHOLD);
    }
    
    /**
     * Sets the values of the dual quadric corresponding to this quadric 
     * instance into provided dualQuadric instance.
     * The dual quadric is equal to the inverse of the quadric matrix.
     * @param dualQuadric Dual quadric instance where the values of the dual 
     * quadric of this quadric instance will be stored.
     * @throws DualQuadricNotAvailableException Raised if the dual quadric does 
     * not exists because this quadric instance is degenerate (its inverse 
     * cannot be computed)
     */
    public void dualQuadric(DualQuadric dualQuadric) 
            throws DualQuadricNotAvailableException{
        
        Matrix quadricMatrix = asMatrix();
        try{
            Matrix invMatrix = com.irurueta.algebra.Utils.inverse(quadricMatrix);
            
            //ensure that resulting matrix after inversion is symmetric
            //by computing the mean of off-diagonal elements
            double a = invMatrix.getElementAt(0, 0);
            double b = invMatrix.getElementAt(1, 1);
            double c = invMatrix.getElementAt(2, 2);
            double d = 0.5 * (invMatrix.getElementAt(0, 1) + 
                    invMatrix.getElementAt(1, 0));
            double e = 0.5 * (invMatrix.getElementAt(2, 1) +
                    invMatrix.getElementAt(1, 2));
            double f = 0.5 * (invMatrix.getElementAt(2, 0) +
                    invMatrix.getElementAt(0, 2));
            double g = 0.5 * (invMatrix.getElementAt(3, 0) +
                    invMatrix.getElementAt(0, 3));
            double h = 0.5 * (invMatrix.getElementAt(3, 1) +
                    invMatrix.getElementAt(1, 3));
            double i = 0.5 * (invMatrix.getElementAt(3, 2) +
                    invMatrix.getElementAt(2, 3));
            double j = invMatrix.getElementAt(3, 3);
            dualQuadric.setParameters(a, b, c, d, e, f, g, h, i, j);
        }catch(AlgebraException e){
            throw new DualQuadricNotAvailableException(e);
        }
    }
    
    /**
     * Computes the dual quadric of this quadric.
     * The dual quadric is equal to the inverse of the quadric matrix
     * @return A new DualQuadric corresponding to the dual quadric of this 
     * instance.
     * @throws DualQuadricNotAvailableException Raised if the dual quadric does
     * not exist because this quadric instance is degenerate (its inverse cannot
     * be computed)
     */
    public DualQuadric getDualQuadric() throws DualQuadricNotAvailableException{
        DualQuadric dualQuadric = new DualQuadric();
        dualQuadric(dualQuadric);
        return dualQuadric;
    }
    
    /**
     * Sets parameters of this quadric so that provided points lie within it 
     * (are locus)
     * @param point1 1st point
     * @param point2 2nd point
     * @param point3 3rd point
     * @param point4 4th point
     * @param point5 5th point
     * @param point6 6th point
     * @param point7 7th point
     * @param point8 8th point
     * @param point9 9th point
     * @throws CoincidentPointsException Raisef if points are coincident or
     * produce a degenerated configuration
     */
    public final void setParametersFromPoints(Point3D point1, Point3D point2,
            Point3D point3, Point3D point4, Point3D point5, Point3D point6,
            Point3D point7, Point3D point8, Point3D point9) 
            throws CoincidentPointsException{
        
        //normalize points to increase accuracy
        point1.normalize();
        point2.normalize();
        point3.normalize();
        point4.normalize();
        point5.normalize();
        point6.normalize();
        point7.normalize();
        point8.normalize();
        point9.normalize();
        
        try{
            //each point belonging to a quadric follows equation:
            //p' * Q * p = 0 ==>
            //x^2 + y^2 + z^2 + 2*x*y + 2*x*z + 2*y*z + 2*x*w + 2*y*w + 
            //2*z*w + w^2 = 0            
            
            Matrix m = new Matrix(9, 10);
            double x = point1.getHomX();
            double y = point1.getHomY();
            double z = point1.getHomZ();
            double w = point1.getHomW();
            m.setElementAt(0, 0, x * x);
            m.setElementAt(0, 1, y * y);
            m.setElementAt(0, 2, z * z);
            m.setElementAt(0, 3, 2.0 * x * y);
            m.setElementAt(0, 4, 2.0 * x * z);
            m.setElementAt(0, 5, 2.0 * y * z);
            m.setElementAt(0, 6, 2.0 * x * w);
            m.setElementAt(0, 7, 2.0 * y * w);
            m.setElementAt(0, 8, 2.0 * z * w);
            m.setElementAt(0, 9, w * w);
            x = point2.getHomX();
            y = point2.getHomY();
            z = point2.getHomZ();
            w = point2.getHomW();
            m.setElementAt(1, 0, x * x);
            m.setElementAt(1, 1, y * y);
            m.setElementAt(1, 2, z * z);
            m.setElementAt(1, 3, 2.0 * x * y);
            m.setElementAt(1, 4, 2.0 * x * z);
            m.setElementAt(1, 5, 2.0 * y * z);
            m.setElementAt(1, 6, 2.0 * x * w);
            m.setElementAt(1, 7, 2.0 * y * w);
            m.setElementAt(1, 8, 2.0 * z * w);
            m.setElementAt(1, 9, w * w);
            x = point3.getHomX();
            y = point3.getHomY();
            z = point3.getHomZ();
            w = point3.getHomW();
            m.setElementAt(2, 0, x * x);
            m.setElementAt(2, 1, y * y);
            m.setElementAt(2, 2, z * z);
            m.setElementAt(2, 3, 2.0 * x * y);
            m.setElementAt(2, 4, 2.0 * x * z);
            m.setElementAt(2, 5, 2.0 * y * z);
            m.setElementAt(2, 6, 2.0 * x * w);
            m.setElementAt(2, 7, 2.0 * y * w);
            m.setElementAt(2, 8, 2.0 * z * w);
            m.setElementAt(2, 9, w * w);
            x = point4.getHomX();
            y = point4.getHomY();
            z = point4.getHomZ();
            w = point4.getHomW();
            m.setElementAt(3, 0, x * x);
            m.setElementAt(3, 1, y * y);
            m.setElementAt(3, 2, z * z);
            m.setElementAt(3, 3, 2.0 * x * y);
            m.setElementAt(3, 4, 2.0 * x * z);
            m.setElementAt(3, 5, 2.0 * y * z);
            m.setElementAt(3, 6, 2.0 * x * w);
            m.setElementAt(3, 7, 2.0 * y * w);
            m.setElementAt(3, 8, 2.0 * z * w);
            m.setElementAt(3, 9, w * w);
            x = point5.getHomX();
            y = point5.getHomY();
            z = point5.getHomZ();
            w = point5.getHomW();
            m.setElementAt(4, 0, x * x);
            m.setElementAt(4, 1, y * y);
            m.setElementAt(4, 2, z * z);
            m.setElementAt(4, 3, 2.0 * x * y);
            m.setElementAt(4, 4, 2.0 * x * z);
            m.setElementAt(4, 5, 2.0 * y * z);
            m.setElementAt(4, 6, 2.0 * x * w);
            m.setElementAt(4, 7, 2.0 * y * w);
            m.setElementAt(4, 8, 2.0 * z * w);
            m.setElementAt(4, 9, w * w);
            x = point6.getHomX();
            y = point6.getHomY();
            z = point6.getHomZ();
            w = point6.getHomW();
            m.setElementAt(5, 0, x * x);
            m.setElementAt(5, 1, y * y);
            m.setElementAt(5, 2, z * z);
            m.setElementAt(5, 3, 2.0 * x * y);
            m.setElementAt(5, 4, 2.0 * x * z);
            m.setElementAt(5, 5, 2.0 * y * z);
            m.setElementAt(5, 6, 2.0 * x * w);
            m.setElementAt(5, 7, 2.0 * y * w);
            m.setElementAt(5, 8, 2.0 * z * w);
            m.setElementAt(5, 9, w * w);
            x = point7.getHomX();
            y = point7.getHomY();
            z = point7.getHomZ();
            w = point7.getHomW();
            m.setElementAt(6, 0, x * x);
            m.setElementAt(6, 1, y * y);
            m.setElementAt(6, 2, z * z);
            m.setElementAt(6, 3, 2.0 * x * y);
            m.setElementAt(6, 4, 2.0 * x * z);
            m.setElementAt(6, 5, 2.0 * y * z);
            m.setElementAt(6, 6, 2.0 * x * w);
            m.setElementAt(6, 7, 2.0 * y * w);
            m.setElementAt(6, 8, 2.0 * z * w);
            m.setElementAt(6, 9, w * w);
            x = point8.getHomX();
            y = point8.getHomY();
            z = point8.getHomZ();
            w = point8.getHomW();
            m.setElementAt(7, 0, x * x);
            m.setElementAt(7, 1, y * y);
            m.setElementAt(7, 2, z * z);
            m.setElementAt(7, 3, 2.0 * x * y);
            m.setElementAt(7, 4, 2.0 * x * z);
            m.setElementAt(7, 5, 2.0 * y * z);
            m.setElementAt(7, 6, 2.0 * x * w);
            m.setElementAt(7, 7, 2.0 * y * w);
            m.setElementAt(7, 8, 2.0 * z * w);
            m.setElementAt(7, 9, w * w);
            x = point9.getHomX();
            y = point9.getHomY();
            z = point9.getHomZ();
            w = point9.getHomW();
            m.setElementAt(8, 0, x * x);
            m.setElementAt(8, 1, y * y);
            m.setElementAt(8, 2, z * z);
            m.setElementAt(8, 3, 2.0 * x * y);
            m.setElementAt(8, 4, 2.0 * x * z);
            m.setElementAt(8, 5, 2.0 * y * z);
            m.setElementAt(8, 6, 2.0 * x * w);
            m.setElementAt(8, 7, 2.0 * y * w);
            m.setElementAt(8, 8, 2.0 * z * w);
            m.setElementAt(8, 9, w * w);
            
            //normalize each row to increase accuracy
            double[] row = new double[10];
            double rowNorm;
                        
            for(int j = 0; j < 9; j++){
                m.getSubmatrixAsArray(j, 0, j, 9, row);
                rowNorm = com.irurueta.algebra.Utils.normF(row);
                for(int i = 0; i < 10; i++) 
                    m.setElementAt(j, i, m.getElementAt(j, i) / rowNorm);
            }     
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
            
            if(decomposer.getRank() < 9) throw new CoincidentPointsException();
            
            //the right null-space of m contains the parameters a, b, c, d, e ,f
            //of the conic
            Matrix V = decomposer.getV();                        
            
            double a = V.getElementAt(0, 9);
            double b = V.getElementAt(1, 9);
            double c = V.getElementAt(2, 9);
            double d = V.getElementAt(3, 9);
            
            double f = V.getElementAt(4, 9);
            double e = V.getElementAt(5, 9);            
            
            double g = V.getElementAt(6, 9);
            double h = V.getElementAt(7, 9);
            double i = V.getElementAt(8, 9);
            double j = V.getElementAt(9, 9);            
            
            setParameters(a, b, c, d, e, f, g, h, i, j);            
        }catch(AlgebraException ex){
            throw new CoincidentPointsException(ex);
        }
    }
    
    /**
     * Returns a plane tangent to this quadric at provided point, as long as
     * the provided point is locus of this quadric
     * @param point point where plane must be tangent to quadric
     * @return a plane tangent to this quadric
     * @throws NotLocusException if provided point is not locus of this quadric
     */
    public Plane getTangentPlaneAt(Point3D point) throws NotLocusException{
        return getTangentPlaneAt(point, DEFAULT_LOCUS_THRESHOLD);
    }
    
    /**
     * Returns a plane tangent to this quadric at provided point, as long as the
     * provided point is locus of this quadric up to provided threshold
     * @param point point where plane must be tangent to quadric
     * @param threshold threshold to determine if provided point is locus or not
     * of this quadric. Usually this is a small value close to zero
     * @return a plane tangent to this quadric
     * @throws NotLocusException if provided point is not locus of this quadric
     */
    public Plane getTangentPlaneAt(Point3D point, double threshold) 
            throws NotLocusException{
        Plane plane = new Plane();
        tangentPlaneAt(point, plane, threshold);
        return plane;
    }
    
    /**
     * Computes a plane tangent to this quadric at provided point, as long as 
     * the provided point is locus of this quadric up to provided threshold.
     * @param point point where plane must be tangent to quadric
     * @param plane plane where computed result will be stored
     * @param threshold threshold to determine if provided point is locus or not
     * of this quadric. Usually this is a small value close to zero
     * @throws NotLocusException  if provided point is not locus of this quadric
     * @throws IllegalArgumentException if provided threshold is negtive
     */
    public void tangentPlaneAt(Point3D point, Plane plane, double threshold)
            throws NotLocusException, IllegalArgumentException{
        
        if(!isLocus(point, threshold)) throw new NotLocusException();
        
        point.normalize();
        normalize();
        
        Matrix Q = asMatrix();
        
        try{
            Matrix p = new Matrix(
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            p.setElementAt(0, 0, point.getHomX());
            p.setElementAt(1, 0, point.getHomY());
            p.setElementAt(2, 0, point.getHomZ());
            p.setElementAt(3, 0, point.getHomW());
            
            Q.multiply(p);
        }catch(WrongSizeException ignore){}
        
        plane.setParameters(Q.getElementAt(0, 0), Q.getElementAt(1, 0), 
                Q.getElementAt(2, 0), Q.getElementAt(3, 0));
        //TODO: must be tested
    }
    
    /**
     * Intersects this quadric with provided plane.
     * @param plane plane to intersect this quadric with.
     * @param result instance where resulting intersection will be stored.
     */
    public void intersectWith(Plane plane, Conic result) {
        //A plane follows expression: A*x + B*y + C*z + D*w = 0
        
        //A quadric has the following matrix form:
        //Q =   [A  D   F   G]
        //      [D  B   E   H]
        //      [F  E   C   I]
        //      [G  H   I   J]
        //[x y z w][A D F G][x] = [x y z w][A*x + D*y + F*z + G*w] = 
        //         [D B E H][y]            [D*x + B*y + E*z + H*w]
        //         [F E C I][z]            [F*x + E*y + C*z + I*w]
        //         [G H I J][w]            [G*x + H*y + I*z + J*w]
        //= A*x^2 + D*x*y + F*x*z + G*x*w + D*x*y + B*y^2 + E*y*z + H*y*w + 
        //F*x*z + E*y*z + C*z^2 + I*z*w + G*x*w + H*y*w + I*z*w + J*w^2 =
        //= A*x^2 + B*y^2 + C*z^2 + 2*D*x*y + 2*E*y*z + 2*F*x*z + 2*G*x*w + 2*H*y*w + 2*I*z*w + J*w^2
        //which follows expression: 
        //A*x^2 + B*y^2 + C*z^2 + 2*D*x*y + 2*E*y*z + 2*F*x*z + 2*G*x*w + 2*H*y*w + 2*I*z*w + J*w^2 = 0
        
        //A conic has the following matrix form:
        //C = [A    B   D]
        //    [B    C   E]
        //    [D    E   F]
        //[x y w][A B D][x] = [x y w][A*x + B*y + D*w] = 
        //       [B C E][y]          [B*x + C*y + E*w]
        //       [D E F][w]          [D*x + E*y + F*w]
        //= A*x^2 + B*x*y + D*x*w + B*x*y + C*y^2 + E*y*w + D*x*w + E*y*w + F*w^2 =
        //= A*x^2 + 2*B*x*y + C*y^2 + 2*D*x*w + 2*E*y*w + F*w^2
        //which follows expression:
        //A*x^2 + 2*B*x*y + C*y^2 + 2*D*x*w + 2*E*y*w + F*w^2 = 0
        
        //Quadric parameters
        double aQ = getA(), bQ = getB(), cQ = getC(), dQ = getD(), eQ = getE(),
                fQ = getF(), gQ = getG(), hQ = getH(), iQ = getI(), jQ = getJ();
        
        //Plane parameters
        double aP = plane.getA(), bP = plane.getB(), cP = plane.getC(),
                dP = plane.getD();
        
        //we solve the following system of equations:
        //aQ*x^2 + bQ*y^2 + cQ*z^2 + 2*dQ*x*y + 2*eQ*y*z + 2*fQ*x*z + 2*gQ*x*w + 2*hQ*y*w + 2*iQ*z*w + jQ*w^2 = 0
        //aP*x + bP*y + cP*z + dP*w = 0
        
        //Isolating z in plane equation:
        //z = (- aP*x - bP*y - dP*w)/cP
        
        //and substituting in quadric equation:
        //aQ*x^2 + bQ*y^2 + cQ*(- aP*x - bP*y - dP*w)^2/cP^2 + 2*dQ*x*y + 
        //2*eQ*y*(- aP*x - bP*y - dP*w)/cP + 2*fQ*x*(- aP*x - bP*y - dP*w)/cP + 
        //2*gQ*x*w + 2*hQ*y*w + 2*iQ*(- aP*x - bP*y - dP*w)/cP*w + jQ*w^2 = 0
        
        //aQ*x^2 + bQ*y^2 + cQ*(- aP*x - bP*y - dP*w)^2/cP^2 + 2*dQ*x*y + 
        //-2*eQ*aP/cP*x*y -2*eQ*bP/cP*y^2 -2*eQ*dP/cP*y*w +
        //-2*fQ*aP/cP*x^2 -2*fQ*bP/cP*x*y -2*fQ*dP/cP*x*w +
        //2*gQ*x*w + 2*hQ*y*w +
        //-2*iQ*aP/cP*x*w -2*iQ*bP/cP*y*w -2*iQ*dP/cP*w^2 +
        //jQ*w^2 = 0

        //aQ*x^2 + bQ*y^2 + 
        //aP^2*cQ/cP^2*x^2 + 2*aP*bP*cQ/cP^2*x*y + 2*aP*cQ*dP/cP^2*x*w + 
        //bP^2*cQ/cP^2*y^2 + 2*bP*cQ*dP/cP^2*y*w + cQ*dP^2/cP^2*w^2 + 
        //2*dQ*x*y + 
        //-2*eQ*aP/cP*x*y -2*eQ*bP/cP*y^2 -2*eQ*dP/cP*y*w +
        //-2*fQ*aP/cP*x^2 -2*fQ*bP/cP*x*y -2*fQ*dP/cP*x*w +
        //2*gQ*x*w + 2*hQ*y*w +
        //-2*iQ*aP/cP*x*w -2*iQ*bP/cP*y*w -2*iQ*dP/cP*w^2 +
        //jQ*w^2 = 0

        //(aQ + aP^2*cQ/cP^2 -2*fQ*aP/cP)*x^2 + 
        //(2*aP*bP*cQ/cP^2 + 2*dQ -2*eQ*aP/cP -2*fQ*bP/cP)*x*y + 
        //(bQ + bP^2*cQ/cP^2 -2*eQ*bP/cP)*y^2 + 
        //(2*aP*cQ*dP/cP^2 -2*fQ*dP/cP + 2*gQ -2*iQ*aP/cP)*x*w + 
        //(2*bP*cQ*dP/cP^2 -2*eQ*dP/cP + 2*hQ -2*iQ*bP/cP)*y*w + 
        //(cQ*dP^2/cP^2 -2*iQ*dP/cP + jQ)*w^2 = 0
        
        //Comparing with conic equation:
        //aC*x^2 + 2*bC*x*y + cC*y^2 + 2*dC*x*w + 2*eC*y*w + fC*w^2 = 0
        
        //then conic parameters become:
        //aC = aQ + aP^2*cQ/cP^2 -2*fQ*aP/cP
        //bC = 2*aP*bP*cQ/cP^2 + 2*dQ -2*eQ*aP/cP -2*fQ*bP/cP
        //cC = bQ + bP^2*cQ/cP^2 -2*eQ*bP/cP
        //dC = 2*aP*cQ*dP/cP^2 -2*fQ*dP/cP + 2*gQ -2*iQ*aP/cP
        //eC = 2*bP*cQ*dP/cP^2 -2*eQ*dP/cP + 2*hQ -2*iQ*bP/cP
        //fC = cQ*dP^2/cP^2 -2*iQ*dP/cP + jQ
        
        double aC = aQ + Math.pow(aP,2.0)*cQ/Math.pow(cP,2.0) - 2.0*fQ*aP/cP;
        double bC = 2.0*aP*bP*cQ/Math.pow(cP,2.0) + 2.0*dQ - 2.0*eQ*aP/cP 
                - 2.0*fQ*bP/cP;
        double cC = bQ + Math.pow(bP,2.0)*cQ/Math.pow(cP,2.0) - 2.0*eQ*bP/cP;
        double dC = 2.0*aP*cQ*dP/Math.pow(cP,2.0) - 2.0*fQ*dP/cP + 2.0*gQ - 
                2.0*iQ*aP/cP;
        double eC = 2.0*bP*cQ*dP/Math.pow(cP,2.0) - 2.0*eQ*dP/cP + 2.0*hQ -
                2.0*iQ*bP/cP;
        double fC = cQ*Math.pow(dP,2.0)/Math.pow(cP,2.0) - 2.0*iQ*dP/cP + jQ;
        
        result.setParameters(aC, bC, cC, dC, eC, fC);
    }
    
    /**
     * Intersects this quadric with provided plane.
     * @param plane plane to intersect this quadric with.
     * @return conic resulting from the intersection.
     */
    public Conic intersectWith(Plane plane) {
        Conic result = new Conic();
        intersectWith(plane, result);
        return result;
    }
    
    //TODO: shorted distance of point to quadric
    //TODO: closest point to quadric
}
