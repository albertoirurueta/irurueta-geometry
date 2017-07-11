/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.Ellipse
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 17, 2017.
 */
package com.irurueta.geometry;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import java.io.Serializable;

/**
 * This class defines an ellipse.
 * This class uses formulas as defined at:
 * https://en.wikipedia.org/wiki/Ellipse
 */
public class Ellipse implements Serializable {
    
    /**
     * Constant defining default threshold value used when none is provided.
     */
    public static final double DEFAULT_THRESHOLD = 1e-9;
    
    /**
     * Constant defining minimum allowed threshold.
     */
    public static final double MIN_THRESHOLD = 0.0;    
    
    /**
     * Center of ellipse.
     */
    private Point2D mCenter;
    
    /**
     * Semi-major axis length (a).
     */
    private double mSemiMajorAxis;
    
    /**
     * Semi-minor axis length (b).
     */
    private double mSemiMinorAxis;
    
    /**
     * Rotation angle.
     */
    private double mRotationAngle;
    
    /**
     * Empty constructor.
     * Creates an ellipse equal to a circle located at space origin (0,0) with
     * radius 1.0.
     */
    public Ellipse() {
        mCenter = Point2D.create();
        mSemiMajorAxis = mSemiMinorAxis = 1.0;
        mRotationAngle = 0.0;
    }
    
    /**
     * Constructor.
     * @param center center of ellipse.
     * @param semiMajorAxis semi-major axis length.
     * @param semiMinorAxis semi-minor axis length.
     * @param rotationAngle rotation angle expressed in radians.
     */
    public Ellipse(Point2D center, double semiMajorAxis, double semiMinorAxis, 
            double rotationAngle) {
        setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, 
                rotationAngle);
    }
    
    /**
     * Constructor.
     * @param center center of ellipse.
     * @param semiMajorAxis semi-major axis length.
     * @param semiMinorAxis semi-minor axis length.
     * @param rotation 2D rotation.
     */    
    public Ellipse(Point2D center, double semiMajorAxis, double semiMinorAxis,
            Rotation2D rotation) {
        setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, 
                rotation);
    }
    
    /**
     * Constructor from 2 points, ellipse center and rotation.
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param center center of ellipse.
     * @param theta rotation angle expressed in radians.
     * @throws ColinearPointsException if points are in a colinear or degenerate 
     * configuration.
     */
    public Ellipse(Point2D point1, Point2D point2, Point2D center, double theta)
            throws ColinearPointsException {
        setParametersFromPointsCenterAndRotation(point1, point2, center,
                theta);
    }
    
    /**
     * Constructor from 5 points.
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param point3 3rd point.
     * @param point4 4th point.
     * @param point5 5th point.
     * @throws ColinearPointsException if points are in a colinear or degenerate
     * configuration.
     */    
    public Ellipse(Point2D point1, Point2D point2, Point2D point3,
            Point2D point4, Point2D point5) throws ColinearPointsException {
        setParametersFromPoints(point1, point2, point3, point4, point5);
    }
    
    /**
     * Constructor from 5 points.
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param point3 3rd point.
     * @param point4 4th point.
     * @param point5 5th point.
     * @param threshold threshold to determine whether points form an ellipse.
     * This is usually a very small value
     * @throws ColinearPointsException if points are in a colinear or degenerate
     * configuration.
     */    
    public Ellipse(Point2D point1, Point2D point2, Point2D point3,
            Point2D point4, Point2D point5, double threshold)
            throws ColinearPointsException {
        setParametersFromPoints(point1, point2, point3, point4, point5, 
                threshold);
    }
    
    /**
     * Constructor setting parameters of canonical equation of an ellipse, which 
     * is:
     * a*x^2 + b*x*y + c*y^2 + d*x + e*y + f = 0
     * @param a a parameter.
     * @param b b parameter.
     * @param c c parameter.
     * @param d d parameter.
     * @param e e parameter.
     * @param f f parameter.
     * @throws IllegalArgumentException if parameters do not follow 
     * b^2 - 4*a*c &lt; 0.0
     */
    public Ellipse(double a, double b, double c, double d, double e, double f) 
            throws IllegalArgumentException {
        setParameters(a, b, c, d, e, f);
    }
            
    /**
     * Constructor setting parameters of canonical equation of an ellipse, which 
     * is:
     * a*x^2 + b*x*y + c*y^2 + d*x + e*y + f = 0
     * @param a a parameter.
     * @param b b parameter.
     * @param c c parameter.
     * @param d d parameter.
     * @param e e parameter.
     * @param f f parameter.
     * @param threshold threshold to determine whether parameters are valid due 
     * to machine precision.
     * @throws IllegalArgumentException if parameters do not follow
     * b^2 - 4*A*c &lt; threshold
     */
    public Ellipse(double a, double b, double c, double d, double e, double f, 
            double threshold) throws IllegalArgumentException {
        setParameters(a, b, c, d, e, f, threshold);
    }
    
    /**
     * Constructor.
     * @param conic conic to build ellipse from. 
     * @throws IllegalArgumentException if provided conic is not an ellipse.
     */
    public Ellipse(Conic conic) throws IllegalArgumentException {
        setFromConic(conic);
    }
    
    /**
     * Constructor.
     * @param circle a circle to set parameters from.
     */
    public Ellipse(Circle circle) {
        setFromCircle(circle);
    }
    
    
    /**
     * Returns center of ellipse.
     * @return center of ellipse.
     */
    public Point2D getCenter() {
        return mCenter;
    }
    
    /**
     * Sets center of ellipse.
     * @param center center of ellipse.
     * @throws NullPointerException raised if provided center is null.
     */
    public void setCenter(Point2D center) throws NullPointerException {
        if (center == null) {
            throw new NullPointerException();
        }
        mCenter = center;
    }
    
    /**
     * Gets semi-major axis length.
     * @return semi-major axis length.
     */
    public double getSemiMajorAxis() {
        return mSemiMajorAxis;
    }
    
    /**
     * Sets semi-major axis length.
     * @param semiMajorAxis semi-major axis length.
     */
    public void setSemiMajorAxis(double semiMajorAxis) {
        mSemiMajorAxis = semiMajorAxis;
    }
    
    /**
     * Gets semi-minor axis length.
     * @return semi-minor axis length.
     */
    public double getSemiMinorAxis() {
        return mSemiMinorAxis;
    }
    
    /**
     * Sets semi-minor axis length.
     * @param semiMinorAxis semi-minor axis length.
     */
    public void setSemiMinorAxis(double semiMinorAxis) {
        mSemiMinorAxis = semiMinorAxis;
    }
    
    /**
     * Gets rotation angle expressed in radians.
     * @return rotation angle expressed in radians.
     */
    public double getRotationAngle() {
        return mRotationAngle;
    }
    
    /**
     * Sets rotation angle expressed in radians.
     * @param rotationAngle rotation angle expressed in radians.
     */
    public void setRotationAngle(double rotationAngle) {
        mRotationAngle = rotationAngle;
    }
    
    /**
     * Gets 2D rotation.
     * @return 2D rotation.
     */
    public Rotation2D getRotation() {
        return new Rotation2D(mRotationAngle);
    }
    
    /**
     * Sets 2D rotation.
     * @param rotation 2D rotation to be set.
     */
    public void setRotation(Rotation2D rotation) {
        mRotationAngle = rotation.getTheta();
    }
    
    /**
     * Gets parameter A of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @return parameter A.
     */
    public double getA() {
        double sint = Math.sin(mRotationAngle);
        double cost = Math.cos(mRotationAngle);
        
        double a = mSemiMajorAxis;
        double b = mSemiMinorAxis;
        
        double sint2 = sint*sint;
        double cost2 = cost*cost;
        
        double a2 = a*a;
        double b2 = b*b;
        
        return a2*sint2 + b2*cost2;
    }
    
    /**
     * Sets parameter A of canonical ellipse equation.
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @param a parameter A to be set.
     */
    public void setA(double a) {
        setParameters(a, getB(), getC(), getD(), getE(), getF());
    }
    
    /**
     * Gets parameter B of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @return parameter B.
     */
    public double getB() {
        double sint = Math.sin(mRotationAngle);
        double cost = Math.cos(mRotationAngle);
        double a = mSemiMajorAxis;
        double b = mSemiMinorAxis;
        double a2 = a*a;
        double b2 = b*b;
        
        return 2.0*(b2 - a2)*sint*cost;
    }
    
    /**
     * Sets parameter B of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @param b parameter B to be set.
     */
    public void setB(double b) {
        setParameters(getA(), b, getC(), getD(), getE(), getF());
    }
    
    /**
     * Gets parameter C of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @return parameter C.
     */
    public double getC() {
        double sint = Math.sin(mRotationAngle);
        double cost = Math.cos(mRotationAngle);
        
        double a = mSemiMajorAxis;
        double b = mSemiMinorAxis;
        
        double sint2 = sint*sint;
        double cost2 = cost*cost;
        
        double a2 = a*a;
        double b2 = b*b;
        
        return a2*cost2 + b2*sint2;
    }
    
    /**
     * Sets parameter C of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @param c parameter C to be set.
     */
    public void setC(double c) {
        setParameters(getA(), getB(), c, getD(), getE(), getF());
    }
    
    /**
     * Gets parameter D of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @return parameter D.
     */
    public double getD() {
        double xc = mCenter.getInhomX();
        double yc = mCenter.getInhomY();
        
        return -2.0*getA()*xc - getB()*yc;
    }
    
    /**
     * Sets parameter D of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @param d parameter D to be set.
     */
    public void setD(double d) {
        setParameters(getA(), getB(), getC(), d, getE(), getF());
    }
    
    /**
     * Gets parameter E of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @return parameter E.
     */
    public double getE() {
        double xc = mCenter.getInhomX();
        double yc = mCenter.getInhomY();
        
        return -getB()*xc - 2.0*getC()*yc;
    }
    
    /**
     * Sets parameter E of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @param e parameter E to be set.
     */
    public void setE(double e) {
        setParameters(getA(), getB(), getC(), getD(), e, getF());
    }
    
    /**
     * Gets parameter F of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @return parameter F.
     */
    public double getF() {
        double xc = mCenter.getInhomX();
        double yc = mCenter.getInhomY();
        
        double a = mSemiMajorAxis;
        double b = mSemiMinorAxis;
        
        double xc2 = xc*xc;
        double yc2 = yc*yc;        
        
        double a2 = a*a;
        double b2 = b*b;
        
        return getA()*xc2 + getB()*xc*yc + getC()*yc2 - a2*b2;
    }
    
    /**
     * Sets parameter F of canonical ellipse equation.
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     * @param f parameter F to be set.
     */
    public void setF(double f) {
        setParameters(getA(), getB(), getC(), getD(), getE(), f);
    }
    
    /**
     * Sets ellipse parameters.
     * @param center center of ellipse.
     * @param semiMajorAxis semi-major axis length.
     * @param semiMinorAxis semi-minor axis length.
     * @param rotationAngle rotation angle expressed in radians.
     */
    public final void setCenterAxesAndRotation(Point2D center, 
            double semiMajorAxis, double semiMinorAxis, double rotationAngle) {
        mCenter = center;        
        mSemiMajorAxis = Math.max(semiMajorAxis, semiMinorAxis);
        mSemiMinorAxis = Math.min(semiMinorAxis, semiMajorAxis);
        mRotationAngle = rotationAngle;        
    }
    
    /**
     * Sets ellipse parameters.
     * @param center center of ellipse.
     * @param semiMajorAxis semi-major axis length.
     * @param semiMinorAxis semi-minor axis length.
     * @param rotation 2D rotation.
     */    
    public final void setCenterAxesAndRotation(Point2D center,
            double semiMajorAxis, double semiMinorAxis, Rotation2D rotation) {
        setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, 
                rotation.getTheta());
    }
    
    /**
     * Sets parameters from 2 points, ellipse center and rotation.
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param center center of ellipse.
     * @param theta rotation angle expressed in radians.
     * @throws ColinearPointsException if points are in a colinear or degenerate 
     * configuration.
     */
    public final void setParametersFromPointsCenterAndRotation(Point2D point1, 
            Point2D point2, Point2D center, double theta) 
            throws ColinearPointsException {
        //unknowns: semi-major axis (a) and semi-minor axis (b)
        
        //equation of an ellipse follows:
        //A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
        
        //where
        //sint = sin(theta)
        //cost = cos(theta)

        //A = a^2*sint^2 + b^2*cost^2
        //B = 2*(b^2 - a^2)*sint*cost
        //C = a^2*cost^2 + b^2*sint^2
        //D = -2*A*xc - B*yc = -2*(a^2*sint^2 + b^2*cost^2)*xc - 2*(b^2 - a^2)*sint*cost*yc
        //E = -B*xc - 2*C*yc = -2*(b^2 - a^2)*sint*cost*xc - 2*(a^2*cost^2 + b^2*sint^2)*yc
        //F = A*xc^2 + B*xc*yc + C*yc^2 - a^2*b^2 = (a^2*sint^2 + b^2*cost^2)*xc^2 + 2*(b^2 - a^2)*sint*cost*xc*yc + (a^2*cost^2 + b^2*sint^2)*yc^2 - a^2*b^2

        //(a^2*sint^2 + b^2*cost^2)*x^2 + 
        //2*(b^2 - a^2)*sint*cost*x*y +
        //(a^2*cost^2 + b^2*sint^2)*y^2 + 
        //(-2*(a^2*sint^2 + b^2*cost^2)*xc - 2*(b^2 - a^2)*sint*cost*yc)*x +
        //(-2*(b^2 - a^2)*sint*cost*xc - 2*(a^2*cost^2 + b^2*sint^2)*yc)*y +
        //(a^2*sint^2 + b^2*cost^2)*xc^2 + 2*(b^2 - a^2)*sint*cost*xc*yc + (a^2*cost^2 + b^2*sint^2)*yc^2 - a^2*b^2 = 0

        //a^2*sint^2*x^2 + b^2*cost^2*x^2 +
        //2*b^2*sint*cost*x*y - 2*a^2*sint*cost*x*y +
        //a^2*cost^2*y^2 + b^2*sint^2*y^2 +
        //-2*a^2*sint^2*xc*x -2*b^2*cost^2*xc*x - 2*b^2*sint*cost*yc*x + 2*a^2*sint*cost*yc*x +
        //-2*b^2*sint*cost*xc*y + 2*a^2*sint*cost*xc*y -2*a^2*cost^2*yc*y -2*b^2*sint^2*yc*y +
        //a^2*sint^2*xc^2 + b^2*cost^2*xc^2 + 2*b^2*sint*cost*xc*yc -2*a^2*sint*cost*xc*yc + a^2*cost^2*yc^2 + b^2*sint^2*yc^2 - a^2*b^2 = 0

        //Divide by b^2

        //(a^2/b^2)*sint^2*x^2 + cost^2*x^2 +
        //2*sint*cost*x*y - (a^2/b^2)*2*sint*cost*x*y +
        //(a^2/b^2)*cost^2*y^2 + sint^2*y^2 +
        //-(a^2/b^2)*2*sint^2*xc*x - 2*cost^2*xc*x - 2*sint*cost*yc*x + (a^2/b^2)*2*sint*cost*yc*x +
        //-2*sint*cost*xc*y + (a^2/b^2)*2*sint*cost*xc*y - (a^2/b^2)*2*cost^2*yc*y - 2*sint^2*yc*y +
        //(a^2/b^2)*sint^2*xc^2 + cost^2*xc^2 + 2*sint*cost*xc*yc - (a^2/b^2)*2*sint*cost*xc*yc + (a^2/b^2)*cost^2*yc^2 + sint^2*yc^2 - a^2 = 0

        //(a^2/b^2)*(sint^2*x^2 - 2*sint*cost*x*y + cost^2*y^2 - 2*sint^2*xc*x + 2*sint*cost*yc*x + 2*sint*cost*xc*y - 2*cost^2*yc*y + sint^2*xc^2 - 2*sint*cost*xc*yc + cost^2*yc^2)
        //- a^2
        //= - cost^2*x^2 - 2*sint*cost*x*y - sint^2*y^2 + 2*cost^2*xc*x + 2*sint*cost*yc*x + 2*sint*cost*xc*y + 2*sint^2*yc*y - cost^2*xc^2 - 2*sint*cost*xc*yc - sint^2*yc^2
        
        //Unknowns are:
        //a^2/b^2 and a^2
        
        try {
            double sint = Math.sin(theta);
            double cost = Math.cos(theta);
            
            double sint2 = sint*sint;
            double cost2 = cost*cost;
            double sintcost = sint*cost;
            
            double xc = center.getInhomX();
            double yc = center.getInhomY();
            double xc2 = xc*xc;
            double yc2 = yc*yc;
            
            
            Matrix m = new Matrix(2, 2);
            double[] b = new double[2];
            
            double[] x = new double[]{
                point1.getInhomX(),
                point2.getInhomX(),
            };
            
            double[] y = new double[]{
                point1.getInhomY(),
                point2.getInhomY(),
            };
            
            double x2, y2, rowNorm;
            for (int i = 0; i < 2; i++) {
                x2 = x[i]*x[i];
                y2 = y[i]*y[i];
                m.setElementAt(i, 0, sint2*x2 - 2.0*sintcost*x[i]*y[i] +
                        cost2*y2 - 2.0*sint2*xc*x[i] + 2.0*sintcost*yc*x[i] +
                        2.0*sintcost*xc*y[i] - 2.0*cost2*yc*y[i] + sint2*xc2 -
                        2.0*sintcost*xc*yc + cost2*yc2);
                m.setElementAt(i, 1, -1.0);
                
                b[i] = - cost2*x2 - 2.0*sintcost*x[i]*y[i] - 
                        sint2*y2 + 2.0*cost2*xc*x[i] + 2.0*sintcost*yc*x[i] +
                        2.0*sintcost*xc*y[i] + 2.0*sint2*yc*y[i] -
                        cost2*xc2  - 2.0*sintcost*xc*yc - sint2*yc2;
                
                //normalize row to increase accuracy
                rowNorm = 0.0;
                for (int j = 0; j < 2; j++) {
                    rowNorm += Math.pow(m.getElementAt(i, j), 2.0);
                }
                rowNorm = Math.sqrt(rowNorm);
                
                for(int j = 0; j < 2; j++) {
                    m.setElementAt(i, j, m.getElementAt(i, j) / rowNorm);
                }
                
                b[i] /= rowNorm;
            }
            
            double[] params = com.irurueta.algebra.Utils.solve(m, b);

            //params[1] = a^2
            double semiMajorAxis = Math.sqrt(Math.abs(params[1]));
            
            //params[0] = a^2/b^2 --> b^2 = a^2/params[0] = params[1]/params[0]
            double semiMinorAxis = Math.sqrt(Math.abs(params[1]/params[0]));
            
            setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, 
                    theta);
        } catch (AlgebraException e) {
            throw new ColinearPointsException(e);
        }
    }
    
    /**
     * Sets parameters from 5 points.
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param point3 3rd point.
     * @param point4 4th point.
     * @param point5 5th point.
     * @throws ColinearPointsException if points are in a colinear or degenerate
     * configuration.
     */
    public final void setParametersFromPoints(Point2D point1, Point2D point2, 
            Point2D point3, Point2D point4, Point2D point5) 
            throws ColinearPointsException {
        setParametersFromPoints(point1, point2, point3, point4, point5, 0.0);
    }
    
    /**
     * Sets parameters from 5 points.
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param point3 3rd point.
     * @param point4 4th point.
     * @param point5 5th point.
     * @param threshold threshold to determine whether points form an ellipse.
     * This is usually a very small value
     * @throws ColinearPointsException if points are in a colinear or degenerate
     * configuration.
     */
    public final void setParametersFromPoints(Point2D point1, Point2D point2,
            Point2D point3, Point2D point4, Point2D point5, double threshold) 
            throws ColinearPointsException {
        //normalize points to increase accuracy
        point1.normalize();
        point2.normalize();
        point3.normalize();
        point4.normalize();
        point5.normalize();
        
        try {
            //each point belonging to a conic follows equation:
            //p' * C * p = 0 ==>
            //x^2 + y^2 + w^2 + 2*x*y + 2*x*w + 2*y*w = 0
            Matrix m = new Matrix(5, 6);
            double x = point1.getHomX();
            double y = point1.getHomY();
            double w = point1.getHomW();
            m.setElementAt(0, 0, x * x);
            m.setElementAt(0, 1, 2.0 * x * y);
            m.setElementAt(0, 2, y * y);
            m.setElementAt(0, 3, 2.0 * x * w);
            m.setElementAt(0, 4, 2.0 * y * w);
            m.setElementAt(0, 5, w * w);
            x = point2.getHomX();
            y = point2.getHomY();
            w = point2.getHomW();
            m.setElementAt(1, 0, x * x);
            m.setElementAt(1, 1, 2.0 * x * y);
            m.setElementAt(1, 2, y * y);
            m.setElementAt(1, 3, 2.0 * x * w);
            m.setElementAt(1, 4, 2.0 * y * w);
            m.setElementAt(1, 5, w * w);
            x = point3.getHomX();
            y = point3.getHomY();
            w = point3.getHomW();
            m.setElementAt(2, 0, x * x);
            m.setElementAt(2, 1, 2.0 * x * y);
            m.setElementAt(2, 2, y * y);
            m.setElementAt(2, 3, 2.0 * x * w);
            m.setElementAt(2, 4, 2.0 * y * w);
            m.setElementAt(2, 5, w * w);
            x = point4.getHomX();
            y = point4.getHomY();
            w = point4.getHomW();
            m.setElementAt(3, 0, x * x);
            m.setElementAt(3, 1, 2.0 * x * y);
            m.setElementAt(3, 2, y * y);
            m.setElementAt(3, 3, 2.0 * x * w);
            m.setElementAt(3, 4, 2.0 * y * w);
            m.setElementAt(3, 5, w * w);
            x = point5.getHomX();
            y = point5.getHomY();
            w = point5.getHomW();
            m.setElementAt(4, 0, x * x);
            m.setElementAt(4, 1, 2.0 * x * y);
            m.setElementAt(4, 2, y * y);
            m.setElementAt(4, 3, 2.0 * x * w);
            m.setElementAt(4, 4, 2.0 * y * w);
            m.setElementAt(4, 5, w * w);
            
            //normalize each row to increase accuracy
            double[] row = new double[6];
            double rowNorm;
            
            for (int j = 0; j < 5; j++) {
                m.getSubmatrixAsArray(j, 0, j, 5, row);
                rowNorm = com.irurueta.algebra.Utils.normF(row);
                for (int i = 0; i < 6; i++) {
                    m.setElementAt(j, i, m.getElementAt(j, i) / rowNorm);
                }
            }            
            
            SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();
            
            if (decomposer.getRank() < 5) throw new ColinearPointsException();
            
            //the right null-space of m contains the parameters a, b, c, d, e ,f
            //of the conic
            Matrix V = decomposer.getV();
            
            double aPrime = V.getElementAt(0, 5);
            double bPrime = V.getElementAt(1, 5);
            double cPrime = V.getElementAt(2, 5);
            double dPrime = V.getElementAt(3, 5);
            double ePrime = V.getElementAt(4, 5);
            double fPrime = V.getElementAt(5, 5);
            
            //an ellipse follows the generic conic equation
            //A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
            
            //Or in matrix form
            //[A    B/2 D/2 ]   [A' B'  D']
            //[B/2  C   E/2 ] = [B' C'  E']
            //[D/2  E/2 F   ]   [D' E'  F']
            
            double a = aPrime;
            double b = 2.0*bPrime;
            double c = cPrime;
            double d = 2.0*dPrime;
            double e = 2.0*ePrime;
            double f = fPrime;
                        
            setParameters(a, b, c, d, e, f, threshold);                        
        } catch (AlgebraException ex) {
            throw new ColinearPointsException(ex);
        } catch (IllegalArgumentException ex) {
            throw new ColinearPointsException(ex);
        }
    }
    
    /**
     * Sets parameters of canonical equation of an ellipse, which is:
     * a*x^2 + b*x*y + c*y^2 + d*x + e*y + f = 0
     * @param a a parameter.
     * @param b b parameter.
     * @param c c parameter.
     * @param d d parameter.
     * @param e e parameter.
     * @param f f parameter.
     * @throws IllegalArgumentException if parameters do not follow 
     * b^2 - 4*a*c &lt; 0.0
     */
    public final void setParameters(double a, double b, double c, double d,
            double e, double f) throws IllegalArgumentException {
        setParameters(a, b, c, d, e, f, 0.0);
    }
    
    /**
     * Sets parameters of canonical equation of an ellipse, which is:
     * a*x^2 + b*x*y + c*y^2 + d*x + e*y + f = 0
     * @param a a parameter.
     * @param b b parameter.
     * @param c c parameter.
     * @param d d parameter.
     * @param e e parameter.
     * @param f f parameter.
     * @param threshold threshold to determine whether parameters are valid due 
     * to machine precision.
     * @throws IllegalArgumentException if parameters do not follow
     * b^2 - 4*A*c &lt; threshold
     */
    public final void setParameters(double a, double b, double c, double d, 
            double e, double f, double threshold) 
            throws IllegalArgumentException {
        if (b*b - 4.0*a*c >= threshold) {
            //not an ellipse
            throw new IllegalArgumentException();
        }   
        
        double semiMajorAxis = - Math.sqrt(
                2.0*(a*e*e + c*d*d - b*d*e + (b*b - 4.0*a*c)*f)*
                (a + c + Math.sqrt((a - c)*(a - c) + b*b))) /
                (b*b - 4*a*c);
        
        double semiMinorAxis = - Math.sqrt(
                2.0*(a*e*e + c*d*d - b*d*e + (b*b - 4.0*a*c)*f)*
                (a + c - Math.sqrt((a - c)*(a - c) + b*b))) /
                (b*b - 4*a*c);
        
        double xc = (2*c*d - b*e) / (b*b - 4.0*a*c);
        double yc = (2*a*e - b*d) / (b*b - 4.0*a*c);        
        Point2D center = new InhomogeneousPoint2D(xc, yc);
        
        double rotationAngle;
        if (Math.abs(b) <= threshold && a < c) {
            if (a < c) {
                rotationAngle = 0.0;
            } else {
                rotationAngle = Math.PI / 2.0;
            }
        } else {
            rotationAngle = Math.atan2(
                    c - a - Math.sqrt((a - c)*(a - c) + b*b), b);
        }
        
        setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, 
                rotationAngle);
    }
    
    /**
     * Gets focus distance of ellipse.
     * Focus determines the distance respect to the center of the ellipse 
     * where the two focus points are located.
     * The sum of the distances from any point P = P(x,y) on the ellipse to 
     * those two foci is constant and equal to the major axis length.
     * @return focus distance.
     */
    public double getFocus() {
        return Math.sqrt(Math.pow(mSemiMajorAxis, 2.0) - 
                Math.pow(mSemiMinorAxis, 2.0));
    }
    
    /**
     * Gets semi major axis x,y coordinates.
     * @param coords array where x, y coordinates of semi major axis will be 
     * stored.
     */
    public void getSemiMajorAxisCoordinates(double[] coords) {
        coords[0] = mSemiMajorAxis * Math.cos(mRotationAngle);
        coords[1] = mSemiMajorAxis * Math.sin(mRotationAngle);
    }
    
    /**
     * Gets semi major axis x,y coordinates.
     * @return array containing x, y coordinates of semi major axis.
     */
    public double[] getSemiMajorAxisCoordinates() {
        double[] coords = new double[
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        getSemiMajorAxisCoordinates(coords);
        return coords;
    }
    
    /**
     * Sets semi major axis coordinates.
     * This method updates rotation angle and semi major axis length.
     * @param coords coordinates of semi major axis.
     */
    public void setSemiMajorAxisCoordinates(double[] coords) {
        mRotationAngle = Math.atan2(coords[1], coords[0]);
        mSemiMajorAxis = com.irurueta.algebra.Utils.normF(coords);
    }
    
    /**
     * Gets semi minor axis x,y coordinates.
     * @param coords array where x, y coordinates of semi minor axis will be 
     * stored.
     */
    public void getSemiMinorAxisCoordinates(double[] coords) {
        coords[0] = -mSemiMinorAxis * Math.sin(mRotationAngle);
        coords[1] = mSemiMinorAxis * Math.cos(mRotationAngle);
    }
    
    /**
     * Gets semi minor axis x,y coordinates.
     * @return array containing x,y coordinates of semi minor axis.
     */
    public double[] getSemiMinorAxisCoordinates() {
        double[] coords = new double[
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        getSemiMinorAxisCoordinates(coords);
        return coords;
    }
    
    /**
     * Sets semi minor axis coordinates.
     * This method updates rotation angle and semi minor axis length.
     * @param coords coordinates of semi minor axis.
     */
    public void setSemiMinorAxisCoordinates(double[] coords) {
        mRotationAngle = Math.atan2(-coords[0], coords[1]);
        mSemiMinorAxis = com.irurueta.algebra.Utils.normF(coords);
    }
    
    /**
     * Gets 1st focus point.
     * The sum of the distances from any point P = P(x,y) on the ellipse to 
     * this focus is constant and equal to the major axis length.
     * @param focusPoint1 1st focus point.
     */
    public void getFocusPoint1(Point2D focusPoint1) {
        double focus = getFocus();
        
        focusPoint1.setInhomogeneousCoordinates(
                mCenter.getInhomX() - focus * Math.cos(mRotationAngle), 
                mCenter.getInhomY() - focus * Math.sin(mRotationAngle));
    }
    
    /**
     * Gets 1st focus point.
     * The sum of the distances from any point P = P(x,y) on the ellipse to 
     * this focus is constant and equal to the major axis length.
     * @return 1st focus point.
     */
    public Point2D getFocusPoint1() {
        Point2D result = Point2D.create();
        getFocusPoint1(result);
        return result;
    }
    
    /**
     * Gets 2nd focus point.
     * The sum of the distances from any point P = P(x,y) on the ellipse to 
     * this focus is constant and equal to the major axis length.
     * @param focusPoint2 2nd focus point.
     */
    public void getFocusPoint2(Point2D focusPoint2) {
        double focus = getFocus();
        
        focusPoint2.setInhomogeneousCoordinates(
                mCenter.getInhomX() + focus * Math.cos(mRotationAngle), 
                mCenter.getInhomY() + focus * Math.sin(mRotationAngle));
    }
    
    /**
     * Gets 2nd focus point.
     * The sum of the distances from any point P = P(x,y) on the ellipse to 
     * this focus is constant and equal to the major axis length.
     * @return 2nd focus point.
     */
    public Point2D getFocusPoint2() {
        Point2D result = Point2D.create();
        getFocusPoint2(result);
        return result;
    }
    
    /**
     * Sets focus points.
     * @param focusPoint1 1st focus point.
     * @param focusPoint2 2nd focus point.
     * @param keepSemiMinorAxis true indicates that semi-minor axis is kept, 
     * false indicates that semi-major axis is kept instead.
     */
    public void setFocusPoints(Point2D focusPoint1, Point2D focusPoint2, 
            boolean keepSemiMinorAxis) {
        InhomogeneousPoint2D center = new InhomogeneousPoint2D(
                (focusPoint1.getInhomX() + focusPoint2.getInhomX()) / 2.0, 
                (focusPoint1.getInhomY() + focusPoint2.getInhomY()) / 2.0);
        double rotationAngle = Math.atan2(
                focusPoint2.getInhomY() - focusPoint1.getInhomY(), 
                focusPoint2.getInhomX() - focusPoint1.getInhomX());
        
        double f = focusPoint1.distanceTo(center);
        double f2 = f*f;
        double a2, b2;
        if(keepSemiMinorAxis) {
            b2 = mSemiMinorAxis * mSemiMinorAxis;
            a2 = f2 + b2;
        } else {
            a2 = mSemiMajorAxis * mSemiMajorAxis;
            b2 = a2 - f2;
        }
        
        double semiMajorAxis = Math.sqrt(a2);
        double semiMinorAxis = Math.sqrt(b2);
        
        setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, 
                rotationAngle);
    }
    
    /**
     * Gets eccentricity of ellipsis.
     * @return eccentricity of ellipsis.
     */
    public double getEccentricity() {
        return getFocus() / mSemiMajorAxis;
    }
    
    /**
     * Returns area of this ellipse.
     * @return area of this ellipse.
     */
    public double getArea() {
        return Math.PI * mSemiMajorAxis * mSemiMinorAxis;
    }
    
    /**
     * Returns perimeter of this ellipse.
     * @return Perimeter of this ellipse.
     */
    public double getPerimeter() {
        double a = mSemiMajorAxis;
        double b = mSemiMinorAxis;
        
        return Math.PI * (3.0 * (a + b) - Math.sqrt((3.0*a + b) * (a + 3.0*b)));
    }    

    /**
     * Gets curvature of ellipse at provided point.
     * @param point point to be checked.
     * @return curvature of ellipse.
     */
    public double getCurvature(Point2D point) {
        double a = mSemiMajorAxis;
        double b = mSemiMinorAxis;
        
        double a2 = a*a;
        double b2 = b*b;
        
        double a4 = a2*a2;
        double b4 = b2*b2;
        
        double x = point.getInhomX() - mCenter.getInhomX();
        double y = point.getInhomY() - mCenter.getInhomY();
        
        double x2 = x*x;
        double y2 = y*y;
        
        return Math.pow(x2 / a4 + y2 / b4, -3.0 / 2.0) / (a2*b2);
    }
    
    /**
     * Converts this circle into a conic.
     * Conics are a more general representation of circles.
     * @return A conic representing this circle
     */
    public Conic toConic() {
        mCenter.normalize();
        //use inhomogeneous center coordinates
        double xc = mCenter.getInhomX();
        double yc = mCenter.getInhomY();
        
        double sint = Math.sin(mRotationAngle);
        double cost = Math.cos(mRotationAngle);
        
        double a = mSemiMajorAxis;
        double b = mSemiMinorAxis;
        
        double xc2 = xc*xc;
        double yc2 = yc*yc;
        
        double sint2 = sint*sint;
        double cost2 = cost*cost;
        
        double a2 = a*a;
        double b2 = b*b;
        
        double A = a2*sint2 + b2*cost2;
        double B = 2.0*(b2 - a2)*sint*cost;
        double C = a2*cost2 + b2*sint2;
        double D = -2.0*A*xc - B*yc;
        double E = -B*xc - 2.0*C*yc;
        double F = A*xc2 + B*xc*yc + C*yc2 - a2*b2;
        
        double aConic = A;
        double bConic = B / 2.0;
        double cConic = C;
        double dConic = D / 2.0;
        double eConic = E / 2.0;
        double fConic = F;
                
        return new Conic(aConic, bConic, cConic, dConic, eConic, fConic);
    }
    
    /**
     * Set parameters of this circle from a valid conic corresponding to a
     * circle.
     * @param conic conic to set parameters from.
     * @throws IllegalArgumentException if provided conic is not an ellipse.
     */
    public final void setFromConic(Conic conic) throws IllegalArgumentException{
        if(conic.getConicType() != ConicType.ELLIPSE_CONIC_TYPE &&
                conic.getConicType() != ConicType.CIRCLE_CONIC_TYPE) {
            throw new IllegalArgumentException();
        }
        
        conic.normalize();

        double aConic = conic.getA();
        double bConic = conic.getB();
        double cConic = conic.getC();
        double dConic = conic.getD();
        double eConic = conic.getE();
        double fConic = conic.getF();
            
        //an ellipse follows the generic conic equation
        //A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
            
        //Or in matrix form
        //[A    B/2 D/2 ]   [A' B'  D']
        //[B/2  C   E/2 ] = [B' C'  E']
        //[D/2  E/2 F   ]   [D' E'  F']
            
        double a = aConic;
        double b = 2.0*bConic;
        double c = cConic;
        double d = 2.0*dConic;
        double e = 2.0*eConic;
        double f = fConic;
                        
        setParameters(a, b, c, d, e, f);                                
    }    
    
    /**
     * Sets parameters of this ellipse from a circle.
     * @param circle a circle to set parameters from.
     */
    public final void setFromCircle(Circle circle) {
        mCenter = circle.getCenter();
        mSemiMajorAxis = mSemiMinorAxis = circle.getRadius();
        mRotationAngle = 0.0;
    }
    
    /**
     * Determines if provided point is inside this ellipse or not up to a 
     * certain threshold.
     * If provided threshold is positive, the ellipse behaves as if it was a 
     * larger ellipse increased by threshold amount, if provided threshold is
     * negative, the ellipse behaves as if it was a smaller ellipse decreased by
     * threshold amount in radius.
     * @param point Point to be checked.
     * @param threshold Threshold to determine if point is inside or not.
     * @return True if point is considered to be inside this circle, false
     * otherwise.
     */
    public boolean isInside(Point2D point, double threshold) {
        mCenter.normalize();
        //use inhomogeneous center coordinates
        double xc = mCenter.getInhomX();
        double yc = mCenter.getInhomY();
        
        double sint = Math.sin(mRotationAngle);
        double cost = Math.cos(mRotationAngle);
        
        double a = mSemiMajorAxis;
        double b = mSemiMinorAxis;
        
        double xc2 = xc*xc;
        double yc2 = yc*yc;
        
        double sint2 = sint*sint;
        double cost2 = cost*cost;
        
        double a2 = a*a;
        double b2 = b*b;
        
        double A = a2*sint2 + b2*cost2;
        double B = 2.0*(b2 - a2)*sint*cost;
        double C = a2*cost2 + b2*sint2;
        double D = -2.0*A*xc - B*yc;
        double E = -B*xc - 2.0*C*yc;
        double F = A*xc2 + B*xc*yc + C*yc2 - a2*b2;
        
        double x = point.getInhomX();
        double y = point.getInhomY();
        
        return A*x*x + B*x*y + C*y*y + D*x + E*y + F <= threshold;
    }

    /**
     * Determines if provided point is inside this circle or not.
     * @param point Point to be checked.
     * @return True if point is considered to be inside this circle, false
     * otherwise.
     */
    public boolean isInside(Point2D point) {
        return isInside(point, 0.0);
    }    
    
    /**
     * Determines whether provided point lies at ellipse boundary or not up to
     * a certain threshold.
     * @param point Point to be checked.
     * @param threshold A small threshold to determine whether point lies at
     * ellipse boundary.
     * @return True if point lies at ellipse boundary, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is 
     * negative.
     */
    public boolean isLocus(Point2D point, double threshold) 
            throws IllegalArgumentException {        
        if (threshold < MIN_THRESHOLD) throw new IllegalArgumentException();
        
        mCenter.normalize();
        //use inhomogeneous center coordinates
        double xc = mCenter.getInhomX();
        double yc = mCenter.getInhomY();
        
        double sint = Math.sin(mRotationAngle);
        double cost = Math.cos(mRotationAngle);
        
        double a = mSemiMajorAxis;
        double b = mSemiMinorAxis;
        
        double xc2 = xc*xc;
        double yc2 = yc*yc;
        
        double sint2 = sint*sint;
        double cost2 = cost*cost;
        
        double a2 = a*a;
        double b2 = b*b;
        
        double A = a2*sint2 + b2*cost2;
        double B = 2.0*(b2 - a2)*sint*cost;
        double C = a2*cost2 + b2*sint2;
        double D = -2.0*A*xc - B*yc;
        double E = -B*xc - 2.0*C*yc;
        double F = A*xc2 + B*xc*yc + C*yc2 - a2*b2;
        
        double x = point.getInhomX();
        double y = point.getInhomY();
        
        return Math.abs(A*x*x + B*x*y + C*y*y + D*x + E*y + F) <= threshold;
    }    
    
    /**
     * Determines whether provided point lies at ellipse boundary or not.
     * @param point Point to be checked.
     * @return True if point lies at ellipse boundary, false otherwise.
     */
    public boolean isLocus(Point2D point) {
        return isLocus(point, DEFAULT_THRESHOLD);
    }    

    /**
     * Returns a line tangent to this ellipse at provided point. Provided point
     * must be locus of this ellipse, otherwise a NotLocusException will be 
     * thrown.
     * @param point a locus point of this ellipse.
     * @return a 2D line tangent to this ellipse at provided point.
     * @throws NotLocusException if provided point is not locus of this ellipse 
     * up to DEFAULT_THRESHOLD.
     */
    public Line2D getTangentLineAt(Point2D point) throws NotLocusException {
        return getTangentLineAt(point, DEFAULT_THRESHOLD);
    }
    
    /**
     * Returns a line tangent to this ellipse at provided point. Provided point
     * must be locus of this ellipse, otherwise a NotLocusException will be 
     * thrown.
     * @param point a locus point of this circle.
     * @param threshold threshold to determine if provided point is locus.
     * @return a 2D line tangent to this circle at provided point.
     * @throws NotLocusException if provided point is not locus of this circle 
     * up to provided threshold.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public Line2D getTangentLineAt(Point2D point, double threshold)
            throws NotLocusException, IllegalArgumentException {
        
        Line2D line = new Line2D();
        tangentLineAt(point, line, threshold);
        return line;
    }
    
    /**
     * Computes a line tangent to this circle at provided point. Provided point
     * must be locus of this circle, otherwise a NotLocusException will be 
     * thrown.
     * @param point a locus point of this circle.
     * @param line instance of a 2D line where result will be stored.
     * @param threshold threshold to determine if provided point is locus.
     * @throws NotLocusException if provided point is not locus of this circle
     * up to provided threshold.
     * @throws IllegalArgumentException if provided threshold is negtive.
     */
    public void tangentLineAt(Point2D point, Line2D line, double threshold)
            throws NotLocusException, IllegalArgumentException {
        
        if(!isLocus(point, threshold)) throw new NotLocusException();
        
        Conic c = toConic();
        c.tangentLineAt(point, line, threshold);
    }        
}