/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.Ellipsoid
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 25, 2017.
 */
package com.irurueta.geometry;

import java.io.Serializable;
import java.util.Arrays;

/**
 * This class defines an ellipsoid.
 */
public class Ellipsoid implements Serializable {
    
    /**
     * Number of dimensions.
     */
    public static final int DIMENSIONS = 3;
    
    /**
     * Constant to compute approximate surface area.
     */
    private static final double P = 1.6075;
    
    /**
     * Center of ellipse.
     */
    private Point3D mCenter;
    
    
    /**
     * Lengths of all three semi-axes.
     */
    double[] mSemiAxesLenghts;
    
    /**
     * Rotation.
     */
    private Rotation3D mRotation;
    
    /**
     * Empty constructor.
     * Creates an ellipsoid equal to a sphere located at space origin (0,0,0) 
     * with radius 1.0.
     */
    public Ellipsoid() {
        mCenter = Point3D.create();
        mSemiAxesLenghts = new double[DIMENSIONS];
        Arrays.fill(mSemiAxesLenghts, 1.0);
        mRotation = Rotation3D.create();
    }
    
    /**
     * Sets ellipsoid parameters.
     * @param center center of ellipsoid.
     * @param semiAxesLengths lengths of all three semi-axes.
     * @param rotation rotation.
     * @throws IllegalArgumentException if length of provided array is not 
     * three.
     */
    public Ellipsoid(Point3D center, double[] semiAxesLengths, 
            Rotation3D rotation) throws IllegalArgumentException {
        setCenterAxesAndRotation(center, semiAxesLengths, rotation);
    }
        
    /**
     * Returns center of ellipsoid.
     * @return center of ellipsoid.
     */
    public Point3D getCenter() {
        return mCenter;
    }
    
    /**
     * Sets center of ellipsoid.
     * @param center center of ellipsoid.
     * @throws NullPointerException raised if provided center is null.
     */
    public void setCenter(Point3D center) throws NullPointerException {
        if (center == null) {
            throw new NullPointerException();
        }
        mCenter = center;
    }
    
    /**
     * Gets lenghts of all three semi-axes.
     * @return lengths of all three semi-axes.
     */
    public double[] getSemiAxesLengths() {
        return mSemiAxesLenghts;
    }
    
    /**
     * Sets lengths of all three semi-axes.
     * @param semiAxesLengths lengths of all three semi-axes.
     * @throws IllegalArgumentException if length of provided array is not 
     * three.
     */
    public void setSemiAxesLengths(double[] semiAxesLengths) 
            throws IllegalArgumentException {
        if (semiAxesLengths.length != DIMENSIONS) {
            throw new IllegalArgumentException();
        }
        mSemiAxesLenghts = semiAxesLengths;
    }
    
    /**
     * Gets rotation.
     * @return rotation.
     */
    public Rotation3D getRotation() {
        return mRotation;
    }
    
    /**
     * Sets rotation.
     * @param rotation rotation.
     */
    public void setRotation(Rotation3D rotation) {
        mRotation = rotation;
    }
    
    /**
     * Sets ellipsoid parameters.
     * @param center center of ellipsoid.
     * @param semiAxesLengths lengths of all three semi-axes.
     * @param rotation rotation.
     * @throws IllegalArgumentException if length of provided array is not 
     * three.
     */
    public final void setCenterAxesAndRotation(Point3D center, 
            double[] semiAxesLengths, Rotation3D rotation) {
        if (semiAxesLengths.length != DIMENSIONS) {
            throw new IllegalArgumentException();
        }        
        mCenter = center;
        mSemiAxesLenghts = semiAxesLengths;
        mRotation = rotation;
    }
    
    /**
     * Returns volume of this ellipsoid.
     * @return volume of htis ellipsoid.
     */
    public double getVolume() {
        double a = mSemiAxesLenghts[0];
        double b = mSemiAxesLenghts[1];
        double c = mSemiAxesLenghts[2];
        return 4.0 / 3.0 * Math.PI * a * b * c;
    }
    
    /**
     * Returns surface of this ellipsoid.
     * @return surface of this ellipsoid.
     */
    public double getSurface() {
        double a = mSemiAxesLenghts[0];
        double b = mSemiAxesLenghts[1];
        double c = mSemiAxesLenghts[2];
        
        return 4.0 * Math.PI * Math.pow(
                (Math.pow(a*b, P) + Math.pow(a*c, P) + Math.pow(b*c, P)) / 3.0, 
                1.0 / P);
    }
    
    /**
     * Converts this ellipsoid into a quadric.
     * Quadrics a re a more general representation of ellipsoids.
     * @return a quadric representinc this ellipsoid.
     * @throws GeometryException if quadric cannot be determined due to 
     * numerical instabilities.
     */
    public Quadric toQuadric() throws GeometryException {
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

        //An Ellipsoid is a particular form of a quadric following expression:
        //x^2 / a^2 + y^2 / b^2 + z^2 / c^2 = 1
        //when centered at origin and having no rotation
        
        //if we take an arbitrary center position:
        double a = 1.0 / Math.pow(mSemiAxesLenghts[0], 2.0);
        double b = 1.0 / Math.pow(mSemiAxesLenghts[1], 2.0);
        double c = 1.0 / Math.pow(mSemiAxesLenghts[2], 2.0);
        double d = 0.0;
        double e = 0.0;
        double f = 0.0;
        double g = 0.0;
        double h = 0.0;
        double i = 0.0;
        double j = -1;
        
        Quadric q = new Quadric(a, b, c, d, e, f, g, h, i, j);
        EuclideanTransformation3D t = 
                new EuclideanTransformation3D(mRotation, new double[]{ 
                    mCenter.getInhomX(), mCenter.getInhomY(), 
                    mCenter.getInhomZ()});
        
        try {
            t.transform(q);
        } catch (NonSymmetricMatrixException ex) {
            throw new GeometryException(ex);
        }
        
        return q;
    }
    
    /**
     * Sets parameters of this ellipsoid from a sphere.
     * @param sphere a sphere to set parameters from.
     */
    public final void setFromSphere(Sphere sphere) {
        mCenter = sphere.getCenter();
        Arrays.fill(mSemiAxesLenghts, sphere.getRadius());
        mRotation = Rotation3D.create();
    }
}