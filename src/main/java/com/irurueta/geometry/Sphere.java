/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.Sphere
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 14, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import java.io.Serializable;

/**
 *  This class defines a sphere
 */
public class Sphere implements Serializable{
    
    /**
     * Center of sphere
     */
    private Point3D mCenter;
    
    /**
     * Radius of sphere
     */
    private double mRadius;
    
    /**
     * Constant defining minimum allowed radius
     */
    public static final double MIN_RADIUS = 0.0;
    
    /**
     * Constant defining default threshold value used when none is provided
     */
    public static final double DEFAULT_THRESHOLD = 1e-9;
    
    /**
     * Constant defining minimum allowed threshold
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Constant defining machine precision
     */
    public static final double EPS = 1e-12;
    
    /**
     * Empty constructor
     * Creates sphere located at space origin (0,0) with radius 1.0
     */
    public Sphere(){
        mCenter = Point3D.create();
        mRadius = 1.0;
    }
    
    /**
     * Constructor.
     * Sets center and radius of sphere.
     * @param center Center of sphere
     * @param radius Radius of sphere
     * @throws IllegalArgumentException Raised if provided radius is negative
     */
    public Sphere(Point3D center, double radius) 
            throws IllegalArgumentException{
        setCenterAndRadius(center, radius);
    }
    
    /**
     * Constructor.
     * Computes a sphere by using 4 points that must belong to its locus
     * @param point1 point 1
     * @param point2 point 2
     * @param point3 point 3
     * @param point4 point 4
     * @throws CoplanarPointsException if provided set of points are coincident 
     * or coplanar (form a single plane). In such cases a singularity occurs
     * since a sphere having an infinite radius would be required to contain all
     * four points in its locus
     */
    public Sphere(Point3D point1, Point3D point2, Point3D point3, 
            Point3D point4) throws CoplanarPointsException{
        setParametersFromPoints(point1, point2, point3, point4);
    }
    
    /**
     * Constructor.
     * Computes a sphere from a valir quadric corresponding to asphere
     * @param quadric a quadric to create a sphere from
     * @throws IllegalArgumentException if provided quadric is not a sphere
     */
    public Sphere(Quadric quadric) throws IllegalArgumentException{
        setFromQuadric(quadric);
    }
    
    /**
     * Returns center of sphere
     * @return Center of sphere
     */
    public Point3D getCenter(){
        return mCenter;
    }
    
    /**
     * Sets center of sphere
     * @param center Center of sphere
     * @throws NullPointerException Raised if provided center is null
     */
    public void setCenter(Point3D center) throws NullPointerException{
        if(center == null) throw new NullPointerException();
        
        mCenter = center;
    }
    
    /**
     * Returns radius of sphere
     * @return Radius of sphere
     */
    public double getRadius(){
        return mRadius;
    }
    
    /**
     * Sets radius of sphere
     * @param radius Radius of sphere
     * @throws IllegalArgumentException Raised if provided radius is negative
     */
    public void setRadius(double radius) throws IllegalArgumentException{
        if(radius < MIN_RADIUS) throw new IllegalArgumentException();
        
        mRadius = radius;
    }
    
    /**
     * Sets center and radius of this sphere
     * @param center Center to be set
     * @param radius Radius to be set
     * @throws IllegalArgumentException Raised if provided radius is negative
     * @throws NullPointerException Raised if provided center is null
     */
    public final void setCenterAndRadius(Point3D center, double radius)
            throws IllegalArgumentException, NullPointerException{
        setRadius(radius);        
        setCenter(center);
    }
    
    /**
     * Sets parameters of a sphere by using four points that must belong to its
     * locus.
     * @param point1 point 1
     * @param point2 point 2
     * @param point3 point 3
     * @param point4 point 4
     * @throws CoplanarPointsException if provided set of points are coincident
     * or coplanar (form a single plane). In such cases a singularity occurs
     * since a sphere having an infinite radius would be required to contain all
     * four points in its locus
     */
    public final void setParametersFromPoints(Point3D point1, Point3D point2, 
            Point3D point3, Point3D point4) throws CoplanarPointsException{
        
        //normalize points to increase accuracy
        point1.normalize();
        point2.normalize();
        point3.normalize();
        point4.normalize();
        
        try{
            Matrix m = new Matrix(4, 4);
            double[] b = new double[4];
            
            //1st point
            double x = point1.getHomX();
            double y = point1.getHomY();
            double z = point1.getHomZ();
            double w = point1.getHomW();
            m.setElementAt(0, 0, 2.0 * x * w);
            m.setElementAt(0, 1, 2.0 * y * w);
            m.setElementAt(0, 2, 2.0 * z * w);
            m.setElementAt(0, 3, w * w);
            b[0] = -x * x - y * y - z * z;
            
            //2nd point
            x = point2.getHomX();
            y = point2.getHomY();
            z = point2.getHomZ();
            w = point2.getHomW();
            m.setElementAt(1, 0, 2.0 * x * w);
            m.setElementAt(1, 1, 2.0 * y * w);
            m.setElementAt(1, 2, 2.0 * z * w);
            m.setElementAt(1, 3, w * w);
            b[1] = -x * x - y * y - z * z;   
            
            //3rd point
            x = point3.getHomX();
            y = point3.getHomY();
            z = point3.getHomZ();
            w = point3.getHomW();
            m.setElementAt(2, 0, 2.0 * x * w);
            m.setElementAt(2, 1, 2.0 * y * w);
            m.setElementAt(2, 2, 2.0 * z * w);
            m.setElementAt(2, 3, w * w);
            b[2] = -x * x - y * y - z * z;  
            
            //4th point
            x = point4.getHomX();
            y = point4.getHomY();
            z = point4.getHomZ();
            w = point4.getHomW();
            m.setElementAt(3, 0, 2.0 * x * w);
            m.setElementAt(3, 1, 2.0 * y * w);
            m.setElementAt(3, 2, 2.0 * z * w);
            m.setElementAt(3, 3, w * w);
            b[3] = -x * x - y * y - z * z;   
            
            //normalize each row to increase accuracy
            double[] row = new double[4];
            double rowNorm;
            
            for(int j = 0; j < 4; j++){
                m.getSubmatrixAsArray(j, 0, j, 3, row);
                rowNorm = com.irurueta.algebra.Utils.normF(row);
                for(int i = 0; i < 4; i++)
                    m.setElementAt(j, i, m.getElementAt(j, i) / rowNorm);
                b[j] /= rowNorm;
            }
            
            double[] params = com.irurueta.algebra.Utils.solve(m, b);
            
            double g = params[0]; //g = -cx
            double h = params[1]; //h = -cy
            double i = params[2]; //i = -cz
            double j = params[3]; //j = cx^2 + cy^2 + cz^2 - R^2
            
            //compute center
            double inhomCx = -g;
            double inhomCy = -h;
            double inhomCz = -i;
            Point3D center = new InhomogeneousPoint3D(inhomCx, inhomCy, 
                    inhomCz);
            
            //compute radius
            double radius = Math.sqrt(inhomCx * inhomCx + inhomCy * inhomCy + 
                    inhomCz * inhomCz - j);
            
            setCenterAndRadius(center, radius);
        }catch(AlgebraException e){
            throw new CoplanarPointsException(e);
        }
    }
    
    /**
     * Returns volume of a sphere having provided radius
     * @param radius Radius of a sphere
     * @return Volume of a sphere having provided radius
     * @throws IllegalArgumentException Raised if provided radius is negative
     */
    public static double volume(double radius) throws IllegalArgumentException{
        if(radius < MIN_RADIUS) throw new IllegalArgumentException();        
        return 4.0 / 3.0 * Math.PI * radius * radius * radius;
    }
    
    /**
     * Returns volume of this sphere
     * @return Volume of this sphere
     */
    public double getVolume(){
        return volume(mRadius);
    }
    
    /**
     * Returns surface of a sphere having provided radius
     * @param radius Radius of a sphere
     * @return Surface of a sphere having provided radius
     * @throws IllegalArgumentException Raised if provided radius is negative
     */
    public static double surface(double radius) 
            throws IllegalArgumentException{
        if(radius < MIN_RADIUS) throw new IllegalArgumentException();        
        return 4.0 * Math.PI * radius * radius;
    }
    
    /**
     * Returns surface of this sphere
     * @return Surface of this sphere
     */
    public double getSurface(){
        return surface(mRadius);
    }
    
    /**
     * Determines if provided point is inside this sphere or not up to a certain
     * threshold.
     * If provided threshold is positive, the sphere behaves as if it was a 
     * larger sphere increased by threshold amount, if provided threshold is
     * negative, the sphere behaves as if it was a smaller sphere decreased by
     * threshold amount in radius.
     * @param point Point to be checked
     * @param threshold Threshold to determine if point is inside or not
     * @return True if point is considered to be inside this sphere, false
     * otherwise
     */
    public boolean isInside(Point3D point, double threshold){
        return point.distanceTo(mCenter) - threshold <= mRadius;
    }
    
    /**
     * Determines if provided point is inside this sphere or not.
     * @param point Point to be checked
     * @return True if point is considered to be inside this sphere, false
     * otherwise
     */    
    public boolean isInside(Point3D point){
        return isInside(point, 0.0);
    }
            
    /**
     * Returns distance from provided point to the closest point located in the
     * sphere boundary.
     * Returned distance will be negative when point is inside of sphere, and
     * positive otherwise
     * @param point Point to be checked
     * @return Distance from point to sphere boundary
     */
    public double getSignedDistance(Point3D point){
        return signedDistance(this, point);
    }
    
    /**
     * Returns distance from provided point to the closest point located in
     * provided sphere boundary.
     * Returned distance will be negative when point is inside of sphere, and
     * positive otherwise
     * @param sphere A sphere
     * @param point Point to be checked
     * @return Distance from point to provided sphere boundary
     */    
    public static double signedDistance(Sphere sphere, Point3D point) {
        return point.distanceTo(sphere.getCenter()) - sphere.getRadius();
    }    
    
    /**
     * Returns distance from provided point to the closest point located in the
     * sphere boundary.
     * @param point Point to be checked
     * @return Distance from point to sphere boundary
     */    
    public double getDistance(Point3D point){
        return Math.abs(getSignedDistance(point));
    }
    
    /**
     * Returns distance from provided point to the closest point located in
     * provided sphere boundary.
     * @param sphere A sphere
     * @param point Point to be checked
     * @return Distance from point to provided sphere boundary
     */    
    public static double distance(Sphere sphere, Point3D point){
        return Math.abs(signedDistance(sphere, point));
    }
    
    /**
     * Returns closest point to provided point that is located in this sphere
     * boundary.
     * @param point A point to be checked
     * @return Closest point laying in sphere boundary
     * @throws UndefinedPointException Raised if provided point is at sphere 
     * center or very close to it.
     */    
    public Point3D getClosestPoint(Point3D point) 
            throws UndefinedPointException{
        Point3D result = Point3D.create();
        closestPoint(point, result);
        return result;
    }
    
    /**
     * Computes closest point to provided point that is located in this sphere
     * boundary and stores the result in provided result instance
     * @param point A point to be checked
     * @param result Instance where result will be stored.
     * @throws UndefinedPointException Raised if provided point is at sphere
     * center or very close to ti.
     */    
    public void closestPoint(Point3D point, Point3D result) 
            throws UndefinedPointException{
        
        double directionX = point.getInhomX() - mCenter.getInhomX();
        double directionY = point.getInhomY() - mCenter.getInhomY();
        double directionZ = point.getInhomZ() - mCenter.getInhomZ();
        //normalize direction and multiply by radius to set result as locus of
        //circle
        double norm = Math.sqrt(directionX * directionX + 
                directionY * directionY + directionZ * directionZ);
        
        //check if point is at center or very close to center, in that case the
        //closest point cannot be found (would be all points of a circle)
        if(norm < EPS) throw new UndefinedPointException();
        
        directionX *= mRadius / norm;
        directionY *= mRadius / norm;
        directionZ *= mRadius / norm;
        
        result.setInhomogeneousCoordinates(mCenter.getInhomX() + directionX, 
                mCenter.getInhomY() + directionY,
                mCenter.getInhomZ() + directionZ);        
    }
    
    /**
     * Determines whether provided point lies at sphere boundary or not up to
     * a certain threshold.
     * @param point Point to be checked
     * @param threshold A small threshold to determine whether point lies at
     * sphere boundary
     * @return True if point lies at sphere boundary, false otherwise
     * @throws IllegalArgumentException Raised if provided threshold is negative
     */    
    public boolean isLocus(Point3D point, double threshold) 
            throws IllegalArgumentException{
        if(threshold < MIN_THRESHOLD) throw new IllegalArgumentException();
        
        return Math.abs(point.distanceTo(mCenter) - mRadius) <= threshold;        
    }    
    
    /**
     * Determines whether provided point lies at sphere boundary or not.
     * @param point Point to be checked
     * @return True if point lies at sphere boundary, false otherwise
     */    
    public boolean isLocus(Point3D point){
        return isLocus(point, DEFAULT_THRESHOLD);
    }
    
    /**
     * Returns a plane tangent to this sphere at provided point. Provided point
     * must be locus of this sphere, otherwise a NotLocusException will be 
     * thrown
     * @param point a locus point of this sphere
     * @return a 3D plane tangent to this sphere at provided point
     * @throws NotLocusException if provided point is not locus of this sphere 
     * up to DEFAULT_THRESHOLD
     */
    public Plane getTangentPlaneAt(Point3D point) throws NotLocusException{
        return getTangentPlaneAt(point, DEFAULT_THRESHOLD);
    }
    
    /**
     * Returns a plane tangent to this sphere at provided point. Provided point
     * must be locus of this sphere, otherwise a NotLocusException will be 
     * thrown
     * @param point a locus point of this sphere
     * @param threshold threshold to determine if provided point is locus
     * @return a 3D plane tangent to this circle at provided point
     * @throws NotLocusException if provided point is not locus of this sphere
     * up to provided threshold
     * @throws IllegalArgumentException if provided threshold is negative
     */
    public Plane getTangentPlaneAt(Point3D point, double threshold)
            throws NotLocusException, IllegalArgumentException{
        Plane plane = new Plane();
        tangentPlaneAt(point, plane, threshold);
        return plane;
    }
    
    /**
     * Computes a plane tangent to this sphere at provided point. Provided point
     * must be locus of this sphere, otherwise a NotLocusException will be 
     * thrown
     * @param point a locus point of this sphere
     * @param plane instance of a 3D plane where result will be stored
     * @param threshold threshold to determine if provided point is locus
     * @throws NotLocusException if provided point is not locus of this sphere
     * @throws IllegalArgumentException if provided threshold is negative
     */
    public void tangentPlaneAt(Point3D point, Plane plane, double threshold)
            throws NotLocusException, IllegalArgumentException{
        
        if(!isLocus(point, threshold)) throw new NotLocusException();
        
        point.normalize();
        mCenter.normalize();
        
        //Q =   [a      d       f       g]
        //      [d      b       e       h]
        //      [f      e       c       i]
        //      [g      h       i       j]

        //Q =   [1      0       0       -cx]
        //      [0      1       0       -cy]
        //      [0      0       1       -cz]
        //      [-cx    -cy     -cz     cx^2 + cy^2 + cz^2 - r^2]
        
        //a = b = c = 1.0
        //d = e = f = 0.0
        //g = -cx, h = -cy, i = -cz
        //j = cx^2 + cy^2 + cz^2 - r^2
        
        //Hence plane is P = Q * p, where Q is the sphere quadric and p is
        //a point in the locus of the sphere
        
        double homX = point.getHomX();
        double homY = point.getHomY();
        double homZ = point.getHomZ();
        double homW = point.getHomW();
        double cx = mCenter.getInhomX();
        double cy = mCenter.getInhomY();
        double cz = mCenter.getInhomZ();
        double quadricG = -cx;
        double quadricH = -cy;
        double quadricI = -cz;
        double quadricJ = cx * cx + cy * cy + cz * cz - mRadius * mRadius;
        
        double planeA = homX + quadricG * homW;
        double planeB = homY + quadricH * homW;
        double planeC = homZ + quadricI * homW;
        double planeD = quadricG * homX + quadricH * homY + quadricI * homZ +
                quadricJ * homW;
        plane.setParameters(planeA, planeB, planeC, planeD);
    }
    
    /**
     * Converts this sphere into a quadric.
     * Quadrics are a more general representation of spheres.
     * @return A quadric representing this circle
     */
    public Quadric toQuadric(){
        mCenter.normalize();
        //use inhomogeneous center coordinates
        double cx = mCenter.getInhomX();
        double cy = mCenter.getInhomY();
        double cz = mCenter.getInhomZ();
        
        double a = 1.0;
        double b = 1.0;
        double c = 1.0;
        double d = 0.0;
        double e = 0.0;
        double f = 0.0;
        double g = -cx;
        double h = -cy;
        double i = -cz;
        double j = cx * cx + cy * cy + cz * cz - mRadius * mRadius;
        
        return new Quadric(a, b, c, d, e, f, g, h, i, j);
    }
    
    /**
     * Sets parameters of this sphere from a valid quadric corresponding to a 
     * sphere
     * @param quadric quadric to set parameters from
     * @throws IllegalArgumentException if provided quadric is not a sphere
     */
    public final void setFromQuadric(Quadric quadric) 
            throws IllegalArgumentException{
        boolean isSphere = quadric.getA() == quadric.getB() && 
                quadric.getB() == quadric.getC() && quadric.getA() != 0.0 &&
                quadric.getD() == 0.0 && quadric.getE() == 0.0 && 
                quadric.getF() == 0.0;
        
        if(!isSphere) throw new IllegalArgumentException();
        
        quadric.normalize();
        
        double a = quadric.getA();
        //normalize parameters so that
        //a = b = c = 1.0
        //d = e = f = 0.0
        //g = -cx, h = -cy, i = -cz
        //j = cx^2 + cy^2 + cz^2 - r^2
        double normG = quadric.getG() / a;
        double normH = quadric.getH() / a;
        double normI = quadric.getI() / a;
        double normJ = quadric.getJ() / a;
        
        double cx = -normG;
        double cy = -normH;
        double cz = -normI;
        double radius = Math.sqrt(cx * cx + cy * cy + cz * cz - normJ);
        
        mCenter = new InhomogeneousPoint3D(cx, cy, cz);
        mRadius = radius;
    }
}
