/**
 * @file
 * This file contains implementation of 
 * com.irurueta.geometry.Polygon3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 */
package com.irurueta.geometry;

import java.io.Serializable;
import java.util.Iterator;
import java.util.List;

/**
 *  This class defines a polygon in 3D space
 */
public class Polygon3D implements Serializable{
    
    /**
     * List containing vertices of this polygon. Each vertex is a 3D point
     */
    private List<Point3D> mVertices;
    
    /**
     * Boolean indicating whether polygon has already been triangulated
     */
    private boolean mTriangulated;
    
    /**
     * List containing triangles found after triangulating this polygon.
     * Initially this list will be null until triangulation is done
     */
    private List<Triangle3D> mTriangles;
    
    /**
     * Method to do triangulation
     */
    private TriangulatorMethod mTriangulatorMethod;
    
    /**
     * Default threshold value. Thresholds are used to determine whether a point
     * lies inside the polygon or not, or if it's locus or not, etc.
     */
    public static final double DEFAULT_THRESHOLD = 1e-9;
    
    /**
     * Minimum allowed threshold value
     */
    public static final double MIN_THRESHOLD = 0.0;
    
    /**
     * Minimum number of vertices that a polygon is allowed to have
     */
    public static final int MIN_VERTICES = 3;
    
    /**
     * Constant defining inhomogeneous coordinates
     */
    public static final int INHOM_COORDS = 3;
    
    /**
     * Default method for triangulation
     */
    public static final TriangulatorMethod DEFAULT_TRIANGULATOR_METHOD = 
            TriangulatorMethod.VAN_GOGH_TRIANGULATOR;
    
    
    /**
     * Constructor
     * @param vertices List of vertices forming this polygon.
     * @throws NotEnoughVerticesException Raised if list does not contain enough
     * vertices
     * @see #MIN_VERTICES
     */
    public Polygon3D(List<Point3D> vertices) throws NotEnoughVerticesException{
        setVertices(vertices);
        mTriangulatorMethod = DEFAULT_TRIANGULATOR_METHOD;
    }
    
    /**
     * Returns triangulator method. Triangulator method determines the way a
     * polygon is divided into triangles.
     * If none has been provided DEFAULT_TRIANGULATOR_METHOD will be returned
     * @return Triangulator method
     */
    public TriangulatorMethod getTriangulatorMethod(){
        return mTriangulatorMethod;
    }
    
    /**
     * Sets triangulator method. A triangulator method determines the way a
     * polygon is divided into triangles.
     * @param triangulatorMethod A triangulator method
     */
    public void setTriangulatorMethod(TriangulatorMethod triangulatorMethod){
        mTriangulatorMethod = triangulatorMethod;
    }
    
    /**
     * Returns the list of vertices forming this polygon
     * @return List of vertices
     */
    public List<Point3D> getVertices(){
        return mVertices;
    }
    
    /**
     * Sets list of vertices forming this polygon
     * @param vertices List of vertices
     * @throws NotEnoughVerticesException Raised if provided list does not have
     * enough vertices
     * @see #MIN_VERTICES
     */
    public final void setVertices(List<Point3D> vertices) 
            throws NotEnoughVerticesException{
        if(vertices.size() < MIN_VERTICES) 
            throw new NotEnoughVerticesException();
        
        mVertices = vertices;
        mTriangulated = false;
        mTriangles = null;
    }
    
    /**
     * Determines whether this polygon has already been triangulated.
     * A polygon will only need to be triangulated once, unless the list of
     * vertices is reset.
     * @return True if polygon has already been triangulated, false otherwise
     */
    public boolean isTriangulated(){
        return mTriangulated;
    }
    
    /**
     * Returns a list of triangles forming this polygon.
     * This method checks whether this polygon has already been triangulated,
     * if not, it performs triangulation first
     * @return A list of triangles forming this polygon
     * @throws TriangulatorException Raised if triangulation was needed and 
     * failed
     */
    public List<Triangle3D> getTriangles() throws TriangulatorException{
        if(!isTriangulated()) triangulate();
        return mTriangles;
    }
     
    /**
     * Returns signed area of this polygon.
     * The sign of the area determines whether vertices of the polygon are 
     * provided in clockwise (negative sign) or clockwise (positive sign) order.
     * @return Signed area of this polygon
     */
    public double getArea(){
        Point3D origin = mVertices.get(0);
        double inhomX0 = origin.getInhomX();
        double inhomY0 = origin.getInhomY();
        double inhomZ0 = origin.getInhomZ();
        
        Iterator<Point3D> iterator = mVertices.iterator();
        
        Point3D prevPoint = iterator.next(); //because there are at least 3 
                                            //vertices
        Point3D curPoint;
        double avgX = 0.0, avgY = 0.0, avgZ = 0.0;
        while(iterator.hasNext()){
            curPoint = iterator.next();
            
            double inhomX1 = prevPoint.getInhomX();
            double inhomY1 = prevPoint.getInhomY();
            double inhomZ1 = prevPoint.getInhomZ();
        
            double inhomX2 = curPoint.getInhomX();
            double inhomY2 = curPoint.getInhomY();
            double inhomZ2 = curPoint.getInhomZ();
        

            //compute cross product of ab = (prevPoint - origin) and 
            //ac = (curPoint - origin)
            double abX = inhomX1 - inhomX0;
            double abY = inhomY1 - inhomY0;
            double abZ = inhomZ1 - inhomZ0;
        
            double acX = inhomX2 - inhomX0;
            double acY = inhomY2 - inhomY0;
            double acZ = inhomZ2 - inhomZ0;
        
            double crossX = abY * acZ - abZ * acY;
            double crossY = abZ * acX - abX * acZ;
            double crossZ = abX * acY - abY * acX;

            avgX += crossX;
            avgY += crossY;
            avgZ += crossZ;
                        
            prevPoint = curPoint;
        }
        
        return 0.5 * Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
    }
        
    /**
     * Returns perimeter of this polygon.
     * The perimeter is computed as the sum of the distances between consecutive
     * pairs of vertices
     * @return Perimeter of this polygon.
     */
    public double getPerimeter(){
        //iterate over all vertices and compute their distance
        Iterator<Point3D> iterator = mVertices.iterator();
        Point3D prevPoint = iterator.next();
        Point3D point;
        double perimeter = 0.0;
        while(iterator.hasNext()){
            point = iterator.next();
            perimeter += prevPoint.distanceTo(point);
            prevPoint = point;
        }
        //get distance from last point with first one
        perimeter += prevPoint.distanceTo(mVertices.get(0));
        return perimeter;
    }
    
    /**
     * Determines if provided point lies within the region defined by this
     * polygon.
     * Notice that this method is only ensured to work for polygons having no
     * holes or crossing borders. It will safely work on any other polygon,
     * no matter if it is regular, non-regular, convex or concave
     * @param point Point to be checked
     * @return True if point lies within the area defined by this polygon, false
     * otherwise
     * @throws TriangulatorException Raised if triangulation was required but
     * failed
     */
    public boolean isInside(Point3D point) throws TriangulatorException{
        return isInside(point, DEFAULT_THRESHOLD);
    }

    /**
     * Determines if provided point lies within the region defined by this
     * polygon.
     * Notice that this method is only ensured to work for polygons having no
     * holes or crossing borders. It will safely work on any other polygon,
     * no matter if it is regular, non-regular, convex or concave.
     * @param point Point to be checked.
     * @param threshold Threshold to determine whether point lies inside this
     * polygon. Usually this value should be small.
     * @return True if point lies within the area defined by this polygon, false
     * otherwise
     * @throws IllegalArgumentException Raised if provided threshold is negative
     * @throws TriangulatorException Raised if triangulation was required but
     * failed
     */
    public boolean isInside(Point3D point, double threshold)
            throws IllegalArgumentException, TriangulatorException{
        if(threshold < MIN_THRESHOLD) throw new IllegalArgumentException();
        
        for(Triangle3D triangle : getTriangles()){
            if(triangle.isInside(point, threshold)) return true;
        }
        return false;
    }
    
    /**
     * Returns the center of this polygon.
     * The center is the average point among all the vertices of this polygon.
     * The center is not ensure to lie within the area formed by this polygon.
     * @return Center of this polygon.
     */
    public Point3D getCenter(){
        Point3D result = Point3D.create();
        center(result);
        return result;        
    }
    
    /**
     * Computes the center of this polygon.
     * The center is the average point among all the vertices of this polygon.
     * The center is not ensured to lie within the area formed by this polygon.
     * @param result Instance where the computed center will be stored.
     */
    public void center(Point3D result){
        //compute average location of all vertices
        double inhomX = 0.0, inhomY = 0.0, inhomZ = 0.0;
        int total = mVertices.size();
        
        for(Point3D point : mVertices){
            inhomX += point.getInhomX() / (double)total;
            inhomY += point.getInhomY() / (double)total;
            inhomZ += point.getInhomZ() / (double)total;
        }
        result.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
    }
             
    /**
     * Determines whether provided point is locus of the borders defined by
     * the vertices of this polygon. A point will be locus if it lies in the
     * line defined by two consecutive vertices up to a certain threshold of
     * error
     * @param point Point to be checked
     * @param threshold Threshold of allowed error. This should usually be a 
     * small value
     * @return True if provided point lies in a border of this polygon, false
     * otherwise
     * @throws IllegalArgumentException Raised if provided threshold is negative
     * @throws TriangulatorException Raised if triangulation was required but
     * failed
     */
    public boolean isLocus(Point3D point, double threshold) 
            throws IllegalArgumentException, TriangulatorException{
        if(threshold < MIN_THRESHOLD) throw new IllegalArgumentException();
        
        //normalize point to increase accuracy
        point.normalize();
        
        Iterator<Point3D> iterator = mVertices.iterator();
        Point3D prevPoint = iterator.next(); //it's ok because there are at 
                                            //least 3 vertices
        prevPoint.normalize(); //normalize to increase accuracy
        
        Point3D curPoint;
        while(iterator.hasNext()){
            curPoint = iterator.next();
            curPoint.normalize(); //normalize to increase accuracy
            if(point.isBetween(prevPoint, curPoint, threshold)) return true;            
            prevPoint = curPoint;
        }
        
        //check last point with first
        return point.isBetween(prevPoint, mVertices.get(0), threshold);
    } 
    
    /**
     * Determines whether provided point is locus of the borders defined by the
     * vertices of this polygon. A point will be locus if it lies in the line
     * defined by two consecutive vertices
     * @param point Point to be checked
     * @return True if provided point lies in a border of this polygon, false
     * otherwise
     * @throws TriangulatorException Raised if triangulation was required but
     * failed
     */
    public boolean isLocus(Point3D point) throws TriangulatorException{
        return isLocus(point, DEFAULT_THRESHOLD);
    }  
    
    /**
     * Returns the shortest distance from provided point to a border of this
     * polygon. Note that borders are segments defined by consecutive vertices
     * @param point Point to be checked.
     * @return Shortest distance from provided point to this polygon.
     * @throws CoincidentPointsException Raised if points in a polygon are too
     * close. This usually indicates numerical instability or polygon degeneracy
     */
    public double getShortestDistance(Point3D point) 
            throws CoincidentPointsException{        
        //iterate over all vertices and compute their distance
        Iterator<Point3D> iterator = mVertices.iterator();
        Point3D prevPoint = iterator.next();
        prevPoint.normalize(); //to increase accuracy
        Point3D curPoint;
        double bestDist = Double.MAX_VALUE, dist;
        boolean found = false;
        Line3D line = null;
        Point3D pointInLine = Point3D.create();
        
        while(iterator.hasNext()){
            curPoint = iterator.next();
            curPoint.normalize(); //to increase accuracy
            
            //check if point lies in the segment of the boundary of this polygon
            if(point.isBetween(curPoint, prevPoint)) return 0.0;
            
            if(line == null){
                line = new Line3D(curPoint, prevPoint);
            }else{
                line.setPlanesFromPoints(curPoint, prevPoint);
            }
            line.normalize(); //to increase accuracy
            
            //find closest point to line
            line.closestPoint(point, pointInLine);
            pointInLine.normalize(); //to increase accuracy
            
            if(pointInLine.isBetween(curPoint, prevPoint)){
                //closest point lies within segment of polygon boundary, so we
                //keep distance
                dist = point.distanceTo(pointInLine);
                if(dist < bestDist){
                    //a better point has been found
                    bestDist = dist;
                    found = true;
                }
            }
            
            prevPoint = curPoint;
        }
        
        //try last vertex with first
        //check if point lies in the segment of the boundary of this polygon
        Point3D first = mVertices.get(0);
        if(point.isBetween(prevPoint, first)) return 0.0;
            
        if(line == null){
            line = new Line3D(prevPoint, first);
        }else{
            line.setPlanesFromPoints(prevPoint, first);
        }
        line.normalize(); //to increase accuracy
            
        //find closest point to line
        line.closestPoint(point, pointInLine);
        pointInLine.normalize(); //to increase accuracy
            
        if(pointInLine.isBetween(prevPoint, first)){
            //closest point lies within segment of polygon boundary, so we
            //keep distance
            dist = point.distanceTo(pointInLine);
            if(dist < bestDist){
                //a better point has been found
                bestDist = dist;
                found = true;
            }
        }
        
        
        
        if(!found){
            //no closest point was found on a segment belonging to polygon 
            //boundary so we search for the closest vertex
            iterator = mVertices.iterator();
            while(iterator.hasNext()){
                //a better vertex has been found
                curPoint = iterator.next();
                dist = point.distanceTo(curPoint);
                if(dist < bestDist) bestDist = dist;
            }
        }
        
        return bestDist;
    }
    
    /**
     * Returns the closest point to provided point that is locus of this
     * polygon (i.e. lies on a border of this polygon)
     * @param point Point to be checked
     * @return Closest point being locus of this polygon
     * @throws CoincidentPointsException Raised if points in a polygon are too
     * close. This usually indicates numerical instability or polygon degeneracy
     */
    public Point3D getClosestPoint(Point3D point) 
            throws CoincidentPointsException{
        Point3D result = Point3D.create();
        closestPoint(point, result);
        return result;        
    }
    
    /**
     * Computes the closes point to provided point that is locus of this
     * polygon (i.e. lies on a border of this polygon)
     * @param point Point to be checked
     * @param result Instance where the closest point will be stored
     * @throws CoincidentPointsException Raised if points in a polygon are too
     * close. This usually indicates numerical instability or polygon degeneracy
     */
    public void closestPoint(Point3D point, Point3D result) 
            throws CoincidentPointsException{
        //iterate over all vertices and compute their distance
        Iterator<Point3D> iterator = mVertices.iterator();
        Point3D prevPoint = iterator.next();
        prevPoint.normalize(); //to increase accuracy
        
        Point3D curPoint;
        double bestDist = Double.MAX_VALUE, dist;
        boolean found = false;
        Line3D line = null;
        Point3D pointInLine = Point3D.create();
        
        while(iterator.hasNext()){
            curPoint = iterator.next();
            curPoint.normalize(); //to increase accuracy
            
            //check if point lies in the segment of the boundary of this polygon
            if(point.isBetween(curPoint, prevPoint)){
                result.setCoordinates(point);
                return;
            }
            
            if(line == null){
                line = new Line3D(curPoint, prevPoint);
            }else{
                line.setPlanesFromPoints(curPoint, prevPoint);
            }
            line.normalize(); //to increase accuracy
            
            //find closest point to line
            line.closestPoint(point, pointInLine);
            pointInLine.normalize(); //to increase accuracy
            
            if(pointInLine.isBetween(curPoint, prevPoint)){
                //closest point lies within segment of polygon boundary, so we
                //keep distance and point
                dist = point.distanceTo(pointInLine);
                if(dist < bestDist){
                    //a better point has been found
                    bestDist = dist; 
                    result.setCoordinates(pointInLine);
                    found = true;
                }
            }
            
            prevPoint = curPoint;
        }
        
        //try last vertex with first
        //check if point lies in the segment of the boundary of this polygon
        Point3D first = mVertices.get(0);
        if(point.isBetween(prevPoint, first)){
            result.setCoordinates(point);
            return;
        }
            
        if(line == null){
            line = new Line3D(prevPoint, first);
        }else{
            line.setPlanesFromPoints(prevPoint, first);
        }
        line.normalize(); //to increase accuracy
            
        //find closest point to line
        line.closestPoint(point, pointInLine);
        pointInLine.normalize(); //to increase accuracy
            
        if(pointInLine.isBetween(prevPoint, first)){
            //closest point lies within segment of polygon boundary, so we
            //keep distance
            dist = point.distanceTo(pointInLine);
            if(dist < bestDist){
                //a better point has been found
                bestDist = dist;
                result.setCoordinates(pointInLine);
                found = true;
            }
        }        
        
        
        if(!found){
            //no closest point was found on a segment belonging to polygon 
            //boundary so we search for the closest vertex
            iterator = mVertices.iterator();
            while(iterator.hasNext()){
                curPoint = iterator.next();
                dist = point.distanceTo(curPoint);
                if(dist < bestDist){
                    //a better vertex has been found
                    bestDist = dist;
                    result.setCoordinates(curPoint);
                }
            }
        }
    }
    
    /**
     * Triangulates this polygon using this polygon's triangulator method.
     * A polygon only will be triangulated once when required or this method is
     * called.
     * This method will make no action if a polygon is already triangulated 
     * unless it's vertices are reset.
     * @throws TriangulatorException Raised if triangulation failed
     * @see #getTriangulatorMethod
     * @see #setTriangulatorMethod(TriangulatorMethod)
     */
    public void triangulate() throws TriangulatorException{
        if(!mTriangulated){
            Triangulator3D triangulator = Triangulator3D.create(
                    mTriangulatorMethod);
            mTriangles = triangulator.triangulate(mVertices);
            mTriangulated = true;
        }
    }
    
    /**
     * Computes the average orientation of a 3D polygon as an array 
     * containing the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @param vertices List of vertices forming a polygon in 3D
     * @param result Array where the estimated orientation will be stored
     * @param threshold Threshold to determine whether the orientation can be
     * estimated or not. Because the estimated orientation needs to be 
     * normalized, it will only be possible to do so when norm has a reasonable
     * value. If estimated norm happens to be smaller than provided value, then
     * it will be assumed that polygon contains a degeneracy or that a 
     * consecutive pair of vertices are coincident.
     * @throws IllegalArgumentException Raised if provided result array does not
     * have length 3, or if the list of provided vertices does not contain at
     * least three vertices, or if provided threshold is negative.
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */
    public static void orientation(List<Point3D> vertices, double[] result,
            double threshold) throws IllegalArgumentException, 
            CoincidentPointsException{
        int numVertices = vertices.size();
        if(numVertices < MIN_VERTICES) throw new IllegalArgumentException();        
        if(result.length != INHOM_COORDS) throw new IllegalArgumentException();
        if(threshold < MIN_THRESHOLD) throw new IllegalArgumentException();
        
        
        double avgX = 0.0, avgY = 0.0, avgZ = 0.0;
        
        Iterator<Point3D> iterator = vertices.iterator();
        Point3D prevPoint = iterator.next();
        Point3D origin = prevPoint, curPoint;
        double inhomX0 = origin.getInhomX();
        double inhomY0 = origin.getInhomY();
        double inhomZ0 = origin.getInhomZ();
        
        
        while(iterator.hasNext()){
            curPoint = iterator.next();
            double inhomX1 = prevPoint.getInhomX();
            double inhomY1 = prevPoint.getInhomY();
            double inhomZ1 = prevPoint.getInhomZ();
            
            double inhomX2 = curPoint.getInhomX();
            double inhomY2 = curPoint.getInhomY();
            double inhomZ2 = curPoint.getInhomZ();
            
            //compute cross product of ab = (vertex1 - origin) and ac = (vertex2
            //- origin)
            double abX = inhomX1 - inhomX0;
            double abY = inhomY1 - inhomY0;
            double abZ = inhomZ1 - inhomZ0;
        
            double acX = inhomX2 - inhomX0;
            double acY = inhomY2 - inhomY0;
            double acZ = inhomZ2 - inhomZ0;
        
            double crossX = abY * acZ - abZ * acY;
            double crossY = abZ * acX - abX * acZ;
            double crossZ = abX * acY - abY * acX;
            
            avgX += crossX;
            avgY += crossY;
            avgZ += crossZ;
            
            prevPoint = curPoint;
        }

        double norm = Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
        
        if(norm < threshold) throw new CoincidentPointsException();
        
        avgX /= norm;
        avgY /= norm;
        avgZ /= norm;
        
        result[0] = avgX;
        result[1] = avgY;
        result[2] = avgZ;        
    }
    
    /**
     * Computes the average orientation of a 3D polygon as an array 
     * containing the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @param vertices List of vertices forming a polygon in 3D
     * @param result Array where the estimated orientation will be stored
     * @throws IllegalArgumentException Raised if provided result array does not
     * have length 3, or if the list of provided vertices does not contain at
     * least three vertices.
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */    
    public static void orientation(List<Point3D> vertices, double[] result) 
            throws IllegalArgumentException, CoincidentPointsException{
        orientation(vertices, result, DEFAULT_THRESHOLD);
    }    
    
    /**
     * Returns the average orientation of a 3D polygon as an array containing 
     * the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @param vertices List of vertices forming a polygon in 3D
     * @param threshold Threshold to determine whether the orientation can be
     * estimated or not. Because the estimated orientation needs to be 
     * normalized, it will only be possible to do so when norm has a reasonable
     * value. If estimated norm happens to be smaller than provided value, then
     * it will be assumed that polygon contains a degeneracy or that a 
     * consecutive pair of vertices are coincident.
     * @return Array containing estimated orientation
     * @throws IllegalArgumentException Raised if the list of provided vertices 
     * does not contain at least three vertices, or if provided threshold is 
     * negative.
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */    
    public static double[] orientation(List<Point3D> vertices, double threshold) 
            throws IllegalArgumentException, CoincidentPointsException{
        double[] out = new double[INHOM_COORDS];
        orientation(vertices, out, threshold);
        return out;
    }
    
    /**
     * Returns the average orientation of a 3D polygon as an array containing 
     * the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @param vertices List of vertices forming a polygon in 3D
     * @return Array containing estimated orientation
     * @throws IllegalArgumentException Raised if the list of provided vertices 
     * does not contain at least three vertices
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */        
    public static double[] orientation(List<Point3D> vertices) 
            throws IllegalArgumentException, CoincidentPointsException{
        return orientation(vertices, DEFAULT_THRESHOLD);
    }
    
    /**
     * Computes the average orientation of provided 3D polygon as an array 
     * containing the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @param polygon A polygon in 3D
     * @param result Array where the estimated orientation will be stored
     * @param threshold Threshold to determine whether the orientation can be
     * estimated or not. Because the estimated orientation needs to be 
     * normalized, it will only be possible to do so when norm has a reasonable
     * value. If estimated norm happens to be smaller than provided value, then
     * it will be assumed that polygon contains a degeneracy or that a 
     * consecutive pair of vertices are coincident.
     * @throws IllegalArgumentException Raised if provided result array does not
     * have length 3 or if provided threshold is negative.
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */    
    public static void orientation(Polygon3D polygon, double[] result, 
            double threshold) throws IllegalArgumentException, 
            CoincidentPointsException{
        orientation(polygon.getVertices(), result, threshold);
    }

    /**
     * Computes the average orientation of provided 3D polygon as an array 
     * containing the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @param polygon A polygon in 3D
     * @param result Array where the estimated orientation will be stored
     * @throws IllegalArgumentException Raised if provided result array does not
     * have length 3
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */        
    public static void orientation(Polygon3D polygon, double[] result) 
            throws IllegalArgumentException, CoincidentPointsException{
        orientation(polygon, result, DEFAULT_THRESHOLD);
    }    
    
    /**
     * Returns the average orientation of provided 3D polygon as an array 
     * containing the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @param polygon A polygon in 3D
     * @param threshold Threshold to determine whether the orientation can be
     * estimated or not. Because the estimated orientation needs to be 
     * normalized, it will only be possible to do so when norm has a reasonable
     * value. If estimated norm happens to be smaller than provided value, then
     * it will be assumed that polygon contains a degeneracy or that a 
     * consecutive pair of vertices are coincident.
     * @return Array containing estimated orientation
     * @throws IllegalArgumentException Raised if provided threshold is 
     * negative.
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */        
    public static double[] orientation(Polygon3D polygon, double threshold)
        throws IllegalArgumentException, CoincidentPointsException{
        return orientation(polygon.getVertices(), threshold);
    }
    
    /**
     * Returns the average orientation of provided 3D polygon as an array 
     * containing the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @param polygon A polygon in 3D
     * @return Array containing estimated orientation
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */            
    public static double[] orientation(Polygon3D polygon)
        throws CoincidentPointsException{
        return orientation(polygon, DEFAULT_THRESHOLD);
    }    
    
    /**
     * Computes the average orientation of this polygon as an array containing 
     * the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @param result Array where the estimated orientation will be stored
     * @param threshold Threshold to determine whether the orientation can be
     * estimated or not. Because the estimated orientation needs to be 
     * normalized, it will only be possible to do so when norm has a reasonable
     * value. If estimated norm happens to be smaller than provided value, then
     * it will be assumed that polygon contains a degeneracy or that a 
     * consecutive pair of vertices are coincident.
     * @throws IllegalArgumentException Raised if provided result array does not
     * have length 3 or if provided threshold is negative.
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */        
    public void orientation(double[] result, double threshold)
        throws IllegalArgumentException, CoincidentPointsException{
        orientation(mVertices, result, threshold);
    }
    
    /**
     * Computes the average orientation of provided 3D polygon as an array 
     * containing the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @param result Array where the estimated orientation will be stored
     * @throws IllegalArgumentException Raised if provided result array does not
     * have length 3
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */        
    public void orientation(double[] result) throws IllegalArgumentException, 
            CoincidentPointsException{
        orientation(result, DEFAULT_THRESHOLD);
    }
    
    /**
     * Returns the average orientation of provided 3D polygon as an array 
     * containing the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @param threshold Threshold to determine whether the orientation can be
     * estimated or not. Because the estimated orientation needs to be 
     * normalized, it will only be possible to do so when norm has a reasonable
     * value. If estimated norm happens to be smaller than provided value, then
     * it will be assumed that polygon contains a degeneracy or that a 
     * consecutive pair of vertices are coincident.
     * @return Array containing estimated orientation
     * @throws IllegalArgumentException Raised if provided threshold is 
     * negative.
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */        
    public double[] getOrientation(double threshold) 
            throws IllegalArgumentException, CoincidentPointsException{
        return orientation(mVertices, threshold);
    }

    /**
     * Returns the average orientation of provided 3D polygon as an array 
     * containing the vector coordinates of the orientation.
     * Notice that if all the vertices of the polygon lie in the same plane,
     * then the estimated orientation will be exact, otherwise an average
     * orientation will be estimated.
     * Estimated orientation will be normalized (the norm of the estimated 
     * vector will be 1)
     * @return Array containing estimated orientation
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     */            
    public double[] getOrientation() throws CoincidentPointsException{
        return getOrientation(DEFAULT_THRESHOLD);
    }
    
    /**
     * Returns the angle between two polygons, assuming that all vertices of 
     * each polygon lie on a given plane. Hence, this is equivalent to 
     * estimating the angle between the planes formed by two polygons.
     * The angle between two polygons is estimated by first estimating their
     * orientation.
     * @param polygon1 1st polygon
     * @param polygon2 2nd polygon
     * @param threshold Threshold to determine when polygon orientation can be
     * estimated.
     * @return Angle between two polygons in radians
     * @throws IllegalArgumentException Raised if provided threshold is negative
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     * @see #orientation(Polygon3D, double[])
     */
    public static double getAngleBetweenPolygons(Polygon3D polygon1, 
            Polygon3D polygon2, double threshold) 
            throws IllegalArgumentException, CoincidentPointsException{
        return getAngleBetweenPolygons(polygon1.getVertices(), 
                polygon2.getVertices(), threshold);
    }

    /**
     * Returns the angle between two polygons, assuming that all vertices of 
     * each polygon lie on a given plane. Hence, this is equivalent to 
     * estimating the angle between the planes formed by two polygons.
     * The angle between two polygons is estimated by first estimating their
     * orientation.
     * @param polygon1 1st polygon
     * @param polygon2 2nd polygon
     * @return Angle between two polygons in radians
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     * @see #orientation(Polygon3D, double[])
     */    
    public static double getAngleBetweenPolygons(Polygon3D polygon1, 
            Polygon3D polygon2) throws CoincidentPointsException{
        return getAngleBetweenPolygons(polygon1, polygon2, DEFAULT_THRESHOLD);
    }
    
    /**
     * Returns the angle between two polygons formed each of them by the 
     * corresponding list of provided vertices and assuming that all vertices of 
     * each polygon lie on a given plane. Hence, this is equivalent to 
     * estimating the angle between the planes formed by two polygons.
     * The angle between two polygons is estimated by first estimating their
     * orientation.
     * @param vertices1 Vertices of 1st polygon
     * @param vertices2 2nd polygon
     * @param threshold Threshold to determine when polygon orientation can be
     * estimated.
     * @return Angle between two polygons in radians
     * @throws IllegalArgumentException Raised if provided threshold is negative
     * or if list of vertices do not contain at least 3 vertices for each 
     * polygon
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     * @see #orientation(Polygon3D, double[])
     */    
    public static double getAngleBetweenPolygons(List<Point3D> vertices1,
            List<Point3D> vertices2, double threshold) 
            throws IllegalArgumentException, CoincidentPointsException{
        return getAngleBetweenOrientations(Polygon3D.orientation(vertices1, 
                threshold), Polygon3D.orientation(vertices2, threshold));
    }

    /**
     * Returns the angle between two polygons formed each of them by the 
     * corresponding list of provided vertices and assuming that all vertices of 
     * each polygon lie on a given plane. Hence, this is equivalent to 
     * estimating the angle between the planes formed by two polygons.
     * The angle between two polygons is estimated by first estimating their
     * orientation.
     * @param vertices1 Vertices of 1st polygon
     * @param vertices2 2nd polygon
     * @return Angle between two polygons in radians
     * @throws IllegalArgumentException Raised if list of vertices do not 
     * contain at least 3 vertices for each polygon
     * @throws CoincidentPointsException Raised usually when it is determined
     * that consecutive vertices of the polygon are too close (i.e. coincident)
     * or when there are polygon degeneracies or numerical instabilities
     * @see #orientation(Polygon3D, double[])
     */        
    public static double getAngleBetweenPolygons(List<Point3D> vertices1,
            List<Point3D> vertices2) throws IllegalArgumentException, 
            CoincidentPointsException{
        return getAngleBetweenPolygons(vertices1, vertices2, DEFAULT_THRESHOLD);
    }
    
    /**
     * Internal method to compute polygon orientation from their respective
     * orientation vectors.
     * Orientation vectors are provided as arrays and can be obtained by calling
     * orientation(Polygon3D, double[]) among other methods
     * @param orientation1 Orientation of 1st polygon
     * @param orientation2 Orientation of 2nd polygon
     * @return Angle between two polygons in radians
     * @throws IllegalArgumentException Raised if any of the orientation arrays
     * do not have length 3
     */
    private static double getAngleBetweenOrientations(double[] orientation1, 
            double[] orientation2) throws IllegalArgumentException{
        if(orientation1.length != INHOM_COORDS || 
                orientation2.length != INHOM_COORDS) 
            throw new IllegalArgumentException();
        
        double x1 = orientation1[0];
        double y1 = orientation1[1];
        double z1 = orientation1[2];
        
        double x2 = orientation2[0];
        double y2 = orientation2[1];
        double z2 = orientation2[2];
        
        double norm1 = Math.sqrt(x1 * x1 + y1 * y1 + z1 * z1);
        double norm2 = Math.sqrt(x2 * x2 + y2 * y2 + z2 * z2);
        
        double dotProduct = (x1 * x2 + y1 * y2 + z1 * z2) / (norm1 * norm2);
        
        return Math.acos(dotProduct);        
    }        
}