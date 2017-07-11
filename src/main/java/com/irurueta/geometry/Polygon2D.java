/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.Polygon2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 17, 2012
 */
package com.irurueta.geometry;

import java.io.Serializable;
import java.util.Iterator;
import java.util.List;

/**
 *  This class defines a polygon in 2D space
 */
public class Polygon2D implements Serializable{
    
    /**
     * List containing vertices of this polygon. Each vertex is a 2D point
     */
    private List<Point2D> mVertices;
    
    /**
     * Boolean indicating whether polygon has already been triangulated
     */
    private boolean mTriangulated;
    
    /**
     * List containing triangles found after triangulating this polygon.
     * Initially this list will be null until triangulation is done
     */
    private List<Triangle2D> mTriangles;
    
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
    public Polygon2D(List<Point2D> vertices) throws NotEnoughVerticesException{
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
    public List<Point2D> getVertices(){
        return mVertices;
    }
    
    /**
     * Sets list of vertices forming this polygon
     * @param vertices List of vertices
     * @throws NotEnoughVerticesException Raised if provided list does not have
     * enough vertices
     * @see #MIN_VERTICES
     */
    public final void setVertices(List<Point2D> vertices) 
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
    public List<Triangle2D> getTriangles() throws TriangulatorException{
        if(!isTriangulated()) triangulate();
        return mTriangles;
    }
     
    /**
     * Returns signed area of this polygon.
     * The sign of the area determines whether vertices of the polygon are 
     * provided in clockwise (negative sign) or clockwise (positive sign) order.
     * @return Signed area of this polygon
     */
    public double getSignedArea(){
        Iterator<Point2D> iterator = mVertices.iterator();
        
        Point2D prevPoint = iterator.next(); //because there are at least 3 
                                            //vertices
        Point2D curPoint;
        double signedArea = 0.0;
        
        while(iterator.hasNext()){
            curPoint = iterator.next();
            
            signedArea += prevPoint.getInhomX() *(curPoint.getInhomY() - 
                    prevPoint.getInhomY());
            prevPoint = curPoint;
        }
        //on last point, check previous with first
        curPoint = mVertices.get(0);
        signedArea += prevPoint.getInhomX() * (curPoint.getInhomY() - 
                    prevPoint.getInhomY());
        
        //signed area is half the sum of cross products of consecutive vertices
        return signedArea;
    }
    
    /**
     * Returns area of this polygon.
     * @return Area of this polygon.
     */
    public double getArea(){
        return Math.abs(getSignedArea());
    }
        
    /**
     * Determines whether vertices of this polygon are in clockwise order or
     * in counterclockwise order
     * @param threshold threshold to determine if vertices are in clockwise 
     * order. Usually this value is zero
     * @return True if vertices are in clockwise order, false otherwise
     */
    public boolean areVerticesClockwise(double threshold){                        
        return getSignedArea() < threshold;
    }

    /**
     * Determines whether vertices of this polygon are in clockwise order or in
     * counterclockwise order
     * @return True if vertices are in clockwise order, false otherwise
     */
    public boolean areVerticesClockwise(){
        return areVerticesClockwise(0.0); //default threshold to check sign
    }    
    
    
    /**
     * Returns perimeter of this polygon.
     * The perimeter is computed as the sum of the distances between consecutive
     * pairs of vertices
     * @return Perimeter of this polygon.
     */
    public double getPerimeter(){
        //iterate over all vertices and compute their distance
        Iterator<Point2D> iterator = mVertices.iterator();
        Point2D prevPoint = iterator.next();
        Point2D point;
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
    public boolean isInside(Point2D point) throws TriangulatorException{
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
    public boolean isInside(Point2D point, double threshold)
            throws IllegalArgumentException, TriangulatorException{
        if(threshold < MIN_THRESHOLD) throw new IllegalArgumentException();
        
        for(Triangle2D triangle : getTriangles()){
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
    public Point2D getCenter(){
        Point2D result = Point2D.create();
        center(result);
        return result;        
    }
    
    /**
     * Computes the center of this polygon.
     * The center is the average point among all the vertices of this polygon.
     * The center is not ensured to lie within the area formed by this polygon.
     * @param result Instance where the computed center will be stored.
     */
    public void center(Point2D result){
        //compute average location of all vertices
        double inhomX = 0.0, inhomY = 0.0;
        int total = mVertices.size();
        
        for(Point2D point : mVertices){
            inhomX += point.getInhomX() / (double)total;
            inhomY += point.getInhomY() / (double)total;
        }
        result.setInhomogeneousCoordinates(inhomX, inhomY);
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
    public boolean isLocus(Point2D point, double threshold) 
            throws IllegalArgumentException, TriangulatorException{
        if(threshold < MIN_THRESHOLD) throw new IllegalArgumentException();
        
        Iterator<Point2D> iterator = mVertices.iterator();
        Point2D prevPoint = iterator.next(); //it's ok because there are at 
                                            //least 3 vertices
        Point2D curPoint;
        while(iterator.hasNext()){
            curPoint = iterator.next();
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
    public boolean isLocus(Point2D point) throws TriangulatorException{
        return isLocus(point, DEFAULT_THRESHOLD);
    }  
    
    /**
     * Returns the shortest distance from provided point to a border of this
     * polygon. Note that borders are segments defined by consecutive vertices
     * @param point Point to be checked.
     * @return Shortest distance from provided point to this polygon.
     */
    public double getShortestDistance(Point2D point){        
        //iterate over all vertices and compute their distance
        Iterator<Point2D> iterator = mVertices.iterator();
        Point2D prevPoint = iterator.next();
        prevPoint.normalize(); //to increase accuracy
        Point2D curPoint;
        double bestDist = Double.MAX_VALUE, dist;
        boolean found = false;
        Line2D line = new Line2D();
        Point2D pointInLine = Point2D.create();
        
        while(iterator.hasNext()){
            curPoint = iterator.next();
            curPoint.normalize(); //to increase accuracy
            
            //check if point lies in the segment of the boundary of this polygon
            if(point.isBetween(curPoint, prevPoint)) return 0.0;
            
            line.setParametersFromPairOfPoints(curPoint, prevPoint);
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
        Point2D first = mVertices.get(0);
        if(point.isBetween(prevPoint, first)) return 0.0;
            
        line.setParametersFromPairOfPoints(prevPoint, first);
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
     */
    public Point2D getClosestPoint(Point2D point){
        Point2D result = Point2D.create();
        closestPoint(point, result);
        return result;        
    }
    
    /**
     * Computes the closes point to provided point that is locus of this
     * polygon (i.e. lies on a border of this polygon)
     * @param point Point to be checked
     * @param result Instance where the closest point will be stored
     */
    public void closestPoint(Point2D point, Point2D result){
        //iterate over all vertices and compute their distance
        Iterator<Point2D> iterator = mVertices.iterator();
        Point2D prevPoint = iterator.next();
        prevPoint.normalize(); //to increase accuracy
        Point2D curPoint;
        double bestDist = Double.MAX_VALUE, dist;
        boolean found = false;
        Line2D line = new Line2D();
        Point2D pointInLine = Point2D.create();
        
        while(iterator.hasNext()){
            curPoint = iterator.next();
            curPoint.normalize(); //to increase accuracy
            
            //check if point lies in the segment of the boundary of this polygon
            if(point.isBetween(curPoint, prevPoint)){
                result.setCoordinates(point);
                return;
            }
            
            line.setParametersFromPairOfPoints(curPoint, prevPoint);
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
        Point2D first = mVertices.get(0);
        if(point.isBetween(prevPoint, first)){
            result.setCoordinates(point);
            return;
        }
            
        line.setParametersFromPairOfPoints(prevPoint, first);
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
            Triangulator2D triangulator = Triangulator2D.create(
                    mTriangulatorMethod);
            mTriangles = triangulator.triangulate(mVertices);
            mTriangulated = true;
        }        
    }
}