/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.Triangulator2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 20, 2012
 */
package com.irurueta.geometry;

import java.util.List;

/**
 * This class defines a triangulator for 3D polygons. Triangulators divide 
 * polygons into triangles, which are the simplest geometric figure.
 * Criteria for triangulation depends on each triangulator implementation.
 */
public abstract class Triangulator3D {
    
    /**
     * Constant defining minimum vertices allowed in a polygon
     */
    public static final int MIN_VERTICES = 3;
    
    /**
     * Constant defining default triangulator method
     */
    public static final TriangulatorMethod DEFAULT_TRIANGULATOR_METHOD = 
            TriangulatorMethod.VAN_GOGH_TRIANGULATOR;
    
    /**
     * Empty constructor
     */
    public Triangulator3D(){}
    
    /**
     * Returns triangulator method.
     * Each method implementation will divide polygons into triangles using
     * different techniques
     * @return Triangulator method.
     */
    public abstract TriangulatorMethod getMethod();
    
    /**
     * Triangulates provided polygon by divinding it into a set of triangles
     * @param polygon Polygon to be triangulated
     * @return List of triangles forming the polygon that has been triangulated
     * @throws TriangulatorException Raised if triangulation cannot be done.
     * Usually this indicates numerical instability or polygon degeneracy.
     */
    public abstract List<Triangle3D> triangulate(Polygon3D polygon) 
            throws TriangulatorException;
    
    /**
     * Triangulates a polygon formed by provided vertices
     * @param vertices List of points considered as vertices of a polygon
     * @return List of triangles forming the polygon that has been triangulated
     * @throws TriangulatorException Raised if triangulation cannot be done.
     * Usually this indicates numerical instability or polygon degeneracy.
     */
    public abstract List<Triangle3D> triangulate(List<Point3D> vertices) 
            throws TriangulatorException;    
    
    /**
     * Triangulates a polygon formed by provided vertices
     * @param vertices List of points considered as vertices of a polygon
     * @param indices List where indices of original vertices will be stored.
     * This list can be used to refer to the original order of vertices. Notice
     * that vertices indices might be repeated because vertices might appear in
     * more than one triangle after triangulation. If this parameter is null, 
     * indices won't be stored in this list.
     * @return List of triangles forming the polygon that has been triangulated
     * @throws TriangulatorException Raised if triangulation cannot be done.
     * Usually this indicates numerical instability or polygon degeneracy.
     */    
    public abstract List<Triangle3D> triangulate(List<Point3D> vertices,
            List<int[]> indices) throws TriangulatorException;
    
    /**
     * Instantiates a triangulator for 3D polygons using default method
     * @return A triangulator for 3D polygons
     */
    public static Triangulator3D create(){
        return create(DEFAULT_TRIANGULATOR_METHOD);
    }
    
    /**
     * Instantiates a triangulator for 3D polygons using provided method
     * @param method A triangulator method
     * @return A triangulator for 3D polygons
     */
    public static Triangulator3D create(TriangulatorMethod method){
        switch(method){
            case VAN_GOGH_TRIANGULATOR:
            default:
                return new VanGoghTriangulator3D();
        }        
    }
}
