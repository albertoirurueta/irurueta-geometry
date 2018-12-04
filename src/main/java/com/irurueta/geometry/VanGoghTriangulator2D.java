/*
 * Copyright (C) 2012 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.geometry;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * This class defines a triangulator for 2D polygons. Triangulators divide 
 * polygons into triangles, which are the simplest geometric figure.
 * This implementation uses Van Gogh or Ear Cutting algorith for triangulation.
 */
public class VanGoghTriangulator2D extends Triangulator2D {
   
    /**
     * Returns triangulator method.
     * Each method implementation will divide polygons into triangles using
     * different techniques.
     * @return Triangulator method.
     */    
    @Override
    public TriangulatorMethod getMethod() {
        return TriangulatorMethod.VAN_GOGH_TRIANGULATOR;
    }
    
    /**
     * Triangulates provided polygon by dividing it into a set of triangles.
     * @param polygon Polygon to be triangulated.
     * @return List of triangles forming the polygon that has been triangulated.
     * @throws TriangulatorException Raised if triangulation cannot be done.
     * Usually this indicates numerical instability or polygon degeneracy.
     */    
    @Override
    public List<Triangle2D> triangulate(Polygon2D polygon) 
            throws TriangulatorException {
        //triangulation will modify provided list of vertices so we make a copy 
        //of it
        List<Point2D> vertices = polygon.getVertices(); //original vertices
        List<Point2D> verticesCopy = new ArrayList<>(
                polygon.getVertices());
        return internalTriangulate(verticesCopy, null, vertices);
    }
            
    /**
     * Triangulates a polygon formed by provided vertices.
     * @param vertices List of points considered as vertices of a polygon.
     * @return List of triangles forming the polygon that has been triangulated.
     * @throws TriangulatorException Raised if triangulation cannot be done.
     * Usually this indicates numerical instability or polygon degeneracy.
     */    
    @Override
    public List<Triangle2D> triangulate(List<Point2D> vertices) 
            throws TriangulatorException {
        if (vertices.size() < MIN_VERTICES) {
            throw new TriangulatorException();
        }

        //triangulation will modify provided list of vertices so we make a copy 
        //of it        
        List<Point2D> verticesCopy = new ArrayList<>(vertices);
        return internalTriangulate(verticesCopy, null, vertices);
    }
    
    /**
     * Triangulates a polygon formed by provided vertices.
     * @param vertices List of points considered as vertices of a polygon.
     * @param indices List where indices of original vertices will be stored.
     * This list can be used to refer to the original order of vertices. Notice
     * that vertices indices might be repeated because vertices might appear in
     * more than one triangle after triangulation. If this parameter is null,
     * indices won't be stored in this list.
     * @return List of triangles forming the polygon that has been triangulated
     * @throws TriangulatorException Raised if triangulation cannot be done.
     * Usuallyt his indicates numerical instability or polygon degeneracy.
     */
    @Override
    public List<Triangle2D> triangulate(List<Point2D> vertices,
            List<int[]> indices) throws TriangulatorException {
        if (vertices.size() < MIN_VERTICES) {
            throw new TriangulatorException();
        }
        
        //triangulation will modify provided list of vertices so we make a copy
        //of it
        List<Point2D> verticesCopy = new ArrayList<>(vertices);
        return internalTriangulate(verticesCopy, indices, vertices);
    }
    
    /**
     * Internal method that computes the actual triangulation.
     * @param verticesCopy List of points considered as verticesCopy of a 
     * polygon. This list will be modified after execution of this method.
     * @param indices List where indices of original verticesCopy will be 
     * stored.
     * This list can be used to refer to the original order of verticesCopy. 
     * Notice that verticesCopy indices might be repeated because verticesCopy 
     * might appear in more than one triangle after triangulation. If this 
     * parameter is null, indices won't be stored in this list.
     * @param originalVertices Reference to original list of vertices that won't
     * be modified.
     * @return List of triangles forming the polygon that has been triangulated.
     * @throws TriangulatorException Raised if triangulation cannot be done.
     * Usually this indicates numerical instability or polygon degeneracy.
     */
    private static List<Triangle2D> internalTriangulate(
            List<Point2D> verticesCopy, List<int[]> indices,
            List<Point2D> originalVertices) throws TriangulatorException {
        if (verticesCopy.size() < MIN_VERTICES) {
            throw new TriangulatorException();
        }
        
        List<Triangle2D> result = new LinkedList<>();

        boolean madeCut;
        
        Triangle2D triangle = null; 
        
        // Second, apply algorithm
        while (verticesCopy.size() > MIN_VERTICES) {
            madeCut = false;
            int lastElement = verticesCopy.size() - 1;
            for (int i = 0; i <= lastElement; i++) {

                if (i == 0) {
                    if (triangle == null) {
                        //instantiate triangle if not already instantiated
                        triangle = new Triangle2D(verticesCopy.get(lastElement),
                                verticesCopy.get(0), verticesCopy.get(1));
                    } else {
                        triangle.setVertices(verticesCopy.get(lastElement),
                                verticesCopy.get(0), verticesCopy.get(1));
                    }
                } else if (i == lastElement) {
                    triangle.setVertices(verticesCopy.get(lastElement - 1),
                            verticesCopy.get(lastElement),
                            verticesCopy.get(0));
                } else {
                    triangle.setVertices(verticesCopy.get(i - 1),
                            verticesCopy.get(i), verticesCopy.get(i + 1));
                }
                
                if (isEar(triangle, verticesCopy)) {
                    // If it is an ear, we build a face out of the triangle being
                    //cut and remove it from polygon by cutting it
                    result.add(triangle);
                    triangle = null; //so that it cannot be reused after being added

                    //cut ear
                    verticesCopy.remove(i);
                    madeCut = true;

                    //Leave from FOR loop to loop again to new reduced vertices set
                    break;
                }
            }
            
            //if arrived here but no cut was made and polygon size contains
            //more than 3 vertices, then the algorithm failed for some reason
            if (!madeCut) {
                throw new TriangulatorException();
            }
        }

        //instantiate final triangle
        triangle = new Triangle2D(verticesCopy.get(0),
                verticesCopy.get(1), verticesCopy.get(2));


        boolean arePointsColinear = triangle.areVerticesColinear();
        
        //only add final triangle if not colinear (area greater than small 
        //threshold)
        if(!arePointsColinear)
            result.add(triangle);

        //add indices of triangles verticesCopy
        computeIndices(originalVertices, result, indices);

        return result;
    }   
    
    /**
     * Computes indices of resulting triangles vertices respect to original 
     * polygon vertices. Indices are stored in provided indices list.
     * @param vertices Vertices of polygon.
     * @param triangles Triangles obtained after triangulation.
     * @param indices Indices of original positions of resulting triangle's
     * vertices.
     */
    private static void computeIndices(List<Point2D> vertices,
            List<Triangle2D> triangles, List<int[]> indices) {
        if (indices != null) {
            int vertexCounter;
            int triangleVertexCounter;
            int[] triangleIndices;
            for (Triangle2D t : triangles) {
                triangleVertexCounter = 0;
                triangleIndices = new int[Triangle2D.NUM_VERTICES];
                for (Point2D p1 : t.getVertices()) {
                    vertexCounter = 0;
                    for (Point2D p2 : vertices) {
                        if (p1 == p2) {
                            triangleIndices[triangleVertexCounter] = 
                                    vertexCounter;
                            break;
                        }
                        vertexCounter++;
                    }
                    triangleVertexCounter++;
                }
                indices.add(triangleIndices);
            }
        }
    }
    
    /**
     * Determines if provided triangle can be considered as an ear of the 
     * remaining polygon formed by provided vertices.
     * An ear is usually a triangle located at a corner of a polygon.
     * A triangle is considered an ear if no other vertex of the polygon lies
     * within the triangle and if the triangle is not convex (is concave).
     * @param triangle A triangle.
     * @param polygonVertices A list of points forming the remaining polygon
     * @return true if triangle is ear, false otherwise.
     */
    private static boolean isEar(Triangle2D triangle, 
            List<Point2D> polygonVertices) {
        
        boolean isInside;
        boolean isNotConvex;
        //in a counterclockwise polygon, reversed orientation means that 
        //triangle is not convex and cannot be an ear
        
        List<Point2D> triangleVertices = triangle.getVertices();
        
        //check that no points in the polygon (aside from points belonging to
        //the triangle) lie inside the triangle
        for (Point2D testPoint : polygonVertices) {
            //Do not compare with polygon elements which are triangle points
            //if end is reached then polygon and triangle is equal and hence the 
            //polygon is the ear            
            if (triangleVertices.contains(testPoint)) {
                continue;
            }
            
            isInside = triangle.isInside(testPoint);
            isNotConvex = triangle.areVerticesClockwise();
            
            //if a point is inside the triangle or orientation is reversed, then
            //it is not an ear
            if (isInside || isNotConvex) {
                return false;
            }
        }
        
        //no points in the polygon where found inside the triangle and 
        //orientation is the same, so an ear is detected
        return true;        
    }                
}
