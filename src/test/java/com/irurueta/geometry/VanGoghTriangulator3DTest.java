/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.VanGoghTriangulator3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 21, 2012
 */
package com.irurueta.geometry;

import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class VanGoghTriangulator3DTest {
    
    public static final double ABSOLUTE_ERROR = 1e-8;
    public static final double DEFAULT_ORIENTATION_THRESHOLD = Math.PI / 2.0;
    
    public static final double MIN_RANDOM_DEGREES = -180.0;
    public static final double MAX_RANDOM_DEGREES = 180.0;
    
    public static final int MIN_SIDES = 6;
    public static final int MAX_SIDES = 12;
    
    public static final double MIN_RADIUS = 1.0;
    public static final double MAX_RADIUS = 10.0;
    
    public static final double MIN_ANGLE_DEGREES = -180.0;
    public static final double MAX_ANGLE_DEGREES = 180.0;
    
    public static final double LARGE_ABSOLUTE_ERROR = 1e-6;
    
    public static final int INHOM_COORDS = 3;
    
    public static final int TIMES = 100;
    
    
    public VanGoghTriangulator3DTest() {
    }
    
    @BeforeClass
    public static void setUpClass() {
    }
    
    @AfterClass
    public static void tearDownClass() {
    }
    
    @Before
    public void setUp() {
    }
    
    @After
    public void tearDown() {
    }
    
    private List<Point3D> buildPolygonVertices(int sides, double radius,
            double theta){
        List<Point3D> vertices = new ArrayList<Point3D>(sides);
        Point3D vertex;        
        for(int i = 0; i < sides; i++){
            double angle = (double)i / (double)sides * 2.0 * Math.PI;
            vertex = new InhomogeneousPoint3D(
                radius * Math.cos(angle) * Math.cos(theta),
                radius * Math.sin(angle) * Math.cos(theta), 
                radius * Math.sin(theta));
            vertices.add(vertex);
        }
        return vertices;
    }
        

    @Test
    public void testConstructorAndGetMethod(){
        VanGoghTriangulator2D triangulator = new VanGoghTriangulator2D();
        assertNotNull(triangulator);
        
        //check method correctness
        assertEquals(triangulator.getMethod(), 
                TriangulatorMethod.VAN_GOGH_TRIANGULATOR);
    }
            
    @Test
    public void testIsPolygonOrientationReversed() 
            throws NotEnoughVerticesException, CoincidentPointsException{
        //X: -1, Y: 0, Z: 5 (W: 1)
        Point3D v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        //X: 1, Y: 0, Z: 5 (W: 1)
        Point3D v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        //X: 2, Y: 1, Z: 5 (W: 1)
        Point3D v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        //X: 0, Y: 1, Z: 5 (W: 1)
        Point3D v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);        
        
        
        //parallelipede
        List<Point3D> vertices1 = new ArrayList<Point3D>();
        List<Point3D> vertices2 = new ArrayList<Point3D>();
        vertices1.add(v1);
        vertices1.add(v2);
        vertices1.add(v3);
        vertices1.add(v4);
        
        vertices2.add(v4);
        vertices2.add(v3);
        vertices2.add(v2);
        vertices2.add(v1);
        
        Polygon3D polygon1 = new Polygon3D(vertices1);
        //because order of vertices is reversed, orientation will have opposite
        //direction
        Polygon3D polygon2 = new Polygon3D(vertices2);
                
        
        double angle = Polygon3D.getAngleBetweenPolygons(vertices1, vertices2);
        assertEquals(angle, Math.PI, ABSOLUTE_ERROR);        
        angle = Polygon3D.getAngleBetweenPolygons(vertices1, vertices2, 
                ABSOLUTE_ERROR);
        assertEquals(angle, Math.PI, ABSOLUTE_ERROR);        
        
        angle = Polygon3D.getAngleBetweenPolygons(polygon1, polygon2);
        assertEquals(angle, Math.PI, ABSOLUTE_ERROR);
        angle = Polygon3D.getAngleBetweenPolygons(polygon1, polygon2, 
                ABSOLUTE_ERROR);
        assertEquals(angle, Math.PI, ABSOLUTE_ERROR);
        
        //check that orientation is reversed
        assertTrue(VanGoghTriangulator3D.isPolygonOrientationReversed(vertices1, 
                vertices2));
        assertTrue(VanGoghTriangulator3D.isPolygonOrientationReversed(vertices1, 
                vertices2, DEFAULT_ORIENTATION_THRESHOLD));
        
        //trying with same polygon will return an angle of zero
        angle = Polygon3D.getAngleBetweenPolygons(vertices1, vertices1);
        assertEquals(angle, 0.0, ABSOLUTE_ERROR);
        angle = Polygon3D.getAngleBetweenPolygons(vertices1, vertices1, 
                ABSOLUTE_ERROR);
        assertEquals(angle, 0.0, ABSOLUTE_ERROR);
        
        angle = Polygon3D.getAngleBetweenPolygons(polygon1, polygon1);
        assertEquals(angle, 0.0, ABSOLUTE_ERROR);
        angle = Polygon3D.getAngleBetweenPolygons(polygon1, polygon1, 
                ABSOLUTE_ERROR);
        assertEquals(angle, 0.0, ABSOLUTE_ERROR);
        
        //now orientation is not reversed
        assertFalse(VanGoghTriangulator3D.isPolygonOrientationReversed(vertices1, 
                vertices1));
        assertFalse(VanGoghTriangulator3D.isPolygonOrientationReversed(vertices1, 
                vertices1, DEFAULT_ORIENTATION_THRESHOLD));                
    }
    
    @Test
    public void testIsOrientationReversed(){
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double angle = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        if(Math.abs(angle) > 
                VanGoghTriangulator3D.DEFAULT_ORIENTATION_THRESHOLD){
            assertTrue(VanGoghTriangulator3D.isOrientationReversed(angle));
        }else{
            assertFalse(VanGoghTriangulator3D.isOrientationReversed(angle));
        }
        
        if(Math.abs(angle) > DEFAULT_ORIENTATION_THRESHOLD){
            assertTrue(VanGoghTriangulator3D.isOrientationReversed(angle, 
                    DEFAULT_ORIENTATION_THRESHOLD));
        }else{
            assertFalse(VanGoghTriangulator3D.isOrientationReversed(angle, 
                    DEFAULT_ORIENTATION_THRESHOLD));            
        }
        
        assertFalse(VanGoghTriangulator3D.isOrientationReversed(
                VanGoghTriangulator3D.DEFAULT_ORIENTATION_THRESHOLD));
        assertTrue(VanGoghTriangulator3D.isOrientationReversed(
                2.0 * VanGoghTriangulator3D.DEFAULT_ORIENTATION_THRESHOLD));
                
        assertFalse(VanGoghTriangulator3D.isOrientationReversed(
                DEFAULT_ORIENTATION_THRESHOLD, DEFAULT_ORIENTATION_THRESHOLD));
        assertTrue(VanGoghTriangulator3D.isOrientationReversed(
                2.0 * DEFAULT_ORIENTATION_THRESHOLD, 
                DEFAULT_ORIENTATION_THRESHOLD));
    }
    
    @Test
    public void testIsEar() throws CoincidentPointsException{
        //X: -1, Y: 0, Z: 5 (W: 1)
        Point3D v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        //X: 1, Y: 0, Z: 5 (W: 1)
        Point3D v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        //X: 2, Y: 1, Z: 5 (W: 1)
        Point3D v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        //X: 0, Y: 1, Z: 5 (W: 1)
        Point3D v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);
        
        //Triangle
        List<Point3D> polygonVertices = new ArrayList<Point3D>();
        
        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v4);
        
        //Check that the triangle is an ear with itself, by using any three
        //consecutive vertices
        
                //1st triangle
        Triangle3D triangle = new Triangle3D(v1, v2, v4);

        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
        
        //2nd triangle
        triangle.setVertices(v2, v4, v1);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
        
        //3rd triangle
        triangle.setVertices(v4, v1, v2);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
        
        
        //Parallepipede
        polygonVertices.clear();
        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v3);
        polygonVertices.add(v4);
        
        //all consecutive vertices of parallepipede are also ears
        
        //1st triangle
        triangle.setVertices(v1, v2, v3);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
        
        //2nd triangle
        triangle.setVertices(v2, v3, v4);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
        
        //3rd triangle
        triangle.setVertices(v3, v4, v1);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
        
        //4th triangle
        triangle.setVertices(v4, v1, v2);
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
        
        
        
        //General polygon
        //X: -1, Y: 0, Z: 5 (W: 1)
        v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        //X: 0, Y: 0.5, Z: 5 (W: 1)
        v2 = new InhomogeneousPoint3D(0.0, 0.5, 5.0);
        //X: 1, Y: 0, Z: 5 (W: 1)
        v3 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        //X: 2, Y: 1, Z: 5 (W: 1)
        v4 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        //X: 0, Y: 1, Z: 5 (W: 1)
        Point3D v5 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);
        
        polygonVertices.clear();
        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v3);
        polygonVertices.add(v4);
        polygonVertices.add(v5);
        
        
        //1st triangle
        triangle.setVertices(v1, v2, v3);
        
        //triangle is not an ear (orientation reversed)
        assertFalse(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
        
        //2nd triangle
        triangle.setVertices(v2, v3, v4);
        
        //triangle is an ear
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
        
        
        //3rd triangle
        triangle.setVertices(v3, v4, v5);
        
        //triangle is an ear
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
        
        
        //4th triangle
        triangle.setVertices(v4, v5, v1);
        
        //triangle is not an ear (point of polygon inside triangle)
        assertFalse(VanGoghTriangulator3D.isEar(triangle, polygonVertices));
        
        
        //5th triangle
        triangle.setVertices(v5, v1, v2);
        
        //triangle is an ear
        assertTrue(VanGoghTriangulator3D.isEar(triangle, polygonVertices));        
    }
    
    @Test
    public void testTriangulateKnownPolygon() throws TriangulatorException{
        //X: -1, Y: 0, Z: 5 (W: 1)
        Point3D v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        //X: 1, Y: 0, Z: 5 (W: 1)
        Point3D v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        //X: 2, Y: 1, Z: 5 (W: 1)
        Point3D v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        //X: 0, Y: 1, Z: 5 (W: 1)
        Point3D v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);        
        
        //Test for Triangle polygon
        List<Point3D> polygonVertices = new ArrayList<Point3D>();
        
        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v4);
        
        VanGoghTriangulator3D triangulator = new VanGoghTriangulator3D();
        List<Triangle3D> triangles  = triangulator.triangulate(polygonVertices);
        
        //triangles contains a single triangle with vertices v1, v2 and v4
        assertEquals(triangles.size(), 1);
        Triangle3D triangle = triangles.get(0);
        assertTrue(triangle.getVertex1().equals(v1, ABSOLUTE_ERROR));
        assertTrue(triangle.getVertex2().equals(v2, ABSOLUTE_ERROR));
        assertTrue(triangle.getVertex3().equals(v4, ABSOLUTE_ERROR));
        
        
        
        //Test for Parallepipede polygon
        polygonVertices.clear();
        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v3);
        polygonVertices.add(v4);
        
        triangles  = triangulator.triangulate(polygonVertices);
        
        //parallepipede is divided into 2 triangles
        assertEquals(triangles.size(), 2);
        //1st triangle is formed by v4, v1 and v2
        Triangle3D triangle1 = triangles.get(0);
        assertTrue(triangle1.getVertex1().equals(v4, ABSOLUTE_ERROR));
        assertTrue(triangle1.getVertex2().equals(v1, ABSOLUTE_ERROR));
        assertTrue(triangle1.getVertex3().equals(v2, ABSOLUTE_ERROR));
        
        //2nd triangle is formed by v2, v3 and v4
        Triangle3D triangle2 = triangles.get(1);
        assertTrue(triangle2.getVertex1().equals(v2, ABSOLUTE_ERROR));
        assertTrue(triangle2.getVertex2().equals(v3, ABSOLUTE_ERROR));
        assertTrue(triangle2.getVertex3().equals(v4, ABSOLUTE_ERROR));
        
        
  
        //General polygon
        //X: -1, Y: 0, Z: 5 (W: 1)
        v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        //X: 0, Y: 0.5, Z: 5 (W: 1)
        v2 = new InhomogeneousPoint3D(0.0, 0.5, 5.0);
        //X: 1, Y: 0, Z: 5 (W: 1)
        v3 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        //X: 2, Y: 1, Z: 5 (W: 1)
        v4 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        //X: 0, Y: 1, Z: 5 (W: 1)
        Point3D v5 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);
        
        polygonVertices.clear();
        polygonVertices.add(v1);
        polygonVertices.add(v2);
        polygonVertices.add(v3);
        polygonVertices.add(v4);
        polygonVertices.add(v5);    
        
        triangles  = triangulator.triangulate(polygonVertices);
        
        //divided in 3 triangles
        assertEquals(triangles.size(), 3);
        //1st triangle is formed by v4, v1 and v2
        triangle1 = triangles.get(0);
        assertTrue(triangle1.getVertex1().equals(v5, ABSOLUTE_ERROR));
        assertTrue(triangle1.getVertex2().equals(v1, ABSOLUTE_ERROR));
        assertTrue(triangle1.getVertex3().equals(v2, ABSOLUTE_ERROR));
        //2nd triangle is formed by v5, v2 and v3
        triangle2 = triangles.get(1);
        assertTrue(triangle2.getVertex1().equals(v5, ABSOLUTE_ERROR));
        assertTrue(triangle2.getVertex2().equals(v2, ABSOLUTE_ERROR));
        assertTrue(triangle2.getVertex3().equals(v3, ABSOLUTE_ERROR));
        //3rd triangle is formed by v3, v4 and v5
        Triangle3D triangle3 = triangles.get(2);
        assertTrue(triangle3.getVertex1().equals(v3, ABSOLUTE_ERROR));
        assertTrue(triangle3.getVertex2().equals(v4, ABSOLUTE_ERROR));
        assertTrue(triangle3.getVertex3().equals(v5, ABSOLUTE_ERROR));
    }
    
    @Test
    public void testTriangulate() throws NotEnoughVerticesException, 
        TriangulatorException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        double phi = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES);
        
        //build vertices list
        List<Point3D> vertices = buildPolygonVertices(sides, radius, phi);
        
        //build polygon
        Polygon3D polygon = new Polygon3D(vertices);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        //create point inside of vertex
        Point3D point1 = new InhomogeneousPoint3D(
                radius / 2.0 * Math.cos(theta) * Math.cos(phi),
                radius / 2.0 * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));
        
        //create point outside of vertex
        Point3D point2 = new InhomogeneousPoint3D(
                2.0 * radius * Math.cos(theta) * Math.cos(phi),
                2.0 * radius * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));        
        
        VanGoghTriangulator3D triangulator = new VanGoghTriangulator3D();
        List<Triangle3D> triangles1 = triangulator.triangulate(polygon);
        List<Triangle3D> triangles2 = triangulator.triangulate(vertices);
        List<int[]> indices = new ArrayList<int[]>();
        List<Triangle3D> triangles3 = triangulator.triangulate(vertices, 
                indices);
        
        
        double area1 = 0.0;
        boolean inside1 = false, inside2 = false;
        for(Triangle3D triangle : triangles1){
            area1 += triangle.getArea();
            if(triangle.isInside(point1)) inside1 = true;
            if(triangle.isInside(point2)) inside2 = false;
        }
        
        assertEquals(area1, polygon.getArea(), ABSOLUTE_ERROR);
        
        //check that point1 lies inside, but point2 does not
        assertTrue(polygon.isInside(point1));
        assertTrue(inside1);
        
        assertFalse(polygon.isInside(point2));
        assertFalse(inside2);
        
        double area2 = 0.0;
        inside1 = inside2 = false;
        for(Triangle3D triangle : triangles2){
            area2 += triangle.getArea();
            if(triangle.isInside(point1)) inside1 = true;
            if(triangle.isInside(point2)) inside2 = false;            
        }
        
        assertEquals(area2, polygon.getArea(), ABSOLUTE_ERROR);
        
        //check that point1 lies inside, but point2 does not
        assertTrue(polygon.isInside(point1));
        assertTrue(inside1);
        
        assertFalse(polygon.isInside(point2));
        assertFalse(inside2);    
        
        double area3 = 0.0;
        inside1 = inside2 = false;
        for(Triangle3D triangle : triangles3){
            area3 += triangle.getArea();
            if(triangle.isInside(point1)) inside1 = true;
            if(triangle.isInside(point2)) inside2 = false;            
        }
        
        assertEquals(area3, polygon.getArea(), ABSOLUTE_ERROR);
        
        //check that point1 lies inside, but point2 does not
        assertTrue(polygon.isInside(point1));
        assertTrue(inside1);
        
        assertFalse(polygon.isInside(point2));
        assertFalse(inside2);                
        
        assertTrue(indices.size() > 0);        
    }    
}