/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.VanGoghTriangulator2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 19, 2012
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

public class VanGoghTriangulator2DTest {
    
    public static final int MIN_SIDES = 3;
    public static final int MAX_SIDES = 12;
    
    public static final double MIN_RADIUS = 1.0;
    public static final double MAX_RADIUS = 10.0;
    
    public static final double MIN_ANGLE_DEGREES = -180.0;
    public static final double MAX_ANGLE_DEGREES = 180.0;    
    
    public static final double ABSOLUTE_ERROR = 1e-8;    
    
    public VanGoghTriangulator2DTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}
    
    private List<Point2D> buildPolygonVertices(int sides, double radius){
        List<Point2D> vertices = new ArrayList<Point2D>(sides);
        Point2D vertex;        
        for(int i = 0; i < sides; i++){
            double angle = (double)i / (double)sides * 2.0 * Math.PI;
            vertex = new InhomogeneousPoint2D(radius * Math.cos(angle),
                    radius * Math.sin(angle));
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
    public void testTriangulate() throws NotEnoughVerticesException, 
        TriangulatorException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        
        //build vertices list
        List<Point2D> vertices = buildPolygonVertices(sides, radius);
        
        //build polygon
        Polygon2D polygon = new Polygon2D(vertices);
        
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        //create point inside of vertex
        Point2D point1 = new InhomogeneousPoint2D(
                radius / 2.0 * Math.cos(theta),
                radius / 2.0 * Math.sin(theta));
        
        //create point outside of vertex
        Point2D point2 = new InhomogeneousPoint2D(
                2.0 * radius * Math.cos(theta),
                2.0 * radius * Math.sin(theta));        
        
        VanGoghTriangulator2D triangulator = new VanGoghTriangulator2D();
        List<Triangle2D> triangles1 = triangulator.triangulate(polygon);
        List<Triangle2D> triangles2 = triangulator.triangulate(vertices);
        List<int[]> indices = new ArrayList<int[]>();
        List<Triangle2D> triangles3 = triangulator.triangulate(vertices, 
                indices);
        
        double signedArea1 = 0.0, area1 = 0.0;
        boolean inside1 = false, inside2 = false;
        for(Triangle2D triangle : triangles1){
            signedArea1 += triangle.getSignedArea();
            area1 += triangle.getArea();
            if(triangle.isInside(point1)) inside1 = true;
            if(triangle.isInside(point2)) inside2 = false;
        }
        
        assertEquals(signedArea1, polygon.getSignedArea(), ABSOLUTE_ERROR);
        assertEquals(area1, polygon.getArea(), ABSOLUTE_ERROR);
        
        //check that point1 lies inside, but point2 does not
        assertTrue(polygon.isInside(point1));
        assertTrue(inside1);
        
        assertFalse(polygon.isInside(point2));
        assertFalse(inside2);
        
        double signedArea2 = 0.0, area2 = 0.0;
        inside1 = inside2 = false;
        for(Triangle2D triangle : triangles2){
            signedArea2 += triangle.getSignedArea();
            area2 += triangle.getArea();
            if(triangle.isInside(point1)) inside1 = true;
            if(triangle.isInside(point2)) inside2 = false;            
        }
        
        assertEquals(signedArea2, polygon.getSignedArea(), ABSOLUTE_ERROR);
        assertEquals(area2, polygon.getArea(), ABSOLUTE_ERROR);
        
        //check that point1 lies inside, but point2 does not
        assertTrue(polygon.isInside(point1));
        assertTrue(inside1);
        
        assertFalse(polygon.isInside(point2));
        assertFalse(inside2);        
        
        double area3 = 0.0;
        inside1 = inside2 = false;
        for(Triangle2D triangle : triangles3){
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
