/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.Triangle2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 12, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
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

public class Triangle2DTest {
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double ABSOLUTE_ERROR = 1e-8;
    
    public static final int TIMES = 100;
    
    public Triangle2DTest() {
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
    
    @Test
    public void testConstructor(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        Point2D point1 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point2 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point3 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        Triangle2D triangle = new Triangle2D(point1, point2, point3);
        
        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);
        
        //Force NullPointerException
        triangle = null;
        try{
            triangle = new Triangle2D(null, point2, point3);
            fail("NullPointerException expected but not thrown");
        }catch(NullPointerException e){}        
        try{
            triangle = new Triangle2D(point1, null, point3);
            fail("NullPointerException expected but not thrown");
        }catch(NullPointerException e){}
        try{
            triangle = new Triangle2D(point1, point2, null);
            fail("NullPointerException expected but not thrown");
        }catch(NullPointerException e){}
        assertNull(triangle);
    }
    
    @Test
    public void testGetSetVertex1(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        Point2D point1 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point2 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point3 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        Triangle2D triangle = new Triangle2D(point1, point2, point3);
        
        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        //new vertex1
        Point2D vertex1 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        triangle.setVertex1(vertex1);
        //check correctness
        assertEquals(triangle.getVertex1(), vertex1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        //Force NullPointerException
        try{
            triangle.setVertex1(null);
            fail("NullPointerException expected but not thrown");
        }catch(NullPointerException e){}
    }
    
    @Test
    public void testGetSetVertex2(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        Point2D point1 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point2 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point3 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        Triangle2D triangle = new Triangle2D(point1, point2, point3);
        
        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        //new vertex1
        Point2D vertex2 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        triangle.setVertex2(vertex2);
        //check correctness
        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), vertex2);
        assertEquals(triangle.getVertex3(), point3);

        //Force NullPointerException
        try{
            triangle.setVertex2(null);
            fail("NullPointerException expected but not thrown");
        }catch(NullPointerException e){}
    }

    @Test
    public void testGetSetVertex3(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        Point2D point1 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point2 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point3 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        Triangle2D triangle = new Triangle2D(point1, point2, point3);
        
        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        //new vertex1
        Point2D vertex3 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        triangle.setVertex3(vertex3);
        //check correctness
        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), vertex3);

        //Force NullPointerException
        try{
            triangle.setVertex3(null);
            fail("NullPointerException expected but not thrown");
        }catch(NullPointerException e){}
    }
    
    @Test
    public void testGetSetVertices(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        Point2D point1 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point2 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point3 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        Triangle2D triangle = new Triangle2D(point1, point2, point3);
        
        assertEquals(triangle.getVertex1(), point1);
        assertEquals(triangle.getVertex2(), point2);
        assertEquals(triangle.getVertex3(), point3);

        Point2D point1b = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point2b = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point3b = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        //Set new vertices
        triangle.setVertices(point1b, point2b, point3b);
        
        //check correctness
        assertEquals(triangle.getVertex1(), point1b);
        assertEquals(triangle.getVertex2(), point2b);
        assertEquals(triangle.getVertex3(), point3b);      
        
        //get vertices as a list
        List<Point2D> vertices = triangle.getVertices();
        List<Point2D> vertices2 = new ArrayList<Point2D>();
        triangle.vertices(vertices2);
        
        assertEquals(vertices.size(), Triangle2D.NUM_VERTICES);
        assertEquals(vertices2.size(), Triangle2D.NUM_VERTICES);
        
        assertTrue(vertices.get(0).equals(point1b, ABSOLUTE_ERROR));
        assertTrue(vertices.get(1).equals(point2b, ABSOLUTE_ERROR));
        assertTrue(vertices.get(2).equals(point3b, ABSOLUTE_ERROR));

        assertTrue(vertices2.get(0).equals(point1b, ABSOLUTE_ERROR));
        assertTrue(vertices2.get(1).equals(point2b, ABSOLUTE_ERROR));
        assertTrue(vertices2.get(2).equals(point3b, ABSOLUTE_ERROR));
        
        //Force NullPointerException
        try{
            triangle.setVertices(null, point2, point3);
            fail("NullPointerException expected but not thrown");
        }catch(NullPointerException e){}
        try{
            triangle.setVertices(point1, null, point3);
            fail("NullPointerException expected but not thrown");
        }catch(NullPointerException e){}
        try{
            triangle.setVertices(point1, point2, null);
            fail("NullPointerException expected but not thrown");
        }catch(NullPointerException e){}
    }
    
    @Test
    public void testAreaSignedAreaAndAreColinearPoints() throws WrongSizeException, 
        DecomposerException{
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());                
        double base = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        double height = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        
        //Test known and simple values
        Point2D point1 = new InhomogeneousPoint2D(0.0, 0.0);
        Point2D point2 = new InhomogeneousPoint2D(base, 0.0);
        Point2D point3 = new InhomogeneousPoint2D(base / 2.0, height);
        
        double expectedArea = base * height / 2.0;
        
        Triangle2D triangle = new Triangle2D(point1, point2, point3);
        
        assertEquals(Math.abs(Triangle2D.signedArea(triangle)), expectedArea, 
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(Triangle2D.signedArea(point1, point2, point3)), 
                expectedArea, ABSOLUTE_ERROR);
        assertEquals(Math.abs(triangle.getSignedArea()), expectedArea,
                ABSOLUTE_ERROR);
        
        assertEquals(Triangle2D.area(triangle), expectedArea, ABSOLUTE_ERROR);
        assertEquals(Triangle2D.area(point1, point2, point3), expectedArea,
                ABSOLUTE_ERROR);
        assertEquals(triangle.getArea(), expectedArea, ABSOLUTE_ERROR);
        
        if(expectedArea > Triangle2D.DEFAULT_THRESHOLD){
            assertFalse(triangle.areVerticesColinear());
        }else{
            assertTrue(triangle.areVerticesColinear());
        }
        
        //if threshold is large enough, points will always be considered to be
        //colinear
        assertTrue(triangle.areVerticesColinear(expectedArea));
        
        
        
        //Test with random values
        point1 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point3 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        Matrix m = new Matrix(2, 2);
        //1st column
        m.setElementAt(0, 0, point2.getInhomX() - point1.getInhomX());
        m.setElementAt(1, 0, point2.getInhomY() - point1.getInhomY());
        //2nd columns
        m.setElementAt(0, 1, point3.getInhomX() - point1.getInhomX());
        m.setElementAt(1, 1, point3.getInhomY() - point1.getInhomY());
        
        //expected area
        double expectedSignedArea = 0.5 * Utils.det(m);
        expectedArea = Math.abs(expectedSignedArea);
        
        triangle = new Triangle2D(point1, point2, point3);
        
        assertEquals(Triangle2D.signedArea(triangle), expectedSignedArea, 
                ABSOLUTE_ERROR);
        assertEquals(Triangle2D.signedArea(point1, point2, point3), 
                expectedSignedArea, ABSOLUTE_ERROR);
        assertEquals(triangle.getSignedArea(), expectedSignedArea,
                ABSOLUTE_ERROR);
        
        assertEquals(Triangle2D.area(triangle), expectedArea, ABSOLUTE_ERROR);
        assertEquals(Triangle2D.area(point1, point2, point3), expectedArea,
                ABSOLUTE_ERROR);
        assertEquals(triangle.getArea(), expectedArea, ABSOLUTE_ERROR);   
        
        if(expectedArea > Triangle2D.DEFAULT_THRESHOLD){
            assertFalse(triangle.areVerticesColinear());
        }else{
            assertTrue(triangle.areVerticesColinear());
        }
        
        //if threshold is large enough, points will always be considered to be
        //colinear
        assertTrue(triangle.areVerticesColinear(expectedArea + ABSOLUTE_ERROR));        
        
        
        //If two points are coincident, then area must be zero or close to zero
        triangle = new Triangle2D(point1, point1, point2);
        assertEquals(Triangle2D.signedArea(triangle), 0.0, ABSOLUTE_ERROR);
        assertEquals(Triangle2D.signedArea(point1, point1, point2), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(triangle.getSignedArea(), 0.0, ABSOLUTE_ERROR);
        
        assertEquals(Triangle2D.area(triangle), 0.0, ABSOLUTE_ERROR);
        assertEquals(Triangle2D.area(point1, point1, point2), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(triangle.getArea(), 0.0, ABSOLUTE_ERROR);
        
        //because area is zero, then points are colinear
        assertTrue(triangle.areVerticesColinear());
        assertTrue(triangle.areVerticesColinear(ABSOLUTE_ERROR));
        
        //Force IllegalArgumentException
        try{
            triangle.areVerticesColinear(-ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testIsInsideShortestDistanceAndIsLocus(){
        
        for(int t = 0; t < TIMES; t++){
            //Test for known values
            UniformRandomizer randomizer = new UniformRandomizer(new Random());                
            double base, height, area;
            do{
                //we iterate until we ensure that triangle is not too small
                base = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                height = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                area = 0.5 * Math.abs(base * height);
            }while(area < 1.0);
            double dist = randomizer.nextDouble(Triangle2D.DEFAULT_THRESHOLD, 
                    MAX_RANDOM_VALUE);
        
            //Test known and simple values
            Point2D point1 = new InhomogeneousPoint2D(0.0, 0.0);
            Point2D point2 = new InhomogeneousPoint2D(base, 0.0);
            Point2D point3 = new InhomogeneousPoint2D(base / 2.0, height);        
        
            Triangle2D triangle = new Triangle2D(point1, point2, point3);
            Point2D center = triangle.getCenter();
        
            //vertices and center lie inside the triangle
            assertTrue(triangle.isInside(point1));
            assertTrue(triangle.isInside(point2));
            assertTrue(triangle.isInside(point3));
            assertTrue(triangle.isInside(center));
        
            //test shortest distance
            assertEquals(triangle.getShortestDistance(point1), 0.0, 0.0);
            assertEquals(triangle.getShortestDistance(point2), 0.0, 0.0);
            assertEquals(triangle.getShortestDistance(point3), 0.0, 0.0);
            Line2D line1 = new Line2D(point1, point2);
            Line2D line2 = new Line2D(point1, point3);
            Line2D line3 = new Line2D(point2, point3);
            double dist1 = Math.abs(line1.signedDistance(center));
            double dist2 = Math.abs(line2.signedDistance(center));
            double dist3 = Math.abs(line3.signedDistance(center));
            double centerDist = Math.min(dist1, Math.min(dist2, dist3));
            assertEquals(triangle.getShortestDistance(center), centerDist, 
                    ABSOLUTE_ERROR);
            
            //test is locus (vertices will be locus, but not center)
            assertTrue(triangle.isLocus(point1));
            assertTrue(triangle.isLocus(point2));
            assertTrue(triangle.isLocus(point3));
            assertFalse(triangle.isLocus(center));
        
            assertTrue(Triangle2D.isInside(triangle, point1));
            assertTrue(Triangle2D.isInside(triangle, point2));
            assertTrue(Triangle2D.isInside(triangle, point3));
            assertTrue(Triangle2D.isInside(triangle, center));
        
            assertEquals(Triangle2D.shortestDistance(triangle, point1), 0.0, 
                    0.0);
            assertEquals(Triangle2D.shortestDistance(triangle, point2), 0.0, 
                    0.0);
            assertEquals(Triangle2D.shortestDistance(triangle, point3), 0.0, 
                    0.0);
            assertEquals(Triangle2D.shortestDistance(triangle, center), 
                    centerDist, ABSOLUTE_ERROR);
        
            assertTrue(Triangle2D.isInside(point1, point2, point3, point1));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point2));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point3));
            assertTrue(Triangle2D.isInside(point1, point2, point3, center));
        
            assertEquals(Triangle2D.shortestDistance(point1, point2, point3, 
                    point1), 0.0, 0.0);
            assertEquals(Triangle2D.shortestDistance(point1, point2, point3, 
                    point2), 0.0, 0.0);
            assertEquals(Triangle2D.shortestDistance(point1, point2, point3, 
                    point3), 0.0, 0.0);
            assertEquals(Triangle2D.shortestDistance(point1, point2, point3, 
                    center), centerDist, ABSOLUTE_ERROR);
        
        
            //the same is true for a small threshold
            assertTrue(triangle.isInside(point1, 0.0));
            assertTrue(triangle.isInside(point2, 0.0));
            assertTrue(triangle.isInside(point3, 0.0));
            
            //check is locus with a small threshold
            assertTrue(triangle.isLocus(point1, 0.0));
            assertTrue(triangle.isLocus(point1, 0.0));
            assertTrue(triangle.isLocus(point1, 0.0));
            assertFalse(triangle.isLocus(center, 0.0));
        
            assertTrue(Triangle2D.isInside(triangle, point1, 0.0));
            assertTrue(Triangle2D.isInside(triangle, point2, 0.0));
            assertTrue(Triangle2D.isInside(triangle, point3, 0.0));

            assertTrue(Triangle2D.isInside(point1, point2, point3, point1, 
                    0.0));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point2, 
                    0.0));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point3, 
                    0.0));
        
            //Force IllegalArgumentException
            try{
                triangle.isInside(point1, -ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException e){}
            try{
                Triangle2D.isInside(triangle, point1, - ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException e){}
            try{
                Triangle2D.isInside(point1, point2, point3, point1, 
                        -ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException e){}
            
            try{
                triangle.isLocus(point3, -ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException e){}
        
            //Check point outside
            Point2D outside = new InhomogeneousPoint2D(-dist, 0.0);
        
            assertFalse(triangle.isInside(outside));
            
            //point outside is not locus
            assertFalse(triangle.isLocus(outside));
        
            assertFalse(Triangle2D.isInside(triangle, outside));
        
            assertFalse(Triangle2D.isInside(point1, point2, point3, outside));
        
            //the same is true for a small threshold, but point is considered to
            //be inside when setting large threshold
            assertFalse(triangle.isInside(outside, 0.0));
            assertTrue(triangle.isInside(outside, 
                    3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));    
        
            assertEquals(triangle.getShortestDistance(outside), dist, 
                    ABSOLUTE_ERROR);
        
            assertFalse(Triangle2D.isInside(triangle, outside, 0.0));
            assertTrue(Triangle2D.isInside(triangle, outside, 
                    3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));
            
            //check locus with a small and large threshold
            assertFalse(triangle.isLocus(outside, 0.0));
            assertTrue(triangle.isLocus(outside, 
                    3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));
        
            assertEquals(Triangle2D.shortestDistance(triangle, outside), dist, 
                    ABSOLUTE_ERROR);

            assertFalse(Triangle2D.isInside(point1, point2, point3, outside, 
                    0.0));
            assertTrue(Triangle2D.isInside(point1, point2, point3, outside, 
                    3.0 * MAX_RANDOM_VALUE * MAX_RANDOM_VALUE));
        
            assertEquals(Triangle2D.shortestDistance(point1, point2, point3, 
                    outside), dist, ABSOLUTE_ERROR);
        
        
            //Testing for a random triangle we can see that vertices and center 
            //lie inside the triangle
            point1 = new InhomogeneousPoint2D(randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            point2 = new InhomogeneousPoint2D(randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            point3 = new InhomogeneousPoint2D(randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
            triangle.setVertices(point1, point2, point3);
            center = triangle.getCenter();
        
            //vertices and center lie inside the triangle
            assertTrue(triangle.isInside(point1));
            assertTrue(triangle.isInside(point2));
            assertTrue(triangle.isInside(point3));
            assertTrue(triangle.isInside(center));
            
            //vertices are locus, but not center
            assertTrue(triangle.isLocus(point1, ABSOLUTE_ERROR));
            assertTrue(triangle.isLocus(point2, ABSOLUTE_ERROR));
            assertTrue(triangle.isLocus(point3, ABSOLUTE_ERROR));
            assertFalse(triangle.isLocus(center, ABSOLUTE_ERROR));
        
            assertEquals(triangle.getShortestDistance(point1), 0.0, 
                    ABSOLUTE_ERROR);
            assertEquals(triangle.getShortestDistance(point2), 0.0, 
                    ABSOLUTE_ERROR);
            assertEquals(triangle.getShortestDistance(point3), 0.0, 
                    ABSOLUTE_ERROR);
            line1 = new Line2D(point1, point2);
            line2 = new Line2D(point1, point3);
            line3 = new Line2D(point2, point3);
            dist1 = Math.abs(line1.signedDistance(center));
            dist2 = Math.abs(line2.signedDistance(center));
            dist3 = Math.abs(line3.signedDistance(center));
            centerDist = Math.min(dist1, Math.min(dist2, dist3));
            assertEquals(triangle.getShortestDistance(center), centerDist, 
                    ABSOLUTE_ERROR);
            
        
            assertTrue(Triangle2D.isInside(triangle, point1));
            assertTrue(Triangle2D.isInside(triangle, point2));
            assertTrue(Triangle2D.isInside(triangle, point3));
            assertTrue(Triangle2D.isInside(triangle, center));
        
            assertEquals(Triangle2D.shortestDistance(triangle, point1), 0.0, 
                    ABSOLUTE_ERROR);
            assertEquals(Triangle2D.shortestDistance(triangle, point2), 0.0, 
                    ABSOLUTE_ERROR);
            assertEquals(Triangle2D.shortestDistance(triangle, point3), 0.0, 
                    ABSOLUTE_ERROR);
            assertEquals(Triangle2D.shortestDistance(triangle, center), 
                    centerDist, ABSOLUTE_ERROR);
        
            assertTrue(Triangle2D.isInside(point1, point2, point3, point1));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point2));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point3));
            assertTrue(Triangle2D.isInside(point1, point2, point3, center));
        
            assertEquals(Triangle2D.shortestDistance(point1, point2, point3, 
                    point1), 0.0, ABSOLUTE_ERROR);
            assertEquals(Triangle2D.shortestDistance(point1, point2, point3, 
                    point2), 0.0, ABSOLUTE_ERROR);
            assertEquals(Triangle2D.shortestDistance(point1, point2, point3, 
                    point3), 0.0, ABSOLUTE_ERROR);
            assertEquals(Triangle2D.shortestDistance(point1, point2, point3, 
                    center), centerDist, ABSOLUTE_ERROR);
        
        
            //the same is true for a small threshold
            assertTrue(triangle.isInside(point1, ABSOLUTE_ERROR));
            assertTrue(triangle.isInside(point2, ABSOLUTE_ERROR));
            assertTrue(triangle.isInside(point3, ABSOLUTE_ERROR));
            assertTrue(triangle.isInside(center, ABSOLUTE_ERROR));
        
            assertTrue(Triangle2D.isInside(triangle, point1, ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(triangle, point2, ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(triangle, point3, ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(triangle, center, ABSOLUTE_ERROR));

            assertTrue(Triangle2D.isInside(point1, point2, point3, point1, 
                    ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point2, 
                    ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(point1, point2, point3, point3, 
                    ABSOLUTE_ERROR));
            assertTrue(Triangle2D.isInside(point1, point2, point3, center, 
                    ABSOLUTE_ERROR));
        }
    }
    
    @Test
    public void testCenter(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        Point2D point1 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point2 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point3 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        Triangle2D triangle = new Triangle2D(point1, point2, point3);

        Point2D expectedCenter = new InhomogeneousPoint2D((point1.getInhomX() +
                point2.getInhomX() + point3.getInhomX()) / 3.0, 
                (point1.getInhomY() + point2.getInhomY() + point3.getInhomY()) / 
                3.0);
        
        assertTrue(triangle.getCenter().equals(expectedCenter, ABSOLUTE_ERROR));
        
        Point2D center = Point2D.create();
        triangle.center(center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));
        
        assertTrue(expectedCenter.equals(Triangle2D.center(point1, point2, 
                point3), ABSOLUTE_ERROR));
        assertTrue(expectedCenter.equals(Triangle2D.center(triangle), 
                ABSOLUTE_ERROR));
        
        
        Triangle2D.center(point1, point2, point3, center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));
        
        Triangle2D.center(triangle, center);
        assertTrue(center.equals(expectedCenter, ABSOLUTE_ERROR));        
    }
    
    @Test
    public void testPerimeter(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        Point2D point1 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point2 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        Point2D point3 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        Triangle2D triangle = new Triangle2D(point1, point2, point3);

        double perimeter = point1.distanceTo(point2) + 
                point1.distanceTo(point3) + point3.distanceTo(point2);
        
        assertEquals(triangle.getPerimeter(), perimeter, ABSOLUTE_ERROR);
        assertEquals(Triangle2D.perimeter(triangle), perimeter, ABSOLUTE_ERROR);
        assertEquals(Triangle2D.perimeter(point1, point2, point3), perimeter,
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testClosestPoint(){
        
        for(int t = 0; t < TIMES; t++){
            //Test for known values
            UniformRandomizer randomizer = new UniformRandomizer(new Random());                
            double base = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                    MAX_RANDOM_VALUE));
            double height = Math.abs(randomizer.nextDouble(
                    MAX_RANDOM_VALUE / 2.0, MAX_RANDOM_VALUE));
            double dist = randomizer.nextDouble(Triangle2D.DEFAULT_THRESHOLD, 
                    MAX_RANDOM_VALUE);
            double area = 0.5 * base * height;
        
            //Test known and simple values
            Point2D point1 = new InhomogeneousPoint2D(0.0, 0.0);
            Point2D point2 = new InhomogeneousPoint2D(base, 0.0);
            Point2D point3 = new InhomogeneousPoint2D(base / 2.0, height);        
        
            Triangle2D triangle = new Triangle2D(point1, point2, point3);
        
            //try for point1
            Point2D testPoint = Point2D.create();
            testPoint.setInhomogeneousCoordinates(-base, 0.0);
            assertTrue(triangle.getClosestPoint(testPoint).equals(point1, 
                    ABSOLUTE_ERROR));
            assertTrue(triangle.getClosestPoint(testPoint, 
                    ABSOLUTE_ERROR).equals(point1, ABSOLUTE_ERROR));
        
            Point2D closestPoint = Point2D.create();
            triangle.closestPoint(testPoint, closestPoint);
            assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
            triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
            assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
        
            //try for point2
            testPoint.setInhomogeneousCoordinates(2.0 * base, 0.0);
            assertTrue(triangle.getClosestPoint(testPoint).equals(point2, 
                    ABSOLUTE_ERROR));
            assertTrue(triangle.getClosestPoint(testPoint, 
                    ABSOLUTE_ERROR).equals(point2, ABSOLUTE_ERROR));
        
            triangle.closestPoint(testPoint, closestPoint);
            assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));
            triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
            assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));
        
            //try for point3
            testPoint.setInhomogeneousCoordinates(base / 2.0, 2.0 * height);
            assertTrue(triangle.getClosestPoint(testPoint).equals(point3, 
                    ABSOLUTE_ERROR));
            assertTrue(triangle.getClosestPoint(testPoint, 
                    ABSOLUTE_ERROR).equals(point3, ABSOLUTE_ERROR));
       
            triangle.closestPoint(testPoint, closestPoint);
            assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));
            triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
            assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));

            //try for a point close to line1
            testPoint.setInhomogeneousCoordinates(base / 2.0, -height);        
            Point2D basePoint = new InhomogeneousPoint2D(base / 2.0, 0.0);        
            assertTrue(triangle.getClosestPoint(testPoint).equals(basePoint, 
                    ABSOLUTE_ERROR));
            assertTrue(triangle.getClosestPoint(testPoint, 
                    ABSOLUTE_ERROR).equals(basePoint, ABSOLUTE_ERROR));
        
            triangle.closestPoint(testPoint, closestPoint);
            assertTrue(closestPoint.equals(basePoint, ABSOLUTE_ERROR));
            triangle.closestPoint(testPoint, closestPoint, ABSOLUTE_ERROR);
            assertTrue(closestPoint.equals(basePoint, ABSOLUTE_ERROR));
        
        
            //try with test point on vertices
            
            //vertex1
            assertTrue(triangle.getClosestPoint(point1).equals(point1, 
                    ABSOLUTE_ERROR));
            assertTrue(triangle.getClosestPoint(point1, ABSOLUTE_ERROR).equals(
                    point1, ABSOLUTE_ERROR));
            triangle.closestPoint(point1, closestPoint);
            assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
            triangle.closestPoint(point1, closestPoint, ABSOLUTE_ERROR);
            assertTrue(closestPoint.equals(point1, ABSOLUTE_ERROR));
        
            //vertex2
            assertTrue(triangle.getClosestPoint(point2).equals(point2, 
                    ABSOLUTE_ERROR));
            assertTrue(triangle.getClosestPoint(point2, ABSOLUTE_ERROR).equals(
                    point2, ABSOLUTE_ERROR));
            triangle.closestPoint(point2, closestPoint);
            assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));
            triangle.closestPoint(point2, closestPoint, ABSOLUTE_ERROR);
            assertTrue(closestPoint.equals(point2, ABSOLUTE_ERROR));

            //vertex3
            assertTrue(triangle.getClosestPoint(point3).equals(point3, 
                    ABSOLUTE_ERROR));
            assertTrue(triangle.getClosestPoint(point3, ABSOLUTE_ERROR).equals(
                    point3, ABSOLUTE_ERROR));
            triangle.closestPoint(point3, closestPoint);
            assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));
            triangle.closestPoint(point3, closestPoint, ABSOLUTE_ERROR);
            assertTrue(closestPoint.equals(point3, ABSOLUTE_ERROR));

        
            //and now try for center
            Point2D center = triangle.getCenter();
            center.normalize();
            
            Line2D line1 = new Line2D(point1, point2);
            Line2D line2 = new Line2D(point1, point3);
            Line2D line3 = new Line2D(point2, point3);
            
            line1.normalize();
            line2.normalize();
            line3.normalize();
            
            
            Point2D pointLine1 = line1.getClosestPoint(center);
            Point2D pointLine2 = line2.getClosestPoint(center);
            Point2D pointLine3 = line3.getClosestPoint(center);
            
            pointLine1.normalize();
            pointLine2.normalize();
            pointLine3.normalize();
            
        
            //pick closest point among the three above to center
            double dist1 = pointLine1.distanceTo(center);
            double dist2 = pointLine2.distanceTo(center);
            double dist3 = pointLine3.distanceTo(center);
        
            Point2D linePoint = Point2D.create();
        
            if(dist1 < dist2 && dist1 < dist3){
                //pick pointLine1
                linePoint.setCoordinates(pointLine1);
            }else if(dist2 < dist1 && dist2 < dist3){
                //pick pointLine2
                linePoint.setCoordinates(pointLine2);
            }else{
                //pick pointLine3
                linePoint.setCoordinates(pointLine3);
            }
        
            //check correctness
            assertTrue(triangle.getClosestPoint(center).equals(linePoint, 
                    ABSOLUTE_ERROR));
            assertTrue(triangle.getClosestPoint(center, ABSOLUTE_ERROR).equals(
                    linePoint, ABSOLUTE_ERROR));
            triangle.closestPoint(center, closestPoint);
            assertTrue(closestPoint.equals(linePoint, ABSOLUTE_ERROR));
            triangle.closestPoint(center, closestPoint, ABSOLUTE_ERROR);
            assertTrue(closestPoint.equals(linePoint, ABSOLUTE_ERROR));
        }
    }
    
    @Test
    public void testAreVerticesClockwise(){
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());                
        double base = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        double height = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        
        //Test known and simple values
        Point2D point1 = new InhomogeneousPoint2D(0.0, 0.0);
        Point2D point2 = new InhomogeneousPoint2D(base, 0.0);
        Point2D point3 = new InhomogeneousPoint2D(base / 2.0, height);

        Triangle2D triangle1 = new Triangle2D(point1, point2, point3);
        Triangle2D triangle2 = new Triangle2D(point3, point2, point1);
        //we know that triangle1 is counterclockwise and triangle2 is clockwise
        assertFalse(triangle1.areVerticesClockwise());
        assertTrue(triangle2.areVerticesClockwise());
        
        //now try with random vertices
        point1 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point3 = new InhomogeneousPoint2D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        
        triangle1 = new Triangle2D(point1, point2, point3);
        double signedArea = triangle1.getSignedArea();
        
        //create triangle with vertices in reversed order
        triangle2 = new Triangle2D(point3, point2, point1);
        
        if(signedArea > 0.0){
            //points are clockwise
            assertFalse(triangle1.areVerticesClockwise());
            assertTrue(triangle2.areVerticesClockwise());
        }else{
            assertTrue(triangle1.areVerticesClockwise());
            assertFalse(triangle2.areVerticesClockwise());
        }
    }
}