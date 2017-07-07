/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.Polygon3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 20, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Utils;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class Polygon3DTest {
    
    public static final int MIN_SIDES = 6;
    public static final int MAX_SIDES = 12;
    
    public static final double MIN_RADIUS = 1.0;
    public static final double MAX_RADIUS = 10.0;
    
    public static final double MIN_ANGLE_DEGREES = -180.0;
    public static final double MAX_ANGLE_DEGREES = 180.0;
    
    public static final double ABSOLUTE_ERROR = 1e-8;
    public static final double LARGE_ABSOLUTE_ERROR = 1e-6;
    public static final double EPS = 1e-9;
    
    public static final int INHOM_COORDS = 3;
    
    public static final int TIMES = 100;
    
    
    public Polygon3DTest() {
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
    
    private double[] absArray(double[] array){
        double[] out = new double[array.length];
        for(int i = 0; i < array.length; i++){
            out[i] = Math.abs(array[i]);
        }
        return out;
    }
    
    @Test
    public void testConstructor() throws NotEnoughVerticesException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES);
        
        //build vertices list
        List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);
        
        //build polygon
        Polygon3D polygon = new Polygon3D(vertices);
        
        //check correctness
        assertFalse(polygon.isTriangulated());
        assertEquals(polygon.getVertices(), vertices);
        assertEquals(polygon.getTriangulatorMethod(), 
                Polygon3D.DEFAULT_TRIANGULATOR_METHOD);
        
        Iterator<Point3D> iterator1 = polygon.getVertices().iterator();
        Iterator<Point3D> iterator2 = vertices.iterator();
        
        Point3D vertex1, vertex2;
        while(iterator1.hasNext() && iterator2.hasNext()){
            vertex1 = iterator1.next();
            vertex2 = iterator2.next();
            assertTrue(vertex1.equals(vertex2, ABSOLUTE_ERROR));
        }
        
        //Force NotEnoughVerticesException
        vertex1 = vertices.get(0);
        vertex2 = vertices.get(1);
        vertices.clear();
        vertices.add(vertex1);
        vertices.add(vertex2);
        polygon = null;
        try{
            polygon = new Polygon3D(vertices);
            fail("NotEnoughVerticesException expected but not thrown");
        }catch(NotEnoughVerticesException e){}
        assertNull(polygon);
    }    
    
    @Test
    public void testGetSetTriangulatorMethod() throws NotEnoughVerticesException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES);
        
        
        //build vertices list
        List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);
        
        //build polygon
        Polygon3D polygon = new Polygon3D(vertices);

        //check correctness
        assertEquals(polygon.getTriangulatorMethod(), 
                Polygon3D.DEFAULT_TRIANGULATOR_METHOD);
        
        //set new method
        polygon.setTriangulatorMethod(TriangulatorMethod.VAN_GOGH_TRIANGULATOR);
        //check correctness
        assertEquals(polygon.getTriangulatorMethod(),
                TriangulatorMethod.VAN_GOGH_TRIANGULATOR);
    }
    
    @Test
    public void testGetSetVertices() throws NotEnoughVerticesException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES);
        
        
        //build vertices list
        List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);
        
        //build polygon vertices
        Polygon3D polygon = new Polygon3D(vertices);
        assertEquals(polygon.getVertices(), vertices);
        
        //build new vertices
        sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        
        List<Point3D> vertices2 = buildPolygonVertices(sides, radius, theta);        
        
        polygon.setVertices(vertices2);
        assertEquals(polygon.getVertices(), vertices2);
    }
    
    @Test
    public void testArea() throws NotEnoughVerticesException{
        //X: -1, Y: 0, Z: 5 (W: 1)
        Point3D v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        //X: 1, Y: 0, Z: 5 (W: 1)
        Point3D v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        //X: 2, Y: 1, Z: 5 (W: 1)
        Point3D v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        //X: 0, Y: 1, Z: 5 (W: 1)
        Point3D v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        List<Point3D> vertices = new ArrayList<Point3D>();
        vertices.add(v1);
        vertices.add(v2);
        vertices.add(v3);
        vertices.add(v4);
        
        Polygon3D polygon = new Polygon3D(vertices);
        assertEquals(polygon.getArea(), 2.0, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testTriangulateIsTriangulatedAndArea() 
            throws NotEnoughVerticesException, TriangulatorException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES);
        
        
        //build vertices list
        List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);
        
        Polygon3D polygon = new Polygon3D(vertices);
        
        VanGoghTriangulator3D triangulator = new VanGoghTriangulator3D();
        
        //expected area is the sum of the areas of the triangles forming this 
        //polygon
        double area = 0.0;
        for(Triangle3D triangle : triangulator.triangulate(vertices)){
            area += triangle.getArea();
        }
                
        assertFalse(polygon.isTriangulated());
        List<Triangle3D> triangles = polygon.getTriangles();
        assertTrue(polygon.isTriangulated());
        
        double areaTriangles = 0.0;
        for(Triangle3D triangle : triangles){
            areaTriangles += triangle.getArea();
        }
        
        
        assertEquals(polygon.getArea(), area, ABSOLUTE_ERROR);
        assertEquals(polygon.getArea(), areaTriangles, ABSOLUTE_ERROR);
    }
        
    @Test
    public void testPerimeter() throws NotEnoughVerticesException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES);
        
        //build vertices list
        List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);
        
        Polygon3D polygon = new Polygon3D(vertices);

        Iterator<Point3D> iterator = vertices.iterator();
        Point3D prevVertex = iterator.next();
        Point3D curVertex;
        double perimeter = 0.0;
        while(iterator.hasNext()){
            curVertex = iterator.next();
            perimeter += curVertex.distanceTo(prevVertex);
            prevVertex = curVertex;
        }
        
        //compare distance of last vertex with first one
        perimeter += prevVertex.distanceTo(vertices.get(0));
        
        assertEquals(polygon.getPerimeter(), perimeter, ABSOLUTE_ERROR);
        
        
        //Test for a triangle
        sides = 3;
        List<Point3D> vertices2 = buildPolygonVertices(sides, radius, theta);
        Triangle3D triangle = new Triangle3D(vertices2.get(0), vertices2.get(1),
                vertices2.get(2));
        polygon.setVertices(vertices2);
        assertEquals(polygon.getPerimeter(), triangle.getPerimeter(), 
                ABSOLUTE_ERROR);
    }  
    

    @Test
    public void testIsInside() throws NotEnoughVerticesException, 
        TriangulatorException{
        
        //for(int t = 0; t < TIMES; t++){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        double phi = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES);

        
        //build vertices list
        List<Point3D> vertices = buildPolygonVertices(sides, radius, phi);
        
        Polygon3D polygon = new Polygon3D(vertices);

        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        //create point inside of vertex
        
            
        Point3D inside = new InhomogeneousPoint3D(
                radius / 2.0 * Math.cos(theta) * Math.cos(phi),
                radius / 2.0 * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));
        
        //create point outside of vertex
        Point3D outside = new InhomogeneousPoint3D(
                2.0 * radius * Math.cos(theta) * Math.cos(phi),
                2.0 * radius * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));
        
        assertTrue(polygon.isInside(inside));
        assertTrue(polygon.isInside(inside, ABSOLUTE_ERROR));
        assertFalse(polygon.isInside(outside));
        assertFalse(polygon.isInside(outside, ABSOLUTE_ERROR));
        
        //check that vertices are inside
        for(Point3D vertex: vertices){
            assertTrue(polygon.isInside(vertex));
            assertTrue(polygon.isInside(vertex, ABSOLUTE_ERROR));
        }
        
        //Force IllegalArgumentException
        try{
            polygon.isInside(inside, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        //}
    }
    
    @Test
    public void testCenter() throws NotEnoughVerticesException{
        //Generate random vertices
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES);
        
        
        //build vertices list
        List<Point3D> vertices = buildPolygonVertices(sides, radius, theta);

        
        double inhomX = 0.0, inhomY = 0.0, inhomZ = 0.0;
        for(Point3D vertex: vertices){
            inhomX += vertex.getInhomX() / (double)sides;
            inhomY += vertex.getInhomY() / (double)sides;
            inhomZ += vertex.getInhomZ() / (double)sides;
        }
        
        Point3D center = new InhomogeneousPoint3D(inhomX, inhomY, inhomZ);
        
        Polygon3D polygon = new Polygon3D(vertices);
        
        assertTrue(center.equals(polygon.getCenter(), ABSOLUTE_ERROR));
        Point3D center2 = Point3D.create();
        polygon.center(center2);
        assertTrue(center.equals(center2, ABSOLUTE_ERROR));
    } 
    
    @Test
    public void testIsLocusGetShortestDistanceAndClosestPoint() 
            throws NotEnoughVerticesException, TriangulatorException, 
            CoincidentPointsException{
        
        for(int t = 0; t < TIMES; t++){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        double dist = randomizer.nextDouble(2.0 * ABSOLUTE_ERROR, radius / 2.0);
        double phi = 0.0; //randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                //MAX_ANGLE_DEGREES);
        
        
        double theta = randomizer.nextDouble(0.0, 
                2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        
        int n = (int)(theta / (2 * Math.PI) * (double)sides);        
        
        List<Point3D> vertices = buildPolygonVertices(sides, radius, phi);
        
        Point3D point1 = vertices.get(n);
        n = (n + 1) % sides;
        Point3D point2 = vertices.get(n);        
        
        Line3D line = new Line3D(point1, point2);
        line.normalize();
        
        Line2D line2D = new Line2D(
                new InhomogeneousPoint2D(point1.getInhomX(), point1.getInhomY()),
                new InhomogeneousPoint2D(point2.getInhomX(), point2.getInhomY()));
        line2D.normalize(); //to increase accuracy
        assertEquals(point1.getInhomZ(), point2.getInhomZ(), ABSOLUTE_ERROR);        
        
        double[] direction = line2D.getDirectorVector();
        double norm = Utils.normF(direction);
        ArrayUtils.multiplyByScalar(direction, 1.0 / norm, direction);
        
        //find point laying on line between polygon vertices
        Point3D testPoint = new InhomogeneousPoint3D(
                radius * Math.cos(theta) * Math.cos(phi),
                radius * Math.sin(theta) * Math.cos(phi),
                radius * Math.sin(phi));
        //point below is locus of polygon
        Point3D locusPoint = line.getClosestPoint(testPoint);
        //locus point is between point1 and point2
        assertTrue(locusPoint.isBetween(point1, point2));
        
        //move point a little bit away from locus
        Point3D notLocusPoint = new InhomogeneousPoint3D(
                locusPoint.getInhomX() + dist * direction[0],
                locusPoint.getInhomY() + dist * direction[1],
                locusPoint.getInhomZ());
        //ensure that notLocusPoint lies outside of polygon
        Polygon3D polygon = new Polygon3D(vertices);
        
        if(polygon.isInside(notLocusPoint)){
            //change sign of dist to move point in opposite direction
            dist *= -1.0;
            notLocusPoint.setInhomogeneousCoordinates(
                locusPoint.getInhomX() + dist * direction[0],
                locusPoint.getInhomY() + dist * direction[1],
                locusPoint.getInhomZ());            
        }
        assertFalse(polygon.isInside(notLocusPoint));
        
        //check that notLocusPoint is at distance dist from line
        assertEquals(line.getDistance(notLocusPoint), Math.abs(dist), 
                ABSOLUTE_ERROR);
        
        
        
        assertTrue(polygon.isLocus(locusPoint));
        assertTrue(polygon.isLocus(locusPoint, ABSOLUTE_ERROR));
        
        //because point is locus, the shortest distance is zero and it is the
        //closest point
        assertEquals(polygon.getShortestDistance(locusPoint), 0.0, 
                ABSOLUTE_ERROR);
        assertTrue(polygon.getClosestPoint(locusPoint).equals(locusPoint, 
                ABSOLUTE_ERROR));
        Point3D closestPoint = Point3D.create();
        polygon.closestPoint(locusPoint, closestPoint);
        assertTrue(closestPoint.equals(locusPoint, ABSOLUTE_ERROR));
        assertTrue(closestPoint.equals(locusPoint, ABSOLUTE_ERROR));
                
        
        assertFalse(polygon.isLocus(notLocusPoint));
        assertFalse(polygon.isLocus(notLocusPoint, EPS));
        
        //not locus point is at distance dist from polygon
        assertEquals(polygon.getShortestDistance(notLocusPoint), Math.abs(dist),
                ABSOLUTE_ERROR);
        
        //and the closest point to polygon is locusPoint
        assertTrue(polygon.getClosestPoint(notLocusPoint).equals(
                new InhomogeneousPoint3D(locusPoint), 
                ABSOLUTE_ERROR));
        
        polygon.closestPoint(notLocusPoint, closestPoint);
        assertTrue(closestPoint.equals(new InhomogeneousPoint3D(locusPoint), 
                ABSOLUTE_ERROR));
        assertTrue(closestPoint.equals(new InhomogeneousPoint3D(locusPoint), 
                ABSOLUTE_ERROR));

        
        //with a large enough threshold, not locus point is considered as locus
        assertTrue(polygon.isLocus(notLocusPoint, radius * radius));
        
        //all vertices of polygon are also locus
        for(Point3D vertex : vertices){
            assertTrue(polygon.isLocus(vertex));
            assertTrue(polygon.isLocus(vertex, ABSOLUTE_ERROR));
            //because vertices are locus, shortest distance is 0.0 and it is
            //a closest point
            assertEquals(polygon.getShortestDistance(vertex), 0.0, 
                    ABSOLUTE_ERROR);
            assertTrue(polygon.getClosestPoint(vertex).equals(vertex, 
                    ABSOLUTE_ERROR));
            polygon.closestPoint(vertex, closestPoint);
            assertTrue(closestPoint.equals(vertex, ABSOLUTE_ERROR));
            assertTrue(closestPoint.equals(vertex, ABSOLUTE_ERROR));
        }
        
        //Force IllegalArgumentException
        try{
            polygon.isLocus(locusPoint, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        }
    }    
    
    @Test
    public void testOrientation() throws NotEnoughVerticesException, 
        CoincidentPointsException,
        ColinearPointsException{
        
        //Test 1st for known values
        
        //X: -1, Y: 0, Z: 5 (W: 1)
        Point3D v1 = new InhomogeneousPoint3D(-1.0, 0.0, 5.0);
        //X: 1, Y: 0, Z: 5 (W: 1)
        Point3D v2 = new InhomogeneousPoint3D(1.0, 0.0, 5.0);
        //X: 2, Y: 1, Z: 5 (W: 1)
        Point3D v3 = new InhomogeneousPoint3D(2.0, 1.0, 5.0);
        //X: 0, Y: 1, Z: 5 (W: 1)
        Point3D v4 = new InhomogeneousPoint3D(0.0, 1.0, 5.0);

        List<Point3D> vertices = new ArrayList<Point3D>();
        vertices.add(v1);
        vertices.add(v2);
        vertices.add(v3);
        vertices.add(v4);
        
        Polygon3D polygon = new Polygon3D(vertices);        
        
        double[] expectedOrientation = new double[INHOM_COORDS];
        expectedOrientation[0] = 0.0;
        expectedOrientation[1] = 0.0;
        expectedOrientation[2] = 1.0;
        
        double[] orientation = polygon.getOrientation();
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        orientation = polygon.getOrientation(ABSOLUTE_ERROR);
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        orientation = new double[INHOM_COORDS];
        polygon.orientation(orientation);
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        polygon.orientation(orientation, ABSOLUTE_ERROR);
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(vertices);
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(vertices, ABSOLUTE_ERROR);
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(polygon);
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(polygon, ABSOLUTE_ERROR);
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        orientation = new double[INHOM_COORDS];
        Polygon3D.orientation(vertices, orientation);
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        Polygon3D.orientation(vertices, orientation, ABSOLUTE_ERROR);
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon, orientation);
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon, orientation, ABSOLUTE_ERROR);
        assertArrayEquals(orientation, expectedOrientation, ABSOLUTE_ERROR);
        
        
        //Force IllegalArgumentException
        try{
            polygon.getOrientation(-ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            polygon.orientation(orientation, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            Polygon3D.orientation(vertices, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            Polygon3D.orientation(polygon, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            Polygon3D.orientation(vertices, orientation, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            Polygon3D.orientation(polygon, orientation, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}



        orientation = new double[INHOM_COORDS + 1];
        try{
            polygon.orientation(orientation);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            polygon.orientation(orientation, ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            Polygon3D.orientation(vertices, orientation);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            Polygon3D.orientation(vertices, orientation, ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            Polygon3D.orientation(polygon, orientation);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            Polygon3D.orientation(polygon, orientation, ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}

        
        
        //Force CoincidentPointsException
        vertices.clear();
        vertices.add(v1);
        vertices.add(v1);
        vertices.add(v1);
        polygon = new Polygon3D(vertices);
        orientation = new double[INHOM_COORDS];
        try{
            polygon.getOrientation();
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        try{
            polygon.getOrientation(ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        try{
            polygon.orientation(orientation);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        try{
            polygon.orientation(orientation, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        try{
            Polygon3D.orientation(vertices);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}        
        try{
            Polygon3D.orientation(vertices, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        try{
            Polygon3D.orientation(polygon);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}        
        try{
            Polygon3D.orientation(polygon, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        try{
            Polygon3D.orientation(vertices, orientation);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}        
        try{
            Polygon3D.orientation(vertices, orientation, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        try{
            Polygon3D.orientation(polygon, orientation);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}        
        try{
            Polygon3D.orientation(polygon, orientation, ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        
        

        //now test with random values
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int sides = randomizer.nextInt(MIN_SIDES, MAX_SIDES);
        double radius = randomizer.nextDouble(MIN_RADIUS, MAX_RADIUS);
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                MAX_ANGLE_DEGREES);
        
        
        //build vertices list
        vertices = buildPolygonVertices(sides, radius, theta);

        polygon = new Polygon3D(vertices);
        
        //build plane from 3 vertices
        Plane plane = new Plane(vertices.get(0), vertices.get(1), 
                vertices.get(2));
        expectedOrientation = plane.getDirectorVector();
        //normalize it
        double norm = Utils.normF(expectedOrientation);
        ArrayUtils.multiplyByScalar(expectedOrientation, 1.0 / norm, 
                expectedOrientation);
        
        
        //check correctness
        orientation = polygon.getOrientation();
        
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), 
                ABSOLUTE_ERROR);
        orientation = polygon.getOrientation(ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), 
                ABSOLUTE_ERROR);
        orientation = new double[INHOM_COORDS];
        polygon.orientation(orientation);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), 
                ABSOLUTE_ERROR);
        polygon.orientation(orientation, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), 
                ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(vertices);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), 
                ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(vertices, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), 
                ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(polygon);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), 
                ABSOLUTE_ERROR);
        orientation = Polygon3D.orientation(polygon, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), 
                ABSOLUTE_ERROR);
        orientation = new double[INHOM_COORDS];
        Polygon3D.orientation(vertices, orientation);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation), 
                ABSOLUTE_ERROR);
        Polygon3D.orientation(vertices, orientation, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation),
                ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon, orientation);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation),
                ABSOLUTE_ERROR);
        Polygon3D.orientation(polygon, orientation, ABSOLUTE_ERROR);
        assertArrayEquals(absArray(orientation), absArray(expectedOrientation),
                ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testAngleBetweenPolygons() throws NotEnoughVerticesException, 
        CoincidentPointsException{
        
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
        
        
        //Force IllegalArgumentException
        try{
            Polygon3D.getAngleBetweenPolygons(vertices1, vertices2, 
                    -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            Polygon3D.getAngleBetweenPolygons(polygon1, polygon2, 
                    -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //Force CoincidentPointsException
        vertices1.clear();
        vertices1.add(v1);
        vertices1.add(v1);
        vertices1.add(v1);
        
        vertices2.clear();
        vertices2.add(v2);
        vertices2.add(v2);
        vertices2.add(v2);
        
        polygon1.setVertices(vertices1);
        polygon2.setVertices(vertices2);
        try{
            Polygon3D.getAngleBetweenPolygons(vertices1, vertices2);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        try{
            Polygon3D.getAngleBetweenPolygons(vertices1, vertices2, 
                    ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        try{
            Polygon3D.getAngleBetweenPolygons(polygon1, polygon2);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}
        try{
            Polygon3D.getAngleBetweenPolygons(polygon1, polygon2, 
                    ABSOLUTE_ERROR);
            fail("CoincidentPointsException expected but not thrown");
        }catch(CoincidentPointsException e){}        
    }
}
