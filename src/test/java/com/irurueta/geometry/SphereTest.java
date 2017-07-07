/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.Sphere
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 14, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class SphereTest {
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double MIN_RANDOM_DEGREES = -180.0;
    public static final double MAX_RANDOM_DEGREES = 180.0;
    
    public static final int TIMES = 100;
    
    public SphereTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructor() throws CoplanarPointsException{
        //Test empty constructor
        Sphere sphere = new Sphere();
        
        //check center is at origin and radius is 1.0;
        assertTrue(sphere.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0,
                0.0), ABSOLUTE_ERROR));
        assertEquals(sphere.getRadius(), 1.0, ABSOLUTE_ERROR);
        
        //Test constructor with center and radius
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));
        
        sphere = new Sphere(center, radius);
        //check correctness
        assertTrue(sphere.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(sphere.getRadius(), radius, ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        sphere = null;
        try{
            sphere = new Sphere(center, -radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(sphere);
        
        //test constructor with four points
        
        //pick 4 points belonging to the sphere locus
        Point3D point1, point2, point3, point4;
        boolean areEqual;
        do{
            double angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            double angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point1 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * 
                    Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * 
                    Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2), 
                    1.0);   
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point2 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * 
                    Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * 
                    Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2), 
                    1.0);   
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point3 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * 
                    Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * 
                    Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2), 
                    1.0);   
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point4 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * 
                    Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * 
                    Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2), 
                    1.0);   
            
            //ensure that all four points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) ||
                    point2.equals(point3, ABSOLUTE_ERROR) ||
                    point3.equals(point4, ABSOLUTE_ERROR) ||
                    point4.equals(point1, ABSOLUTE_ERROR);
        }while(areEqual);
        
        //compute sphere
        Sphere sphere2 = new Sphere(point1, point2, point3, point4);
        
        //check that both spheres are equal
        sphere = new Sphere(center, radius);
        assertEquals(sphere.getCenter().distanceTo(sphere2.getCenter()), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(sphere.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);
        
        //Force CoplanarPointsException
        sphere = null;
        try{
            sphere = new Sphere(point1, point2, point2, point4);
            fail("CoplanarPointsException expected but not thrown");
        }catch(CoplanarPointsException e){}
        assertNull(sphere);
        
        //test from quadric
        sphere = new Sphere(center, radius);
        Quadric quadric = sphere.toQuadric();
        sphere2 = new Sphere(quadric);
        assertEquals(sphere.getCenter().distanceTo(sphere2.getCenter()), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(sphere.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        quadric = new Quadric();
        sphere = null;
        try{
            sphere = new Sphere(quadric);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        assertNull(sphere);
    }
    
    @Test
    public void testGetSetCenter(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        Sphere sphere = new Sphere();
        //check center
        assertTrue(sphere.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0,
                0.0), ABSOLUTE_ERROR));
        
        //set center
        sphere.setCenter(center);
        //check correctness
        assertTrue(sphere.getCenter().equals(center, ABSOLUTE_ERROR));
        
        //Force NullPointerException
        try{
            sphere.setCenter(null);
            fail("NullPointerException expected but not thrown");
        }catch(NullPointerException e){}
    }
    
    @Test
    public void testGetSetRadius(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));
        
        Sphere sphere = new Sphere();
        //check radius
        assertEquals(sphere.getRadius(), 1.0, 0.0);
        
        //set radius
        sphere.setRadius(radius);
        //check correctness
        assertEquals(sphere.getRadius(), radius, 0.0);
        
        //Force IllegalArgumentException
        try{
            sphere.setRadius(-radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }
    
    @Test
    public void testSetCenterAndRadius(){
        //Test constructor with center and radius
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
              
        Sphere sphere = new Sphere();
        //check center
        assertTrue(sphere.getCenter().equals(new InhomogeneousPoint3D(0.0, 0.0,
                0.0), ABSOLUTE_ERROR));    
        //check radius
        assertEquals(sphere.getRadius(), 1.0, 0.0);
        
        //set center and radius
        sphere.setCenterAndRadius(center, radius);
        //check correctness
        assertTrue(sphere.getCenter().equals(center, ABSOLUTE_ERROR));
        assertEquals(sphere.getRadius(), radius, 0.0);
        
        //Force IllegalArgumentException
        try{
            sphere.setCenterAndRadius(center, -radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        //Force NullPointerException
        try{
            sphere.setCenterAndRadius(null, radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(NullPointerException e){}
    }
    
    @Test
    public void testSetParametersFromPoints() throws CoplanarPointsException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE));
        
        Sphere sphere1 = new Sphere(center, radius);
        
        //pick 4 points belonging to the sphere locus
        Point3D point1, point2, point3, point4;
        boolean areEqual;
        do{
            double angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            double angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point1 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * 
                    Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * 
                    Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2), 
                    1.0);   
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point2 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * 
                    Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * 
                    Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2), 
                    1.0);   
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point3 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * 
                    Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * 
                    Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2), 
                    1.0);   
            angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            point4 = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * 
                    Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * 
                    Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2), 
                    1.0);   
            
            //ensure that all four points are different
            areEqual = point1.equals(point2, ABSOLUTE_ERROR) ||
                    point2.equals(point3, ABSOLUTE_ERROR) ||
                    point3.equals(point4, ABSOLUTE_ERROR) ||
                    point4.equals(point1, ABSOLUTE_ERROR);
        }while(areEqual);
        
        //create new sphere and set parameters
        Sphere sphere2 = new Sphere();
        
        sphere2.setParametersFromPoints(point1, point2, point3, point4);
        
        //check that both spheres are equal
        assertEquals(sphere1.getCenter().distanceTo(sphere2.getCenter()), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(sphere1.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);
        
        //Force CoplanarPointsException
        try{
            sphere2.setParametersFromPoints(point1, point2, point2, point4);
            fail("CoplanarPointsException expected but not thrown");
        }catch(CoplanarPointsException e){}
    }
    
    @Test
    public void testVolume(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));

        Sphere sphere = new Sphere(center, radius);
        
        double volume = 4.0 / 3.0 * Math.PI * radius * radius * radius;
        
        //Check correctness
        assertEquals(sphere.getVolume(), volume, ABSOLUTE_ERROR);
        assertEquals(Sphere.volume(radius), volume, ABSOLUTE_ERROR);        
    }
    
   @Test
    public void testSurface(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));

        Sphere sphere = new Sphere(center, radius);
        
        double surface = 4.0 * Math.PI * radius * radius;
        
        //Check correctness
        assertEquals(sphere.getSurface(), surface, ABSOLUTE_ERROR);
        assertEquals(Sphere.surface(radius), surface, ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testIsInside(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        double value = randomizer.nextDouble(0.2, 0.8);
        double value2 = 1.0 + value;
        double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        double phi = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        

        Sphere sphere = new Sphere(center, radius);

        Point3D inside = new InhomogeneousPoint3D(
                center.getInhomX() + value * radius * Math.cos(theta) * 
                Math.sin(phi),
                center.getInhomY() + value * radius * Math.sin(theta) * 
                Math.sin(phi),
                center.getInhomZ() + value * radius * Math.cos(phi));
        Point3D outside = new InhomogeneousPoint3D(
                center.getInhomX() + value2 * radius * Math.cos(theta) *
                Math.sin(phi),
                center.getInhomY() + value2 * radius * Math.sin(theta) *
                Math.sin(phi),
                center.getInhomZ() + value2 * radius * Math.cos(phi));
        
        //check correctness
        assertTrue(sphere.isInside(inside));
        assertTrue(sphere.isInside(inside, ABSOLUTE_ERROR));        
        
        assertFalse(sphere.isInside(outside));
        assertFalse(sphere.isInside(outside, ABSOLUTE_ERROR));
        
        //test for a large positive threshold 
        assertTrue(sphere.isInside(inside, radius));
        assertTrue(sphere.isInside(outside, radius));
        
        assertFalse(sphere.isInside(inside, -radius));
        assertFalse(sphere.isInside(outside, -radius));        
    }

    @Test
    public void testSignedDistanceDistanceAndIsLocus(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        double value = randomizer.nextDouble(0.2, 0.8);
        double value2 = 1.0 + value;
        double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        double phi = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        

        Sphere sphere = new Sphere(center, radius);
        
        //center is not locus
        assertFalse(sphere.isLocus(center));
        assertFalse(sphere.isLocus(center, ABSOLUTE_ERROR));
        //but for a large enough threshold it might be
        assertTrue(sphere.isLocus(center, 2.0 * radius));

        Point3D inside = new InhomogeneousPoint3D(
                center.getInhomX() + value * radius * Math.cos(theta) *
                Math.sin(phi),
                center.getInhomY() + value * radius * Math.sin(theta) *
                Math.sin(phi),
                center.getInhomZ() + value * radius * Math.cos(phi));
        Point3D outside = new InhomogeneousPoint3D(
                center.getInhomX() + value2 * radius * Math.cos(theta) *
                Math.sin(phi),
                center.getInhomY() + value2 * radius * Math.sin(theta) *
                Math.sin(phi),
                center.getInhomZ() + value2 * radius * Math.cos(phi));
        Point3D zero = new InhomogeneousPoint3D(
                center.getInhomX() + radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + radius * Math.cos(phi));
        
        //check correctness
        assertEquals(sphere.getSignedDistance(inside), (value - 1.0) * radius, 
                ABSOLUTE_ERROR);
        assertEquals(Sphere.signedDistance(sphere, inside), 
                (value - 1.0) * radius, ABSOLUTE_ERROR);
        
        assertEquals(sphere.getDistance(inside), 
                Math.abs((value - 1.0) * radius), ABSOLUTE_ERROR);
        assertEquals(Sphere.distance(sphere, inside), 
                Math.abs((value - 1.0) * radius), ABSOLUTE_ERROR);
                
        //for inside point signed distance is negative        
        assertTrue(sphere.getSignedDistance(inside) <= 0.0);
        
        assertFalse(sphere.isLocus(inside));
        assertFalse(sphere.isLocus(inside, ABSOLUTE_ERROR));
        assertTrue(sphere.isLocus(inside, radius)); //true for a large anough 
                                                    //threshold        
        
        assertEquals(sphere.getSignedDistance(outside), (value2 - 1.0) * radius, 
                ABSOLUTE_ERROR);
        assertEquals(Sphere.signedDistance(sphere, outside), 
                (value2 - 1.0) * radius, ABSOLUTE_ERROR);
        
        assertEquals(sphere.getDistance(outside), 
                Math.abs((value2 - 1.0) * radius), ABSOLUTE_ERROR);
        assertEquals(Sphere.distance(sphere, outside), 
                Math.abs((value2 - 1.0) * radius), ABSOLUTE_ERROR);
        
        
        //for outside point distance is positive
        assertTrue(sphere.getSignedDistance(outside) >= 0.0);

        assertFalse(sphere.isLocus(outside));
        assertFalse(sphere.isLocus(outside, ABSOLUTE_ERROR));
        assertTrue(sphere.isLocus(outside, radius)); //true for a large anough 
                                                    //threshold
        
        
        //for point at locus of circle, distance is zero
        assertEquals(sphere.getSignedDistance(zero), 0.0, ABSOLUTE_ERROR);
        assertEquals(Sphere.signedDistance(sphere, zero), 0.0, ABSOLUTE_ERROR);
        
        //zero is locus
        assertTrue(sphere.isLocus(zero));
        assertTrue(sphere.isLocus(zero, radius));
        
        //Force IllegalArgumentExcepetion
        try{
            sphere.isLocus(zero, -radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testClosestPointAndIsLocus() throws UndefinedPointException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        double value = randomizer.nextDouble(0.2, 0.8);
        double value2 = 1.0 + value;
        double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        double phi = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        

        Sphere sphere = new Sphere(center, radius);
        
        //center is not locus
        assertFalse(sphere.isLocus(center));
        assertFalse(sphere.isLocus(center, ABSOLUTE_ERROR));
        //but for a large enough threshold it might be
        assertTrue(sphere.isLocus(center, 2.0 * radius));
        

        Point3D inside = new InhomogeneousPoint3D(
                center.getInhomX() + value * radius * Math.cos(theta) * 
                Math.sin(phi),
                center.getInhomY() + value * radius * Math.sin(theta) *
                Math.sin(phi),
                center.getInhomZ() + value * radius * Math.cos(phi));
        Point3D outside = new InhomogeneousPoint3D(
                center.getInhomX() + value2 * radius * Math.cos(theta) *
                Math.sin(phi),
                center.getInhomY() + value2 * radius * Math.sin(theta) *
                Math.sin(phi),
                center.getInhomZ() + value2 * radius * Math.cos(phi));
        Point3D zero = new InhomogeneousPoint3D(
                center.getInhomX() + radius * Math.cos(theta) * Math.sin(phi),
                center.getInhomY() + radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + radius * Math.cos(phi));     
        
        Point3D expectedPoint = new InhomogeneousPoint3D(zero);
        
        Point3D result = Point3D.create();
        
        //test for point inside (but far from center)
        assertTrue(sphere.getClosestPoint(inside).equals(expectedPoint, 
                ABSOLUTE_ERROR));
        sphere.closestPoint(inside, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));
        
        assertFalse(sphere.isLocus(inside));
        assertFalse(sphere.isLocus(inside, ABSOLUTE_ERROR));
        assertTrue(sphere.isLocus(inside, radius)); //true for a large enough 
                                                    //threshold        
        
        
        //test for point outside of circle
        assertTrue(sphere.getClosestPoint(outside).equals(expectedPoint, 
                ABSOLUTE_ERROR));
        sphere.closestPoint(outside, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));
        
        assertFalse(sphere.isLocus(outside));
        assertFalse(sphere.isLocus(outside, ABSOLUTE_ERROR));
        assertTrue(sphere.isLocus(outside, radius)); //true for a large enough 
                                                    //threshold
        
        
        //test for point in circle boundary
        assertTrue(sphere.getClosestPoint(zero).equals(expectedPoint, 
                ABSOLUTE_ERROR));
        sphere.closestPoint(zero, result);
        assertTrue(result.equals(expectedPoint, ABSOLUTE_ERROR));
        
        //zero is locus
        assertTrue(sphere.isLocus(zero));
        assertTrue(sphere.isLocus(zero, radius));
        
        
        //Force UndefinedPointException (by testing at center)
        try{
            sphere.getClosestPoint(center);
            fail("UndefinedPointException expected but not thrown");
        }catch(UndefinedPointException e){}
        try{
            sphere.closestPoint(center, result);
            fail("UndefinedPointException expected but not thrown");
        }catch(UndefinedPointException e){}
        
        //Force IllegalArgumentExcepetion
        try{
            sphere.isLocus(zero, -radius);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }  
    
    @Test
    public void testGetTangentPlaneAt() throws NotLocusException{
        for(int t = 0; t < TIMES; t++){
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE));
        
            Sphere sphere = new Sphere(center, radius);
            
            double angle1 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            double angle2 = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                    MAX_RANDOM_DEGREES) * Math.PI / 180.0;
            Point3D point = new HomogeneousPoint3D(
                    center.getInhomX() + radius * Math.cos(angle1) * 
                    Math.cos(angle2),
                    center.getInhomY() + radius * Math.sin(angle1) * 
                    Math.cos(angle2),
                    center.getInhomZ() + radius * Math.sin(angle2), 
                    1.0);   
            point.normalize();
            
            assertTrue(sphere.isLocus(point));
            
            //find tangent plane at locus point
            Plane plane = sphere.getTangentPlaneAt(point);
            
            //check that point is also at plane's locus
            assertTrue(plane.isLocus(point));
            
            double[] directorVector = plane.getDirectorVector();
        
            double[] pointVector = new double[]{
                point.getInhomX() - center.getInhomX(),
                point.getInhomY() - center.getInhomY(),
                point.getInhomZ() - center.getInhomZ()
            };
            
            //normalize both vectors
            double norm1 = com.irurueta.algebra.Utils.normF(directorVector);
            double norm2 = com.irurueta.algebra.Utils.normF(pointVector);
            
            double[] vector1 = new double[3];
            double[] vector2 = new double[3];
            ArrayUtils.multiplyByScalar(directorVector, 1.0 / norm1, vector1);
            ArrayUtils.multiplyByScalar(pointVector, 1.0 / norm2, vector2);
            //check that both normalized vectors are equal
            assertArrayEquals(vector1, vector2, ABSOLUTE_ERROR);
        }
    }
    
    @Test
    public void testToQuadric() throws WrongSizeException{
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new HomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 1.0);
        double radius = Math.abs(randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE));
        double theta = randomizer.nextDouble(MIN_RANDOM_DEGREES, 
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        double phi = randomizer.nextDouble(MIN_RANDOM_DEGREES,
                MAX_RANDOM_DEGREES) * Math.PI / 180.0;
        center.normalize();

        Sphere sphere = new Sphere(center, radius);
        Quadric quadric = sphere.toQuadric();
        
        //center is not locus
        assertFalse(sphere.isLocus(center));
        assertFalse(sphere.isLocus(center, ABSOLUTE_ERROR));
        assertFalse(quadric.isLocus(center));
        assertFalse(quadric.isLocus(center, ABSOLUTE_ERROR));
        //but for a large enough threshold it might be
        assertTrue(sphere.isLocus(center, 2.0 * radius));
        assertTrue(quadric.isLocus(center, 2.0 * radius));


        Point3D locus = new HomogeneousPoint3D(
                center.getInhomX() + radius * Math.sin(theta) * Math.cos(phi),
                center.getInhomY() + radius * Math.sin(theta) * Math.sin(phi),
                center.getInhomZ() + radius * Math.cos(theta),
                1.0); 
        locus.normalize();
        
        //check is locus
        assertTrue(sphere.isLocus(locus, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(locus, ABSOLUTE_ERROR));
        
        //check correctness of estimated quadric matrix (up to scale)
        Matrix m = new Matrix(Quadric.BASEQUADRIC_MATRIX_ROW_SIZE,
                Quadric.BASEQUADRIC_MATRIX_COLUMN_SIZE);
        m.setElementAt(0, 0, 1.0);
        m.setElementAt(1, 0, 0.0);
        m.setElementAt(2, 0, 0.0);
        m.setElementAt(3, 0, -center.getInhomX());
        
        m.setElementAt(0, 1, 0.0);
        m.setElementAt(1, 1, 1.0);
        m.setElementAt(2, 1, 0.0);
        m.setElementAt(3, 1, -center.getInhomY());
        
        m.setElementAt(0, 2, 0.0);
        m.setElementAt(1, 2, 0.0);
        m.setElementAt(2, 2, 1.0);
        m.setElementAt(3, 2, -center.getInhomZ());
        
        m.setElementAt(0, 3, -center.getInhomX());
        m.setElementAt(1, 3, -center.getInhomY());
        m.setElementAt(2, 3, -center.getInhomZ());
        m.setElementAt(3, 3, center.getInhomX() * center.getInhomX() +
                center.getInhomY() * center.getInhomY() + 
                center.getInhomZ() * center.getInhomZ() - radius * radius);
        
        double norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);
        
        assertTrue(m.equals(quadric.asMatrix(), ABSOLUTE_ERROR));
    }   
    
    @Test
    public void testSetFromQuadric(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE));
        
        Sphere sphere = new Sphere(center, radius);
        
        //test from quadric
        Quadric quadric = sphere.toQuadric();
        Sphere sphere2 = new Sphere();
        sphere2.setFromQuadric(quadric);
        
        //check correctness
        assertEquals(sphere.getCenter().distanceTo(sphere2.getCenter()), 0.0, 
                ABSOLUTE_ERROR);
        assertEquals(sphere.getRadius(), sphere2.getRadius(), ABSOLUTE_ERROR);
        
        //Force IllegalArgumentException
        quadric = new Quadric();
        try{
            sphere2.setFromQuadric(quadric);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
}
