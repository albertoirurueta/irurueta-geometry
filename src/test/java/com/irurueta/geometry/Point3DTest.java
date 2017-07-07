/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.Point3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 1, 2012
 */
package com.irurueta.geometry;

import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.*;
import static org.junit.Assert.*;

public class Point3DTest {
    
    public static final int HOM_COORDS = 4;
    public static final int INHOM_COORDS = 3;
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public static final int MIN_POINTS = 100;
    public static final int MAX_POINTS = 500;
    
    public Point3DTest() {
    }

    @BeforeClass
    public static void setUpClass() throws Exception {
    }

    @AfterClass
    public static void tearDownClass() throws Exception {
    }
    
    @Before
    public void setUp() {
    }
    
    @After
    public void tearDown() {
    }
    
    @Test
    public void testCreate(){
        Point3D point;
        
        point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(point.getType(), CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(point.getInhomX(), 0.0, 0.0);
        assertEquals(point.getInhomY(), 0.0, 0.0);
        assertEquals(point.getInhomZ(), 0.0, 0.0);
        assertEquals(point.getHomX(), 0.0, 0.0);
        assertEquals(point.getHomY(), 0.0, 0.0);
        assertEquals(point.getHomZ(), 0.0, 0.0);
        assertEquals(point.getHomW(), 1.0, 0.0);
        
        point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(point.getType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(point.getInhomX(), 0.0, 0.0);
        assertEquals(point.getInhomY(), 0.0, 0.0);
        assertEquals(point.getInhomZ(), 0.0, 0.0);
        assertEquals(point.getHomX(), 0.0, 0.0);
        assertEquals(point.getHomY(), 0.0, 0.0);
        assertEquals(point.getHomZ(), 0.0, 0.0);
        assertEquals(point.getHomW(), 1.0, 0.0);
        
        double[] array = new double[HOM_COORDS];
        double[] iArray = new double[INHOM_COORDS];
        
        point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array);
        assertEquals(point.getType(), CoordinatesType.HOMOGENEOUS_COORDINATES);
        
        point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, 
                iArray);
        assertEquals(point.getType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        
        //Force IllegalArgumentException
        try{
            Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, array);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        try{
            Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, iArray);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        
        point = Point3D.create(array);
        assertEquals(point.getType(), Point3D.DEFAULT_COORDINATES_TYPE);
        
        //Force IllegalArgumentException
        try{
            Point3D.create(iArray);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        point = Point3D.create();
        assertEquals(point.getType(), Point3D.DEFAULT_COORDINATES_TYPE);        
    }
    
    @Test
    public void testDistanceTo(){
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D point1 = Point3D.create();
        Point3D point2 = Point3D.create();
        
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        double diffX = point1.getInhomX() - point2.getInhomX();
        double diffY = point1.getInhomY() - point2.getInhomY();
        double diffZ = point1.getInhomZ() - point2.getInhomZ();
        double distance = Math.sqrt(diffX * diffX + diffY * diffY + 
                diffZ * diffZ);
        
        //check distance
        assertEquals(point1.distanceTo(point2), distance, ABSOLUTE_ERROR);
        assertEquals(point2.distanceTo(point1), distance, ABSOLUTE_ERROR);
        
        //check distance to themselves
        assertEquals(point1.distanceTo(point1), 0.0, ABSOLUTE_ERROR);
        assertEquals(point2.distanceTo(point2), 0.0, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testDotProduct(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D point1 = Point3D.create();
        Point3D point2 = Point3D.create();
        Point3D point3 = Point3D.create();
        
        point1.setHomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setHomogeneousCoordinates(-point1.getHomX(), -point1.getHomY(), 
                -point1.getHomZ(), -point1.getHomW());
        point3.setHomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        //dot product for same point
        assertEquals(point1.dotProduct(point1), 1.0, ABSOLUTE_ERROR);
        //dot product for oposite signs
        assertEquals(point1.dotProduct(point2), -1.0, ABSOLUTE_ERROR);
        //dot product for random points
        point1.normalize();
        point3.normalize();
        assertEquals(point1.dotProduct(point3), 
                point1.getHomX() * point3.getHomX() + 
                point1.getHomY() * point3.getHomY() + 
                point1.getHomZ() * point3.getHomZ() + 
                point1.getHomW() * point3.getHomW(), ABSOLUTE_ERROR);
    }    

    @Test
    public void testIsBetween(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double valueBetween = randomizer.nextDouble(0.2, 0.8);
        double valueOutside = 1.0 + valueBetween;
        
        Point3D point1 = Point3D.create();
        Point3D point2 = Point3D.create();
        
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        double diffX = point2.getInhomX() - point1.getInhomX();
        double diffY = point2.getInhomY() - point1.getInhomY();
        double diffZ = point2.getInhomZ() - point1.getInhomZ();
        double dist = Math.sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);
        
        Point3D betweenPoint = Point3D.create();
        betweenPoint.setInhomogeneousCoordinates(
                point1.getInhomX() + valueBetween * diffX, 
                point1.getInhomY() + valueBetween * diffY,
                point1.getInhomZ() + valueBetween * diffZ);

        Point3D outsidePoint = Point3D.create();
        outsidePoint.setInhomogeneousCoordinates(
                point1.getInhomX() + valueOutside * diffX, 
                point1.getInhomY() + valueOutside * diffY,
                point1.getInhomZ() + valueOutside * diffZ);
        
        assertTrue(betweenPoint.isBetween(point1, point2));
        assertFalse(outsidePoint.isBetween(point1, point2));
        
        //the same is true for a small threshold
        assertTrue(betweenPoint.isBetween(point1, point2, ABSOLUTE_ERROR));
        assertFalse(outsidePoint.isBetween(point1, point2, ABSOLUTE_ERROR));
        
        //with a large enough threshold, even outside point is considered to lie
        //in between
        assertTrue(betweenPoint.isBetween(point1, point2, 2.0 * dist));
        assertTrue(outsidePoint.isBetween(point1, point2, 2.0 * dist));
        
        //Force IllegalArgumentException
        try{
            betweenPoint.isBetween(point1, point2, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }  
    
    @Test
    public void testCentroid() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int numPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
        
        List<Point3D> points = new ArrayList<Point3D>();
        double x, y, z, meanX = 0.0, meanY = 0.0, meanZ = 0.0;
        for(int i = 0; i < numPoints; i++) {
            x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            
            points.add(new InhomogeneousPoint3D(x, y, z));
            
            meanX += x;
            meanY += y;
            meanZ += z;
        }
        
        meanX /= numPoints;
        meanY /= numPoints;
        meanZ /= numPoints;
        
        Point3D mean1 = Point3D.create();
        Point3D.centroid(points, mean1);
        Point3D mean2 = Point3D.centroid(points);
        
        assertEquals(mean1, mean2);
        assertEquals(mean1.getInhomX(), meanX, ABSOLUTE_ERROR);
        assertEquals(mean1.getInhomY(), meanY, ABSOLUTE_ERROR);
        assertEquals(mean1.getInhomZ(), meanZ, ABSOLUTE_ERROR);
    }
}
