/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.InhomogeneousPoint2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 27, 2012
 */
package com.irurueta.geometry;

import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import static org.junit.Assert.*;
import org.junit.*;

public class InhomogeneousPoint2DTest {

    public static final int HOM_COORDS = 3;
    public static final int INHOM_COORDS = 2;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double RELATIVE_ERROR = 1.0;
    public static final double MIN_RANDOM_VALUE = 1.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final int TIMES = 100;
    
    public InhomogeneousPoint2DTest() {
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
    public void testConstructor(){
        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D();
        assertEquals(iPoint.getInhomX(), 0.0, 0.0);
        assertEquals(iPoint.getInhomY(), 0.0, 0.0);
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        iPoint = new InhomogeneousPoint2D(array);
        
        double[] array2 = iPoint.asArray();
        
        assertArrayEquals(array, array2, 0.0);
        
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        iPoint = new InhomogeneousPoint2D(a, b);
        array = iPoint.asArray();
        
        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        
        Point2D point = Point2D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setInhomogeneousCoordinates(a, b);
        iPoint = new InhomogeneousPoint2D(point);
        assertEquals(iPoint.getInhomX(), a, 0.0);
        assertEquals(iPoint.getInhomY(), b, 0.0);
        assertEquals(iPoint.getType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
    }

    @Test
    public void testGettersAndSetters(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D();
        
        iPoint.setX(x);
        iPoint.setY(y);
        assertEquals(iPoint.getX(), x, 0.0);
        assertEquals(iPoint.getY(), y, 0.0);
        
        iPoint.setHomogeneousCoordinates(homX, homY, homW);
        double constantX = iPoint.getHomX() / homX;
        double constantY = iPoint.getHomY() / homY;
        double constantW = iPoint.getHomW() / homW;
        assertEquals(constantX, constantY, ABSOLUTE_ERROR);
        assertEquals(constantY, constantW, ABSOLUTE_ERROR);
        assertEquals(constantW, constantX, ABSOLUTE_ERROR);
                
        iPoint.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(iPoint.getInhomX(), inhomX, 0.0);
        assertEquals(iPoint.getInhomY(), inhomY, 0.0);
        assertEquals(iPoint.getX(), inhomX, 0.0);
        assertEquals(iPoint.getY(), inhomY, 0.0);        
    }
    
    @Test
    public void testToHomogeneous(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(array);
        
        HomogeneousPoint2D hPoint = iPoint.toHomogeneous();
        
        //check that inhomogeneous coordinates are almost equal
        assertEquals(hPoint.getInhomX(), iPoint.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getInhomY(), iPoint.getInhomY(), ABSOLUTE_ERROR);
        
        //check that homogeneous coordinates are up to scale
        double scaleX = hPoint.getHomX() / iPoint.getHomX();
        double scaleY = hPoint.getHomY() / iPoint.getHomY();
        double scaleW = hPoint.getHomW() / iPoint.getHomW();
        
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testSetCoordinates(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D();
        iPoint.setCoordinates(array);
        double[] array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);
        
        //Force IllegalArgumentException
        array = new double[INHOM_COORDS + 1];
        iPoint = new InhomogeneousPoint2D();
        try{
            iPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        array = new double[INHOM_COORDS - 1];
        iPoint = new InhomogeneousPoint2D();
        try{
            iPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}

        
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint = new InhomogeneousPoint2D();
        iPoint.setCoordinates(x, y);
        assertEquals(iPoint.getX(), x, 0.0);
        assertEquals(iPoint.getY(), y, 0.0);
        
        double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        iPoint = new InhomogeneousPoint2D();
        iPoint.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(iPoint.getInhomX(), inhomX, 0.0);
        assertEquals(iPoint.getInhomY(), inhomY, 0.0);
        assertEquals(iPoint.getX(), inhomX, 0.0);
        assertEquals(iPoint.getY(), inhomY, 0.0);
        
        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        InhomogeneousPoint2D iPoint2 = new InhomogeneousPoint2D(array);
        iPoint = new InhomogeneousPoint2D();
        //pass another point to set coordinates
        iPoint.setCoordinates(iPoint2);
        array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);
        
        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        HomogeneousPoint2D hPoint = new HomogeneousPoint2D(array);
        iPoint = new InhomogeneousPoint2D();
        iPoint.setCoordinates(hPoint);
        array2 = iPoint.asArray();
        assertEquals(array.length, HOM_COORDS);
        assertEquals(array2.length, INHOM_COORDS);
        assertEquals(array[0] / array[2], array2[0], 0.0);
        assertEquals(array[1] / array[2], array2[1], 0.0);
    }
    
    @Test
    public void testArray(){
        double[] array = new double[INHOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(array);
        double[] array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);
        
        array2 = new double[INHOM_COORDS];
        iPoint.asArray(array2);
        assertArrayEquals(array, array2, 0.0);        
    }
    
    @Test
    public void testEquals(){
        for(int i = 0; i < TIMES; i++){
            double[] array = new double[INHOM_COORDS];
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            InhomogeneousPoint2D iPoint1 = new InhomogeneousPoint2D(array);
            InhomogeneousPoint2D iPoint2 = new InhomogeneousPoint2D(array);
            assertTrue(iPoint1.equals(iPoint2, 0.0));
            assertTrue(iPoint1.equals((Point2D)iPoint2, 0.0));
        
            array[0] = iPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            array[1] = iPoint1.getY();
            iPoint2 = new InhomogeneousPoint2D(array);
            assertFalse(iPoint1.equals(iPoint2, 0.0));
            assertFalse(iPoint1.equals((Point2D)iPoint2, 0.0));
        
            array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            iPoint1 = new InhomogeneousPoint2D(array);
            array[0] += 1.0;
            iPoint2 = new InhomogeneousPoint2D(array);
            assertTrue(iPoint1.equals(iPoint2, 1.0 + ABSOLUTE_ERROR));
            assertTrue(iPoint1.equals((Point2D)iPoint2, 1.0 + ABSOLUTE_ERROR));
            assertFalse(iPoint1.equals(iPoint2, 0.5));
            assertFalse(iPoint1.equals(iPoint2, 0.5));
        
            //Testing equals from one homogenous point
            double[] hArray = new double[HOM_COORDS];
            randomizer.fill(hArray, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            array[0] = hArray[0] / hArray[2];
            array[1] = hArray[1] / hArray[2];
        
            InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(array);
            HomogeneousPoint2D hPoint = new HomogeneousPoint2D(hArray);
            assertTrue(iPoint.equals(hPoint, ABSOLUTE_ERROR));
            assertTrue(iPoint.equals((Point2D)hPoint, ABSOLUTE_ERROR));
        
            hArray[0] = iPoint.getHomX() + 1.0;
            hArray[1] = iPoint.getHomY() + 1.0;
            hArray[2] = iPoint.getHomW() + 1.0;
            hPoint = new HomogeneousPoint2D(hArray);
            assertFalse(iPoint.equals(hPoint, 0.0));
            assertFalse(iPoint.equals((Point2D)hPoint, 0.0));
        
            randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            iPoint = new InhomogeneousPoint2D(array);
        
            hArray[0] = iPoint.getHomX() + iPoint.getHomW();
            hArray[1] = iPoint.getHomY() + iPoint.getHomW();
            hArray[2] = iPoint.getHomW();
            hPoint = new HomogeneousPoint2D(hArray);
            assertTrue(iPoint.equals(hPoint, 1.1));
            assertTrue(iPoint.equals((Point2D)hPoint, 1.1));
            assertFalse(iPoint.equals(hPoint, 0.5));
            assertFalse(iPoint.equals((Point2D)hPoint, 0.5));
            
            //Force IllegalArgumentException
            try{
                iPoint.equals(hPoint, -ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException e){}
            try{
                iPoint.equals(iPoint, -ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException e){}
            try{
                iPoint.equals((Point2D)iPoint, -ABSOLUTE_ERROR);
                fail("IllegalArgumentException expected but not thrown");
            }catch(IllegalArgumentException e){}            
        }
    }
    
    @Test
    public void testIsAtInfinity(){
        double[] array = new double[INHOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        
        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(array);
        iPoint.setX(Double.POSITIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());
        
        iPoint = new InhomogeneousPoint2D(array);
        iPoint.setY(Double.POSITIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        
        iPoint = new InhomogeneousPoint2D(array);
        assertFalse(iPoint.isAtInfinity());

        
        iPoint = new InhomogeneousPoint2D(array);
        iPoint.setX(Double.NEGATIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());
        
        iPoint = new InhomogeneousPoint2D(array);
        iPoint.setY(Double.NEGATIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());
        

        iPoint = new InhomogeneousPoint2D(array);
        iPoint.setX(Double.NaN);
        assertTrue(iPoint.isAtInfinity());
        
        iPoint = new InhomogeneousPoint2D(array);
        iPoint.setY(Double.NaN);
        assertTrue(iPoint.isAtInfinity());
    }   
}
