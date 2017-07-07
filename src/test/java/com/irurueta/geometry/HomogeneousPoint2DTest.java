/**
 * @file
 * Thi file contains Unit Tests for
 * com.irurueta.geometry.HomogeneousPoint2D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 27, 2012
 */
package com.irurueta.geometry;

import com.irurueta.statistics.UniformRandomizer;
import java.util.Arrays;
import java.util.Random;
import static org.junit.Assert.*;
import org.junit.*;

public class HomogeneousPoint2DTest {
    
    public static final int HOM_COORDS = 3;
    public static final int INHOM_COORDS = 2;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double RELATIVE_ERROR = 1.0;
    public static final double MIN_RANDOM_VALUE = 1.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public HomogeneousPoint2DTest() {
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
        HomogeneousPoint2D hPoint = new HomogeneousPoint2D();
        assertEquals(hPoint.getHomX(), 0.0, 0.0);
        assertEquals(hPoint.getHomY(), 0.0, 0.0);
        assertEquals(hPoint.getHomW(), 1.0, 0.0);
        assertFalse(hPoint.isNormalized());
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        hPoint = new HomogeneousPoint2D(array);
        
        double[] array2 = hPoint.asArray();
        
        assertArrayEquals(array, array2, 0.0);
        assertFalse(hPoint.isNormalized());
        
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        hPoint = new HomogeneousPoint2D(a, b, c);
        array = hPoint.asArray();
        
        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);
        assertFalse(hPoint.isNormalized());
        
        Point2D point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        point.setHomogeneousCoordinates(a, b, c);
        hPoint = new HomogeneousPoint2D(point);
        assertEquals(hPoint.getHomX(), a, 0.0);
        assertEquals(hPoint.getHomY(), b, 0.0);
        assertEquals(hPoint.getHomW(), c, 0.0);
        assertEquals(hPoint.getType(), CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertFalse(hPoint.isNormalized());
    }
    
    @Test
    public void testGettersAndSetters(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        
        HomogeneousPoint2D hPoint = new HomogeneousPoint2D();
        
        hPoint.setX(x);
        hPoint.setY(y);
        hPoint.setW(w);
        assertEquals(hPoint.getX(), x, 0.0);
        assertEquals(hPoint.getY(), y, 0.0);
        assertEquals(hPoint.getW(), w, 0.0);
        
        hPoint.setHomogeneousCoordinates(homX, homY, homW);
        assertEquals(hPoint.getHomX(), homX, 0.0);
        assertEquals(hPoint.getHomY(), homY, 0.0);
        assertEquals(hPoint.getHomW(), homW, 0.0);
        assertEquals(hPoint.getX(), homX, 0.0);
        assertEquals(hPoint.getY(), homY, 0.0);
        assertEquals(hPoint.getW(), homW, 0.0);
        
        hPoint.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(hPoint.getInhomX(), inhomX, 0.0);
        assertEquals(hPoint.getInhomY(), inhomY, 0.0);
        assertEquals(hPoint.getX() / hPoint.getW(), inhomX, 0.0);
        assertEquals(hPoint.getY() / hPoint.getW(), inhomY, 0.0);
    }
    
    @Test
    public void testToInhomogeneous(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        HomogeneousPoint2D hPoint = new HomogeneousPoint2D(array);
        
        InhomogeneousPoint2D iPoint = hPoint.toInhomogeneous();
        
        assertEquals(iPoint.getX(), hPoint.getX() / hPoint.getW(), 
                ABSOLUTE_ERROR);
        assertEquals(iPoint.getY(), hPoint.getY() / hPoint.getW(),
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testSetCoordinates(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double[] array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        HomogeneousPoint2D hPoint = new HomogeneousPoint2D();
        hPoint.setCoordinates(array);
        double[] array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);
        
        //Force IllegalArgumentException
        array = new double[HOM_COORDS + 1];
        hPoint = new HomogeneousPoint2D();
        try{
            hPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        array = new double[HOM_COORDS - 1];
        hPoint = new HomogeneousPoint2D();
        try{
            hPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}

        
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint = new HomogeneousPoint2D();
        hPoint.setCoordinates(x, y, w);
        assertEquals(hPoint.getX(), x, 0.0);
        assertEquals(hPoint.getY(), y, 0.0);
        assertEquals(hPoint.getW(), w, 0.0);
        
        double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        hPoint = new HomogeneousPoint2D();
        hPoint.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(hPoint.getInhomX(), inhomX, 0.0);
        assertEquals(hPoint.getInhomY(), inhomY, 0.0);
        assertEquals(hPoint.getX() / hPoint.getW(), inhomX, 0.0);
        assertEquals(hPoint.getY() / hPoint.getW(), inhomY, 0.0);
        
        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        HomogeneousPoint2D hPoint2 = new HomogeneousPoint2D(array);
        hPoint = new HomogeneousPoint2D();
        //pass another point to set coordinates
        hPoint.setCoordinates(hPoint2);
        array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);
        
        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(array);
        hPoint = new HomogeneousPoint2D();
        hPoint.setCoordinates(iPoint);
        array2 = hPoint.asArray();
        assertEquals(array.length, INHOM_COORDS);
        assertEquals(array2.length, HOM_COORDS);
        assertEquals(array[0], array2[0], 0.0);
        assertEquals(array[1], array2[1], 0.0);
        assertEquals(1.0, array2[2], 0.0);
    }
    
    @Test
    public void testArray(){
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        HomogeneousPoint2D hPoint = new HomogeneousPoint2D(array);
        double[] array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);
        
        array2 = new double[HOM_COORDS];
        hPoint.asArray(array2);
        assertArrayEquals(array, array2, 0.0);
    }
    
    @Test
    public void testEquals(){
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        HomogeneousPoint2D hPoint1 = new HomogeneousPoint2D(array);
        HomogeneousPoint2D hPoint2 = new HomogeneousPoint2D(array);
        assertTrue(hPoint1.equals(hPoint2, 0.0));
        assertTrue(hPoint1.equals((Point2D)hPoint2, 0.0));
        
        array[0] = hPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        array[1] = hPoint1.getY();
        array[2] = hPoint1.getW();
        hPoint2 = new HomogeneousPoint2D(array);
        assertFalse(hPoint1.equals(hPoint2, 0.0));
        assertFalse(hPoint1.equals((Point2D)hPoint2, 0.0));
        
        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint1 = new HomogeneousPoint2D(array);
        array[0] *= 2.0;
        hPoint2 = new HomogeneousPoint2D(array);
        assertTrue(hPoint1.equals(hPoint2, 2.0));
        assertTrue(hPoint1.equals((Point2D)hPoint2, 2.0));
        assertFalse(hPoint1.equals(hPoint2, 0.0));
        
        //Testing equals from one inhomogenous point
        double[] iArray = new double[INHOM_COORDS];
        iArray[0] = array[0] / array[2];
        iArray[1] = array[1] / array[2];
        
        HomogeneousPoint2D hPoint = new HomogeneousPoint2D(array);
        InhomogeneousPoint2D iPoint = new InhomogeneousPoint2D(iArray);
        assertTrue(hPoint.equals(iPoint, ABSOLUTE_ERROR));
        assertTrue(hPoint.equals((Point2D)iPoint, ABSOLUTE_ERROR));
        
        iArray[0] = hPoint.getInhomX() + 1.0;
        iArray[1] = hPoint.getInhomY() + 1.0;
        iPoint = new InhomogeneousPoint2D(iArray);
        assertFalse(hPoint.equals(iPoint, 0.0));
        assertFalse(hPoint.equals((Point2D)iPoint, 0.0));
        
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint = new HomogeneousPoint2D(array);
        
        iArray[0] = hPoint.getInhomX() + 1.0;
        iArray[1] = hPoint.getInhomY() + 1.0;
        iPoint = new InhomogeneousPoint2D(iArray);
        assertTrue(hPoint.equals(iPoint, 1.1));
        assertTrue(hPoint.equals((Point2D)iPoint, 1.1));
        assertFalse(hPoint.equals(iPoint, 0.5));
        assertFalse(hPoint.equals((Point2D)iPoint, 0.5));
        
        //Force IllegalArgumentException
        try{
            hPoint.equals(hPoint, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            hPoint.equals(iPoint, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            hPoint.equals((Point2D)iPoint, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testIsAtInfinity(){
        HomogeneousPoint2D hPoint = new HomogeneousPoint2D();
        //Sets point at infinity
        hPoint.setW(0.0);
        assertTrue(hPoint.isAtInfinity());
        //Forces point not being at infinity
        hPoint.setW(1.0);
        assertFalse(hPoint.isAtInfinity());
        
        //Testing with threshold
        hPoint.setW(4.0);
        assertTrue(hPoint.isAtInfinity(4.5));
        hPoint.setW(5.0);
        assertFalse(hPoint.isAtInfinity(4.5));
        
        //Force IllegalArgumentException
        try{
            hPoint.isAtInfinity(-ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
    }
    
    @Test
    public void testNormalize(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        HomogeneousPoint2D point = new HomogeneousPoint2D(homX, homY, homW);
        assertEquals(point.getHomX(), homX, 0.0);
        assertEquals(point.getHomY(), homY, 0.0);
        assertEquals(point.getHomW(), homW, 0.0);
        assertFalse(point.isNormalized());
        
        //compute norm
        double norm = Math.sqrt(homX * homX + homY * homY + homW * homW);
        
        //normalize
        point.normalize();
        assertTrue(point.isNormalized());
        
        //check correctness after normalization
        assertEquals(point.getHomX(), homX / norm,  ABSOLUTE_ERROR);
        assertEquals(point.getHomY(), homY / norm,  ABSOLUTE_ERROR);
        assertEquals(point.getHomW(), homW / norm,  ABSOLUTE_ERROR);
        
        point.setHomogeneousCoordinates(homX, homY, homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());
        
        point.setCoordinates(homX, homY, homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());
        
        point.setInhomogeneousCoordinates(homX / homW, homY / homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());
        
        double[] array = new double[HOM_COORDS];
        Arrays.fill(array, 0.0); //initialize to zeros
        point.setCoordinates(array);
        assertFalse(point.isNormalized());
        point.normalize(); //normalization is not done because of numerical 
                            //instability
        assertFalse(point.isNormalized());
        
        //fill array with random values
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setCoordinates(array);
        assertFalse(point.isNormalized());
        point.normalize(); //now normalization is done
        assertTrue(point.isNormalized());
        
        HomogeneousPoint2D point2 = new HomogeneousPoint2D();
        point.setCoordinates(point);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());
        
        point.setX(homX);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());
        
        point.setY(homY);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());
        
        point.setW(homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());
    }
}
