/**
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.HomogeneousPoint3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date June 30, 2012
 */
package com.irurueta.geometry;

import com.irurueta.statistics.UniformRandomizer;
import java.util.Arrays;
import java.util.Random;
import org.junit.*;
import static org.junit.Assert.*;

public class HomogeneousPoint3DTest {
    
    public static final int HOM_COORDS = 4;
    public static final int INHOM_COORDS = 3;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double RELATIVE_ERROR = 1.0;
    public static final double MIN_RANDOM_VALUE = 1.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public HomogeneousPoint3DTest() {
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
        HomogeneousPoint3D hPoint = new HomogeneousPoint3D();
        assertEquals(hPoint.getHomX(), 0.0, 0.0);
        assertEquals(hPoint.getHomY(), 0.0, 0.0);
        assertEquals(hPoint.getHomZ(), 0.0, 0.0);
        assertEquals(hPoint.getHomW(), 0.0, 1.0);
        assertFalse(hPoint.isNormalized());
        
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        hPoint = new HomogeneousPoint3D(array);
        
        double[] array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);
        assertFalse(hPoint.isNormalized());
        
        double a = randomizer.nextDouble();
        double b = randomizer.nextDouble();
        double c = randomizer.nextDouble();
        double d = randomizer.nextDouble();
        
        hPoint = new HomogeneousPoint3D(a, b, c, d);
        array = hPoint.asArray();
        
        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);
        assertEquals(d, array[3], 0.0);
        assertFalse(hPoint.isNormalized());
        
        Point3D point = Point3D.create(
                CoordinatesType.HOMOGENEOUS_COORDINATES);
        hPoint = new HomogeneousPoint3D(point);
        assertEquals(hPoint.getType(), 
                CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertFalse(hPoint.isNormalized());
        
        point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        hPoint = new HomogeneousPoint3D(point);
        assertEquals(hPoint.getType(),
                CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertFalse(hPoint.isNormalized());
    }
    
    @Test
    public void testGettersAndSetters(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        double x = randomizer.nextDouble();
        double y = randomizer.nextDouble();
        double z = randomizer.nextDouble();
        double w = randomizer.nextDouble();
        double homX = randomizer.nextDouble();
        double homY = randomizer.nextDouble();
        double homZ = randomizer.nextDouble();
        double homW = randomizer.nextDouble();
        double inhomX = randomizer.nextDouble();
        double inhomY = randomizer.nextDouble();
        double inhomZ = randomizer.nextDouble();
        
        HomogeneousPoint3D hPoint = new HomogeneousPoint3D();
        hPoint.setX(x);
        hPoint.setY(y);
        hPoint.setZ(z);
        hPoint.setW(w);
        assertEquals(hPoint.getX(), x, 0.0);
        assertEquals(hPoint.getY(), y, 0.0);
        assertEquals(hPoint.getZ(), z, 0.0);
        assertEquals(hPoint.getW(), w, 0.0);
        
        hPoint.setHomogeneousCoordinates(homX, homY, homZ, homW);
        assertEquals(hPoint.getHomX(), homX, 0.0);
        assertEquals(hPoint.getHomY(), homY, 0.0);
        assertEquals(hPoint.getHomZ(), homZ, 0.0);
        assertEquals(hPoint.getHomW(), homW, 0.0);
        assertEquals(hPoint.getX(), homX, 0.0);
        assertEquals(hPoint.getY(), homY, 0.0);
        assertEquals(hPoint.getW(), homW, 0.0);
        
        hPoint.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(hPoint.getInhomX(), inhomX, 0.0);
        assertEquals(hPoint.getInhomY(), inhomY, 0.0);
        assertEquals(hPoint.getInhomZ(), inhomZ, 0.0);
        assertEquals(hPoint.getX() / hPoint.getW(), inhomX, 0.0);
        assertEquals(hPoint.getY() / hPoint.getW(), inhomY, 0.0);
        assertEquals(hPoint.getZ() / hPoint.getW(), inhomZ, 0.0);
    }
    
    @Test
    public void testToInhomogeneous(){
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        HomogeneousPoint3D hPoint = new HomogeneousPoint3D(array);
        
        InhomogeneousPoint3D iPoint = hPoint.toInhomogeneous();
        
        assertEquals(iPoint.getX(), hPoint.getX() / hPoint.getW(), 
                ABSOLUTE_ERROR);
        assertEquals(iPoint.getY(), hPoint.getY() / hPoint.getW(), 
                ABSOLUTE_ERROR);
        assertEquals(iPoint.getZ(), hPoint.getZ() / hPoint.getW(), 
                ABSOLUTE_ERROR);
        
    }
    
    @Test
    public void testSetCoordinates(){
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        HomogeneousPoint3D hPoint = new HomogeneousPoint3D();
        hPoint.setCoordinates(array);
        double[] array2 = hPoint.asArray();
        
        assertArrayEquals(array, array2, 0.0);
        
        //Force IllegalArgumentException
        array = new double[HOM_COORDS + 1];
        hPoint = new HomogeneousPoint3D();
        try{
            hPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        array = new double[HOM_COORDS - 1];
        hPoint = new HomogeneousPoint3D();
        try{
            hPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        double x = randomizer.nextDouble();
        double y = randomizer.nextDouble();
        double z = randomizer.nextDouble();
        double w = randomizer.nextDouble();
        hPoint = new HomogeneousPoint3D();
        hPoint.setCoordinates(x, y, z, w);
        assertEquals(hPoint.getX(), x, 0.0);
        assertEquals(hPoint.getY(), y, 0.0);
        assertEquals(hPoint.getZ(), z, 0.0);
        assertEquals(hPoint.getW(), w, 0.0);
        
        double inhomX = randomizer.nextDouble();
        double inhomY = randomizer.nextDouble();
        double inhomZ = randomizer.nextDouble();
        hPoint = new HomogeneousPoint3D();
        hPoint.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(hPoint.getInhomX(), inhomX, 0.0);
        assertEquals(hPoint.getInhomY(), inhomY, 0.0);
        assertEquals(hPoint.getInhomZ(), inhomZ, 0.0);
        assertEquals(hPoint.getX() / hPoint.getW(), inhomX, 0.0);
        assertEquals(hPoint.getY() / hPoint.getW(), inhomY, 0.0);
        assertEquals(hPoint.getZ() / hPoint.getW(), inhomZ, 0.0);

        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        HomogeneousPoint3D hPoint2 = new HomogeneousPoint3D(array);        
        hPoint = new HomogeneousPoint3D();
        //pass another point to set coordinates
        hPoint.setCoordinates(hPoint2);
        array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        
        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D(array);
        hPoint = new HomogeneousPoint3D();
        //pass another point to set coordinates
        hPoint.setCoordinates(iPoint);
        array2 = hPoint.asArray();
        assertEquals(array[0], array2[0], 0.0);
        assertEquals(array[1], array2[1], 0.0);
        assertEquals(array[2], array2[2], 0.0);
        assertEquals(1.0, array2[3], 0.0);
    }
    
    @Test
    public void testAsArray(){
        double[] array = new double[HOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        HomogeneousPoint3D hPoint = new HomogeneousPoint3D(array);
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
        
        HomogeneousPoint3D hPoint1 = new HomogeneousPoint3D(array);
        HomogeneousPoint3D hPoint2 = new HomogeneousPoint3D(array);
        assertTrue(hPoint1.equals(hPoint2, 0.0));
        assertTrue(hPoint1.equals((Point3D)hPoint2, 0.0));
        
        array[0] = hPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        array[1] = hPoint1.getY();
        array[2] = hPoint1.getZ();
        array[3] = hPoint1.getW();
        
        hPoint2 = new HomogeneousPoint3D(array);
        assertFalse(hPoint1.equals(hPoint2, 0.0));
        assertFalse(hPoint1.equals((Point3D)hPoint2, 0.0));
        
        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[3] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        hPoint1 = new HomogeneousPoint3D(array);
        array[0] *= 2.0;
        hPoint2 = new HomogeneousPoint3D(array);
        assertTrue(hPoint1.equals(hPoint2, 2.0));
        assertTrue(hPoint1.equals((Point3D)hPoint2, 2.0));
        assertFalse(hPoint1.equals(hPoint2, 0.0));
        assertFalse(hPoint1.equals((Point3D)hPoint2, 0.0));
        
        //Testing equals from one inhomogeneous point
        double[] iArray = new double[INHOM_COORDS];
        iArray[0] = array[0] / array[3];
        iArray[1] = array[1] / array[3];
        iArray[2] = array[2] / array[3];
        
        HomogeneousPoint3D hPoint = new HomogeneousPoint3D(array);
        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D(iArray);        
        assertTrue(hPoint.equals(iPoint, ABSOLUTE_ERROR));
        assertTrue(hPoint.equals((Point3D)iPoint, ABSOLUTE_ERROR));
        
        iArray[0] = hPoint.getInhomX() + 1.0;
        iArray[1] = hPoint.getInhomY() + 1.0;
        iArray[2] = hPoint.getInhomZ() + 1.0;
        iPoint = new InhomogeneousPoint3D(iArray);
        assertFalse(hPoint.equals(iPoint, 0.0));
        assertFalse(hPoint.equals((Point3D)iPoint, 0.0));
        
        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[3] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint = new HomogeneousPoint3D(array);
        
        iArray[0] = hPoint.getInhomX() + 1.0;
        iArray[1] = hPoint.getInhomY() + 1.0;
        iArray[2] = hPoint.getInhomZ() + 1.0;
        iPoint = new InhomogeneousPoint3D(iArray);
        assertTrue(hPoint.equals(iPoint, 1.1));
        assertTrue(hPoint.equals((Point3D)iPoint, 1.1));
        assertFalse(hPoint.equals(iPoint, 0.5));
        assertFalse(hPoint.equals((Point3D)iPoint, 0.5));
        
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
            hPoint.equals((Point3D)iPoint, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}        
    }    
    
    @Test
    public void testIsAtInfinity(){
        HomogeneousPoint3D hPoint = new HomogeneousPoint3D();
        //sets point at infinity
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
    
    public void testNormalize(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        HomogeneousPoint3D point = new HomogeneousPoint3D(homX, homY, homZ, 
                homW);
        assertEquals(point.getHomX(), homX, 0.0);
        assertEquals(point.getHomY(), homY, 0.0);
        assertEquals(point.getHomZ(), homZ, 0.0);
        assertEquals(point.getHomW(), homW, 0.0);
        assertFalse(point.isNormalized());
        
        //compute norm
        double norm = Math.sqrt(homX * homX + homY * homY + homZ * homZ +
                homW * homW);
        
        //normalize
        point.normalize();
        assertTrue(point.isNormalized());
        
        //check correctness after normalization
        assertEquals(point.getHomX(), homX / norm,  ABSOLUTE_ERROR);
        assertEquals(point.getHomY(), homY / norm,  ABSOLUTE_ERROR);
        assertEquals(point.getHomZ(), homZ / norm,  ABSOLUTE_ERROR);
        assertEquals(point.getHomW(), homW / norm,  ABSOLUTE_ERROR);
        
        point.setHomogeneousCoordinates(homX, homY, homZ, homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());
        
        point.setCoordinates(homX, homY, homZ, homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        point.setInhomogeneousCoordinates(homX / homW, homY / homW, 
                homZ / homW);
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
        
        HomogeneousPoint3D point2 = new HomogeneousPoint3D();
        point.setCoordinates(point2);
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

        point.setZ(homZ);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());

        point.setW(homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());        
    }    
}
