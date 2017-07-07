/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.InhomogeneousPoint3D
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date July 1, 2012
 */
package com.irurueta.geometry;

import com.irurueta.statistics.UniformRandomizer;
import java.util.Random;
import static org.junit.Assert.*;
import org.junit.*;

public class InhomogeneousPoint3DTest {
    
    public static final int INHOM_COORDS = 3;
    public static final int HOM_COORDS = 4;
    
    public static final double RELATIVE_ERROR = 1.0;
    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double MIN_RANDOM_VALUE = 1.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public InhomogeneousPoint3DTest() {
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
        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D();
        assertEquals(iPoint.getInhomX(), 0.0, 0.0);
        assertEquals(iPoint.getInhomY(), 0.0, 0.0);
        assertEquals(iPoint.getInhomZ(), 0.0, 0.0);
        
        double[] array = new double[INHOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint = new InhomogeneousPoint3D(array);
        double[] array2 = iPoint.asArray();
        assertArrayEquals(array, array2, 0.0);
        
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        iPoint = new InhomogeneousPoint3D(a, b, c);
        array = iPoint.asArray();
        
        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);
        
        Point3D point = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        iPoint = new InhomogeneousPoint3D(point);
        assertEquals(iPoint.getType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        
        point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        iPoint = new InhomogeneousPoint3D(point);
        assertEquals(iPoint.getType(),
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
    }
    
    @Test
    public void testGettersAndSetters(){
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double inhomZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D();
        iPoint.setX(x);
        iPoint.setY(y);
        iPoint.setZ(z);
        assertEquals(iPoint.getX(), x, 0.0);
        assertEquals(iPoint.getY(), y, 0.0);
        assertEquals(iPoint.getZ(), z, 0.0);
        
        iPoint.setHomogeneousCoordinates(homX, homY, homZ, homW);
        double constantX = iPoint.getHomX() / homX;
        double constantY = iPoint.getHomY() / homY;
        double constantZ = iPoint.getHomZ() / homZ;
        double constantW = iPoint.getHomW() / homW;
        assertEquals(constantX, constantY, ABSOLUTE_ERROR);
        assertEquals(constantY, constantZ, ABSOLUTE_ERROR);
        assertEquals(constantZ, constantW, ABSOLUTE_ERROR);
        assertEquals(constantW, constantX, ABSOLUTE_ERROR);
        
        iPoint.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(iPoint.getInhomX(), inhomX, 0.0);
        assertEquals(iPoint.getInhomY(), inhomY, 0.0);
        assertEquals(iPoint.getInhomZ(), inhomZ, 0.0);
        assertEquals(iPoint.getX(), inhomX, 0.0);
        assertEquals(iPoint.getY(), inhomY, 0.0);
        assertEquals(iPoint.getZ(), inhomZ, 0.0);
    }
    
    @Test
    public void testToHomogeneous(){
        double[] array = new double[INHOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D(array);
        HomogeneousPoint3D hPoint = iPoint.toHomogeneous();
        
        //check that inhomogeneous coordinates are almost equal
        assertEquals(hPoint.getInhomX(), iPoint.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getInhomY(), iPoint.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getInhomZ(), iPoint.getInhomZ(), ABSOLUTE_ERROR);
        
        //check that homogeneous coordinates are up to scale
        double scaleX = hPoint.getHomX() / iPoint.getHomX();
        double scaleY = hPoint.getHomY() / iPoint.getHomY();
        double scaleZ = hPoint.getHomZ() / iPoint.getHomZ();
        double scaleW = hPoint.getHomW() / iPoint.getHomW();
        
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testSetCoordinates(){
        double[] array = new double[INHOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D();
        iPoint.setCoordinates(array);
        double[] array2 = iPoint.asArray();
        
        assertArrayEquals(array, array2, 0.0);
        
        //Force IllegalArgumentException
        array = new double[INHOM_COORDS + 1];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        try{
            iPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        array = new double[INHOM_COORDS - 1];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        try{
            iPoint.setCoordinates(array);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        
        double x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double z = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint = new InhomogeneousPoint3D();
        iPoint.setCoordinates(x, y, z);
        assertEquals(iPoint.getX(), x, 0.0);
        assertEquals(iPoint.getY(), y, 0.0);
        assertEquals(iPoint.getZ(), z, 0.0);
        
        double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        double inhomZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        iPoint = new InhomogeneousPoint3D();
        iPoint.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);
        assertEquals(iPoint.getInhomX(), inhomX, 0.0);
        assertEquals(iPoint.getInhomY(), inhomY, 0.0);
        assertEquals(iPoint.getInhomZ(), inhomZ, 0.0);
        assertEquals(iPoint.getX(), inhomX, 0.0);
        assertEquals(iPoint.getY(), inhomY, 0.0);
        assertEquals(iPoint.getZ(), inhomZ, 0.0);
        
        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        InhomogeneousPoint3D iPoint2 = new InhomogeneousPoint3D(array);
        iPoint = new InhomogeneousPoint3D();
        iPoint.setCoordinates(iPoint2);
        
        array2 = iPoint.asArray();        
        assertArrayEquals(array, array2, 0.0);

        
        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);        
        HomogeneousPoint3D hPoint = new HomogeneousPoint3D(array);
        iPoint = new InhomogeneousPoint3D();
        iPoint.setCoordinates(hPoint);
        
        array2 = iPoint.asArray();
        assertEquals(array[0] / array[3], array2[0], 0.0);
        assertEquals(array[1] / array[3], array2[1], 0.0);
        assertEquals(array[2] / array[3], array2[2], 0.0);
    }
    
    @Test
    public void testAsArray(){
        double[] array = new double[INHOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D(array);
        double[] array2 = iPoint.asArray();        
        assertArrayEquals(array, array2, 0.0);
        
        array2 = new double[INHOM_COORDS];
        iPoint.asArray(array2);
        assertArrayEquals(array, array2, 0.0);
    }
    
    @Test
    public void testEquals(){
        double[] array = new double[INHOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        InhomogeneousPoint3D iPoint1 = new InhomogeneousPoint3D(array);
        InhomogeneousPoint3D iPoint2 = new InhomogeneousPoint3D(array);
        
        assertTrue(iPoint1.equals(iPoint2, 0.0));
        assertTrue(iPoint1.equals((Point3D)iPoint2, 0.0));
        
        array[0] = iPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE, 
                MAX_RANDOM_VALUE);
        array[1] = iPoint1.getY();
        array[2] = iPoint1.getZ();
        iPoint2 = new InhomogeneousPoint3D(array);
        assertFalse(iPoint1.equals(iPoint2, 0.0));
        assertFalse(iPoint1.equals((Point3D)iPoint2, 0.0));
        
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint1 = new InhomogeneousPoint3D(array);
        array[0] += 1.0;
        iPoint2 = new InhomogeneousPoint3D(array);
        assertTrue(iPoint1.equals(iPoint2, 2.0));
        assertTrue(iPoint1.equals((Point3D)iPoint2, 2.0));
        assertFalse(iPoint1.equals(iPoint2, 0.5));
        assertFalse(iPoint1.equals((Point3D)iPoint2, 0.5));
        
        //Testing equals from one homogeneous point
        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        double[] iArray = new double[INHOM_COORDS];
        iArray[0] = array[0] / array[3];
        iArray[1] = array[1] / array[3];
        iArray[2] = array[2] / array[3];
        
        iPoint1 = new InhomogeneousPoint3D(iArray);
        
        HomogeneousPoint3D hPoint = new HomogeneousPoint3D(array);
        
        assertTrue(iPoint1.equals(hPoint, ABSOLUTE_ERROR));
        assertTrue(iPoint1.equals((Point3D)hPoint, ABSOLUTE_ERROR));
        
        array[0] = iPoint1.getHomX() + 1.0;
        array[1] = iPoint1.getHomY() + 1.0;
        array[2] = iPoint1.getHomZ() + 1.0;
        array[3] = iPoint1.getHomW() + 1.0;
        hPoint = new HomogeneousPoint3D(array);
        assertFalse(iPoint1.equals(hPoint, 0.0));
        assertFalse(iPoint1.equals((Point3D)hPoint, 0.0));
        
        iArray[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iArray[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iArray[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        iPoint1 = new InhomogeneousPoint3D(iArray);
        
        array[0] = iPoint1.getHomX() + iPoint1.getHomW();
        array[1] = iPoint1.getHomY() + iPoint1.getHomW();
        array[2] = iPoint1.getHomZ() + iPoint1.getHomW();
        array[3] = iPoint1.getHomW();
        hPoint = new HomogeneousPoint3D(array);
        assertTrue(iPoint1.equals(hPoint, 1.0 + ABSOLUTE_ERROR));
        assertTrue(iPoint1.equals((Point3D)hPoint, 1.0 + ABSOLUTE_ERROR));
        assertFalse(iPoint1.equals(hPoint, 0.5));
        assertFalse(iPoint1.equals((Point3D)hPoint, 0.5));
        
        //Force IllegalArgumentException
        try{
            iPoint1.equals(hPoint, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            iPoint1.equals(iPoint1, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}
        try{
            iPoint1.equals((Point3D)iPoint1, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        }catch(IllegalArgumentException e){}                    
    }
    
    @Test
    public void testIsAtInfinity(){
        double[] array = new double[INHOM_COORDS];
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        InhomogeneousPoint3D iPoint = new InhomogeneousPoint3D(array);
        iPoint.setX(Double.POSITIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());
        
        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setY(Double.POSITIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setZ(Double.POSITIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());


        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setX(Double.NEGATIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());
        
        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setY(Double.NEGATIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setZ(Double.NEGATIVE_INFINITY);
        assertTrue(iPoint.isAtInfinity());


        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setX(Double.NaN);
        assertTrue(iPoint.isAtInfinity());
        
        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setY(Double.NaN);
        assertTrue(iPoint.isAtInfinity());

        iPoint = new InhomogeneousPoint3D(array);
        iPoint.setZ(Double.NaN);
        assertTrue(iPoint.isAtInfinity());

        
        iPoint = new InhomogeneousPoint3D(array);
        assertFalse(iPoint.isAtInfinity());
        
    }
}
