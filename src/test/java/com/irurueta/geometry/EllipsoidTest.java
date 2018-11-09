/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.geometry;

import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Arrays;
import java.util.Random;

import static org.junit.Assert.*;

public class EllipsoidTest {
    
    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;
    
    private static final double MIN_RANDOM_DEGREES = -180.0;
    private static final double MAX_RANDOM_DEGREES = 180.0;
    
    public EllipsoidTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        //test empty constructor.
        Ellipsoid ellipsoid = new Ellipsoid();
        
        //check center is at originan axes are unitary
        assertTrue(ellipsoid.getCenter().equals(new InhomogeneousPoint3D(0.0, 
                0.0, 0.0), ABSOLUTE_ERROR));        
        assertArrayEquals(ellipsoid.getSemiAxesLengths(), 
                new double[]{1.0, 1.0, 1.0}, ABSOLUTE_ERROR);        
        assertEquals(ellipsoid.getRotation(), Rotation3D.create());
        
        //test constructo with center, axes and rotation
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));    
        double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        Quaternion rotation = new Quaternion(roll, pitch, yaw);
        ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);
        
        //check correctness
        assertEquals(ellipsoid.getCenter(), center);
        assertArrayEquals(ellipsoid.getSemiAxesLengths(), semiAxesLengths, 0.0);
        assertEquals(ellipsoid.getRotation(), rotation);
        
        //Force IllegalArgumentException
        ellipsoid = null;
        try {
            ellipsoid = new Ellipsoid(center, new double[1], rotation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(ellipsoid);
    }
    
    @Test
    public void testGetSetCenter() {
        Ellipsoid ellipsoid = new Ellipsoid();
        
        //check default value
        assertTrue(ellipsoid.getCenter().equals(new InhomogeneousPoint3D(0.0, 
                0.0, 0.0), ABSOLUTE_ERROR));        
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));    
        ellipsoid.setCenter(center);
        
        //check correctness
        assertEquals(ellipsoid.getCenter(), center);
    }
    
    @Test
    public void testGetSetSemiAxesLengths() {
        Ellipsoid ellipsoid = new Ellipsoid();
        
        //check default value
        assertArrayEquals(ellipsoid.getSemiAxesLengths(), 
                new double[]{1.0, 1.0, 1.0}, ABSOLUTE_ERROR);        

        //set new value
        double[] axes = new double[Ellipsoid.DIMENSIONS];
        ellipsoid.setSemiAxesLengths(axes);
        
        //check correctness
        assertSame(ellipsoid.getSemiAxesLengths(), axes);
        
        //Force IllegalArgumentException
        try {
            ellipsoid.setSemiAxesLengths(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testGetSetRotation() {
        Ellipsoid ellipsoid = new Ellipsoid();
        
        //check default value
        assertEquals(ellipsoid.getRotation(), Rotation3D.create());
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        Quaternion rotation = new Quaternion(roll, pitch, yaw);
        ellipsoid.setRotation(rotation);
        
        //check correctness
        assertEquals(ellipsoid.getRotation(), rotation);        
    }
    
    @Test
    public void testGetVolume() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));    
        double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        Quaternion rotation = new Quaternion(roll, pitch, yaw);
        Ellipsoid ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);

        //check correctness
        double a = semiAxesLengths[0];
        double b = semiAxesLengths[1];
        double c = semiAxesLengths[2];
        
        assertEquals(ellipsoid.getVolume(), 4.0 / 3.0 * Math.PI * a * b * c, 
                ABSOLUTE_ERROR);
        
        //test from circle
        double radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        Sphere sphere = new Sphere(center, radius);
        ellipsoid.setFromSphere(sphere);
        
        assertEquals(ellipsoid.getVolume(),  sphere.getVolume(), 
                ABSOLUTE_ERROR);
    }
    
    @Test
    public void testGetSurface() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));    
        double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        randomizer.fill(semiAxesLengths, MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_RANDOM_DEGREES, MAX_RANDOM_DEGREES));
        Quaternion rotation = new Quaternion(roll, pitch, yaw);
        Ellipsoid ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);

        //check correctness
        double a = semiAxesLengths[0];
        double b = semiAxesLengths[1];
        double c = semiAxesLengths[2];
        
        double p = 1.6075;
        assertEquals(ellipsoid.getSurface(), 4.0 * Math.PI * Math.pow(
                (Math.pow(a*b, p) + Math.pow(a*c, p) + Math.pow(b*c, p)) / 3.0, 
                1.0 / p), ABSOLUTE_ERROR);
        
        //test from circle
        double radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        Sphere sphere = new Sphere(center, radius);
        ellipsoid.setFromSphere(sphere);
        
        assertEquals(ellipsoid.getSurface(),  sphere.getSurface(), 
                ABSOLUTE_ERROR);        
    }
    
    @Test
    public void testToQuadric() throws GeometryException {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point3D center = new InhomogeneousPoint3D(randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), randomizer.nextDouble(
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));  
        double radius = randomizer.nextDouble(MAX_RANDOM_VALUE / 2.0, 
                MAX_RANDOM_VALUE);
        double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        Arrays.fill(semiAxesLengths, radius);
        Rotation3D rotation = Rotation3D.create();
        Ellipsoid ellipsoid = new Ellipsoid(center, semiAxesLengths, rotation);
        Quadric quadric1 = ellipsoid.toQuadric();
        
        Sphere sphere = new Sphere(center, radius);
        Quadric quadric2 = sphere.toQuadric();
        
        quadric1.normalize();
        quadric2.normalize();
        
        assertEquals(Math.abs(quadric1.getA()), Math.abs(quadric2.getA()), 
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getB()), Math.abs(quadric2.getB()), 
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getC()), Math.abs(quadric2.getC()), 
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getD()), Math.abs(quadric2.getD()), 
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getE()), Math.abs(quadric2.getE()), 
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getF()), Math.abs(quadric2.getF()), 
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getG()), Math.abs(quadric2.getG()), 
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getH()), Math.abs(quadric2.getH()), 
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getI()), Math.abs(quadric2.getI()), 
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(quadric1.getJ()), Math.abs(quadric2.getJ()), 
                ABSOLUTE_ERROR);
    }
}
