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
import java.util.Random;
import org.junit.*;
import static org.junit.Assert.*;

public class Point2DTest {
    
    public static final int HOM_COORDS = 3;
    public static final int INHOM_COORDS = 2;
    
    public static final double MIN_RANDOM_VALUE = -100.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    
    public Point2DTest() { }

    @BeforeClass
    public static void setUpClass() throws Exception { }

    @AfterClass
    public static void tearDownClass() throws Exception { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testCreate() {
        Point2D point;
        
        point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(point.getType(), CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(point.getDimensions(), 2);
        assertEquals(point.getInhomX(), 0.0, 0.0);
        assertEquals(point.getInhomY(), 0.0, 0.0);
        assertEquals(point.getHomX(), 0.0, 0.0);
        assertEquals(point.getHomY(), 0.0, 0.0);
        assertEquals(point.getHomW(), 1.0, 0.0);
        
        point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(point.getType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(point.getDimensions(), 2);
        assertEquals(point.getInhomX(), 0.0, 0.0);
        assertEquals(point.getInhomY(), 0.0, 0.0);
        assertEquals(point.getHomX(), 0.0, 0.0);
        assertEquals(point.getHomY(), 0.0, 0.0);
        assertEquals(point.getHomW(), 1.0, 0.0);
        
        double[] array = new double[HOM_COORDS];
        double[] iArray = new double[INHOM_COORDS];
        
        point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array);
        assertEquals(point.getType(), CoordinatesType.HOMOGENEOUS_COORDINATES);
        
        point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, 
                iArray);
        assertEquals(point.getType(), 
                CoordinatesType.INHOMOGENEOUS_COORDINATES);
        
        //Force IllegalArgumentException
        try {
            Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, array);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        try {
            Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, iArray);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        
        point = Point2D.create(array);
        assertEquals(point.getType(), Point2D.DEFAULT_COORDINATES_TYPE);
        
        //Force IllegalArgumentException
        try {
            Point2D.create(iArray);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        
        point = Point2D.create();
        assertEquals(point.getType(), Point2D.DEFAULT_COORDINATES_TYPE);        
    }

    @Test
    public void testGetSetInhomX() {
        Point2D point;

        //check default values for homogeneous coordinates
        point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(point.getType(), CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(point.getDimensions(), 2);
        assertEquals(point.getInhomX(), 0.0, 0.0);
        assertEquals(point.getInhomY(), 0.0, 0.0);

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setInhomX(inhomX);

        //check
        assertEquals(point.getInhomX(), inhomX, ABSOLUTE_ERROR);
        assertEquals(point.getInhomY(), 0.0, 0.0);

        //check default values for inhomogeneous coordinates
        point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(point.getType(), CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(point.getDimensions(), 2);
        assertEquals(point.getInhomX(), 0.0, 0.0);
        assertEquals(point.getInhomY(), 0.0, 0.0);

        //set new value
        point.setInhomX(inhomX);

        //check
        assertEquals(point.getInhomX(), inhomX, ABSOLUTE_ERROR);
        assertEquals(point.getInhomY(), 0.0, 0.0);
    }

    @Test
    public void testGetSetInhomY() {
        Point2D point;

        //check default values for homogeneous coordinates
        point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(point.getType(), CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(point.getDimensions(), 2);
        assertEquals(point.getInhomX(), 0.0, 0.0);
        assertEquals(point.getInhomY(), 0.0, 0.0);

        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setInhomY(inhomY);

        //check
        assertEquals(point.getInhomY(), inhomY, ABSOLUTE_ERROR);
        assertEquals(point.getInhomX(), 0.0, 0.0);

        //check default values for inhomogeneous coordinates
        point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(point.getType(), CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(point.getDimensions(), 2);
        assertEquals(point.getInhomX(), 0.0, 0.0);
        assertEquals(point.getInhomY(), 0.0, 0.0);

        //set new value
        point.setInhomY(inhomY);

        //check
        assertEquals(point.getInhomY(), inhomY, ABSOLUTE_ERROR);
        assertEquals(point.getInhomX(), 0.0, 0.0);

    }

    @Test
    public void testGetSetInhomogeneousCoordinate() {
        Point2D point;

        //check default values
        point = Point2D.create();
        assertEquals(point.getInhomX(), 0.0, 0.0);
        assertEquals(point.getInhomY(), 0.0, 0.0);
        assertEquals(point.getInhomogeneousCoordinate(0), point.getInhomX(), 0.0);
        assertEquals(point.getInhomogeneousCoordinate(1), point.getInhomY(), 0.0);

        //set new values
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        point.setInhomogeneousCoordinate(0, inhomX);

        //check
        assertEquals(point.getInhomogeneousCoordinate(0), inhomX, ABSOLUTE_ERROR);
        assertEquals(point.getInhomX(), inhomX, ABSOLUTE_ERROR);

        point.setInhomogeneousCoordinate(1, inhomY);

        //check
        assertEquals(point.getInhomogeneousCoordinate(1), inhomY, ABSOLUTE_ERROR);
        assertEquals(point.getInhomY(), inhomY, ABSOLUTE_ERROR);

        //Force IllegalArgumentException
        try {
            point.getInhomogeneousCoordinate(-1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            point.getInhomogeneousCoordinate(2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            point.setInhomogeneousCoordinate(-1, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            point.setInhomogeneousCoordinate(2, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
    
    @Test
    public void testDistanceTo() {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D point1 = Point2D.create();
        Point2D point2 = Point2D.create();
        
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        double diffX = point1.getInhomX() - point2.getInhomX();
        double diffY = point1.getInhomY() - point2.getInhomY();
        double distance = Math.sqrt(diffX * diffX + diffY * diffY);
        
        //check distance
        assertEquals(point1.distanceTo(point2), distance, ABSOLUTE_ERROR);
        assertEquals(point2.distanceTo(point1), distance, ABSOLUTE_ERROR);
        
        //check distance to themselves
        assertEquals(point1.distanceTo(point1), 0.0, ABSOLUTE_ERROR);
        assertEquals(point2.distanceTo(point2), 0.0, ABSOLUTE_ERROR);
    }

    @Test
    public void testSqrDistanceTo() {

        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D point1 = Point2D.create();
        Point2D point2 = Point2D.create();

        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        double diffX = point1.getInhomX() - point2.getInhomX();
        double diffY = point1.getInhomY() - point2.getInhomY();
        double sqrDistance = diffX * diffX + diffY * diffY;

        //check distance
        assertEquals(point1.sqrDistanceTo(point2), sqrDistance, ABSOLUTE_ERROR);
        assertEquals(point2.sqrDistanceTo(point1), sqrDistance, ABSOLUTE_ERROR);

        //check distance to themselves
        assertEquals(point1.sqrDistanceTo(point1), 0.0, ABSOLUTE_ERROR);
        assertEquals(point2.sqrDistanceTo(point2), 0.0, ABSOLUTE_ERROR);
    }
    
    @Test
    public void testDotProduct() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        Point2D point1 = Point2D.create();
        Point2D point2 = Point2D.create();
        Point2D point3 = Point2D.create();
        
        point1.setHomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setHomogeneousCoordinates(-point1.getHomX(), -point1.getHomY(), 
                -point1.getHomW());
        point3.setHomogeneousCoordinates(
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
                point1.getHomW() * point3.getHomW(), ABSOLUTE_ERROR);
    }
    
    @Test
    public void testIsBetween() {
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double valueBetween = randomizer.nextDouble(0.2, 0.8);
        double valueOutside = 1.0 + valueBetween;
        
        Point2D point1 = Point2D.create();
        Point2D point2 = Point2D.create();
        
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        
        double diffX = point2.getInhomX() - point1.getInhomX();
        double diffY = point2.getInhomY() - point1.getInhomY();
        double dist = Math.sqrt(diffX * diffX + diffY * diffY);
        
        Point2D betweenPoint = Point2D.create();
        betweenPoint.setInhomogeneousCoordinates(
                point1.getInhomX() + valueBetween * diffX, 
                point1.getInhomY() + valueBetween * diffY);

        Point2D outsidePoint = Point2D.create();
        outsidePoint.setInhomogeneousCoordinates(
                point1.getInhomX() + valueOutside * diffX, 
                point1.getInhomY() + valueOutside * diffY);
        
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
        try {
            betweenPoint.isBetween(point1, point2, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
    }
}
