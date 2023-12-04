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
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class Point2DTest {

    private static final int HOM_COORDS = 3;
    private static final int INHOM_COORDS = 2;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    public void testConstants() {
        assertEquals(1e-10, Point2D.DEFAULT_COMPARISON_THRESHOLD, 0.0);
        assertEquals(0.0, Point2D.MIN_THRESHOLD, 0.0);
        assertEquals(3, Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(2, Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, Point2D.DEFAULT_COORDINATES_TYPE);
    }

    @Test
    public void testCreate() {
        Point2D point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(2, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getHomX(), 0.0);
        assertEquals(0.0, point.getHomY(), 0.0);
        assertEquals(1.0, point.getHomW(), 0.0);

        point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(2, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getHomX(), 0.0);
        assertEquals(0.0, point.getHomY(), 0.0);
        assertEquals(1.0, point.getHomW(), 0.0);

        final double[] array = new double[HOM_COORDS];
        final double[] iArray = new double[INHOM_COORDS];

        point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, point.getType());

        point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, iArray);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, point.getType());

        // Force IllegalArgumentException
        try {
            Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, array);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        try {
            Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, iArray);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        point = Point2D.create(array);
        assertEquals(Point2D.DEFAULT_COORDINATES_TYPE, point.getType());

        // Force IllegalArgumentException
        try {
            Point2D.create(iArray);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) {
        }

        point = Point2D.create();
        assertEquals(Point2D.DEFAULT_COORDINATES_TYPE, point.getType());
    }

    @Test
    public void testGetSetInhomX() {
        // check default values for homogeneous coordinates
        Point2D point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(2, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setInhomX(inhomX);

        // check
        assertEquals(inhomX, point.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(0.0, point.getInhomY(), 0.0);

        // check default values for inhomogeneous coordinates
        point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(2, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);

        // set new value
        point.setInhomX(inhomX);

        // check
        assertEquals(inhomX, point.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(0.0, point.getInhomY(), 0.0);
    }

    @Test
    public void testGetSetInhomY() {
        // check default values for homogeneous coordinates
        Point2D point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(2, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setInhomY(inhomY);

        // check
        assertEquals(point.getInhomY(), inhomY, ABSOLUTE_ERROR);
        assertEquals(0.0, point.getInhomX(), 0.0);

        // check default values for inhomogeneous coordinates
        point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(2, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);

        // set new value
        point.setInhomY(inhomY);

        // check
        assertEquals(inhomY, point.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(0.0, point.getInhomX(), 0.0);
    }

    @Test
    public void testGetSetInhomogeneousCoordinate() {
        // check default values
        Point2D point = Point2D.create();
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(point.getInhomX(), point.getInhomogeneousCoordinate(0), 0.0);
        assertEquals(point.getInhomY(), point.getInhomogeneousCoordinate(1), 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        point.setInhomogeneousCoordinate(0, inhomX);

        // check
        assertEquals(inhomX, point.getInhomogeneousCoordinate(0), ABSOLUTE_ERROR);
        assertEquals(inhomX, point.getInhomX(), ABSOLUTE_ERROR);

        point.setInhomogeneousCoordinate(1, inhomY);

        // check
        assertEquals(inhomY, point.getInhomogeneousCoordinate(1), ABSOLUTE_ERROR);
        assertEquals(inhomY, point.getInhomY(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        try {
            point.getInhomogeneousCoordinate(-1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            point.getInhomogeneousCoordinate(2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            point.setInhomogeneousCoordinate(-1, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            point.setInhomogeneousCoordinate(2, 0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testDistanceTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D point1 = Point2D.create();
        final Point2D point2 = Point2D.create();

        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final double diffX = point1.getInhomX() - point2.getInhomX();
        final double diffY = point1.getInhomY() - point2.getInhomY();
        final double distance = Math.sqrt(diffX * diffX + diffY * diffY);

        // check distance
        assertEquals(distance, point1.distanceTo(point2), ABSOLUTE_ERROR);
        assertEquals(distance, point2.distanceTo(point1), ABSOLUTE_ERROR);

        // check distance to themselves
        assertEquals(0.0, point1.distanceTo(point1), ABSOLUTE_ERROR);
        assertEquals(0.0, point2.distanceTo(point2), ABSOLUTE_ERROR);
    }

    @Test
    public void testSqrDistanceTo() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D point1 = Point2D.create();
        final Point2D point2 = Point2D.create();

        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final double diffX = point1.getInhomX() - point2.getInhomX();
        final double diffY = point1.getInhomY() - point2.getInhomY();
        final double sqrDistance = diffX * diffX + diffY * diffY;

        // check distance
        assertEquals(sqrDistance, point1.sqrDistanceTo(point2), ABSOLUTE_ERROR);
        assertEquals(sqrDistance, point2.sqrDistanceTo(point1), ABSOLUTE_ERROR);

        // check distance to themselves
        assertEquals(0.0, point1.sqrDistanceTo(point1), ABSOLUTE_ERROR);
        assertEquals(0.0, point2.sqrDistanceTo(point2), ABSOLUTE_ERROR);
    }

    @Test
    public void testDotProduct() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Point2D point1 = Point2D.create();
        final Point2D point2 = Point2D.create();
        final Point2D point3 = Point2D.create();

        point1.setHomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setHomogeneousCoordinates(-point1.getHomX(), -point1.getHomY(), -point1.getHomW());
        point3.setHomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // dot product for same point
        assertEquals(1.0, point1.dotProduct(point1), ABSOLUTE_ERROR);
        // dot product for opposite signs
        assertEquals(-1.0, point1.dotProduct(point2), ABSOLUTE_ERROR);
        // dot product for random points
        point1.normalize();
        point3.normalize();
        assertEquals(point1.getHomX() * point3.getHomX() + point1.getHomY() * point3.getHomY()
                + point1.getHomW() * point3.getHomW(), point1.dotProduct(point3), ABSOLUTE_ERROR);
    }

    @Test
    public void testIsBetween() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double valueBetween = randomizer.nextDouble(0.2, 0.8);
        final double valueOutside = 1.0 + valueBetween;

        final Point2D point1 = Point2D.create();
        final Point2D point2 = Point2D.create();

        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final double diffX = point2.getInhomX() - point1.getInhomX();
        final double diffY = point2.getInhomY() - point1.getInhomY();
        final double dist = Math.sqrt(diffX * diffX + diffY * diffY);

        final Point2D betweenPoint = Point2D.create();
        betweenPoint.setInhomogeneousCoordinates(
                point1.getInhomX() + valueBetween * diffX,
                point1.getInhomY() + valueBetween * diffY);

        final Point2D outsidePoint = Point2D.create();
        outsidePoint.setInhomogeneousCoordinates(
                point1.getInhomX() + valueOutside * diffX,
                point1.getInhomY() + valueOutside * diffY);

        assertTrue(betweenPoint.isBetween(point1, point2));
        assertFalse(outsidePoint.isBetween(point1, point2));

        // the same is true for a small threshold
        assertTrue(betweenPoint.isBetween(point1, point2, ABSOLUTE_ERROR));
        assertFalse(outsidePoint.isBetween(point1, point2, ABSOLUTE_ERROR));

        // with a large enough threshold, even outside point is considered to lie
        // in between
        assertTrue(betweenPoint.isBetween(point1, point2, 2.0 * dist));
        assertTrue(outsidePoint.isBetween(point1, point2, 2.0 * dist));

        // Force IllegalArgumentException
        try {
            betweenPoint.isBetween(point1, point2, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Point2D point1 = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        point1.setInhomogeneousCoordinates(inhomX, inhomY);

        // check
        assertEquals(inhomX, point1.getInhomX(), 0.0);
        assertEquals(inhomY, point1.getInhomY(), 0.0);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(point1);
        final Point2D point2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(point1, point2);
        assertNotSame(point1, point2);
    }
}
