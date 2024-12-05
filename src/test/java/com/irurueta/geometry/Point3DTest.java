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
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.*;

class Point3DTest {

    private static final int HOM_COORDS = 4;
    private static final int INHOM_COORDS = 3;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int MIN_POINTS = 100;
    private static final int MAX_POINTS = 500;

    @Test
    void testConstants() {
        assertEquals(1e-10, Point3D.DEFAULT_COMPARISON_THRESHOLD, 0.0);
        assertEquals(0.0, Point3D.MIN_THRESHOLD, 0.0);
        assertEquals(4, Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(3, Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, Point3D.DEFAULT_COORDINATES_TYPE);
    }

    @Test
    void testCreate() {
        var point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(3, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);
        assertEquals(0.0, point.getHomX(), 0.0);
        assertEquals(0.0, point.getHomY(), 0.0);
        assertEquals(0.0, point.getHomZ(), 0.0);
        assertEquals(1.0, point.getHomW(), 0.0);

        point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(3, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);
        assertEquals(0.0, point.getHomX(), 0.0);
        assertEquals(0.0, point.getHomY(), 0.0);
        assertEquals(0.0, point.getHomZ(), 0.0);
        assertEquals(1.0, point.getHomW(), 0.0);

        final var array = new double[HOM_COORDS];
        final var iArray = new double[INHOM_COORDS];

        point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, array);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, point.getType());

        point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, iArray);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, point.getType());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, array));
        assertThrows(IllegalArgumentException.class,
                () -> Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES, iArray));

        point = Point3D.create(array);
        assertEquals(Point3D.DEFAULT_COORDINATES_TYPE, point.getType());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> Point3D.create(iArray));

        point = Point3D.create();
        assertEquals(Point3D.DEFAULT_COORDINATES_TYPE, point.getType());
    }

    @Test
    void testGetSetInhomX() {
        // check default values for homogeneous coordinates
        var point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(3, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setInhomX(inhomX);

        // check
        assertEquals(inhomX, point.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);

        // check default values for inhomogeneous coordinates
        point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(3, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);

        // set new value
        point.setInhomX(inhomX);

        // check
        assertEquals(inhomX, point.getInhomX(), ABSOLUTE_ERROR);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);
    }

    @Test
    void testGetSetInhomY() {
        // check default values for homogeneous coordinates
        var point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(3, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setInhomY(inhomY);

        // check
        assertEquals(inhomY, point.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);

        // check default values for inhomogeneous coordinates
        point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(3, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);

        // set new value
        point.setInhomY(inhomY);

        // check
        assertEquals(inhomY, point.getInhomY(), ABSOLUTE_ERROR);
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);
    }

    @Test
    void testGetSetInhomZ() {
        // check default values for homogeneous coordinates
        var point = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(3, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var inhomZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setInhomZ(inhomZ);

        // check
        assertEquals(inhomZ, point.getInhomZ(), ABSOLUTE_ERROR);
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);

        // check default values for inhomogeneous coordinates
        point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        assertEquals(CoordinatesType.INHOMOGENEOUS_COORDINATES, point.getType());
        assertEquals(3, point.getDimensions());
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);

        // set new value
        point.setInhomZ(inhomZ);

        // check
        assertEquals(inhomZ, point.getInhomZ(), ABSOLUTE_ERROR);
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
    }

    @Test
    void testGetSetInhomogeneousCoordinate() {
        // check default values
        final var point = Point3D.create();
        assertEquals(0.0, point.getInhomX(), 0.0);
        assertEquals(0.0, point.getInhomY(), 0.0);
        assertEquals(0.0, point.getInhomZ(), 0.0);
        assertEquals(point.getInhomX(), point.getInhomogeneousCoordinate(0), 0.0);
        assertEquals(point.getInhomY(), point.getInhomogeneousCoordinate(1), 0.0);
        assertEquals(point.getInhomZ(), point.getInhomogeneousCoordinate(2), 0.0);

        // set new values
        final var randomizer = new UniformRandomizer();
        final var inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        point.setInhomogeneousCoordinate(0, inhomX);

        // check
        assertEquals(inhomX, point.getInhomogeneousCoordinate(0), ABSOLUTE_ERROR);
        assertEquals(inhomX, point.getInhomX(), ABSOLUTE_ERROR);

        point.setInhomogeneousCoordinate(1, inhomY);

        // check
        assertEquals(inhomY, point.getInhomogeneousCoordinate(1), ABSOLUTE_ERROR);
        assertEquals(inhomY, point.getInhomY(), ABSOLUTE_ERROR);

        point.setInhomogeneousCoordinate(2, inhomZ);

        // check
        assertEquals(inhomZ, point.getInhomogeneousCoordinate(2), ABSOLUTE_ERROR);
        assertEquals(inhomZ, point.getInhomZ(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> point.getInhomogeneousCoordinate(-1));
        assertThrows(IllegalArgumentException.class, () -> point.getInhomogeneousCoordinate(3));
        assertThrows(IllegalArgumentException.class, () -> point.setInhomogeneousCoordinate(-1, 0.0));
        assertThrows(IllegalArgumentException.class, () -> point.setInhomogeneousCoordinate(3, 0.0));
    }

    @Test
    void testDistanceTo() {
        final var randomizer = new UniformRandomizer();
        final var point1 = Point3D.create();
        final var point2 = Point3D.create();

        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var diffX = point1.getInhomX() - point2.getInhomX();
        final var diffY = point1.getInhomY() - point2.getInhomY();
        final var diffZ = point1.getInhomZ() - point2.getInhomZ();
        final var distance = Math.sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);

        // check distance
        assertEquals(point1.distanceTo(point2), distance, ABSOLUTE_ERROR);
        assertEquals(point2.distanceTo(point1), distance, ABSOLUTE_ERROR);

        // check distance to themselves
        assertEquals(0.0, point1.distanceTo(point1), ABSOLUTE_ERROR);
        assertEquals(0.0, point2.distanceTo(point2), ABSOLUTE_ERROR);
    }

    @Test
    void testSqrDistanceTo() {
        final var randomizer = new UniformRandomizer();
        final var point1 = Point3D.create();
        final var point2 = Point3D.create();

        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var diffX = point1.getInhomX() - point2.getInhomX();
        final var diffY = point1.getInhomY() - point2.getInhomY();
        final var diffZ = point1.getInhomZ() - point2.getInhomZ();
        final var sqrDistance = diffX * diffX + diffY * diffY + diffZ * diffZ;

        // check distance
        assertEquals(sqrDistance, point1.sqrDistanceTo(point2), ABSOLUTE_ERROR);
        assertEquals(sqrDistance, point2.sqrDistanceTo(point1), ABSOLUTE_ERROR);

        // check distance to themselves
        assertEquals(0.0, point1.sqrDistanceTo(point1), ABSOLUTE_ERROR);
        assertEquals(0.0, point2.sqrDistanceTo(point2), ABSOLUTE_ERROR);
    }

    @Test
    void testDotProduct() {
        final var randomizer = new UniformRandomizer();
        final var point1 = Point3D.create();
        final var point2 = Point3D.create();
        final var point3 = Point3D.create();

        point1.setHomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setHomogeneousCoordinates(-point1.getHomX(), -point1.getHomY(), -point1.getHomZ(), -point1.getHomW());
        point3.setHomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
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
        assertEquals(point1.dotProduct(point3), point1.getHomX() * point3.getHomX()
                + point1.getHomY() * point3.getHomY()
                + point1.getHomZ() * point3.getHomZ()
                + point1.getHomW() * point3.getHomW(), ABSOLUTE_ERROR);
    }

    @Test
    void testIsBetween() {
        final var randomizer = new UniformRandomizer();
        final var valueBetween = randomizer.nextDouble(0.2, 0.8);
        final var valueOutside = 1.0 + valueBetween;

        final var point1 = Point3D.create();
        final var point2 = Point3D.create();

        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var diffX = point2.getInhomX() - point1.getInhomX();
        final var diffY = point2.getInhomY() - point1.getInhomY();
        final var diffZ = point2.getInhomZ() - point1.getInhomZ();
        final var dist = Math.sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ);

        final var betweenPoint = Point3D.create();
        betweenPoint.setInhomogeneousCoordinates(
                point1.getInhomX() + valueBetween * diffX,
                point1.getInhomY() + valueBetween * diffY,
                point1.getInhomZ() + valueBetween * diffZ);

        final var outsidePoint = Point3D.create();
        outsidePoint.setInhomogeneousCoordinates(
                point1.getInhomX() + valueOutside * diffX,
                point1.getInhomY() + valueOutside * diffY,
                point1.getInhomZ() + valueOutside * diffZ);

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
        assertThrows(IllegalArgumentException.class, () -> betweenPoint.isBetween(point1, point2, -ABSOLUTE_ERROR));
    }

    @Test
    void testCentroid() {
        final var randomizer = new UniformRandomizer();
        final var numPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var points = new ArrayList<Point3D>();
        double x;
        double y;
        double z;
        var meanX = 0.0;
        var meanY = 0.0;
        var meanZ = 0.0;
        for (var i = 0; i < numPoints; i++) {
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

        final var mean1 = Point3D.create();
        Point3D.centroid(points, mean1);
        final var mean2 = Point3D.centroid(points);

        assertEquals(mean1, mean2);
        assertEquals(mean1.getInhomX(), meanX, ABSOLUTE_ERROR);
        assertEquals(mean1.getInhomY(), meanY, ABSOLUTE_ERROR);
        assertEquals(mean1.getInhomZ(), meanZ, ABSOLUTE_ERROR);
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();
        final var inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var point1 = Point3D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        point1.setInhomogeneousCoordinates(inhomX, inhomY, inhomZ);

        // check
        assertEquals(inhomX, point1.getInhomX(), 0.0);
        assertEquals(inhomY, point1.getInhomY(), 0.0);
        assertEquals(inhomZ, point1.getInhomZ(), 0.0);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(point1);
        final var point2 = SerializationHelper.<Point3D>deserialize(bytes);

        // check
        assertEquals(point1, point2);
        assertNotSame(point1, point2);
    }
}
