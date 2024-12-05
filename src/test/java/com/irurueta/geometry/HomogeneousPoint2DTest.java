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
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class HomogeneousPoint2DTest {

    private static final int HOM_COORDS = 3;
    private static final int INHOM_COORDS = 2;

    private static final double ABSOLUTE_ERROR = 1e-6;
    private static final double MIN_RANDOM_VALUE = 1.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    @Test
    void testConstructor() {
        var hPoint = new HomogeneousPoint2D();
        assertEquals(0.0, hPoint.getHomX(), 0.0);
        assertEquals(0.0, hPoint.getHomY(), 0.0);
        assertEquals(1.0, hPoint.getHomW(), 0.0);
        assertFalse(hPoint.isNormalized());

        final var randomizer = new UniformRandomizer();
        var array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        hPoint = new HomogeneousPoint2D(array);

        final var array2 = hPoint.asArray();

        assertArrayEquals(array, array2, 0.0);
        assertFalse(hPoint.isNormalized());

        var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        hPoint = new HomogeneousPoint2D(a, b, c);
        array = hPoint.asArray();

        assertEquals(a, array[0], 0.0);
        assertEquals(b, array[1], 0.0);
        assertEquals(c, array[2], 0.0);
        assertFalse(hPoint.isNormalized());

        final var point = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setHomogeneousCoordinates(a, b, c);
        hPoint = new HomogeneousPoint2D(point);
        assertEquals(a, hPoint.getHomX(), 0.0);
        assertEquals(b, hPoint.getHomY(), 0.0);
        assertEquals(c, hPoint.getHomW(), 0.0);
        assertEquals(CoordinatesType.HOMOGENEOUS_COORDINATES, hPoint.getType());
        assertFalse(hPoint.isNormalized());
    }

    @Test
    void testGettersAndSetters() {
        final var randomizer = new UniformRandomizer();
        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var hPoint = new HomogeneousPoint2D();

        hPoint.setX(x);
        hPoint.setY(y);
        hPoint.setW(w);
        assertEquals(x, hPoint.getX(), 0.0);
        assertEquals(y, hPoint.getY(), 0.0);
        assertEquals(w, hPoint.getW(), 0.0);

        hPoint.setHomogeneousCoordinates(homX, homY, homW);
        assertEquals(homX, hPoint.getHomX(), 0.0);
        assertEquals(homY, hPoint.getHomY(), 0.0);
        assertEquals(homW, hPoint.getHomW(), 0.0);
        assertEquals(homX, hPoint.getX(), 0.0);
        assertEquals(homY, hPoint.getY(), 0.0);
        assertEquals(homW, hPoint.getW(), 0.0);

        hPoint.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(inhomX, hPoint.getInhomX(), 0.0);
        assertEquals(inhomY, hPoint.getInhomY(), 0.0);
        assertEquals(inhomX,hPoint.getX() / hPoint.getW(), 0.0);
        assertEquals(inhomY,hPoint.getY() / hPoint.getW(), 0.0);
    }

    @Test
    void testToInhomogeneous() {
        final var randomizer = new UniformRandomizer();
        final var array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var hPoint = new HomogeneousPoint2D(array);

        final var iPoint = hPoint.toInhomogeneous();

        assertEquals(hPoint.getX() / hPoint.getW(), iPoint.getX(), ABSOLUTE_ERROR);
        assertEquals(hPoint.getY() / hPoint.getW(), iPoint.getY(), ABSOLUTE_ERROR);
    }

    @Test
    void testSetCoordinates() {
        final var randomizer = new UniformRandomizer();
        var array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        var hPoint = new HomogeneousPoint2D();
        hPoint.setCoordinates(array);
        var array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        // Force IllegalArgumentException
        final var array3 = new double[HOM_COORDS + 1];
        final var hPoint3 = new HomogeneousPoint2D();
        assertThrows(IllegalArgumentException.class, () -> hPoint3.setCoordinates(array3));

        final var array4 = new double[HOM_COORDS - 1];
        final var hPoint4 = new HomogeneousPoint2D();
        assertThrows(IllegalArgumentException.class, () -> hPoint4.setCoordinates(array4));

        final var x = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var y = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var w = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint = new HomogeneousPoint2D();
        hPoint.setCoordinates(x, y, w);
        assertEquals(x, hPoint.getX(), 0.0);
        assertEquals(y, hPoint.getY(), 0.0);
        assertEquals(w, hPoint.getW(), 0.0);

        final var inhomX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var inhomY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint = new HomogeneousPoint2D();
        hPoint.setInhomogeneousCoordinates(inhomX, inhomY);
        assertEquals(inhomX, hPoint.getInhomX(), 0.0);
        assertEquals(inhomY, hPoint.getInhomY(), 0.0);
        assertEquals(inhomX, hPoint.getX() / hPoint.getW(), 0.0);
        assertEquals(inhomY, hPoint.getY() / hPoint.getW(), 0.0);

        array = new double[HOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hPoint2 = new HomogeneousPoint2D(array);
        hPoint = new HomogeneousPoint2D();
        // pass another point to set coordinates
        hPoint.setCoordinates(hPoint2);
        array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array = new double[INHOM_COORDS];
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var iPoint = new InhomogeneousPoint2D(array);
        hPoint = new HomogeneousPoint2D();
        hPoint.setCoordinates(iPoint);
        array2 = hPoint.asArray();
        assertEquals(INHOM_COORDS, array.length);
        assertEquals(HOM_COORDS, array2.length);
        assertEquals(array[0], array2[0], 0.0);
        assertEquals(array[1], array2[1], 0.0);
        assertEquals(1.0, array2[2], 0.0);
    }

    @Test
    void testArray() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var hPoint = new HomogeneousPoint2D(array);
        var array2 = hPoint.asArray();
        assertArrayEquals(array, array2, 0.0);

        array2 = new double[HOM_COORDS];
        hPoint.asArray(array2);
        assertArrayEquals(array, array2, 0.0);
    }

    @Test
    void testEqualsAndHashCode() {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var hPoint1 = new HomogeneousPoint2D(array);
        var hPoint2 = new HomogeneousPoint2D(array);
        assertTrue(hPoint1.equals(hPoint2, 0.0));
        assertTrue(hPoint1.equals((Point2D) hPoint2, 0.0));
        assertEquals(hPoint1.hashCode(), hPoint2.hashCode());

        array[0] = hPoint1.getX() + randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = hPoint1.getY();
        array[2] = hPoint1.getW();
        hPoint2 = new HomogeneousPoint2D(array);
        assertFalse(hPoint1.equals(hPoint2, 0.0));
        assertFalse(hPoint1.equals((Point2D) hPoint2, 0.0));

        array[0] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[1] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        array[2] = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        hPoint1 = new HomogeneousPoint2D(array);
        array[0] *= 2.0;
        hPoint2 = new HomogeneousPoint2D(array);
        assertTrue(hPoint1.equals(hPoint2, 2.0));
        assertTrue(hPoint1.equals((Point2D) hPoint2, 2.0));
        assertFalse(hPoint1.equals(hPoint2, 0.0));
        assertFalse(hPoint1.equals((Point2D) hPoint2, 0.0));

        // Testing equals from one inhomogeneous point
        final var iArray = new double[INHOM_COORDS];
        iArray[0] = array[0] / array[2];
        iArray[1] = array[1] / array[2];

        final var hPoint = new HomogeneousPoint2D(array);
        final var iPoint = new InhomogeneousPoint2D(iArray);
        assertTrue(hPoint.equals(iPoint, ABSOLUTE_ERROR));
        assertTrue(hPoint.equals((Point2D) iPoint, ABSOLUTE_ERROR));

        iArray[0] = hPoint.getInhomX() + 1.0;
        iArray[1] = hPoint.getInhomY() + 1.0;
        final var iPoint2 = new InhomogeneousPoint2D(iArray);
        assertFalse(hPoint.equals(iPoint2, 0.0));
        assertFalse(hPoint.equals((Point2D) iPoint2, 0.0));

        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hPoint3 = new HomogeneousPoint2D(array);

        iArray[0] = hPoint3.getInhomX() + 1.0;
        iArray[1] = hPoint3.getInhomY() + 1.0;
        final var iPoint3 = new InhomogeneousPoint2D(iArray);
        assertTrue(hPoint3.equals(iPoint3, 1.1));
        assertTrue(hPoint3.equals((Point2D) iPoint3, 1.1));
        assertFalse(hPoint3.equals(iPoint3, 0.5));
        assertFalse(hPoint3.equals((Point2D) iPoint3, 0.5));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> hPoint3.equals(hPoint3, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> hPoint3.equals(iPoint, -ABSOLUTE_ERROR));
        assertThrows(IllegalArgumentException.class, () -> hPoint3.equals((Point2D) iPoint, -ABSOLUTE_ERROR));

        final var hPoint4 = new HomogeneousPoint2D();
        final var iPoint4 = new InhomogeneousPoint2D();
        assertTrue(hPoint4.equals(iPoint4));
        assertEquals(iPoint4.hashCode(), hPoint4.hashCode());
        hPoint2 = new HomogeneousPoint2D(hPoint4);
        assertTrue(hPoint4.equals(hPoint2));
        assertEquals(hPoint4.hashCode(), hPoint2.hashCode());
        assertTrue(hPoint4.equals((Point2D) iPoint4));
        assertTrue(hPoint4.equals((Point2D) hPoint2));
    }

    @Test
    void testIsAtInfinity() {
        final var hPoint = new HomogeneousPoint2D();
        // Sets point at infinity
        hPoint.setW(0.0);
        assertTrue(hPoint.isAtInfinity());
        // Forces point not being at infinity
        hPoint.setW(1.0);
        assertFalse(hPoint.isAtInfinity());

        // Testing with threshold
        hPoint.setW(4.0);
        assertTrue(hPoint.isAtInfinity(4.5));
        hPoint.setW(5.0);
        assertFalse(hPoint.isAtInfinity(4.5));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> hPoint.isAtInfinity(-ABSOLUTE_ERROR));
    }

    @Test
    void testNormalize() {
        final var randomizer = new UniformRandomizer();
        final var homX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var homW = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var point = new HomogeneousPoint2D(homX, homY, homW);
        assertEquals(point.getHomX(), homX, 0.0);
        assertEquals(point.getHomY(), homY, 0.0);
        assertEquals(point.getHomW(), homW, 0.0);
        assertFalse(point.isNormalized());

        // compute norm
        final var norm = Math.sqrt(homX * homX + homY * homY + homW * homW);

        // normalize
        point.normalize();
        assertTrue(point.isNormalized());

        // check correctness after normalization
        assertEquals(homX / norm, point.getHomX(), ABSOLUTE_ERROR);
        assertEquals(homY / norm, point.getHomY(),  ABSOLUTE_ERROR);
        assertEquals(homW / norm, point.getHomW(), ABSOLUTE_ERROR);

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

        final var array = new double[HOM_COORDS];
        // initialize to zeros
        Arrays.fill(array, 0.0);
        point.setCoordinates(array);
        assertFalse(point.isNormalized());
        // normalization is not done because of numerical
        point.normalize();
        // instability
        assertFalse(point.isNormalized());

        // fill array with random values
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        point.setCoordinates(array);
        assertFalse(point.isNormalized());
        // now normalization is done
        point.normalize();
        assertTrue(point.isNormalized());

        final var point2 = new HomogeneousPoint2D();
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

        point.setW(homW);
        assertFalse(point.isNormalized());
        point.normalize();
        assertTrue(point.isNormalized());
    }

    @Test
    void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final var array = new double[HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(array, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var hPoint1 = new HomogeneousPoint2D(array);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(hPoint1);
        final var hPoint2 = SerializationHelper.<HomogeneousPoint2D>deserialize(bytes);

        // check
        assertEquals(hPoint1, hPoint2);
    }
}
