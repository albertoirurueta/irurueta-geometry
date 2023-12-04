/*
 * Copyright (C) 2012 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class AffineParameters2DTest {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final int INHOM_COORDS = 2;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testConstructors() throws WrongSizeException {
        // Test empty constructor
        AffineParameters2D params = new AffineParameters2D();

        // check correctness
        assertEquals(AffineParameters2D.DEFAULT_SCALE, params.getScaleX(), 0.0);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, params.getScaleY(), 0.0);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, params.getSkewness(), 0.0);

        // Test constructor with scale
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        params = new AffineParameters2D(scale);

        // check correctness
        assertEquals(scale, params.getScaleX(), 0.0);
        assertEquals(scale, params.getScaleY(), 0.0);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, params.getSkewness(), 0.0);

        // Test constructor with scale and skewness
        final double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        params = new AffineParameters2D(scale, skewness);

        // check correctness
        assertEquals(scale, params.getScaleX(), 0.0);
        assertEquals(scale, params.getScaleY(), 0.0);
        assertEquals(skewness, params.getSkewness(), 0.0);

        // Test constructor with scaleX, scaleY and skewness
        final double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        params = new AffineParameters2D(scaleX, scaleY, skewness);

        // check correctness
        assertEquals(scaleX, params.getScaleX(), 0.0);
        assertEquals(scaleY, params.getScaleY(), 0.0);
        assertEquals(skewness, params.getSkewness(), 0.0);

        // Test constructor with matrix
        final Matrix m = new Matrix(INHOM_COORDS, INHOM_COORDS);
        m.setElementAt(0, 0, scaleX);
        m.setElementAt(1, 1, scaleY);
        m.setElementAt(0, 1, skewness);

        params = new AffineParameters2D(m);

        // check correctness
        assertEquals(scaleX, params.getScaleX(), 0.0);
        assertEquals(scaleY, params.getScaleY(), 0.0);
        assertEquals(skewness, params.getSkewness(), 0.0);

        // Force IllegalArgumentException (invalid size)
        final Matrix badM1 = new Matrix(INHOM_COORDS + 1, INHOM_COORDS + 1);
        params = null;
        try {
            params = new AffineParameters2D(badM1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force IllegalArgumentException (non upper triangular matrix
        final Matrix badM2 = new Matrix(INHOM_COORDS, INHOM_COORDS);
        badM2.setElementAt(1, 0, 1.0);
        try {
            params = new AffineParameters2D(badM2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(params);

        // Test constructor with matrix and threshold
        final double threshold = randomizer.nextDouble();
        params = new AffineParameters2D(m, threshold);

        // check correctness
        assertEquals(scaleX, params.getScaleX(), 0.0);
        assertEquals(scaleY, params.getScaleY(), 0.0);
        assertEquals(skewness, params.getSkewness(), 0.0);

        // Force IllegalArgumentException (invalid size)
        params = null;
        try {
            params = new AffineParameters2D(badM1, threshold);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force IllegalArgumentException (non upper triangular matrix)
        badM2.setElementAt(1, 0, threshold + 1.0);
        try {
            params = new AffineParameters2D(badM2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force IllegalArgumentException (with negative threshold)
        try {
            params = new AffineParameters2D(m, -threshold);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(params);
    }

    @Test
    public void testGetSetScaleX() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters2D params = new AffineParameters2D();

        // check default values
        assertEquals(AffineParameters2D.DEFAULT_SCALE, params.getScaleX(), 0.0);

        // set new value
        params.setScaleX(scaleX);

        // check correctness
        assertEquals(scaleX, params.getScaleX(), 0.0);
    }

    @Test
    public void testGetSetScaleY() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters2D params = new AffineParameters2D();

        // check default values
        assertEquals(AffineParameters2D.DEFAULT_SCALE, params.getScaleY(), 0.0);

        // set new value
        params.setScaleY(scaleY);

        // check correctness
        assertEquals(scaleY, params.getScaleY(), 0.0);
    }

    @Test
    public void testSetScale() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters2D params = new AffineParameters2D();

        // check default values
        assertEquals(AffineParameters2D.DEFAULT_SCALE, params.getScaleX(), 0.0);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, params.getScaleY(), 0.0);

        // set new value
        params.setScale(scale);

        // check correctness
        assertEquals(scale, params.getScaleX(), 0.0);
        assertEquals(scale, params.getScaleY(), 0.0);
    }

    @Test
    public void testGetSetSkewness() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters2D params = new AffineParameters2D();

        // check default values
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, params.getSkewness(), 0.0);

        // set new value
        params.setSkewness(skewness);

        // check correctness
        assertEquals(skewness, params.getSkewness(), 0.0);
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters2D params = new AffineParameters2D(scaleX, scaleY,
                skewness);

        final Matrix expectedMatrix = new Matrix(INHOM_COORDS, INHOM_COORDS);
        expectedMatrix.initialize(0.0);
        expectedMatrix.setElementAt(0, 0, scaleX);
        expectedMatrix.setElementAt(1, 1, scaleY);
        expectedMatrix.setElementAt(0, 1, skewness);

        final Matrix m1 = params.asMatrix();
        final Matrix m2 = new Matrix(INHOM_COORDS, INHOM_COORDS);
        params.asMatrix(m2);

        assertTrue(expectedMatrix.equals(m1, ABSOLUTE_ERROR));
        assertTrue(expectedMatrix.equals(m2, ABSOLUTE_ERROR));
    }

    @Test
    public void testFromMatrixAndIsValidMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double threshold = randomizer.nextDouble();

        // build valid matrix
        final Matrix m = new Matrix(INHOM_COORDS, INHOM_COORDS);
        m.setElementAt(0, 0, scaleX);
        m.setElementAt(1, 1, scaleY);
        m.setElementAt(0, 1, skewness);

        // check matrix is valid
        assertTrue(AffineParameters2D.isValidMatrix(m));
        assertTrue(AffineParameters2D.isValidMatrix(m, threshold));

        // Force IllegalArgumentException when checking validity (negative
        // threshold)
        try {
            AffineParameters2D.isValidMatrix(m, -threshold);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final AffineParameters2D params = new AffineParameters2D();

        // set parameters from matrix
        params.fromMatrix(m);

        // check correctness
        assertEquals(scaleX, params.getScaleX(), 0.0);
        assertEquals(scaleY, params.getScaleY(), 0.0);
        assertEquals(skewness, params.getSkewness(), 0.0);

        // Force IllegalArgumentException (invalid size)
        final Matrix badM1 = new Matrix(INHOM_COORDS + 1, INHOM_COORDS + 1);
        try {
            params.fromMatrix(badM1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force IllegalArgumentException (non upper triangular matrix
        final Matrix badM2 = new Matrix(INHOM_COORDS, INHOM_COORDS);
        badM2.setElementAt(1, 0, 1.0);
        try {
            params.fromMatrix(badM2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Test from matrix with threshold
        params.fromMatrix(m, threshold);

        // check correctness
        assertEquals(scaleX, params.getScaleX(), 0.0);
        assertEquals(scaleY, params.getScaleY(), 0.0);
        assertEquals(skewness, params.getSkewness(), 0.0);

        // Force IllegalArgumentException (invalid size)
        try {
            params.fromMatrix(badM1, threshold);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force IllegalArgumentException (non upper triangular matrix
        badM2.setElementAt(1, 0, threshold + 1.0);
        try {
            params.fromMatrix(badM2, threshold);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force IllegalArgumentException (with negative threshold)
        try {
            params.fromMatrix(m, -threshold);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters2D params1 = new AffineParameters2D(
                scaleX, scaleY, skewness);

        // check
        assertEquals(scaleX, params1.getScaleX(), 0.0);
        assertEquals(scaleY, params1.getScaleY(), 0.0);
        assertEquals(skewness, params1.getSkewness(), 0.0);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(params1);
        final AffineParameters2D params2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(params1.getScaleX(), params2.getScaleX(), 0.0);
        assertEquals(params1.getScaleY(), params2.getScaleY(), 0.0);
        assertEquals(params1.getSkewness(), params2.getSkewness(), 0.0);
    }
}
