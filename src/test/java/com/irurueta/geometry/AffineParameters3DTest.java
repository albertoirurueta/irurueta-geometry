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

public class AffineParameters3DTest {

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final int INHOM_COORDS = 3;

    private static final double ABSOLUTE_ERROR = 1e-8;

    @Test
    public void testConstructors() throws WrongSizeException {
        // Test empty constructor
        AffineParameters3D params = new AffineParameters3D();

        // check correctness
        assertEquals(AffineParameters3D.DEFAULT_SCALE, params.getScaleX(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, params.getScaleY(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, params.getScaleZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, params.getSkewnessXY(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, params.getSkewnessXZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, params.getSkewnessYZ(), 0.0);

        // Test constructor with scale
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        params = new AffineParameters3D(scale);

        // check correctness
        assertEquals(scale, params.getScaleX(), 0.0);
        assertEquals(scale, params.getScaleY(), 0.0);
        assertEquals(scale, params.getScaleZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, params.getSkewnessXY(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, params.getSkewnessXZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, params.getSkewnessYZ(), 0.0);

        // Test constructor with scale and skewness
        final double skewness = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        params = new AffineParameters3D(scale, skewness);

        // check correctness
        assertEquals(scale, params.getScaleX(), 0.0);
        assertEquals(scale, params.getScaleY(), 0.0);
        assertEquals(scale, params.getScaleZ(), 0.0);
        assertEquals(skewness, params.getSkewnessXY(), 0.0);
        assertEquals(skewness, params.getSkewnessXZ(), 0.0);
        assertEquals(skewness, params.getSkewnessYZ(), 0.0);

        // Test constructor with scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ
        // and skewnessYZ
        final double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        params = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY,
                skewnessXZ, skewnessYZ);

        // check correctness
        assertEquals(scaleX, params.getScaleX(), 0.0);
        assertEquals(scaleY, params.getScaleY(), 0.0);
        assertEquals(scaleZ, params.getScaleZ(), 0.0);
        assertEquals(skewnessXY, params.getSkewnessXY(), 0.0);
        assertEquals(skewnessXZ, params.getSkewnessXZ(), 0.0);
        assertEquals(skewnessYZ, params.getSkewnessYZ(), 0.0);

        // test constructor with matrix
        final Matrix m = new Matrix(INHOM_COORDS, INHOM_COORDS);
        m.setElementAt(0, 0, scaleX);
        m.setElementAt(1, 1, scaleY);
        m.setElementAt(2, 2, scaleZ);
        m.setElementAt(0, 1, skewnessXY);
        m.setElementAt(0, 2, skewnessXZ);
        m.setElementAt(1, 2, skewnessYZ);

        params = new AffineParameters3D(m);

        // check correctness
        assertEquals(scaleX, params.getScaleX(), 0.0);
        assertEquals(scaleY, params.getScaleY(), 0.0);
        assertEquals(scaleZ, params.getScaleZ(),  0.0);
        assertEquals(skewnessXY, params.getSkewnessXY(), 0.0);
        assertEquals(skewnessXZ, params.getSkewnessXZ(), 0.0);
        assertEquals(skewnessYZ, params.getSkewnessYZ(), 0.0);

        // Force IllegalArgumentException (invalid size)
        final Matrix badM1 = new Matrix(INHOM_COORDS + 1, INHOM_COORDS + 1);
        params = null;
        try {
            params = new AffineParameters3D(badM1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force IllegalArgumentException (non upper triangular matrix
        final Matrix badM2 = new Matrix(INHOM_COORDS, INHOM_COORDS);
        badM2.setElementAt(1, 0, 1.0);
        try {
            params = new AffineParameters3D(badM2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(params);

        // Test constructor with matrix and threshold
        final double threshold = randomizer.nextDouble();
        params = new AffineParameters3D(m, threshold);

        // check correctness
        assertEquals(scaleX, params.getScaleX(), 0.0);
        assertEquals(scaleY, params.getScaleY(), 0.0);
        assertEquals(scaleZ, params.getScaleZ(), 0.0);
        assertEquals(skewnessXY, params.getSkewnessXY(), 0.0);
        assertEquals(skewnessXZ, params.getSkewnessXZ(), 0.0);
        assertEquals(skewnessYZ, params.getSkewnessYZ(), 0.0);

        // Force IllegalArgumentException (invalid size)
        params = null;
        try {
            params = new AffineParameters3D(badM1, threshold);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force IllegalArgumentException (non upper triangular matrix)
        badM2.setElementAt(1, 0, threshold + 1.0);
        try {
            params = new AffineParameters3D(badM2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force IllegalArgumentException (with negative threshold)
        try {
            params = new AffineParameters3D(m, -threshold);
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

        final AffineParameters3D params = new AffineParameters3D();

        // check default values
        assertEquals(AffineParameters3D.DEFAULT_SCALE, params.getScaleX(), 0.0);

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

        final AffineParameters3D params = new AffineParameters3D();

        // check default values
        assertEquals(AffineParameters3D.DEFAULT_SCALE, params.getScaleY(), 0.0);

        // set new value
        params.setScaleY(scaleY);

        // check correctness
        assertEquals(scaleY, params.getScaleY(), 0.0);
    }

    @Test
    public void testGetSetScaleZ() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters3D params = new AffineParameters3D();

        // check default values
        assertEquals(AffineParameters3D.DEFAULT_SCALE, params.getScaleZ(), 0.0);

        // set new value
        params.setScaleZ(scaleZ);

        // check correctness
        assertEquals(scaleZ, params.getScaleZ(), 0.0);
    }

    @Test
    public void testSetScale() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters3D params = new AffineParameters3D();

        // check default values
        assertEquals(AffineParameters3D.DEFAULT_SCALE, params.getScaleX(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, params.getScaleY(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, params.getScaleZ(), 0.0);

        // set new value
        params.setScale(scale);

        // check correctness
        assertEquals(scale, params.getScaleX(), 0.0);
        assertEquals(scale, params.getScaleY(), 0.0);
        assertEquals(scale, params.getScaleZ(), 0.0);
    }

    @Test
    public void testGetSetSkewnessXY() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters3D params = new AffineParameters3D();

        // check default values
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS,
                params.getSkewnessXY(), 0.0);

        // set new value
        params.setSkewnessXY(skewnessXY);

        // check correctness
        assertEquals(skewnessXY, params.getSkewnessXY(), 0.0);
    }

    @Test
    public void testGetSetSkewnessXZ() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters3D params = new AffineParameters3D();

        // check default values
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS,
                params.getSkewnessXZ(), 0.0);

        // set new value
        params.setSkewnessXZ(skewnessXZ);

        // check correctness
        assertEquals(skewnessXZ, params.getSkewnessXZ(), 0.0);
    }

    @Test
    public void testGetSetSkewnessYZ() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters3D params = new AffineParameters3D();

        // check default values
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS,
                params.getSkewnessYZ(), 0.0);

        // set new value
        params.setSkewnessYZ(skewnessYZ);

        // check correctness
        assertEquals(skewnessYZ, params.getSkewnessYZ(), 0.0);
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters3D params = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final Matrix expectedMatrix = new Matrix(INHOM_COORDS, INHOM_COORDS);
        expectedMatrix.initialize(0.0);
        expectedMatrix.setElementAt(0, 0, scaleX);
        expectedMatrix.setElementAt(1, 1, scaleY);
        expectedMatrix.setElementAt(2, 2, scaleZ);
        expectedMatrix.setElementAt(0, 1, skewnessXY);
        expectedMatrix.setElementAt(0, 2, skewnessXZ);
        expectedMatrix.setElementAt(1, 2, skewnessYZ);

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
        final double scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double threshold = randomizer.nextDouble();

        // build valid matrix
        final Matrix m = new Matrix(INHOM_COORDS, INHOM_COORDS);
        m.setElementAt(0, 0, scaleX);
        m.setElementAt(1, 1, scaleY);
        m.setElementAt(2, 2, scaleZ);
        m.setElementAt(0, 1, skewnessXY);
        m.setElementAt(0, 2, skewnessXZ);
        m.setElementAt(1, 2, skewnessYZ);

        // check matrix is valid
        assertTrue(AffineParameters3D.isValidMatrix(m));
        assertTrue(AffineParameters3D.isValidMatrix(m, threshold));

        // Force IllegalArgumentException when checking validity (negative
        // threshold)
        try {
            AffineParameters3D.isValidMatrix(m, -threshold);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        final AffineParameters3D params = new AffineParameters3D();

        // set parameters from matrix
        params.fromMatrix(m);

        // check correctness
        assertEquals(scaleX, params.getScaleX(), 0.0);
        assertEquals(scaleY, params.getScaleY(), 0.0);
        assertEquals(scaleZ, params.getScaleZ(), 0.0);
        assertEquals(skewnessXY, params.getSkewnessXY(), 0.0);
        assertEquals(skewnessXZ, params.getSkewnessXZ(), 0.0);
        assertEquals(skewnessYZ, params.getSkewnessYZ(), 0.0);

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
        assertEquals(scaleZ, params.getScaleZ(), 0.0);
        assertEquals(skewnessXY, params.getSkewnessXY(), 0.0);
        assertEquals(skewnessXZ, params.getSkewnessXZ(), 0.0);
        assertEquals(skewnessYZ, params.getSkewnessYZ(), 0.0);

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
        final double scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters3D params1 = new AffineParameters3D(
                scaleX, scaleY, scaleZ,
                skewnessXY, skewnessXZ, skewnessYZ);

        // check
        assertEquals(scaleX, params1.getScaleX(), 0.0);
        assertEquals(scaleY, params1.getScaleY(), 0.0);
        assertEquals(scaleZ, params1.getScaleZ(), 0.0);
        assertEquals(skewnessXY, params1.getSkewnessXY(), 0.0);
        assertEquals(skewnessXZ, params1.getSkewnessXZ(), 0.0);
        assertEquals(skewnessYZ, params1.getSkewnessYZ(), 0.0);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(params1);
        final AffineParameters3D params2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(params1.getScaleX(), params2.getScaleX(), 0.0);
        assertEquals(params1.getScaleY(), params2.getScaleY(), 0.0);
        assertEquals(params1.getScaleZ(), params2.getScaleZ(), 0.0);
        assertEquals(params1.getSkewnessXY(), params2.getSkewnessXY(), 0.0);
        assertEquals(params1.getSkewnessXZ(), params2.getSkewnessXZ(), 0.0);
        assertEquals(params1.getSkewnessYZ(), params2.getSkewnessYZ(), 0.0);
    }
}
