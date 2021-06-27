/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.NormalDist;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class Accuracy2DTest {

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;
    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    public void testConstructor() throws AlgebraException, GeometryException {
        // empty constructor
        Accuracy2D accuracy = new Accuracy2D();

        // check default values
        assertNull(accuracy.getCovarianceMatrix());
        assertEquals(accuracy.getStandardDeviationFactor(), 2.0, 0.0);
        assertEquals(accuracy.getConfidence(), 0.9544, 1e-2);
        assertEquals(accuracy.getConfidence(),
                2.0 * NormalDist.cdf(2.0, 0.0, 1.0) - 1.0, 0.0);
        assertEquals(accuracy.getSmallestAccuracy(), Double.POSITIVE_INFINITY, 0.0);
        assertEquals(accuracy.getLargestAccuracy(), Double.POSITIVE_INFINITY, 0.0);
        assertEquals(accuracy.getAverageAccuracy(), Double.POSITIVE_INFINITY, 0.0);
        assertEquals(accuracy.getNumberOfDimensions(), 2);


        // constructor with covariance matrix
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double semiMinorAxis = randomizer.nextDouble();
        final double semiMajorAxis = semiMinorAxis + randomizer.nextDouble();
        final double rotationAngle = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Ellipse ellipse = new Ellipse(Point2D.create(), semiMajorAxis, semiMinorAxis,
                rotationAngle);

        final Rotation2D rotation2D = new Rotation2D(rotationAngle);
        final Matrix rotationMatrix = rotation2D.asInhomogeneousMatrix();
        final Matrix covarianceMatrix = rotationMatrix.multiplyAndReturnNew(
                Matrix.diagonal(new double[]{
                        semiMajorAxis * semiMajorAxis,
                        semiMinorAxis * semiMinorAxis}).
                        multiplyAndReturnNew(rotationMatrix));

        accuracy = new Accuracy2D(covarianceMatrix);

        // check
        assertSame(accuracy.getCovarianceMatrix(), covarianceMatrix);
        assertEquals(accuracy.getStandardDeviationFactor(), 2.0, 0.0);
        assertEquals(accuracy.getConfidence(), 0.9544, 1e-2);
        assertEquals(accuracy.getConfidence(),
                2.0 * NormalDist.cdf(2.0, 0.0, 1.0) - 1.0, 0.0);
        assertEquals(accuracy.getSmallestAccuracy(),
                accuracy.getStandardDeviationFactor() * semiMinorAxis, ABSOLUTE_ERROR);
        assertEquals(accuracy.getLargestAccuracy(),
                accuracy.getStandardDeviationFactor() * semiMajorAxis, ABSOLUTE_ERROR);
        assertEquals(accuracy.getAverageAccuracy(),
                0.5 * (accuracy.getSmallestAccuracy() + accuracy.getLargestAccuracy()),
                ABSOLUTE_ERROR);
        assertEquals(accuracy.getNumberOfDimensions(), 2);

        Ellipse ellipse2 = accuracy.toEllipse();
        assertEquals(ellipse.getCenter(), ellipse2.getCenter());
        assertEquals(ellipse.getRotationAngle(), ellipse2.getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(ellipse2.getSemiMajorAxis(),
                semiMajorAxis * accuracy.getStandardDeviationFactor(), ABSOLUTE_ERROR);
        assertEquals(ellipse2.getSemiMinorAxis(),
                semiMinorAxis * accuracy.getStandardDeviationFactor(), ABSOLUTE_ERROR);
        assertEquals(ellipse2.getRotationAngle(), rotationAngle, ABSOLUTE_ERROR);


        // force IllegalArgumentException
        accuracy = null;
        try {
            accuracy = new Accuracy2D(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(accuracy);

        // force NonSymmetricPositiveDefiniteMatrixException
        final Matrix m = Matrix.diagonal(new double[]{Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});

        try {
            accuracy = new Accuracy2D(m);
            fail("NonSymmetricPositiveDefiniteMatrixException expected but not thrown");
        } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
        }
        assertNull(accuracy);


        // constructor with confidence
        final double conf = randomizer.nextDouble(0.0, 1.0);
        accuracy = new Accuracy2D(conf);

        // check default values
        assertNull(accuracy.getCovarianceMatrix());
        assertTrue(accuracy.getStandardDeviationFactor() > 0.0);
        assertEquals(accuracy.getStandardDeviationFactor(),
                NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0), 0.0);
        assertEquals(accuracy.getConfidence(), conf, 0.0);
        assertEquals(accuracy.getSmallestAccuracy(), Double.POSITIVE_INFINITY, 0.0);
        assertEquals(accuracy.getLargestAccuracy(), Double.POSITIVE_INFINITY, 0.0);
        assertEquals(accuracy.getAverageAccuracy(), Double.POSITIVE_INFINITY, 0.0);
        assertEquals(accuracy.getNumberOfDimensions(), 2);

        // force IllegalArgumentException
        accuracy = null;
        try {
            accuracy = new Accuracy2D(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            accuracy = new Accuracy2D(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(accuracy);


        // constructor with covariance matrix and confidence
        accuracy = new Accuracy2D(covarianceMatrix, conf);

        // check
        assertSame(accuracy.getCovarianceMatrix(), covarianceMatrix);
        assertTrue(accuracy.getStandardDeviationFactor() > 0.0);
        assertEquals(accuracy.getStandardDeviationFactor(),
                NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0), 0.0);
        assertEquals(accuracy.getConfidence(), conf, 0.0);
        assertEquals(accuracy.getSmallestAccuracy(),
                accuracy.getStandardDeviationFactor() * semiMinorAxis, ABSOLUTE_ERROR);
        assertEquals(accuracy.getLargestAccuracy(),
                accuracy.getStandardDeviationFactor() * semiMajorAxis, ABSOLUTE_ERROR);
        assertEquals(accuracy.getAverageAccuracy(),
                0.5 * (accuracy.getSmallestAccuracy() + accuracy.getLargestAccuracy()),
                ABSOLUTE_ERROR);
        assertEquals(accuracy.getNumberOfDimensions(), 2);

        ellipse2 = accuracy.toEllipse();
        assertEquals(ellipse.getCenter(), ellipse2.getCenter());
        assertEquals(ellipse.getRotationAngle(), ellipse2.getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(ellipse2.getSemiMajorAxis(),
                semiMajorAxis * accuracy.getStandardDeviationFactor(), ABSOLUTE_ERROR);
        assertEquals(ellipse2.getSemiMinorAxis(),
                semiMinorAxis * accuracy.getStandardDeviationFactor(), ABSOLUTE_ERROR);
        assertEquals(ellipse2.getRotationAngle(), rotationAngle, ABSOLUTE_ERROR);


        // force IllegalArgumentException
        accuracy = null;
        try {
            accuracy = new Accuracy2D(new Matrix(1, 1), conf);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            accuracy = new Accuracy2D(covarianceMatrix, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            accuracy = new Accuracy2D(covarianceMatrix, 2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(accuracy);

        // force NonSymmetricPositiveDefiniteMatrixException
        try {
            accuracy = new Accuracy2D(m, conf);
            fail("NonSymmetricPositiveDefiniteMatrixException expected but not thrown");
        } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
        }
        assertNull(accuracy);
    }

    @Test
    public void testGetSetCovarianceMatrix() throws AlgebraException {
        final Accuracy2D accuracy = new Accuracy2D();

        // check default value
        assertNull(accuracy.getCovarianceMatrix());

        // set new value
        final Matrix covarianceMatrix = Matrix.identity(2, 2);
        accuracy.setCovarianceMatrix(covarianceMatrix);

        // check
        assertSame(accuracy.getCovarianceMatrix(), covarianceMatrix);

        // force IllegalArgumentException
        try {
            accuracy.setCovarianceMatrix(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // force NonSymmetricPositiveDefiniteMatrixException
        final Matrix m = Matrix.diagonal(new double[]{Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});
        try {
            accuracy.setCovarianceMatrix(m);
            fail("NonSymmetricPositiveDefiniteMatrixException expected but not thrown");
        } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
        }
    }

    @Test
    public void testGetSetStandardDeviationFactor() {
        final Accuracy2D accuracy = new Accuracy2D();

        // check default value
        assertEquals(accuracy.getStandardDeviationFactor(), 2.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double factor = randomizer.nextDouble(10.0);
        accuracy.setStandardDeviationFactor(factor);

        // check
        assertEquals(accuracy.getStandardDeviationFactor(), factor, 0.0);
        assertEquals(accuracy.getConfidence(),
                2.0 * NormalDist.cdf(factor, 0.0, 1.0) - 1.0, 0.0);

        // force IllegalArgumentException
        try {
            accuracy.setStandardDeviationFactor(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() {
        final Accuracy2D accuracy = new Accuracy2D();

        // check default value
        assertEquals(accuracy.getConfidence(), 0.9544, 1e-2);
        assertEquals(accuracy.getConfidence(),
                2.0 * NormalDist.cdf(2.0, 0.0, 1.0) - 1.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double conf = randomizer.nextDouble();
        accuracy.setConfidence(conf);

        // check
        assertEquals(accuracy.getConfidence(), conf, 0.0);
        assertEquals(accuracy.getStandardDeviationFactor(),
                NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0), 0.0);

        // force IllegalArgumentException
        try {
            accuracy.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            accuracy.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException,
            NonSymmetricPositiveDefiniteMatrixException, IOException,
            ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double semiMinorAxis = randomizer.nextDouble();
        final double semiMajorAxis = semiMinorAxis + randomizer.nextDouble();
        final double rotationAngle = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final Rotation2D rotation2D = new Rotation2D(rotationAngle);
        final Matrix rotationMatrix = rotation2D.asInhomogeneousMatrix();
        final Matrix covarianceMatrix = rotationMatrix.multiplyAndReturnNew(
                Matrix.diagonal(new double[]{
                        semiMajorAxis * semiMajorAxis,
                        semiMinorAxis * semiMinorAxis}).
                        multiplyAndReturnNew(rotationMatrix));
        final double conf = randomizer.nextDouble(0.0, 1.0);

        final Accuracy2D accuracy1 = new Accuracy2D(covarianceMatrix, conf);

        // check
        assertSame(accuracy1.getCovarianceMatrix(), covarianceMatrix);
        assertTrue(accuracy1.getStandardDeviationFactor() > 0.0);
        assertEquals(accuracy1.getConfidence(), conf, 0.0);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(accuracy1);
        final Accuracy2D accuracy2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(accuracy1.getCovarianceMatrix(),
                accuracy2.getCovarianceMatrix());
        assertEquals(accuracy1.getStandardDeviationFactor(),
                accuracy2.getStandardDeviationFactor(), 0.0);
        assertEquals(accuracy1.getConfidence(),
                accuracy2.getConfidence(), 0.0);
    }
}
