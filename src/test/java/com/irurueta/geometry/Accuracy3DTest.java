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

import com.irurueta.algebra.*;
import com.irurueta.statistics.NormalDist;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Arrays;
import java.util.Random;

import static org.junit.Assert.*;

public class Accuracy3DTest {

    private static final double MIN_ANGLE_DEGREES = 0.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;
    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;

    private static final int TIMES = 50;

    @Test
    public void testConstructor() throws AlgebraException, GeometryException {
        for (int t = 0; t < TIMES; t++) {
            // empty constructor
            Accuracy3D accuracy = new Accuracy3D();

            // check default values
            assertNull(accuracy.getCovarianceMatrix());
            assertEquals(accuracy.getStandardDeviationFactor(), 2.0, 0.0);
            assertEquals(accuracy.getConfidence(), 0.9544, 1e-2);
            assertEquals(accuracy.getConfidence(),
                    2.0 * NormalDist.cdf(2.0, 0.0, 1.0) - 1.0, 0.0);
            assertEquals(accuracy.getSmallestAccuracy(), Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getLargestAccuracy(), Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getAverageAccuracy(), Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getNumberOfDimensions(), 3);

            // constructor with covariance matrix
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
            double previous = 0.0;
            for (int i = Ellipsoid.DIMENSIONS - 1; i >= 0; i--) {
                semiAxesLengths[i] = previous + randomizer.nextDouble();
                previous = semiAxesLengths[i];
            }

            final double[] sqrSemiAxesLengths = new double[Ellipsoid.DIMENSIONS];
            for (int i = 0; i < Ellipsoid.DIMENSIONS; i++) {
                sqrSemiAxesLengths[i] = semiAxesLengths[i] * semiAxesLengths[i];
            }

            final double roll = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final Rotation3D rotation = new MatrixRotation3D(new Quaternion(roll, pitch, yaw));

            final Ellipsoid ellipsoid = new Ellipsoid(Point3D.create(), semiAxesLengths, rotation);

            final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
            final Matrix covarianceMatrix = rotationMatrix.multiplyAndReturnNew(
                    Matrix.diagonal(sqrSemiAxesLengths).multiplyAndReturnNew(rotationMatrix));

            accuracy = new Accuracy3D(covarianceMatrix);

            // check
            assertSame(accuracy.getCovarianceMatrix(), covarianceMatrix);
            assertEquals(accuracy.getStandardDeviationFactor(), 2.0, 0.0);
            assertEquals(accuracy.getConfidence(), 0.9544, 1e-2);
            assertEquals(accuracy.getConfidence(),
                    2.0 * NormalDist.cdf(2.0, 0.0, 1.0) - 1.0, 0.0);
            assertEquals(accuracy.getSmallestAccuracy(),
                    accuracy.getStandardDeviationFactor() * semiAxesLengths[2],
                    ABSOLUTE_ERROR);
            assertEquals(accuracy.getLargestAccuracy(),
                    accuracy.getStandardDeviationFactor() * semiAxesLengths[0],
                    ABSOLUTE_ERROR);
            assertEquals(accuracy.getAverageAccuracy(), accuracy.getStandardDeviationFactor() *
                            (semiAxesLengths[0] + semiAxesLengths[1] + semiAxesLengths[2]) / 3.0,
                    ABSOLUTE_ERROR);
            assertEquals(accuracy.getNumberOfDimensions(), 3);

            Ellipsoid ellipsoid2 = accuracy.toEllipsoid();
            assertEquals(ellipsoid.getCenter(), ellipsoid2.getCenter());
            assertArrayEquals(ArrayUtils.multiplyByScalarAndReturnNew(
                    ellipsoid.getSemiAxesLengths(), accuracy.getStandardDeviationFactor()),
                    ellipsoid2.getSemiAxesLengths(), ABSOLUTE_ERROR);
            assertTrue(ellipsoid.getRotation().equals(ellipsoid2.getRotation(), ABSOLUTE_ERROR));
            assertArrayEquals(ellipsoid2.getSemiAxesLengths(),
                    ArrayUtils.multiplyByScalarAndReturnNew(semiAxesLengths,
                            accuracy.getStandardDeviationFactor()), ABSOLUTE_ERROR);
            assertTrue(ellipsoid2.getRotation().equals(rotation, ABSOLUTE_ERROR));

            // force IllegalArgumentException
            accuracy = null;
            try {
                accuracy = new Accuracy3D(new Matrix(1, 1));
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            assertNull(accuracy);

            // force NonSymmetricPositiveDefiniteMatrixException
            final Matrix m = Matrix.diagonal(new double[]{
                    Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});

            try {
                accuracy = new Accuracy3D(m);
                fail("NonSymmetricPositiveDefiniteMatrixException expected but not thrown");
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
            }
            assertNull(accuracy);

            // constructor with confidence
            final double conf = randomizer.nextDouble(0.0, 1.0);
            accuracy = new Accuracy3D(conf);

            // check default values
            assertNull(accuracy.getCovarianceMatrix());
            assertTrue(accuracy.getStandardDeviationFactor() > 0.0);
            assertEquals(accuracy.getStandardDeviationFactor(),
                    NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0), 0.0);
            assertEquals(accuracy.getConfidence(), conf, 0.0);
            assertEquals(accuracy.getSmallestAccuracy(), Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getLargestAccuracy(), Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getAverageAccuracy(), Double.POSITIVE_INFINITY, 0.0);
            assertEquals(accuracy.getNumberOfDimensions(), 3);

            // force IllegalArgumentException
            accuracy = null;
            try {
                accuracy = new Accuracy3D(-1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            try {
                accuracy = new Accuracy3D(2.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            assertNull(accuracy);

            // constructor with covariance matrix and confidence
            accuracy = new Accuracy3D(covarianceMatrix, conf);

            // check
            assertSame(accuracy.getCovarianceMatrix(), covarianceMatrix);
            assertTrue(accuracy.getStandardDeviationFactor() > 0.0);
            assertEquals(accuracy.getStandardDeviationFactor(),
                    NormalDist.invcdf((conf + 1.0) / 2.0, 0.0, 1.0), 0.0);
            assertEquals(accuracy.getConfidence(), conf, 0.0);
            assertEquals(accuracy.getSmallestAccuracy(),
                    accuracy.getStandardDeviationFactor() * semiAxesLengths[2],
                    ABSOLUTE_ERROR);
            assertEquals(accuracy.getLargestAccuracy(),
                    accuracy.getStandardDeviationFactor() * semiAxesLengths[0],
                    ABSOLUTE_ERROR);
            assertEquals(accuracy.getAverageAccuracy(), accuracy.getStandardDeviationFactor() *
                            (semiAxesLengths[0] + semiAxesLengths[1] + semiAxesLengths[2]) / 3.0,
                    ABSOLUTE_ERROR);
            assertEquals(accuracy.getNumberOfDimensions(), 3);

            ellipsoid2 = accuracy.toEllipsoid();
            assertEquals(ellipsoid.getCenter(), ellipsoid2.getCenter());
            assertArrayEquals(ArrayUtils.multiplyByScalarAndReturnNew(
                    ellipsoid.getSemiAxesLengths(), accuracy.getStandardDeviationFactor()),
                    ellipsoid2.getSemiAxesLengths(), ABSOLUTE_ERROR);
            assertTrue(ellipsoid.getRotation().equals(ellipsoid2.getRotation(), ABSOLUTE_ERROR));
            assertArrayEquals(ellipsoid2.getSemiAxesLengths(),
                    ArrayUtils.multiplyByScalarAndReturnNew(semiAxesLengths,
                            accuracy.getStandardDeviationFactor()), ABSOLUTE_ERROR);
            assertTrue(ellipsoid2.getRotation().equals(rotation, ABSOLUTE_ERROR));

            // force IllegalArgumentException
            accuracy = null;
            try {
                accuracy = new Accuracy3D(new Matrix(1, 1), conf);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            try {
                accuracy = new Accuracy3D(covarianceMatrix, -1.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            try {
                accuracy = new Accuracy3D(covarianceMatrix, 2.0);
                fail("IllegalArgumentException expected but not thrown");
            } catch (final IllegalArgumentException ignore) {
            }
            assertNull(accuracy);

            // force NonSymmetricPositiveDefiniteMatrixException
            try {
                accuracy = new Accuracy3D(m, conf);
                fail("NonSymmetricPositiveDefiniteMatrixException expected but not thrown");
            } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
            }
            assertNull(accuracy);
        }
    }

    @Test
    public void testGetSetCovarianceMatrix() throws AlgebraException {
        final Accuracy3D accuracy = new Accuracy3D();

        // check default value
        assertNull(accuracy.getCovarianceMatrix());

        // set new value
        final Matrix covarianceMatrix = Matrix.identity(3, 3);
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
        final Matrix m = Matrix.diagonal(new double[]{
                Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});
        try {
            accuracy.setCovarianceMatrix(m);
            fail("NonSymmetricPositiveDefiniteMatrixException expected but not thrown");
        } catch (final NonSymmetricPositiveDefiniteMatrixException ignore) {
        }
    }

    @Test
    public void testGetSetStandardDeviationFactor() {
        final Accuracy3D accuracy = new Accuracy3D();

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
        final Accuracy3D accuracy = new Accuracy3D();

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
    public void testFlattenTo2D() throws AlgebraException, GeometryException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        double previous = 0.0;
        for (int i = Ellipsoid.DIMENSIONS - 1; i >= 0; i--) {
            semiAxesLengths[i] = previous + randomizer.nextDouble();
            previous = semiAxesLengths[i];
        }

        final double[] sqrSemiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        for (int i = 0; i < Ellipsoid.DIMENSIONS; i++) {
            sqrSemiAxesLengths[i] = semiAxesLengths[i] * semiAxesLengths[i];
        }

        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Rotation3D rotation = new MatrixRotation3D(new Quaternion(roll, pitch, yaw));

        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
        final Matrix covarianceMatrix = rotationMatrix.multiplyAndReturnNew(
                Matrix.diagonal(sqrSemiAxesLengths).multiplyAndReturnNew(rotationMatrix));

        final Accuracy3D accuracy = new Accuracy3D(covarianceMatrix);

        final Accuracy2D flattenedAccuracy = accuracy.flattenTo2D();

        final Ellipse ellipse = accuracy.intersectWithPlane();
        final Ellipse flattenedEllipse = flattenedAccuracy.toEllipse();

        assertEquals(ellipse.getSemiMajorAxis(),
                flattenedEllipse.getSemiMajorAxis(), ABSOLUTE_ERROR);
        assertEquals(ellipse.getSemiMinorAxis(),
                flattenedEllipse.getSemiMinorAxis(), ABSOLUTE_ERROR);
        // because ellipses are symmetric, there is a rotation ambiguity
        assertTrue(Math.abs(ellipse.getRotationAngle() - flattenedEllipse.getRotationAngle()) <= ABSOLUTE_ERROR ||
                Math.abs(Math.abs(ellipse.getRotationAngle() - flattenedEllipse.getRotationAngle()) - Math.PI) <= ABSOLUTE_ERROR);

        assertEquals(ellipse.getCenter(), Point2D.create());
        assertEquals(flattenedEllipse.getCenter(), Point2D.create());
    }

    @Test
    public void testIntersectWithPlaneSphere() throws AlgebraException, GeometryException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double radius = Math.abs(randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE));
        final double[] sqrSemiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        Arrays.fill(sqrSemiAxesLengths, radius * radius);

        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Rotation3D rotation = new MatrixRotation3D(new Quaternion(roll, pitch, yaw));

        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
        final Matrix covarianceMatrix = rotationMatrix.multiplyAndReturnNew(
                Matrix.diagonal(sqrSemiAxesLengths).multiplyAndReturnNew(rotationMatrix));

        final Accuracy3D accuracy = new Accuracy3D(covarianceMatrix);

        final Ellipse ellipse = accuracy.intersectWithPlane();

        // check
        assertEquals(ellipse.getCenter(), Point2D.create());
        assertEquals(ellipse.getSemiMajorAxis(),
                radius * accuracy.getStandardDeviationFactor(), ABSOLUTE_ERROR);
        assertEquals(ellipse.getSemiMinorAxis(),
                radius * accuracy.getStandardDeviationFactor(), ABSOLUTE_ERROR);
    }

    @Test
    public void testIntersectWithPlaneEllipsoid() throws AlgebraException, GeometryException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        double previous = 0.0;
        for (int i = Ellipsoid.DIMENSIONS - 1; i >= 0; i--) {
            semiAxesLengths[i] = previous + randomizer.nextDouble();
            previous = semiAxesLengths[i];
        }

        final double[] sqrSemiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        for (int i = 0; i < Ellipsoid.DIMENSIONS; i++) {
            sqrSemiAxesLengths[i] = semiAxesLengths[i] * semiAxesLengths[i];
        }


        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Rotation3D rotation = new MatrixRotation3D(new Quaternion(roll, pitch, yaw));

        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
        final Matrix covarianceMatrix = rotationMatrix.multiplyAndReturnNew(
                Matrix.diagonal(sqrSemiAxesLengths).multiplyAndReturnNew(rotationMatrix));

        final Accuracy3D accuracy = new Accuracy3D(covarianceMatrix);
        accuracy.setStandardDeviationFactor(1.0);

        final Ellipse ellipse = accuracy.intersectWithPlane();

        // check
        assertEquals(ellipse.getCenter(), Point2D.create());
        assertTrue(ellipse.getSemiMajorAxis() <= semiAxesLengths[0] &&
                ellipse.getSemiMajorAxis() >= semiAxesLengths[2]);
        assertTrue(ellipse.getSemiMinorAxis() <= semiAxesLengths[0] &&
                ellipse.getSemiMinorAxis() >= semiAxesLengths[2]);
        assertTrue(ellipse.getSemiMajorAxis() >= ellipse.getSemiMinorAxis());
    }

    @Test
    public void testIntersectWithPlaneEllipsoidOnlyZAxisRotation() throws AlgebraException, GeometryException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        double previous = 0.0;
        for (int i = Ellipsoid.DIMENSIONS - 1; i >= 0; i--) {
            semiAxesLengths[i] = previous + randomizer.nextDouble();
            previous = semiAxesLengths[i];
        }

        final double[] sqrSemiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        for (int i = 0; i < Ellipsoid.DIMENSIONS; i++) {
            sqrSemiAxesLengths[i] = semiAxesLengths[i] * semiAxesLengths[i];
        }

        final double angle = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double[] axis = new double[]{0.0, 0.0, 1.0};
        final AxisRotation3D rotation = new AxisRotation3D(axis, angle);

        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
        final Matrix covarianceMatrix = rotationMatrix.multiplyAndReturnNew(
                Matrix.diagonal(sqrSemiAxesLengths).multiplyAndReturnNew(rotationMatrix));

        final Accuracy3D accuracy = new Accuracy3D(covarianceMatrix);
        accuracy.setStandardDeviationFactor(1.0);

        final Ellipse ellipse = accuracy.intersectWithPlane();

        // check
        assertEquals(ellipse.getCenter(), Point2D.create());
        assertEquals(ellipse.getSemiMajorAxis(), semiAxesLengths[0], ABSOLUTE_ERROR);
        assertEquals(ellipse.getSemiMinorAxis(), semiAxesLengths[1], ABSOLUTE_ERROR);

        // because ellipses are symmetric, there is a rotation ambiguity
        assertTrue(Math.abs(ellipse.getRotationAngle() - angle) <= ABSOLUTE_ERROR ||
                Math.abs(Math.abs(ellipse.getRotationAngle() - angle) - Math.PI) <= ABSOLUTE_ERROR);
    }

    @Test
    public void testSerializeDeserialize() throws WrongSizeException,
            NonSymmetricPositiveDefiniteMatrixException, IOException, ClassNotFoundException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] semiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        double previous = 0.0;
        for (int i = Ellipsoid.DIMENSIONS - 1; i >= 0; i--) {
            semiAxesLengths[i] = previous + randomizer.nextDouble();
            previous = semiAxesLengths[i];
        }

        final double[] sqrSemiAxesLengths = new double[Ellipsoid.DIMENSIONS];
        for (int i = 0; i < Ellipsoid.DIMENSIONS; i++) {
            sqrSemiAxesLengths[i] = semiAxesLengths[i] * semiAxesLengths[i];
        }

        final double roll = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Utils.convertToRadians(randomizer.nextDouble(
                MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Rotation3D rotation = new MatrixRotation3D(new Quaternion(roll, pitch, yaw));

        final Matrix rotationMatrix = rotation.asInhomogeneousMatrix();
        final Matrix covarianceMatrix = rotationMatrix.multiplyAndReturnNew(
                Matrix.diagonal(sqrSemiAxesLengths).multiplyAndReturnNew(rotationMatrix));
        final double conf = randomizer.nextDouble(0.0, 1.0);

        final Accuracy3D accuracy1 = new Accuracy3D(covarianceMatrix, conf);

        // check
        assertSame(accuracy1.getCovarianceMatrix(), covarianceMatrix);
        assertTrue(accuracy1.getStandardDeviationFactor() > 0.0);
        assertEquals(accuracy1.getConfidence(), conf, 0.0);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(accuracy1);
        final Accuracy3D accuracy2 = SerializationHelper.deserialize(bytes);

        // check
        assertEquals(accuracy1.getCovarianceMatrix(),
                accuracy2.getCovarianceMatrix());
        assertEquals(accuracy1.getStandardDeviationFactor(),
                accuracy2.getStandardDeviationFactor(), 0.0);
        assertEquals(accuracy1.getConfidence(),
                accuracy2.getConfidence(), 0.0);
    }
}
