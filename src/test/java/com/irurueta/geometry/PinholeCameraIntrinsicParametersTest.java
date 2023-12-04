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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.Random;

import static org.junit.Assert.*;

public class PinholeCameraIntrinsicParametersTest {

    private static final int INTRINSIC_ROWS = 3;
    private static final int INTRINSIC_COLS = 3;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_FOCAL_LENGTH = 1.0;
    private static final double MAX_FOCAL_LENGTH = 100.0;

    private static final double MIN_SKEWNESS = -1.0;
    private static final double MAX_SKEWNESS = 1.0;

    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_ASPECT_RATIO = 0.5;
    private static final double MAX_ASPECT_RATIO = 2.0;

    private static final int MIN_IMAGE_WIDTH = 640;
    private static final int MAX_IMAGE_WIDTH = 1024;

    private static final int MIN_IMAGE_HEIGHT = 480;
    private static final int MAX_IMAGE_HEIGHT = 768;

    @Test
    public void testConstants() {
        assertEquals(3, PinholeCameraIntrinsicParameters.INTRINSIC_MATRIX_ROWS);
        assertEquals(3, PinholeCameraIntrinsicParameters.INTRINSIC_MATRIX_COLS);
        assertEquals(1e-12, PinholeCameraIntrinsicParameters.DEFAULT_VALID_THRESHOLD, 0.0);
    }

    @Test
    public void testConstructor() throws WrongSizeException,
            InvalidPinholeCameraIntrinsicParametersException {

        PinholeCameraIntrinsicParameters k = new PinholeCameraIntrinsicParameters();

        // check that internal matrix is the identity, which corresponds to
        // canonical intrinsic parameters
        assertTrue(k.getInternalMatrix().equals(Matrix.identity(INTRINSIC_ROWS,
                INTRINSIC_COLS), ABSOLUTE_ERROR));

        // Test constructor providing an internal matrix

        // Build a random upper triangular matrix
        final Matrix triangularMatrix = Matrix.createWithUniformRandomValues(
                INTRINSIC_ROWS, INTRINSIC_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        for (int u = 0; u < INTRINSIC_ROWS; u++) {
            for (int v = 0; v < u; v++) {
                triangularMatrix.setElementAt(u, v, 0.0);
            }
        }

        // Use transposed upper triangular matrix (which is lower triangular) to
        // build any random non-triangular matrix
        final Matrix transTriangularMatrix = triangularMatrix.transposeAndReturnNew();

        Matrix kMatrix = transTriangularMatrix.multiplyAndReturnNew(
                triangularMatrix);

        // Force InvalidPinholeCameraIntrinsicParametersException when matrix is
        // not upper triangular
        k = null;
        try {
            k = new PinholeCameraIntrinsicParameters(kMatrix);
            fail("InvalidPinholeCameraIntrinsicParametersException expected but not thrown");
        } catch (final InvalidPinholeCameraIntrinsicParametersException ignore) {
        }
        try {
            k = new PinholeCameraIntrinsicParameters(kMatrix, ABSOLUTE_ERROR);
            fail("InvalidPinholeCameraIntrinsicParametersException expected but not thrown");
        } catch (final InvalidPinholeCameraIntrinsicParametersException ignore) {
        }
        // Force IllegalArgumentException
        try {
            k = new PinholeCameraIntrinsicParameters(triangularMatrix,
                    -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(k);

        // Use upper-triangular matrix as KMatrix but set last element to a value
        // different of zero. After calling constructor check that provided
        // matrix has been normalized and that value is now 1.0
        kMatrix = triangularMatrix;
        triangularMatrix.setElementAt(2, 2, 2.0);
        k = new PinholeCameraIntrinsicParameters(kMatrix);

        // check correctness of internal matrix
        assertTrue(k.getInternalMatrix().equals(triangularMatrix, ABSOLUTE_ERROR));
        assertEquals(1.0, triangularMatrix.getElementAt(2, 2), ABSOLUTE_ERROR);

        // Test constructor with parameters
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double aspectRatio = verticalFocalLength / horizontalFocalLength;

        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double skewnessAngle = Math.atan2(skewness, verticalFocalLength);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        k = new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint,
                verticalPrincipalPoint, skewness);
        assertEquals(horizontalFocalLength, k.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(verticalFocalLength, k.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(aspectRatio, k.getAspectRatio(), ABSOLUTE_ERROR);
        assertEquals(horizontalPrincipalPoint, k.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(verticalPrincipalPoint, k.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(skewness, k.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(skewnessAngle, k.getSkewnessAngle(), ABSOLUTE_ERROR);

        final Matrix kMatrix2 = new Matrix(INTRINSIC_ROWS, INTRINSIC_COLS);
        kMatrix2.setElementAt(0, 0, horizontalFocalLength);
        kMatrix2.setElementAt(1, 1, verticalFocalLength);
        kMatrix2.setElementAt(2, 2, 1.0);
        kMatrix2.setElementAt(0, 1, skewness);
        kMatrix2.setElementAt(0, 2, horizontalPrincipalPoint);
        kMatrix2.setElementAt(1, 2, verticalPrincipalPoint);

        assertTrue(k.getInternalMatrix().equals(kMatrix2, ABSOLUTE_ERROR));

        // Test copy constructor
        k = new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                verticalFocalLength, horizontalPrincipalPoint,
                verticalPrincipalPoint, skewness);
        final PinholeCameraIntrinsicParameters k2 =
                new PinholeCameraIntrinsicParameters(k);
        assertTrue(k.getInternalMatrix().equals(k2.getInternalMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetInternalMatrix() throws WrongSizeException,
            InvalidPinholeCameraIntrinsicParametersException {

        // Build a random upper triangular matrix
        final Matrix triangularMatrix = Matrix.createWithUniformRandomValues(
                INTRINSIC_ROWS, INTRINSIC_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        for (int u = 0; u < INTRINSIC_ROWS; u++) {
            for (int v = 0; v < u; v++) {
                triangularMatrix.setElementAt(u, v, 0.0);
            }
        }

        // Use transposed upper triangular matrix (which is lower triangular) to
        // build any random non-triangular matrix
        final Matrix transTriangularMatrix = triangularMatrix.transposeAndReturnNew();

        Matrix kMatrix = transTriangularMatrix.multiplyAndReturnNew(
                triangularMatrix);

        final PinholeCameraIntrinsicParameters k = new PinholeCameraIntrinsicParameters();

        // Force InvalidPinholeCameraIntrinsicParametersException when matrix is
        // not upper triangular
        try {
            k.setInternalMatrix(kMatrix);
            fail("InvalidPinholeCameraIntrinsicParametersException expected but not thrown");
        } catch (final InvalidPinholeCameraIntrinsicParametersException ignore) {
        }
        try {
            k.setInternalMatrix(kMatrix, ABSOLUTE_ERROR);
            fail("InvalidPinholeCameraIntrinsicParametersException expected but not thrown");
        } catch (final InvalidPinholeCameraIntrinsicParametersException ignore) {
        }
        // Force IllegalArgumentException
        try {
            k.setInternalMatrix(triangularMatrix, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Use upper-triangular matrix as KMatrix
        kMatrix = triangularMatrix;
        // Force InvalidPinholeCameraIntrinsicParametersException because
        // lower-right element is not 1.0 and normalization is disabled
        triangularMatrix.setElementAt(2, 2, 2.0);

        k.setInternalMatrix(kMatrix);

        // check that now lower-right element of KMatrix is 1.0 (because it has
        // been normalized)
        assertEquals(1.0, triangularMatrix.getElementAt(2, 2), ABSOLUTE_ERROR);

        assertTrue(kMatrix.equals(k.getInternalMatrix(), ABSOLUTE_ERROR));

        // try again with a threshold
        k.setInternalMatrix(kMatrix, ABSOLUTE_ERROR);
        assertTrue(kMatrix.equals(k.getInternalMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    public void testGetInverseInternalMatrix() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        final Matrix k = intrinsic.getInternalMatrix();
        final Matrix invK = com.irurueta.algebra.Utils.inverse(k);

        final Matrix invK2 = intrinsic.getInverseInternalMatrix();
        final Matrix invK3 = new Matrix(INTRINSIC_ROWS, INTRINSIC_COLS);
        intrinsic.getInverseInternalMatrix(invK3);

        // check correctness
        assertTrue(invK.equals(invK2, ABSOLUTE_ERROR));
        assertTrue(invK.equals(invK3, ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetParameters() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double aspectRatio = verticalFocalLength / horizontalFocalLength;
        final double horizontalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength2 = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double aspectRatio2 = verticalFocalLength2 / horizontalFocalLength2;

        final double aspectRatio3 = randomizer.nextDouble(MIN_ASPECT_RATIO,
                MAX_ASPECT_RATIO);
        final double aspectRatio4 = randomizer.nextDouble(MIN_ASPECT_RATIO,
                MAX_ASPECT_RATIO);

        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double skewnessAngle = Math.atan2(skewness, verticalFocalLength);
        final double skewness2 = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double skewnessAngle2 = Math.atan2(skewness2, verticalFocalLength);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double horizontalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint2 = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters k =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        // Test getting parameters
        assertEquals(horizontalFocalLength, k.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(verticalFocalLength, k.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(aspectRatio, k.getAspectRatio(), ABSOLUTE_ERROR);
        assertEquals(horizontalPrincipalPoint, k.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(verticalPrincipalPoint, k.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(skewness, k.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(skewnessAngle, k.getSkewnessAngle(), ABSOLUTE_ERROR);

        // Simulate internal matrix
        final Matrix kMatrix2 = new Matrix(INTRINSIC_ROWS, INTRINSIC_COLS);
        kMatrix2.setElementAt(0, 0, horizontalFocalLength);
        kMatrix2.setElementAt(1, 1, verticalFocalLength);
        kMatrix2.setElementAt(2, 2, 1.0);
        kMatrix2.setElementAt(0, 1, skewness);
        kMatrix2.setElementAt(0, 2, horizontalPrincipalPoint);
        kMatrix2.setElementAt(1, 2, verticalPrincipalPoint);

        final Matrix kMatrix = k.getInternalMatrix();

        // And compare both matrices
        assertTrue(kMatrix.equals(kMatrix2, ABSOLUTE_ERROR));

        // set new parameters
        k.setHorizontalFocalLength(horizontalFocalLength2);
        assertEquals(horizontalFocalLength2, k.getHorizontalFocalLength(), ABSOLUTE_ERROR);

        k.setVerticalFocalLength(verticalFocalLength2);
        assertEquals(verticalFocalLength2, k.getVerticalFocalLength(), ABSOLUTE_ERROR);

        assertEquals(aspectRatio2, k.getAspectRatio(), ABSOLUTE_ERROR);

        k.setAspectRatioKeepingHorizontalFocalLength(aspectRatio3);
        assertEquals(aspectRatio3, k.getAspectRatio(), ABSOLUTE_ERROR);
        assertEquals(horizontalFocalLength2, k.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(aspectRatio3 * horizontalFocalLength2,
                k.getVerticalFocalLength(), ABSOLUTE_ERROR);

        k.setAspectRatioKeepingVerticalFocalLength(aspectRatio4);
        assertEquals(aspectRatio4, k.getAspectRatio(), ABSOLUTE_ERROR);
        assertEquals(aspectRatio3 * horizontalFocalLength2 / aspectRatio4,
                k.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(aspectRatio3 * horizontalFocalLength2,
                k.getVerticalFocalLength(), ABSOLUTE_ERROR);

        k.setHorizontalPrincipalPoint(horizontalPrincipalPoint2);
        assertEquals(horizontalPrincipalPoint2, k.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);

        k.setVerticalPrincipalPoint(verticalPrincipalPoint2);
        assertEquals(verticalPrincipalPoint2, k.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);

        k.setSkewness(skewness2);
        assertEquals(skewness2, k.getSkewness(), ABSOLUTE_ERROR);
        k.setVerticalFocalLength(verticalFocalLength2);
        k.setSkewnessAngle(skewnessAngle2);
        assertEquals(skewnessAngle2, k.getSkewnessAngle(), ABSOLUTE_ERROR);
    }

    @Test
    public void testCreateCanonicalIntrinsicParameters()
            throws WrongSizeException {
        final PinholeCameraIntrinsicParameters k = PinholeCameraIntrinsicParameters.
                createCanonicalIntrinsicParameters();

        assertTrue(k.getInternalMatrix().equals(Matrix.identity(INTRINSIC_ROWS,
                INTRINSIC_COLS), ABSOLUTE_ERROR));
    }

    @Test
    public void testCreateTypicalIntrinsicParameters()
            throws WrongSizeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int imageWidth = randomizer.nextInt(MIN_IMAGE_WIDTH, MAX_IMAGE_WIDTH);
        final int imageHeight = randomizer.nextInt(MIN_IMAGE_HEIGHT, MAX_IMAGE_HEIGHT);

        final PinholeCameraIntrinsicParameters k = PinholeCameraIntrinsicParameters.
                createTypicalIntrinsicParameters(imageWidth, imageHeight);

        final Matrix kMatrix = k.getInternalMatrix();

        final Matrix kMatrix2 = new Matrix(INTRINSIC_ROWS, INTRINSIC_COLS);
        kMatrix2.setElementAt(0, 0, (double) (imageWidth + imageHeight) / 2.0);
        kMatrix2.setElementAt(1, 1, (double) (imageWidth + imageHeight) / 2.0);
        kMatrix2.setElementAt(2, 2, 1.0);
        kMatrix2.setElementAt(0, 2, (double) imageWidth / 2.0);
        kMatrix2.setElementAt(1, 2, (double) imageHeight / 2.0);

        assertTrue(kMatrix.equals(kMatrix2, ABSOLUTE_ERROR));

        assertEquals((double) (imageWidth + imageHeight) / 2.0, k.getHorizontalFocalLength(),
                ABSOLUTE_ERROR);
        assertEquals((double) (imageWidth + imageHeight) / 2.0, k.getVerticalFocalLength(),
                ABSOLUTE_ERROR);
        assertEquals(0.0, k.getSkewness(), ABSOLUTE_ERROR);
        assertEquals((double) imageWidth / 2.0, k.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals((double) imageHeight / 2.0, k.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
    }

    @Test
    public void testClone() {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);

        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        // Create K
        final PinholeCameraIntrinsicParameters k =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        // Clone K
        final PinholeCameraIntrinsicParameters k2 = new PinholeCameraIntrinsicParameters(k);

        // check both intrinsic parameters are equal
        assertTrue(k.getInternalMatrix().equals(k2.getInternalMatrix(), ABSOLUTE_ERROR));

        // if we change K parameters, K2 will still remain the same
        k.setHorizontalFocalLength(horizontalFocalLength + 1.0);
        k.setVerticalFocalLength(verticalFocalLength + 1.0);
        k.setSkewness(skewness + 1.0);
        k.setHorizontalPrincipalPoint(horizontalPrincipalPoint + 1.0);
        k.setVerticalPrincipalPoint(verticalPrincipalPoint + 1.0);
        assertEquals(horizontalFocalLength + 1.0, k.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(horizontalFocalLength, k2.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(verticalFocalLength + 1.0, k.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(verticalFocalLength, k2.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(skewness + 1.0, k.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(skewness, k2.getSkewness(), ABSOLUTE_ERROR);
        assertEquals(horizontalPrincipalPoint + 1.0, k.getHorizontalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(horizontalPrincipalPoint, k2.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(verticalPrincipalPoint + 1.0, k.getVerticalPrincipalPoint(),
                ABSOLUTE_ERROR);
        assertEquals(verticalPrincipalPoint, k2.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
    }

    @Test
    public void testIsValidMatrix() throws WrongSizeException {

        // Build a random upper triangular matrix
        final Matrix triangularMatrix = Matrix.createWithUniformRandomValues(
                INTRINSIC_ROWS, INTRINSIC_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        for (int u = 0; u < INTRINSIC_ROWS; u++) {
            for (int v = 0; v < u; v++) {
                triangularMatrix.setElementAt(u, v, 0.0);
            }
        }

        // use transposed upper triangular matrix (which is lower triangular) to
        // build any random non-triangular matrix
        final Matrix transTriangularMatrix = triangularMatrix.transposeAndReturnNew();

        Matrix kMatrix = transTriangularMatrix.multiplyAndReturnNew(
                triangularMatrix);

        assertFalse(PinholeCameraIntrinsicParameters.isValidMatrix(kMatrix));
        assertFalse(PinholeCameraIntrinsicParameters.isValidMatrix(kMatrix, ABSOLUTE_ERROR));

        // use upper triangular matrix
        kMatrix = triangularMatrix;
        // make matrix invalid because lower-right element is not 10
        kMatrix.setElementAt(2, 2, 2.0);

        assertFalse(PinholeCameraIntrinsicParameters.isValidMatrix(kMatrix));
        assertFalse(PinholeCameraIntrinsicParameters.isValidMatrix(kMatrix, ABSOLUTE_ERROR));

        // we can normalize KMatrix by setting its lower-right element to 1.0
        kMatrix.multiplyByScalar(0.5);

        // now KMatrix should be valid
        assertTrue(PinholeCameraIntrinsicParameters.isValidMatrix(kMatrix));
        assertTrue(PinholeCameraIntrinsicParameters.isValidMatrix(kMatrix, ABSOLUTE_ERROR));
    }

    @Test
    public void testCreate() {
        // For nexus 5
        double focalLength = 3.97; // mm
        double sensorWidth = 4.6032; // mm
        double sensorHeight = 3.5168; // mm
        int imageWidth = 2048; // px
        int imageHeight = 1536; // px

        final PinholeCameraIntrinsicParameters intrinsic =
                PinholeCameraIntrinsicParameters.create(focalLength,
                        sensorWidth, sensorHeight, imageWidth, imageHeight);

        // check
        assertEquals(focalLength * imageWidth / sensorWidth,
                intrinsic.getHorizontalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(focalLength * imageHeight / sensorHeight,
                intrinsic.getVerticalFocalLength(), ABSOLUTE_ERROR);
        assertEquals(0.0, intrinsic.getSkewness(), 0.0);
        assertEquals(0.0, intrinsic.getHorizontalPrincipalPoint(), ABSOLUTE_ERROR);
        assertEquals(0.0, intrinsic.getVerticalPrincipalPoint(), ABSOLUTE_ERROR);
    }

    @Test
    public void testSerializeDeserialize() throws IOException, ClassNotFoundException {
        // Test constructor with parameters
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double aspectRatio = verticalFocalLength / horizontalFocalLength;

        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);

        final double horizontalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(
                MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters k1 = new PinholeCameraIntrinsicParameters(
                horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        // check
        assertEquals(horizontalFocalLength, k1.getHorizontalFocalLength(), 0.0);
        assertEquals(verticalFocalLength, k1.getVerticalFocalLength(), 0.0);
        assertEquals(aspectRatio, k1.getAspectRatio(), 0.0);
        assertEquals(horizontalPrincipalPoint,
                k1.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(verticalPrincipalPoint,
                k1.getVerticalPrincipalPoint(), 0.0);
        assertEquals(skewness, k1.getSkewness(), 0.0);

        // serialize and deserialize
        final byte[] bytes = SerializationHelper.serialize(k1);
        final PinholeCameraIntrinsicParameters k2 = SerializationHelper.deserialize(bytes);

        // check
        assertNotSame(k1, k2);
        assertEquals(k1.getInternalMatrix(), k2.getInternalMatrix());
        assertNotSame(k1.getInternalMatrix(), k2.getInternalMatrix());

        assertEquals(k1.getHorizontalFocalLength(),
                k2.getHorizontalFocalLength(), 0.0);
        assertEquals(k1.getVerticalFocalLength(),
                k2.getVerticalFocalLength(), 0.0);
        assertEquals(k1.getAspectRatio(),
                k2.getAspectRatio(), 0.0);
        assertEquals(k1.getHorizontalPrincipalPoint(),
                k2.getHorizontalPrincipalPoint(), 0.0);
        assertEquals(k1.getVerticalPrincipalPoint(),
                k2.getVerticalPrincipalPoint(), 0.0);
    }
}
