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

import com.irurueta.algebra.NotAvailableException;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class MatrixRotation3DTest {

    private static final int ROTATION_ROWS = 3;
    private static final int ROTATION_COLS = 3;
    private static final int INHOM_COORDS = 3;

    private static final int HOM_ROTATION_ROWS = 4;
    private static final int HOM_ROTATION_COLS = 4;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_RANDOM_SCALE = 2.0;
    private static final double MAX_RANDOM_SCALE = 3.0;

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 100;

    @Test
    void testConstants() {
        assertEquals(3, MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS);
        assertEquals(3, MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS);
        assertEquals(4, MatrixRotation3D.ROTATION3D_HOM_MATRIX_ROWS);
        assertEquals(4, MatrixRotation3D.ROTATION3D_HOM_MATRIX_COLS);
        assertEquals(1e-6, MatrixRotation3D.GIMBAL_THRESHOLD, 0.0);
    }

    @Test
    void testConstructors() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException, RotationException {

        for (var t = 0; t < TIMES; t++) {

            // Test empty constructor
            var rotation = new MatrixRotation3D();
            assertNotNull(rotation);

            final var internalMatrix = rotation.getInternalMatrix();

            assertEquals(ROTATION_ROWS, internalMatrix.getRows());
            assertEquals(ROTATION_COLS, internalMatrix.getColumns());

            for (var v = 0; v < ROTATION_COLS; v++) {
                for (var u = 0; u < ROTATION_ROWS; u++) {
                    if (u == v) {
                        assertEquals(1.0, internalMatrix.getElementAt(u, v), ABSOLUTE_ERROR);
                    } else {
                        assertEquals(0.0, internalMatrix.getElementAt(u, v), ABSOLUTE_ERROR);
                    }
                }
            }

            // Test constructor with any random orthonormal matrix
            var a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, ROTATION_COLS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            var decomposer = new SingularValueDecomposer(a);

            decomposer.decompose();

            var rotationMatrix = decomposer.getV();

            rotation = new MatrixRotation3D(rotationMatrix, ABSOLUTE_ERROR);
            assertEquals(rotationMatrix, rotation.getInternalMatrix());

            // Attempt Force InvalidRotationMatrixException
            // (when threshold is too small, the following might fail)
            final var finalRotationMatrix = rotationMatrix;
            assertThrows(InvalidRotationMatrixException.class,
                    () -> new MatrixRotation3D(finalRotationMatrix, 0.0));

            // or when matrix is not orthogonal
            final var transRotationMatrix = rotationMatrix.transposeAndReturnNew();

            final var diagonal = new double[ROTATION_ROWS];
            Arrays.fill(diagonal, 1.0);
            //make last singular value equal to zero, so that
            // matrix is singular
            diagonal[2] = 0.0;

            final var diagMatrix = Matrix.diagonal(diagonal);

            var rotationMatrix2 = transRotationMatrix.multiplyAndReturnNew(
                    diagMatrix.multiplyAndReturnNew(rotationMatrix));

            final var finalRotationMatrix2 = rotationMatrix2;
            assertThrows(InvalidRotationMatrixException.class,
                    () -> new MatrixRotation3D(finalRotationMatrix2, ABSOLUTE_ERROR));

            // Test copy constructor
            rotation = new MatrixRotation3D(rotationMatrix, ABSOLUTE_ERROR);
            final var rotation2 = new MatrixRotation3D(rotation);
            assertEquals(rotation.getInternalMatrix(), rotation2.getInternalMatrix());

            // test constructor using euler angles
            final var randomizer = new UniformRandomizer();
            final var alphaEuler = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES));
            final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var gammaEuler = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES,
                    2.0 * MAX_ANGLE_DEGREES));
            // theta will be used along with axis
            final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);
            // alpha ambiguity: can be alpha or alpha - pi
            var valid = false;
            if (Math.abs(rotation.getAlphaEulerAngle() - alphaEuler) < ABSOLUTE_ERROR) {
                valid = true;
            } else if (Math.abs(rotation.getAlphaEulerAngle() - (alphaEuler - Math.PI)) < ABSOLUTE_ERROR) {
                valid = true;
            }
            assertTrue(valid);
            // beta ambiguity: can be beta or -beta
            valid = false;
            if (Math.abs(rotation.getBetaEulerAngle() - betaEuler) < ABSOLUTE_ERROR) {
                valid = true;
            } else if (Math.abs(rotation.getBetaEulerAngle() + betaEuler) < ABSOLUTE_ERROR) {
                valid = true;
            }
            assertTrue(valid);
            // gamma ambiguity: gamma can be gamma or gamma - pi
            valid = false;
            if (Math.abs(rotation.getGammaEulerAngle() - gammaEuler) < ABSOLUTE_ERROR) {
                valid = true;
            } else if (Math.abs(rotation.getGammaEulerAngle() - (gammaEuler - Math.PI)) < ABSOLUTE_ERROR) {
                valid = true;
            }
            assertTrue(valid);

            // check internal matrix
            rotationMatrix = rotation.getInternalMatrix();

            rotationMatrix2 = new Matrix(ROTATION_ROWS, ROTATION_COLS);
            rotationMatrix2.setElementAt(0, 0, Math.cos(alphaEuler)
                    * Math.cos(gammaEuler) - Math.sin(alphaEuler) * Math.sin(betaEuler) * Math.sin(gammaEuler));
            rotationMatrix2.setElementAt(1, 0, Math.cos(alphaEuler) * Math.sin(gammaEuler)
                    + Math.sin(alphaEuler) * Math.sin(betaEuler) * Math.cos(gammaEuler));
            rotationMatrix2.setElementAt(2, 0, -Math.sin(alphaEuler) * Math.cos(betaEuler));

            rotationMatrix2.setElementAt(0, 1, -Math.cos(betaEuler) * Math.sin(gammaEuler));
            rotationMatrix2.setElementAt(1, 1, Math.cos(betaEuler) * Math.cos(gammaEuler));
            rotationMatrix2.setElementAt(2, 1, Math.sin(betaEuler));

            rotationMatrix2.setElementAt(0, 2, Math.sin(alphaEuler) * Math.cos(gammaEuler)
                    + Math.cos(alphaEuler) * Math.sin(betaEuler) * Math.sin(gammaEuler));
            rotationMatrix2.setElementAt(1, 2, Math.sin(alphaEuler) * Math.sin(gammaEuler)
                    - Math.cos(alphaEuler) * Math.sin(betaEuler) * Math.cos(gammaEuler));
            rotationMatrix2.setElementAt(2, 2, Math.cos(alphaEuler) * Math.cos(betaEuler));

            // check that rotation matrices are equal (up to a certain error)
            assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));

            // test constructor using axis and angle

            // Force WrongSizeException
            var axis = new double[INHOM_COORDS + 1];
            final var finalAxis = axis;
            assertThrows(IllegalArgumentException.class, () -> new MatrixRotation3D(finalAxis, theta));

            // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
            // the remaining two will lie on the rotation plane and will be used
            // to test theta angle

            // To find 3 orthogonal vectors, we use V matrix of a singular
            // decomposition of any Nx3 matrix
            a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer = new SingularValueDecomposer(a);

            decomposer.decompose();

            final var vMatrix = decomposer.getV();

            // axis of rotation
            final var vAxis = vMatrix.getSubmatrixAsArray(0, 0,
                    2, 0);
            final var mAxis = Matrix.newFromArray(vAxis, true);

            // inhomogeneous coordinates of point laying on rotation plane
            final var vPoint = vMatrix.getSubmatrixAsArray(0, 1,
                    2, 1);

            rotation = new MatrixRotation3D(vAxis, theta);

            // To test correctness of rotation, axis should remain equal
            rotationMatrix = rotation.getInternalMatrix();

            final var mAxis2 = rotationMatrix.multiplyAndReturnNew(mAxis);

            assertEquals(mAxis.getElementAtIndex(0), mAxis2.getElementAtIndex(0), ABSOLUTE_ERROR);
            assertEquals(mAxis.getElementAtIndex(1), mAxis2.getElementAtIndex(1), ABSOLUTE_ERROR);
            assertEquals(mAxis.getElementAtIndex(2), mAxis2.getElementAtIndex(2), ABSOLUTE_ERROR);

            axis = rotation.getRotationAxis();

            final var scaleX = axis[0] / vAxis[0];
            final var scaleY = axis[0] / vAxis[0];
            final var scaleZ = axis[0] / vAxis[0];

            assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
            assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
            assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);

            // and point on rotated plane should be rotated exactly theta
            // radians
            final var mPoint = Matrix.newFromArray(vPoint);
            // below: rotated point
            final var mPoint2 = rotationMatrix.multiplyAndReturnNew(mPoint);

            // because vPoint is already normalized (from SVD decomposition)
            // we only need to normalize rotated point to compute the rotation
            // angle as the arc-cosine of their dot product
            final var norm2 = Utils.normF(mPoint2);
            mPoint2.multiplyByScalar(1.0 / norm2);

            final var dotProduct = mPoint.transposeAndReturnNew().multiplyAndReturnNew(mPoint2).getElementAtIndex(0);

            final var theta2 = Math.acos(dotProduct);
            final var theta2b = rotation.getRotationAngle();

            // check correctness of angles (up to sign for this test)
            assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);

            // check correctness of angles (including sign) for method in class
            if (scaleX > 0.0) {
                assertEquals(theta2b, theta, ABSOLUTE_ERROR);
            } else {
                assertEquals(-theta2b, theta, ABSOLUTE_ERROR);
            }
        }
    }

    @Test
    void testGetSetInternalMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {

        // Create identity rotation
        final var rotation = new MatrixRotation3D();

        // Obtain internal matrix
        final var internalMatrix = rotation.getInternalMatrix();

        // Check internal matrix is the identity
        assertEquals(ROTATION_ROWS, internalMatrix.getRows());
        assertEquals(ROTATION_COLS, internalMatrix.getColumns());
        assertTrue(internalMatrix.equals(Matrix.identity(ROTATION_ROWS, ROTATION_COLS), ABSOLUTE_ERROR));

        // Set any random orthonormal matrix
        final var a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        // Use V matrix from SVD to obtain a random orthonormal matrix
        final var rotationMatrix = decomposer.getV();

        rotation.setInternalMatrix(rotationMatrix, ABSOLUTE_ERROR);
        assertTrue(rotationMatrix.equals(rotation.getInternalMatrix(), ABSOLUTE_ERROR));

        // Force InvalidRotationMatrixException

        // when threshold is too small
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.setInternalMatrix(rotationMatrix,
                0.0));
        // but still rotation matrix remains from previous valid assignment
        assertTrue(rotationMatrix.equals(rotation.getInternalMatrix(), ABSOLUTE_ERROR));

        // or when matrix is not orthogonal
        final var transRotationMatrix = rotationMatrix.transposeAndReturnNew();

        final var diag = new double[ROTATION_ROWS];
        Arrays.fill(diag, 1.0);
        diag[2] = 0.0;

        final var diagMatrix = Matrix.diagonal(diag);

        final var rotationMatrix2 = transRotationMatrix.multiplyAndReturnNew(
                diagMatrix.multiplyAndReturnNew(rotationMatrix));
        assertThrows(InvalidRotationMatrixException.class,
                () -> rotation.setInternalMatrix(rotationMatrix2, ABSOLUTE_ERROR));

        // but still rotation matrix remains from previous valid assignment
        assertTrue(rotationMatrix.equals(rotation.getInternalMatrix(), ABSOLUTE_ERROR));

        rotation.setInternalMatrix(Matrix.identity(ROTATION_ROWS, ROTATION_COLS));

        assertEquals(Matrix.identity(ROTATION_ROWS, ROTATION_COLS), rotation.getInternalMatrix());
    }

    @Test
    void testGetSetEulerAngles() throws WrongSizeException {

        // randomizer to randomly compute euler angles
        final var randomizer = new UniformRandomizer();
        final var alphaEuler = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var betaEuler = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gammaEuler = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        var valid = false;

        final var rotation = new MatrixRotation3D();

        // set euler angles
        rotation.setEulerAngles(alphaEuler, betaEuler, gammaEuler);

        // retrieve euler angles to check correctness
        if (Math.abs(rotation.getAlphaEulerAngle() - alphaEuler) < ABSOLUTE_ERROR) {
            valid = true;
        } else if (Math.abs(rotation.getAlphaEulerAngle() - (alphaEuler - Math.PI)) < ABSOLUTE_ERROR) {
            valid = true;
        }
        assertTrue(valid);
        // beta ambiguity: can be beta or -beta
        valid = false;
        if (Math.abs(rotation.getBetaEulerAngle() - betaEuler) < ABSOLUTE_ERROR) {
            valid = true;
        } else if (Math.abs(rotation.getBetaEulerAngle() + betaEuler) < ABSOLUTE_ERROR) {
            valid = true;
        }
        assertTrue(valid);
        // gamma ambiguity: gamma can be gamma or gamma - pi
        valid = false;
        if (Math.abs(rotation.getGammaEulerAngle() - gammaEuler) < ABSOLUTE_ERROR) {
            valid = true;
        } else if (Math.abs(rotation.getGammaEulerAngle() - (gammaEuler - Math.PI)) < ABSOLUTE_ERROR) {
            valid = true;
        }
        assertTrue(valid);

        // check internal matrix
        final var rotationMatrix = rotation.getInternalMatrix();

        final var rotationMatrix2 = new Matrix(ROTATION_ROWS, ROTATION_COLS);
        rotationMatrix2.setElementAt(0, 0, Math.cos(alphaEuler) * Math.cos(gammaEuler)
                - Math.sin(alphaEuler) * Math.sin(betaEuler) * Math.sin(gammaEuler));
        rotationMatrix2.setElementAt(1, 0, Math.cos(alphaEuler) * Math.sin(gammaEuler)
                + Math.sin(alphaEuler) * Math.sin(betaEuler) * Math.cos(gammaEuler));
        rotationMatrix2.setElementAt(2, 0, -Math.sin(alphaEuler) * Math.cos(betaEuler));

        rotationMatrix2.setElementAt(0, 1, -Math.cos(betaEuler) * Math.sin(gammaEuler));
        rotationMatrix2.setElementAt(1, 1, Math.cos(betaEuler) * Math.cos(gammaEuler));
        rotationMatrix2.setElementAt(2, 1, Math.sin(betaEuler));

        rotationMatrix2.setElementAt(0, 2, Math.sin(alphaEuler) * Math.cos(gammaEuler)
                + Math.cos(alphaEuler) * Math.sin(betaEuler) * Math.sin(gammaEuler));
        rotationMatrix2.setElementAt(1, 2, Math.sin(alphaEuler) * Math.sin(gammaEuler)
                - Math.cos(alphaEuler) * Math.sin(betaEuler) * Math.cos(gammaEuler));
        rotationMatrix2.setElementAt(2, 2, Math.cos(alphaEuler) * Math.cos(betaEuler));

        // check that rotation matrices are equal (up to a certain error)
        assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetAxisAndRotation() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException, RotationException {

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new MatrixRotation3D();

        // Force IllegalArgumentException
        var axis = new double[INHOM_COORDS + 1];
        final var finalAxis = axis;
        assertThrows(IllegalArgumentException.class, () -> rotation.setAxisAndRotation(finalAxis, theta));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and the
        // remaining two will lie on the rotation plane and will be used to test
        // theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular value
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var v = decomposer.getV();

        // axis of rotation
        final var axisMatrix = v.getSubmatrix(0, 0,
                2, 0);

        // inhomogeneous coordinates of point laying on rotation plane
        final var pointMatrix = v.getSubmatrix(0, 1,
                2, 1);

        axis = axisMatrix.toArray();
        rotation.setAxisAndRotation(axis, theta);

        // To test correctness of rotation, axis should remain equal
        final var rotationMatrix = rotation.getInternalMatrix();

        final var axisMatrix2 = rotationMatrix.multiplyAndReturnNew(axisMatrix);

        assertTrue(axisMatrix.equals(axisMatrix2, ABSOLUTE_ERROR));

        final var axis2 = rotation.getRotationAxis();

        final var scaleX = axis[0] / axis2[0];
        final var scaleY = axis[0] / axis2[0];
        final var scaleZ = axis[0] / axis2[0];

        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleX, ABSOLUTE_ERROR);

        // and point on rotated plane should be rotated exactly theta
        // radians
        final var pointMatrix2 = rotationMatrix.multiplyAndReturnNew(pointMatrix);

        // because vPoint is already normalized (from SVD decomposition)
        // we only need to normalize rotated point to compute the rotation
        // angle as the arc-cosine of their dot product
        final var norm2 = Utils.normF(pointMatrix2);
        pointMatrix2.multiplyByScalar(1.0 / norm2);

        final var dotProduct = pointMatrix.transposeAndReturnNew().multiplyAndReturnNew(pointMatrix2)
                .getElementAtIndex(0);

        final var theta2 = Math.acos(dotProduct);
        final var theta2b = rotation.getRotationAngle();

        // check correctness of angles (up to sign for this test)
        assertEquals(Math.abs(theta), Math.abs(theta2), ABSOLUTE_ERROR);

        // check correctness of angles (including sign) for method in class
        if (scaleX > 0.0) {
            assertEquals(theta2b, theta, ABSOLUTE_ERROR);
        } else {
            assertEquals(-theta2b, theta, ABSOLUTE_ERROR);
        }
    }

    @Test
    void testIsValidRotationMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException {

        final var a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, ROTATION_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // Use SVD to obtain an orthonormal matrix from V matrix
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var rotationMatrix = decomposer.getV();

        // rotation matrix shouldn't be valid for tiny thresholds
        assertFalse(MatrixRotation3D.isValidRotationMatrix(rotationMatrix, 0.0));

        // for reasonable thresholds, matrix should be valid because it is
        //orthonormal with some imprecision due to machine precision
        assertTrue(MatrixRotation3D.isValidRotationMatrix(rotationMatrix, ABSOLUTE_ERROR));
        assertTrue(MatrixRotation3D.isValidRotationMatrix(rotationMatrix));

        final var randomizer = new UniformRandomizer();
        final var scale = randomizer.nextDouble(MIN_RANDOM_SCALE, MAX_RANDOM_SCALE);

        // change scale of matrix to make it orthogonal instead of orthonormal
        rotationMatrix.multiplyByScalar(scale);

        // now matrix shouldn't be valid even for reasonable thresholds
        assertFalse(MatrixRotation3D.isValidRotationMatrix(rotationMatrix, ABSOLUTE_ERROR));
        assertFalse(MatrixRotation3D.isValidRotationMatrix(rotationMatrix));
    }

    @Test
    void testInverseRotation() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {

        // Create random rotation
        final var a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, ROTATION_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // Use SVD to obtain an orthonormal matrix from V matrix
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var rotationMatrix = decomposer.getV();

        final var rotation = new MatrixRotation3D(rotationMatrix, ABSOLUTE_ERROR);

        // compute inverse rotation
        final var invRotation = rotation.inverseRotationAndReturnNew();
        final var invRotation2 = new MatrixRotation3D();
        rotation.inverseRotation(invRotation2);

        // check correctness
        final var invRotMatrix = invRotation.getInternalMatrix();
        final var invRotMatrix2 = invRotation2.getInternalMatrix();

        final var identity = Matrix.identity(ROTATION_ROWS, ROTATION_COLS);

        assertTrue(rotationMatrix.multiplyAndReturnNew(invRotMatrix).equals(identity, ABSOLUTE_ERROR));
        assertTrue(rotationMatrix.multiplyAndReturnNew(invRotMatrix2).equals(identity, ABSOLUTE_ERROR));

        // we can also invert the original rotation
        rotation.inverseRotation();
        assertTrue(invRotMatrix.equals(rotation.getInternalMatrix(), ABSOLUTE_ERROR));
    }

    @Test
    void testAsInhomogeneousMatrix() throws WrongSizeException, NotReadyException, LockedException,
            DecomposerException, NotAvailableException, InvalidRotationMatrixException {

        // Create random rotation
        final var a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, ROTATION_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // Use SVD to obtain an orthonormal matrix from V matrix
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var rotationMatrix = decomposer.getV();

        final var rotation = new MatrixRotation3D(rotationMatrix, ABSOLUTE_ERROR);

        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());

        final var rotationMatrix2 = new Matrix(ROTATION_ROWS, ROTATION_COLS);
        rotation.asInhomogeneousMatrix(rotationMatrix2);
        assertEquals(rotationMatrix, rotationMatrix2);

        // Force IllegalArgumentException
        final var m = new Matrix(HOM_ROTATION_ROWS, HOM_ROTATION_COLS);
        assertThrows(IllegalArgumentException.class, () -> rotation.asInhomogeneousMatrix(m));
    }

    @Test
    void testAsHomogeneousMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {

        // Create random rotation
        final var a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, ROTATION_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // Use SVD to obtain an orthonormal matrix from V matrix
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var rotationMatrix = decomposer.getV();

        final var homRotationMatrix = Matrix.identity(HOM_ROTATION_ROWS, HOM_ROTATION_COLS);
        // set top-left 3x3 sub-matrix
        homRotationMatrix.setSubmatrix(0, 0, 2, 2,
                rotationMatrix);


        final var rotation = new MatrixRotation3D(homRotationMatrix, ABSOLUTE_ERROR);

        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());

        final var rotationMatrix2 = new Matrix(HOM_ROTATION_ROWS, HOM_ROTATION_COLS);
        rotation.asHomogeneousMatrix(rotationMatrix2);
        assertEquals(homRotationMatrix, rotationMatrix2);

        // Force IllegalArgumentException
        final var m = new Matrix(ROTATION_ROWS, ROTATION_COLS);
        assertThrows(IllegalArgumentException.class, () -> rotation.asHomogeneousMatrix(m));
    }

    @Test
    void testFromMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {

        // Create random rotation
        final var a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, ROTATION_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // Use SVD to obtain an orthonormal matrix from V matrix
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var rotationMatrix = decomposer.getV();

        final var homRotationMatrix = Matrix.identity(HOM_ROTATION_ROWS, HOM_ROTATION_COLS);
        // set top-left 3x3 sub-matrix
        homRotationMatrix.setSubmatrix(0, 0, 2, 2,
                rotationMatrix);

        final var rotation = new MatrixRotation3D();
        // test with inhomogeneous matrix
        rotation.fromMatrix(rotationMatrix);
        // Check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());

        rotation.fromMatrix(rotationMatrix, ABSOLUTE_ERROR);
        // Check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());

        // test with homogeneous matrix
        rotation.fromMatrix(homRotationMatrix);
        // check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());

        rotation.fromMatrix(homRotationMatrix, ABSOLUTE_ERROR);
        // check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());

        // Force InvalidRotationMatrixException (using a tiny threshold)
        assertThrows(InvalidRotationMatrixException.class, () -> rotation.fromMatrix(rotationMatrix, 0.0));

        // or using a non orthonormal matrix
        homRotationMatrix.setElementAt(3, 3, 0.0); //makes matrix singular
        assertThrows(InvalidRotationMatrixException.class,
                () -> rotation.fromMatrix(homRotationMatrix, ABSOLUTE_ERROR));
    }

    @Test
    void testFromInhomogeneousMatrix() throws WrongSizeException, NotReadyException, LockedException,
            DecomposerException, NotAvailableException, InvalidRotationMatrixException {

        // Create random rotation
        final var a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, ROTATION_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // Use SVD to obtain an orthonormal matrix from V matrix
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var rotationMatrix = decomposer.getV();

        final var rotation = new MatrixRotation3D();
        // test with inhomogeneous matrix
        rotation.fromInhomogeneousMatrix(rotationMatrix);
        // Check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());

        rotation.fromInhomogeneousMatrix(rotationMatrix, ABSOLUTE_ERROR);
        // Check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());

        // Force InvalidRotationMatrixException (using a tiny threshold)
        assertThrows(InvalidRotationMatrixException.class,
                () -> rotation.fromInhomogeneousMatrix(rotationMatrix, 0.0));

        // or using a non orthonormal matrix
        // because matrix is orthonormal, it's enough to scale it to some value
        final var randomizer = new UniformRandomizer();
        final var scale = randomizer.nextDouble(MIN_RANDOM_SCALE, MAX_RANDOM_SCALE);
        rotationMatrix.multiplyByScalar(scale);
        assertThrows(InvalidRotationMatrixException.class,
                () -> rotation.fromInhomogeneousMatrix(rotationMatrix, ABSOLUTE_ERROR));
    }

    @Test
    void testFromHomogeneousMatrix() throws WrongSizeException, NotReadyException, LockedException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException {

        // Create random rotation
        final var a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, ROTATION_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // Use SVD to obtain an orthonormal matrix from V matrix
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var rotationMatrix = decomposer.getV();

        final var homRotationMatrix = Matrix.identity(HOM_ROTATION_ROWS, HOM_ROTATION_COLS);
        // set top-left 3x3 sub-matrix
        homRotationMatrix.setSubmatrix(0, 0, 2, 2,
                rotationMatrix);


        final var rotation = new MatrixRotation3D();
        // test with homogeneous matrix
        rotation.fromHomogeneousMatrix(homRotationMatrix);
        // check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());

        rotation.fromHomogeneousMatrix(homRotationMatrix, ABSOLUTE_ERROR);
        // check correctness
        assertEquals(rotationMatrix, rotation.asInhomogeneousMatrix());
        assertEquals(rotationMatrix, rotation.getInternalMatrix());
        assertEquals(homRotationMatrix, rotation.asHomogeneousMatrix());

        // Force InvalidRotationMatrixException (using a tiny threshold)
        assertThrows(InvalidRotationMatrixException.class,
                () -> rotation.fromHomogeneousMatrix(rotationMatrix, 0.0));

        // or using a non orthonormal matrix
        // makes matrix singular
        homRotationMatrix.setElementAt(3, 3, 0.0);
        assertThrows(InvalidRotationMatrixException.class,
                () -> rotation.fromHomogeneousMatrix(homRotationMatrix, ABSOLUTE_ERROR));
    }

    @Test
    void testRotate() throws WrongSizeException, ColinearPointsException {
        final var randomizer = new UniformRandomizer();
        final var alpha = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var beta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gamma = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new MatrixRotation3D(alpha, beta, gamma);

        // create 3 random points
        final var point1 = new HomogeneousPoint3D();
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var point2 = new HomogeneousPoint3D();
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final var point3 = new HomogeneousPoint3D();
        point3.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // ensure that points are not co-linear

        while (Plane.areColinearPoints(point1, point2, point3)) {
            point2.setInhomogeneousCoordinates(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
            point3.setInhomogeneousCoordinates(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        }

        // create plane passing through all three points
        final var plane = new Plane(point1, point2, point3);
        assertTrue(plane.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point3, ABSOLUTE_ERROR));

        // now rotate points and plane
        final var rotPoint1A = rotation.rotate(point1);
        final var rotPoint1B = Point3D.create();
        rotation.rotate(point1, rotPoint1B);

        final var rotPoint2A = rotation.rotate(point2);
        final var rotPoint2B = Point3D.create();
        rotation.rotate(point2, rotPoint2B);

        final var rotPoint3A = rotation.rotate(point3);
        final var rotPoint3B = Point3D.create();
        rotation.rotate(point3, rotPoint3B);

        // check that rotated points A and B are equal
        assertTrue(rotPoint1A.equals(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPoint2A.equals(rotPoint2B, ABSOLUTE_ERROR));
        assertTrue(rotPoint3A.equals(rotPoint3B, ABSOLUTE_ERROR));

        // check points have been correctly rotated
        final var point1Mat = Matrix.newFromArray(point1.asArray(), true);
        final var point2Mat = Matrix.newFromArray(point2.asArray(), true);
        final var point3Mat = Matrix.newFromArray(point3.asArray(), true);
        final var r = rotation.asHomogeneousMatrix();
        final var rotPoint1Mat = r.multiplyAndReturnNew(point1Mat);
        final var rotPoint2Mat = r.multiplyAndReturnNew(point2Mat);
        final var rotPoint3Mat = r.multiplyAndReturnNew(point3Mat);

        // check correctness
        var scaleX = rotPoint1A.getHomX() / rotPoint1Mat.getElementAtIndex(0);
        var scaleY = rotPoint1A.getHomY() / rotPoint1Mat.getElementAtIndex(1);
        var scaleZ = rotPoint1A.getHomZ() / rotPoint1Mat.getElementAtIndex(2);
        var scaleW = rotPoint1A.getHomW() / rotPoint1Mat.getElementAtIndex(3);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        scaleX = rotPoint2A.getHomX() / rotPoint2Mat.getElementAtIndex(0);
        scaleY = rotPoint2A.getHomY() / rotPoint2Mat.getElementAtIndex(1);
        scaleZ = rotPoint2A.getHomZ() / rotPoint2Mat.getElementAtIndex(2);
        scaleW = rotPoint2A.getHomW() / rotPoint2Mat.getElementAtIndex(3);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        scaleX = rotPoint3A.getHomX() / rotPoint3Mat.getElementAtIndex(0);
        scaleY = rotPoint3A.getHomY() / rotPoint3Mat.getElementAtIndex(1);
        scaleZ = rotPoint3A.getHomZ() / rotPoint3Mat.getElementAtIndex(2);
        scaleW = rotPoint3A.getHomW() / rotPoint3Mat.getElementAtIndex(3);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleZ, ABSOLUTE_ERROR);
        assertEquals(scaleZ, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        // ensure that points where correctly rotated using inhomogeneous
        // coordinates
        final var inhomPoint = new Matrix(INHOM_COORDS, 1);
        inhomPoint.setElementAtIndex(0, point1.getInhomX());
        inhomPoint.setElementAtIndex(1, point1.getInhomY());
        inhomPoint.setElementAtIndex(2, point1.getInhomZ());
        final var inhomR = rotation.asInhomogeneousMatrix();
        final var inhomRotPoint = inhomR.multiplyAndReturnNew(inhomPoint);
        final var rotP = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, inhomRotPoint.toArray());
        assertTrue(rotP.equals(rotPoint1A, ABSOLUTE_ERROR));

        final var rotPlaneA = rotation.rotate(plane);
        final var rotPlaneB = new Plane();
        rotation.rotate(plane, rotPlaneB);

        // check both rotated lines are equal
        var scaleA = rotPlaneA.getA() / rotPlaneB.getA();
        var scaleB = rotPlaneA.getB() / rotPlaneB.getB();
        var scaleC = rotPlaneA.getC() / rotPlaneB.getC();
        var scaleD = rotPlaneA.getD() / rotPlaneB.getD();

        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);

        // check plane has been correctly rotated by ensuring that rotated points
        // belong into rotated line
        assertTrue(rotPlaneA.isLocus(rotPoint1A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint2A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint2B, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint3A, ABSOLUTE_ERROR));
        assertTrue(rotPlaneA.isLocus(rotPoint3B, ABSOLUTE_ERROR));

        // and by ensuring that rotated plane follow appropriate equation
        final var planeMat = Matrix.newFromArray(plane.asArray(), true);
        final var rotPlaneMat = r.multiplyAndReturnNew(planeMat);

        scaleA = rotPlaneA.getA() / rotPlaneMat.getElementAtIndex(0);
        scaleB = rotPlaneA.getB() / rotPlaneMat.getElementAtIndex(1);
        scaleC = rotPlaneA.getC() / rotPlaneMat.getElementAtIndex(2);
        scaleD = rotPlaneA.getD() / rotPlaneMat.getElementAtIndex(3);

        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleD, ABSOLUTE_ERROR);
        assertEquals(scaleD, scaleA, ABSOLUTE_ERROR);
    }

    @Test
    void testCombine() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var alpha1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var alpha2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var beta1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var beta2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gamma1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var gamma2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rot1 = new MatrixRotation3D(alpha1, beta1, gamma1);
        final var rot2 = new MatrixRotation3D(alpha2, beta2, gamma2);

        final var rot = new MatrixRotation3D();
        MatrixRotation3D.combine(rot1, rot2, rot);

        assertTrue(rot.getInternalMatrix().equals(rot1.getInternalMatrix()
                        .multiplyAndReturnNew(rot2.getInternalMatrix()), ABSOLUTE_ERROR));

        final var rot3 = rot.combineAndReturnNew(rot2);
        assertTrue(rot3.getInternalMatrix().equals(rot.getInternalMatrix()
                        .multiplyAndReturnNew(rot2.getInternalMatrix()), ABSOLUTE_ERROR));

        final var rotationMatrix = rot.getInternalMatrix();
        rot.combine(rot1);
        assertTrue(rot.getInternalMatrix().equals(rotationMatrix
                        .multiplyAndReturnNew(rot1.getInternalMatrix()), ABSOLUTE_ERROR));
    }

    @Test
    void testType() {
        final var rotation = new MatrixRotation3D();
        assertEquals(Rotation3DType.MATRIX_ROTATION3D, rotation.getType());
    }

    @Test
    void testGetSetRollPitchYawAngles() throws WrongSizeException {
        // randomizer to randomly compute euler angles
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var rotation = new MatrixRotation3D();

        // set angles
        rotation.setRollPitchYaw(roll, pitch, yaw);

        // retrieve angles to check correctness
        assertTrue(Math.abs(rotation.getRollAngle() - roll) < ABSOLUTE_ERROR
                || Math.abs(rotation.getRollAngle2() - roll) < ABSOLUTE_ERROR);
        assertTrue(Math.abs(rotation.getPitchAngle() - pitch) < ABSOLUTE_ERROR
                || Math.abs(rotation.getPitchAngle2() - roll) < ABSOLUTE_ERROR);
        assertTrue(Math.abs(rotation.getYawAngle() - yaw) < ABSOLUTE_ERROR
                || Math.abs(rotation.getYawAngle2() - yaw) < ABSOLUTE_ERROR);

        // check internal matrix
        final var rotationMatrix = rotation.getInternalMatrix();

        final var rotationMatrix2 = new Matrix(ROTATION_ROWS, ROTATION_COLS);
        rotationMatrix2.setElementAt(0, 0, Math.cos(pitch) * Math.cos(yaw));
        rotationMatrix2.setElementAt(1, 0, Math.cos(pitch) * Math.sin(yaw));
        rotationMatrix2.setElementAt(2, 0, -Math.sin(pitch));

        rotationMatrix2.setElementAt(0, 1, -Math.cos(roll) * Math.sin(yaw)
                + Math.sin(roll) * Math.sin(pitch) * Math.cos(yaw));
        rotationMatrix2.setElementAt(1, 1, Math.cos(roll) * Math.cos(yaw)
                + Math.sin(roll) * Math.sin(pitch) * Math.sin(yaw));
        rotationMatrix2.setElementAt(2, 1, Math.sin(roll) * Math.cos(pitch));

        rotationMatrix2.setElementAt(0, 2, Math.sin(roll) * Math.sin(yaw)
                + Math.cos(roll) * Math.sin(pitch) * Math.cos(yaw));
        rotationMatrix2.setElementAt(1, 2, -Math.sin(roll) * Math.cos(yaw)
                + Math.cos(roll) * Math.sin(pitch) * Math.sin(yaw));
        rotationMatrix2.setElementAt(2, 2, Math.cos(roll) * Math.cos(pitch));

        // check that rotation matrices are equal (up to a certain error)
        assertTrue(rotationMatrix.equals(rotationMatrix2, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetRollPitchYaw() {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES));

        final var rotation = new MatrixRotation3D();

        // set values
        rotation.setRollPitchYaw(roll, pitch, yaw);

        // check correctness
        assertTrue(Math.abs(roll - rotation.getRollAngle()) <= ABSOLUTE_ERROR
                || Math.abs(roll - rotation.getRollAngle2()) <= ABSOLUTE_ERROR);

        assertTrue(Math.abs(pitch - rotation.getPitchAngle()) <= ABSOLUTE_ERROR
                || Math.abs(pitch - rotation.getPitchAngle2()) <= ABSOLUTE_ERROR);

        assertTrue(Math.abs(yaw - rotation.getYawAngle()) <= ABSOLUTE_ERROR
                || Math.abs(yaw - rotation.getYawAngle2()) <= ABSOLUTE_ERROR);
    }

    @Test
    void testHasGimbalLock() {
        final var randomizer = new UniformRandomizer();
        final var roll = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        var pitch = 0.0;
        final var yaw = randomizer.nextDouble(2.0 * MIN_ANGLE_DEGREES, 2.0 * MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final var rotation = new MatrixRotation3D();

        // set values
        rotation.setRollPitchYaw(roll, pitch, yaw);

        // check correctness
        assertFalse(rotation.hasGimbalLock());

        // set new pitch
        pitch = Math.PI / 4.0;
        rotation.setRollPitchYaw(roll, pitch, yaw);

        // check correctness
        assertFalse(rotation.hasGimbalLock());

        // set new pitch
        pitch = Math.PI / 2.0;
        rotation.setRollPitchYaw(roll, pitch, yaw);

        // check correctness
        assertTrue(rotation.hasGimbalLock());

        // set new pitch
        pitch = -Math.PI / 2.0;
        rotation.setRollPitchYaw(roll, pitch, yaw);

        // check correctness
        assertTrue(rotation.hasGimbalLock());
    }

    @Test
    void testFromRotation() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var matrixRotation = new MatrixRotation3D();
        final var axisRotation = new AxisRotation3D();
        final var quaternion = new Quaternion();

        matrixRotation.setAxisAndRotation(axis, theta);
        axisRotation.setAxisAndRotation(axis, theta);
        quaternion.setAxisAndRotation(axis, theta);

        // test from matrix rotation
        final var rotation1 = new MatrixRotation3D();
        rotation1.fromRotation(matrixRotation);

        // check correctness
        assertEquals(matrixRotation, rotation1);

        // test from axis rotation
        final var rotation2 = new MatrixRotation3D();
        rotation2.fromRotation(axisRotation);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(axisRotation, rotation2);

        // test from quaternion
        final var rotation3 = new MatrixRotation3D();
        rotation3.fromRotation(quaternion);

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(quaternion, rotation3);
    }

    @Test
    void testToMatrixRotation() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0,
                2, 0);

        final var rotation = new MatrixRotation3D(axis, theta);

        // to matrix rotation
        final var matrixRotation1 = new MatrixRotation3D();
        rotation.toMatrixRotation(matrixRotation1);
        final var matrixRotation2 = rotation.toMatrixRotation();

        // check correctness
        assertEquals(rotation, matrixRotation1);
        assertEquals(rotation, matrixRotation2);
    }

    @Test
    void testToAxisRotation() throws AlgebraException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();
            final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
            // the remaining two will lie on the rotation plane and will be used
            // to test theta angle

            // To find 3 orthogonal vectors, we use V matrix of a singular
            // decomposition of any Nx3 matrix
            final var a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            final var decomposer = new SingularValueDecomposer(a);

            decomposer.decompose();

            final var vMatrix = decomposer.getV();

            // axis of rotation
            final var axis = vMatrix.getSubmatrixAsArray(0, 0,
                    2, 0);

            final var rotation = new MatrixRotation3D(axis, theta);

            // to axis rotation
            final var axisRotation1 = new AxisRotation3D();
            rotation.toAxisRotation(axisRotation1);
            final var axisRotation2 = rotation.toAxisRotation();

            // check correctness
            if (!rotation.equals(axisRotation1)) {
                continue;
            }
            //noinspection AssertBetweenInconvertibleTypes
            assertEquals(rotation, axisRotation1);
            if (!rotation.equals(axisRotation2)) {
                continue;
            }
            //noinspection AssertBetweenInconvertibleTypes
            assertEquals(rotation, axisRotation2);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testToQuaternion() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        // Find any 3 orthogonal vectors, 1st will be axis of rotation, and
        // the remaining two will lie on the rotation plane and will be used
        // to test theta angle

        // To find 3 orthogonal vectors, we use V matrix of a singular
        // decomposition of any Nx3 matrix
        final var a = Matrix.createWithUniformRandomValues(1, ROTATION_COLS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var vMatrix = decomposer.getV();

        // axis of rotation
        final var axis = vMatrix.getSubmatrixAsArray(0, 0, 2, 0);

        final var rotation = new MatrixRotation3D(axis, theta);

        // to quaternion
        final var quaternion1 = new Quaternion();
        rotation.toQuaternion(quaternion1);
        final var quaternion2 = rotation.toQuaternion();

        // check correctness
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(rotation, quaternion1);
        //noinspection AssertBetweenInconvertibleTypes
        assertEquals(rotation, quaternion2);
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, LockedException, NotReadyException, DecomposerException,
            NotAvailableException, InvalidRotationMatrixException, IOException, ClassNotFoundException {
        final var a = Matrix.createWithUniformRandomValues(ROTATION_ROWS, ROTATION_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(a);

        decomposer.decompose();

        final var rotationMatrix = decomposer.getV();

        final var rotation1 = new MatrixRotation3D(rotationMatrix, ABSOLUTE_ERROR);

        // check
        assertEquals(rotation1.getInternalMatrix(), rotationMatrix);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(rotation1);
        final var rotation2 = SerializationHelper.<MatrixRotation3D>deserialize(bytes);

        // check
        assertEquals(rotation1, rotation2);
        assertNotSame(rotation1, rotation2);
    }
}
