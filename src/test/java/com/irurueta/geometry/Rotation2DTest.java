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

import java.util.Random;

import static org.junit.Assert.*;

public class Rotation2DTest {

    private static final double MIN_THETA = -Math.PI;
    private static final double MAX_THETA = Math.PI;
    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final int INHOM_COORDS = 2;

    @Test
    public void testConstants() {
        assertEquals(2, Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS);
        assertEquals(2, Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        assertEquals(3, Rotation2D.ROTATION2D_HOM_MATRIX_ROWS);
        assertEquals(3, Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        assertEquals(1e-9, Rotation2D.MATRIX_VALID_THRESHOLD, 0.0);
        assertEquals(0.0, Rotation2D.MIN_THRESHOLD, 0.0);
        assertEquals(1e-9, Rotation2D.DEFAULT_COMPARISON_THRESHOLD, 0.0);
        assertEquals(0.0, Rotation2D.MIN_COMPARISON_THRESHOLD, 0.0);
    }

    @Test
    public void testConstructor() throws WrongSizeException, InvalidRotationMatrixException {

        Rotation2D rotation;

        // test empty constructor
        assertNotNull(rotation = new Rotation2D());
        assertEquals(rotation.getTheta(), 0.0, 0.0);

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        // test from rotation angle
        rotation = new Rotation2D(theta);
        assertEquals(rotation.getTheta(), theta, 0.0);

        // test from another rotation
        final Rotation2D rotation2 = new Rotation2D(rotation);
        assertEquals(rotation2.getTheta(), theta, 0.0);

        // test from inhomogeneous rotation matrix
        final Matrix inhomRotMat = new Matrix(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        final double cosTheta = Math.cos(theta);
        final double sinTheta = Math.sin(theta);
        inhomRotMat.setElementAt(0, 0, cosTheta);
        inhomRotMat.setElementAt(1, 0, sinTheta);
        inhomRotMat.setElementAt(0, 1, -sinTheta);
        inhomRotMat.setElementAt(1, 1, cosTheta);

        rotation = new Rotation2D(inhomRotMat);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // now using threshold
        rotation = new Rotation2D(inhomRotMat, ABSOLUTE_ERROR);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // Force IllegalArgumentException by using a negative threshold
        rotation = null;
        try {
            rotation = new Rotation2D(inhomRotMat, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(rotation);

        // test from homogeneous rotation matrix
        final Matrix homRotMat = Matrix.identity(
                Rotation2D.ROTATION2D_HOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        homRotMat.setSubmatrix(0, 0, 1, 1, inhomRotMat);

        rotation = new Rotation2D(homRotMat);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // now using threshold
        rotation = new Rotation2D(homRotMat, ABSOLUTE_ERROR);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // Force IllegalArgumentException by using a negative threshold
        rotation = null;
        try {
            rotation = new Rotation2D(inhomRotMat, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(rotation);

        // Force InvalidRotationMatrixException
        // because of wrong size
        Matrix m = new Matrix(2, 3);
        try {
            rotation = new Rotation2D(m);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }
        assertNull(rotation);

        // because determinant is not 1
        m = Matrix.identity(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        m.multiplyByScalar(2.0);
        try {
            rotation = new Rotation2D(m);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }
        assertNull(rotation);
    }

    @Test
    public void testGetSetTheta() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final Rotation2D rotation = new Rotation2D();
        assertEquals(rotation.getTheta(), 0.0, 0.0);

        // set new theta
        rotation.setTheta(theta);
        // check correctness
        assertEquals(rotation.getTheta(), theta, 0.0);
    }

    @Test
    public void testInverseRotation() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final Rotation2D rotation = new Rotation2D(theta);
        assertEquals(rotation.getTheta(), theta, 0.0);

        // get inverse rotation
        final Rotation2D invRotation = rotation.inverseRotation();
        assertEquals(invRotation.getTheta(), -theta, 0.0);

        // check inverse rotation matrix is indeed the inverse
        final Matrix rotInhomMatrix = rotation.asInhomogeneousMatrix();
        final Matrix rotHomMatrix = rotation.asHomogeneousMatrix();
        Matrix invRotInhomMatrix = invRotation.asInhomogeneousMatrix();
        Matrix invRotHomMatrix = invRotation.asHomogeneousMatrix();

        assertTrue(rotInhomMatrix.multiplyAndReturnNew(invRotInhomMatrix).
                equals(Matrix.identity(
                        Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                        Rotation2D.ROTATION2D_INHOM_MATRIX_COLS), ABSOLUTE_ERROR));
        assertTrue(rotHomMatrix.multiplyAndReturnNew(invRotHomMatrix).
                equals(Matrix.identity(
                        Rotation2D.ROTATION2D_HOM_MATRIX_ROWS,
                        Rotation2D.ROTATION2D_HOM_MATRIX_COLS), ABSOLUTE_ERROR));

        // inverse rotation using provided rotation instance
        rotation.inverseRotation(rotation);
        assertEquals(rotation.getTheta(), -theta, 0.0);


        // check again inverse rotation matrix is indeed the inverse
        invRotInhomMatrix = rotation.asInhomogeneousMatrix();
        invRotHomMatrix = rotation.asHomogeneousMatrix();

        assertTrue(rotInhomMatrix.multiplyAndReturnNew(invRotInhomMatrix).
                equals(Matrix.identity(
                        Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                        Rotation2D.ROTATION2D_INHOM_MATRIX_COLS), ABSOLUTE_ERROR));
        assertTrue(rotHomMatrix.multiplyAndReturnNew(invRotHomMatrix).
                equals(Matrix.identity(
                        Rotation2D.ROTATION2D_HOM_MATRIX_ROWS,
                        Rotation2D.ROTATION2D_HOM_MATRIX_COLS), ABSOLUTE_ERROR));
    }

    @Test
    public void testAsInhomogeneousMatrix()
            throws InvalidRotationMatrixException, WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final Rotation2D rotation = new Rotation2D(theta);

        Matrix rotMatrix = rotation.asInhomogeneousMatrix();

        final double sinTheta = Math.sin(theta);
        final double cosTheta = Math.cos(theta);

        // check matrix correctness
        assertEquals(rotMatrix.getRows(),
                Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS);
        assertEquals(rotMatrix.getColumns(),
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        assertEquals(rotMatrix.getElementAt(0, 0), cosTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(1, 0), sinTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(0, 1), -sinTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(1, 1), cosTheta, 0.0);

        // build rotation from matrix and check correctness
        Rotation2D rotation2 = new Rotation2D(rotMatrix);
        assertEquals(rotation2.getTheta(), theta, ABSOLUTE_ERROR);


        // try again providing matrix where rotation will be stored
        rotMatrix = new Matrix(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        rotation.asInhomogeneousMatrix(rotMatrix);

        // check matrix correctness
        assertEquals(rotMatrix.getRows(),
                Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS);
        assertEquals(rotMatrix.getColumns(),
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        assertEquals(rotMatrix.getElementAt(0, 0), cosTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(1, 0), sinTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(0, 1), -sinTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(1, 1), cosTheta, 0.0);

        // build rotation from matrix and check correctness
        rotation2 = new Rotation2D(rotMatrix);
        assertEquals(rotation2.getTheta(), theta, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        rotMatrix = new Matrix(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        try {
            rotation.asInhomogeneousMatrix(rotMatrix);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testAsHomogeneousMatrix()
            throws InvalidRotationMatrixException, WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final Rotation2D rotation = new Rotation2D(theta);

        Matrix rotMatrix = rotation.asHomogeneousMatrix();

        final double sinTheta = Math.sin(theta);
        final double cosTheta = Math.cos(theta);

        // check matrix correctness
        assertEquals(rotMatrix.getRows(),
                Rotation2D.ROTATION2D_HOM_MATRIX_ROWS);
        assertEquals(rotMatrix.getColumns(),
                Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        assertEquals(rotMatrix.getElementAt(0, 0), cosTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(1, 0), sinTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(2, 0), 0.0, 0.0);
        assertEquals(rotMatrix.getElementAt(0, 1), -sinTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(1, 1), cosTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(2, 1), 0.0, 0.0);
        assertEquals(rotMatrix.getElementAt(0, 2), 0.0, 0.0);
        assertEquals(rotMatrix.getElementAt(1, 2), 0.0, 0.0);
        assertEquals(rotMatrix.getElementAt(2, 2), 1.0, 0.0);

        // build rotation from matrix and check correctness
        Rotation2D rotation2 = new Rotation2D(rotMatrix);
        assertEquals(rotation2.getTheta(), theta, ABSOLUTE_ERROR);

        // try again providing matrix where rotation will be stored
        rotMatrix = new Matrix(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        rotation.asHomogeneousMatrix(rotMatrix);

        // check matrix correctness
        assertEquals(rotMatrix.getRows(),
                Rotation2D.ROTATION2D_HOM_MATRIX_ROWS);
        assertEquals(rotMatrix.getColumns(),
                Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        assertEquals(rotMatrix.getElementAt(0, 0), cosTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(1, 0), sinTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(2, 0), 0.0, 0.0);
        assertEquals(rotMatrix.getElementAt(0, 1), -sinTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(1, 1), cosTheta, 0.0);
        assertEquals(rotMatrix.getElementAt(2, 1), 0.0, 0.0);
        assertEquals(rotMatrix.getElementAt(0, 2), 0.0, 0.0);
        assertEquals(rotMatrix.getElementAt(1, 2), 0.0, 0.0);
        assertEquals(rotMatrix.getElementAt(2, 2), 1.0, 0.0);

        // build rotation from matrix and check correctness
        rotation2 = new Rotation2D(rotMatrix);
        assertEquals(rotation2.getTheta(), theta, ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        rotMatrix = new Matrix(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        try {
            rotation.asHomogeneousMatrix(rotMatrix);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFromMatrixAndIsValidRotationMatrix()
            throws WrongSizeException, InvalidRotationMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final Rotation2D rotation = new Rotation2D();
        assertEquals(rotation.getTheta(), 0.0, 0.0);

        // test from inhomogeneous rotation matrix
        final Matrix inhomRotMat = new Matrix(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        final double cosTheta = Math.cos(theta);
        final double sinTheta = Math.sin(theta);
        inhomRotMat.setElementAt(0, 0, cosTheta);
        inhomRotMat.setElementAt(1, 0, sinTheta);
        inhomRotMat.setElementAt(0, 1, -sinTheta);
        inhomRotMat.setElementAt(1, 1, cosTheta);

        // check rotation matrix is valid
        assertTrue(Rotation2D.isValidRotationMatrix(inhomRotMat));
        assertTrue(Rotation2D.isValidRotationMatrix(inhomRotMat,
                ABSOLUTE_ERROR));

        // set rotation from inhomogeneous matrix
        rotation.fromMatrix(inhomRotMat);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // reset theta
        rotation.setTheta(0.0);
        assertEquals(rotation.getTheta(), 0.0, 0.0);

        // try again with threshold
        rotation.fromMatrix(inhomRotMat, ABSOLUTE_ERROR);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // reset theta
        rotation.setTheta(0.0);
        assertEquals(rotation.getTheta(), 0.0, 0.0);

        // test from homogeneous rotation matrix
        final Matrix homRotMat = Matrix.identity(
                Rotation2D.ROTATION2D_HOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        homRotMat.setSubmatrix(0, 0, 1, 1, inhomRotMat);

        // check rotation matrix is valid
        assertTrue(Rotation2D.isValidRotationMatrix(homRotMat));
        assertTrue(Rotation2D.isValidRotationMatrix(homRotMat,
                ABSOLUTE_ERROR));

        rotation.fromMatrix(homRotMat);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // reset theta
        rotation.setTheta(0.0);
        assertEquals(rotation.getTheta(), 0.0, 0.0);

        // try again with threshold
        rotation.fromMatrix(homRotMat, ABSOLUTE_ERROR);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // Force IllegalArgumentException by using a negative threshold
        try {
            rotation.fromMatrix(homRotMat, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force InvalidRotationMatrix
        // because of wrong size
        Matrix m = new Matrix(2, 3);
        assertFalse(Rotation2D.isValidRotationMatrix(m));
        assertFalse(Rotation2D.isValidRotationMatrix(m, ABSOLUTE_ERROR));
        try {
            rotation.fromMatrix(m);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }
        try {
            rotation.fromMatrix(m, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }

        // because determinant is not 1
        m = Matrix.identity(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        m.multiplyByScalar(2.0);
        assertFalse(Rotation2D.isValidRotationMatrix(m));
        assertFalse(Rotation2D.isValidRotationMatrix(m, ABSOLUTE_ERROR));
        try {
            rotation.fromMatrix(m);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }
        try {
            rotation.fromMatrix(m, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }

        // Force IllegalArgumentException into isValidRotationMatrix because
        // threshold is negative
        try {
            Rotation2D.isValidRotationMatrix(m, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFromInhomogeneousMatrixAndIsValidRotationMatrix()
            throws WrongSizeException, InvalidRotationMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final Rotation2D rotation = new Rotation2D();
        assertEquals(rotation.getTheta(), 0.0, 0.0);

        // test from inhomogeneous rotation matrix
        final Matrix inhomRotMat = new Matrix(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        final double cosTheta = Math.cos(theta);
        final double sinTheta = Math.sin(theta);
        inhomRotMat.setElementAt(0, 0, cosTheta);
        inhomRotMat.setElementAt(1, 0, sinTheta);
        inhomRotMat.setElementAt(0, 1, -sinTheta);
        inhomRotMat.setElementAt(1, 1, cosTheta);

        // check rotation matrix is valid
        assertTrue(Rotation2D.isValidRotationMatrix(inhomRotMat));
        assertTrue(Rotation2D.isValidRotationMatrix(inhomRotMat,
                ABSOLUTE_ERROR));

        // set rotation from inhomogeneous matrix
        rotation.fromInhomogeneousMatrix(inhomRotMat);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // reset theta
        rotation.setTheta(0.0);
        assertEquals(rotation.getTheta(), 0.0, 0.0);

        // try again with threshold
        rotation.fromInhomogeneousMatrix(inhomRotMat, ABSOLUTE_ERROR);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // Force IllegalArgumentException by using a negative threshold
        try {
            rotation.fromInhomogeneousMatrix(inhomRotMat, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force InvalidRotationMatrix
        // because of wrong size
        Matrix m = new Matrix(2, 3);
        assertFalse(Rotation2D.isValidRotationMatrix(m));
        assertFalse(Rotation2D.isValidRotationMatrix(m, ABSOLUTE_ERROR));
        try {
            rotation.fromInhomogeneousMatrix(m);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }
        try {
            rotation.fromInhomogeneousMatrix(m, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }

        // because determinant is not 1
        m = Matrix.identity(Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS);
        m.multiplyByScalar(2.0);
        assertFalse(Rotation2D.isValidRotationMatrix(m));
        assertFalse(Rotation2D.isValidRotationMatrix(m, ABSOLUTE_ERROR));
        try {
            rotation.fromInhomogeneousMatrix(m);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }
        try {
            rotation.fromInhomogeneousMatrix(m, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }

        // Force IllegalArgumentException into isValidRotationMatrix because
        // threshold is negative
        try {
            Rotation2D.isValidRotationMatrix(m, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testFromHomogeneousMatrixAndIsValidRotationMatrix()
            throws WrongSizeException, InvalidRotationMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final Rotation2D rotation = new Rotation2D();
        assertEquals(rotation.getTheta(), 0.0, 0.0);

        // test from inhomogeneous rotation matrix
        final Matrix homRotMat = new Matrix(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        final double cosTheta = Math.cos(theta);
        final double sinTheta = Math.sin(theta);
        homRotMat.setElementAt(0, 0, cosTheta);
        homRotMat.setElementAt(1, 0, sinTheta);
        homRotMat.setElementAt(2, 0, 0.0);
        homRotMat.setElementAt(0, 1, -sinTheta);
        homRotMat.setElementAt(1, 1, cosTheta);
        homRotMat.setElementAt(2, 1, 0.0);
        homRotMat.setElementAt(0, 2, 0.0);
        homRotMat.setElementAt(1, 2, 0.0);
        homRotMat.setElementAt(2, 2, 1.0);

        // check rotation matrix is valid
        assertTrue(Rotation2D.isValidRotationMatrix(homRotMat));
        assertTrue(Rotation2D.isValidRotationMatrix(homRotMat,
                ABSOLUTE_ERROR));

        // set rotation from homogeneous matrix
        rotation.fromHomogeneousMatrix(homRotMat);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // reset theta
        rotation.setTheta(0.0);
        assertEquals(rotation.getTheta(), 0.0, 0.0);

        // try again with threshold
        rotation.fromHomogeneousMatrix(homRotMat, ABSOLUTE_ERROR);
        assertEquals(rotation.getTheta(), theta, ABSOLUTE_ERROR);

        // Force IllegalArgumentException by using a negative threshold
        try {
            rotation.fromHomogeneousMatrix(homRotMat, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }

        // Force InvalidRotationMatrix
        // because of wrong size
        Matrix m = new Matrix(2, 3);
        assertFalse(Rotation2D.isValidRotationMatrix(m));
        assertFalse(Rotation2D.isValidRotationMatrix(m, ABSOLUTE_ERROR));
        try {
            rotation.fromHomogeneousMatrix(m);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }
        try {
            rotation.fromHomogeneousMatrix(m, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }

        // because determinant is not 1
        m = Matrix.identity(Rotation2D.ROTATION2D_HOM_MATRIX_ROWS,
                Rotation2D.ROTATION2D_HOM_MATRIX_COLS);
        m.multiplyByScalar(2.0);
        assertFalse(Rotation2D.isValidRotationMatrix(m));
        assertFalse(Rotation2D.isValidRotationMatrix(m, ABSOLUTE_ERROR));
        try {
            rotation.fromHomogeneousMatrix(m);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }
        try {
            rotation.fromHomogeneousMatrix(m, ABSOLUTE_ERROR);
            fail("InvalidRotationMatrixException expected but not thrown");
        } catch (final InvalidRotationMatrixException ignore) {
        }

        // Force IllegalArgumentException into isValidRotationMatrix because
        // threshold is negative
        try {
            Rotation2D.isValidRotationMatrix(m, -ABSOLUTE_ERROR);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testRotate() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final Rotation2D rotation = new Rotation2D(theta);

        // create 2 random points
        final HomogeneousPoint2D point1 = new HomogeneousPoint2D();
        point1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final HomogeneousPoint2D point2 = new HomogeneousPoint2D();
        point2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // ensure that points are not coincident
        while (point1.equals(point2)) {
            point2.setInhomogeneousCoordinates(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        }

        // create line passing through both points
        final Line2D line = new Line2D(point1, point2);
        assertTrue(line.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(line.isLocus(point2, ABSOLUTE_ERROR));

        // now rotate points and line
        final Point2D rotPoint1A = rotation.rotate(point1);
        final Point2D rotPoint1B = Point2D.create();
        rotation.rotate(point1, rotPoint1B);

        final Point2D rotPoint2A = rotation.rotate(point2);
        final Point2D rotPoint2B = Point2D.create();
        rotation.rotate(point2, rotPoint2B);

        // check that rotated points A and B are equal
        assertTrue(rotPoint1A.equals(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotPoint2A.equals(rotPoint2B, ABSOLUTE_ERROR));

        // check points have been correctly rotated
        final Matrix point1Mat = Matrix.newFromArray(point1.asArray(), true);
        final Matrix point2Mat = Matrix.newFromArray(point2.asArray(), true);
        final Matrix r = rotation.asHomogeneousMatrix();
        final Matrix rotPoint1Mat = r.multiplyAndReturnNew(point1Mat);
        final Matrix rotPoint2Mat = r.multiplyAndReturnNew(point2Mat);

        // check correctness
        double scaleX = rotPoint1A.getHomX() /
                rotPoint1Mat.getElementAtIndex(0);
        double scaleY = rotPoint1A.getHomY() /
                rotPoint1Mat.getElementAtIndex(1);
        double scaleW = rotPoint1A.getHomW() /
                rotPoint1Mat.getElementAtIndex(2);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        scaleX = rotPoint2A.getHomX() / rotPoint2Mat.getElementAtIndex(0);
        scaleY = rotPoint2A.getHomY() / rotPoint2Mat.getElementAtIndex(1);
        scaleW = rotPoint2A.getHomW() / rotPoint2Mat.getElementAtIndex(2);
        assertEquals(scaleX, scaleY, ABSOLUTE_ERROR);
        assertEquals(scaleY, scaleW, ABSOLUTE_ERROR);
        assertEquals(scaleW, scaleX, ABSOLUTE_ERROR);

        // ensure that points where correctly rotated using inhomogeneous
        // coordinates
        final Matrix inhomPoint = new Matrix(INHOM_COORDS, 1);
        inhomPoint.setElementAtIndex(0, point1.getInhomX());
        inhomPoint.setElementAtIndex(1, point1.getInhomY());
        final Matrix inhomR = rotation.asInhomogeneousMatrix();
        final Matrix inhomRotPoint = inhomR.multiplyAndReturnNew(inhomPoint);
        final Point2D rotP = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES,
                inhomRotPoint.toArray());
        assertTrue(rotP.equals(rotPoint1A, ABSOLUTE_ERROR));

        final Line2D rotLineA = rotation.rotate(line);
        final Line2D rotLineB = new Line2D();
        rotation.rotate(line, rotLineB);

        // check both rotated lines are equal
        double scaleA = rotLineA.getA() / rotLineB.getA();
        double scaleB = rotLineA.getB() / rotLineB.getB();
        double scaleC = rotLineA.getC() / rotLineB.getC();

        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleA, ABSOLUTE_ERROR);

        // check line has been correctly rotated by ensuring that rotated points
        // belong into rotated line
        assertTrue(rotLineA.isLocus(rotPoint1A, ABSOLUTE_ERROR));
        assertTrue(rotLineA.isLocus(rotPoint1B, ABSOLUTE_ERROR));
        assertTrue(rotLineA.isLocus(rotPoint2A, ABSOLUTE_ERROR));
        assertTrue(rotLineA.isLocus(rotPoint2B, ABSOLUTE_ERROR));

        // and by ensuring that rotated line follow appropriate equation
        final Matrix lineMat = Matrix.newFromArray(line.asArray(), true);
        final Matrix rotLineMat = r.multiplyAndReturnNew(lineMat);

        scaleA = rotLineA.getA() / rotLineMat.getElementAtIndex(0);
        scaleB = rotLineA.getB() / rotLineMat.getElementAtIndex(1);
        scaleC = rotLineA.getC() / rotLineMat.getElementAtIndex(2);

        assertEquals(scaleA, scaleB, ABSOLUTE_ERROR);
        assertEquals(scaleB, scaleC, ABSOLUTE_ERROR);
        assertEquals(scaleC, scaleA, ABSOLUTE_ERROR);
    }

    @Test
    public void testCombine() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta1 = randomizer.nextDouble(MIN_THETA, MAX_THETA);
        final double theta2 = randomizer.nextDouble(MIN_THETA, MAX_THETA);

        final Rotation2D rot1 = new Rotation2D(theta1);
        final Rotation2D rot2 = new Rotation2D(theta2);

        final Rotation2D rot = new Rotation2D();
        Rotation2D.combine(rot1, rot2, rot);
        assertEquals(rot.getTheta(), theta1 + theta2, 0.0);

        final Rotation2D rot3 = rot.combineAndReturnNew(rot2);
        assertEquals(rot3.getTheta(), theta1 + 2.0 * theta2, ABSOLUTE_ERROR);

        rot.combine(rot1);
        assertEquals(rot.getTheta(), 2.0 * theta1 + theta2, ABSOLUTE_ERROR);
    }

    @Test
    public void testEqualsAndHashCode() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_THETA, MAX_THETA);
        final double threshold = randomizer.nextDouble(
                Rotation2D.DEFAULT_COMPARISON_THRESHOLD,
                2.0 * Rotation2D.DEFAULT_COMPARISON_THRESHOLD);
        final double theta2 = theta + threshold;

        // test from rotation angle
        final Rotation2D rotation1 = new Rotation2D(theta);
        assertEquals(rotation1.getTheta(), theta, 0.0);

        final Rotation2D rotation2 = new Rotation2D(theta);
        final Rotation2D rotation3 = new Rotation2D(theta2);

        // check equalness
        assertEquals(rotation1, rotation1);
        assertEquals(rotation1.hashCode(), rotation1.hashCode());

        assertEquals(rotation1, rotation2);
        assertEquals(rotation1.hashCode(), rotation2.hashCode());

        assertNotEquals(rotation1, rotation3);
        assertTrue(rotation1.equals(rotation2,
                Rotation2D.DEFAULT_COMPARISON_THRESHOLD));
        assertFalse(rotation1.equals(rotation3,
                Rotation2D.DEFAULT_COMPARISON_THRESHOLD));
        // check with larger threshold
        assertTrue(rotation1.equals(rotation3, 2.0 * threshold));
    }
}
