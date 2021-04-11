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

import com.irurueta.algebra.Utils;
import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class MetricTransformation2DTest {

    private static final int HOM_COORDS = 3;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final int MIN_POINTS = 3;
    private static final int MAX_POINTS = 50;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final double MIN_ANGLE_DEGREES2 = -90.0;
    private static final double MAX_ANGLE_DEGREES2 = 90.0;

    private static final double MIN_TRANSLATION2 = -100.0;
    private static final double MAX_TRANSLATION2 = 100.0;

    private static final double MIN_SCALE2 = 0.5;
    private static final double MAX_SCALE2 = 2.0;

    @Test
    public void testConstructor() throws CoincidentPointsException {

        // Test empty constructor
        MetricTransformation2D transformation = new MetricTransformation2D();

        // check correctness
        assertEquals(transformation.getRotation().getTheta(), 0.0, 0.0);
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getScale(),
                MetricTransformation2D.DEFAULT_SCALE, 0.0);

        // Test constructor with rotation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        Rotation2D rotation = new Rotation2D(theta);

        transformation = new MetricTransformation2D(rotation);

        // check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, 0.0);
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getScale(),
                MetricTransformation2D.DEFAULT_SCALE, 0.0);

        // Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation2D((Rotation2D) null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        assertNull(transformation);


        // Test constructor with translation
        double[] translation =
                new double[MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        transformation = new MetricTransformation2D(translation);

        // check correctness
        assertEquals(transformation.getRotation().getTheta(), 0.0, 0.0);
        assertEquals(transformation.getTranslation().length, translation.length);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getScale(),
                MetricTransformation2D.DEFAULT_SCALE, 0.0);

        // Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation2D((double[]) null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // Force IllegalArgumentException
        final double[] badTranslation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation = new MetricTransformation2D(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(transformation);

        // Test constructor with scale
        double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        transformation = new MetricTransformation2D(scale);

        // check correctness
        assertEquals(transformation.getRotation().getTheta(), 0.0, 0.0);
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getScale(), scale, 0.0);

        // Test constructor with rotation, translation and scale
        transformation = new MetricTransformation2D(rotation, translation,
                scale);

        // check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, 0.0);
        assertEquals(transformation.getTranslation().length, translation.length);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getScale(), scale, 0.0);

        // Force NullPointerException
        transformation = null;
        try {
            transformation = new MetricTransformation2D(null, translation,
                    scale);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            transformation = new MetricTransformation2D(rotation, null, scale);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        assertNull(transformation);

        // Force IllegalArgumentException
        transformation = null;
        try {
            transformation = new MetricTransformation2D(rotation,
                    badTranslation, scale);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(transformation);

        // test constructor with corresponding points
        theta = randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;
        rotation = new Rotation2D(theta);
        translation =
                new double[MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);
        scale = randomizer.nextDouble(MIN_SCALE2, MAX_SCALE2);

        final InhomogeneousPoint2D inputPoint1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final InhomogeneousPoint2D inputPoint2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final InhomogeneousPoint2D inputPoint3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        transformation = new MetricTransformation2D(rotation, translation,
                scale);

        final Point2D outputPoint1 = transformation.transformAndReturnNew(
                inputPoint1);
        final Point2D outputPoint2 = transformation.transformAndReturnNew(
                inputPoint2);
        final Point2D outputPoint3 = transformation.transformAndReturnNew(
                inputPoint3);

        final MetricTransformation2D transformation2 =
                new MetricTransformation2D(inputPoint1, inputPoint2,
                        inputPoint3, outputPoint1, outputPoint2,
                        outputPoint3);

        // check correctness
        final Rotation2D rotation2 = transformation2.getRotation();
        final double[] translation2 = transformation2.getTranslation();

        assertEquals(rotation2.getTheta(), rotation.getTheta(),
                ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(transformation2.getScale(), scale, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetRotation() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Rotation2D rotation = new Rotation2D(theta);

        // test default values
        assertEquals(transformation.getRotation().getTheta(), 0.0, 0.0);

        // set new value
        transformation.setRotation(rotation);

        // check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, 0.0);

        // Force NullPointerException
        try {
            transformation.setRotation(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testAddRotation() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double combinedTheta = theta1 + theta2;

        final Rotation2D rotation1 = new Rotation2D(theta1);
        final Rotation2D rotation2 = new Rotation2D(theta2);

        // set rotation1
        transformation.setRotation(rotation1);

        // check correctness
        assertEquals(transformation.getRotation().getTheta(), theta1, 0.0);

        // add second rotation
        transformation.addRotation(rotation2);

        // check correctness
        assertEquals(transformation.getRotation().getTheta(), combinedTheta,
                0.0);
    }

    @Test
    public void testGetSetTranslation() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);

        // Force IllegalArgumentException
        final double[] badTranslation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS + 1];

        try {
            transformation.setTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testAddTranslation() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] translation1 = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double[] translation2 = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);

        // set new value
        final double[] translationCopy = Arrays.copyOf(translation1,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation1[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation1[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation1[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation1[1], 0.0);

        // add translation
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation1[0] +
                translation2[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation1[1] +
                translation2[1], 0.0);
        assertEquals(transformation.getTranslationX(), translation1[0] +
                translation2[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation1[1] +
                translation2[1], 0.0);

        // Force IllegalArgumentException
        final double[] badTranslation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation.addTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTranslationX() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);

        // set new value
        transformation.setTranslationX(translationX);

        // check correctness
        assertEquals(transformation.getTranslationX(), translationX, 0.0);
    }

    @Test
    public void testGetSetTranslationY() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);

        // set new value
        transformation.setTranslationY(translationY);

        // check correctness
        assertEquals(transformation.getTranslationY(), translationY, 0.0);
    }

    @Test
    public void testSetTranslationCoordinates() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translationX, translationY);

        // check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translationX,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translationY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translationX,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translationY,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetTranslationPoint() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final InhomogeneousPoint2D translation = new InhomogeneousPoint2D(
                translationX, translationY);

        // check default value
        assertEquals(transformation.getTranslation().length,
                MetricTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translationX,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translationY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translationX,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translationY,
                ABSOLUTE_ERROR);

        final Point2D translation2 = transformation.getTranslationPoint();
        final Point2D translation3 = Point2D.create();
        transformation.getTranslationPoint(translation3);

        // check correctness
        assertEquals(translation, translation2);
        assertEquals(translation, translation3);
    }

    @Test
    public void testAddTranslationX() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);

        // set value
        transformation.setTranslationX(translationX1);

        // check correctness
        assertEquals(transformation.getTranslationX(), translationX1, 0.0);

        // add translation x
        transformation.addTranslationX(translationX2);

        // check correctness
        assertEquals(transformation.getTranslationX(),
                translationX1 + translationX2, 0.0);
    }

    @Test
    public void testAddTranslationY() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);

        // set value
        transformation.setTranslationY(translationY1);

        // check correctness
        assertEquals(transformation.getTranslationY(), translationY1, 0.0);

        // add translation y
        transformation.addTranslationY(translationY2);

        // check correctness
        assertEquals(transformation.getTranslationY(),
                translationY1 + translationY2, 0.0);
    }

    @Test
    public void testAddTranslationCoordinates() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);

        // set values
        transformation.setTranslation(translationX1, translationY1);

        // add translation
        transformation.addTranslation(translationX2, translationY2);

        // check correctness
        assertEquals(transformation.getTranslationX(),
                translationX1 + translationX2, 0.0);
        assertEquals(transformation.getTranslationY(),
                translationY1 + translationY2, 0.0);
    }

    @Test
    public void testAddTranslationPoint() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);

        // set values
        transformation.setTranslation(translationX1, translationY1);

        // add translation
        final Point2D translation2 = new InhomogeneousPoint2D(translationX2,
                translationY2);
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(transformation.getTranslationX(),
                translationX1 + translationX2, 0.0);
        assertEquals(transformation.getTranslationY(),
                translationY1 + translationY2, 0.0);
    }

    @Test
    public void testGetSetScale() {
        final MetricTransformation2D transformation = new MetricTransformation2D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getScale(),
                MetricTransformation2D.DEFAULT_SCALE, 0.0);

        // set value
        transformation.setScale(scale);

        // check correctness
        assertEquals(transformation.getScale(), scale, 0.0);
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final Rotation2D rotation = new Rotation2D(theta);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final Matrix m = Matrix.identity(3, 3);
        m.setSubmatrix(0, 0, 1, 1,
                rotation.asInhomogeneousMatrix().multiplyByScalarAndReturnNew(
                        scale));
        m.setSubmatrix(0, 2, 1, 2, translation);

        final Matrix transMatrix1 = transformation.asMatrix();
        final Matrix transMatrix2 = new Matrix(MetricTransformation2D.HOM_COORDS,
                MetricTransformation2D.HOM_COORDS);
        transformation.asMatrix(transMatrix2);

        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));

        final Matrix t = new Matrix(MetricTransformation2D.HOM_COORDS + 1,
                MetricTransformation2D.HOM_COORDS + 1);
        try {
            transformation.asMatrix(t);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testTransformPoint() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] coords = new double[
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final Point2D point = Point2D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Point2D expectedPoint = Point2D.create();
        transformPoint(point, expectedPoint, rotation, translation, scale);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final Point2D outPoint1 = transformation.transformAndReturnNew(point);
        final Point2D outPoint2 = Point2D.create();
        transformation.transform(point, outPoint2);

        // check correctness
        assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
        assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));

        // update point
        transformation.transform(point);

        // check correctness
        assertTrue(point.equals(expectedPoint, ABSOLUTE_ERROR));
    }

    @Test
    public void testTransformPoints() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final ArrayList<Point2D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point2D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final List<Point2D> outPoints1 = transformation.transformPointsAndReturnNew(inputPoints);
        final List<Point2D> outPoints2 = new ArrayList<>();
        transformation.transformPoints(inputPoints, outPoints2);

        // check correctness
        assertEquals(outPoints1.size(), inputPoints.size());
        assertEquals(outPoints2.size(), inputPoints.size());
        for (int i = 0; i < size; i++) {
            final Point2D expectedPoint = expectedPoints.get(i);

            final Point2D outPoint1 = outPoints1.get(i);
            final Point2D outPoint2 = outPoints2.get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testTransformAndOverwritePoints() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final ArrayList<Point2D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point2D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        transformation.transformAndOverwritePoints(inputPoints);

        // check correctness
        assertEquals(inputPoints.size(), size);
        for (int i = 0; i < size; i++) {
            final Point2D expectedPoint = expectedPoints.get(i);

            final Point2D point = inputPoints.get(i);

            assertTrue(point.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testTransformConic() throws AlgebraException,
            NonSymmetricMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // create input conic
        // Constructor with params
        final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Conic conic = new Conic(a, b, c, d, e, f);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        // compute expected value
        final Conic expectedConic = new Conic();
        transformConic(conic, expectedConic, transformation);
        expectedConic.normalize();

        // make transformation
        final Conic outConic1 = transformation.transformAndReturnNew(conic);
        final Conic outConic2 = new Conic();
        transformation.transform(conic, outConic2);

        // check correctness
        outConic1.normalize();
        outConic2.normalize();

        assertEquals(expectedConic.getA(), outConic1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getB(), outConic1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getC(), outConic1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getD(), outConic1.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getE(), outConic1.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getF(), outConic1.getF(), ABSOLUTE_ERROR);

        assertEquals(expectedConic.getA(), outConic2.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getB(), outConic2.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getC(), outConic2.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getD(), outConic2.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getE(), outConic2.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getF(), outConic2.getF(), ABSOLUTE_ERROR);

        transformation.transform(conic);

        // check correctness
        conic.normalize();

        assertEquals(expectedConic.getA(), conic.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getB(), conic.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getC(), conic.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getD(), conic.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getE(), conic.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getF(), conic.getF(), ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformConicAndPoints() throws AlgebraException,
            GeometryException {

        // create Conic from 5 points
        Conic conic = null;
        Point2D point1;
        Point2D point2;
        Point2D point3;
        Point2D point4;
        Point2D point5;
        do {
            final Matrix m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                    m.getElementAt(0, 1), 1.0);
            point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                    m.getElementAt(1, 1), 1.0);
            point3 = new HomogeneousPoint2D(m.getElementAt(2, 0),
                    m.getElementAt(2, 1), 1.0);
            point4 = new HomogeneousPoint2D(m.getElementAt(3, 0),
                    m.getElementAt(3, 1), 1.0);
            point5 = new HomogeneousPoint2D(m.getElementAt(4, 0),
                    m.getElementAt(4, 1), 1.0);

            try {
                conic = new Conic(point1, point2, point3, point4, point5);
            } catch (final GeometryException ignore) {
            }
        } while (conic == null);

        // check that points belong to conic
        assertTrue(conic.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point3, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point4, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point5, ABSOLUTE_ERROR));

        // create transformation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        // compute expected value
        final Conic expectedConic = new Conic();
        transformConic(conic, expectedConic, transformation);
        expectedConic.normalize();

        // transform conic and points
        final Conic outConic = transformation.transformAndReturnNew(conic);
        final Point2D outPoint1 = transformation.transformAndReturnNew(point1);
        final Point2D outPoint2 = transformation.transformAndReturnNew(point2);
        final Point2D outPoint3 = transformation.transformAndReturnNew(point3);
        final Point2D outPoint4 = transformation.transformAndReturnNew(point4);
        final Point2D outPoint5 = transformation.transformAndReturnNew(point5);

        // check that transformed points still belong to transformed conic
        assertTrue(outConic.isLocus(outPoint1, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint2, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint3, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint4, ABSOLUTE_ERROR));
        assertTrue(outConic.isLocus(outPoint5, ABSOLUTE_ERROR));

        // check conic correctness
        outConic.normalize();

        assertEquals(expectedConic.getA(), outConic.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getB(), outConic.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getC(), outConic.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getD(), outConic.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getE(), outConic.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedConic.getF(), outConic.getF(), ABSOLUTE_ERROR);
    }

    @Test
    public void tesTransformDualConic() throws NonSymmetricMatrixException,
            AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // create input conic
        // Constructor with params
        final double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final DualConic dualConic = new DualConic(a, b, c, d, e, f);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        // compute expected value
        final DualConic expectedDualConic = new DualConic();
        transformDualConic(dualConic, expectedDualConic, transformation);
        expectedDualConic.normalize();

        // make transformation
        final DualConic outDualConic1 = transformation.transformAndReturnNew(dualConic);
        final DualConic outDualConic2 = new DualConic();
        transformation.transform(dualConic, outDualConic2);

        // check correctness
        outDualConic1.normalize();
        outDualConic2.normalize();

        assertEquals(expectedDualConic.getA(), outDualConic1.getA(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), outDualConic1.getB(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), outDualConic1.getC(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), outDualConic1.getD(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), outDualConic1.getE(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), outDualConic1.getF(),
                ABSOLUTE_ERROR);

        assertEquals(expectedDualConic.getA(), outDualConic2.getA(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), outDualConic2.getB(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), outDualConic2.getC(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), outDualConic2.getD(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), outDualConic2.getE(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), outDualConic2.getF(),
                ABSOLUTE_ERROR);

        transformation.transform(dualConic);

        // check correctness
        dualConic.normalize();

        assertEquals(expectedDualConic.getA(), dualConic.getA(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), dualConic.getB(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), dualConic.getC(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), dualConic.getD(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), dualConic.getE(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), dualConic.getF(),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformDualConicAndLines() throws AlgebraException,
            GeometryException {

        // create dual conic from 5 lines
        DualConic dualConic = null;
        Line2D line1;
        Line2D line2;
        Line2D line3;
        Line2D line4;
        Line2D line5;
        do {
            final Matrix m = Matrix.createWithUniformRandomValues(5, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            line1 = new Line2D(m.getElementAt(0, 0),
                    m.getElementAt(0, 1),
                    m.getElementAt(0, 2));
            line2 = new Line2D(m.getElementAt(1, 0),
                    m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            line3 = new Line2D(m.getElementAt(2, 0),
                    m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            line4 = new Line2D(m.getElementAt(3, 0),
                    m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            line5 = new Line2D(m.getElementAt(4, 0),
                    m.getElementAt(4, 1),
                    m.getElementAt(4, 2));

            line1.normalize();
            line2.normalize();
            line3.normalize();
            line4.normalize();
            line5.normalize();

            try {
                dualConic = new DualConic(line1, line2, line3, line4, line5);
            } catch (final GeometryException ignore) {
            }
        } while (dualConic == null);

        // check that lines belong to dual conic
        assertTrue(dualConic.isLocus(line1, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line2, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line3, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line4, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line5, ABSOLUTE_ERROR));

        // create transformation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        // compute expected value
        final DualConic expectedDualConic = new DualConic();
        transformDualConic(dualConic, expectedDualConic, transformation);
        expectedDualConic.normalize();

        // transform dual conic and lines
        final DualConic outDualConic = transformation.transformAndReturnNew(dualConic);
        final Line2D outLine1 = transformation.transformAndReturnNew(line1);
        final Line2D outLine2 = transformation.transformAndReturnNew(line2);
        final Line2D outLine3 = transformation.transformAndReturnNew(line3);
        final Line2D outLine4 = transformation.transformAndReturnNew(line4);
        final Line2D outLine5 = transformation.transformAndReturnNew(line5);

        // check that transformed lines still belong to transformed dual conic
        assertTrue(outDualConic.isLocus(outLine1, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine2, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine3, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine4, ABSOLUTE_ERROR));
        assertTrue(outDualConic.isLocus(outLine5, ABSOLUTE_ERROR));

        // check dual conic correctness
        outDualConic.normalize();

        assertEquals(expectedDualConic.getA(), outDualConic.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), outDualConic.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), outDualConic.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), outDualConic.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), outDualConic.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), outDualConic.getF(), ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformLine() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] params = new double[
                Line2D.LINE_NUMBER_PARAMS];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Line2D line = new Line2D(params);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final Line2D expectedLine = new Line2D();
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();

        final Line2D outLine1 = transformation.transformAndReturnNew(line);
        final Line2D outLine2 = new Line2D();
        transformation.transform(line, outLine2);

        outLine1.normalize();
        outLine2.normalize();

        // check correctness
        assertEquals(expectedLine.getA(), outLine1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), outLine1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), outLine1.getC(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getA(), outLine2.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), outLine2.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), outLine2.getC(), ABSOLUTE_ERROR);

        transformation.transform(line);

        line.normalize();

        // check correctness
        assertEquals(expectedLine.getA(), line.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), line.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), line.getC(), ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformLineAndPoints() throws AlgebraException {
        // create line from 2 points
        Matrix m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        // ensure we create a matrix with 2 non linear dependent rows
        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final Point2D point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        final Point2D point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));

        point1.normalize();
        point2.normalize();

        final Line2D line = new Line2D(point1, point2);

        // check that points belong to the line
        assertTrue(line.isLocus(point1));
        assertTrue(line.isLocus(point2));

        // create transformation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final Line2D expectedLine = new Line2D();
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();

        // transform line and points
        final Line2D outLine = transformation.transformAndReturnNew(line);
        final Point2D outPoint1 = transformation.transformAndReturnNew(point1);
        final Point2D outPoint2 = transformation.transformAndReturnNew(point2);

        // check that transformed points still belong to transformed line
        assertTrue(outLine.isLocus(outPoint1));
        assertTrue(outLine.isLocus(outPoint2));

        // check line correctness
        outLine.normalize();

        assertEquals(expectedLine.getA(), outLine.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), outLine.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), outLine.getC(), ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformLines() throws AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final ArrayList<Line2D> inputLines = new ArrayList<>(size);
        final ArrayList<Line2D> expectedLines = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] params = new double[Line2D.LINE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Line2D line = new Line2D(params);
            inputLines.add(line);

            final Line2D expectedLine = new Line2D();
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }

        final List<Line2D> outLines1 = transformation.transformLinesAndReturnNew(inputLines);
        final List<Line2D> outLines2 = new ArrayList<>();
        transformation.transformLines(inputLines, outLines2);

        // check correctness
        assertEquals(outLines1.size(), inputLines.size());
        assertEquals(outLines2.size(), inputLines.size());
        for (int i = 0; i < size; i++) {
            final Line2D expectedLine = expectedLines.get(i);

            final Line2D outLine1 = outLines1.get(i);
            final Line2D outLine2 = outLines2.get(i);

            expectedLine.normalize();
            outLine1.normalize();
            outLine2.normalize();

            // check correctness
            assertEquals(expectedLine.getA(), outLine1.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), outLine1.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), outLine1.getC(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getA(), outLine2.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), outLine2.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), outLine2.getC(), ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testTransformAndOverwriteLines() throws AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final ArrayList<Line2D> inputLines = new ArrayList<>(size);
        final ArrayList<Line2D> expectedLines = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] params = new double[Line2D.LINE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Line2D line = new Line2D(params);
            inputLines.add(line);

            final Line2D expectedLine = new Line2D();
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }

        transformation.transformAndOverwriteLines(inputLines);

        // check correctness
        assertEquals(inputLines.size(), size);
        for (int i = 0; i < size; i++) {
            final Line2D expectedLine = expectedLines.get(i);

            final Line2D line = inputLines.get(i);

            expectedLine.normalize();
            line.normalize();

            // check correctness
            assertEquals(expectedLine.getA(), line.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), line.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), line.getC(), ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testTransformPolygon() throws NotEnoughVerticesException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final ArrayList<Point2D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point2D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final Polygon2D inputPolygon = new Polygon2D(inputPoints);
        final Polygon2D expectedPolygon = new Polygon2D(expectedPoints);


        final Polygon2D outPolygon1 = transformation.transformAndReturnNew(inputPolygon);
        final Polygon2D outPolygon2 = new Polygon2D(inputPoints);
        transformation.transform(inputPolygon, outPolygon2);

        // check correctness
        assertEquals(outPolygon1.getVertices().size(),
                inputPolygon.getVertices().size());
        assertEquals(outPolygon2.getVertices().size(),
                inputPolygon.getVertices().size());
        for (int i = 0; i < size; i++) {
            final Point2D expectedPoint = expectedPolygon.getVertices().get(i);

            final Point2D outPoint1 = outPolygon1.getVertices().get(i);
            final Point2D outPoint2 = outPolygon2.getVertices().get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }

        transformation.transform(inputPolygon);

        // check correctness
        assertEquals(expectedPolygon.getVertices().size(),
                inputPolygon.getVertices().size());
        for (int i = 0; i < size; i++) {
            final Point2D expectedPoint = expectedPolygon.getVertices().get(i);

            final Point2D outPoint = outPolygon1.getVertices().get(i);

            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testTransformTriangle() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = Triangle2D.NUM_VERTICES;

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final ArrayList<Point2D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point2D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point2D point = Point2D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point2D expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, rotation, translation, scale);

            expectedPoints.add(expectedPoint);
        }

        final Triangle2D inputTriangle = new Triangle2D(inputPoints.get(0),
                inputPoints.get(1), inputPoints.get(2));
        final Triangle2D expectedTriangle = new Triangle2D(expectedPoints.get(0),
                expectedPoints.get(1), expectedPoints.get(2));

        final Triangle2D outTriangle1 = transformation.transformAndReturnNew(inputTriangle);
        final Triangle2D outTriangle2 = new Triangle2D(
                new InhomogeneousPoint2D(inputPoints.get(0)),
                new InhomogeneousPoint2D(inputPoints.get(1)),
                new InhomogeneousPoint2D(inputPoints.get(2)));
        transformation.transform(inputTriangle, outTriangle2);

        // check correctness
        for (int i = 0; i < size; i++) {
            final Point2D expectedPoint = expectedTriangle.getVertices().get(i);

            final Point2D outPoint1 = outTriangle1.getVertices().get(i);
            final Point2D outPoint2 = outTriangle2.getVertices().get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }

        transformation.transform(inputTriangle);

        // check correctness
        for (int i = 0; i < size; i++) {
            final Point2D expectedPoint = expectedTriangle.getVertices().get(i);

            final Point2D outPoint = inputTriangle.getVertices().get(i);

            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testInverse() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                MetricTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final Transformation2D invTransformation1 =
                transformation.inverseAndReturnNew();
        final MetricTransformation2D invTransformation2 =
                new MetricTransformation2D();
        transformation.inverse(invTransformation2);

        // check that inverse transformation matrix is the inverse matrix of
        // current transformation
        assertTrue(invTransformation1.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation2D.HOM_COORDS,
                MetricTransformation2D.HOM_COORDS), ABSOLUTE_ERROR));

        assertTrue(invTransformation2.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation2D.HOM_COORDS,
                MetricTransformation2D.HOM_COORDS), ABSOLUTE_ERROR));

        // test transforming a random point by transformation and then by its
        // inverse to ensure it remains the same
        final double[] params = new double[
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Point2D inputPoint = Point2D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, params);

        final Point2D transfPoint = transformation.transformAndReturnNew(inputPoint);

        final Point2D invTransfPoint1 = invTransformation1.transformAndReturnNew(transfPoint);
        final Point2D invTransfPoint2 = invTransformation2.transformAndReturnNew(transfPoint);

        // check correctness
        assertTrue(inputPoint.equals(invTransfPoint1, ABSOLUTE_ERROR));
        assertTrue(inputPoint.equals(invTransfPoint2, ABSOLUTE_ERROR));

        // try inverting original transformation
        transformation.inverse();
        final Point2D outPoint = transformation.transformAndReturnNew(transfPoint);

        assertTrue(inputPoint.equals(outPoint, ABSOLUTE_ERROR));
    }

    @Test
    public void testToMetric() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final Matrix expectedMatrix = transformation.asMatrix();

        final Matrix metricMatrix = transformation.toMetric().asMatrix();

        // check correctness
        assertTrue(expectedMatrix.equals(metricMatrix, ABSOLUTE_ERROR));
    }

    @Test
    public void testToAffine() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation = new double[
                EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scale = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final Matrix expectedMatrix = transformation.asMatrix();

        final Matrix metricMatrix = transformation.toAffine().asMatrix();

        // check correctness
        assertTrue(expectedMatrix.equals(metricMatrix, ABSOLUTE_ERROR));
    }

    @Test
    public void testCombine() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final Rotation2D rotation1 = new Rotation2D(theta1);
        final Rotation2D rotation2 = new Rotation2D(theta2);

        final double[] translation1 = new double[
                EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double[] translation2 = new double[
                EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final double scale1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double scale2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        MetricTransformation2D transformation1 =
                new MetricTransformation2D(rotation1, translation1, scale1);
        final MetricTransformation2D transformation2 =
                new MetricTransformation2D(rotation2, translation2, scale2);

        Matrix expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(
                transformation2.asMatrix());

        Rotation2D expectedRotation = rotation1.combineAndReturnNew(rotation2);
        Matrix rotM1 = rotation1.asInhomogeneousMatrix();
        Matrix t2 = Matrix.newFromArray(translation2, true);
        rotM1.multiply(t2);
        rotM1.multiplyByScalar(scale1);
        double[] expectedTranslation = rotM1.toArray();
        ArrayUtils.sum(expectedTranslation, translation1, expectedTranslation);
        double expectedScale = scale1 * scale2;

        // combine and return result as a new transformation
        MetricTransformation2D transformation3 =
                transformation1.combineAndReturnNew(transformation2);
        // combine into transformation1
        transformation1.combine(transformation2);

        // both matrices m1 and m3 need to be equal
        Matrix m3 = transformation3.asMatrix();
        Matrix m1 = transformation1.asMatrix();

        // check correctness
        assertTrue(m1.equals(m3, ABSOLUTE_ERROR));

        // besides, resulting transformation matrices need to be equal to
        // expected matrix
        assertTrue(m1.equals(expectedMatrix, ABSOLUTE_ERROR));
        assertTrue(m3.equals(expectedMatrix, ABSOLUTE_ERROR));

        // check also correctness of rotation and translation
        assertEquals(transformation1.getRotation().getTheta(),
                expectedRotation.getTheta(), ABSOLUTE_ERROR);
        assertEquals(transformation3.getRotation().getTheta(),
                expectedRotation.getTheta(), ABSOLUTE_ERROR);

        assertArrayEquals(transformation1.getTranslation(), expectedTranslation,
                ABSOLUTE_ERROR);
        assertArrayEquals(transformation3.getTranslation(), expectedTranslation,
                ABSOLUTE_ERROR);

        assertEquals(transformation1.getScale(), expectedScale, ABSOLUTE_ERROR);
        assertEquals(transformation3.getScale(), expectedScale, ABSOLUTE_ERROR);

        // now try combining with euclidean transformations
        transformation1 =
                new MetricTransformation2D(rotation1, translation1, scale1);
        EuclideanTransformation2D euclideanTransformation =
                new EuclideanTransformation2D(rotation2, translation2);

        expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(
                euclideanTransformation.asMatrix());
        expectedRotation = rotation1.combineAndReturnNew(rotation2);
        rotM1 = rotation1.asInhomogeneousMatrix();
        t2 = Matrix.newFromArray(translation2, true);
        rotM1.multiply(t2);
        rotM1.multiplyByScalar(scale1);
        expectedTranslation = rotM1.toArray();
        ArrayUtils.sum(expectedTranslation, translation1, expectedTranslation);
        // scale2 is assumed to be 1 because
        // transformation is euclidean
        expectedScale = scale1;

        // combine and return result as a new transformation
        transformation3 =
                transformation1.combineAndReturnNew(euclideanTransformation);
        // combine into transformation1
        transformation1.combine(euclideanTransformation);

        // both matrices m1 and m3 need to be equal
        m3 = transformation3.asMatrix();
        m1 = transformation1.asMatrix();

        // check correctness
        assertTrue(m1.equals(m3, ABSOLUTE_ERROR));

        // besides, resulting transformation matrices need to be equal to
        // expected matrix
        assertTrue(m1.equals(expectedMatrix, ABSOLUTE_ERROR));
        assertTrue(m3.equals(expectedMatrix, ABSOLUTE_ERROR));

        // check also correctness of rotation and translation
        assertEquals(transformation1.getRotation().getTheta(),
                expectedRotation.getTheta(), ABSOLUTE_ERROR);
        assertEquals(transformation3.getRotation().getTheta(),
                expectedRotation.getTheta(), ABSOLUTE_ERROR);

        assertArrayEquals(transformation1.getTranslation(), expectedTranslation,
                ABSOLUTE_ERROR);
        assertArrayEquals(transformation3.getTranslation(), expectedTranslation,
                ABSOLUTE_ERROR);

        assertEquals(transformation1.getScale(), expectedScale, ABSOLUTE_ERROR);
        assertEquals(transformation3.getScale(), expectedScale, ABSOLUTE_ERROR);
    }

    @Test
    public void testSetTransformationFromPoints()
            throws CoincidentPointsException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES2,
                MAX_ANGLE_DEGREES2) * Math.PI / 180.0;
        final Rotation2D rotation = new Rotation2D(theta);

        final double[] translation =
                new double[EuclideanTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_TRANSLATION2, MAX_TRANSLATION2);
        final double scale = randomizer.nextDouble(MIN_SCALE2, MAX_SCALE2);

        final MetricTransformation2D transformation =
                new MetricTransformation2D(rotation, translation, scale);

        final InhomogeneousPoint2D inputPoint1 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final InhomogeneousPoint2D inputPoint2 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final InhomogeneousPoint2D inputPoint3 = new InhomogeneousPoint2D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        final Point2D outputPoint1 = transformation.transformAndReturnNew(
                inputPoint1);
        final Point2D outputPoint2 = transformation.transformAndReturnNew(
                inputPoint2);
        final Point2D outputPoint3 = transformation.transformAndReturnNew(
                inputPoint3);

        final MetricTransformation2D transformation2 =
                new MetricTransformation2D();
        transformation2.setTransformationFromPoints(inputPoint1,
                inputPoint2, inputPoint3, outputPoint1, outputPoint2,
                outputPoint3);

        // check correctness
        final Rotation2D rotation2 = transformation2.getRotation();
        final double[] translation2 = transformation2.getTranslation();

        assertEquals(rotation2.getTheta(), rotation.getTheta(),
                ABSOLUTE_ERROR);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(transformation2.getScale(), scale, ABSOLUTE_ERROR);
    }

    private static void transformPoint(
            final Point2D inputPoint, final Point2D outputPoint,
            final Rotation2D rotation, final double[] translation, final double scale) {
        inputPoint.normalize();
        rotation.rotate(inputPoint, outputPoint);
        outputPoint.setInhomogeneousCoordinates(
                scale * outputPoint.getInhomX() + translation[0],
                scale * outputPoint.getInhomY() + translation[1]);
    }

    private static void transformLine(
            final Line2D inputLine, final Line2D outputLine,
            final MetricTransformation2D transformation) throws WrongSizeException,
            RankDeficientMatrixException, DecomposerException {
        inputLine.normalize();
        final Matrix t = transformation.asMatrix();
        double norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final Matrix invT = Utils.inverse(t);
        norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final Matrix transInvT = invT.transposeAndReturnNew();
        final Matrix l = Matrix.newFromArray(inputLine.asArray(), true);

        outputLine.setParameters(transInvT.multiplyAndReturnNew(l).toArray());
    }

    private static void transformConic(
            final Conic inputConic, final Conic outputConic,
            final MetricTransformation2D transformation) throws AlgebraException,
            NonSymmetricMatrixException {

        final Matrix t = transformation.asMatrix();
        final Matrix invT = Utils.inverse(t);
        double norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final Matrix transInvT = invT.transposeAndReturnNew();

        inputConic.normalize();
        final Matrix c = inputConic.asMatrix();

        final Matrix transC = transInvT.multiplyAndReturnNew(c.multiplyAndReturnNew(invT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transC);
        transC.multiplyByScalar(1.0 / norm);

        outputConic.setParameters(transC);
    }

    private static void transformDualConic(
            final DualConic inputDualConic,
            final DualConic outputDualConic,
            final MetricTransformation2D transformation)
            throws WrongSizeException, NonSymmetricMatrixException {

        final Matrix t = transformation.asMatrix();
        double norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final Matrix transT = t.transposeAndReturnNew();

        inputDualConic.normalize();
        final Matrix dualC = inputDualConic.asMatrix();

        final Matrix transDualC = t.multiplyAndReturnNew(
                dualC.multiplyAndReturnNew(transT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transDualC);
        transDualC.multiplyByScalar(1.0 / norm);

        outputDualConic.setParameters(transDualC);
    }
}
