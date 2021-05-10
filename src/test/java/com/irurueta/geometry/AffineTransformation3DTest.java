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


public class AffineTransformation3DTest {

    private static final int PINHOLE_CAMERA_ROWS = 3;
    private static final int PINHOLE_CAMERA_COLS = 4;

    private static final int HOM_COORDS = 4;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;

    private static final double MIN_SCALE = 0.2;
    private static final double MAX_SCALE = 5.0;

    private static final int MIN_POINTS = 3;
    private static final int MAX_POINTS = 50;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double MIN_RANDOM_POINT_VALUE = 50.0;
    private static final double MAX_RANDOM_POINT_VALUE = 100.0;

    private static final double MIN_FOCAL_LENGTH = 110.0;
    private static final double MAX_FOCAL_LENGTH = 130.0;

    private static final double MIN_SKEWNESS = -0.001;
    private static final double MAX_SKEWNESS = 0.001;

    private static final double MIN_PRINCIPAL_POINT = 0.0;
    private static final double MAX_PRINCIPAL_POINT = 100.0;

    private static final double MIN_ANGLE_DEGREES2 = 10.0;
    private static final double MAX_ANGLE_DEGREES2 = 15.0;

    private static final int TIMES = 100;

    @Test
    public void testConstructor() throws AlgebraException, RotationException {

        // Test empty constructor
        AffineTransformation3D transformation = new AffineTransformation3D();

        // check correctness
        assertEquals(transformation.getRotation().getRotationAngle(), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2], 1.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters3D.DEFAULT_SCALE, 0.0);
        assertEquals(transformation.getParameters().getScaleZ(),
                AffineParameters3D.DEFAULT_SCALE, 0.0);
        assertEquals(transformation.getParameters().getScaleZ(),
                AffineParameters3D.DEFAULT_SCALE, 0.0);
        assertEquals(transformation.getParameters().getSkewnessXY(),
                AffineParameters3D.DEFAULT_SKEWNESS, 0.0);
        assertEquals(transformation.getParameters().getSkewnessXZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, 0.0);
        assertEquals(transformation.getParameters().getSkewnessYZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, 0.0);
        assertTrue(transformation.getA().equals(
                transformation.getParameters().asMatrix().multiplyAndReturnNew(
                        transformation.getRotation().asInhomogeneousMatrix()),
                ABSOLUTE_ERROR));
        assertNotNull(transformation.getA());

        // Test constructor with A matrix
        Matrix a = Matrix.createWithUniformRandomValues(
                AffineTransformation3D.INHOM_COORDS,
                AffineTransformation3D.INHOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        transformation = new AffineTransformation3D(a);

        // check correctness
        assertNotNull(transformation.getRotation());
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertNotNull(transformation.getParameters());

        // Force NullPointerException
        transformation = null;
        try {
            transformation = new AffineTransformation3D((Matrix) null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // Force IllegalArgumentException
        final Matrix badA = new Matrix(AffineTransformation3D.INHOM_COORDS + 1,
                AffineTransformation3D.INHOM_COORDS + 1);
        try {
            transformation = new AffineTransformation3D(badA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(transformation);

        // Test constructor with scale
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        transformation = new AffineTransformation3D(scale);

        // check correctness
        assertEquals(transformation.getRotation().getRotationAngle(), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2], 1.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[2], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);
        assertEquals(transformation.getParameters().getScaleX(),
                scale, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(),
                scale, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleZ(),
                scale, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXY(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessYZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());

        // Test constructor with rotation
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        transformation = new AffineTransformation3D(rotation);

        // check correctness
        assertEquals(Math.abs(transformation.getRotation().getRotationAngle()),
                Math.abs(theta), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[0]),
                Math.abs(rotAxis[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[1]),
                Math.abs(rotAxis[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[2]),
                Math.abs(rotAxis[2]), ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[2], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleZ(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXY(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessYZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());

        // Force NullPointerException
        transformation = null;
        try {
            //noinspection all
            transformation = new AffineTransformation3D((Rotation3D) null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        assertNull(transformation);

        // Test constructor with scale and rotation
        transformation = new AffineTransformation3D(scale, rotation);

        // check correctness
        assertEquals(Math.abs(transformation.getRotation().getRotationAngle()),
                Math.abs(theta), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[0]),
                Math.abs(rotAxis[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[1]),
                Math.abs(rotAxis[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[2]),
                Math.abs(rotAxis[2]), ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[2], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);
        assertEquals(transformation.getParameters().getScaleX(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleZ(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXY(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessYZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());

        // Force NullPointerException
        transformation = null;
        try {
            //noinspection all
            transformation = new AffineTransformation3D(scale,
                    (Rotation3D) null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        assertNull(transformation);

        // Test constructor with affine parameters and rotation
        final double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double scaleZ = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters3D params = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);
        transformation = new AffineTransformation3D(params, rotation);

        // check correctness
        assertEquals(Math.abs(transformation.getRotation().getRotationAngle()),
                Math.abs(theta), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[0]),
                Math.abs(rotAxis[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[1]),
                Math.abs(rotAxis[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[2]),
                Math.abs(rotAxis[2]), ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[1], 0.0, 0.0);
        assertEquals(transformation.getTranslation()[2], 0.0, 0.0);
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);
        assertEquals(transformation.getParameters().getScaleX(), scaleX,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scaleY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleZ(), scaleZ,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXY(), skewnessXY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXZ(), skewnessXZ,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessYZ(), skewnessYZ,
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());

        // Force NullPointerException
        transformation = null;
        try {
            //noinspection all
            transformation = new AffineTransformation3D(
                    null, rotation);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            //noinspection all
            transformation = new AffineTransformation3D(params,
                    null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        assertNull(transformation);

        // Test constructor with translation
        final double[] translation = new double[AffineParameters3D.INHOM_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        transformation = new AffineTransformation3D(translation);

        // check correctness
        assertEquals(transformation.getRotation().getRotationAngle(), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 0.0,
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[2]), 1.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslation()[2], translation[2], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getTranslationZ(), translation[2], 0.0);
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleZ(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXY(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessYZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());

        // Force NullPointerException
        transformation = null;
        try {
            //noinspection all
            transformation = new AffineTransformation3D((double[]) null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        final double[] badTranslation = new double[
                AffineTransformation3D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation = new AffineTransformation3D(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(transformation);

        // Test constructor with matrix A and translation
        transformation = new AffineTransformation3D(a, translation);

        // check correctness
        assertNotNull(transformation.getRotation());
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslation()[2], translation[2], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getTranslationZ(), translation[2], 0.0);
        assertNotNull(transformation.getParameters());
        assertTrue(a.equals(transformation.getA(), ABSOLUTE_ERROR));

        // Force NullPointerException
        transformation = null;
        try {
            transformation = new AffineTransformation3D((Matrix) null,
                    translation);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            //noinspection all
            transformation = new AffineTransformation3D(a, null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // Force IllegalArgumentException
        try {
            transformation = new AffineTransformation3D(badA, translation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            transformation = new AffineTransformation3D(a, badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(transformation);

        // Test constructor with scale and translation
        transformation = new AffineTransformation3D(scale, translation);

        // Check correctness
        assertEquals(transformation.getRotation().getRotationAngle(), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 0.0,
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[2]), 1.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslation()[2], translation[2], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getTranslationZ(), translation[2], 0.0);
        assertEquals(transformation.getParameters().getScaleX(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleZ(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXY(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessYZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);

        // Force NullPointerException
        transformation = null;
        try {
            //noinspection all
            transformation = new AffineTransformation3D(scale, (double[]) null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // Force IllegalArgumentException
        try {
            transformation = new AffineTransformation3D(scale, badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(transformation);

        // Test constructor with rotation and translation
        transformation = new AffineTransformation3D(rotation, translation);

        // check correctness
        assertEquals(Math.abs(transformation.getRotation().getRotationAngle()),
                Math.abs(theta), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[0]),
                Math.abs(rotAxis[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[1]),
                Math.abs(rotAxis[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[2]),
                Math.abs(rotAxis[2]), ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslation()[2], translation[2], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getTranslationZ(), translation[2], 0.0);
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleZ(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXY(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessYZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);

        // Force NullPointerException
        transformation = null;
        try {
            transformation = new AffineTransformation3D((Rotation3D) null,
                    translation);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            //noinspection all
            transformation = new AffineTransformation3D(rotation, null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // Force IllegalArgumentException
        try {
            transformation = new AffineTransformation3D(rotation,
                    badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(transformation);


        // Test constructor with scale, rotation and translation
        transformation = new AffineTransformation3D(scale, rotation,
                translation);

        // check correctness
        assertEquals(Math.abs(transformation.getRotation().getRotationAngle()),
                Math.abs(theta), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[0]),
                Math.abs(rotAxis[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[1]),
                Math.abs(rotAxis[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[2]),
                Math.abs(rotAxis[2]), ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslation()[2], translation[2], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getTranslationZ(), translation[2], 0.0);
        assertEquals(transformation.getParameters().getScaleX(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleZ(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXY(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessYZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);

        // Force NullPointerException
        transformation = null;
        try {
            transformation = new AffineTransformation3D(scale, null,
                    translation);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            //noinspection all
            transformation = new AffineTransformation3D(scale, rotation,
                    null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // Force IllegalArgumentException
        try {
            transformation = new AffineTransformation3D(scale, rotation,
                    badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(transformation);

        // Test constructor with affine parameters, rotation and translation
        transformation = new AffineTransformation3D(params, rotation,
                translation);

        // check correctness
        assertEquals(Math.abs(transformation.getRotation().getRotationAngle()),
                Math.abs(theta), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[0]),
                Math.abs(rotAxis[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[1]),
                Math.abs(rotAxis[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[2]),
                Math.abs(rotAxis[2]), ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0], 0.0);
        assertEquals(transformation.getTranslation()[1], translation[1], 0.0);
        assertEquals(transformation.getTranslation()[2], translation[2], 0.0);
        assertEquals(transformation.getTranslationX(), translation[0], 0.0);
        assertEquals(transformation.getTranslationY(), translation[1], 0.0);
        assertEquals(transformation.getTranslationZ(), translation[2], 0.0);
        assertEquals(transformation.getParameters().getScaleX(), scaleX,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scaleY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleZ(), scaleZ,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXY(), skewnessXY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXZ(), skewnessXZ,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessYZ(), skewnessYZ,
                ABSOLUTE_ERROR);

        // Force NullPointerException
        transformation = null;
        try {
            transformation = new AffineTransformation3D(null, rotation,
                    translation);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            transformation = new AffineTransformation3D(params, null,
                    translation);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
        try {
            //noinspection all
            transformation = new AffineTransformation3D(params, rotation,
                    null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // Force IllegalArgumentException
        try {
            transformation = new AffineTransformation3D(params, rotation,
                    badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(transformation);
    }

    @Test
    public void testGetSetA() throws WrongSizeException {
        final Matrix a = Matrix.createWithUniformRandomValues(
                AffineTransformation3D.INHOM_COORDS,
                AffineTransformation3D.INHOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineTransformation3D transformation = new AffineTransformation3D();

        // check default value
        assertTrue(transformation.getA().equals(Matrix.identity(
                AffineParameters3D.INHOM_COORDS,
                AffineParameters3D.INHOM_COORDS), ABSOLUTE_ERROR));

        // set matrix A
        transformation.setA(a);

        // check correctness
        assertTrue(transformation.getA().equals(a, ABSOLUTE_ERROR));

        // Force NullPointerException
        try {
            transformation.setA(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }

        // Force IllegalArgumentException
        final Matrix badA = new Matrix(AffineTransformation3D.INHOM_COORDS + 1,
                AffineTransformation3D.INHOM_COORDS + 1);

        try {
            transformation.setA(badA);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetRotation() throws RotationException,
            AlgebraException {
        final AffineTransformation3D transformation = new AffineTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;

        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize rotation axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        // test default values
        assertEquals(transformation.getRotation().getRotationAngle(), 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[0], 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[1], 0.0,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getRotation().getRotationAxis()[2], 1.0,
                ABSOLUTE_ERROR);

        // set new value
        transformation.setRotation(rotation);

        // check correctness
        assertEquals(Math.abs(transformation.getRotation().getRotationAngle()),
                Math.abs(theta), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[0]),
                Math.abs(rotAxis[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[1]),
                Math.abs(rotAxis[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(
                transformation.getRotation().getRotationAxis()[2]),
                Math.abs(rotAxis[2]), ABSOLUTE_ERROR);

        // Force NullPointerException
        try {
            //noinspection ConstantConditions
            transformation.setRotation(null);
            fail("NullPointerException expected but not thrown");
        } catch (final NullPointerException ignore) {
        }
    }

    @Test
    public void testAddRotation() throws RotationException, AlgebraException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final AffineTransformation3D transformation = new AffineTransformation3D();

            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            final double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            final double[] rotAxis1 = new double[Rotation3D.INHOM_COORDS];
            randomizer.fill(rotAxis1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            double norm = Utils.normF(rotAxis1);
            ArrayUtils.multiplyByScalar(rotAxis1, 1.0 / norm, rotAxis1);

            final double[] rotAxis2 = new double[Rotation3D.INHOM_COORDS];
            randomizer.fill(rotAxis2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            norm = Utils.normF(rotAxis2);
            ArrayUtils.multiplyByScalar(rotAxis2, 1.0 / norm, rotAxis2);

            final Rotation3D rotation1 = Rotation3D.create(rotAxis1, theta1);
            final Rotation3D rotation2 = Rotation3D.create(rotAxis2, theta2);

            final Rotation3D combinedRotation = rotation1.combineAndReturnNew(rotation2);

            // set rotation1
            transformation.setRotation(rotation1);

            // check correctness
            if (Math.abs(Math.abs(transformation.getRotation().getRotationAngle()) -
                    Math.abs(theta1)) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(transformation.getRotation().getRotationAngle()),
                    Math.abs(theta1), ABSOLUTE_ERROR);
            assertEquals(Math.abs(
                    transformation.getRotation().getRotationAxis()[0]),
                    Math.abs(rotAxis1[0]), ABSOLUTE_ERROR);
            assertEquals(Math.abs(
                    transformation.getRotation().getRotationAxis()[1]),
                    Math.abs(rotAxis1[1]), ABSOLUTE_ERROR);
            assertEquals(Math.abs(
                    transformation.getRotation().getRotationAxis()[2]),
                    Math.abs(rotAxis1[2]), ABSOLUTE_ERROR);

            // add second rotation
            transformation.addRotation(rotation2);

            // check correctness
            if (Math.abs(Math.abs(transformation.getRotation().getRotationAngle()) -
                    Math.abs(combinedRotation.getRotationAngle())) > 5.0 * ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(transformation.getRotation().getRotationAngle()),
                    Math.abs(combinedRotation.getRotationAngle()), 5.0 * ABSOLUTE_ERROR);
            assertEquals(
                    Math.abs(transformation.getRotation().getRotationAxis()[0]),
                    Math.abs(combinedRotation.getRotationAxis()[0]),
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(
                    transformation.getRotation().getRotationAxis()[1]),
                    Math.abs(combinedRotation.getRotationAxis()[1]),
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(
                    transformation.getRotation().getRotationAxis()[2]),
                    Math.abs(combinedRotation.getRotationAxis()[2]),
                    ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testGetSetParameters() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double scaleZ = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final AffineParameters3D params = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        // instantiate transformation
        final AffineTransformation3D transformation = new AffineTransformation3D();

        // check default values
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleZ(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXY(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessYZ(),
                AffineParameters3D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);

        final AffineParameters3D defaultParams = new AffineParameters3D();
        transformation.getParameters(defaultParams);

        assertEquals(defaultParams.getScaleX(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(defaultParams.getScaleY(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(defaultParams.getScaleZ(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(defaultParams.getSkewnessXY(),
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(defaultParams.getSkewnessXZ(),
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);
        assertEquals(defaultParams.getSkewnessYZ(),
                AffineParameters2D.DEFAULT_SKEWNESS, ABSOLUTE_ERROR);

        // set parameters
        transformation.setParameters(params);

        // check correctness
        assertEquals(transformation.getParameters().getScaleX(), scaleX,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleY(), scaleY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleZ(), scaleZ,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXY(), skewnessXY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessXZ(), skewnessXZ,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getSkewnessYZ(), skewnessYZ,
                ABSOLUTE_ERROR);

        final AffineParameters3D params2 = new AffineParameters3D();
        transformation.getParameters(params2);

        assertEquals(params2.getScaleX(), scaleX, ABSOLUTE_ERROR);
        assertEquals(params2.getScaleY(), scaleY, ABSOLUTE_ERROR);
        assertEquals(params2.getScaleZ(), scaleZ, ABSOLUTE_ERROR);
        assertEquals(params2.getSkewnessXY(), skewnessXY, ABSOLUTE_ERROR);
        assertEquals(params2.getSkewnessXZ(), skewnessXZ, ABSOLUTE_ERROR);
        assertEquals(params2.getSkewnessYZ(), skewnessYZ, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetTranslation() {
        final AffineTransformation3D transformation = new AffineTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] translation = new double[
                AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), 0.0, ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation[0],
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translation[1],
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], translation[2],
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translation[0],
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translation[1],
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), translation[2],
                ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final double[] badTranslation = new double[
                AffineTransformation3D.NUM_TRANSLATION_COORDS + 1];

        try {
            transformation.setTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testAddTranslation() {
        final AffineTransformation3D transformation = new AffineTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] translation1 = new double[
                AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double[] translation2 = new double[
                AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);

        // set new value
        final double[] translationCopy = Arrays.copyOf(translation1,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation1[0],
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translation1[1],
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], translation1[2],
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translation1[0],
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translation1[1],
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), translation1[2],
                ABSOLUTE_ERROR);

        // add translation
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translation1[0] +
                translation2[0], ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translation1[1] +
                translation2[1], ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], translation1[2] +
                translation2[2], ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translation1[0] +
                translation2[0], ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translation1[1] +
                translation2[1], ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), translation1[2] +
                translation2[2], ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final double[] badTranslation = new double[
                AffineTransformation3D.NUM_TRANSLATION_COORDS + 1];
        try {
            transformation.addTranslation(badTranslation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTranslationX() {
        final AffineTransformation3D transformation = new AffineTransformation3D();

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
        final AffineTransformation3D transformation = new AffineTransformation3D();

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
    public void testGetSetTranslationZ() {
        final AffineTransformation3D transformation = new AffineTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);

        // set new value
        transformation.setTranslationZ(translationZ);

        // check correctness
        assertEquals(transformation.getTranslationZ(), translationZ, 0.0);
    }

    @Test
    public void testSetTranslationCoordinates() {
        final AffineTransformation3D transformation = new AffineTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), 0.0, ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translationX, translationY, translationZ);

        // check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translationX,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translationY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], translationZ,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translationX,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translationY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), translationZ,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testGetSetTranslationPoint() {
        final AffineTransformation3D transformation = new AffineTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final InhomogeneousPoint3D translation = new InhomogeneousPoint3D(
                translationX, translationY, translationZ);

        // check default value
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), 0.0, ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), 0.0, ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(transformation.getTranslation().length,
                AffineTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(transformation.getTranslation()[0], translationX,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[1], translationY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslation()[2], translationZ,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationX(), translationX,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationY(), translationY,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getTranslationZ(), translationZ,
                ABSOLUTE_ERROR);

        final Point3D translation2 = transformation.getTranslationPoint();
        final Point3D translation3 = Point3D.create();
        transformation.getTranslationPoint(translation3);

        // check correctness
        assertEquals(translation, translation2);
        assertEquals(translation, translation3);
    }

    @Test
    public void testAddTranslationX() {
        final AffineTransformation3D transformation = new AffineTransformation3D();

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
        final AffineTransformation3D transformation = new AffineTransformation3D();

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
    public void testAddTranslationZ() {
        final AffineTransformation3D transformation = new AffineTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);

        // set value
        transformation.setTranslationZ(translationZ1);

        // Check correctness
        assertEquals(transformation.getTranslationZ(), translationZ1, 0.0);

        // add translationZ
        transformation.addTranslationZ(translationZ2);

        // check correctness
        assertEquals(transformation.getTranslationZ(),
                translationZ1 + translationZ2, 0.0);
    }

    @Test
    public void testAddTranslationCoordinates() {
        final AffineTransformation3D transformation = new AffineTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);

        // set values
        transformation.setTranslation(translationX1, translationY1,
                translationZ1);

        // add translation
        transformation.addTranslation(translationX2, translationY2,
                translationZ2);

        // check correctness
        assertEquals(transformation.getTranslationX(),
                translationX1 + translationX2, 0.0);
        assertEquals(transformation.getTranslationY(),
                translationY1 + translationY2, 0.0);
        assertEquals(transformation.getTranslationZ(),
                translationZ1 + translationZ2, 0.0);
    }

    @Test
    public void testAddTranslationPoint() {
        final AffineTransformation3D transformation = new AffineTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        // check default value
        assertEquals(transformation.getTranslationX(), 0.0, 0.0);
        assertEquals(transformation.getTranslationY(), 0.0, 0.0);
        assertEquals(transformation.getTranslationZ(), 0.0, 0.0);

        // set values
        transformation.setTranslation(translationX1, translationY1,
                translationZ1);

        // add translation
        final Point3D translation2 = new InhomogeneousPoint3D(translationX2,
                translationY2, translationZ2);
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(transformation.getTranslationX(),
                translationX1 + translationX2, 0.0);
        assertEquals(transformation.getTranslationY(),
                translationY1 + translationY2, 0.0);
        assertEquals(transformation.getTranslationZ(),
                translationZ1 + translationZ2, 0.0);
    }

    @Test
    public void testGetSetScale() throws AlgebraException {
        final AffineTransformation3D transformation = new AffineTransformation3D();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double scale = randomizer.nextDouble(MIN_SCALE,
                MAX_SCALE);

        // check default value
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleX(),
                AffineParameters3D.DEFAULT_SCALE, ABSOLUTE_ERROR);

        // set value
        transformation.setScale(scale);

        // check correctness
        assertEquals(transformation.getParameters().getScaleX(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleX(), scale,
                ABSOLUTE_ERROR);
        assertEquals(transformation.getParameters().getScaleX(), scale,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testAsMatrix() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        final double[] translation = new double[
                AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double scaleZ = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);
        final AffineParameters3D params = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        AffineTransformation3D transformation =
                new AffineTransformation3D(params, rotation, translation);

        final Matrix m = Matrix.identity(AffineTransformation3D.HOM_COORDS,
                AffineTransformation3D.HOM_COORDS);
        m.setSubmatrix(0, 0, 2, 2,
                params.asMatrix().multiplyAndReturnNew(
                        rotation.asInhomogeneousMatrix()));
        m.setSubmatrix(0, 3, 2, 3, translation);

        Matrix transMatrix1 = transformation.asMatrix();
        Matrix transMatrix2 = new Matrix(AffineTransformation3D.HOM_COORDS,
                AffineTransformation3D.HOM_COORDS);
        transformation.asMatrix(transMatrix2);

        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));

        // test again by providing A matrix
        Matrix a = Matrix.createWithUniformRandomValues(
                AffineTransformation3D.INHOM_COORDS,
                AffineTransformation3D.INHOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        transformation = new AffineTransformation3D(a, translation);

        m.setSubmatrix(0, 0, 2, 2, a);

        transMatrix1 = transformation.asMatrix();
        transMatrix2 = new Matrix(AffineTransformation3D.HOM_COORDS,
                AffineTransformation3D.HOM_COORDS);
        transformation.asMatrix(transMatrix2);

        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        Matrix t = new Matrix(AffineTransformation3D.HOM_COORDS + 1,
                AffineTransformation3D.HOM_COORDS + 1);
        try {
            transformation.asMatrix(t);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testTransformPoint() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] coords = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Point3D point = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineTransformation3D transformation =
                new AffineTransformation3D(params, rotation, translation);

        final Point3D expectedPoint = Point3D.create();
        transformPoint(point, expectedPoint, transformation);


        final Point3D outPoint1 = transformation.transformAndReturnNew(point);
        final Point3D outPoint2 = Point3D.create();
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
    public void testTransformPoints() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineTransformation3D transformation =
                new AffineTransformation3D(params, rotation, translation);

        final ArrayList<Point3D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point3D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, transformation);

            expectedPoints.add(expectedPoint);
        }

        final List<Point3D> outPoints1 = transformation.transformPointsAndReturnNew(
                inputPoints);
        final List<Point3D> outPoints2 = new ArrayList<>();
        transformation.transformPoints(inputPoints, outPoints2);

        // check correctness
        assertEquals(outPoints1.size(), inputPoints.size());
        assertEquals(outPoints2.size(), inputPoints.size());
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedPoints.get(i);

            final Point3D outPoint1 = outPoints1.get(i);
            final Point3D outPoint2 = outPoints2.get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testTransformAndOverwritePoints() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineTransformation3D transformation =
                new AffineTransformation3D(params, rotation, translation);

        final ArrayList<Point3D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point3D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, transformation);

            expectedPoints.add(expectedPoint);
        }

        transformation.transformAndOverwritePoints(inputPoints);

        // check correctness
        assertEquals(inputPoints.size(), size);
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedPoints.get(i);

            final Point3D point = inputPoints.get(i);

            assertTrue(point.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testTransformQuadric() throws NonSymmetricMatrixException,
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
        final double g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Quadric quadric = new Quadric(a, b, c, d, e, f, g, h, i, j);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineTransformation3D transformation =
                new AffineTransformation3D(params, rotation, translation);

        // compute expected value
        final Quadric expectedQuadric = new Quadric();
        transformQuadric(quadric, expectedQuadric, transformation);
        expectedQuadric.normalize();

        // make transformation
        final Quadric outQuadric1 = transformation.transformAndReturnNew(quadric);
        final Quadric outQuadric2 = new Quadric();
        transformation.transform(quadric, outQuadric2);

        // check correctness
        outQuadric1.normalize();
        outQuadric2.normalize();

        assertEquals(expectedQuadric.getA(), outQuadric1.getA(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), outQuadric1.getB(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), outQuadric1.getC(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), outQuadric1.getD(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), outQuadric1.getE(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), outQuadric1.getF(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), outQuadric1.getG(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), outQuadric1.getH(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), outQuadric1.getI(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), outQuadric1.getJ(),
                ABSOLUTE_ERROR);

        assertEquals(expectedQuadric.getA(), outQuadric2.getA(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), outQuadric2.getB(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), outQuadric2.getC(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), outQuadric2.getD(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), outQuadric2.getE(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), outQuadric2.getF(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), outQuadric2.getG(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), outQuadric2.getH(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), outQuadric2.getI(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), outQuadric2.getJ(),
                ABSOLUTE_ERROR);

        transformation.transform(quadric);

        // check correctness
        quadric.normalize();

        assertEquals(expectedQuadric.getA(), quadric.getA(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), quadric.getB(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), quadric.getC(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), quadric.getD(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), quadric.getE(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), quadric.getF(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), quadric.getG(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), quadric.getH(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), quadric.getI(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), quadric.getJ(),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformQuadricAndPoints() throws AlgebraException,
            GeometryException {

        // create Quadric from 9 points
        Quadric quadric = null;
        Point3D point1;
        Point3D point2;
        Point3D point3;
        Point3D point4;
        Point3D point5;
        Point3D point6;
        Point3D point7;
        Point3D point8;
        Point3D point9;
        do {
            final Matrix m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                    m.getElementAt(0, 1), m.getElementAt(0, 2), 1.0);
            point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                    m.getElementAt(1, 1), m.getElementAt(1, 2), 1.0);
            point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                    m.getElementAt(2, 1), m.getElementAt(2, 2), 1.0);
            point4 = new HomogeneousPoint3D(m.getElementAt(3, 0),
                    m.getElementAt(3, 1), m.getElementAt(3, 2), 1.0);
            point5 = new HomogeneousPoint3D(m.getElementAt(4, 0),
                    m.getElementAt(4, 1), m.getElementAt(4, 2), 1.0);
            point6 = new HomogeneousPoint3D(m.getElementAt(5, 0),
                    m.getElementAt(5, 1), m.getElementAt(5, 2), 1.0);
            point7 = new HomogeneousPoint3D(m.getElementAt(6, 0),
                    m.getElementAt(6, 1), m.getElementAt(6, 2), 1.0);
            point8 = new HomogeneousPoint3D(m.getElementAt(7, 0),
                    m.getElementAt(7, 1), m.getElementAt(7, 2), 1.0);
            point9 = new HomogeneousPoint3D(m.getElementAt(8, 0),
                    m.getElementAt(8, 1), m.getElementAt(8, 2), 1.0);

            try {
                quadric = new Quadric(point1, point2, point3, point4, point5, point6,
                        point7, point8, point9);
            } catch (final GeometryException ignore) {
            }

        } while (quadric == null);

        // check that points belong to quadric
        assertTrue(quadric.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point3, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point4, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point5, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point6, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point7, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point8, ABSOLUTE_ERROR));
        assertTrue(quadric.isLocus(point9, ABSOLUTE_ERROR));

        // create transformation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        // compute expected value
        final Quadric expectedQuadric = new Quadric();
        transformQuadric(quadric, expectedQuadric, transformation);
        expectedQuadric.normalize();

        // transform quadric and points
        final Quadric outQuadric = transformation.transformAndReturnNew(quadric);
        final Point3D outPoint1 = transformation.transformAndReturnNew(point1);
        final Point3D outPoint2 = transformation.transformAndReturnNew(point2);
        final Point3D outPoint3 = transformation.transformAndReturnNew(point3);
        final Point3D outPoint4 = transformation.transformAndReturnNew(point4);
        final Point3D outPoint5 = transformation.transformAndReturnNew(point5);
        final Point3D outPoint6 = transformation.transformAndReturnNew(point6);
        final Point3D outPoint7 = transformation.transformAndReturnNew(point7);
        final Point3D outPoint8 = transformation.transformAndReturnNew(point8);
        final Point3D outPoint9 = transformation.transformAndReturnNew(point9);

        // check that transformed points still belong to transformed quadric
        assertTrue(outQuadric.isLocus(outPoint1, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint2, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint3, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint4, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint5, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint6, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint7, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint8, ABSOLUTE_ERROR));
        assertTrue(outQuadric.isLocus(outPoint9, ABSOLUTE_ERROR));

        // check quadric correctness
        outQuadric.normalize();

        assertEquals(expectedQuadric.getA(), outQuadric.getA(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), outQuadric.getB(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), outQuadric.getC(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), outQuadric.getD(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), outQuadric.getE(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), outQuadric.getF(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), outQuadric.getG(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), outQuadric.getH(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), outQuadric.getI(),
                ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), outQuadric.getJ(),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformDualQuadric() throws NonSymmetricMatrixException,
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
        final double g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final DualQuadric dualQuadric = new DualQuadric(a, b, c, d, e, f, g, h, i, j);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineTransformation3D transformation =
                new AffineTransformation3D(params, rotation, translation);

        // compute expected value
        final DualQuadric expectedDualQuadric = new DualQuadric();
        transformDualQuadric(dualQuadric, expectedDualQuadric, transformation);
        expectedDualQuadric.normalize();

        // make transformation
        final DualQuadric outDualQuadric1 = transformation.transformAndReturnNew(
                dualQuadric);
        final DualQuadric outDualQuadric2 = new DualQuadric();
        transformation.transform(dualQuadric, outDualQuadric2);

        // check correctness
        outDualQuadric1.normalize();
        outDualQuadric2.normalize();

        assertEquals(expectedDualQuadric.getA(), outDualQuadric1.getA(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), outDualQuadric1.getB(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), outDualQuadric1.getC(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), outDualQuadric1.getD(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), outDualQuadric1.getE(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), outDualQuadric1.getF(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), outDualQuadric1.getG(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), outDualQuadric1.getH(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), outDualQuadric1.getI(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), outDualQuadric1.getJ(),
                ABSOLUTE_ERROR);

        assertEquals(expectedDualQuadric.getA(), outDualQuadric1.getA(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), outDualQuadric1.getB(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), outDualQuadric1.getC(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), outDualQuadric1.getD(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), outDualQuadric1.getE(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), outDualQuadric1.getF(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), outDualQuadric1.getG(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), outDualQuadric1.getH(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), outDualQuadric1.getI(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), outDualQuadric1.getJ(),
                ABSOLUTE_ERROR);

        transformation.transform(dualQuadric);

        // check correctness
        dualQuadric.normalize();

        assertEquals(expectedDualQuadric.getA(), dualQuadric.getA(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), dualQuadric.getB(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), dualQuadric.getC(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), dualQuadric.getD(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), dualQuadric.getE(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), dualQuadric.getF(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), dualQuadric.getG(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), dualQuadric.getH(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), dualQuadric.getI(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), dualQuadric.getJ(),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformDualQuadricAndPlanes() throws AlgebraException,
            GeometryException {

        // create dual quadric from 9 planes
        DualQuadric dualQuadric = null;
        Plane plane1;
        Plane plane2;
        Plane plane3;
        Plane plane4;
        Plane plane5;
        Plane plane6;
        Plane plane7;
        Plane plane8;
        Plane plane9;
        do {
            final Matrix m = Matrix.createWithUniformRandomValues(9, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            plane1 = new Plane(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2), m.getElementAt(0, 3));
            plane2 = new Plane(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2), m.getElementAt(1, 3));
            plane3 = new Plane(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2), m.getElementAt(2, 3));
            plane4 = new Plane(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2), m.getElementAt(3, 3));
            plane5 = new Plane(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2), m.getElementAt(4, 3));
            plane6 = new Plane(m.getElementAt(5, 0), m.getElementAt(5, 1),
                    m.getElementAt(5, 2), m.getElementAt(5, 3));
            plane7 = new Plane(m.getElementAt(6, 0), m.getElementAt(6, 1),
                    m.getElementAt(6, 2), m.getElementAt(6, 3));
            plane8 = new Plane(m.getElementAt(7, 0), m.getElementAt(7, 1),
                    m.getElementAt(7, 2), m.getElementAt(7, 3));
            plane9 = new Plane(m.getElementAt(8, 0), m.getElementAt(8, 1),
                    m.getElementAt(8, 2), m.getElementAt(8, 3));

            plane1.normalize();
            plane2.normalize();
            plane3.normalize();
            plane4.normalize();
            plane5.normalize();
            plane6.normalize();
            plane7.normalize();
            plane8.normalize();
            plane9.normalize();

            try {
                dualQuadric = new DualQuadric(plane1, plane2, plane3, plane4, plane5,
                        plane6, plane7, plane8, plane9);
            } catch (final GeometryException ignore) {
            }

        } while (dualQuadric == null);

        // check that planes belong to dual quadric
        assertTrue(dualQuadric.isLocus(plane1, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane2, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane3, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane4, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane5, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane6, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane7, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane8, ABSOLUTE_ERROR));
        assertTrue(dualQuadric.isLocus(plane9, ABSOLUTE_ERROR));

        // create transformation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        // compute expected value
        final DualQuadric expectedDualQuadric = new DualQuadric();
        transformDualQuadric(dualQuadric, expectedDualQuadric, transformation);
        expectedDualQuadric.normalize();

        // transform dual quadric and planes
        final DualQuadric outDualQuadric = transformation.transformAndReturnNew(dualQuadric);
        final Plane outPlane1 = transformation.transformAndReturnNew(plane1);
        final Plane outPlane2 = transformation.transformAndReturnNew(plane2);
        final Plane outPlane3 = transformation.transformAndReturnNew(plane3);
        final Plane outPlane4 = transformation.transformAndReturnNew(plane4);
        final Plane outPlane5 = transformation.transformAndReturnNew(plane5);
        final Plane outPlane6 = transformation.transformAndReturnNew(plane6);
        final Plane outPlane7 = transformation.transformAndReturnNew(plane7);
        final Plane outPlane8 = transformation.transformAndReturnNew(plane8);
        final Plane outPlane9 = transformation.transformAndReturnNew(plane9);

        // check that transformed planes still belong to transformed dual quadric
        assertTrue(outDualQuadric.isLocus(outPlane1, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane2, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane3, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane4, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane5, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane6, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane7, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane8, ABSOLUTE_ERROR));
        assertTrue(outDualQuadric.isLocus(outPlane9, ABSOLUTE_ERROR));

        // check dual quadric correctness
        outDualQuadric.normalize();

        assertEquals(expectedDualQuadric.getA(), outDualQuadric.getA(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), outDualQuadric.getB(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), outDualQuadric.getC(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), outDualQuadric.getD(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), outDualQuadric.getE(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), outDualQuadric.getF(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), outDualQuadric.getG(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), outDualQuadric.getH(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), outDualQuadric.getI(),
                ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), outDualQuadric.getJ(),
                ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformPlane() throws AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] params = new double[Plane.PLANE_NUMBER_PARAMS];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Plane plane = new Plane(params);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        final Plane expectedPlane = new Plane();
        transformPlane(plane, expectedPlane, transformation);
        expectedPlane.normalize();

        final Plane outPlane1 = transformation.transformAndReturnNew(plane);
        final Plane outPlane2 = new Plane();
        transformation.transform(plane, outPlane2);

        outPlane1.normalize();
        outPlane2.normalize();

        // check correctness
        assertEquals(expectedPlane.getA(), outPlane1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getB(), outPlane1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getC(), outPlane1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getD(), outPlane1.getD(), ABSOLUTE_ERROR);

        assertEquals(expectedPlane.getA(), outPlane2.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getB(), outPlane2.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getC(), outPlane2.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getD(), outPlane2.getD(), ABSOLUTE_ERROR);

        transformation.transform(plane);

        plane.normalize();

        // check correctness
        assertEquals(expectedPlane.getA(), plane.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getB(), plane.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getC(), plane.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getD(), plane.getD(), ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformPlaneAndPoints() throws AlgebraException,
            GeometryException {

        // create plane from 3 points
        Matrix m = Matrix.createWithUniformRandomValues(3, HOM_COORDS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        // ensure we create a matrix with 3 non linear dependent rows
        while (decomposer.getRank() < 3) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final Point3D point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2),
                m.getElementAt(0, 3));
        final Point3D point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2),
                m.getElementAt(1, 3));
        final Point3D point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1),
                m.getElementAt(2, 2),
                m.getElementAt(2, 3));

        point1.normalize();
        point2.normalize();
        point3.normalize();

        final Plane plane = new Plane(point1, point2, point3);

        // check that points belong to the plane
        assertTrue(plane.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point3, ABSOLUTE_ERROR));

        // create transformation
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        final Plane expectedPlane = new Plane();
        transformPlane(plane, expectedPlane, transformation);
        expectedPlane.normalize();

        // transform plane and points
        final Plane outPlane = transformation.transformAndReturnNew(plane);
        final Point3D outPoint1 = transformation.transformAndReturnNew(point1);
        final Point3D outPoint2 = transformation.transformAndReturnNew(point1);
        final Point3D outPoint3 = transformation.transformAndReturnNew(point1);

        // check that transformed points still belong to transformed plane
        assertTrue(outPlane.isLocus(outPoint1));
        assertTrue(outPlane.isLocus(outPoint2));
        assertTrue(outPlane.isLocus(outPoint3));

        // check plane correctness
        outPlane.normalize();

        assertEquals(expectedPlane.getA(), outPlane.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getB(), outPlane.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getC(), outPlane.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedPlane.getD(), outPlane.getD(), ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformPlanes() throws AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        final ArrayList<Plane> inputPlanes = new ArrayList<>(size);
        final ArrayList<Plane> expectedPlanes = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] params = new double[Plane.PLANE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Plane plane = new Plane(params);
            inputPlanes.add(plane);

            final Plane expectedPlane = new Plane();
            transformPlane(plane, expectedPlane, transformation);

            expectedPlanes.add(expectedPlane);
        }

        final List<Plane> outPlanes1 = transformation.transformPlanesAndReturnNew(inputPlanes);
        final List<Plane> outPlanes2 = new ArrayList<>();
        transformation.transformPlanes(inputPlanes, outPlanes2);

        // check correctness
        assertEquals(outPlanes1.size(), inputPlanes.size());
        assertEquals(outPlanes2.size(), inputPlanes.size());
        for (int i = 0; i < size; i++) {
            final Plane expectedPlane = expectedPlanes.get(i);

            final Plane outPlane1 = outPlanes1.get(i);
            final Plane outPlane2 = outPlanes2.get(i);

            expectedPlane.normalize();
            outPlane1.normalize();
            outPlane2.normalize();

            // check correctness
            assertEquals(expectedPlane.getA(), outPlane1.getA(),
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getB(), outPlane1.getB(),
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getC(), outPlane1.getC(),
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getD(), outPlane1.getD(),
                    ABSOLUTE_ERROR);

            assertEquals(expectedPlane.getA(), outPlane2.getA(),
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getB(), outPlane2.getB(),
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getC(), outPlane2.getC(),
                    ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getD(), outPlane2.getD(),
                    ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testTransformAndOverwritePlanes() throws AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        final ArrayList<Plane> inputPlanes = new ArrayList<>(size);
        final ArrayList<Plane> expectedPlanes = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] params = new double[Plane.PLANE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Plane plane = new Plane(params);
            inputPlanes.add(plane);

            final Plane expectedPlane = new Plane();
            transformPlane(plane, expectedPlane, transformation);

            expectedPlanes.add(expectedPlane);
        }

        transformation.transformAndOverwritePlanes(inputPlanes);

        // check correctness
        assertEquals(inputPlanes.size(), size);
        for (int i = 0; i < size; i++) {
            final Plane expectedPlane = expectedPlanes.get(i);

            final Plane plane = inputPlanes.get(i);

            expectedPlane.normalize();
            plane.normalize();

            // check correctness
            assertEquals(expectedPlane.getA(), plane.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getB(), plane.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getC(), plane.getC(), ABSOLUTE_ERROR);
            assertEquals(expectedPlane.getD(), plane.getD(), ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testTransformLine() throws CoincidentPointsException,
            CoincidentPlanesException, AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] coords = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Point3D point1 = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final Point3D point2 = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        final Line3D line = new Line3D(point1, point2);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        final Line3D expectedLine = new Line3D(point1, point2);
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();

        final Line3D outLine1 = transformation.transformAndReturnNew(line);
        final Line3D outLine2 = new Line3D(point1, point2);
        transformation.transform(line, outLine2);

        outLine1.normalize();
        outLine2.normalize();

        // check correctness
        assertEquals(expectedLine.getPlane1().getA(),
                outLine1.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(),
                outLine1.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(),
                outLine1.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(),
                outLine1.getPlane1().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane2().getA(),
                outLine1.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(),
                outLine1.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(),
                outLine1.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(),
                outLine1.getPlane2().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane1().getA(),
                outLine2.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(),
                outLine2.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(),
                outLine2.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(),
                outLine2.getPlane1().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane2().getA(),
                outLine2.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(),
                outLine2.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(),
                outLine2.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(),
                outLine2.getPlane2().getD(), ABSOLUTE_ERROR);

        transformation.transform(line);

        line.normalize();

        // check correctness
        assertEquals(expectedLine.getPlane1().getA(),
                line.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(),
                line.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(),
                line.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(),
                line.getPlane1().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane2().getA(),
                line.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(),
                line.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(),
                line.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(),
                line.getPlane2().getD(), ABSOLUTE_ERROR);
    }

    @Test
    public void testTransformLines() throws CoincidentPlanesException,
            CoincidentPointsException, AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        final ArrayList<Line3D> inputLines = new ArrayList<>(size);
        final ArrayList<Line3D> expectedLines = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final Point3D point1 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final Point3D point2 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            final Line3D line = new Line3D(point1, point2);
            inputLines.add(line);

            final Line3D expectedLine = new Line3D(point1, point2);
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }

        final List<Line3D> outLines1 = transformation.transformLines(inputLines);
        final List<Line3D> outLines2 = new ArrayList<>();
        transformation.transformLines(inputLines, outLines2);

        // check correctness
        assertEquals(outLines1.size(), inputLines.size());
        assertEquals(outLines2.size(), inputLines.size());
        for (int i = 0; i < size; i++) {
            final Line3D expectedLine = expectedLines.get(i);

            final Line3D outLine1 = outLines1.get(i);
            final Line3D outLine2 = outLines2.get(i);

            expectedLine.normalize();
            outLine1.normalize();
            outLine2.normalize();

            // check correctness
            assertEquals(expectedLine.getPlane1().getA(),
                    outLine1.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(),
                    outLine1.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(),
                    outLine1.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(),
                    outLine1.getPlane1().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane2().getA(),
                    outLine1.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(),
                    outLine1.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(),
                    outLine1.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(),
                    outLine1.getPlane2().getD(), ABSOLUTE_ERROR);


            assertEquals(expectedLine.getPlane1().getA(),
                    outLine2.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(),
                    outLine2.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(),
                    outLine2.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(),
                    outLine2.getPlane1().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane2().getA(),
                    outLine2.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(),
                    outLine2.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(),
                    outLine2.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(),
                    outLine2.getPlane2().getD(), ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testTransformAndOverwriteLines() throws CoincidentPointsException,
            CoincidentPlanesException, AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        final ArrayList<Line3D> inputLines = new ArrayList<>(size);
        final ArrayList<Line3D> expectedLines = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final Point3D point1 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final Point3D point2 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            final Line3D line = new Line3D(point1, point2);
            inputLines.add(line);

            final Line3D expectedLine = new Line3D(point1, point2);
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }

        transformation.transformAndOverwriteLines(inputLines);

        // check correctness
        assertEquals(inputLines.size(), size);
        for (int i = 0; i < size; i++) {
            final Line3D expectedLine = expectedLines.get(i);

            final Line3D line = inputLines.get(i);

            expectedLine.normalize();
            line.normalize();

            // check correctness
            assertEquals(expectedLine.getPlane1().getA(),
                    line.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(),
                    line.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(),
                    line.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(),
                    line.getPlane1().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane2().getA(),
                    line.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(),
                    line.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(),
                    line.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(),
                    line.getPlane2().getD(), ABSOLUTE_ERROR);
        }
    }

    @Test
    public void testTransformPolygon() throws NotEnoughVerticesException,
            AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        final ArrayList<Point3D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point3D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, transformation);

            expectedPoints.add(expectedPoint);
        }

        final Polygon3D inputPolygon = new Polygon3D(inputPoints);
        final Polygon3D expectedPolygon = new Polygon3D(expectedPoints);


        final Polygon3D outPolygon1 = transformation.transformAndReturnNew(
                inputPolygon);
        final Polygon3D outPolygon2 = new Polygon3D(inputPoints);
        transformation.transform(inputPolygon, outPolygon2);

        // check correctness
        assertEquals(outPolygon1.getVertices().size(),
                inputPolygon.getVertices().size());
        assertEquals(outPolygon2.getVertices().size(),
                inputPolygon.getVertices().size());
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedPolygon.getVertices().get(i);

            final Point3D outPoint1 = outPolygon1.getVertices().get(i);
            final Point3D outPoint2 = outPolygon2.getVertices().get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }

        transformation.transform(inputPolygon);

        // check correctness
        assertEquals(expectedPolygon.getVertices().size(),
                inputPolygon.getVertices().size());
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedPolygon.getVertices().get(i);

            final Point3D outPoint = outPolygon1.getVertices().get(i);

            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testTransformTriangle() throws AlgebraException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final int size = Triangle2D.NUM_VERTICES;

        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        final ArrayList<Point3D> inputPoints = new ArrayList<>(size);
        final ArrayList<Point3D> expectedPoints = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final Point3D point = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final Point3D expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, transformation);

            expectedPoints.add(expectedPoint);
        }

        final Triangle3D inputTriangle = new Triangle3D(inputPoints.get(0),
                inputPoints.get(1), inputPoints.get(2));
        final Triangle3D expectedTriangle = new Triangle3D(expectedPoints.get(0),
                expectedPoints.get(1), expectedPoints.get(2));

        final Triangle3D outTriangle1 = transformation.transformAndReturnNew(
                inputTriangle);
        final Triangle3D outTriangle2 = new Triangle3D(
                new InhomogeneousPoint3D(inputPoints.get(0)),
                new InhomogeneousPoint3D(inputPoints.get(1)),
                new InhomogeneousPoint3D(inputPoints.get(2)));
        transformation.transform(inputTriangle, outTriangle2);

        // check correctness
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedTriangle.getVertices().get(i);

            final Point3D outPoint1 = outTriangle1.getVertices().get(i);
            final Point3D outPoint2 = outTriangle2.getVertices().get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }

        transformation.transform(inputTriangle);

        // check correctness
        for (int i = 0; i < size; i++) {
            final Point3D expectedPoint = expectedTriangle.getVertices().get(i);

            final Point3D outPoint = inputTriangle.getVertices().get(i);

            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testTransformCamera() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // generate random metric 3D point
        final InhomogeneousPoint3D metricPoint = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // generate random camera
        final Matrix cameraMatrix = Matrix.createWithUniformRandomValues(
                PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final PinholeCamera camera = new PinholeCamera(cameraMatrix);

        // project metric point
        final Point2D p1 = camera.project(metricPoint);

        // generate arbitrary transformation
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);
        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        // transform metric point and camera
        final Point3D affinePoint = transformation.transformAndReturnNew(metricPoint);
        final PinholeCamera affineCamera1 = transformation.transformAndReturnNew(
                camera);
        final PinholeCamera affineCamera2 = new PinholeCamera();
        transformation.transform(camera, affineCamera2);

        transformation.transform(camera);

        // project affine point with affine camera
        final Point2D p2 = affineCamera1.project(affinePoint);
        final Point2D p3 = affineCamera2.project(affinePoint);
        final Point2D p4 = camera.project(affinePoint);

        // check that all projected points p1, p2, p3, p4 are still the same
        assertTrue(p1.equals(p2, ABSOLUTE_ERROR));
        assertTrue(p1.equals(p3, ABSOLUTE_ERROR));
        assertTrue(p1.equals(p4, ABSOLUTE_ERROR));
    }

    @Test
    public void testTransformCameraAndPoints() throws AlgebraException,
            GeometryException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        // create intrinsic parameters
        final double horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                MAX_FOCAL_LENGTH);
        final double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final double horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT,
                MAX_PRINCIPAL_POINT);
        final double verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT,
                MAX_PRINCIPAL_POINT);

        final PinholeCameraIntrinsicParameters intrinsic =
                new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                        verticalFocalLength, horizontalPrincipalPoint,
                        verticalPrincipalPoint, skewness);

        // create rotation parameters
        final double alphaEuler = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final double betaEuler = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final double gammaEuler = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));

        final MatrixRotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler,
                gammaEuler);

        // create camera center
        final Point3D cameraCenter = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));

        final PinholeCamera camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // normalize camera to improve accuracy
        camera.normalize();

        // create 6 random point correspondences
        final Point3D point3D1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final Point3D point3D2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final Point3D point3D3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final Point3D point3D4 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final Point3D point3D5 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final Point3D point3D6 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));

        final Point2D point2D1 = camera.project(point3D1);
        final Point2D point2D2 = camera.project(point3D2);
        final Point2D point2D3 = camera.project(point3D3);
        final Point2D point2D4 = camera.project(point3D4);
        final Point2D point2D5 = camera.project(point3D5);
        final Point2D point2D6 = camera.project(point3D6);

        // create transformation
        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);
        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        // transform camera and points
        final PinholeCamera outCamera = transformation.transformAndReturnNew(camera);
        final Point3D outPoint3D1 = transformation.transformAndReturnNew(point3D1);
        final Point3D outPoint3D2 = transformation.transformAndReturnNew(point3D2);
        final Point3D outPoint3D3 = transformation.transformAndReturnNew(point3D3);
        final Point3D outPoint3D4 = transformation.transformAndReturnNew(point3D4);
        final Point3D outPoint3D5 = transformation.transformAndReturnNew(point3D5);
        final Point3D outPoint3D6 = transformation.transformAndReturnNew(point3D6);

        final Point2D outPoint2D1 = outCamera.project(outPoint3D1);
        final Point2D outPoint2D2 = outCamera.project(outPoint3D2);
        final Point2D outPoint2D3 = outCamera.project(outPoint3D3);
        final Point2D outPoint2D4 = outCamera.project(outPoint3D4);
        final Point2D outPoint2D5 = outCamera.project(outPoint3D5);
        final Point2D outPoint2D6 = outCamera.project(outPoint3D6);

        outCamera.decompose();

        final Point3D outCameraCenter = transformation.transformAndReturnNew(cameraCenter);
        final Point3D outCameraCenter2 = outCamera.getCameraCenter();

        // check that projection of transformed points on transformed camera does
        // not change projected points
        assertTrue(outPoint2D1.equals(point2D1, ABSOLUTE_ERROR));
        assertTrue(outPoint2D2.equals(point2D2, ABSOLUTE_ERROR));
        assertTrue(outPoint2D3.equals(point2D3, ABSOLUTE_ERROR));
        assertTrue(outPoint2D4.equals(point2D4, ABSOLUTE_ERROR));
        assertTrue(outPoint2D5.equals(point2D5, ABSOLUTE_ERROR));
        assertTrue(outPoint2D6.equals(point2D6, ABSOLUTE_ERROR));

        // check that camera center has been transformed
        assertTrue(outCameraCenter.equals(outCameraCenter2, ABSOLUTE_ERROR));
    }

    @Test
    public void testInverse() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        final Transformation3D invTransformation1 =
                transformation.inverseAndReturnNew();
        final AffineTransformation3D invTransformation2 =
                new AffineTransformation3D();
        transformation.inverse(invTransformation2);

        // check that inverse transformation matrix is the inverse matrix of
        // current transformation
        assertTrue(invTransformation1.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation3D.HOM_COORDS,
                MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));

        assertTrue(invTransformation2.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation3D.HOM_COORDS,
                MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));

        // test transforming a random point by transformation and then by its
        // inverse to ensure it remains the same
        final double[] params = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final Point3D inputPoint = Point3D.create(
                CoordinatesType.INHOMOGENEOUS_COORDINATES, params);

        final Point3D transfPoint = transformation.transformAndReturnNew(inputPoint);

        final Point3D invTransfPoint1 = invTransformation1.transformAndReturnNew(transfPoint);
        final Point3D invTransfPoint2 = invTransformation2.transformAndReturnNew(transfPoint);

        // check correctness
        assertTrue(inputPoint.equals(invTransfPoint1, ABSOLUTE_ERROR));
        assertTrue(inputPoint.equals(invTransfPoint2, ABSOLUTE_ERROR));

        // try inverting original transformation
        transformation.inverse();
        final Point3D outPoint = transformation.transformAndReturnNew(transfPoint);

        assertTrue(inputPoint.equals(outPoint, ABSOLUTE_ERROR));
    }

    @Test
    public void testToProjective() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double theta = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        double norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final Rotation3D rotation = Rotation3D.create(rotAxis, theta);

        final double[] translation = new double[
                MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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

        final AffineParameters3D affineParams = new AffineParameters3D(scaleX, scaleY,
                scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final AffineTransformation3D transformation =
                new AffineTransformation3D(affineParams, rotation, translation);

        final Matrix expectedMatrix = transformation.asMatrix();
        norm = Utils.normF(expectedMatrix);
        expectedMatrix.multiplyByScalar(1.0 / norm);

        final Matrix projectiveMatrix = transformation.toProjective().asMatrix();
        norm = Utils.normF(projectiveMatrix);
        projectiveMatrix.multiplyByScalar(1.0 / norm);

        // check equal-ness up to scale
        assertTrue(expectedMatrix.equals(projectiveMatrix, ABSOLUTE_ERROR));
    }

    @Test
    public void testCombine() throws AlgebraException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] translation1 = new double[
                AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scaleX1 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double scaleY1 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double scaleZ1 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double skewnessXY1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessYZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double theta1 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis1 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        double norm = Utils.normF(rotAxis1);
        ArrayUtils.multiplyByScalar(rotAxis1, 1.0 / norm, rotAxis1);

        final double[] translation2 = new double[
                AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final double scaleX2 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double scaleY2 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double scaleZ2 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final double skewnessXY2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessXZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double skewnessYZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);
        final double theta2 = randomizer.nextDouble(MIN_ANGLE_DEGREES,
                MAX_ANGLE_DEGREES) * Math.PI / 180.0;
        final double[] rotAxis2 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        norm = Utils.normF(rotAxis2);
        ArrayUtils.multiplyByScalar(rotAxis2, 1.0 / norm, rotAxis2);

        final Rotation3D rotation1 = Rotation3D.create(rotAxis1, theta1);
        final AffineParameters3D affineParams1 = new AffineParameters3D(scaleX1,
                scaleY1, scaleZ1, skewnessXY1, skewnessXZ1, skewnessYZ1);

        final AffineTransformation3D transformation1 =
                new AffineTransformation3D(affineParams1, rotation1,
                        translation1);

        final Rotation3D rotation2 = Rotation3D.create(rotAxis2, theta2);
        final AffineParameters3D affineParams2 = new AffineParameters3D(scaleX2,
                scaleY2, scaleZ2, skewnessXY2, skewnessXZ2, skewnessYZ2);

        final AffineTransformation3D transformation2 =
                new AffineTransformation3D(affineParams2, rotation2,
                        translation2);

        final Matrix expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(
                transformation2.asMatrix());
        final Matrix expectedA = transformation1.getA().multiplyAndReturnNew(
                transformation2.getA());

        final Matrix a1 = new Matrix(transformation1.getA());
        final Matrix t2 = Matrix.newFromArray(translation2, true);
        a1.multiply(t2);
        final double[] expectedTranslation = a1.toArray();
        ArrayUtils.sum(expectedTranslation, translation1, expectedTranslation);

        // combine and return result as a new transformation
        final AffineTransformation3D transformation3 =
                transformation1.combineAndReturnNew(transformation2);
        // combine into transformation1
        transformation1.combine(transformation2);

        // both matrices m1 and m3 need to be equal
        final Matrix m3 = transformation3.asMatrix();
        final Matrix m1 = transformation1.asMatrix();

        // check correctness
        assertTrue(m1.equals(m3, ABSOLUTE_ERROR));

        // besides, resulting transformation matrices need to be equal to
        // expected matrix
        assertTrue(m1.equals(expectedMatrix, ABSOLUTE_ERROR));
        assertTrue(m3.equals(expectedMatrix, ABSOLUTE_ERROR));

        // check correctness of A and translation
        assertTrue(transformation1.getA().equals(expectedA, ABSOLUTE_ERROR));
        assertTrue(transformation3.getA().equals(expectedA, ABSOLUTE_ERROR));

        assertArrayEquals(transformation1.getTranslation(), expectedTranslation,
                ABSOLUTE_ERROR);
        assertArrayEquals(transformation3.getTranslation(), expectedTranslation,
                ABSOLUTE_ERROR);
    }

    @Test
    public void testSetTransformationFromPoints() throws WrongSizeException,
            DecomposerException, CoincidentPointsException {

        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(
                    AffineTransformation3D.INHOM_COORDS,
                    AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
            final double norm = Utils.normF(a);
            // normalize T to increase accuracy
            a.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

        final double[] translation = new double[
                AffineTransformation3D.INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(translation, -1.0, 1.0);

        final AffineTransformation3D transformation1 =
                new AffineTransformation3D(a, translation);

        // generate 4 non coincident random points
        Point3D inputPoint1;
        Point3D inputPoint2;
        Point3D inputPoint3;
        Point3D inputPoint4;
        Point3D outputPoint1;
        Point3D outputPoint2;
        Point3D outputPoint3;
        Point3D outputPoint4;
        // build matrix initialized to zero
        final Matrix m = new Matrix(12, 13);
        do {
            final Matrix coordsMatrix = Matrix.createWithUniformRandomValues(4, 3,
                    -1.0, 1.0);

            double[] coords = coordsMatrix.getSubmatrixAsArray(0, 0, 0,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint1 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(1, 0, 1,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint2 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(2, 0, 2,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint3 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(3, 0, 3,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint4 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            // Transform points using transformation
            outputPoint1 = transformation1.transformAndReturnNew(
                    inputPoint1);
            outputPoint2 = transformation1.transformAndReturnNew(
                    inputPoint2);
            outputPoint3 = transformation1.transformAndReturnNew(
                    inputPoint3);
            outputPoint4 = transformation1.transformAndReturnNew(
                    inputPoint4);

            // 1st pair of points
            double iX = inputPoint1.getHomX();
            double iY = inputPoint1.getHomY();
            double iZ = inputPoint1.getHomZ();
            double iW = inputPoint1.getHomW();

            double oX = outputPoint1.getHomX();
            double oY = outputPoint1.getHomY();
            double oZ = outputPoint1.getHomZ();
            double oW = outputPoint1.getHomW();

            double oWiX = oW * iX;
            double oWiY = oW * iY;
            double oWiZ = oW * iZ;
            double oWiW = oW * iW;

            double oXiW = oX * iW;
            double oYiW = oY * iW;
            double oZiW = oZ * iW;

            double norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY +
                    oWiZ * oWiZ + oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(0, 0, oWiX / norm);
            m.setElementAt(0, 1, oWiY / norm);
            m.setElementAt(0, 2, oWiZ / norm);
            m.setElementAt(0, 9, oWiW / norm);
            m.setElementAt(0, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(1, 3, oWiX / norm);
            m.setElementAt(1, 4, oWiY / norm);
            m.setElementAt(1, 5, oWiZ / norm);
            m.setElementAt(1, 10, oWiW / norm);
            m.setElementAt(1, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oZiW * oZiW);

            m.setElementAt(2, 6, oWiX / norm);
            m.setElementAt(2, 7, oWiY / norm);
            m.setElementAt(2, 8, oWiZ / norm);
            m.setElementAt(2, 11, oWiW / norm);
            m.setElementAt(2, 12, -oZiW / norm);


            // 2nd pair of points
            iX = inputPoint2.getHomX();
            iY = inputPoint2.getHomY();
            iZ = inputPoint2.getHomZ();
            iW = inputPoint2.getHomW();

            oX = outputPoint2.getHomX();
            oY = outputPoint2.getHomY();
            oZ = outputPoint2.getHomZ();
            oW = outputPoint2.getHomW();

            oWiX = oW * iX;
            oWiY = oW * iY;
            oWiZ = oW * iZ;
            oWiW = oW * iW;

            oXiW = oX * iW;
            oYiW = oY * iW;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(3, 0, oWiX / norm);
            m.setElementAt(3, 1, oWiY / norm);
            m.setElementAt(3, 2, oWiZ / norm);
            m.setElementAt(3, 9, oWiW / norm);
            m.setElementAt(3, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(4, 3, oWiX / norm);
            m.setElementAt(4, 4, oWiY / norm);
            m.setElementAt(4, 5, oWiZ / norm);
            m.setElementAt(4, 10, oWiW / norm);
            m.setElementAt(4, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oZiW * oZiW);

            m.setElementAt(5, 6, oWiX / norm);
            m.setElementAt(5, 7, oWiY / norm);
            m.setElementAt(5, 8, oWiZ / norm);
            m.setElementAt(5, 11, oWiW / norm);
            m.setElementAt(5, 12, -oZiW / norm);


            // 3rd pair of points
            iX = inputPoint3.getHomX();
            iY = inputPoint3.getHomY();
            iZ = inputPoint3.getHomZ();
            iW = inputPoint3.getHomW();

            oX = outputPoint3.getHomX();
            oY = outputPoint3.getHomY();
            oZ = outputPoint3.getHomZ();
            oW = outputPoint3.getHomW();

            oWiX = oW * iX;
            oWiY = oW * iY;
            oWiZ = oW * iZ;
            oWiW = oW * iW;

            oXiW = oX * iW;
            oYiW = oY * iW;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(6, 0, oWiX / norm);
            m.setElementAt(6, 1, oWiY / norm);
            m.setElementAt(6, 2, oWiZ / norm);
            m.setElementAt(6, 9, oWiW / norm);
            m.setElementAt(6, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(7, 3, oWiX / norm);
            m.setElementAt(7, 4, oWiY / norm);
            m.setElementAt(7, 5, oWiZ / norm);
            m.setElementAt(7, 10, oWiW / norm);
            m.setElementAt(7, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oZiW * oZiW);

            m.setElementAt(8, 6, oWiX / norm);
            m.setElementAt(8, 7, oWiY / norm);
            m.setElementAt(8, 8, oWiZ / norm);
            m.setElementAt(8, 11, oWiW / norm);
            m.setElementAt(8, 12, -oZiW / norm);

            // 4th pair of points
            iX = inputPoint4.getHomX();
            iY = inputPoint4.getHomY();
            iZ = inputPoint4.getHomZ();
            iW = inputPoint4.getHomW();

            oX = outputPoint4.getHomX();
            oY = outputPoint4.getHomY();
            oZ = outputPoint4.getHomZ();
            oW = outputPoint4.getHomW();

            oWiX = oW * iX;
            oWiY = oW * iY;
            oWiZ = oW * iZ;
            oWiW = oW * iW;

            oXiW = oX * iW;
            oYiW = oY * iW;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(9, 0, oWiX / norm);
            m.setElementAt(9, 1, oWiY / norm);
            m.setElementAt(9, 2, oWiZ / norm);
            m.setElementAt(9, 9, oWiW / norm);
            m.setElementAt(9, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(10, 3, oWiX / norm);
            m.setElementAt(10, 4, oWiY / norm);
            m.setElementAt(10, 5, oWiZ / norm);
            m.setElementAt(10, 10, oWiW / norm);
            m.setElementAt(10, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oZiW * oZiW);

            m.setElementAt(11, 6, oWiX / norm);
            m.setElementAt(11, 7, oWiY / norm);
            m.setElementAt(11, 8, oWiZ / norm);
            m.setElementAt(11, 11, oWiW / norm);
            m.setElementAt(11, 12, -oZiW / norm);
        } while (Utils.rank(m) < 12);


        // Now build another transformation
        final AffineTransformation3D transformation2 =
                new AffineTransformation3D();

        // estimate transformation from corresponding points
        transformation2.setTransformationFromPoints(inputPoint1,
                inputPoint2, inputPoint3, inputPoint4, outputPoint1,
                outputPoint2, outputPoint3, outputPoint4);

        // check correctness of transformation by checking transformed points
        assertTrue(outputPoint1.equals(new InhomogeneousPoint3D(
                        transformation2.transformAndReturnNew(inputPoint1)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint2.equals(new InhomogeneousPoint3D(
                        transformation2.transformAndReturnNew(inputPoint2)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint3.equals(new InhomogeneousPoint3D(
                        transformation2.transformAndReturnNew(inputPoint3)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint4.equals(new InhomogeneousPoint3D(
                        transformation2.transformAndReturnNew(inputPoint4)),
                ABSOLUTE_ERROR));

        // Force CoincidentPointsException
        try {
            transformation2.setTransformationFromPoints(inputPoint1,
                    inputPoint1, inputPoint3, inputPoint4, outputPoint1,
                    outputPoint1, outputPoint3, outputPoint4);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
    }

    @Test
    public void testConstructorFromPoints() throws WrongSizeException,
            DecomposerException, CoincidentPointsException {

        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(
                    AffineTransformation3D.INHOM_COORDS,
                    AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
            final double norm = Utils.normF(a);
            // normalize T to increase accuracy
            a.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

        final double[] translation = new double[
                AffineTransformation3D.INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(translation, -1.0, 1.0);

        final AffineTransformation3D transformation1 =
                new AffineTransformation3D(a, translation);

        // generate 4 non coincident random points
        Point3D inputPoint1;
        Point3D inputPoint2;
        Point3D inputPoint3;
        Point3D inputPoint4;
        Point3D outputPoint1;
        Point3D outputPoint2;
        Point3D outputPoint3;
        Point3D outputPoint4;
        // build matrix initialized to zero
        final Matrix m = new Matrix(12, 13);
        do {
            final Matrix coordsMatrix = Matrix.createWithUniformRandomValues(4, 3,
                    -1.0, 1.0);

            double[] coords = coordsMatrix.getSubmatrixAsArray(0, 0, 0,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint1 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(1, 0, 1,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint2 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(2, 0, 2,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint3 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(3, 0, 3,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint4 = Point3D.create(
                    CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            // Transform points using transformation
            outputPoint1 = transformation1.transformAndReturnNew(
                    inputPoint1);
            outputPoint2 = transformation1.transformAndReturnNew(
                    inputPoint2);
            outputPoint3 = transformation1.transformAndReturnNew(
                    inputPoint3);
            outputPoint4 = transformation1.transformAndReturnNew(
                    inputPoint4);

            // 1st pair of points
            double iX = inputPoint1.getHomX();
            double iY = inputPoint1.getHomY();
            double iZ = inputPoint1.getHomZ();
            double iW = inputPoint1.getHomW();

            double oX = outputPoint1.getHomX();
            double oY = outputPoint1.getHomY();
            double oZ = outputPoint1.getHomZ();
            double oW = outputPoint1.getHomW();

            double oWiX = oW * iX;
            double oWiY = oW * iY;
            double oWiZ = oW * iZ;
            double oWiW = oW * iW;

            double oXiW = oX * iW;
            double oYiW = oY * iW;
            double oZiW = oZ * iW;

            double norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY +
                    oWiZ * oWiZ + oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(0, 0, oWiX / norm);
            m.setElementAt(0, 1, oWiY / norm);
            m.setElementAt(0, 2, oWiZ / norm);
            m.setElementAt(0, 9, oWiW / norm);
            m.setElementAt(0, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(1, 3, oWiX / norm);
            m.setElementAt(1, 4, oWiY / norm);
            m.setElementAt(1, 5, oWiZ / norm);
            m.setElementAt(1, 10, oWiW / norm);
            m.setElementAt(1, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oZiW * oZiW);

            m.setElementAt(2, 6, oWiX / norm);
            m.setElementAt(2, 7, oWiY / norm);
            m.setElementAt(2, 8, oWiZ / norm);
            m.setElementAt(2, 11, oWiW / norm);
            m.setElementAt(2, 12, -oZiW / norm);

            // 2nd pair of points
            iX = inputPoint2.getHomX();
            iY = inputPoint2.getHomY();
            iZ = inputPoint2.getHomZ();
            iW = inputPoint2.getHomW();

            oX = outputPoint2.getHomX();
            oY = outputPoint2.getHomY();
            oZ = outputPoint2.getHomZ();
            oW = outputPoint2.getHomW();

            oWiX = oW * iX;
            oWiY = oW * iY;
            oWiZ = oW * iZ;
            oWiW = oW * iW;

            oXiW = oX * iW;
            oYiW = oY * iW;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(3, 0, oWiX / norm);
            m.setElementAt(3, 1, oWiY / norm);
            m.setElementAt(3, 2, oWiZ / norm);
            m.setElementAt(3, 9, oWiW / norm);
            m.setElementAt(3, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(4, 3, oWiX / norm);
            m.setElementAt(4, 4, oWiY / norm);
            m.setElementAt(4, 5, oWiZ / norm);
            m.setElementAt(4, 10, oWiW / norm);
            m.setElementAt(4, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oZiW * oZiW);

            m.setElementAt(5, 6, oWiX / norm);
            m.setElementAt(5, 7, oWiY / norm);
            m.setElementAt(5, 8, oWiZ / norm);
            m.setElementAt(5, 11, oWiW / norm);
            m.setElementAt(5, 12, -oZiW / norm);

            // 3rd pair of points
            iX = inputPoint3.getHomX();
            iY = inputPoint3.getHomY();
            iZ = inputPoint3.getHomZ();
            iW = inputPoint3.getHomW();

            oX = outputPoint3.getHomX();
            oY = outputPoint3.getHomY();
            oZ = outputPoint3.getHomZ();
            oW = outputPoint3.getHomW();

            oWiX = oW * iX;
            oWiY = oW * iY;
            oWiZ = oW * iZ;
            oWiW = oW * iW;

            oXiW = oX * iW;
            oYiW = oY * iW;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(6, 0, oWiX / norm);
            m.setElementAt(6, 1, oWiY / norm);
            m.setElementAt(6, 2, oWiZ / norm);
            m.setElementAt(6, 9, oWiW / norm);
            m.setElementAt(6, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(7, 3, oWiX / norm);
            m.setElementAt(7, 4, oWiY / norm);
            m.setElementAt(7, 5, oWiZ / norm);
            m.setElementAt(7, 10, oWiW / norm);
            m.setElementAt(7, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oZiW * oZiW);

            m.setElementAt(8, 6, oWiX / norm);
            m.setElementAt(8, 7, oWiY / norm);
            m.setElementAt(8, 8, oWiZ / norm);
            m.setElementAt(8, 11, oWiW / norm);
            m.setElementAt(8, 12, -oZiW / norm);

            // 4th pair of points
            iX = inputPoint4.getHomX();
            iY = inputPoint4.getHomY();
            iZ = inputPoint4.getHomZ();
            iW = inputPoint4.getHomW();

            oX = outputPoint4.getHomX();
            oY = outputPoint4.getHomY();
            oZ = outputPoint4.getHomZ();
            oW = outputPoint4.getHomW();

            oWiX = oW * iX;
            oWiY = oW * iY;
            oWiZ = oW * iZ;
            oWiW = oW * iW;

            oXiW = oX * iW;
            oYiW = oY * iW;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(9, 0, oWiX / norm);
            m.setElementAt(9, 1, oWiY / norm);
            m.setElementAt(9, 2, oWiZ / norm);
            m.setElementAt(9, 9, oWiW / norm);
            m.setElementAt(9, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(10, 3, oWiX / norm);
            m.setElementAt(10, 4, oWiY / norm);
            m.setElementAt(10, 5, oWiZ / norm);
            m.setElementAt(10, 10, oWiW / norm);
            m.setElementAt(10, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ +
                    oWiW * oWiW + oZiW * oZiW);

            m.setElementAt(11, 6, oWiX / norm);
            m.setElementAt(11, 7, oWiY / norm);
            m.setElementAt(11, 8, oWiZ / norm);
            m.setElementAt(11, 11, oWiW / norm);
            m.setElementAt(11, 12, -oZiW / norm);
        } while (Utils.rank(m) < 12);


        // Now build another transformation
        AffineTransformation3D transformation2 =
                new AffineTransformation3D(inputPoint1,
                        inputPoint2, inputPoint3, inputPoint4, outputPoint1,
                        outputPoint2, outputPoint3, outputPoint4);

        // check correctness of transformation by checking transformed points
        assertTrue(outputPoint1.equals(new InhomogeneousPoint3D(
                        transformation2.transformAndReturnNew(inputPoint1)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint2.equals(new InhomogeneousPoint3D(
                        transformation2.transformAndReturnNew(inputPoint2)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint3.equals(new InhomogeneousPoint3D(
                        transformation2.transformAndReturnNew(inputPoint3)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint4.equals(new InhomogeneousPoint3D(
                        transformation2.transformAndReturnNew(inputPoint4)),
                ABSOLUTE_ERROR));

        // Force CoincidentPointsException
        transformation2 = null;
        try {
            transformation2 = new AffineTransformation3D(inputPoint1,
                    inputPoint1, inputPoint3, inputPoint4, outputPoint1,
                    outputPoint1, outputPoint3, outputPoint4);
            fail("CoincidentPointsException expected but not thrown");
        } catch (final CoincidentPointsException ignore) {
        }
        assertNull(transformation2);
    }

    @Test
    public void testSetTransformationFromPlanes() throws CoincidentPlanesException,
            AlgebraException {

        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(
                    AffineTransformation3D.INHOM_COORDS,
                    AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
            final double norm = Utils.normF(a);
            // normalize T to increase accuracy
            a.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

        final double[] translation = new double[
                AffineTransformation3D.INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(translation, -1.0, 1.0);

        final AffineTransformation3D transformation1 =
                new AffineTransformation3D(a, translation);

        // generate 4 non coincident random planes
        Plane inputPlane1;
        Plane inputPlane2;
        Plane inputPlane3;
        Plane inputPlane4;
        Plane outputPlane1;
        Plane outputPlane2;
        Plane outputPlane3;
        Plane outputPlane4;
        // build matrix initialized to zero
        final Matrix m = new Matrix(12, 13);
        do {
            final Matrix paramsMatrix = Matrix.createWithUniformRandomValues(4, 4,
                    -1.0, 1.0);

            double[] params = paramsMatrix.getSubmatrixAsArray(0, 0, 0,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane1 = new Plane(params);

            params = paramsMatrix.getSubmatrixAsArray(1, 0, 1,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane2 = new Plane(params);

            params = paramsMatrix.getSubmatrixAsArray(2, 0, 2,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane3 = new Plane(params);

            params = paramsMatrix.getSubmatrixAsArray(3, 0, 3,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane4 = new Plane(params);

            // Transform planes using transformation
            outputPlane1 = transformation1.transformAndReturnNew(inputPlane1);
            outputPlane2 = transformation1.transformAndReturnNew(inputPlane2);
            outputPlane3 = transformation1.transformAndReturnNew(inputPlane3);
            outputPlane4 = transformation1.transformAndReturnNew(inputPlane4);

            // 1st pair of planes
            double iA = inputPlane1.getA();
            double iB = inputPlane1.getB();
            double iC = inputPlane1.getC();
            double iD = inputPlane1.getD();

            double oA = outputPlane1.getA();
            double oB = outputPlane1.getB();
            double oC = outputPlane1.getC();
            double oD = outputPlane1.getD();

            double oDiA = oD * iA;
            double oDiB = oD * iB;
            double oDiC = oD * iC;
            double oDiD = oD * iD;

            double oAiD = oA * iD;
            double oBiD = oB * iD;
            double oCiD = oC * iD;

            double norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB +
                    oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(0, 0, oDiA / norm);
            m.setElementAt(0, 1, oDiB / norm);
            m.setElementAt(0, 2, oDiC / norm);
            m.setElementAt(0, 9, oDiD / norm);
            m.setElementAt(0, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(1, 3, oDiA / norm);
            m.setElementAt(1, 4, oDiB / norm);
            m.setElementAt(1, 5, oDiC / norm);
            m.setElementAt(1, 10, oDiD / norm);
            m.setElementAt(1, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oCiD * oCiD);

            m.setElementAt(2, 6, oDiA / norm);
            m.setElementAt(2, 7, oDiB / norm);
            m.setElementAt(2, 8, oDiC / norm);
            m.setElementAt(2, 11, oDiD / norm);
            m.setElementAt(2, 12, -oCiD / norm);

            // 2nd pair of planes
            iA = inputPlane2.getA();
            iB = inputPlane2.getB();
            iC = inputPlane2.getC();
            iD = inputPlane2.getD();

            oA = outputPlane2.getA();
            oB = outputPlane2.getB();
            oC = outputPlane2.getC();
            oD = outputPlane2.getD();

            oDiA = oD * iA;
            oDiB = oD * iB;
            oDiC = oD * iC;
            oDiD = oD * iD;

            oAiD = oA * iD;
            oBiD = oB * iD;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(3, 0, oDiA / norm);
            m.setElementAt(3, 1, oDiB / norm);
            m.setElementAt(3, 2, oDiC / norm);
            m.setElementAt(3, 9, oDiD / norm);
            m.setElementAt(3, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(4, 3, oDiA / norm);
            m.setElementAt(4, 4, oDiB / norm);
            m.setElementAt(4, 5, oDiC / norm);
            m.setElementAt(4, 10, oDiD / norm);
            m.setElementAt(4, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oCiD * oCiD);

            m.setElementAt(5, 6, oDiA / norm);
            m.setElementAt(5, 7, oDiB / norm);
            m.setElementAt(5, 8, oDiC / norm);
            m.setElementAt(5, 11, oDiD / norm);
            m.setElementAt(5, 12, -oCiD / norm);

            // 3rd pair of planes
            iA = inputPlane3.getA();
            iB = inputPlane3.getB();
            iC = inputPlane3.getC();
            iD = inputPlane3.getD();

            oA = outputPlane3.getA();
            oB = outputPlane3.getB();
            oC = outputPlane3.getC();
            oD = outputPlane3.getD();

            oDiA = oD * iA;
            oDiB = oD * iB;
            oDiC = oD * iC;
            oDiD = oD * iD;

            oAiD = oA * iD;
            oBiD = oB * iD;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(6, 0, oDiA / norm);
            m.setElementAt(6, 1, oDiB / norm);
            m.setElementAt(6, 2, oDiC / norm);
            m.setElementAt(6, 9, oDiD / norm);
            m.setElementAt(6, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(7, 3, oDiA / norm);
            m.setElementAt(7, 4, oDiB / norm);
            m.setElementAt(7, 5, oDiC / norm);
            m.setElementAt(7, 10, oDiD / norm);
            m.setElementAt(7, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oCiD * oCiD);

            m.setElementAt(8, 6, oDiA / norm);
            m.setElementAt(8, 7, oDiB / norm);
            m.setElementAt(8, 8, oDiC / norm);
            m.setElementAt(8, 11, oDiD / norm);
            m.setElementAt(8, 12, -oCiD / norm);

            // 4th pair of planes
            iA = inputPlane4.getA();
            iB = inputPlane4.getB();
            iC = inputPlane4.getC();
            iD = inputPlane4.getD();

            oA = outputPlane4.getA();
            oB = outputPlane4.getB();
            oC = outputPlane4.getC();
            oD = outputPlane4.getD();

            oDiA = oD * iA;
            oDiB = oD * iB;
            oDiC = oD * iC;
            oDiD = oD * iD;

            oAiD = oA * iD;
            oBiD = oB * iD;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(9, 0, oDiA / norm);
            m.setElementAt(9, 1, oDiB / norm);
            m.setElementAt(9, 2, oDiC / norm);
            m.setElementAt(9, 9, oDiD / norm);
            m.setElementAt(9, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(10, 3, oDiA / norm);
            m.setElementAt(10, 4, oDiB / norm);
            m.setElementAt(10, 5, oDiC / norm);
            m.setElementAt(10, 10, oDiD / norm);
            m.setElementAt(10, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oCiD * oCiD);

            m.setElementAt(11, 6, oDiA / norm);
            m.setElementAt(11, 7, oDiB / norm);
            m.setElementAt(11, 8, oDiC / norm);
            m.setElementAt(11, 11, oDiD / norm);
            m.setElementAt(11, 12, -oCiD / norm);
        } while (Utils.rank(m) < 12);

        // Now build another transformation
        final AffineTransformation3D transformation2 =
                new AffineTransformation3D();

        // estimate transformation from corresponding planes
        transformation2.setTransformationFromPlanes(inputPlane1,
                inputPlane2, inputPlane3, inputPlane4, outputPlane1,
                outputPlane2, outputPlane3, outputPlane4);

        // check correctness of transformation by checking transformed planes
        Plane p = transformation2.transformAndReturnNew(inputPlane1);
        p.normalize();

        assertEquals(Math.abs(outputPlane1.getA()), Math.abs(p.getA()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getB()), Math.abs(p.getB()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getC()), Math.abs(p.getC()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getD()), Math.abs(p.getD()),
                ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane2);
        p.normalize();

        assertEquals(Math.abs(outputPlane2.getA()), Math.abs(p.getA()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getB()), Math.abs(p.getB()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getC()), Math.abs(p.getC()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getD()), Math.abs(p.getD()),
                ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane3);
        p.normalize();

        assertEquals(Math.abs(outputPlane3.getA()), Math.abs(p.getA()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getB()), Math.abs(p.getB()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getC()), Math.abs(p.getC()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getD()), Math.abs(p.getD()),
                ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane4);
        p.normalize();

        assertEquals(Math.abs(outputPlane4.getA()), Math.abs(p.getA()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getB()), Math.abs(p.getB()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getC()), Math.abs(p.getC()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getD()), Math.abs(p.getD()),
                ABSOLUTE_ERROR);

        // Force CoincidentPlanesException
        try {
            transformation2.setTransformationFromPlanes(inputPlane1,
                    inputPlane1, inputPlane3, inputPlane4, outputPlane1,
                    outputPlane1, outputPlane3, outputPlane4);
            fail("CoincidentPlanesException expected but not thrown");
        } catch (final CoincidentPlanesException ignore) {
        }
    }

    @Test
    public void testConstructorFromPlanes() throws CoincidentPlanesException,
            AlgebraException {

        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(
                    AffineTransformation3D.INHOM_COORDS,
                    AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
            final double norm = Utils.normF(a);
            // normalize T to increase accuracy
            a.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

        final double[] translation = new double[
                AffineTransformation3D.INHOM_COORDS];
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        randomizer.fill(translation, -1.0, 1.0);

        final AffineTransformation3D transformation1 =
                new AffineTransformation3D(a, translation);

        // generate 4 non coincident random planes
        Plane inputPlane1;
        Plane inputPlane2;
        Plane inputPlane3;
        Plane inputPlane4;
        Plane outputPlane1;
        Plane outputPlane2;
        Plane outputPlane3;
        Plane outputPlane4;
        // build matrix initialized to zero
        final Matrix m = new Matrix(12, 13);
        do {
            final Matrix paramsMatrix = Matrix.createWithUniformRandomValues(4, 4,
                    -1.0, 1.0);

            double[] params = paramsMatrix.getSubmatrixAsArray(0, 0, 0,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane1 = new Plane(params);

            params = paramsMatrix.getSubmatrixAsArray(1, 0, 1,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane2 = new Plane(params);

            params = paramsMatrix.getSubmatrixAsArray(2, 0, 2,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane3 = new Plane(params);

            params = paramsMatrix.getSubmatrixAsArray(3, 0, 3,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane4 = new Plane(params);

            // Transform planes using transformation
            outputPlane1 = transformation1.transformAndReturnNew(
                    inputPlane1);
            outputPlane2 = transformation1.transformAndReturnNew(
                    inputPlane2);
            outputPlane3 = transformation1.transformAndReturnNew(
                    inputPlane3);
            outputPlane4 = transformation1.transformAndReturnNew(
                    inputPlane4);

            // 1st pair of planes
            double iA = inputPlane1.getA();
            double iB = inputPlane1.getB();
            double iC = inputPlane1.getC();
            double iD = inputPlane1.getD();

            double oA = outputPlane1.getA();
            double oB = outputPlane1.getB();
            double oC = outputPlane1.getC();
            double oD = outputPlane1.getD();

            double oDiA = oD * iA;
            double oDiB = oD * iB;
            double oDiC = oD * iC;
            double oDiD = oD * iD;

            double oAiD = oA * iD;
            double oBiD = oB * iD;
            double oCiD = oC * iD;

            double norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB +
                    oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(0, 0, oDiA / norm);
            m.setElementAt(0, 1, oDiB / norm);
            m.setElementAt(0, 2, oDiC / norm);
            m.setElementAt(0, 9, oDiD / norm);
            m.setElementAt(0, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(1, 3, oDiA / norm);
            m.setElementAt(1, 4, oDiB / norm);
            m.setElementAt(1, 5, oDiC / norm);
            m.setElementAt(1, 10, oDiD / norm);
            m.setElementAt(1, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oCiD * oCiD);

            m.setElementAt(2, 6, oDiA / norm);
            m.setElementAt(2, 7, oDiB / norm);
            m.setElementAt(2, 8, oDiC / norm);
            m.setElementAt(2, 11, oDiD / norm);
            m.setElementAt(2, 12, -oCiD / norm);

            // 2nd pair of planes
            iA = inputPlane2.getA();
            iB = inputPlane2.getB();
            iC = inputPlane2.getC();
            iD = inputPlane2.getD();

            oA = outputPlane2.getA();
            oB = outputPlane2.getB();
            oC = outputPlane2.getC();
            oD = outputPlane2.getD();

            oDiA = oD * iA;
            oDiB = oD * iB;
            oDiC = oD * iC;
            oDiD = oD * iD;

            oAiD = oA * iD;
            oBiD = oB * iD;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(3, 0, oDiA / norm);
            m.setElementAt(3, 1, oDiB / norm);
            m.setElementAt(3, 2, oDiC / norm);
            m.setElementAt(3, 9, oDiD / norm);
            m.setElementAt(3, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(4, 3, oDiA / norm);
            m.setElementAt(4, 4, oDiB / norm);
            m.setElementAt(4, 5, oDiC / norm);
            m.setElementAt(4, 10, oDiD / norm);
            m.setElementAt(4, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oCiD * oCiD);

            m.setElementAt(5, 6, oDiA / norm);
            m.setElementAt(5, 7, oDiB / norm);
            m.setElementAt(5, 8, oDiC / norm);
            m.setElementAt(5, 11, oDiD / norm);
            m.setElementAt(5, 12, -oCiD / norm);

            // 3rd pair of planes
            iA = inputPlane3.getA();
            iB = inputPlane3.getB();
            iC = inputPlane3.getC();
            iD = inputPlane3.getD();

            oA = outputPlane3.getA();
            oB = outputPlane3.getB();
            oC = outputPlane3.getC();
            oD = outputPlane3.getD();

            oDiA = oD * iA;
            oDiB = oD * iB;
            oDiC = oD * iC;
            oDiD = oD * iD;

            oAiD = oA * iD;
            oBiD = oB * iD;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(6, 0, oDiA / norm);
            m.setElementAt(6, 1, oDiB / norm);
            m.setElementAt(6, 2, oDiC / norm);
            m.setElementAt(6, 9, oDiD / norm);
            m.setElementAt(6, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(7, 3, oDiA / norm);
            m.setElementAt(7, 4, oDiB / norm);
            m.setElementAt(7, 5, oDiC / norm);
            m.setElementAt(7, 10, oDiD / norm);
            m.setElementAt(7, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oCiD * oCiD);

            m.setElementAt(8, 6, oDiA / norm);
            m.setElementAt(8, 7, oDiB / norm);
            m.setElementAt(8, 8, oDiC / norm);
            m.setElementAt(8, 11, oDiD / norm);
            m.setElementAt(8, 12, -oCiD / norm);

            // 4th pair of planes
            iA = inputPlane4.getA();
            iB = inputPlane4.getB();
            iC = inputPlane4.getC();
            iD = inputPlane4.getD();

            oA = outputPlane4.getA();
            oB = outputPlane4.getB();
            oC = outputPlane4.getC();
            oD = outputPlane4.getD();

            oDiA = oD * iA;
            oDiB = oD * iB;
            oDiC = oD * iC;
            oDiD = oD * iD;

            oAiD = oA * iD;
            oBiD = oB * iD;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(9, 0, oDiA / norm);
            m.setElementAt(9, 1, oDiB / norm);
            m.setElementAt(9, 2, oDiC / norm);
            m.setElementAt(9, 9, oDiD / norm);
            m.setElementAt(9, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(10, 3, oDiA / norm);
            m.setElementAt(10, 4, oDiB / norm);
            m.setElementAt(10, 5, oDiC / norm);
            m.setElementAt(10, 10, oDiD / norm);
            m.setElementAt(10, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                    oDiD * oDiD + oCiD * oCiD);

            m.setElementAt(11, 6, oDiA / norm);
            m.setElementAt(11, 7, oDiB / norm);
            m.setElementAt(11, 8, oDiC / norm);
            m.setElementAt(11, 11, oDiD / norm);
            m.setElementAt(11, 12, -oCiD / norm);
        } while (Utils.rank(m) < 12);

        // Now build another transformation
        AffineTransformation3D transformation2 =
                new AffineTransformation3D(inputPlane1,
                        inputPlane2, inputPlane3, inputPlane4, outputPlane1,
                        outputPlane2, outputPlane3, outputPlane4);

        // check correctness of transformation by checking transformed planes
        Plane p = transformation2.transformAndReturnNew(inputPlane1);
        p.normalize();

        assertEquals(Math.abs(outputPlane1.getA()), Math.abs(p.getA()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getB()), Math.abs(p.getB()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getC()), Math.abs(p.getC()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getD()), Math.abs(p.getD()),
                ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane2);
        p.normalize();

        assertEquals(Math.abs(outputPlane2.getA()), Math.abs(p.getA()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getB()), Math.abs(p.getB()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getC()), Math.abs(p.getC()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getD()), Math.abs(p.getD()),
                ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane3);
        p.normalize();

        assertEquals(Math.abs(outputPlane3.getA()), Math.abs(p.getA()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getB()), Math.abs(p.getB()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getC()), Math.abs(p.getC()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getD()), Math.abs(p.getD()),
                ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane4);
        p.normalize();

        assertEquals(Math.abs(outputPlane4.getA()), Math.abs(p.getA()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getB()), Math.abs(p.getB()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getC()), Math.abs(p.getC()),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getD()), Math.abs(p.getD()),
                ABSOLUTE_ERROR);

        // Force CoincidentPlanesException
        transformation2 = null;
        try {
            transformation2 = new AffineTransformation3D(inputPlane1,
                    inputPlane1, inputPlane3, inputPlane4, outputPlane1,
                    outputPlane1, outputPlane3, outputPlane4);
            fail("CoincidentPlanesException expected but not thrown");
        } catch (final CoincidentPlanesException ignore) {
        }
        assertNull(transformation2);
    }

    @Test
    public void testSetTransformationFromLines() throws CoincidentLinesException,
            AlgebraException, CoincidentPlanesException {

        for (int t = 0; t < 1; t++) {

            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(
                        AffineTransformation3D.INHOM_COORDS,
                        AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
                final double norm = Utils.normF(a);
                // normalize T to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

            final double[] translation = new double[
                    AffineTransformation3D.INHOM_COORDS];
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);

            final AffineTransformation3D transformation1 =
                    new AffineTransformation3D(a, translation);

            // generate 4 non coincident random lines
            Plane inputPlane1;
            Plane inputPlane2;
            Plane inputPlane3;
            Plane inputPlane4;
            Line3D inputLine1;
            Line3D inputLine2;
            Line3D outputLine1;
            Line3D outputLine2;
            // build matrix initialized to zero
            final Matrix m = new Matrix(12, 13);
            do {
                final Matrix paramsMatrix = Matrix.createWithUniformRandomValues(6, 4,
                        -1.0, 1.0);

                double[] params = paramsMatrix.getSubmatrixAsArray(0, 0, 0,
                        Plane.PLANE_NUMBER_PARAMS - 1);
                inputPlane1 = new Plane(params);

                params = paramsMatrix.getSubmatrixAsArray(1, 0, 1,
                        Plane.PLANE_NUMBER_PARAMS - 1);
                inputPlane2 = new Plane(params);

                inputLine1 = new Line3D(inputPlane1, inputPlane2);

                params = paramsMatrix.getSubmatrixAsArray(2, 0, 2,
                        Plane.PLANE_NUMBER_PARAMS - 1);
                inputPlane3 = new Plane(params);

                params = paramsMatrix.getSubmatrixAsArray(3, 0, 3,
                        Plane.PLANE_NUMBER_PARAMS - 1);
                inputPlane4 = new Plane(params);

                inputLine2 = new Line3D(inputPlane3, inputPlane4);

                // Transform lines using transformation
                outputLine1 = transformation1.transformAndReturnNew(inputLine1);
                outputLine2 = transformation1.transformAndReturnNew(inputLine2);

                // 1st pair of planes
                double iA = inputLine1.getPlane1().getA();
                double iB = inputLine1.getPlane1().getB();
                double iC = inputLine1.getPlane1().getC();
                double iD = inputLine1.getPlane1().getD();

                double oA = outputLine1.getPlane1().getA();
                double oB = outputLine1.getPlane1().getB();
                double oC = outputLine1.getPlane1().getC();
                double oD = outputLine1.getPlane1().getD();

                double oDiA = oD * iA;
                double oDiB = oD * iB;
                double oDiC = oD * iC;
                double oDiD = oD * iD;

                double oAiD = oA * iD;
                double oBiD = oB * iD;
                double oCiD = oC * iD;

                double norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB +
                        oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(0, 0, oDiA / norm);
                m.setElementAt(0, 1, oDiB / norm);
                m.setElementAt(0, 2, oDiC / norm);
                m.setElementAt(0, 9, oDiD / norm);
                m.setElementAt(0, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(1, 3, oDiA / norm);
                m.setElementAt(1, 4, oDiB / norm);
                m.setElementAt(1, 5, oDiC / norm);
                m.setElementAt(1, 10, oDiD / norm);
                m.setElementAt(1, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oCiD * oCiD);

                m.setElementAt(2, 6, oDiA / norm);
                m.setElementAt(2, 7, oDiB / norm);
                m.setElementAt(2, 8, oDiC / norm);
                m.setElementAt(2, 11, oDiD / norm);
                m.setElementAt(2, 12, -oCiD / norm);


                // 2nd pair of planes
                iA = inputLine1.getPlane2().getA();
                iB = inputLine1.getPlane2().getB();
                iC = inputLine1.getPlane2().getC();
                iD = inputLine1.getPlane2().getD();

                oA = outputLine1.getPlane2().getA();
                oB = outputLine1.getPlane2().getB();
                oC = outputLine1.getPlane2().getC();
                oD = outputLine1.getPlane2().getD();

                oDiA = oD * iA;
                oDiB = oD * iB;
                oDiC = oD * iC;
                oDiD = oD * iD;

                oAiD = oA * iD;
                oBiD = oB * iD;
                oCiD = oC * iD;

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(3, 0, oDiA / norm);
                m.setElementAt(3, 1, oDiB / norm);
                m.setElementAt(3, 2, oDiC / norm);
                m.setElementAt(3, 9, oDiD / norm);
                m.setElementAt(3, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(4, 3, oDiA / norm);
                m.setElementAt(4, 4, oDiB / norm);
                m.setElementAt(4, 5, oDiC / norm);
                m.setElementAt(4, 10, oDiD / norm);
                m.setElementAt(4, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oCiD * oCiD);

                m.setElementAt(5, 6, oDiA / norm);
                m.setElementAt(5, 7, oDiB / norm);
                m.setElementAt(5, 8, oDiC / norm);
                m.setElementAt(5, 11, oDiD / norm);
                m.setElementAt(5, 12, -oCiD / norm);

                // 3rd pair of planes
                iA = inputLine2.getPlane1().getA();
                iB = inputLine2.getPlane1().getB();
                iC = inputLine2.getPlane1().getC();
                iD = inputLine2.getPlane1().getD();

                oA = outputLine2.getPlane1().getA();
                oB = outputLine2.getPlane1().getB();
                oC = outputLine2.getPlane1().getC();
                oD = outputLine2.getPlane1().getD();

                oDiA = oD * iA;
                oDiB = oD * iB;
                oDiC = oD * iC;
                oDiD = oD * iD;

                oAiD = oA * iD;
                oBiD = oB * iD;
                oCiD = oC * iD;

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(6, 0, oDiA / norm);
                m.setElementAt(6, 1, oDiB / norm);
                m.setElementAt(6, 2, oDiC / norm);
                m.setElementAt(6, 9, oDiD / norm);
                m.setElementAt(6, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(7, 3, oDiA / norm);
                m.setElementAt(7, 4, oDiB / norm);
                m.setElementAt(7, 5, oDiC / norm);
                m.setElementAt(7, 10, oDiD / norm);
                m.setElementAt(7, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oCiD * oCiD);

                m.setElementAt(8, 6, oDiA / norm);
                m.setElementAt(8, 7, oDiB / norm);
                m.setElementAt(8, 8, oDiC / norm);
                m.setElementAt(8, 11, oDiD / norm);
                m.setElementAt(8, 12, -oCiD / norm);

                // 4th pair of planes
                iA = inputLine2.getPlane2().getA();
                iB = inputLine2.getPlane2().getB();
                iC = inputLine2.getPlane2().getC();
                iD = inputLine2.getPlane2().getD();

                oA = outputLine2.getPlane2().getA();
                oB = outputLine2.getPlane2().getB();
                oC = outputLine2.getPlane2().getC();
                oD = outputLine2.getPlane2().getD();

                oDiA = oD * iA;
                oDiB = oD * iB;
                oDiC = oD * iC;
                oDiD = oD * iD;

                oAiD = oA * iD;
                oBiD = oB * iD;
                oCiD = oC * iD;

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(9, 0, oDiA / norm);
                m.setElementAt(9, 1, oDiB / norm);
                m.setElementAt(9, 2, oDiC / norm);
                m.setElementAt(9, 9, oDiD / norm);
                m.setElementAt(9, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(10, 3, oDiA / norm);
                m.setElementAt(10, 4, oDiB / norm);
                m.setElementAt(10, 5, oDiC / norm);
                m.setElementAt(10, 10, oDiD / norm);
                m.setElementAt(10, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oCiD * oCiD);

                m.setElementAt(11, 6, oDiA / norm);
                m.setElementAt(11, 7, oDiB / norm);
                m.setElementAt(11, 8, oDiC / norm);
                m.setElementAt(11, 11, oDiD / norm);
                m.setElementAt(11, 12, -oCiD / norm);

            } while (Utils.rank(m) < 8);


            // Now build another transformation
            final AffineTransformation3D transformation2 =
                    new AffineTransformation3D();

            // estimate transformation from corresponding lines
            transformation2.setTransformationFromLines(inputLine1, inputLine2,
                    outputLine1, outputLine2);

            // check correctness of lines (up to scale)
            Plane p = transformation2.transformAndReturnNew(inputLine1.getPlane1());
            p.normalize();

            assertEquals(Math.abs(outputLine1.getPlane1().getA()),
                    Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getB()),
                    Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getC()),
                    Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getD()),
                    Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine1.getPlane2());
            p.normalize();

            assertEquals(Math.abs(outputLine1.getPlane2().getA()),
                    Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getB()),
                    Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getC()),
                    Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getD()),
                    Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine2.getPlane1());
            p.normalize();

            assertEquals(Math.abs(outputLine2.getPlane1().getA()),
                    Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getB()),
                    Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getC()),
                    Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getD()),
                    Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine2.getPlane2());
            p.normalize();

            assertEquals(Math.abs(outputLine2.getPlane2().getA()),
                    Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getB()),
                    Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getC()),
                    Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getD()),
                    Math.abs(p.getD()), ABSOLUTE_ERROR);

            // Force CoincidentLinesException
            try {
                transformation2.setTransformationFromLines(inputLine1,
                        inputLine1, outputLine1, outputLine1);
                fail("CoincidentLinesException expected but not thrown");
            } catch (final CoincidentLinesException ignore) {
            }
        }
    }

    @Test
    public void testConstructorFromLines() throws CoincidentLinesException,
            AlgebraException, CoincidentPlanesException {

        for (int t = 0; t < 1; t++) {

            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(
                        AffineTransformation3D.INHOM_COORDS,
                        AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
                final double norm = Utils.normF(a);
                // normalize A to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

            final double[] translation = new double[
                    AffineTransformation3D.INHOM_COORDS];
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            randomizer.fill(translation, -1.0, 1.0);

            final AffineTransformation3D transformation1 =
                    new AffineTransformation3D(a, translation);

            // generate 4 non coincident random lines
            Plane inputPlane1;
            Plane inputPlane2;
            Plane inputPlane3;
            Plane inputPlane4;
            Line3D inputLine1;
            Line3D inputLine2;
            Line3D outputLine1;
            Line3D outputLine2;
            // build matrix initialized to zero
            final Matrix m = new Matrix(12, 13);
            do {
                final Matrix paramsMatrix = Matrix.createWithUniformRandomValues(6, 4,
                        -1.0, 1.0);

                double[] params = paramsMatrix.getSubmatrixAsArray(0, 0, 0,
                        Plane.PLANE_NUMBER_PARAMS - 1);
                inputPlane1 = new Plane(params);

                params = paramsMatrix.getSubmatrixAsArray(1, 0, 1,
                        Plane.PLANE_NUMBER_PARAMS - 1);
                inputPlane2 = new Plane(params);

                inputLine1 = new Line3D(inputPlane1, inputPlane2);

                params = paramsMatrix.getSubmatrixAsArray(2, 0, 2,
                        Plane.PLANE_NUMBER_PARAMS - 1);
                inputPlane3 = new Plane(params);

                params = paramsMatrix.getSubmatrixAsArray(3, 0, 3,
                        Plane.PLANE_NUMBER_PARAMS - 1);
                inputPlane4 = new Plane(params);

                inputLine2 = new Line3D(inputPlane3, inputPlane4);

                // Transform lines using transformation
                outputLine1 = transformation1.transformAndReturnNew(inputLine1);
                outputLine2 = transformation1.transformAndReturnNew(inputLine2);

                // 1st pair of planes
                double iA = inputLine1.getPlane1().getA();
                double iB = inputLine1.getPlane1().getB();
                double iC = inputLine1.getPlane1().getC();
                double iD = inputLine1.getPlane1().getD();

                double oA = outputLine1.getPlane1().getA();
                double oB = outputLine1.getPlane1().getB();
                double oC = outputLine1.getPlane1().getC();
                double oD = outputLine1.getPlane1().getD();

                double oDiA = oD * iA;
                double oDiB = oD * iB;
                double oDiC = oD * iC;
                double oDiD = oD * iD;

                double oAiD = oA * iD;
                double oBiD = oB * iD;
                double oCiD = oC * iD;

                double norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB +
                        oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(0, 0, oDiA / norm);
                m.setElementAt(0, 1, oDiB / norm);
                m.setElementAt(0, 2, oDiC / norm);
                m.setElementAt(0, 9, oDiD / norm);
                m.setElementAt(0, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(1, 3, oDiA / norm);
                m.setElementAt(1, 4, oDiB / norm);
                m.setElementAt(1, 5, oDiC / norm);
                m.setElementAt(1, 10, oDiD / norm);
                m.setElementAt(1, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oCiD * oCiD);

                m.setElementAt(2, 6, oDiA / norm);
                m.setElementAt(2, 7, oDiB / norm);
                m.setElementAt(2, 8, oDiC / norm);
                m.setElementAt(2, 11, oDiD / norm);
                m.setElementAt(2, 12, -oCiD / norm);

                // 2nd pair of planes
                iA = inputLine1.getPlane2().getA();
                iB = inputLine1.getPlane2().getB();
                iC = inputLine1.getPlane2().getC();
                iD = inputLine1.getPlane2().getD();

                oA = outputLine1.getPlane2().getA();
                oB = outputLine1.getPlane2().getB();
                oC = outputLine1.getPlane2().getC();
                oD = outputLine1.getPlane2().getD();

                oDiA = oD * iA;
                oDiB = oD * iB;
                oDiC = oD * iC;
                oDiD = oD * iD;

                oAiD = oA * iD;
                oBiD = oB * iD;
                oCiD = oC * iD;

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(3, 0, oDiA / norm);
                m.setElementAt(3, 1, oDiB / norm);
                m.setElementAt(3, 2, oDiC / norm);
                m.setElementAt(3, 9, oDiD / norm);
                m.setElementAt(3, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(4, 3, oDiA / norm);
                m.setElementAt(4, 4, oDiB / norm);
                m.setElementAt(4, 5, oDiC / norm);
                m.setElementAt(4, 10, oDiD / norm);
                m.setElementAt(4, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oCiD * oCiD);

                m.setElementAt(5, 6, oDiA / norm);
                m.setElementAt(5, 7, oDiB / norm);
                m.setElementAt(5, 8, oDiC / norm);
                m.setElementAt(5, 11, oDiD / norm);
                m.setElementAt(5, 12, -oCiD / norm);

                // 3rd pair of planes
                iA = inputLine2.getPlane1().getA();
                iB = inputLine2.getPlane1().getB();
                iC = inputLine2.getPlane1().getC();
                iD = inputLine2.getPlane1().getD();

                oA = outputLine2.getPlane1().getA();
                oB = outputLine2.getPlane1().getB();
                oC = outputLine2.getPlane1().getC();
                oD = outputLine2.getPlane1().getD();

                oDiA = oD * iA;
                oDiB = oD * iB;
                oDiC = oD * iC;
                oDiD = oD * iD;

                oAiD = oA * iD;
                oBiD = oB * iD;
                oCiD = oC * iD;

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(6, 0, oDiA / norm);
                m.setElementAt(6, 1, oDiB / norm);
                m.setElementAt(6, 2, oDiC / norm);
                m.setElementAt(6, 9, oDiD / norm);
                m.setElementAt(6, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(7, 3, oDiA / norm);
                m.setElementAt(7, 4, oDiB / norm);
                m.setElementAt(7, 5, oDiC / norm);
                m.setElementAt(7, 10, oDiD / norm);
                m.setElementAt(7, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oCiD * oCiD);

                m.setElementAt(8, 6, oDiA / norm);
                m.setElementAt(8, 7, oDiB / norm);
                m.setElementAt(8, 8, oDiC / norm);
                m.setElementAt(8, 11, oDiD / norm);
                m.setElementAt(8, 12, -oCiD / norm);

                // 4th pair of planes
                iA = inputLine2.getPlane2().getA();
                iB = inputLine2.getPlane2().getB();
                iC = inputLine2.getPlane2().getC();
                iD = inputLine2.getPlane2().getD();

                oA = outputLine2.getPlane2().getA();
                oB = outputLine2.getPlane2().getB();
                oC = outputLine2.getPlane2().getC();
                oD = outputLine2.getPlane2().getD();

                oDiA = oD * iA;
                oDiB = oD * iB;
                oDiC = oD * iC;
                oDiD = oD * iD;

                oAiD = oA * iD;
                oBiD = oB * iD;
                oCiD = oC * iD;

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(9, 0, oDiA / norm);
                m.setElementAt(9, 1, oDiB / norm);
                m.setElementAt(9, 2, oDiC / norm);
                m.setElementAt(9, 9, oDiD / norm);
                m.setElementAt(9, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(10, 3, oDiA / norm);
                m.setElementAt(10, 4, oDiB / norm);
                m.setElementAt(10, 5, oDiC / norm);
                m.setElementAt(10, 10, oDiD / norm);
                m.setElementAt(10, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC +
                        oDiD * oDiD + oCiD * oCiD);

                m.setElementAt(11, 6, oDiA / norm);
                m.setElementAt(11, 7, oDiB / norm);
                m.setElementAt(11, 8, oDiC / norm);
                m.setElementAt(11, 11, oDiD / norm);
                m.setElementAt(11, 12, -oCiD / norm);

            } while (Utils.rank(m) < 8);


            // Now build another transformation
            AffineTransformation3D transformation2 =
                    new AffineTransformation3D(inputLine1, inputLine2,
                            outputLine1, outputLine2);

            // check correctness of lines (up to scale)
            Plane p = transformation2.transformAndReturnNew(inputLine1.getPlane1());
            p.normalize();

            assertEquals(Math.abs(outputLine1.getPlane1().getA()),
                    Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getB()),
                    Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getC()),
                    Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getD()),
                    Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine1.getPlane2());
            p.normalize();

            assertEquals(Math.abs(outputLine1.getPlane2().getA()),
                    Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getB()),
                    Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getC()),
                    Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getD()),
                    Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine2.getPlane1());
            p.normalize();

            assertEquals(Math.abs(outputLine2.getPlane1().getA()),
                    Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getB()),
                    Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getC()),
                    Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getD()),
                    Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine2.getPlane2());
            p.normalize();

            assertEquals(Math.abs(outputLine2.getPlane2().getA()),
                    Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getB()),
                    Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getC()),
                    Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getD()),
                    Math.abs(p.getD()), ABSOLUTE_ERROR);

            // Force CoincidentLinesException
            transformation2 = null;
            try {
                transformation2 = new AffineTransformation3D(inputLine1,
                        inputLine1, outputLine1, outputLine1);
                fail("CoincidentLinesException expected but not thrown");
            } catch (final CoincidentLinesException ignore) {
            }
            assertNull(transformation2);
        }
    }

    private static void transformPoint(
            final Point3D inputPoint,
            final Point3D outputPoint,
            final AffineTransformation3D transformation)
            throws AlgebraException {
        inputPoint.normalize();

        final Matrix a = new Matrix(transformation.getA());

        final double[] coords = new double[
                Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        coords[0] = inputPoint.getInhomX();
        coords[1] = inputPoint.getInhomY();
        coords[2] = inputPoint.getInhomZ();

        final Matrix p = Matrix.newFromArray(coords, true);

        a.multiply(p);

        final double[] translation = transformation.getTranslation();

        outputPoint.setInhomogeneousCoordinates(
                a.getElementAtIndex(0) + translation[0],
                a.getElementAtIndex(1) + translation[1],
                a.getElementAtIndex(2) + translation[2]);
    }

    private static void transformPlane(
            final Plane inputPlane, final Plane outputPlane,
            final AffineTransformation3D transformation) throws WrongSizeException,
            RankDeficientMatrixException, DecomposerException {
        inputPlane.normalize();
        final Matrix t = transformation.asMatrix();
        double norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final Matrix invT = Utils.inverse(t);
        norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final Matrix transInvT = invT.transposeAndReturnNew();
        final Matrix p = Matrix.newFromArray(inputPlane.asArray(), true);

        outputPlane.setParameters(transInvT.multiplyAndReturnNew(p).toArray());
    }

    private static void transformQuadric(
            final Quadric inputQuadric, final Quadric outputQuadric,
            final AffineTransformation3D transformation)
            throws AlgebraException, NonSymmetricMatrixException {

        final Matrix t = transformation.asMatrix();
        final Matrix invT = Utils.inverse(t);
        double norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final Matrix transInvT = invT.transposeAndReturnNew();

        inputQuadric.normalize();
        final Matrix q = inputQuadric.asMatrix();

        final Matrix transQ = transInvT.multiplyAndReturnNew(q.multiplyAndReturnNew(invT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transQ);
        transQ.multiplyByScalar(1.0 / norm);

        outputQuadric.setParameters(transQ);
    }

    private static void transformDualQuadric(
            final DualQuadric inputDualQuadric,
            final DualQuadric outputDualQuadric,
            final AffineTransformation3D transformation) throws WrongSizeException,
            NonSymmetricMatrixException {

        final Matrix t = transformation.asMatrix();
        double norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final Matrix transT = t.transposeAndReturnNew();

        inputDualQuadric.normalize();
        final Matrix dualQ = inputDualQuadric.asMatrix();

        final Matrix transDualQ = t.multiplyAndReturnNew(
                dualQ.multiplyAndReturnNew(transT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transDualQ);
        transDualQ.multiplyByScalar(1.0 / norm);

        outputDualQuadric.setParameters(transDualQ);
    }

    private static void transformLine(
            final Line3D inputLine, final Line3D outputLine,
            final AffineTransformation3D transformation) throws WrongSizeException,
            RankDeficientMatrixException, DecomposerException,
            CoincidentPlanesException {

        inputLine.normalize();
        final Plane inputPlane1 = inputLine.getPlane1();
        final Plane inputPlane2 = inputLine.getPlane2();

        final Plane outputPlane1 = new Plane();
        final Plane outputPlane2 = new Plane();

        transformPlane(inputPlane1, outputPlane1, transformation);
        transformPlane(inputPlane2, outputPlane2, transformation);

        outputLine.setPlanes(outputPlane1, outputPlane2);
    }
}
