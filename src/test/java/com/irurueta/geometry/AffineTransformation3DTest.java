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
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class AffineTransformation3DTest {

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
    void testConstructor() throws AlgebraException, RotationException {
        // Test empty constructor
        var transformation = new AffineTransformation3D();

        // check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleX(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXY(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessYZ(), 0.0);
        assertTrue(transformation.getA().equals(transformation.getParameters().asMatrix().multiplyAndReturnNew(
                transformation.getRotation().asInhomogeneousMatrix()), ABSOLUTE_ERROR));
        assertNotNull(transformation.getA());

        // Test constructor with A matrix
        var a = Matrix.createWithUniformRandomValues(AffineTransformation3D.INHOM_COORDS,
                AffineTransformation3D.INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        transformation = new AffineTransformation3D(a);

        // check correctness
        assertNotNull(transformation.getRotation());
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertNotNull(transformation.getParameters());

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D((Matrix) null));

        // Force IllegalArgumentException
        final var badA = new Matrix(AffineTransformation3D.INHOM_COORDS + 1,
                AffineTransformation3D.INHOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> new AffineTransformation3D(badA));

        // Test constructor with scale
        final var randomizer = new UniformRandomizer();
        final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        transformation = new AffineTransformation3D(scale);

        // check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslation()[2], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);
        assertEquals(scale, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());

        // Test constructor with rotation
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);
        transformation = new AffineTransformation3D(rotation);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslation()[2], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D((Rotation3D) null));

        // Test constructor with scale and rotation
        transformation = new AffineTransformation3D(scale, rotation);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslation()[2], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);
        assertEquals(scale, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D(scale, (Rotation3D) null));

        // Test constructor with affine parameters and rotation
        final var scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleZ = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var params = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);
        transformation = new AffineTransformation3D(params, rotation);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslation()[2], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);
        assertEquals(scaleX, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, transformation.getParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scaleZ, transformation.getParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessXY, transformation.getParameters().getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(skewnessXZ, transformation.getParameters().getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessYZ, transformation.getParameters().getSkewnessYZ(), ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D(null, rotation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D(params, null));

        // Test constructor with translation
        final var translation = new double[AffineParameters3D.INHOM_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        transformation = new AffineTransformation3D(translation);

        // check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[2], transformation.getTranslation()[2], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);
        assertEquals(translation[2], transformation.getTranslationZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D((double[]) null));

        final var badTranslation = new double[AffineTransformation3D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> new AffineTransformation3D(badTranslation));

        // Test constructor with matrix A and translation
        transformation = new AffineTransformation3D(a, translation);

        // check correctness
        assertNotNull(transformation.getRotation());
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[2], transformation.getTranslation()[2], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);
        assertEquals(translation[2], transformation.getTranslationZ(), 0.0);
        assertNotNull(transformation.getParameters());
        assertTrue(a.equals(transformation.getA(), ABSOLUTE_ERROR));

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D((Matrix) null, translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D(a, null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new AffineTransformation3D(badA, translation));
        assertThrows(IllegalArgumentException.class, () -> new AffineTransformation3D(a, badTranslation));

        // Test constructor with scale and translation
        transformation = new AffineTransformation3D(scale, translation);

        // Check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[2], transformation.getTranslation()[2], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);
        assertEquals(translation[2], transformation.getTranslationZ(), 0.0);
        assertEquals(scale, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D(scale, (double[]) null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new AffineTransformation3D(scale, badTranslation));

        // Test constructor with rotation and translation
        transformation = new AffineTransformation3D(rotation, translation);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[2], transformation.getTranslation()[2], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);
        assertEquals(translation[2], transformation.getTranslationZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D((Rotation3D) null, translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D(rotation, null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new AffineTransformation3D(rotation, badTranslation));

        // Test constructor with scale, rotation and translation
        transformation = new AffineTransformation3D(scale, rotation, translation);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[2], transformation.getTranslation()[2], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);
        assertEquals(translation[2], transformation.getTranslationZ(), 0.0);
        assertEquals(scale, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D(scale, null, translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D(scale, rotation, null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new AffineTransformation3D(scale, rotation, badTranslation));

        // Test constructor with affine parameters, rotation and translation
        transformation = new AffineTransformation3D(params, rotation, translation);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], 0.0);
        assertEquals(translation[1], transformation.getTranslation()[1], 0.0);
        assertEquals(translation[2], transformation.getTranslation()[2], 0.0);
        assertEquals(translation[0], transformation.getTranslationX(), 0.0);
        assertEquals(translation[1], transformation.getTranslationY(), 0.0);
        assertEquals(translation[2], transformation.getTranslationZ(), 0.0);
        assertEquals(scaleX, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, transformation.getParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scaleZ, transformation.getParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessXY, transformation.getParameters().getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(skewnessXZ, transformation.getParameters().getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessYZ, transformation.getParameters().getSkewnessYZ(), ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D(null, rotation, translation));
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D(params, null, translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new AffineTransformation3D(params, rotation, null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new AffineTransformation3D(params, rotation,
                badTranslation));
    }

    @Test
    void testGetSetA() throws WrongSizeException {
        final var a = Matrix.createWithUniformRandomValues(AffineTransformation3D.INHOM_COORDS,
                AffineTransformation3D.INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new AffineTransformation3D();

        // check default value
        assertTrue(transformation.getA().equals(Matrix.identity(AffineParameters3D.INHOM_COORDS,
                AffineParameters3D.INHOM_COORDS), ABSOLUTE_ERROR));

        // set matrix A
        transformation.setA(a);

        // check correctness
        assertTrue(transformation.getA().equals(a, ABSOLUTE_ERROR));

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> transformation.setA(null));

        // Force IllegalArgumentException
        final var badA = new Matrix(AffineTransformation3D.INHOM_COORDS + 1,
                AffineTransformation3D.INHOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> transformation.setA(badA));
    }

    @Test
    void testGetSetRotation() throws RotationException, AlgebraException {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize rotation axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        // test default values
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, transformation.getRotation().getRotationAxis()[2], ABSOLUTE_ERROR);

        // set new value
        transformation.setRotation(rotation);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> transformation.setRotation(null));
    }

    @Test
    void testAddRotation() throws RotationException, AlgebraException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var transformation = new AffineTransformation3D();

            final var randomizer = new UniformRandomizer();
            final var theta1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final var theta2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final var rotAxis1 = new double[Rotation3D.INHOM_COORDS];
            randomizer.fill(rotAxis1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            var norm = Utils.normF(rotAxis1);
            ArrayUtils.multiplyByScalar(rotAxis1, 1.0 / norm, rotAxis1);

            final var rotAxis2 = new double[Rotation3D.INHOM_COORDS];
            randomizer.fill(rotAxis2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            norm = Utils.normF(rotAxis2);
            ArrayUtils.multiplyByScalar(rotAxis2, 1.0 / norm, rotAxis2);

            final var rotation1 = Rotation3D.create(rotAxis1, theta1);
            final var rotation2 = Rotation3D.create(rotAxis2, theta2);

            final var combinedRotation = rotation1.combineAndReturnNew(rotation2);

            // set rotation1
            transformation.setRotation(rotation1);

            // check correctness
            if (Math.abs(Math.abs(transformation.getRotation().getRotationAngle()) - Math.abs(theta1))
                    > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(transformation.getRotation().getRotationAngle()), Math.abs(theta1), ABSOLUTE_ERROR);
            assertEquals(Math.abs(transformation.getRotation().getRotationAxis()[0]), Math.abs(rotAxis1[0]),
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(transformation.getRotation().getRotationAxis()[1]), Math.abs(rotAxis1[1]),
                    ABSOLUTE_ERROR);
            assertEquals(Math.abs(transformation.getRotation().getRotationAxis()[2]), Math.abs(rotAxis1[2]),
                    ABSOLUTE_ERROR);

            // add second rotation
            transformation.addRotation(rotation2);

            // check correctness
            if (Math.abs(Math.abs(transformation.getRotation().getRotationAngle())
                    - Math.abs(combinedRotation.getRotationAngle())) > 5.0 * ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(transformation.getRotation().getRotationAngle()),
                    Math.abs(combinedRotation.getRotationAngle()), 5.0 * ABSOLUTE_ERROR);
            assertEquals(Math.abs(transformation.getRotation().getRotationAxis()[0]),
                    Math.abs(combinedRotation.getRotationAxis()[0]), ABSOLUTE_ERROR);
            assertEquals(Math.abs(transformation.getRotation().getRotationAxis()[1]),
                    Math.abs(combinedRotation.getRotationAxis()[1]), ABSOLUTE_ERROR);
            assertEquals(Math.abs(transformation.getRotation().getRotationAxis()[2]),
                    Math.abs(combinedRotation.getRotationAxis()[2]), ABSOLUTE_ERROR);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testGetSetParameters() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleZ = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var params = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        // instantiate transformation
        final var transformation = new AffineTransformation3D();

        // check default values
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getParameters().getSkewnessYZ(), ABSOLUTE_ERROR);

        final var defaultParams = new AffineParameters3D();
        transformation.getParameters(defaultParams);

        assertEquals(AffineParameters3D.DEFAULT_SCALE, defaultParams.getScaleX(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, defaultParams.getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, defaultParams.getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, defaultParams.getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, defaultParams.getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, defaultParams.getSkewnessYZ(), ABSOLUTE_ERROR);

        // set parameters
        transformation.setParameters(params);

        // check correctness
        assertEquals(scaleX, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, transformation.getParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scaleZ, transformation.getParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessXY, transformation.getParameters().getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(skewnessXZ, transformation.getParameters().getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessYZ, transformation.getParameters().getSkewnessYZ(), ABSOLUTE_ERROR);

        final var params2 = new AffineParameters3D();
        transformation.getParameters(params2);

        assertEquals(scaleX, params2.getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, params2.getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scaleZ, params2.getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessXY, params2.getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(skewnessXZ, params2.getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessYZ, params2.getSkewnessYZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslation() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translation = new double[AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var badTranslation = new double[AffineTransformation3D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> transformation.setTranslation(badTranslation));
    }

    @Test
    void testAddTranslation() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translation1 = new double[AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation2 = new double[AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        final var translationCopy = Arrays.copyOf(translation1, AffineTransformation3D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation1[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation1[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // add translation
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[2] + translation2[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation1[2] + translation2[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var badTranslation = new double[AffineTransformation3D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> transformation.addTranslation(badTranslation));
    }

    @Test
    void testGetSetTranslationX() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);

        // set new value
        transformation.setTranslationX(translationX);

        // check correctness
        assertEquals(translationX, transformation.getTranslationX(), 0.0);
    }

    @Test
    void testGetSetTranslationY() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // set new value
        transformation.setTranslationY(translationY);

        // check correctness
        assertEquals(translationY, transformation.getTranslationY(), 0.0);
    }

    @Test
    void testGetSetTranslationZ() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);

        // set new value
        transformation.setTranslationZ(translationZ);

        // check correctness
        assertEquals(translationZ, transformation.getTranslationZ(), 0.0);
    }

    @Test
    void testSetTranslationCoordinates() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translationX, translationY, translationZ);

        // check correctness
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translationX, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translationX, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslationZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslationPoint() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation = new InhomogeneousPoint3D(translationX, translationY, translationZ);

        // check default value
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translationX, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translationX, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        final var translation2 = transformation.getTranslationPoint();
        final var translation3 = Point3D.create();
        transformation.getTranslationPoint(translation3);

        // check correctness
        assertEquals(translation, translation2);
        assertEquals(translation, translation3);
    }

    @Test
    void testAddTranslationX() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);

        // set value
        transformation.setTranslationX(translationX1);

        // check correctness
        assertEquals(translationX1, transformation.getTranslationX(), 0.0);

        // add translation x
        transformation.addTranslationX(translationX2);

        // check correctness
        assertEquals(translationX1 + translationX2, transformation.getTranslationX(), 0.0);
    }

    @Test
    void testAddTranslationY() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // set value
        transformation.setTranslationY(translationY1);

        // check correctness
        assertEquals(translationY1, transformation.getTranslationY(), 0.0);

        // add translation y
        transformation.addTranslationY(translationY2);

        // check correctness
        assertEquals(translationY1 + translationY2, transformation.getTranslationY(), 0.0);
    }

    @Test
    void testAddTranslationZ() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);

        // set value
        transformation.setTranslationZ(translationZ1);

        // Check correctness
        assertEquals(translationZ1, transformation.getTranslationZ(), 0.0);

        // add translationZ
        transformation.addTranslationZ(translationZ2);

        // check correctness
        assertEquals(translationZ1 + translationZ2, transformation.getTranslationZ(), 0.0);
    }

    @Test
    void testAddTranslationCoordinates() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);

        // set values
        transformation.setTranslation(translationX1, translationY1, translationZ1);

        // add translation
        transformation.addTranslation(translationX2, translationY2, translationZ2);

        // check correctness
        assertEquals(translationX1 + translationX2, transformation.getTranslationX(), 0.0);
        assertEquals(translationY1 + translationY2, transformation.getTranslationY(), 0.0);
        assertEquals(translationZ1 + translationZ2, transformation.getTranslationZ(), 0.0);
    }

    @Test
    void testAddTranslationPoint() {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);

        // set values
        transformation.setTranslation(translationX1, translationY1, translationZ1);

        // add translation
        final var translation2 = new InhomogeneousPoint3D(translationX2, translationY2, translationZ2);
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(translationX1 + translationX2, transformation.getTranslationX(), 0.0);
        assertEquals(translationY1 + translationY2, transformation.getTranslationY(), 0.0);
        assertEquals(translationZ1 + translationZ2, transformation.getTranslationZ(), 0.0);
    }

    @Test
    void testGetSetScale() throws AlgebraException {
        final var transformation = new AffineTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

        // check default value
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);

        // set value
        transformation.setScale(scale);

        // check correctness
        assertEquals(scale, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getParameters().getScaleX(), ABSOLUTE_ERROR);
    }

    @Test
    void testAsMatrix() throws WrongSizeException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);
        final var translation = new double[AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleZ = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var rotation = Rotation3D.create(rotAxis, theta);
        final var params = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        var transformation = new AffineTransformation3D(params, rotation, translation);

        final var m = Matrix.identity(AffineTransformation3D.HOM_COORDS, AffineTransformation3D.HOM_COORDS);
        m.setSubmatrix(0, 0, 2, 2,
                params.asMatrix().multiplyAndReturnNew(rotation.asInhomogeneousMatrix()));
        m.setSubmatrix(0, 3, 2, 3, translation);

        var transMatrix1 = transformation.asMatrix();
        var transMatrix2 = new Matrix(AffineTransformation3D.HOM_COORDS, AffineTransformation3D.HOM_COORDS);
        transformation.asMatrix(transMatrix2);

        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));

        // test again by providing A matrix
        Matrix a = Matrix.createWithUniformRandomValues(AffineTransformation3D.INHOM_COORDS,
                AffineTransformation3D.INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var transformation2 = new AffineTransformation3D(a, translation);

        m.setSubmatrix(0, 0, 2, 2, a);

        transMatrix1 = transformation2.asMatrix();
        transMatrix2 = new Matrix(AffineTransformation3D.HOM_COORDS, AffineTransformation3D.HOM_COORDS);
        transformation2.asMatrix(transMatrix2);

        assertTrue(transMatrix1.equals(m, ABSOLUTE_ERROR));
        assertTrue(transMatrix2.equals(m, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        var t = new Matrix(AffineTransformation3D.HOM_COORDS + 1, AffineTransformation3D.HOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> transformation2.asMatrix(t));
    }

    @Test
    void testTransformPoint() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var params = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(params, rotation, translation);

        final var expectedPoint = Point3D.create();
        transformPoint(point, expectedPoint, transformation);

        final var outPoint1 = transformation.transformAndReturnNew(point);
        final var outPoint2 = Point3D.create();
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
    void testTransformPoints() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var params = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(params, rotation, translation);

        final var inputPoints = new ArrayList<Point3D>(size);
        final var expectedPoints = new ArrayList<Point3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, transformation);

            expectedPoints.add(expectedPoint);
        }

        final var outPoints1 = transformation.transformPointsAndReturnNew(inputPoints);
        final var outPoints2 = new ArrayList<Point3D>();
        transformation.transformPoints(inputPoints, outPoints2);

        // check correctness
        assertEquals(outPoints1.size(), inputPoints.size());
        assertEquals(outPoints2.size(), inputPoints.size());
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedPoints.get(i);

            final var outPoint1 = outPoints1.get(i);
            final var outPoint2 = outPoints2.get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testTransformAndOverwritePoints() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var params = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(params, rotation, translation);

        final var inputPoints = new ArrayList<Point3D>(size);
        final var expectedPoints = new ArrayList<Point3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, transformation);

            expectedPoints.add(expectedPoint);
        }

        transformation.transformAndOverwritePoints(inputPoints);

        // check correctness
        assertEquals(inputPoints.size(), size);
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedPoints.get(i);

            final var point = inputPoints.get(i);

            assertTrue(point.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testTransformQuadric() throws NonSymmetricMatrixException, AlgebraException {

        final var randomizer = new UniformRandomizer();

        // create input conic
        // Constructor with params
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var quadric = new Quadric(a, b, c, d, e, f, g, h, i, j);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var params = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(params, rotation, translation);

        // compute expected value
        final var expectedQuadric = new Quadric();
        transformQuadric(quadric, expectedQuadric, transformation);
        expectedQuadric.normalize();

        // make transformation
        final var outQuadric1 = transformation.transformAndReturnNew(quadric);
        final var outQuadric2 = new Quadric();
        transformation.transform(quadric, outQuadric2);

        // check correctness
        outQuadric1.normalize();
        outQuadric2.normalize();

        assertEquals(expectedQuadric.getA(), outQuadric1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), outQuadric1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), outQuadric1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), outQuadric1.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), outQuadric1.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), outQuadric1.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), outQuadric1.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), outQuadric1.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), outQuadric1.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), outQuadric1.getJ(), ABSOLUTE_ERROR);

        assertEquals(expectedQuadric.getA(), outQuadric2.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), outQuadric2.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), outQuadric2.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), outQuadric2.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), outQuadric2.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), outQuadric2.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), outQuadric2.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), outQuadric2.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), outQuadric2.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), outQuadric2.getJ(), ABSOLUTE_ERROR);

        transformation.transform(quadric);

        // check correctness
        quadric.normalize();

        assertEquals(expectedQuadric.getA(), quadric.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), quadric.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), quadric.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), quadric.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), quadric.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), quadric.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), quadric.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), quadric.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), quadric.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), quadric.getJ(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformQuadricAndPoints() throws AlgebraException, GeometryException {

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
            final var m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            point1 = new HomogeneousPoint3D(m.getElementAt(0, 0), m.getElementAt(0, 1),
                    m.getElementAt(0, 2), 1.0);
            point2 = new HomogeneousPoint3D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2), 1.0);
            point3 = new HomogeneousPoint3D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2), 1.0);
            point4 = new HomogeneousPoint3D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2), 1.0);
            point5 = new HomogeneousPoint3D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2), 1.0);
            point6 = new HomogeneousPoint3D(m.getElementAt(5, 0), m.getElementAt(5, 1),
                    m.getElementAt(5, 2), 1.0);
            point7 = new HomogeneousPoint3D(m.getElementAt(6, 0), m.getElementAt(6, 1),
                    m.getElementAt(6, 2), 1.0);
            point8 = new HomogeneousPoint3D(m.getElementAt(7, 0), m.getElementAt(7, 1),
                    m.getElementAt(7, 2), 1.0);
            point9 = new HomogeneousPoint3D(m.getElementAt(8, 0), m.getElementAt(8, 1),
                    m.getElementAt(8, 2), 1.0);

            try {
                quadric = new Quadric(point1, point2, point3, point4, point5, point6, point7, point8, point9);
            } catch (final GeometryException ignore) {
                // if points are not valid, ignore and continue
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
        final var randomizer = new UniformRandomizer();

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        // compute expected value
        final var expectedQuadric = new Quadric();
        transformQuadric(quadric, expectedQuadric, transformation);
        expectedQuadric.normalize();

        // transform quadric and points
        final var outQuadric = transformation.transformAndReturnNew(quadric);
        final var outPoint1 = transformation.transformAndReturnNew(point1);
        final var outPoint2 = transformation.transformAndReturnNew(point2);
        final var outPoint3 = transformation.transformAndReturnNew(point3);
        final var outPoint4 = transformation.transformAndReturnNew(point4);
        final var outPoint5 = transformation.transformAndReturnNew(point5);
        final var outPoint6 = transformation.transformAndReturnNew(point6);
        final var outPoint7 = transformation.transformAndReturnNew(point7);
        final var outPoint8 = transformation.transformAndReturnNew(point8);
        final var outPoint9 = transformation.transformAndReturnNew(point9);

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

        assertEquals(expectedQuadric.getA(), outQuadric.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getB(), outQuadric.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getC(), outQuadric.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getD(), outQuadric.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getE(), outQuadric.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getF(), outQuadric.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getG(), outQuadric.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getH(), outQuadric.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getI(), outQuadric.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedQuadric.getJ(), outQuadric.getJ(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformDualQuadric() throws NonSymmetricMatrixException, AlgebraException {

        final var randomizer = new UniformRandomizer();

        // create input conic
        // Constructor with params
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var g = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var h = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var i = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var j = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var dualQuadric = new DualQuadric(a, b, c, d, e, f, g, h, i, j);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var params = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(params, rotation, translation);

        // compute expected value
        final var expectedDualQuadric = new DualQuadric();
        transformDualQuadric(dualQuadric, expectedDualQuadric, transformation);
        expectedDualQuadric.normalize();

        // make transformation
        final var outDualQuadric1 = transformation.transformAndReturnNew(dualQuadric);
        final var outDualQuadric2 = new DualQuadric();
        transformation.transform(dualQuadric, outDualQuadric2);

        // check correctness
        outDualQuadric1.normalize();
        outDualQuadric2.normalize();

        assertEquals(expectedDualQuadric.getA(), outDualQuadric1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), outDualQuadric1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), outDualQuadric1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), outDualQuadric1.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), outDualQuadric1.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), outDualQuadric1.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), outDualQuadric1.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), outDualQuadric1.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), outDualQuadric1.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), outDualQuadric1.getJ(), ABSOLUTE_ERROR);

        assertEquals(expectedDualQuadric.getA(), outDualQuadric1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), outDualQuadric1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), outDualQuadric1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), outDualQuadric1.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), outDualQuadric1.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), outDualQuadric1.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), outDualQuadric1.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), outDualQuadric1.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), outDualQuadric1.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), outDualQuadric1.getJ(), ABSOLUTE_ERROR);

        transformation.transform(dualQuadric);

        // check correctness
        dualQuadric.normalize();

        assertEquals(expectedDualQuadric.getA(), dualQuadric.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), dualQuadric.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), dualQuadric.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), dualQuadric.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), dualQuadric.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), dualQuadric.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), dualQuadric.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), dualQuadric.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), dualQuadric.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), dualQuadric.getJ(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformDualQuadricAndPlanes() throws AlgebraException, GeometryException {

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
            final var m = Matrix.createWithUniformRandomValues(9, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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
                dualQuadric = new DualQuadric(plane1, plane2, plane3, plane4, plane5, plane6, plane7, plane8, plane9);
            } catch (final GeometryException ignore) {
                // if planes are not valid, ignore and continue
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
        final var randomizer = new UniformRandomizer();

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        // compute expected value
        final var expectedDualQuadric = new DualQuadric();
        transformDualQuadric(dualQuadric, expectedDualQuadric, transformation);
        expectedDualQuadric.normalize();

        // transform dual quadric and planes
        final var outDualQuadric = transformation.transformAndReturnNew(dualQuadric);
        final var outPlane1 = transformation.transformAndReturnNew(plane1);
        final var outPlane2 = transformation.transformAndReturnNew(plane2);
        final var outPlane3 = transformation.transformAndReturnNew(plane3);
        final var outPlane4 = transformation.transformAndReturnNew(plane4);
        final var outPlane5 = transformation.transformAndReturnNew(plane5);
        final var outPlane6 = transformation.transformAndReturnNew(plane6);
        final var outPlane7 = transformation.transformAndReturnNew(plane7);
        final var outPlane8 = transformation.transformAndReturnNew(plane8);
        final var outPlane9 = transformation.transformAndReturnNew(plane9);

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

        assertEquals(expectedDualQuadric.getA(), outDualQuadric.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getB(), outDualQuadric.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getC(), outDualQuadric.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getD(), outDualQuadric.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getE(), outDualQuadric.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getF(), outDualQuadric.getF(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getG(), outDualQuadric.getG(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getH(), outDualQuadric.getH(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getI(), outDualQuadric.getI(), ABSOLUTE_ERROR);
        assertEquals(expectedDualQuadric.getJ(), outDualQuadric.getJ(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformPlane() throws AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var params = new double[Plane.PLANE_NUMBER_PARAMS];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var plane = new Plane(params);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        final var expectedPlane = new Plane();
        transformPlane(plane, expectedPlane, transformation);
        expectedPlane.normalize();

        final var outPlane1 = transformation.transformAndReturnNew(plane);
        final var outPlane2 = new Plane();
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
    void testTransformPlaneAndPoints() throws AlgebraException, GeometryException {

        // create plane from 3 points
        var m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        // ensure we create a matrix with 3 non-linear dependent rows
        while (decomposer.getRank() < 3) {
            m = Matrix.createWithUniformRandomValues(3, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final var point1 = new HomogeneousPoint3D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2),
                m.getElementAt(0, 3));
        final var point2 = new HomogeneousPoint3D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2),
                m.getElementAt(1, 3));
        final var point3 = new HomogeneousPoint3D(m.getElementAt(2, 0),
                m.getElementAt(2, 1),
                m.getElementAt(2, 2),
                m.getElementAt(2, 3));

        point1.normalize();
        point2.normalize();
        point3.normalize();

        final var plane = new Plane(point1, point2, point3);

        // check that points belong to the plane
        assertTrue(plane.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(plane.isLocus(point3, ABSOLUTE_ERROR));

        // create transformation
        final var randomizer = new UniformRandomizer();

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        final var expectedPlane = new Plane();
        transformPlane(plane, expectedPlane, transformation);
        expectedPlane.normalize();

        // transform plane and points
        final var outPlane = transformation.transformAndReturnNew(plane);
        final var outPoint1 = transformation.transformAndReturnNew(point1);
        final var outPoint2 = transformation.transformAndReturnNew(point1);
        final var outPoint3 = transformation.transformAndReturnNew(point1);

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
    void testTransformPlanes() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        final var inputPlanes = new ArrayList<Plane>(size);
        final var expectedPlanes = new ArrayList<Plane>(size);
        for (var i = 0; i < size; i++) {
            final var params = new double[Plane.PLANE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var plane = new Plane(params);
            inputPlanes.add(plane);

            final var expectedPlane = new Plane();
            transformPlane(plane, expectedPlane, transformation);

            expectedPlanes.add(expectedPlane);
        }

        final var outPlanes1 = transformation.transformPlanesAndReturnNew(inputPlanes);
        final var outPlanes2 = new ArrayList<Plane>();
        transformation.transformPlanes(inputPlanes, outPlanes2);

        // check correctness
        assertEquals(outPlanes1.size(), inputPlanes.size());
        assertEquals(outPlanes2.size(), inputPlanes.size());
        for (var i = 0; i < size; i++) {
            final var expectedPlane = expectedPlanes.get(i);

            final var outPlane1 = outPlanes1.get(i);
            final var outPlane2 = outPlanes2.get(i);

            expectedPlane.normalize();
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
        }
    }

    @Test
    void testTransformAndOverwritePlanes() throws AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        final var inputPlanes = new ArrayList<Plane>(size);
        final var expectedPlanes = new ArrayList<Plane>(size);
        for (var i = 0; i < size; i++) {
            final var params = new double[Plane.PLANE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var plane = new Plane(params);
            inputPlanes.add(plane);

            final var expectedPlane = new Plane();
            transformPlane(plane, expectedPlane, transformation);

            expectedPlanes.add(expectedPlane);
        }

        transformation.transformAndOverwritePlanes(inputPlanes);

        // check correctness
        assertEquals(inputPlanes.size(), size);
        for (var i = 0; i < size; i++) {
            final var expectedPlane = expectedPlanes.get(i);

            final var plane = inputPlanes.get(i);

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
    void testTransformLine() throws CoincidentPointsException, CoincidentPlanesException, AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var point1 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var point2 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        final var line = new Line3D(point1, point2);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        final var expectedLine = new Line3D(point1, point2);
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();

        final var outLine1 = transformation.transformAndReturnNew(line);
        final var outLine2 = new Line3D(point1, point2);
        transformation.transform(line, outLine2);

        outLine1.normalize();
        outLine2.normalize();

        // check correctness
        assertEquals(expectedLine.getPlane1().getA(), outLine1.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(), outLine1.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(), outLine1.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(), outLine1.getPlane1().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane2().getA(), outLine1.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(), outLine1.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(), outLine1.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(), outLine1.getPlane2().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane1().getA(), outLine2.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(), outLine2.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(), outLine2.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(), outLine2.getPlane1().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane2().getA(), outLine2.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(), outLine2.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(), outLine2.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(), outLine2.getPlane2().getD(), ABSOLUTE_ERROR);

        transformation.transform(line);

        line.normalize();

        // check correctness
        assertEquals(expectedLine.getPlane1().getA(), line.getPlane1().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getB(), line.getPlane1().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getC(), line.getPlane1().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane1().getD(), line.getPlane1().getD(), ABSOLUTE_ERROR);

        assertEquals(expectedLine.getPlane2().getA(), line.getPlane2().getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getB(), line.getPlane2().getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getC(), line.getPlane2().getC(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getPlane2().getD(), line.getPlane2().getD(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformLines() throws CoincidentPlanesException, CoincidentPointsException, AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        final var inputLines = new ArrayList<Line3D>(size);
        final var expectedLines = new ArrayList<Line3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var point1 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var point2 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            final var line = new Line3D(point1, point2);
            inputLines.add(line);

            final var expectedLine = new Line3D(point1, point2);
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }

        final var outLines1 = transformation.transformLines(inputLines);
        final var outLines2 = new ArrayList<Line3D>();
        transformation.transformLines(inputLines, outLines2);

        // check correctness
        assertEquals(inputLines.size(), outLines1.size());
        assertEquals(inputLines.size(), outLines2.size());
        for (var i = 0; i < size; i++) {
            final var expectedLine = expectedLines.get(i);

            final var outLine1 = outLines1.get(i);
            final var outLine2 = outLines2.get(i);

            expectedLine.normalize();
            outLine1.normalize();
            outLine2.normalize();

            // check correctness
            assertEquals(expectedLine.getPlane1().getA(), outLine1.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(), outLine1.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(), outLine1.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(), outLine1.getPlane1().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane2().getA(), outLine1.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(), outLine1.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(), outLine1.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(), outLine1.getPlane2().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane1().getA(), outLine2.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(), outLine2.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(), outLine2.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(), outLine2.getPlane1().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane2().getA(), outLine2.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(), outLine2.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(), outLine2.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(), outLine2.getPlane2().getD(), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testTransformAndOverwriteLines() throws CoincidentPointsException, CoincidentPlanesException,
            AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        final var inputLines = new ArrayList<Line3D>(size);
        final var expectedLines = new ArrayList<Line3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var point1 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            final var point2 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            final var line = new Line3D(point1, point2);
            inputLines.add(line);

            final var expectedLine = new Line3D(point1, point2);
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }

        transformation.transformAndOverwriteLines(inputLines);

        // check correctness
        assertEquals(inputLines.size(), size);
        for (var i = 0; i < size; i++) {
            final var expectedLine = expectedLines.get(i);

            final var line = inputLines.get(i);

            expectedLine.normalize();
            line.normalize();

            // check correctness
            assertEquals(expectedLine.getPlane1().getA(), line.getPlane1().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getB(), line.getPlane1().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getC(), line.getPlane1().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane1().getD(), line.getPlane1().getD(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getPlane2().getA(), line.getPlane2().getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getB(), line.getPlane2().getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getC(), line.getPlane2().getC(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getPlane2().getD(), line.getPlane2().getD(), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testTransformPolygon() throws NotEnoughVerticesException, AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        final var inputPoints = new ArrayList<Point3D>(size);
        final var expectedPoints = new ArrayList<Point3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, transformation);

            expectedPoints.add(expectedPoint);
        }

        final var inputPolygon = new Polygon3D(inputPoints);
        final var expectedPolygon = new Polygon3D(expectedPoints);


        final var outPolygon1 = transformation.transformAndReturnNew(inputPolygon);
        final var outPolygon2 = new Polygon3D(inputPoints);
        transformation.transform(inputPolygon, outPolygon2);

        // check correctness
        assertEquals(outPolygon1.getVertices().size(), inputPolygon.getVertices().size());
        assertEquals(outPolygon2.getVertices().size(), inputPolygon.getVertices().size());
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedPolygon.getVertices().get(i);

            final var outPoint1 = outPolygon1.getVertices().get(i);
            final var outPoint2 = outPolygon2.getVertices().get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }

        transformation.transform(inputPolygon);

        // check correctness
        assertEquals(expectedPolygon.getVertices().size(), inputPolygon.getVertices().size());
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedPolygon.getVertices().get(i);

            final var outPoint = outPolygon1.getVertices().get(i);

            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testTransformTriangle() throws AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var size = Triangle2D.NUM_VERTICES;

        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        final var inputPoints = new ArrayList<Point3D>(size);
        final var expectedPoints = new ArrayList<Point3D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point3D.create();
            transformPoint(point, expectedPoint, transformation);

            expectedPoints.add(expectedPoint);
        }

        final var inputTriangle = new Triangle3D(inputPoints.get(0), inputPoints.get(1), inputPoints.get(2));
        final var expectedTriangle = new Triangle3D(expectedPoints.get(0), expectedPoints.get(1),
                expectedPoints.get(2));

        final var outTriangle1 = transformation.transformAndReturnNew(inputTriangle);
        final var outTriangle2 = new Triangle3D(
                new InhomogeneousPoint3D(inputPoints.get(0)),
                new InhomogeneousPoint3D(inputPoints.get(1)),
                new InhomogeneousPoint3D(inputPoints.get(2)));
        transformation.transform(inputTriangle, outTriangle2);

        // check correctness
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedTriangle.getVertices().get(i);

            final var outPoint1 = outTriangle1.getVertices().get(i);
            final var outPoint2 = outTriangle2.getVertices().get(i);

            assertTrue(outPoint1.equals(expectedPoint, ABSOLUTE_ERROR));
            assertTrue(outPoint2.equals(expectedPoint, ABSOLUTE_ERROR));
        }

        transformation.transform(inputTriangle);

        // check correctness
        for (var i = 0; i < size; i++) {
            final var expectedPoint = expectedTriangle.getVertices().get(i);

            final var outPoint = inputTriangle.getVertices().get(i);

            assertTrue(outPoint.equals(expectedPoint, ABSOLUTE_ERROR));
        }
    }

    @Test
    void testTransformCamera() throws AlgebraException {
        final var randomizer = new UniformRandomizer();

        // generate random metric 3D point
        final var metricPoint = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // generate random camera
        final var cameraMatrix = Matrix.createWithUniformRandomValues(PINHOLE_CAMERA_ROWS, PINHOLE_CAMERA_COLS,
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var camera = new PinholeCamera(cameraMatrix);

        // project metric point
        final var p1 = camera.project(metricPoint);

        // generate arbitrary transformation
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);
        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        // transform metric point and camera
        final var affinePoint = transformation.transformAndReturnNew(metricPoint);
        final var affineCamera1 = transformation.transformAndReturnNew(camera);
        final var affineCamera2 = new PinholeCamera();
        transformation.transform(camera, affineCamera2);

        transformation.transform(camera);

        // project affine point with affine camera
        final var p2 = affineCamera1.project(affinePoint);
        final var p3 = affineCamera2.project(affinePoint);
        final var p4 = camera.project(affinePoint);

        // check that all projected points p1, p2, p3, p4 are still the same
        assertTrue(p1.equals(p2, ABSOLUTE_ERROR));
        assertTrue(p1.equals(p3, ABSOLUTE_ERROR));
        assertTrue(p1.equals(p4, ABSOLUTE_ERROR));
    }

    @Test
    void testTransformCameraAndPoints() throws AlgebraException, GeometryException {
        final var randomizer = new UniformRandomizer();

        // create intrinsic parameters
        final var horizontalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
        final var skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
        final var horizontalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
        final var verticalPrincipalPoint = randomizer.nextDouble(MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

        final var intrinsic = new PinholeCameraIntrinsicParameters(horizontalFocalLength, verticalFocalLength,
                horizontalPrincipalPoint, verticalPrincipalPoint, skewness);

        // create rotation parameters
        final var alphaEuler = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final var betaEuler = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));
        final var gammaEuler = com.irurueta.geometry.Utils.convertToRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES2, MAX_ANGLE_DEGREES2));

        final var rotation = new MatrixRotation3D(alphaEuler, betaEuler, gammaEuler);

        // create camera center
        final var cameraCenter = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));

        final var camera = new PinholeCamera(intrinsic, rotation, cameraCenter);

        // normalize camera to improve accuracy
        camera.normalize();

        // create 6 random point correspondences
        final var point3D1 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final var point3D2 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final var point3D3 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final var point3D4 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final var point3D5 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));
        final var point3D6 = new InhomogeneousPoint3D(
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE),
                randomizer.nextDouble(MIN_RANDOM_POINT_VALUE, MAX_RANDOM_POINT_VALUE));

        final var point2D1 = camera.project(point3D1);
        final var point2D2 = camera.project(point3D2);
        final var point2D3 = camera.project(point3D3);
        final var point2D4 = camera.project(point3D4);
        final var point2D5 = camera.project(point3D5);
        final var point2D6 = camera.project(point3D6);

        // create transformation
        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);
        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        // transform camera and points
        final var outCamera = transformation.transformAndReturnNew(camera);
        final var outPoint3D1 = transformation.transformAndReturnNew(point3D1);
        final var outPoint3D2 = transformation.transformAndReturnNew(point3D2);
        final var outPoint3D3 = transformation.transformAndReturnNew(point3D3);
        final var outPoint3D4 = transformation.transformAndReturnNew(point3D4);
        final var outPoint3D5 = transformation.transformAndReturnNew(point3D5);
        final var outPoint3D6 = transformation.transformAndReturnNew(point3D6);

        final var outPoint2D1 = outCamera.project(outPoint3D1);
        final var outPoint2D2 = outCamera.project(outPoint3D2);
        final var outPoint2D3 = outCamera.project(outPoint3D3);
        final var outPoint2D4 = outCamera.project(outPoint3D4);
        final var outPoint2D5 = outCamera.project(outPoint3D5);
        final var outPoint2D6 = outCamera.project(outPoint3D6);

        outCamera.decompose();

        final var outCameraCenter = transformation.transformAndReturnNew(cameraCenter);
        final var outCameraCenter2 = outCamera.getCameraCenter();

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
    void testInverse() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        final var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        final var invTransformation1 = transformation.inverseAndReturnNew();
        final var invTransformation2 = new AffineTransformation3D();
        transformation.inverse(invTransformation2);

        // check that inverse transformation matrix is the inverse matrix of
        // current transformation
        assertTrue(invTransformation1.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(MetricTransformation3D.HOM_COORDS,
                MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));

        assertTrue(invTransformation2.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(MetricTransformation3D.HOM_COORDS,
                MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));

        // test transforming a random point by transformation and then by its
        // inverse to ensure it remains the same
        final var params = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var inputPoint = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, params);

        final var transfPoint = transformation.transformAndReturnNew(inputPoint);

        final var invTransfPoint1 = invTransformation1.transformAndReturnNew(transfPoint);
        final var invTransfPoint2 = invTransformation2.transformAndReturnNew(transfPoint);

        // check correctness
        assertTrue(inputPoint.equals(invTransfPoint1, ABSOLUTE_ERROR));
        assertTrue(inputPoint.equals(invTransfPoint2, ABSOLUTE_ERROR));

        // try inverting original transformation
        transformation.inverse();
        final var outPoint = transformation.transformAndReturnNew(transfPoint);

        assertTrue(inputPoint.equals(outPoint, ABSOLUTE_ERROR));
    }

    @Test
    void testToProjective() {
        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        var norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);

        final var translation = new double[MetricTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var affineParams = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        final var transformation = new AffineTransformation3D(affineParams, rotation, translation);

        final var expectedMatrix = transformation.asMatrix();
        norm = Utils.normF(expectedMatrix);
        expectedMatrix.multiplyByScalar(1.0 / norm);

        final var projectiveMatrix = transformation.toProjective().asMatrix();
        norm = Utils.normF(projectiveMatrix);
        projectiveMatrix.multiplyByScalar(1.0 / norm);

        // check equal-ness up to scale
        assertTrue(expectedMatrix.equals(projectiveMatrix, ABSOLUTE_ERROR));
    }

    @Test
    void testCombine() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var translation1 = new double[AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX1 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleY1 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleZ1 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var skewnessXY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var theta1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis1 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        var norm = Utils.normF(rotAxis1);
        ArrayUtils.multiplyByScalar(rotAxis1, 1.0 / norm, rotAxis1);

        final var translation2 = new double[AffineTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var scaleX2 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleY2 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleZ2 = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var skewnessXY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var theta2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis2 = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        norm = Utils.normF(rotAxis2);
        ArrayUtils.multiplyByScalar(rotAxis2, 1.0 / norm, rotAxis2);

        final var rotation1 = Rotation3D.create(rotAxis1, theta1);
        final var affineParams1 = new AffineParameters3D(scaleX1, scaleY1, scaleZ1, skewnessXY1, skewnessXZ1,
                skewnessYZ1);

        final var transformation1 = new AffineTransformation3D(affineParams1, rotation1, translation1);

        final var rotation2 = Rotation3D.create(rotAxis2, theta2);
        final var affineParams2 = new AffineParameters3D(scaleX2, scaleY2, scaleZ2, skewnessXY2, skewnessXZ2,
                skewnessYZ2);

        final var transformation2 = new AffineTransformation3D(affineParams2, rotation2, translation2);

        final var expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(transformation2.asMatrix());
        final var expectedA = transformation1.getA().multiplyAndReturnNew(transformation2.getA());

        final var a1 = new Matrix(transformation1.getA());
        final var t2 = Matrix.newFromArray(translation2, true);
        a1.multiply(t2);
        final var expectedTranslation = a1.toArray();
        ArrayUtils.sum(expectedTranslation, translation1, expectedTranslation);

        // combine and return result as a new transformation
        final var transformation3 = transformation1.combineAndReturnNew(transformation2);
        // combine into transformation1
        transformation1.combine(transformation2);

        // both matrices m1 and m3 need to be equal
        final var m3 = transformation3.asMatrix();
        final var m1 = transformation1.asMatrix();

        // check correctness
        assertTrue(m1.equals(m3, ABSOLUTE_ERROR));

        // besides, resulting transformation matrices need to be equal to
        // expected matrix
        assertTrue(m1.equals(expectedMatrix, ABSOLUTE_ERROR));
        assertTrue(m3.equals(expectedMatrix, ABSOLUTE_ERROR));

        // check correctness of A and translation
        assertTrue(transformation1.getA().equals(expectedA, ABSOLUTE_ERROR));
        assertTrue(transformation3.getA().equals(expectedA, ABSOLUTE_ERROR));

        assertArrayEquals(expectedTranslation, transformation1.getTranslation(), ABSOLUTE_ERROR);
        assertArrayEquals(expectedTranslation, transformation3.getTranslation(), ABSOLUTE_ERROR);
    }

    @Test
    void testSetTransformationFromPoints() throws WrongSizeException, DecomposerException, CoincidentPointsException {
        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(AffineTransformation3D.INHOM_COORDS,
                    AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
            final var norm = Utils.normF(a);
            // normalize T to increase accuracy
            a.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

        final var translation = new double[AffineTransformation3D.INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(translation, -1.0, 1.0);

        final var transformation1 = new AffineTransformation3D(a, translation);

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
        final var m = new Matrix(12, 13);
        do {
            final var coordsMatrix = Matrix.createWithUniformRandomValues(4, 3, -1.0,
                    1.0);

            var coords = coordsMatrix.getSubmatrixAsArray(0, 0, 0,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint1 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(1, 0, 1,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint2 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(2, 0, 2,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint3 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(3, 0, 3,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint4 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            // Transform points using transformation
            outputPoint1 = transformation1.transformAndReturnNew(inputPoint1);
            outputPoint2 = transformation1.transformAndReturnNew(inputPoint2);
            outputPoint3 = transformation1.transformAndReturnNew(inputPoint3);
            outputPoint4 = transformation1.transformAndReturnNew(inputPoint4);

            // 1st pair of points
            var iX = inputPoint1.getHomX();
            var iY = inputPoint1.getHomY();
            var iZ = inputPoint1.getHomZ();
            var iW = inputPoint1.getHomW();

            var oX = outputPoint1.getHomX();
            var oY = outputPoint1.getHomY();
            var oZ = outputPoint1.getHomZ();
            var oW = outputPoint1.getHomW();

            var oWiX = oW * iX;
            var oWiY = oW * iY;
            var oWiZ = oW * iZ;
            var oWiW = oW * iW;

            var oXiW = oX * iW;
            var oYiW = oY * iW;
            var oZiW = oZ * iW;

            var norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(0, 0, oWiX / norm);
            m.setElementAt(0, 1, oWiY / norm);
            m.setElementAt(0, 2, oWiZ / norm);
            m.setElementAt(0, 9, oWiW / norm);
            m.setElementAt(0, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(1, 3, oWiX / norm);
            m.setElementAt(1, 4, oWiY / norm);
            m.setElementAt(1, 5, oWiZ / norm);
            m.setElementAt(1, 10, oWiW / norm);
            m.setElementAt(1, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiW * oZiW);

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

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(3, 0, oWiX / norm);
            m.setElementAt(3, 1, oWiY / norm);
            m.setElementAt(3, 2, oWiZ / norm);
            m.setElementAt(3, 9, oWiW / norm);
            m.setElementAt(3, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(4, 3, oWiX / norm);
            m.setElementAt(4, 4, oWiY / norm);
            m.setElementAt(4, 5, oWiZ / norm);
            m.setElementAt(4, 10, oWiW / norm);
            m.setElementAt(4, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiW * oZiW);

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

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(6, 0, oWiX / norm);
            m.setElementAt(6, 1, oWiY / norm);
            m.setElementAt(6, 2, oWiZ / norm);
            m.setElementAt(6, 9, oWiW / norm);
            m.setElementAt(6, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(7, 3, oWiX / norm);
            m.setElementAt(7, 4, oWiY / norm);
            m.setElementAt(7, 5, oWiZ / norm);
            m.setElementAt(7, 10, oWiW / norm);
            m.setElementAt(7, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiW * oZiW);

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

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(9, 0, oWiX / norm);
            m.setElementAt(9, 1, oWiY / norm);
            m.setElementAt(9, 2, oWiZ / norm);
            m.setElementAt(9, 9, oWiW / norm);
            m.setElementAt(9, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(10, 3, oWiX / norm);
            m.setElementAt(10, 4, oWiY / norm);
            m.setElementAt(10, 5, oWiZ / norm);
            m.setElementAt(10, 10, oWiW / norm);
            m.setElementAt(10, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiW * oZiW);

            m.setElementAt(11, 6, oWiX / norm);
            m.setElementAt(11, 7, oWiY / norm);
            m.setElementAt(11, 8, oWiZ / norm);
            m.setElementAt(11, 11, oWiW / norm);
            m.setElementAt(11, 12, -oZiW / norm);
        } while (Utils.rank(m) < 12);

        // Now build another transformation
        final var transformation2 = new AffineTransformation3D();

        // estimate transformation from corresponding points
        transformation2.setTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3, inputPoint4, outputPoint1,
                outputPoint2, outputPoint3, outputPoint4);

        // check correctness of transformation by checking transformed points
        assertTrue(outputPoint1.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint1)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint2.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint2)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint3.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint3)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint4.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint4)),
                ABSOLUTE_ERROR));

        // Force CoincidentPointsException
        final var finalInputPoint1 = inputPoint1;
        final var finalInputPoint3 = inputPoint3;
        final var finalInputPoint4 = inputPoint4;
        final var finalOutputPoint1 = outputPoint1;
        final var finalOutputPoint3 = outputPoint3;
        final var finalOutputPoint4 = outputPoint4;
        assertThrows(CoincidentPointsException.class, () -> transformation2.setTransformationFromPoints(
                finalInputPoint1, finalInputPoint1, finalInputPoint3, finalInputPoint4, finalOutputPoint1,
                finalOutputPoint1, finalOutputPoint3, finalOutputPoint4));
    }

    @Test
    void testConstructorFromPoints() throws WrongSizeException, DecomposerException, CoincidentPointsException {
        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(AffineTransformation3D.INHOM_COORDS,
                    AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
            final var norm = Utils.normF(a);
            // normalize T to increase accuracy
            a.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

        final var translation = new double[AffineTransformation3D.INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(translation, -1.0, 1.0);

        final var transformation1 = new AffineTransformation3D(a, translation);

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
        final var m = new Matrix(12, 13);
        do {
            final var coordsMatrix = Matrix.createWithUniformRandomValues(4, 3,
                    -1.0, 1.0);

            var coords = coordsMatrix.getSubmatrixAsArray(0, 0, 0,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint1 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(1, 0, 1,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint2 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(2, 0, 2,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint3 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            coords = coordsMatrix.getSubmatrixAsArray(3, 0, 3,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint4 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            // Transform points using transformation
            outputPoint1 = transformation1.transformAndReturnNew(inputPoint1);
            outputPoint2 = transformation1.transformAndReturnNew(inputPoint2);
            outputPoint3 = transformation1.transformAndReturnNew(inputPoint3);
            outputPoint4 = transformation1.transformAndReturnNew(inputPoint4);

            // 1st pair of points
            var iX = inputPoint1.getHomX();
            var iY = inputPoint1.getHomY();
            var iZ = inputPoint1.getHomZ();
            var iW = inputPoint1.getHomW();

            var oX = outputPoint1.getHomX();
            var oY = outputPoint1.getHomY();
            var oZ = outputPoint1.getHomZ();
            var oW = outputPoint1.getHomW();

            var oWiX = oW * iX;
            var oWiY = oW * iY;
            var oWiZ = oW * iZ;
            var oWiW = oW * iW;

            var oXiW = oX * iW;
            var oYiW = oY * iW;
            var oZiW = oZ * iW;

            var norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(0, 0, oWiX / norm);
            m.setElementAt(0, 1, oWiY / norm);
            m.setElementAt(0, 2, oWiZ / norm);
            m.setElementAt(0, 9, oWiW / norm);
            m.setElementAt(0, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(1, 3, oWiX / norm);
            m.setElementAt(1, 4, oWiY / norm);
            m.setElementAt(1, 5, oWiZ / norm);
            m.setElementAt(1, 10, oWiW / norm);
            m.setElementAt(1, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiW * oZiW);

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

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(3, 0, oWiX / norm);
            m.setElementAt(3, 1, oWiY / norm);
            m.setElementAt(3, 2, oWiZ / norm);
            m.setElementAt(3, 9, oWiW / norm);
            m.setElementAt(3, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(4, 3, oWiX / norm);
            m.setElementAt(4, 4, oWiY / norm);
            m.setElementAt(4, 5, oWiZ / norm);
            m.setElementAt(4, 10, oWiW / norm);
            m.setElementAt(4, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiW * oZiW);

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

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(6, 0, oWiX / norm);
            m.setElementAt(6, 1, oWiY / norm);
            m.setElementAt(6, 2, oWiZ / norm);
            m.setElementAt(6, 9, oWiW / norm);
            m.setElementAt(6, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(7, 3, oWiX / norm);
            m.setElementAt(7, 4, oWiY / norm);
            m.setElementAt(7, 5, oWiZ / norm);
            m.setElementAt(7, 10, oWiW / norm);
            m.setElementAt(7, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiW * oZiW);

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

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiW * oXiW);

            m.setElementAt(9, 0, oWiX / norm);
            m.setElementAt(9, 1, oWiY / norm);
            m.setElementAt(9, 2, oWiZ / norm);
            m.setElementAt(9, 9, oWiW / norm);
            m.setElementAt(9, 12, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiW * oYiW);

            m.setElementAt(10, 3, oWiX / norm);
            m.setElementAt(10, 4, oWiY / norm);
            m.setElementAt(10, 5, oWiZ / norm);
            m.setElementAt(10, 10, oWiW / norm);
            m.setElementAt(10, 12, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiW * oZiW);

            m.setElementAt(11, 6, oWiX / norm);
            m.setElementAt(11, 7, oWiY / norm);
            m.setElementAt(11, 8, oWiZ / norm);
            m.setElementAt(11, 11, oWiW / norm);
            m.setElementAt(11, 12, -oZiW / norm);
        } while (Utils.rank(m) < 12);

        // Now build another transformation
        var transformation2 = new AffineTransformation3D(inputPoint1, inputPoint2, inputPoint3, inputPoint4,
                outputPoint1, outputPoint2, outputPoint3, outputPoint4);

        // check correctness of transformation by checking transformed points
        assertTrue(outputPoint1.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint1)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint2.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint2)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint3.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint3)),
                ABSOLUTE_ERROR));
        assertTrue(outputPoint4.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint4)),
                ABSOLUTE_ERROR));

        // Force CoincidentPointsException
        final var finalInputPoint1 = inputPoint1;
        final var finalInputPoint3 = inputPoint3;
        final var finalInputPoint4 = inputPoint4;
        final var finalOutputPoint1 = outputPoint1;
        final var finalOutputPoint3 = outputPoint3;
        final var finalOutputPoint4 = outputPoint4;
        assertThrows(CoincidentPointsException.class, () -> new AffineTransformation3D(finalInputPoint1,
                finalInputPoint1, finalInputPoint3, finalInputPoint4, finalOutputPoint1, finalOutputPoint1,
                finalOutputPoint3, finalOutputPoint4));
    }

    @Test
    void testSetTransformationFromPlanes() throws CoincidentPlanesException, AlgebraException {
        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(AffineTransformation3D.INHOM_COORDS,
                    AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
            final var norm = Utils.normF(a);
            // normalize T to increase accuracy
            a.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

        final var translation = new double[AffineTransformation3D.INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(translation, -1.0, 1.0);

        final var transformation1 = new AffineTransformation3D(a, translation);

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
        final var m = new Matrix(12, 13);
        do {
            final var paramsMatrix = Matrix.createWithUniformRandomValues(4, 4, -1.0,
                    1.0);

            var params = paramsMatrix.getSubmatrixAsArray(0, 0, 0,
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
            var iA = inputPlane1.getA();
            var iB = inputPlane1.getB();
            var iC = inputPlane1.getC();
            var iD = inputPlane1.getD();

            var oA = outputPlane1.getA();
            var oB = outputPlane1.getB();
            var oC = outputPlane1.getC();
            var oD = outputPlane1.getD();

            var oDiA = oD * iA;
            var oDiB = oD * iB;
            var oDiC = oD * iC;
            var oDiD = oD * iD;

            var oAiD = oA * iD;
            var oBiD = oB * iD;
            var oCiD = oC * iD;

            var norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(0, 0, oDiA / norm);
            m.setElementAt(0, 1, oDiB / norm);
            m.setElementAt(0, 2, oDiC / norm);
            m.setElementAt(0, 9, oDiD / norm);
            m.setElementAt(0, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(1, 3, oDiA / norm);
            m.setElementAt(1, 4, oDiB / norm);
            m.setElementAt(1, 5, oDiC / norm);
            m.setElementAt(1, 10, oDiD / norm);
            m.setElementAt(1, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(3, 0, oDiA / norm);
            m.setElementAt(3, 1, oDiB / norm);
            m.setElementAt(3, 2, oDiC / norm);
            m.setElementAt(3, 9, oDiD / norm);
            m.setElementAt(3, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(4, 3, oDiA / norm);
            m.setElementAt(4, 4, oDiB / norm);
            m.setElementAt(4, 5, oDiC / norm);
            m.setElementAt(4, 10, oDiD / norm);
            m.setElementAt(4, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(6, 0, oDiA / norm);
            m.setElementAt(6, 1, oDiB / norm);
            m.setElementAt(6, 2, oDiC / norm);
            m.setElementAt(6, 9, oDiD / norm);
            m.setElementAt(6, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(7, 3, oDiA / norm);
            m.setElementAt(7, 4, oDiB / norm);
            m.setElementAt(7, 5, oDiC / norm);
            m.setElementAt(7, 10, oDiD / norm);
            m.setElementAt(7, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(9, 0, oDiA / norm);
            m.setElementAt(9, 1, oDiB / norm);
            m.setElementAt(9, 2, oDiC / norm);
            m.setElementAt(9, 9, oDiD / norm);
            m.setElementAt(9, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(10, 3, oDiA / norm);
            m.setElementAt(10, 4, oDiB / norm);
            m.setElementAt(10, 5, oDiC / norm);
            m.setElementAt(10, 10, oDiD / norm);
            m.setElementAt(10, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

            m.setElementAt(11, 6, oDiA / norm);
            m.setElementAt(11, 7, oDiB / norm);
            m.setElementAt(11, 8, oDiC / norm);
            m.setElementAt(11, 11, oDiD / norm);
            m.setElementAt(11, 12, -oCiD / norm);
        } while (Utils.rank(m) < 12);

        // Now build another transformation
        final var transformation2 = new AffineTransformation3D();

        // estimate transformation from corresponding planes
        transformation2.setTransformationFromPlanes(inputPlane1, inputPlane2, inputPlane3, inputPlane4, outputPlane1,
                outputPlane2, outputPlane3, outputPlane4);

        // check correctness of transformation by checking transformed planes
        var p = transformation2.transformAndReturnNew(inputPlane1);
        p.normalize();

        assertEquals(Math.abs(outputPlane1.getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane2);
        p.normalize();

        assertEquals(Math.abs(outputPlane2.getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane3);
        p.normalize();

        assertEquals(Math.abs(outputPlane3.getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane4);
        p.normalize();

        assertEquals(Math.abs(outputPlane4.getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

        // Force CoincidentPlanesException
        final var finalInputPlane1 = inputPlane1;
        final var finalInputPlane3 = inputPlane3;
        final var finalInputPlane4 = inputPlane4;
        final var finalOutputPlane1 = outputPlane1;
        final var finalOutputPlane3 = outputPlane3;
        final var finalOutputPlane4 = outputPlane4;
        assertThrows(CoincidentPlanesException.class, () -> transformation2.setTransformationFromPlanes(
                finalInputPlane1, finalInputPlane1, finalInputPlane3, finalInputPlane4, finalOutputPlane1,
                finalOutputPlane1, finalOutputPlane3, finalOutputPlane4));
    }

    @Test
    void testConstructorFromPlanes() throws CoincidentPlanesException, AlgebraException {
        Matrix a;
        do {
            // ensure A matrix is invertible
            a = Matrix.createWithUniformRandomValues(AffineTransformation3D.INHOM_COORDS,
                    AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
            final var norm = Utils.normF(a);
            // normalize T to increase accuracy
            a.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

        final var translation = new double[AffineTransformation3D.INHOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(translation, -1.0, 1.0);

        final var transformation1 = new AffineTransformation3D(a, translation);

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
        final var m = new Matrix(12, 13);
        do {
            final var paramsMatrix = Matrix.createWithUniformRandomValues(4, 4, -1.0,
                    1.0);

            var params = paramsMatrix.getSubmatrixAsArray(0, 0, 0,
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
            var iA = inputPlane1.getA();
            var iB = inputPlane1.getB();
            var iC = inputPlane1.getC();
            var iD = inputPlane1.getD();

            var oA = outputPlane1.getA();
            var oB = outputPlane1.getB();
            var oC = outputPlane1.getC();
            var oD = outputPlane1.getD();

            var oDiA = oD * iA;
            var oDiB = oD * iB;
            var oDiC = oD * iC;
            var oDiD = oD * iD;

            var oAiD = oA * iD;
            var oBiD = oB * iD;
            var oCiD = oC * iD;

            var norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(0, 0, oDiA / norm);
            m.setElementAt(0, 1, oDiB / norm);
            m.setElementAt(0, 2, oDiC / norm);
            m.setElementAt(0, 9, oDiD / norm);
            m.setElementAt(0, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(1, 3, oDiA / norm);
            m.setElementAt(1, 4, oDiB / norm);
            m.setElementAt(1, 5, oDiC / norm);
            m.setElementAt(1, 10, oDiD / norm);
            m.setElementAt(1, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(3, 0, oDiA / norm);
            m.setElementAt(3, 1, oDiB / norm);
            m.setElementAt(3, 2, oDiC / norm);
            m.setElementAt(3, 9, oDiD / norm);
            m.setElementAt(3, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(4, 3, oDiA / norm);
            m.setElementAt(4, 4, oDiB / norm);
            m.setElementAt(4, 5, oDiC / norm);
            m.setElementAt(4, 10, oDiD / norm);
            m.setElementAt(4, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(6, 0, oDiA / norm);
            m.setElementAt(6, 1, oDiB / norm);
            m.setElementAt(6, 2, oDiC / norm);
            m.setElementAt(6, 9, oDiD / norm);
            m.setElementAt(6, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(7, 3, oDiA / norm);
            m.setElementAt(7, 4, oDiB / norm);
            m.setElementAt(7, 5, oDiC / norm);
            m.setElementAt(7, 10, oDiD / norm);
            m.setElementAt(7, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

            m.setElementAt(9, 0, oDiA / norm);
            m.setElementAt(9, 1, oDiB / norm);
            m.setElementAt(9, 2, oDiC / norm);
            m.setElementAt(9, 9, oDiD / norm);
            m.setElementAt(9, 12, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

            m.setElementAt(10, 3, oDiA / norm);
            m.setElementAt(10, 4, oDiB / norm);
            m.setElementAt(10, 5, oDiC / norm);
            m.setElementAt(10, 10, oDiD / norm);
            m.setElementAt(10, 12, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

            m.setElementAt(11, 6, oDiA / norm);
            m.setElementAt(11, 7, oDiB / norm);
            m.setElementAt(11, 8, oDiC / norm);
            m.setElementAt(11, 11, oDiD / norm);
            m.setElementAt(11, 12, -oCiD / norm);
        } while (Utils.rank(m) < 12);

        // Now build another transformation
        var transformation2 = new AffineTransformation3D(inputPlane1, inputPlane2, inputPlane3, inputPlane4,
                outputPlane1, outputPlane2, outputPlane3, outputPlane4);

        // check correctness of transformation by checking transformed planes
        var p = transformation2.transformAndReturnNew(inputPlane1);
        p.normalize();

        assertEquals(Math.abs(outputPlane1.getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane2);
        p.normalize();

        assertEquals(Math.abs(outputPlane2.getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane3);
        p.normalize();

        assertEquals(Math.abs(outputPlane3.getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane4);
        p.normalize();

        assertEquals(Math.abs(outputPlane4.getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

        // Force CoincidentPlanesException
        final var finalInputPlane1 = inputPlane1;
        final var finalInputPlane3 = inputPlane3;
        final var finalInputPlane4 = inputPlane4;
        final var finalOutputPlane1 = outputPlane1;
        final var finalOutputPlane3 = outputPlane3;
        final var finalOutputPlane4 = outputPlane4;
        assertThrows(CoincidentPlanesException.class, () -> new AffineTransformation3D(finalInputPlane1,
                finalInputPlane1, finalInputPlane3, finalInputPlane4, finalOutputPlane1, finalOutputPlane1,
                finalOutputPlane3, finalOutputPlane4));
    }

    @Test
    void testSetTransformationFromLines() throws CoincidentLinesException, AlgebraException, CoincidentPlanesException {
        for (var t = 0; t < 1; t++) {
            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(AffineTransformation3D.INHOM_COORDS,
                        AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
                final var norm = Utils.normF(a);
                // normalize T to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

            final var translation = new double[AffineTransformation3D.INHOM_COORDS];
            final var randomizer = new UniformRandomizer();
            randomizer.fill(translation, -1.0, 1.0);

            final var transformation1 = new AffineTransformation3D(a, translation);

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
            final var m = new Matrix(12, 13);
            do {
                final var paramsMatrix = Matrix.createWithUniformRandomValues(6, 4, -1.0,
                        1.0);

                var params = paramsMatrix.getSubmatrixAsArray(0, 0, 0,
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
                var iA = inputLine1.getPlane1().getA();
                var iB = inputLine1.getPlane1().getB();
                var iC = inputLine1.getPlane1().getC();
                var iD = inputLine1.getPlane1().getD();

                var oA = outputLine1.getPlane1().getA();
                var oB = outputLine1.getPlane1().getB();
                var oC = outputLine1.getPlane1().getC();
                var oD = outputLine1.getPlane1().getD();

                var oDiA = oD * iA;
                var oDiB = oD * iB;
                var oDiC = oD * iC;
                var oDiD = oD * iD;

                var oAiD = oA * iD;
                var oBiD = oB * iD;
                var oCiD = oC * iD;

                var norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(0, 0, oDiA / norm);
                m.setElementAt(0, 1, oDiB / norm);
                m.setElementAt(0, 2, oDiC / norm);
                m.setElementAt(0, 9, oDiD / norm);
                m.setElementAt(0, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(1, 3, oDiA / norm);
                m.setElementAt(1, 4, oDiB / norm);
                m.setElementAt(1, 5, oDiC / norm);
                m.setElementAt(1, 10, oDiD / norm);
                m.setElementAt(1, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(3, 0, oDiA / norm);
                m.setElementAt(3, 1, oDiB / norm);
                m.setElementAt(3, 2, oDiC / norm);
                m.setElementAt(3, 9, oDiD / norm);
                m.setElementAt(3, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(4, 3, oDiA / norm);
                m.setElementAt(4, 4, oDiB / norm);
                m.setElementAt(4, 5, oDiC / norm);
                m.setElementAt(4, 10, oDiD / norm);
                m.setElementAt(4, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(6, 0, oDiA / norm);
                m.setElementAt(6, 1, oDiB / norm);
                m.setElementAt(6, 2, oDiC / norm);
                m.setElementAt(6, 9, oDiD / norm);
                m.setElementAt(6, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(7, 3, oDiA / norm);
                m.setElementAt(7, 4, oDiB / norm);
                m.setElementAt(7, 5, oDiC / norm);
                m.setElementAt(7, 10, oDiD / norm);
                m.setElementAt(7, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(9, 0, oDiA / norm);
                m.setElementAt(9, 1, oDiB / norm);
                m.setElementAt(9, 2, oDiC / norm);
                m.setElementAt(9, 9, oDiD / norm);
                m.setElementAt(9, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(10, 3, oDiA / norm);
                m.setElementAt(10, 4, oDiB / norm);
                m.setElementAt(10, 5, oDiC / norm);
                m.setElementAt(10, 10, oDiD / norm);
                m.setElementAt(10, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

                m.setElementAt(11, 6, oDiA / norm);
                m.setElementAt(11, 7, oDiB / norm);
                m.setElementAt(11, 8, oDiC / norm);
                m.setElementAt(11, 11, oDiD / norm);
                m.setElementAt(11, 12, -oCiD / norm);

            } while (Utils.rank(m) < 8);

            // Now build another transformation
            final var transformation2 = new AffineTransformation3D();

            // estimate transformation from corresponding lines
            transformation2.setTransformationFromLines(inputLine1, inputLine2, outputLine1, outputLine2);

            // check correctness of lines (up to scale)
            var p = transformation2.transformAndReturnNew(inputLine1.getPlane1());
            p.normalize();

            assertEquals(Math.abs(outputLine1.getPlane1().getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine1.getPlane2());
            p.normalize();

            assertEquals(Math.abs(outputLine1.getPlane2().getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine2.getPlane1());
            p.normalize();

            assertEquals(Math.abs(outputLine2.getPlane1().getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine2.getPlane2());
            p.normalize();

            assertEquals(Math.abs(outputLine2.getPlane2().getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

            // Force CoincidentLinesException
            final var finalInputLine1 = inputLine1;
            final var finalOutputLine1 = outputLine1;
            assertThrows(CoincidentLinesException.class, () -> transformation2.setTransformationFromLines(
                    finalInputLine1, finalInputLine1, finalOutputLine1, finalOutputLine1));
        }
    }

    @Test
    void testConstructorFromLines() throws CoincidentLinesException, AlgebraException, CoincidentPlanesException {
        for (var t = 0; t < 1; t++) {

            Matrix a;
            do {
                // ensure A matrix is invertible
                a = Matrix.createWithUniformRandomValues(AffineTransformation3D.INHOM_COORDS,
                        AffineTransformation3D.INHOM_COORDS, -1.0, 1.0);
                final var norm = Utils.normF(a);
                // normalize A to increase accuracy
                a.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(a) < AffineTransformation3D.INHOM_COORDS);

            final var translation = new double[AffineTransformation3D.INHOM_COORDS];
            final var randomizer = new UniformRandomizer();
            randomizer.fill(translation, -1.0, 1.0);

            final var transformation1 = new AffineTransformation3D(a, translation);

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
            final var m = new Matrix(12, 13);
            do {
                final var paramsMatrix = Matrix.createWithUniformRandomValues(6, 4, -1.0,
                        1.0);

                var params = paramsMatrix.getSubmatrixAsArray(0, 0, 0,
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
                var iA = inputLine1.getPlane1().getA();
                var iB = inputLine1.getPlane1().getB();
                var iC = inputLine1.getPlane1().getC();
                var iD = inputLine1.getPlane1().getD();

                var oA = outputLine1.getPlane1().getA();
                var oB = outputLine1.getPlane1().getB();
                var oC = outputLine1.getPlane1().getC();
                var oD = outputLine1.getPlane1().getD();

                var oDiA = oD * iA;
                var oDiB = oD * iB;
                var oDiC = oD * iC;
                var oDiD = oD * iD;

                var oAiD = oA * iD;
                var oBiD = oB * iD;
                var oCiD = oC * iD;

                var norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(0, 0, oDiA / norm);
                m.setElementAt(0, 1, oDiB / norm);
                m.setElementAt(0, 2, oDiC / norm);
                m.setElementAt(0, 9, oDiD / norm);
                m.setElementAt(0, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(1, 3, oDiA / norm);
                m.setElementAt(1, 4, oDiB / norm);
                m.setElementAt(1, 5, oDiC / norm);
                m.setElementAt(1, 10, oDiD / norm);
                m.setElementAt(1, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(3, 0, oDiA / norm);
                m.setElementAt(3, 1, oDiB / norm);
                m.setElementAt(3, 2, oDiC / norm);
                m.setElementAt(3, 9, oDiD / norm);
                m.setElementAt(3, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(4, 3, oDiA / norm);
                m.setElementAt(4, 4, oDiB / norm);
                m.setElementAt(4, 5, oDiC / norm);
                m.setElementAt(4, 10, oDiD / norm);
                m.setElementAt(4, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(6, 0, oDiA / norm);
                m.setElementAt(6, 1, oDiB / norm);
                m.setElementAt(6, 2, oDiC / norm);
                m.setElementAt(6, 9, oDiD / norm);
                m.setElementAt(6, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(7, 3, oDiA / norm);
                m.setElementAt(7, 4, oDiB / norm);
                m.setElementAt(7, 5, oDiC / norm);
                m.setElementAt(7, 10, oDiD / norm);
                m.setElementAt(7, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

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

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiD * oAiD);

                m.setElementAt(9, 0, oDiA / norm);
                m.setElementAt(9, 1, oDiB / norm);
                m.setElementAt(9, 2, oDiC / norm);
                m.setElementAt(9, 9, oDiD / norm);
                m.setElementAt(9, 12, -oAiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiD * oBiD);

                m.setElementAt(10, 3, oDiA / norm);
                m.setElementAt(10, 4, oDiB / norm);
                m.setElementAt(10, 5, oDiC / norm);
                m.setElementAt(10, 10, oDiD / norm);
                m.setElementAt(10, 12, -oBiD / norm);

                norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiD * oCiD);

                m.setElementAt(11, 6, oDiA / norm);
                m.setElementAt(11, 7, oDiB / norm);
                m.setElementAt(11, 8, oDiC / norm);
                m.setElementAt(11, 11, oDiD / norm);
                m.setElementAt(11, 12, -oCiD / norm);

            } while (Utils.rank(m) < 8);

            // Now build another transformation
            var transformation2 = new AffineTransformation3D(inputLine1, inputLine2, outputLine1, outputLine2);

            // check correctness of lines (up to scale)
            var p = transformation2.transformAndReturnNew(inputLine1.getPlane1());
            p.normalize();

            assertEquals(Math.abs(outputLine1.getPlane1().getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane1().getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine1.getPlane2());
            p.normalize();

            assertEquals(Math.abs(outputLine1.getPlane2().getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine1.getPlane2().getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine2.getPlane1());
            p.normalize();

            assertEquals(Math.abs(outputLine2.getPlane1().getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane1().getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

            p = transformation2.transformAndReturnNew(inputLine2.getPlane2());
            p.normalize();

            assertEquals(Math.abs(outputLine2.getPlane2().getA()), Math.abs(p.getA()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getB()), Math.abs(p.getB()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getC()), Math.abs(p.getC()), ABSOLUTE_ERROR);
            assertEquals(Math.abs(outputLine2.getPlane2().getD()), Math.abs(p.getD()), ABSOLUTE_ERROR);

            // Force CoincidentLinesException
            final var finalInputLine1 = inputLine1;
            final var finalOutputLine1 = outputLine1;
            assertThrows(CoincidentLinesException.class, () -> new AffineTransformation3D(finalInputLine1,
                    finalInputLine1, finalOutputLine1, finalOutputLine1));
        }
    }

    @Test
    void testSerializeDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var randomizer = new UniformRandomizer();

        final var a = Matrix.createWithUniformRandomValues(AffineTransformation3D.INHOM_COORDS,
                AffineTransformation3D.INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var translation = new double[AffineParameters3D.INHOM_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation1 = new AffineTransformation3D(a, translation);

        assertSame(a, transformation1.getA());
        assertArrayEquals(translation, transformation1.getTranslation(), 0.0);

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(transformation1);
        final var transformation2 = SerializationHelper.<AffineTransformation3D>deserialize(bytes);

        // check
        assertEquals(transformation1.getA(), transformation2.getA());
        assertArrayEquals(transformation1.getTranslation(), transformation2.getTranslation(), 0.0);
    }

    private static void transformPoint(
            final Point3D inputPoint, final Point3D outputPoint, final AffineTransformation3D transformation)
            throws AlgebraException {
        inputPoint.normalize();

        final var a = new Matrix(transformation.getA());

        final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        coords[0] = inputPoint.getInhomX();
        coords[1] = inputPoint.getInhomY();
        coords[2] = inputPoint.getInhomZ();

        final var p = Matrix.newFromArray(coords, true);

        a.multiply(p);

        final var translation = transformation.getTranslation();

        outputPoint.setInhomogeneousCoordinates(
                a.getElementAtIndex(0) + translation[0],
                a.getElementAtIndex(1) + translation[1],
                a.getElementAtIndex(2) + translation[2]);
    }

    private static void transformPlane(
            final Plane inputPlane, final Plane outputPlane, final AffineTransformation3D transformation)
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException {
        inputPlane.normalize();
        final var t = transformation.asMatrix();
        var norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final var invT = Utils.inverse(t);
        norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final var transInvT = invT.transposeAndReturnNew();
        final var p = Matrix.newFromArray(inputPlane.asArray(), true);

        outputPlane.setParameters(transInvT.multiplyAndReturnNew(p).toArray());
    }

    private static void transformQuadric(
            final Quadric inputQuadric, final Quadric outputQuadric, final AffineTransformation3D transformation)
            throws AlgebraException, NonSymmetricMatrixException {

        final var t = transformation.asMatrix();
        final var invT = Utils.inverse(t);
        var norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final var transInvT = invT.transposeAndReturnNew();

        inputQuadric.normalize();
        final var q = inputQuadric.asMatrix();

        final var transQ = transInvT.multiplyAndReturnNew(q.multiplyAndReturnNew(invT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transQ);
        transQ.multiplyByScalar(1.0 / norm);

        outputQuadric.setParameters(transQ);
    }

    private static void transformDualQuadric(
            final DualQuadric inputDualQuadric, final DualQuadric outputDualQuadric,
            final AffineTransformation3D transformation) throws WrongSizeException, NonSymmetricMatrixException {

        final var t = transformation.asMatrix();
        var norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final var transT = t.transposeAndReturnNew();

        inputDualQuadric.normalize();
        final var dualQ = inputDualQuadric.asMatrix();

        final var transDualQ = t.multiplyAndReturnNew(dualQ.multiplyAndReturnNew(transT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transDualQ);
        transDualQ.multiplyByScalar(1.0 / norm);

        outputDualQuadric.setParameters(transDualQ);
    }

    private static void transformLine(
            final Line3D inputLine, final Line3D outputLine, final AffineTransformation3D transformation)
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException, CoincidentPlanesException {

        inputLine.normalize();
        final var inputPlane1 = inputLine.getPlane1();
        final var inputPlane2 = inputLine.getPlane2();

        final var outputPlane1 = new Plane();
        final var outputPlane2 = new Plane();

        transformPlane(inputPlane1, outputPlane1, transformation);
        transformPlane(inputPlane2, outputPlane2, transformation);

        outputLine.setPlanes(outputPlane1, outputPlane2);
    }
}
