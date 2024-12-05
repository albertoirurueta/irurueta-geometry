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
import java.util.ArrayList;
import java.util.Arrays;

import static org.junit.jupiter.api.Assertions.*;

class ProjectiveTransformation3DTest {

    private static final int PINHOLE_CAMERA_ROWS = 3;
    private static final int PINHOLE_CAMERA_COLS = 4;

    private static final int HOM_COORDS = 4;

    private static final double MIN_ANGLE_DEGREES = -90.0;
    private static final double MAX_ANGLE_DEGREES = 90.0;

    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;

    private static final double MIN_SCALE = 0.2;
    private static final double MAX_SCALE = 5.0;

    private static final int MIN_POINTS = 3;
    private static final int MAX_POINTS = 50;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

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

    private static final int TIMES = 10;

    @Test
    void testConstants() {
        assertEquals(3, ProjectiveTransformation3D.NUM_TRANSLATION_COORDS);
        assertEquals(4, ProjectiveTransformation3D.NUM_PROJECTIVE_PARAMS);
        assertEquals(3, ProjectiveTransformation3D.INHOM_COORDS);
        assertEquals(4, ProjectiveTransformation3D.HOM_COORDS);
        assertEquals(1e-12, ProjectiveTransformation3D.EPS, 0.0);
    }

    @Test
    void testConstructor() throws AlgebraException, RotationException {

        // Test empty constructor
        var transformation = new ProjectiveTransformation3D();

        // check correctness

        // matrix is identity up to scale
        assertTrue(transformation.asMatrix().equals(Matrix.identity(
                ProjectiveTransformation3D.HOM_COORDS, ProjectiveTransformation3D.HOM_COORDS).
                        multiplyByScalarAndReturnNew(1.0 / Math.sqrt(ProjectiveTransformation3D.HOM_COORDS)),
                ABSOLUTE_ERROR));
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
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleZ(), 0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXY(),
                0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXZ(),
                0.0);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessYZ(),
                0.0);
        assertTrue(transformation.getA().equals(transformation.getAffineParameters().asMatrix().multiplyAndReturnNew(
                transformation.getRotation().asInhomogeneousMatrix()), ABSOLUTE_ERROR));
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        final var projectiveParameters1 = transformation.getProjectiveParameters();
        var norm = Utils.normF(projectiveParameters1);
        ArrayUtils.multiplyByScalar(projectiveParameters1, 1.0 / norm, projectiveParameters1);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters1, ABSOLUTE_ERROR);

        // Test constructor with T matrix
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // ensure last element is not zero
        t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1, ProjectiveTransformation3D.HOM_COORDS - 1,
                1.0);
        transformation = new ProjectiveTransformation3D(t);

        // check correctness
        assertNotNull(transformation.getRotation());
        assertNotNull(transformation.getTranslation());
        assertNotNull(transformation.getAffineParameters());
        assertNotNull(transformation.getProjectiveParameters());

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D((Matrix) null));

        // Force IllegalArgumentException
        final var badT = new Matrix(ProjectiveTransformation3D.HOM_COORDS + 1,
                ProjectiveTransformation3D.HOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(badT));

        // Test constructor with scale
        final var randomizer = new UniformRandomizer();
        final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        transformation = new ProjectiveTransformation3D(scale);

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
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        final var projectiveParameters2 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters2);
        ArrayUtils.multiplyByScalar(projectiveParameters2, 1.0 / norm, projectiveParameters2);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters2, ABSOLUTE_ERROR);

        // Test constructor with rotation
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotAxis = new double[Rotation3D.INHOM_COORDS];
        randomizer.fill(rotAxis, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // normalize axis
        norm = Utils.normF(rotAxis);
        ArrayUtils.multiplyByScalar(rotAxis, 1.0 / norm, rotAxis);

        final var rotation = Rotation3D.create(rotAxis, theta);
        transformation = new ProjectiveTransformation3D(rotation);

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
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        final var projectiveParameters3 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters3);
        ArrayUtils.multiplyByScalar(projectiveParameters3, 1.0 / norm, projectiveParameters3);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters3, ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D((Rotation3D) null));

        // Test constructor with scale and rotation
        transformation = new ProjectiveTransformation3D(scale, rotation);

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
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        final var projectiveParameters4 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters4);
        ArrayUtils.multiplyByScalar(projectiveParameters4, 1.0 / norm, projectiveParameters4);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters4, ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(scale, (Rotation3D) null));

        // Test constructor with affine parameters and rotation
        final var scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleZ = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var params = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);
        transformation = new ProjectiveTransformation3D(params, rotation);

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
        assertEquals(scaleX, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scaleZ, transformation.getAffineParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessXY, transformation.getAffineParameters().getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(skewnessXZ, transformation.getAffineParameters().getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessYZ, transformation.getAffineParameters().getSkewnessYZ(), ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        final var projectiveParameters5 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters5);
        ArrayUtils.multiplyByScalar(projectiveParameters5, 1.0 / norm, projectiveParameters5);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters5, ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(null, rotation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(params, null));

        // Test constructor with translation
        final var translation = new double[AffineParameters3D.INHOM_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        transformation = new ProjectiveTransformation3D(translation);

        // check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        final var projectiveParameters6 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters6);
        ArrayUtils.multiplyByScalar(projectiveParameters6, 1.0 / norm, projectiveParameters6);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters6, ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D((double[]) null));

        final var badTranslation = new double[ProjectiveTransformation3D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(badTranslation));

        // Test constructor with matrix A and translation
        Matrix a = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.INHOM_COORDS,
                ProjectiveTransformation3D.INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        transformation = new ProjectiveTransformation3D(a, translation);

        // check correctness
        assertNotNull(transformation.getRotation());
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertNotNull(transformation.getAffineParameters());
        assertTrue(a.equals(transformation.getA(), ABSOLUTE_ERROR));
        // check projective parameters are equal up to scale
        final var projectiveParameters7 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters7);
        ArrayUtils.multiplyByScalar(projectiveParameters7, 1.0 / norm, projectiveParameters7);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters7, ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D((Matrix) null, translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(a, null));

        // Force IllegalArgumentException
        final var badA = new Matrix(ProjectiveTransformation3D.INHOM_COORDS + 1,
                ProjectiveTransformation3D.INHOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(badA, translation));
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(a, badTranslation));

        // Test constructor with scale and translation
        transformation = new ProjectiveTransformation3D(scale, translation);

        // Check correctness
        assertEquals(0.0, transformation.getRotation().getRotationAngle(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getRotation().getRotationAxis()[1], ABSOLUTE_ERROR);
        assertEquals(1.0, Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        final var projectiveParameters8 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters8);
        ArrayUtils.multiplyByScalar(projectiveParameters8, 1.0 / norm, projectiveParameters8);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters8, ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(scale, (double[]) null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(scale, badTranslation));

        // Test constructor with rotation and translation
        transformation = new ProjectiveTransformation3D(rotation, translation);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        final var projectiveParameters9 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters9);
        ArrayUtils.multiplyByScalar(projectiveParameters9, 1.0 / norm, projectiveParameters9);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters9, ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D((Rotation3D) null, translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(rotation, null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(rotation, badTranslation));

        // Test constructor with scale, rotation and translation
        transformation = new ProjectiveTransformation3D(scale, rotation, translation);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        final var projectiveParameters10 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters10);
        ArrayUtils.multiplyByScalar(projectiveParameters10, 1.0 / norm, projectiveParameters10);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters10, ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(scale, null,
                translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(scale, rotation, null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(scale, rotation,
                badTranslation));

        // Test constructor with scale, rotation, translation and projective
        // parameters
        final var projectiveParameters11 = new double[ProjectiveTransformation3D.HOM_COORDS];
        do {
            // repeat to ensure that last element in projective parameters is
            // large enough (we use positive values to ensure that rotations
            // don't change sign
            randomizer.fill(projectiveParameters11, 0, MAX_RANDOM_VALUE);
        } while (Math.abs(projectiveParameters11[ProjectiveTransformation2D.HOM_COORDS - 1]) < ABSOLUTE_ERROR);
        transformation = new ProjectiveTransformation3D(scale, rotation, translation, projectiveParameters11);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        final var projectiveParameters12 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters12);
        ArrayUtils.multiplyByScalar(projectiveParameters12, 1.0 / norm, projectiveParameters12);
        norm = Utils.normF(projectiveParameters11);
        ArrayUtils.multiplyByScalar(projectiveParameters11, 1.0 / norm, projectiveParameters11);
        assertArrayEquals(projectiveParameters12, projectiveParameters11, ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(scale, null, translation,
                projectiveParameters12));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(scale, rotation, null,
                projectiveParameters12));
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(scale, rotation, translation,
                null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(scale, rotation,
                badTranslation, projectiveParameters12));
        final var badProjectiveParameters = new double[ProjectiveTransformation3D.HOM_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(scale, rotation,
                translation, badProjectiveParameters));

        // Test constructor with affine parameters, rotation and translation
        transformation = new ProjectiveTransformation3D(params, rotation, translation);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(scaleX, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scaleZ, transformation.getAffineParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessXY, transformation.getAffineParameters().getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(skewnessXZ, transformation.getAffineParameters().getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessYZ, transformation.getAffineParameters().getSkewnessYZ(), ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        final var projectiveParameters13 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters13);
        ArrayUtils.multiplyByScalar(projectiveParameters13, 1.0 / norm, projectiveParameters13);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters13, ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(null, rotation,
                translation));
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(params, null,
                translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(params, rotation,
                null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(params, rotation,
                badTranslation));

        // Test constructor with affine parameters, rotation, translation and
        // projective parameters
        transformation = new ProjectiveTransformation3D(params, rotation, translation, projectiveParameters11);

        // check correctness
        assertEquals(Math.abs(theta), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);
        assertEquals(scaleX, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scaleZ, transformation.getAffineParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessXY, transformation.getAffineParameters().getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(skewnessXZ, transformation.getAffineParameters().getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessYZ, transformation.getAffineParameters().getSkewnessYZ(), ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        final var projectiveParameters14 = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters14);
        ArrayUtils.multiplyByScalar(projectiveParameters14, 1.0 / norm, projectiveParameters14);
        norm = Utils.normF(projectiveParameters11);
        ArrayUtils.multiplyByScalar(projectiveParameters11, 1.0 / norm, projectiveParameters11);
        assertArrayEquals(projectiveParameters14, projectiveParameters11, ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(null, rotation,
                translation, projectiveParameters14));
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(params, null,
                translation, projectiveParameters14));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(params, rotation, null,
                projectiveParameters14));
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation3D(params, rotation, translation,
                null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(params, rotation,
                badTranslation, projectiveParameters14));
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation3D(params, rotation,
                translation, badProjectiveParameters));
    }

    @Test
    void testGetSetT() throws WrongSizeException {
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation3D();

        // check default value (identity up to scale
        assertTrue(transformation.getT()
                .equals(Matrix.identity(ProjectiveTransformation3D.HOM_COORDS, ProjectiveTransformation3D.HOM_COORDS)
                                .multiplyByScalarAndReturnNew(1.0 / Math.sqrt(ProjectiveTransformation3D.HOM_COORDS)),
                        ABSOLUTE_ERROR));

        // set new value
        transformation.setT(t);

        // check correctness
        assertTrue(transformation.getT().equals(t, ABSOLUTE_ERROR));

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> transformation.setT(null));

        // Force IllegalArgumentException
        final var badT = new Matrix(ProjectiveTransformation3D.HOM_COORDS + 1,
                ProjectiveTransformation3D.HOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> transformation.setT(badT));
    }

    @Test
    void testIsDegenerate() throws WrongSizeException, LockedException, NotReadyException, DecomposerException,
            NotAvailableException {
        // create non singular matrix
        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        assertFalse(ProjectiveTransformation3D.isDegenerate(t));

        final var transformation = new ProjectiveTransformation3D(t);
        transformation.setT(t);

        assertFalse(transformation.isDegenerate());

        // make matrix singular
        final var row = new double[ProjectiveTransformation3D.HOM_COORDS];
        Arrays.fill(row, 0.0);
        t.setSubmatrix(ProjectiveTransformation3D.HOM_COORDS - 1, 0,
                ProjectiveTransformation3D.HOM_COORDS - 1,
                ProjectiveTransformation3D.HOM_COORDS - 1, row);

        decomposer.setInputMatrix(t);
        decomposer.decompose();
        assertTrue(decomposer.isSingular());

        // check correctness
        ProjectiveTransformation3D.isDegenerate(t);

        transformation.setT(t);

        assertTrue(transformation.isDegenerate());

        // Force IllegalArgumentException
        final var wrongT = new Matrix(ProjectiveTransformation3D.HOM_COORDS + 1,
                ProjectiveTransformation3D.HOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation3D.isDegenerate(wrongT));
    }

    @Test
    void testGetSetA() throws WrongSizeException {
        final var a = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.INHOM_COORDS,
                ProjectiveTransformation3D.INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation3D();

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
        final var badA = new Matrix(ProjectiveTransformation3D.INHOM_COORDS + 1,
                ProjectiveTransformation3D.INHOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> transformation.setA(badA));
    }

    @Test
    void testNormalize() throws WrongSizeException {
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var norm = Utils.normF(t);
        final var normT = t.multiplyByScalarAndReturnNew(1.0 / norm);

        final var transformation = new ProjectiveTransformation3D();

        transformation.setT(t);

        assertTrue(transformation.getT().equals(t, ABSOLUTE_ERROR));

        // normalize
        transformation.normalize();

        // check equal-ness with normalized T matrix
        assertTrue(transformation.getT().equals(normT, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetRotation() throws RotationException, AlgebraException {
        final var transformation = new ProjectiveTransformation3D();

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
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> transformation.setRotation(null));
    }

    @Test
    void testAddRotation() throws RotationException, AlgebraException {
        final var transformation = new ProjectiveTransformation3D();

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
        assertEquals(Math.abs(theta1), Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis1[0]), Math.abs(transformation.getRotation().getRotationAxis()[0]),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis1[1]), Math.abs(transformation.getRotation().getRotationAxis()[1]),
                ABSOLUTE_ERROR);
        assertEquals(Math.abs(rotAxis1[2]), Math.abs(transformation.getRotation().getRotationAxis()[2]),
                ABSOLUTE_ERROR);

        // add second rotation
        transformation.addRotation(rotation2);

        // check correctness
        assertEquals(Math.abs(combinedRotation.getRotationAngle()),
                Math.abs(transformation.getRotation().getRotationAngle()), ABSOLUTE_ERROR);
        assertEquals(Math.abs(combinedRotation.getRotationAxis()[0]),
                Math.abs(transformation.getRotation().getRotationAxis()[0]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(combinedRotation.getRotationAxis()[1]),
                Math.abs(transformation.getRotation().getRotationAxis()[1]), ABSOLUTE_ERROR);
        assertEquals(Math.abs(combinedRotation.getRotationAxis()[2]),
                Math.abs(transformation.getRotation().getRotationAxis()[2]), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetAffineParameters() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleZ = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var skewnessXY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessXZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var skewnessYZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var params = new AffineParameters3D(scaleX, scaleY, scaleZ, skewnessXY, skewnessXZ, skewnessYZ);

        // instantiate transformation
        final var transformation = new ProjectiveTransformation3D();

        // check default values
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessXZ(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewnessYZ(),
                ABSOLUTE_ERROR);

        final var defaultParams = new AffineParameters3D();
        transformation.getAffineParameters(defaultParams);

        assertEquals(AffineParameters3D.DEFAULT_SCALE, defaultParams.getScaleX(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, defaultParams.getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, defaultParams.getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, defaultParams.getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, defaultParams.getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, defaultParams.getSkewnessYZ(), ABSOLUTE_ERROR);

        // set parameters
        transformation.setAffineParameters(params);

        // check correctness
        assertEquals(scaleX, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scaleZ, transformation.getAffineParameters().getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessXY, transformation.getAffineParameters().getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(skewnessXZ, transformation.getAffineParameters().getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessYZ, transformation.getAffineParameters().getSkewnessYZ(), ABSOLUTE_ERROR);

        final var params2 = new AffineParameters3D();
        transformation.getAffineParameters(params2);

        assertEquals(scaleX, params2.getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, params2.getScaleY(), ABSOLUTE_ERROR);
        assertEquals(scaleZ, params2.getScaleZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessXY, params2.getSkewnessXY(), ABSOLUTE_ERROR);
        assertEquals(skewnessXZ, params2.getSkewnessXZ(), ABSOLUTE_ERROR);
        assertEquals(skewnessYZ, params2.getSkewnessYZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetProjectiveParameters() {
        final var transformation = new ProjectiveTransformation3D();

        // check projective parameters are equal up to scale
        var projectiveParameters = transformation.getProjectiveParameters();
        final var norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // set new value
        projectiveParameters = new double[ProjectiveTransformation3D.HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(projectiveParameters);

        // set new value
        transformation.setProjectiveParameters(projectiveParameters);

        // check correctness
        assertArrayEquals(projectiveParameters, transformation.getProjectiveParameters(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslation() {
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translation = new double[ProjectiveTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        final var translation2 = transformation.getTranslation();
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, translation2.length);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var badTranslation = new double[ProjectiveTransformation3D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> transformation.setTranslation(badTranslation));
        assertThrows(WrongSizeException.class, () -> transformation.getTranslation(badTranslation));
    }

    @Test
    void testAddTranslation() {
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translation1 = new double[ProjectiveTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation2 = new double[ProjectiveTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        final var translationCopy = Arrays.copyOf(translation1, ProjectiveTransformation3D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation1[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation1[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // add translation
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[2] + translation2[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation1[2] + translation2[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var badTranslation = new double[ProjectiveTransformation3D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> transformation.addTranslation(badTranslation));
    }

    @Test
    void testAddTranslation2() {
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translation1 = new double[ProjectiveTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation2 = new double[ProjectiveTransformation3D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        final var translationCopy = Arrays.copyOf(translation1, ProjectiveTransformation3D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation1[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation1[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // add translation
        transformation.addTranslation(translation2[0], translation2[1], translation2[2]);

        // check correctness
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[2] + translation2[2], transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation1[2] + translation2[2], transformation.getTranslationZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testAddTranslation3() {
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translation1 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        translation1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var translation2 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        translation2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // check default value
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        final var translationCopy = new InhomogeneousPoint3D(translation1);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1.getInhomX(), transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomY(), transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomZ(), transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomX(), transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomY(), transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomZ(), transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // add translation
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1.getInhomX() + translation2.getInhomX(), transformation.getTranslation()[0],
                ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomY() + translation2.getInhomY(), transformation.getTranslation()[1],
                ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomZ() + translation2.getInhomZ(), transformation.getTranslation()[2],
                ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomX() + translation2.getInhomX(), transformation.getTranslationX(),
                ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomY() + translation2.getInhomY(), transformation.getTranslationY(),
                ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomZ() + translation2.getInhomZ(), transformation.getTranslationZ(),
                ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslationX() {
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), 0.0);

        // set new value
        transformation.setTranslationX(translationX);

        // check correctness
        assertEquals(translationX, transformation.getTranslationX(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslationY() {
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationY(), 0.0);

        // set new value
        transformation.setTranslationY(translationY);

        // check correctness
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslationZ() {
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationZ(), 0.0);

        // set new value
        transformation.setTranslationZ(translationZ);

        // check correctness
        assertEquals(translationZ, transformation.getTranslationZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testSetTranslationCoordinates() {
        final var transformation = new ProjectiveTransformation3D();

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
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translationX, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(translationX, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(translationZ, transformation.getTranslationZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslationPoint() {
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation = new InhomogeneousPoint3D(translationX, translationY, translationZ);

        // check default value
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[2], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(ProjectiveTransformation3D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
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
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationX1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationX2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);

        // set value
        transformation.setTranslationX(translationX1);

        // check correctness
        assertEquals(translationX1, transformation.getTranslationX(), ABSOLUTE_ERROR);

        // add translation x
        transformation.addTranslationX(translationX2);

        // check correctness
        assertEquals(translationX1 + translationX2, transformation.getTranslationX(), ABSOLUTE_ERROR);
    }

    @Test
    void testAddTranslationY() {
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationY1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set value
        transformation.setTranslationY(translationY1);

        // check correctness
        assertEquals(translationY1, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // add translation y
        transformation.addTranslationY(translationY2);

        // check correctness
        assertEquals(translationY1 + translationY2, transformation.getTranslationY(), ABSOLUTE_ERROR);
    }

    @Test
    void testAddTranslationZ() {
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var translationZ1 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationZ2 = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // set value
        transformation.setTranslationZ(translationZ1);

        // Check correctness
        assertEquals(translationZ1, transformation.getTranslationZ(), ABSOLUTE_ERROR);

        // add translationZ
        transformation.addTranslationZ(translationZ2);

        // check correctness
        assertEquals(translationZ1 + translationZ2, transformation.getTranslationZ(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetScale() throws AlgebraException {
        final var transformation = new ProjectiveTransformation3D();

        final var randomizer = new UniformRandomizer();
        final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

        // check default value
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters3D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);

        // set value
        transformation.setScale(scale);

        // check correctness
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
    }

    @Test
    void testAsMatrix() throws WrongSizeException {
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation3D();

        transformation.setT(t);

        // check correctness
        assertTrue(transformation.asMatrix().equals(t, ABSOLUTE_ERROR));

        final var t2 = new Matrix(ProjectiveTransformation3D.HOM_COORDS, ProjectiveTransformation3D.HOM_COORDS);
        transformation.asMatrix(t2);

        assertTrue(t2.equals(t, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var badT = new Matrix(ProjectiveTransformation3D.HOM_COORDS + 1,
                ProjectiveTransformation3D.HOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> transformation.asMatrix(badT));
    }

    @Test
    void testTransformPoint() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var coords = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation3D(t);

        final var point = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

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

        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation3D(t);

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

        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation3D(t);

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

        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation3D(t);

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
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation3D(t);

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
    void tesTransformDualQuadric() throws NonSymmetricMatrixException, AlgebraException {

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

        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation3D(t);

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
        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation3D(t);

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

        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation3D(t);

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
        Matrix t;
        final var luDecomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            luDecomposer.setInputMatrix(t);
            luDecomposer.decompose();
        } while (luDecomposer.isSingular());

        final var transformation = new ProjectiveTransformation3D(t);

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

        Matrix t;
        var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(
                    ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation3D(t);

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

        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation3D(t);

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

        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation3D(t);

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

        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation3D(t);

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
        assertEquals(outLines1.size(), inputLines.size());
        assertEquals(outLines2.size(), inputLines.size());
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

        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation3D(t);

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
        assertEquals(size, inputLines.size());
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

        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation3D(t);

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

        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation3D(t);

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
        final var expectedTriangle = new Triangle3D(expectedPoints.get(0), expectedPoints.get(1), expectedPoints.get(2));

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
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation3D(t);

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
        var numValid = 0;
        for (var times = 0; times < TIMES; times++) {
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
            final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var transformation = new ProjectiveTransformation3D(t);

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
            if (!outCameraCenter.equals(outCameraCenter2, ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(outCameraCenter.equals(outCameraCenter2, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testInverse() throws AlgebraException {

        // generate invertible matrix
        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation3D(t);

        final var invTransformation1 = transformation.inverseAndReturnNew();
        final var invTransformation2 = new ProjectiveTransformation3D();
        transformation.inverse(invTransformation2);

        // check that inverse transformation matrix is the inverse matrix of
        // current transformation
        assertTrue(invTransformation1.asMatrix().multiplyAndReturnNew(transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation3D.HOM_COORDS, MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));

        assertTrue(invTransformation2.asMatrix().multiplyAndReturnNew(transformation.asMatrix()).equals(Matrix.identity(
                MetricTransformation3D.HOM_COORDS, MetricTransformation3D.HOM_COORDS), ABSOLUTE_ERROR));

        // test transforming a random point by transformation and then by its
        // inverse to ensure it remains the same
        final var params = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
        final var randomizer = new UniformRandomizer();
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
    void testCombine() throws AlgebraException {
        final var t1 = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var t2 = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation1 = new ProjectiveTransformation3D(t1);

        final var transformation2 = new ProjectiveTransformation3D(t2);

        final var expectedMatrix = transformation1.asMatrix().multiplyAndReturnNew(transformation2.asMatrix());

        // combine and return result as a new transformation
        final var transformation3 = transformation1.combineAndReturnNew(transformation2);
        // combine into transformation1
        transformation1.combine(transformation2);

        // both matrices m1 and m3 need to be equal
        final var m3 = transformation3.asMatrix();
        final var m1 = transformation1.asMatrix();

        // check correctness
        assertTrue(m1.equals(m3, ABSOLUTE_ERROR));

        // check matrices are equal up to scale
        var norm = Utils.normF(expectedMatrix);
        expectedMatrix.multiplyByScalar(1.0 / norm);

        norm = Utils.normF(m1);
        m1.multiplyByScalar(1.0 / norm);

        norm = Utils.normF(m3);
        m3.multiplyByScalar(1.0 / norm);

        assertTrue(m1.equals(expectedMatrix, ABSOLUTE_ERROR));
        assertTrue(m3.equals(expectedMatrix, ABSOLUTE_ERROR));
    }

    @Test
    void testSetTransformationFromPoints() throws WrongSizeException, DecomposerException, CoincidentPointsException {

        Matrix t;
        do {
            // ensure transformation matrix is invertible
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, -1.0, 1.0);
            t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                    ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);
            final var norm = Utils.normF(t);
            // normalize T to increase accuracy
            t.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(t) < ProjectiveTransformation3D.HOM_COORDS);

        final var transformation1 = new ProjectiveTransformation3D(t);

        // generate 4 non coincident random points
        Point3D inputPoint1;
        Point3D inputPoint2;
        Point3D inputPoint3;
        Point3D inputPoint4;
        Point3D inputPoint5;
        Point3D outputPoint1;
        Point3D outputPoint2;
        Point3D outputPoint3;
        Point3D outputPoint4;
        Point3D outputPoint5;
        // build matrix initialized to zero
        final var m = new Matrix(15, 16);
        do {
            final var coordsMatrix = Matrix.createWithUniformRandomValues(5, 3, -1.0,
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

            coords = coordsMatrix.getSubmatrixAsArray(4, 0, 4,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint5 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            // Transform points using transformation
            outputPoint1 = transformation1.transformAndReturnNew(inputPoint1);
            outputPoint2 = transformation1.transformAndReturnNew(inputPoint2);
            outputPoint3 = transformation1.transformAndReturnNew(inputPoint3);
            outputPoint4 = transformation1.transformAndReturnNew(inputPoint4);
            outputPoint5 = transformation1.transformAndReturnNew(inputPoint5);

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

            var oXiX = oX * iX;
            var oXiY = oX * iY;
            var oXiZ = oX * iZ;
            var oXiW = oX * iW;

            var oYiX = oY * iX;
            var oYiY = oY * iY;
            var oYiZ = oY * iZ;
            var oYiW = oY * iW;

            var oZiX = oZ * iX;
            var oZiY = oZ * iY;
            var oZiZ = oZ * iZ;
            var oZiW = oZ * iW;

            var norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY +
                    oXiZ * oXiZ + oXiW * oXiW);

            m.setElementAt(0, 0, oWiX / norm);
            m.setElementAt(0, 1, oWiY / norm);
            m.setElementAt(0, 2, oWiZ / norm);
            m.setElementAt(0, 3, oWiW / norm);

            m.setElementAt(0, 12, -oXiX / norm);
            m.setElementAt(0, 13, -oXiY / norm);
            m.setElementAt(0, 14, -oXiZ / norm);
            m.setElementAt(0, 15, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY
                    + oYiZ * oYiZ + oYiW * oYiW);

            m.setElementAt(1, 4, oWiX / norm);
            m.setElementAt(1, 5, oWiY / norm);
            m.setElementAt(1, 6, oWiZ / norm);
            m.setElementAt(1, 7, oWiW / norm);

            m.setElementAt(1, 12, -oYiX / norm);
            m.setElementAt(1, 13, -oYiY / norm);
            m.setElementAt(1, 14, -oYiZ / norm);
            m.setElementAt(1, 15, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiX * oZiX + oZiY * oZiY
                    + oZiZ * oZiZ + oZiW * oZiW);

            m.setElementAt(2, 8, oWiX / norm);
            m.setElementAt(2, 9, oWiY / norm);
            m.setElementAt(2, 10, oWiZ / norm);
            m.setElementAt(2, 11, oWiW / norm);

            m.setElementAt(2, 12, -oZiX / norm);
            m.setElementAt(2, 13, -oZiY / norm);
            m.setElementAt(2, 14, -oZiZ / norm);
            m.setElementAt(2, 15, -oZiW / norm);

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

            oXiX = oX * iX;
            oXiY = oX * iY;
            oXiZ = oX * iZ;
            oXiW = oX * iW;

            oYiX = oY * iX;
            oYiY = oY * iY;
            oYiZ = oY * iZ;
            oYiW = oY * iW;

            oZiX = oZ * iX;
            oZiY = oZ * iY;
            oZiZ = oZ * iZ;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY
                    + oXiZ * oXiZ + oXiW * oXiW);

            m.setElementAt(3, 0, oWiX / norm);
            m.setElementAt(3, 1, oWiY / norm);
            m.setElementAt(3, 2, oWiZ / norm);
            m.setElementAt(3, 3, oWiW / norm);

            m.setElementAt(3, 12, -oXiX / norm);
            m.setElementAt(3, 13, -oXiY / norm);
            m.setElementAt(3, 14, -oXiZ / norm);
            m.setElementAt(3, 15, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY
                    + oYiZ * oYiZ + oYiW * oYiW);

            m.setElementAt(4, 4, oWiX / norm);
            m.setElementAt(4, 5, oWiY / norm);
            m.setElementAt(4, 6, oWiZ / norm);
            m.setElementAt(4, 7, oWiW / norm);

            m.setElementAt(4, 12, -oYiX / norm);
            m.setElementAt(4, 13, -oYiY / norm);
            m.setElementAt(4, 14, -oYiZ / norm);
            m.setElementAt(4, 15, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiX * oZiX + oZiY * oZiY
                    + oZiZ * oZiZ + oZiW * oZiW);

            m.setElementAt(5, 8, oWiX / norm);
            m.setElementAt(5, 9, oWiY / norm);
            m.setElementAt(5, 10, oWiZ / norm);
            m.setElementAt(5, 11, oWiW / norm);

            m.setElementAt(5, 12, -oZiX / norm);
            m.setElementAt(5, 13, -oZiY / norm);
            m.setElementAt(5, 14, -oZiZ / norm);
            m.setElementAt(5, 15, -oZiW / norm);

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

            oXiX = oX * iX;
            oXiY = oX * iY;
            oXiZ = oX * iZ;
            oXiW = oX * iW;

            oYiX = oY * iX;
            oYiY = oY * iY;
            oYiZ = oY * iZ;
            oYiW = oY * iW;

            oZiX = oZ * iX;
            oZiY = oZ * iY;
            oZiZ = oZ * iZ;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY
                    + oXiZ * oXiZ + oXiW * oXiW);

            m.setElementAt(6, 0, oWiX / norm);
            m.setElementAt(6, 1, oWiY / norm);
            m.setElementAt(6, 2, oWiZ / norm);
            m.setElementAt(6, 3, oWiW / norm);

            m.setElementAt(6, 12, -oXiX / norm);
            m.setElementAt(6, 13, -oXiY / norm);
            m.setElementAt(6, 14, -oXiZ / norm);
            m.setElementAt(6, 15, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY
                    + oYiZ * oYiZ + oYiW * oYiW);

            m.setElementAt(7, 4, oWiX / norm);
            m.setElementAt(7, 5, oWiY / norm);
            m.setElementAt(7, 6, oWiZ / norm);
            m.setElementAt(7, 7, oWiW / norm);

            m.setElementAt(7, 12, -oYiX / norm);
            m.setElementAt(7, 13, -oYiY / norm);
            m.setElementAt(7, 14, -oYiZ / norm);
            m.setElementAt(7, 15, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiX * oZiX + oZiY * oZiY
                    + oZiZ * oZiZ + oZiW * oZiW);

            m.setElementAt(8, 8, oWiX / norm);
            m.setElementAt(8, 9, oWiY / norm);
            m.setElementAt(8, 10, oWiZ / norm);
            m.setElementAt(8, 11, oWiW / norm);

            m.setElementAt(8, 12, -oZiX / norm);
            m.setElementAt(8, 13, -oZiY / norm);
            m.setElementAt(8, 14, -oZiZ / norm);
            m.setElementAt(8, 15, -oZiW / norm);

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

            oXiX = oX * iX;
            oXiY = oX * iY;
            oXiZ = oX * iZ;
            oXiW = oX * iW;

            oYiX = oY * iX;
            oYiY = oY * iY;
            oYiZ = oY * iZ;
            oYiW = oY * iW;

            oZiX = oZ * iX;
            oZiY = oZ * iY;
            oZiZ = oZ * iZ;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY
                    + oXiZ * oXiZ + oXiW * oXiW);

            m.setElementAt(9, 0, oWiX / norm);
            m.setElementAt(9, 1, oWiY / norm);
            m.setElementAt(9, 2, oWiZ / norm);
            m.setElementAt(9, 3, oWiW / norm);

            m.setElementAt(9, 12, -oXiX / norm);
            m.setElementAt(9, 13, -oXiY / norm);
            m.setElementAt(9, 14, -oXiZ / norm);
            m.setElementAt(9, 15, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY
                    + oYiZ * oYiZ + oYiW * oYiW);

            m.setElementAt(10, 4, oWiX / norm);
            m.setElementAt(10, 5, oWiY / norm);
            m.setElementAt(10, 6, oWiZ / norm);
            m.setElementAt(10, 7, oWiW / norm);

            m.setElementAt(10, 12, -oYiX / norm);
            m.setElementAt(10, 13, -oYiY / norm);
            m.setElementAt(10, 14, -oYiZ / norm);
            m.setElementAt(10, 15, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiX * oZiX + oZiY * oZiY
                    + oZiZ * oZiZ + oZiW * oZiW);

            m.setElementAt(11, 8, oWiX / norm);
            m.setElementAt(11, 9, oWiY / norm);
            m.setElementAt(11, 10, oWiZ / norm);
            m.setElementAt(11, 11, oWiW / norm);

            m.setElementAt(11, 12, -oZiX / norm);
            m.setElementAt(11, 13, -oZiY / norm);
            m.setElementAt(11, 14, -oZiZ / norm);
            m.setElementAt(11, 15, -oZiW / norm);

            // 5th pair of points
            iX = inputPoint5.getHomX();
            iY = inputPoint5.getHomY();
            iZ = inputPoint5.getHomZ();
            iW = inputPoint5.getHomW();

            oX = outputPoint5.getHomX();
            oY = outputPoint5.getHomY();
            oZ = outputPoint5.getHomZ();
            oW = outputPoint5.getHomW();

            oWiX = oW * iX;
            oWiY = oW * iY;
            oWiZ = oW * iZ;
            oWiW = oW * iW;

            oXiX = oX * iX;
            oXiY = oX * iY;
            oXiZ = oX * iZ;
            oXiW = oX * iW;

            oYiX = oY * iX;
            oYiY = oY * iY;
            oYiZ = oY * iZ;
            oYiW = oY * iW;

            oZiX = oZ * iX;
            oZiY = oZ * iY;
            oZiZ = oZ * iZ;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY
                    + oXiZ * oXiZ + oXiW * oXiW);

            m.setElementAt(12, 0, oWiX / norm);
            m.setElementAt(12, 1, oWiY / norm);
            m.setElementAt(12, 2, oWiZ / norm);
            m.setElementAt(12, 3, oWiW / norm);

            m.setElementAt(12, 12, -oXiX / norm);
            m.setElementAt(12, 13, -oXiY / norm);
            m.setElementAt(12, 14, -oXiZ / norm);
            m.setElementAt(12, 15, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY
                    + oYiZ * oYiZ + oYiW * oYiW);

            m.setElementAt(13, 4, oWiX / norm);
            m.setElementAt(13, 5, oWiY / norm);
            m.setElementAt(13, 6, oWiZ / norm);
            m.setElementAt(13, 7, oWiW / norm);

            m.setElementAt(13, 12, -oYiX / norm);
            m.setElementAt(13, 13, -oYiY / norm);
            m.setElementAt(13, 14, -oYiZ / norm);
            m.setElementAt(13, 15, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiX * oZiX + oZiY * oZiY
                    + oZiZ * oZiZ + oZiW * oZiW);

            m.setElementAt(14, 8, oWiX / norm);
            m.setElementAt(14, 9, oWiY / norm);
            m.setElementAt(14, 10, oWiZ / norm);
            m.setElementAt(14, 11, oWiW / norm);

            m.setElementAt(14, 12, -oZiX / norm);
            m.setElementAt(14, 13, -oZiY / norm);
            m.setElementAt(14, 14, -oZiZ / norm);
            m.setElementAt(14, 15, -oZiW / norm);

        } while (Utils.rank(m) < 8);

        // Now build another transformation
        final var transformation2 = new ProjectiveTransformation3D();

        // estimate transformation from corresponding points
        transformation2.setTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3, inputPoint4, inputPoint5,
                outputPoint1, outputPoint2, outputPoint3, outputPoint4, outputPoint5);

        // check correctness of transformation by checking transformed points
        assertTrue(outputPoint1.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint1)),
                5 * LARGE_ABSOLUTE_ERROR));
        assertTrue(outputPoint2.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint2)),
                5 * LARGE_ABSOLUTE_ERROR));
        assertTrue(outputPoint3.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint3)),
                5 * LARGE_ABSOLUTE_ERROR));
        assertTrue(outputPoint4.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint4)),
                5 * LARGE_ABSOLUTE_ERROR));
        assertTrue(outputPoint5.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint5)),
                5 * LARGE_ABSOLUTE_ERROR));

        // Force CoincidentPointsException
        final var finalInputPoint1 = inputPoint1;
        final var finalInputPoint3 = inputPoint3;
        final var finalInputPoint4 = inputPoint4;
        final var finalInputPoint5 = inputPoint5;
        final var finalOutputPoint1 = outputPoint1;
        final var finalOutputPoint3 = outputPoint3;
        final var finalOutputPoint4 = outputPoint4;
        final var finalOutputPoint5 = outputPoint5;
        assertThrows(CoincidentPointsException.class, () -> transformation2.setTransformationFromPoints(
                finalInputPoint1, finalInputPoint1, finalInputPoint3, finalInputPoint4, finalInputPoint5,
                finalOutputPoint1, finalOutputPoint1, finalOutputPoint3, finalOutputPoint4, finalOutputPoint5));
    }

    @Test
    void testConstructorFromPoints() throws WrongSizeException, DecomposerException, CoincidentPointsException {
        Matrix t;
        do {
            // ensure transformation matrix is invertible
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, -1.0, 1.0);
            t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                    ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);
            final var norm = Utils.normF(t);
            // normalize T to increase accuracy
            t.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(t) < ProjectiveTransformation3D.HOM_COORDS);

        final var transformation1 = new ProjectiveTransformation3D(t);

        // generate 4 non coincident random points
        Point3D inputPoint1;
        Point3D inputPoint2;
        Point3D inputPoint3;
        Point3D inputPoint4;
        Point3D inputPoint5;
        Point3D outputPoint1;
        Point3D outputPoint2;
        Point3D outputPoint3;
        Point3D outputPoint4;
        Point3D outputPoint5;
        // build matrix initialized to zero
        final var m = new Matrix(15, 16);
        do {
            final var coordsMatrix = Matrix.createWithUniformRandomValues(5, 3, -1.0,
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

            coords = coordsMatrix.getSubmatrixAsArray(4, 0, 4,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
            inputPoint5 = Point3D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

            // Transform points using transformation
            outputPoint1 = transformation1.transformAndReturnNew(inputPoint1);
            outputPoint2 = transformation1.transformAndReturnNew(inputPoint2);
            outputPoint3 = transformation1.transformAndReturnNew(inputPoint3);
            outputPoint4 = transformation1.transformAndReturnNew(inputPoint4);
            outputPoint5 = transformation1.transformAndReturnNew(inputPoint5);

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

            var oXiX = oX * iX;
            var oXiY = oX * iY;
            var oXiZ = oX * iZ;
            var oXiW = oX * iW;

            var oYiX = oY * iX;
            var oYiY = oY * iY;
            var oYiZ = oY * iZ;
            var oYiW = oY * iW;

            var oZiX = oZ * iX;
            var oZiY = oZ * iY;
            var oZiZ = oZ * iZ;
            var oZiW = oZ * iW;

            var norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY
                    + oXiZ * oXiZ + oXiW * oXiW);

            m.setElementAt(0, 0, oWiX / norm);
            m.setElementAt(0, 1, oWiY / norm);
            m.setElementAt(0, 2, oWiZ / norm);
            m.setElementAt(0, 3, oWiW / norm);

            m.setElementAt(0, 12, -oXiX / norm);
            m.setElementAt(0, 13, -oXiY / norm);
            m.setElementAt(0, 14, -oXiZ / norm);
            m.setElementAt(0, 15, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY
                    + oYiZ * oYiZ + oYiW * oYiW);

            m.setElementAt(1, 4, oWiX / norm);
            m.setElementAt(1, 5, oWiY / norm);
            m.setElementAt(1, 6, oWiZ / norm);
            m.setElementAt(1, 7, oWiW / norm);

            m.setElementAt(1, 12, -oYiX / norm);
            m.setElementAt(1, 13, -oYiY / norm);
            m.setElementAt(1, 14, -oYiZ / norm);
            m.setElementAt(1, 15, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiX * oZiX + oZiY * oZiY
                    + oZiZ * oZiZ + oZiW * oZiW);

            m.setElementAt(2, 8, oWiX / norm);
            m.setElementAt(2, 9, oWiY / norm);
            m.setElementAt(2, 10, oWiZ / norm);
            m.setElementAt(2, 11, oWiW / norm);

            m.setElementAt(2, 12, -oZiX / norm);
            m.setElementAt(2, 13, -oZiY / norm);
            m.setElementAt(2, 14, -oZiZ / norm);
            m.setElementAt(2, 15, -oZiW / norm);

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

            oXiX = oX * iX;
            oXiY = oX * iY;
            oXiZ = oX * iZ;
            oXiW = oX * iW;

            oYiX = oY * iX;
            oYiY = oY * iY;
            oYiZ = oY * iZ;
            oYiW = oY * iW;

            oZiX = oZ * iX;
            oZiY = oZ * iY;
            oZiZ = oZ * iZ;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY
                    + oXiZ * oXiZ + oXiW * oXiW);

            m.setElementAt(3, 0, oWiX / norm);
            m.setElementAt(3, 1, oWiY / norm);
            m.setElementAt(3, 2, oWiZ / norm);
            m.setElementAt(3, 3, oWiW / norm);

            m.setElementAt(3, 12, -oXiX / norm);
            m.setElementAt(3, 13, -oXiY / norm);
            m.setElementAt(3, 14, -oXiZ / norm);
            m.setElementAt(3, 15, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY
                    + oYiZ * oYiZ + oYiW * oYiW);

            m.setElementAt(4, 4, oWiX / norm);
            m.setElementAt(4, 5, oWiY / norm);
            m.setElementAt(4, 6, oWiZ / norm);
            m.setElementAt(4, 7, oWiW / norm);

            m.setElementAt(4, 12, -oYiX / norm);
            m.setElementAt(4, 13, -oYiY / norm);
            m.setElementAt(4, 14, -oYiZ / norm);
            m.setElementAt(4, 15, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiX * oZiX + oZiY * oZiY
                    + oZiZ * oZiZ + oZiW * oZiW);

            m.setElementAt(5, 8, oWiX / norm);
            m.setElementAt(5, 9, oWiY / norm);
            m.setElementAt(5, 10, oWiZ / norm);
            m.setElementAt(5, 11, oWiW / norm);

            m.setElementAt(5, 12, -oZiX / norm);
            m.setElementAt(5, 13, -oZiY / norm);
            m.setElementAt(5, 14, -oZiZ / norm);
            m.setElementAt(5, 15, -oZiW / norm);

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

            oXiX = oX * iX;
            oXiY = oX * iY;
            oXiZ = oX * iZ;
            oXiW = oX * iW;

            oYiX = oY * iX;
            oYiY = oY * iY;
            oYiZ = oY * iZ;
            oYiW = oY * iW;

            oZiX = oZ * iX;
            oZiY = oZ * iY;
            oZiZ = oZ * iZ;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY
                    + oXiZ * oXiZ + oXiW * oXiW);

            m.setElementAt(6, 0, oWiX / norm);
            m.setElementAt(6, 1, oWiY / norm);
            m.setElementAt(6, 2, oWiZ / norm);
            m.setElementAt(6, 3, oWiW / norm);

            m.setElementAt(6, 12, -oXiX / norm);
            m.setElementAt(6, 13, -oXiY / norm);
            m.setElementAt(6, 14, -oXiZ / norm);
            m.setElementAt(6, 15, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY
                    + oYiZ * oYiZ + oYiW * oYiW);

            m.setElementAt(7, 4, oWiX / norm);
            m.setElementAt(7, 5, oWiY / norm);
            m.setElementAt(7, 6, oWiZ / norm);
            m.setElementAt(7, 7, oWiW / norm);

            m.setElementAt(7, 12, -oYiX / norm);
            m.setElementAt(7, 13, -oYiY / norm);
            m.setElementAt(7, 14, -oYiZ / norm);
            m.setElementAt(7, 15, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiX * oZiX + oZiY * oZiY
                    + oZiZ * oZiZ + oZiW * oZiW);

            m.setElementAt(8, 8, oWiX / norm);
            m.setElementAt(8, 9, oWiY / norm);
            m.setElementAt(8, 10, oWiZ / norm);
            m.setElementAt(8, 11, oWiW / norm);

            m.setElementAt(8, 12, -oZiX / norm);
            m.setElementAt(8, 13, -oZiY / norm);
            m.setElementAt(8, 14, -oZiZ / norm);
            m.setElementAt(8, 15, -oZiW / norm);

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

            oXiX = oX * iX;
            oXiY = oX * iY;
            oXiZ = oX * iZ;
            oXiW = oX * iW;

            oYiX = oY * iX;
            oYiY = oY * iY;
            oYiZ = oY * iZ;
            oYiW = oY * iW;

            oZiX = oZ * iX;
            oZiY = oZ * iY;
            oZiZ = oZ * iZ;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY
                    + oXiZ * oXiZ + oXiW * oXiW);

            m.setElementAt(9, 0, oWiX / norm);
            m.setElementAt(9, 1, oWiY / norm);
            m.setElementAt(9, 2, oWiZ / norm);
            m.setElementAt(9, 3, oWiW / norm);

            m.setElementAt(9, 12, -oXiX / norm);
            m.setElementAt(9, 13, -oXiY / norm);
            m.setElementAt(9, 14, -oXiZ / norm);
            m.setElementAt(9, 15, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY
                    + oYiZ * oYiZ + oYiW * oYiW);

            m.setElementAt(10, 4, oWiX / norm);
            m.setElementAt(10, 5, oWiY / norm);
            m.setElementAt(10, 6, oWiZ / norm);
            m.setElementAt(10, 7, oWiW / norm);

            m.setElementAt(10, 12, -oYiX / norm);
            m.setElementAt(10, 13, -oYiY / norm);
            m.setElementAt(10, 14, -oYiZ / norm);
            m.setElementAt(10, 15, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiX * oZiX + oZiY * oZiY
                    + oZiZ * oZiZ + oZiW * oZiW);

            m.setElementAt(11, 8, oWiX / norm);
            m.setElementAt(11, 9, oWiY / norm);
            m.setElementAt(11, 10, oWiZ / norm);
            m.setElementAt(11, 11, oWiW / norm);

            m.setElementAt(11, 12, -oZiX / norm);
            m.setElementAt(11, 13, -oZiY / norm);
            m.setElementAt(11, 14, -oZiZ / norm);
            m.setElementAt(11, 15, -oZiW / norm);

            // 5th pair of points
            iX = inputPoint5.getHomX();
            iY = inputPoint5.getHomY();
            iZ = inputPoint5.getHomZ();
            iW = inputPoint5.getHomW();

            oX = outputPoint5.getHomX();
            oY = outputPoint5.getHomY();
            oZ = outputPoint5.getHomZ();
            oW = outputPoint5.getHomW();

            oWiX = oW * iX;
            oWiY = oW * iY;
            oWiZ = oW * iZ;
            oWiW = oW * iW;

            oXiX = oX * iX;
            oXiY = oX * iY;
            oXiZ = oX * iZ;
            oXiW = oX * iW;

            oYiX = oY * iX;
            oYiY = oY * iY;
            oYiZ = oY * iZ;
            oYiW = oY * iW;

            oZiX = oZ * iX;
            oZiY = oZ * iY;
            oZiZ = oZ * iZ;
            oZiW = oZ * iW;

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY
                    + oXiZ * oXiZ + oXiW * oXiW);

            m.setElementAt(12, 0, oWiX / norm);
            m.setElementAt(12, 1, oWiY / norm);
            m.setElementAt(12, 2, oWiZ / norm);
            m.setElementAt(12, 3, oWiW / norm);

            m.setElementAt(12, 12, -oXiX / norm);
            m.setElementAt(12, 13, -oXiY / norm);
            m.setElementAt(12, 14, -oXiZ / norm);
            m.setElementAt(12, 15, -oXiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY
                    + oYiZ * oYiZ + oYiW * oYiW);

            m.setElementAt(13, 4, oWiX / norm);
            m.setElementAt(13, 5, oWiY / norm);
            m.setElementAt(13, 6, oWiZ / norm);
            m.setElementAt(13, 7, oWiW / norm);

            m.setElementAt(13, 12, -oYiX / norm);
            m.setElementAt(13, 13, -oYiY / norm);
            m.setElementAt(13, 14, -oYiZ / norm);
            m.setElementAt(13, 15, -oYiW / norm);

            norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW + oZiX * oZiX + oZiY * oZiY
                    + oZiZ * oZiZ + oZiW * oZiW);

            m.setElementAt(14, 8, oWiX / norm);
            m.setElementAt(14, 9, oWiY / norm);
            m.setElementAt(14, 10, oWiZ / norm);
            m.setElementAt(14, 11, oWiW / norm);

            m.setElementAt(14, 12, -oZiX / norm);
            m.setElementAt(14, 13, -oZiY / norm);
            m.setElementAt(14, 14, -oZiZ / norm);
            m.setElementAt(14, 15, -oZiW / norm);

        } while (Utils.rank(m) < 8);

        // Now build another transformation
        var transformation2 = new ProjectiveTransformation3D(inputPoint1, inputPoint2, inputPoint3, inputPoint4,
                inputPoint5, outputPoint1, outputPoint2, outputPoint3, outputPoint4, outputPoint5);

        // check correctness of transformation by checking transformed points
        assertTrue(outputPoint1.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint1)),
                LARGE_ABSOLUTE_ERROR));
        assertTrue(outputPoint2.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint2)),
                LARGE_ABSOLUTE_ERROR));
        assertTrue(outputPoint3.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint3)),
                LARGE_ABSOLUTE_ERROR));
        assertTrue(outputPoint4.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint4)),
                LARGE_ABSOLUTE_ERROR));
        assertTrue(outputPoint5.equals(new InhomogeneousPoint3D(transformation2.transformAndReturnNew(inputPoint5)),
                LARGE_ABSOLUTE_ERROR));

        // Force CoincidentPointsException
        final var finalInputPoint1 = inputPoint1;
        final var finalInputPoint3 = inputPoint3;
        final var finalInputPoint4 = inputPoint4;
        final var finalInputPoint5 = inputPoint5;
        final var finalOutputPoint1 = outputPoint1;
        final var finalOutputPoint3 = outputPoint3;
        final var finalOutputPoint4 = outputPoint4;
        final var finalOutputPoint5 = outputPoint5;
        assertThrows(CoincidentPointsException.class, () -> new ProjectiveTransformation3D(finalInputPoint1,
                finalInputPoint1, finalInputPoint3, finalInputPoint4, finalInputPoint5, finalOutputPoint1,
                finalOutputPoint1, finalOutputPoint3, finalOutputPoint4, finalOutputPoint5));
    }

    @Test
    void testSetTransformationFromPlanes() throws CoincidentPlanesException, AlgebraException {
        Matrix t;
        do {
            // ensure transformation matrix is invertible
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, -1.0, 1.0);
            t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                    ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);
            final var norm = Utils.normF(t);
            // normalize T to increase accuracy
            t.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(t) < ProjectiveTransformation3D.HOM_COORDS);

        final var transformation1 = new ProjectiveTransformation3D(t);

        // generate 4 non coincident random planes
        Plane inputPlane1;
        Plane inputPlane2;
        Plane inputPlane3;
        Plane inputPlane4;
        Plane inputPlane5;
        Plane outputPlane1;
        Plane outputPlane2;
        Plane outputPlane3;
        Plane outputPlane4;
        Plane outputPlane5;
        // build matrix initialized to zero
        final var m = new Matrix(15, 16);
        do {
            final var paramsMatrix = Matrix.createWithUniformRandomValues(5, 4, -1.0,
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

            params = paramsMatrix.getSubmatrixAsArray(4, 0, 4,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane5 = new Plane(params);

            // Transform planes using transformation
            outputPlane1 = transformation1.transformAndReturnNew(inputPlane1);
            outputPlane2 = transformation1.transformAndReturnNew(inputPlane2);
            outputPlane3 = transformation1.transformAndReturnNew(inputPlane3);
            outputPlane4 = transformation1.transformAndReturnNew(inputPlane4);
            outputPlane5 = transformation1.transformAndReturnNew(inputPlane5);

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

            var oAiA = oA * iA;
            var oAiB = oA * iB;
            var oAiC = oA * iC;
            var oAiD = oA * iD;

            var oBiA = oB * iA;
            var oBiB = oB * iB;
            var oBiC = oB * iC;
            var oBiD = oB * iD;

            var oCiA = oC * iA;
            var oCiB = oC * iB;
            var oCiC = oC * iC;
            var oCiD = oC * iD;

            var norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(0, 0, oDiA / norm);
            m.setElementAt(0, 1, oDiB / norm);
            m.setElementAt(0, 2, oDiC / norm);
            m.setElementAt(0, 3, oDiD / norm);

            m.setElementAt(0, 12, -oAiA / norm);
            m.setElementAt(0, 13, -oAiB / norm);
            m.setElementAt(0, 14, -oAiC / norm);
            m.setElementAt(0, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(1, 4, oDiA / norm);
            m.setElementAt(1, 5, oDiB / norm);
            m.setElementAt(1, 6, oDiC / norm);
            m.setElementAt(1, 7, oDiD / norm);

            m.setElementAt(1, 12, -oBiA / norm);
            m.setElementAt(1, 13, -oBiB / norm);
            m.setElementAt(1, 14, -oBiC / norm);
            m.setElementAt(1, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(2, 8, oDiA / norm);
            m.setElementAt(2, 9, oDiB / norm);
            m.setElementAt(2, 10, oDiC / norm);
            m.setElementAt(2, 11, oDiD / norm);

            m.setElementAt(2, 12, -oCiA / norm);
            m.setElementAt(2, 13, -oCiB / norm);
            m.setElementAt(2, 14, -oCiC / norm);
            m.setElementAt(2, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(3, 0, oDiA / norm);
            m.setElementAt(3, 1, oDiB / norm);
            m.setElementAt(3, 2, oDiC / norm);
            m.setElementAt(3, 3, oDiD / norm);

            m.setElementAt(3, 12, -oAiA / norm);
            m.setElementAt(3, 13, -oAiB / norm);
            m.setElementAt(3, 14, -oAiC / norm);
            m.setElementAt(3, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(4, 4, oDiA / norm);
            m.setElementAt(4, 5, oDiB / norm);
            m.setElementAt(4, 6, oDiC / norm);
            m.setElementAt(4, 7, oDiD / norm);

            m.setElementAt(4, 12, -oBiA / norm);
            m.setElementAt(4, 13, -oBiB / norm);
            m.setElementAt(4, 14, -oBiC / norm);
            m.setElementAt(4, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(5, 8, oDiA / norm);
            m.setElementAt(5, 9, oDiB / norm);
            m.setElementAt(5, 10, oDiC / norm);
            m.setElementAt(5, 11, oDiD / norm);

            m.setElementAt(5, 12, -oCiA / norm);
            m.setElementAt(5, 13, -oCiB / norm);
            m.setElementAt(5, 14, -oCiC / norm);
            m.setElementAt(5, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(6, 0, oDiA / norm);
            m.setElementAt(6, 1, oDiB / norm);
            m.setElementAt(6, 2, oDiC / norm);
            m.setElementAt(6, 3, oDiD / norm);

            m.setElementAt(6, 12, -oAiA / norm);
            m.setElementAt(6, 13, -oAiB / norm);
            m.setElementAt(6, 14, -oAiC / norm);
            m.setElementAt(6, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(7, 4, oDiA / norm);
            m.setElementAt(7, 5, oDiB / norm);
            m.setElementAt(7, 6, oDiC / norm);
            m.setElementAt(7, 7, oDiD / norm);

            m.setElementAt(7, 12, -oBiA / norm);
            m.setElementAt(7, 13, -oBiB / norm);
            m.setElementAt(7, 14, -oBiC / norm);
            m.setElementAt(7, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(8, 8, oDiA / norm);
            m.setElementAt(8, 9, oDiB / norm);
            m.setElementAt(8, 10, oDiC / norm);
            m.setElementAt(8, 11, oDiD / norm);

            m.setElementAt(8, 12, -oCiA / norm);
            m.setElementAt(8, 13, -oCiB / norm);
            m.setElementAt(8, 14, -oCiC / norm);
            m.setElementAt(8, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(9, 0, oDiA / norm);
            m.setElementAt(9, 1, oDiB / norm);
            m.setElementAt(9, 2, oDiC / norm);
            m.setElementAt(9, 3, oDiD / norm);

            m.setElementAt(9, 12, -oAiA / norm);
            m.setElementAt(9, 13, -oAiB / norm);
            m.setElementAt(9, 14, -oAiC / norm);
            m.setElementAt(9, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(10, 4, oDiA / norm);
            m.setElementAt(10, 5, oDiB / norm);
            m.setElementAt(10, 6, oDiC / norm);
            m.setElementAt(10, 7, oDiD / norm);

            m.setElementAt(10, 12, -oBiA / norm);
            m.setElementAt(10, 13, -oBiB / norm);
            m.setElementAt(10, 14, -oBiC / norm);
            m.setElementAt(10, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(11, 8, oDiA / norm);
            m.setElementAt(11, 9, oDiB / norm);
            m.setElementAt(11, 10, oDiC / norm);
            m.setElementAt(11, 11, oDiD / norm);

            m.setElementAt(11, 12, -oCiA / norm);
            m.setElementAt(11, 13, -oCiB / norm);
            m.setElementAt(11, 14, -oCiC / norm);
            m.setElementAt(11, 15, -oCiD / norm);

            // 5th pair of planes
            iA = inputPlane5.getA();
            iB = inputPlane5.getB();
            iC = inputPlane5.getC();
            iD = inputPlane5.getD();

            oA = outputPlane5.getA();
            oB = outputPlane5.getB();
            oC = outputPlane5.getC();
            oD = outputPlane5.getD();

            oDiA = oD * iA;
            oDiB = oD * iB;
            oDiC = oD * iC;
            oDiD = oD * iD;

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(12, 0, oDiA / norm);
            m.setElementAt(12, 1, oDiB / norm);
            m.setElementAt(12, 2, oDiC / norm);
            m.setElementAt(12, 3, oDiD / norm);

            m.setElementAt(12, 12, -oAiA / norm);
            m.setElementAt(12, 13, -oAiB / norm);
            m.setElementAt(12, 14, -oAiC / norm);
            m.setElementAt(12, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(13, 4, oDiA / norm);
            m.setElementAt(13, 5, oDiB / norm);
            m.setElementAt(13, 6, oDiC / norm);
            m.setElementAt(13, 7, oDiD / norm);

            m.setElementAt(13, 12, -oBiA / norm);
            m.setElementAt(13, 13, -oBiB / norm);
            m.setElementAt(13, 14, -oBiC / norm);
            m.setElementAt(13, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(14, 8, oDiA / norm);
            m.setElementAt(14, 9, oDiB / norm);
            m.setElementAt(14, 10, oDiC / norm);
            m.setElementAt(14, 11, oDiD / norm);

            m.setElementAt(14, 12, -oCiA / norm);
            m.setElementAt(14, 13, -oCiB / norm);
            m.setElementAt(14, 14, -oCiC / norm);
            m.setElementAt(14, 15, -oCiD / norm);

        } while (Utils.rank(m) < 8);

        // Now build another transformation
        final var transformation2 = new ProjectiveTransformation3D();

        // estimate transformation from corresponding planes
        transformation2.setTransformationFromPlanes(inputPlane1, inputPlane2, inputPlane3, inputPlane4, inputPlane5,
                outputPlane1, outputPlane2, outputPlane3, outputPlane4, outputPlane5);

        // check correctness of lines (up to scale)
        var p = transformation2.transformAndReturnNew(inputPlane1);
        p.normalize();

        assertEquals(Math.abs(outputPlane1.getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane2);
        p.normalize();

        assertEquals(Math.abs(outputPlane2.getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane3);
        p.normalize();

        assertEquals(Math.abs(outputPlane3.getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane4);
        p.normalize();

        assertEquals(Math.abs(outputPlane4.getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane5);
        p.normalize();

        assertEquals(Math.abs(outputPlane5.getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane5.getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane5.getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane5.getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        // Force CoincidentPlanesException
        final var finalInputPlane1 = inputPlane1;
        final var finalInputPlane3 = inputPlane3;
        final var finalInputPlane4 = inputPlane4;
        final var finalInputPlane5 = inputPlane5;
        final var finalOutputPlane1 = outputPlane1;
        final var finalOutputPlane3 = outputPlane3;
        final var finalOutputPlane4 = outputPlane4;
        final var finalOutputPlane5 = outputPlane5;
        assertThrows(CoincidentPlanesException.class, () -> transformation2.setTransformationFromPlanes(
                finalInputPlane1, finalInputPlane1, finalInputPlane3, finalInputPlane4, finalInputPlane5,
                finalOutputPlane1, finalOutputPlane1, finalOutputPlane3, finalOutputPlane4, finalOutputPlane5));
    }

    @Test
    void testConstructorFromPlanes() throws CoincidentPlanesException, AlgebraException {

        Matrix t;
        do {
            // ensure transformation matrix is invertible
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, -1.0, 1.0);
            t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                    ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);
            final var norm = Utils.normF(t);
            // normalize T to increase accuracy
            t.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(t) < ProjectiveTransformation3D.HOM_COORDS);

        final var transformation1 = new ProjectiveTransformation3D(t);

        // generate 4 non coincident random planes
        Plane inputPlane1;
        Plane inputPlane2;
        Plane inputPlane3;
        Plane inputPlane4;
        Plane inputPlane5;
        Plane outputPlane1;
        Plane outputPlane2;
        Plane outputPlane3;
        Plane outputPlane4;
        Plane outputPlane5;
        // build matrix initialized to zero
        final var m = new Matrix(15, 16);
        do {
            final var paramsMatrix = Matrix.createWithUniformRandomValues(5, 4, -1.0,
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

            params = paramsMatrix.getSubmatrixAsArray(4, 0, 4,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane5 = new Plane(params);

            // Transform planes using transformation
            outputPlane1 = transformation1.transformAndReturnNew(inputPlane1);
            outputPlane2 = transformation1.transformAndReturnNew(inputPlane2);
            outputPlane3 = transformation1.transformAndReturnNew(inputPlane3);
            outputPlane4 = transformation1.transformAndReturnNew(inputPlane4);
            outputPlane5 = transformation1.transformAndReturnNew(inputPlane5);

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

            var oAiA = oA * iA;
            var oAiB = oA * iB;
            var oAiC = oA * iC;
            var oAiD = oA * iD;

            var oBiA = oB * iA;
            var oBiB = oB * iB;
            var oBiC = oB * iC;
            var oBiD = oB * iD;

            var oCiA = oC * iA;
            var oCiB = oC * iB;
            var oCiC = oC * iC;
            var oCiD = oC * iD;

            var norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(0, 0, oDiA / norm);
            m.setElementAt(0, 1, oDiB / norm);
            m.setElementAt(0, 2, oDiC / norm);
            m.setElementAt(0, 3, oDiD / norm);

            m.setElementAt(0, 12, -oAiA / norm);
            m.setElementAt(0, 13, -oAiB / norm);
            m.setElementAt(0, 14, -oAiC / norm);
            m.setElementAt(0, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(1, 4, oDiA / norm);
            m.setElementAt(1, 5, oDiB / norm);
            m.setElementAt(1, 6, oDiC / norm);
            m.setElementAt(1, 7, oDiD / norm);

            m.setElementAt(1, 12, -oBiA / norm);
            m.setElementAt(1, 13, -oBiB / norm);
            m.setElementAt(1, 14, -oBiC / norm);
            m.setElementAt(1, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(2, 8, oDiA / norm);
            m.setElementAt(2, 9, oDiB / norm);
            m.setElementAt(2, 10, oDiC / norm);
            m.setElementAt(2, 11, oDiD / norm);

            m.setElementAt(2, 12, -oCiA / norm);
            m.setElementAt(2, 13, -oCiB / norm);
            m.setElementAt(2, 14, -oCiC / norm);
            m.setElementAt(2, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(3, 0, oDiA / norm);
            m.setElementAt(3, 1, oDiB / norm);
            m.setElementAt(3, 2, oDiC / norm);
            m.setElementAt(3, 3, oDiD / norm);

            m.setElementAt(3, 12, -oAiA / norm);
            m.setElementAt(3, 13, -oAiB / norm);
            m.setElementAt(3, 14, -oAiC / norm);
            m.setElementAt(3, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(4, 4, oDiA / norm);
            m.setElementAt(4, 5, oDiB / norm);
            m.setElementAt(4, 6, oDiC / norm);
            m.setElementAt(4, 7, oDiD / norm);

            m.setElementAt(4, 12, -oBiA / norm);
            m.setElementAt(4, 13, -oBiB / norm);
            m.setElementAt(4, 14, -oBiC / norm);
            m.setElementAt(4, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(5, 8, oDiA / norm);
            m.setElementAt(5, 9, oDiB / norm);
            m.setElementAt(5, 10, oDiC / norm);
            m.setElementAt(5, 11, oDiD / norm);

            m.setElementAt(5, 12, -oCiA / norm);
            m.setElementAt(5, 13, -oCiB / norm);
            m.setElementAt(5, 14, -oCiC / norm);
            m.setElementAt(5, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(6, 0, oDiA / norm);
            m.setElementAt(6, 1, oDiB / norm);
            m.setElementAt(6, 2, oDiC / norm);
            m.setElementAt(6, 3, oDiD / norm);

            m.setElementAt(6, 12, -oAiA / norm);
            m.setElementAt(6, 13, -oAiB / norm);
            m.setElementAt(6, 14, -oAiC / norm);
            m.setElementAt(6, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(7, 4, oDiA / norm);
            m.setElementAt(7, 5, oDiB / norm);
            m.setElementAt(7, 6, oDiC / norm);
            m.setElementAt(7, 7, oDiD / norm);

            m.setElementAt(7, 12, -oBiA / norm);
            m.setElementAt(7, 13, -oBiB / norm);
            m.setElementAt(7, 14, -oBiC / norm);
            m.setElementAt(7, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(8, 8, oDiA / norm);
            m.setElementAt(8, 9, oDiB / norm);
            m.setElementAt(8, 10, oDiC / norm);
            m.setElementAt(8, 11, oDiD / norm);

            m.setElementAt(8, 12, -oCiA / norm);
            m.setElementAt(8, 13, -oCiB / norm);
            m.setElementAt(8, 14, -oCiC / norm);
            m.setElementAt(8, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(9, 0, oDiA / norm);
            m.setElementAt(9, 1, oDiB / norm);
            m.setElementAt(9, 2, oDiC / norm);
            m.setElementAt(9, 3, oDiD / norm);

            m.setElementAt(9, 12, -oAiA / norm);
            m.setElementAt(9, 13, -oAiB / norm);
            m.setElementAt(9, 14, -oAiC / norm);
            m.setElementAt(9, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(10, 4, oDiA / norm);
            m.setElementAt(10, 5, oDiB / norm);
            m.setElementAt(10, 6, oDiC / norm);
            m.setElementAt(10, 7, oDiD / norm);

            m.setElementAt(10, 12, -oBiA / norm);
            m.setElementAt(10, 13, -oBiB / norm);
            m.setElementAt(10, 14, -oBiC / norm);
            m.setElementAt(10, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(11, 8, oDiA / norm);
            m.setElementAt(11, 9, oDiB / norm);
            m.setElementAt(11, 10, oDiC / norm);
            m.setElementAt(11, 11, oDiD / norm);

            m.setElementAt(11, 12, -oCiA / norm);
            m.setElementAt(11, 13, -oCiB / norm);
            m.setElementAt(11, 14, -oCiC / norm);
            m.setElementAt(11, 15, -oCiD / norm);

            // 5th pair of planes
            iA = inputPlane5.getA();
            iB = inputPlane5.getB();
            iC = inputPlane5.getC();
            iD = inputPlane5.getD();

            oA = outputPlane5.getA();
            oB = outputPlane5.getB();
            oC = outputPlane5.getC();
            oD = outputPlane5.getD();

            oDiA = oD * iA;
            oDiB = oD * iB;
            oDiC = oD * iC;
            oDiD = oD * iD;

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(12, 0, oDiA / norm);
            m.setElementAt(12, 1, oDiB / norm);
            m.setElementAt(12, 2, oDiC / norm);
            m.setElementAt(12, 3, oDiD / norm);

            m.setElementAt(12, 12, -oAiA / norm);
            m.setElementAt(12, 13, -oAiB / norm);
            m.setElementAt(12, 14, -oAiC / norm);
            m.setElementAt(12, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(13, 4, oDiA / norm);
            m.setElementAt(13, 5, oDiB / norm);
            m.setElementAt(13, 6, oDiC / norm);
            m.setElementAt(13, 7, oDiD / norm);

            m.setElementAt(13, 12, -oBiA / norm);
            m.setElementAt(13, 13, -oBiB / norm);
            m.setElementAt(13, 14, -oBiC / norm);
            m.setElementAt(13, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(14, 8, oDiA / norm);
            m.setElementAt(14, 9, oDiB / norm);
            m.setElementAt(14, 10, oDiC / norm);
            m.setElementAt(14, 11, oDiD / norm);

            m.setElementAt(14, 12, -oCiA / norm);
            m.setElementAt(14, 13, -oCiB / norm);
            m.setElementAt(14, 14, -oCiC / norm);
            m.setElementAt(14, 15, -oCiD / norm);

        } while (Utils.rank(m) < 8);

        // Now build another transformation
        var transformation2 = new ProjectiveTransformation3D(inputPlane1, inputPlane2, inputPlane3, inputPlane4,
                inputPlane5, outputPlane1, outputPlane2, outputPlane3, outputPlane4, outputPlane5);

        // check correctness of lines (up to scale)
        var p = transformation2.transformAndReturnNew(inputPlane1);
        p.normalize();

        assertEquals(Math.abs(outputPlane1.getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane1.getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane2);
        p.normalize();

        assertEquals(Math.abs(outputPlane2.getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane2.getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane3);
        p.normalize();

        assertEquals(Math.abs(outputPlane3.getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane3.getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane4);
        p.normalize();

        assertEquals(Math.abs(outputPlane4.getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane4.getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputPlane5);
        p.normalize();

        assertEquals(Math.abs(outputPlane5.getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane5.getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane5.getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputPlane5.getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        // Force CoincidentPlanesException
        final var finalInputPlane1 = inputPlane3;
        final var finalInputPlane3 = inputPlane3;
        final var finalInputPlane4 = inputPlane4;
        final var finalInputPlane5 = inputPlane5;
        final var finalOutputPlane1 = outputPlane1;
        final var finalOutputPlane3 = outputPlane3;
        final var finalOutputPlane4 = outputPlane4;
        final var finalOutputPlane5 = outputPlane5;
        assertThrows(CoincidentPlanesException.class, () -> new ProjectiveTransformation3D(finalInputPlane1,
                finalInputPlane1, finalInputPlane3, finalInputPlane4, finalInputPlane5, finalOutputPlane1,
                finalOutputPlane1, finalOutputPlane3, finalOutputPlane4, finalOutputPlane5));
    }

    @Test
    void testSetTransformationFromLines() throws CoincidentLinesException, AlgebraException, CoincidentPlanesException {
        Matrix t;
        do {
            // ensure transformation matrix is invertible
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, -1.0, 1.0);
            t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                    ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);
            final var norm = Utils.normF(t);
            // normalize T to increase accuracy
            t.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(t) < ProjectiveTransformation3D.HOM_COORDS);

        final var transformation1 = new ProjectiveTransformation3D(t);

        // generate 4 non coincident random planes
        Plane inputPlane1;
        Plane inputPlane2;
        Plane inputPlane3;
        Plane inputPlane4;
        Plane inputPlane5;
        Plane inputPlane6;
        Line3D inputLine1;
        Line3D inputLine2;
        Line3D inputLine3;
        Line3D outputLine1;
        Line3D outputLine2;
        Line3D outputLine3;
        // build matrix initialized to zero
        final var m = new Matrix(15, 16);
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

            params = paramsMatrix.getSubmatrixAsArray(4, 0, 4,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane5 = new Plane(params);

            params = paramsMatrix.getSubmatrixAsArray(5, 0, 5,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane6 = new Plane(params);

            inputLine3 = new Line3D(inputPlane5, inputPlane6);

            // Transform lines using transformation
            outputLine1 = transformation1.transformAndReturnNew(inputLine1);
            outputLine2 = transformation1.transformAndReturnNew(inputLine2);
            outputLine3 = transformation1.transformAndReturnNew(inputLine3);

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

            var oAiA = oA * iA;
            var oAiB = oA * iB;
            var oAiC = oA * iC;
            var oAiD = oA * iD;

            var oBiA = oB * iA;
            var oBiB = oB * iB;
            var oBiC = oB * iC;
            var oBiD = oB * iD;

            var oCiA = oC * iA;
            var oCiB = oC * iB;
            var oCiC = oC * iC;
            var oCiD = oC * iD;

            var norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(0, 0, oDiA / norm);
            m.setElementAt(0, 1, oDiB / norm);
            m.setElementAt(0, 2, oDiC / norm);
            m.setElementAt(0, 3, oDiD / norm);

            m.setElementAt(0, 12, -oAiA / norm);
            m.setElementAt(0, 13, -oAiB / norm);
            m.setElementAt(0, 14, -oAiC / norm);
            m.setElementAt(0, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(1, 4, oDiA / norm);
            m.setElementAt(1, 5, oDiB / norm);
            m.setElementAt(1, 6, oDiC / norm);
            m.setElementAt(1, 7, oDiD / norm);

            m.setElementAt(1, 12, -oBiA / norm);
            m.setElementAt(1, 13, -oBiB / norm);
            m.setElementAt(1, 14, -oBiC / norm);
            m.setElementAt(1, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(2, 8, oDiA / norm);
            m.setElementAt(2, 9, oDiB / norm);
            m.setElementAt(2, 10, oDiC / norm);
            m.setElementAt(2, 11, oDiD / norm);

            m.setElementAt(2, 12, -oCiA / norm);
            m.setElementAt(2, 13, -oCiB / norm);
            m.setElementAt(2, 14, -oCiC / norm);
            m.setElementAt(2, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(3, 0, oDiA / norm);
            m.setElementAt(3, 1, oDiB / norm);
            m.setElementAt(3, 2, oDiC / norm);
            m.setElementAt(3, 3, oDiD / norm);

            m.setElementAt(3, 12, -oAiA / norm);
            m.setElementAt(3, 13, -oAiB / norm);
            m.setElementAt(3, 14, -oAiC / norm);
            m.setElementAt(3, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(4, 4, oDiA / norm);
            m.setElementAt(4, 5, oDiB / norm);
            m.setElementAt(4, 6, oDiC / norm);
            m.setElementAt(4, 7, oDiD / norm);

            m.setElementAt(4, 12, -oBiA / norm);
            m.setElementAt(4, 13, -oBiB / norm);
            m.setElementAt(4, 14, -oBiC / norm);
            m.setElementAt(4, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(5, 8, oDiA / norm);
            m.setElementAt(5, 9, oDiB / norm);
            m.setElementAt(5, 10, oDiC / norm);
            m.setElementAt(5, 11, oDiD / norm);

            m.setElementAt(5, 12, -oCiA / norm);
            m.setElementAt(5, 13, -oCiB / norm);
            m.setElementAt(5, 14, -oCiC / norm);
            m.setElementAt(5, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(6, 0, oDiA / norm);
            m.setElementAt(6, 1, oDiB / norm);
            m.setElementAt(6, 2, oDiC / norm);
            m.setElementAt(6, 3, oDiD / norm);

            m.setElementAt(6, 12, -oAiA / norm);
            m.setElementAt(6, 13, -oAiB / norm);
            m.setElementAt(6, 14, -oAiC / norm);
            m.setElementAt(6, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(7, 4, oDiA / norm);
            m.setElementAt(7, 5, oDiB / norm);
            m.setElementAt(7, 6, oDiC / norm);
            m.setElementAt(7, 7, oDiD / norm);

            m.setElementAt(7, 12, -oBiA / norm);
            m.setElementAt(7, 13, -oBiB / norm);
            m.setElementAt(7, 14, -oBiC / norm);
            m.setElementAt(7, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(8, 8, oDiA / norm);
            m.setElementAt(8, 9, oDiB / norm);
            m.setElementAt(8, 10, oDiC / norm);
            m.setElementAt(8, 11, oDiD / norm);

            m.setElementAt(8, 12, -oCiA / norm);
            m.setElementAt(8, 13, -oCiB / norm);
            m.setElementAt(8, 14, -oCiC / norm);
            m.setElementAt(8, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(9, 0, oDiA / norm);
            m.setElementAt(9, 1, oDiB / norm);
            m.setElementAt(9, 2, oDiC / norm);
            m.setElementAt(9, 3, oDiD / norm);

            m.setElementAt(9, 12, -oAiA / norm);
            m.setElementAt(9, 13, -oAiB / norm);
            m.setElementAt(9, 14, -oAiC / norm);
            m.setElementAt(9, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(10, 4, oDiA / norm);
            m.setElementAt(10, 5, oDiB / norm);
            m.setElementAt(10, 6, oDiC / norm);
            m.setElementAt(10, 7, oDiD / norm);

            m.setElementAt(10, 12, -oBiA / norm);
            m.setElementAt(10, 13, -oBiB / norm);
            m.setElementAt(10, 14, -oBiC / norm);
            m.setElementAt(10, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(11, 8, oDiA / norm);
            m.setElementAt(11, 9, oDiB / norm);
            m.setElementAt(11, 10, oDiC / norm);
            m.setElementAt(11, 11, oDiD / norm);

            m.setElementAt(11, 12, -oCiA / norm);
            m.setElementAt(11, 13, -oCiB / norm);
            m.setElementAt(11, 14, -oCiC / norm);
            m.setElementAt(11, 15, -oCiD / norm);

            // 5th pair of planes
            iA = inputLine3.getPlane1().getA();
            iB = inputLine3.getPlane1().getB();
            iC = inputLine3.getPlane1().getC();
            iD = inputLine3.getPlane1().getD();

            oA = outputLine3.getPlane1().getA();
            oB = outputLine3.getPlane1().getB();
            oC = outputLine3.getPlane1().getC();
            oD = outputLine3.getPlane1().getD();

            oDiA = oD * iA;
            oDiB = oD * iB;
            oDiC = oD * iC;
            oDiD = oD * iD;

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(12, 0, oDiA / norm);
            m.setElementAt(12, 1, oDiB / norm);
            m.setElementAt(12, 2, oDiC / norm);
            m.setElementAt(12, 3, oDiD / norm);

            m.setElementAt(12, 12, -oAiA / norm);
            m.setElementAt(12, 13, -oAiB / norm);
            m.setElementAt(12, 14, -oAiC / norm);
            m.setElementAt(12, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(13, 4, oDiA / norm);
            m.setElementAt(13, 5, oDiB / norm);
            m.setElementAt(13, 6, oDiC / norm);
            m.setElementAt(13, 7, oDiD / norm);

            m.setElementAt(13, 12, -oBiA / norm);
            m.setElementAt(13, 13, -oBiB / norm);
            m.setElementAt(13, 14, -oBiC / norm);
            m.setElementAt(13, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(14, 8, oDiA / norm);
            m.setElementAt(14, 9, oDiB / norm);
            m.setElementAt(14, 10, oDiC / norm);
            m.setElementAt(14, 11, oDiD / norm);

            m.setElementAt(14, 12, -oCiA / norm);
            m.setElementAt(14, 13, -oCiB / norm);
            m.setElementAt(14, 14, -oCiC / norm);
            m.setElementAt(14, 15, -oCiD / norm);

        } while (Utils.rank(m) < 8);

        // Now build another transformation
        var transformation2 = new ProjectiveTransformation3D();

        // estimate transformation from corresponding lines
        transformation2.setTransformationFromLines(inputLine1, inputLine2, inputLine3, outputLine1, outputLine2,
                outputLine3);

        // check correctness of lines (up to scale)
        var p = transformation2.transformAndReturnNew(inputLine1.getPlane1());
        p.normalize();

        assertEquals(Math.abs(outputLine1.getPlane1().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane1().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane1().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane1().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputLine1.getPlane2());
        p.normalize();

        assertEquals(Math.abs(outputLine1.getPlane2().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane2().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane2().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane2().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputLine2.getPlane1());
        p.normalize();

        assertEquals(Math.abs(outputLine2.getPlane1().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane1().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane1().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane1().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputLine2.getPlane2());
        p.normalize();

        assertEquals(Math.abs(outputLine2.getPlane2().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane2().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane2().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane2().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputLine3.getPlane1());
        p.normalize();

        assertEquals(Math.abs(outputLine3.getPlane1().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane1().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane1().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane1().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputLine3.getPlane2());
        p.normalize();

        assertEquals(Math.abs(outputLine3.getPlane2().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane2().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane2().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane2().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        // Force CoincidentLinesException
        final var finalInputLine1 = inputLine1;
        final var finalInputLine3 = inputLine3;
        final var finalOutputLine1 = outputLine1;
        final var finalOutputLine3 = outputLine3;
        assertThrows(CoincidentLinesException.class, () -> transformation2.setTransformationFromLines(finalInputLine1,
                finalInputLine1, finalInputLine3, finalOutputLine1, finalOutputLine1, finalOutputLine3));
    }

    @Test
    void testConstructorFromLines() throws CoincidentLinesException, AlgebraException, CoincidentPlanesException {
        Matrix t;
        do {
            // ensure transformation matrix is invertible
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                    ProjectiveTransformation3D.HOM_COORDS, -1.0, 1.0);
            t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                    ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);
            final var norm = Utils.normF(t);
            // normalize T to increase accuracy
            t.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(t) < ProjectiveTransformation3D.HOM_COORDS);

        final var transformation1 = new ProjectiveTransformation3D(t);

        // generate 4 non coincident random planes
        Plane inputPlane1;
        Plane inputPlane2;
        Plane inputPlane3;
        Plane inputPlane4;
        Plane inputPlane5;
        Plane inputPlane6;
        Line3D inputLine1;
        Line3D inputLine2;
        Line3D inputLine3;
        Line3D outputLine1;
        Line3D outputLine2;
        Line3D outputLine3;
        // build matrix initialized to zero
        final var m = new Matrix(15, 16);
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

            params = paramsMatrix.getSubmatrixAsArray(4, 0, 4,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane5 = new Plane(params);

            params = paramsMatrix.getSubmatrixAsArray(5, 0, 5,
                    Plane.PLANE_NUMBER_PARAMS - 1);
            inputPlane6 = new Plane(params);

            inputLine3 = new Line3D(inputPlane5, inputPlane6);

            // Transform lines using transformation
            outputLine1 = transformation1.transformAndReturnNew(inputLine1);
            outputLine2 = transformation1.transformAndReturnNew(inputLine2);
            outputLine3 = transformation1.transformAndReturnNew(inputLine3);

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

            var oAiA = oA * iA;
            var oAiB = oA * iB;
            var oAiC = oA * iC;
            var oAiD = oA * iD;

            var oBiA = oB * iA;
            var oBiB = oB * iB;
            var oBiC = oB * iC;
            var oBiD = oB * iD;

            var oCiA = oC * iA;
            var oCiB = oC * iB;
            var oCiC = oC * iC;
            var oCiD = oC * iD;

            var norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(0, 0, oDiA / norm);
            m.setElementAt(0, 1, oDiB / norm);
            m.setElementAt(0, 2, oDiC / norm);
            m.setElementAt(0, 3, oDiD / norm);

            m.setElementAt(0, 12, -oAiA / norm);
            m.setElementAt(0, 13, -oAiB / norm);
            m.setElementAt(0, 14, -oAiC / norm);
            m.setElementAt(0, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(1, 4, oDiA / norm);
            m.setElementAt(1, 5, oDiB / norm);
            m.setElementAt(1, 6, oDiC / norm);
            m.setElementAt(1, 7, oDiD / norm);

            m.setElementAt(1, 12, -oBiA / norm);
            m.setElementAt(1, 13, -oBiB / norm);
            m.setElementAt(1, 14, -oBiC / norm);
            m.setElementAt(1, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(2, 8, oDiA / norm);
            m.setElementAt(2, 9, oDiB / norm);
            m.setElementAt(2, 10, oDiC / norm);
            m.setElementAt(2, 11, oDiD / norm);

            m.setElementAt(2, 12, -oCiA / norm);
            m.setElementAt(2, 13, -oCiB / norm);
            m.setElementAt(2, 14, -oCiC / norm);
            m.setElementAt(2, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(3, 0, oDiA / norm);
            m.setElementAt(3, 1, oDiB / norm);
            m.setElementAt(3, 2, oDiC / norm);
            m.setElementAt(3, 3, oDiD / norm);

            m.setElementAt(3, 12, -oAiA / norm);
            m.setElementAt(3, 13, -oAiB / norm);
            m.setElementAt(3, 14, -oAiC / norm);
            m.setElementAt(3, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(4, 4, oDiA / norm);
            m.setElementAt(4, 5, oDiB / norm);
            m.setElementAt(4, 6, oDiC / norm);
            m.setElementAt(4, 7, oDiD / norm);

            m.setElementAt(4, 12, -oBiA / norm);
            m.setElementAt(4, 13, -oBiB / norm);
            m.setElementAt(4, 14, -oBiC / norm);
            m.setElementAt(4, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(5, 8, oDiA / norm);
            m.setElementAt(5, 9, oDiB / norm);
            m.setElementAt(5, 10, oDiC / norm);
            m.setElementAt(5, 11, oDiD / norm);

            m.setElementAt(5, 12, -oCiA / norm);
            m.setElementAt(5, 13, -oCiB / norm);
            m.setElementAt(5, 14, -oCiC / norm);
            m.setElementAt(5, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(6, 0, oDiA / norm);
            m.setElementAt(6, 1, oDiB / norm);
            m.setElementAt(6, 2, oDiC / norm);
            m.setElementAt(6, 3, oDiD / norm);

            m.setElementAt(6, 12, -oAiA / norm);
            m.setElementAt(6, 13, -oAiB / norm);
            m.setElementAt(6, 14, -oAiC / norm);
            m.setElementAt(6, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(7, 4, oDiA / norm);
            m.setElementAt(7, 5, oDiB / norm);
            m.setElementAt(7, 6, oDiC / norm);
            m.setElementAt(7, 7, oDiD / norm);

            m.setElementAt(7, 12, -oBiA / norm);
            m.setElementAt(7, 13, -oBiB / norm);
            m.setElementAt(7, 14, -oBiC / norm);
            m.setElementAt(7, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(8, 8, oDiA / norm);
            m.setElementAt(8, 9, oDiB / norm);
            m.setElementAt(8, 10, oDiC / norm);
            m.setElementAt(8, 11, oDiD / norm);

            m.setElementAt(8, 12, -oCiA / norm);
            m.setElementAt(8, 13, -oCiB / norm);
            m.setElementAt(8, 14, -oCiC / norm);
            m.setElementAt(8, 15, -oCiD / norm);

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

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(9, 0, oDiA / norm);
            m.setElementAt(9, 1, oDiB / norm);
            m.setElementAt(9, 2, oDiC / norm);
            m.setElementAt(9, 3, oDiD / norm);

            m.setElementAt(9, 12, -oAiA / norm);
            m.setElementAt(9, 13, -oAiB / norm);
            m.setElementAt(9, 14, -oAiC / norm);
            m.setElementAt(9, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(10, 4, oDiA / norm);
            m.setElementAt(10, 5, oDiB / norm);
            m.setElementAt(10, 6, oDiC / norm);
            m.setElementAt(10, 7, oDiD / norm);

            m.setElementAt(10, 12, -oBiA / norm);
            m.setElementAt(10, 13, -oBiB / norm);
            m.setElementAt(10, 14, -oBiC / norm);
            m.setElementAt(10, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(11, 8, oDiA / norm);
            m.setElementAt(11, 9, oDiB / norm);
            m.setElementAt(11, 10, oDiC / norm);
            m.setElementAt(11, 11, oDiD / norm);

            m.setElementAt(11, 12, -oCiA / norm);
            m.setElementAt(11, 13, -oCiB / norm);
            m.setElementAt(11, 14, -oCiC / norm);
            m.setElementAt(11, 15, -oCiD / norm);

            // 5th pair of planes
            iA = inputLine3.getPlane1().getA();
            iB = inputLine3.getPlane1().getB();
            iC = inputLine3.getPlane1().getC();
            iD = inputLine3.getPlane1().getD();

            oA = outputLine3.getPlane1().getA();
            oB = outputLine3.getPlane1().getB();
            oC = outputLine3.getPlane1().getC();
            oD = outputLine3.getPlane1().getD();

            oDiA = oD * iA;
            oDiB = oD * iB;
            oDiC = oD * iC;
            oDiD = oD * iD;

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;
            oAiD = oA * iD;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;
            oBiD = oB * iD;

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;
            oCiD = oC * iD;

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oAiA * oAiA + oAiB * oAiB
                    + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(12, 0, oDiA / norm);
            m.setElementAt(12, 1, oDiB / norm);
            m.setElementAt(12, 2, oDiC / norm);
            m.setElementAt(12, 3, oDiD / norm);

            m.setElementAt(12, 12, -oAiA / norm);
            m.setElementAt(12, 13, -oAiB / norm);
            m.setElementAt(12, 14, -oAiC / norm);
            m.setElementAt(12, 15, -oAiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oBiA * oBiA + oBiB * oBiB
                    + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(13, 4, oDiA / norm);
            m.setElementAt(13, 5, oDiB / norm);
            m.setElementAt(13, 6, oDiC / norm);
            m.setElementAt(13, 7, oDiD / norm);

            m.setElementAt(13, 12, -oBiA / norm);
            m.setElementAt(13, 13, -oBiB / norm);
            m.setElementAt(13, 14, -oBiC / norm);
            m.setElementAt(13, 15, -oBiD / norm);

            norm = Math.sqrt(oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD + oCiA * oCiA + oCiB * oCiB
                    + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(14, 8, oDiA / norm);
            m.setElementAt(14, 9, oDiB / norm);
            m.setElementAt(14, 10, oDiC / norm);
            m.setElementAt(14, 11, oDiD / norm);

            m.setElementAt(14, 12, -oCiA / norm);
            m.setElementAt(14, 13, -oCiB / norm);
            m.setElementAt(14, 14, -oCiC / norm);
            m.setElementAt(14, 15, -oCiD / norm);

        } while (Utils.rank(m) < 8);

        // Now build another transformation
        var transformation2 = new ProjectiveTransformation3D(inputLine1, inputLine2, inputLine3, outputLine1,
                outputLine2, outputLine3);

        // check correctness of lines (up to scale)
        var p = transformation2.transformAndReturnNew(inputLine1.getPlane1());
        p.normalize();

        assertEquals(Math.abs(outputLine1.getPlane1().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane1().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane1().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane1().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputLine1.getPlane2());
        p.normalize();

        assertEquals(Math.abs(outputLine1.getPlane2().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane2().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane2().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getPlane2().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputLine2.getPlane1());
        p.normalize();

        assertEquals(Math.abs(outputLine2.getPlane1().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane1().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane1().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane1().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputLine2.getPlane2());
        p.normalize();

        assertEquals(Math.abs(outputLine2.getPlane2().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane2().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane2().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getPlane2().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputLine3.getPlane1());
        p.normalize();

        assertEquals(Math.abs(outputLine3.getPlane1().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane1().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane1().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane1().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        p = transformation2.transformAndReturnNew(inputLine3.getPlane2());
        p.normalize();

        assertEquals(Math.abs(outputLine3.getPlane2().getA()), Math.abs(p.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane2().getB()), Math.abs(p.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane2().getC()), Math.abs(p.getC()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getPlane2().getD()), Math.abs(p.getD()), LARGE_ABSOLUTE_ERROR);

        // Force CoincidentLinesException
        final var finalInputLine1 = inputLine1;
        final var finalInputLine3 = inputLine3;
        final var finalOutputLine1 = outputLine1;
        final var finalOutputLine3 = outputLine3;
        assertThrows(CoincidentLinesException.class, () -> new ProjectiveTransformation3D(finalInputLine1,
                finalInputLine1, finalInputLine3, finalOutputLine1, finalOutputLine1, finalOutputLine3));
    }

    @Test
    void testSerializeAndDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation3D.HOM_COORDS,
                ProjectiveTransformation3D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // ensure last element is not zero
        t.setElementAt(ProjectiveTransformation3D.HOM_COORDS - 1,
                ProjectiveTransformation3D.HOM_COORDS - 1, 1.0);
        final var transformation1 = new ProjectiveTransformation3D(t);

        // check
        assertSame(t, transformation1.getT());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(transformation1);
        final var transformation2 = SerializationHelper.<ProjectiveTransformation3D>deserialize(bytes);

        // check
        assertEquals(transformation1.getT(), transformation2.getT());
    }

    private static void transformPoint(
            final Point3D inputPoint, final Point3D outputPoint, final ProjectiveTransformation3D transformation)
            throws AlgebraException {
        inputPoint.normalize();
        transformation.normalize();

        final var t = transformation.asMatrix();

        final var coords = new double[Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH];
        coords[0] = inputPoint.getHomX();
        coords[1] = inputPoint.getHomY();
        coords[2] = inputPoint.getHomZ();
        coords[3] = inputPoint.getHomW();

        final var p = Matrix.newFromArray(coords, true);

        t.multiply(p);

        outputPoint.setHomogeneousCoordinates(t.getElementAtIndex(0), t.getElementAtIndex(1), t.getElementAtIndex(2),
                t.getElementAtIndex(3));
    }

    private static void transformPlane(
            final Plane inputPlane, final Plane outputPlane, final ProjectiveTransformation3D transformation)
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
            final Quadric inputQuadric, final Quadric outputQuadric, final ProjectiveTransformation3D transformation)
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
            final ProjectiveTransformation3D transformation) throws WrongSizeException, NonSymmetricMatrixException {

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
            final Line3D inputLine, final Line3D outputLine, final ProjectiveTransformation3D transformation)
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
