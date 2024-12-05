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

class ProjectiveTransformation2DTest {

    private static final int HOM_COORDS = 3;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_RANDOM_VALUE = -100.0;
    private static final double MAX_RANDOM_VALUE = 100.0;

    private static final double MIN_SCALE = 0.2;
    private static final double MAX_SCALE = 5.0;

    private static final int MIN_POINTS = 3;
    private static final int MAX_POINTS = 50;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-6;

    private static final int TIMES = 10;

    @Test
    void testConstants() {
        assertEquals(2, ProjectiveTransformation2D.NUM_TRANSLATION_COORDS);
        assertEquals(3, ProjectiveTransformation2D.NUM_PROJECTIVE_PARAMS);
        assertEquals(2, ProjectiveTransformation2D.INHOM_COORDS);
        assertEquals(3, ProjectiveTransformation2D.HOM_COORDS);
        assertEquals(1e-12, ProjectiveTransformation2D.EPS, 0.0);
    }

    @Test
    void testConstructor() throws AlgebraException {
        // Test empty constructor
        var transformation = new ProjectiveTransformation2D();

        // check correctness

        // matrix is identity up to scale
        assertTrue(transformation.asMatrix().equals(Matrix.identity(
                ProjectiveTransformation2D.HOM_COORDS, ProjectiveTransformation2D.HOM_COORDS)
                        .multiplyByScalarAndReturnNew(1.0 / Math.sqrt(ProjectiveTransformation2D.HOM_COORDS)),
                ABSOLUTE_ERROR));
        assertEquals(0.0, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewness(),
                ABSOLUTE_ERROR);
        assertTrue(transformation.getA().equals(
                transformation.getAffineParameters().asMatrix().multiplyAndReturnNew(
                        transformation.getRotation().asInhomogeneousMatrix()), ABSOLUTE_ERROR));
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        var projectiveParameters = transformation.getProjectiveParameters();
        var norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // Test constructor with T matrix
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // ensure last element is not zero
        t.setElementAt(ProjectiveTransformation2D.HOM_COORDS - 1,
                ProjectiveTransformation2D.HOM_COORDS - 1, 1.0);
        transformation = new ProjectiveTransformation2D(t);

        // check correctness
        assertNotNull(transformation.getRotation());
        assertNotNull(transformation.getTranslation());
        assertNotNull(transformation.getAffineParameters());
        assertNotNull(transformation.getProjectiveParameters());

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D((Matrix) null));

        // Force IllegalArgumentException
        final var badT = new Matrix(ProjectiveTransformation2D.HOM_COORDS + 1,
                ProjectiveTransformation2D.HOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation2D(badT));

        // Test constructor with scale
        final var randomizer = new UniformRandomizer();
        final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        transformation = new ProjectiveTransformation2D(scale);

        // check correctness
        assertEquals(0.0, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewness(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // Test constructor with rotation
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var rotation = new Rotation2D(theta);
        transformation = new ProjectiveTransformation2D(rotation);

        // check correctness
        assertEquals(transformation.getRotation().getTheta(), theta, ABSOLUTE_ERROR);
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewness(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D((Rotation2D) null));

        // Test constructor with scale and rotation
        transformation = new ProjectiveTransformation2D(scale, rotation);

        // check correctness
        assertEquals(theta, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewness(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(scale, (Rotation2D) null));

        // Test constructor with affine parameters and rotation
        final var scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        var params = new AffineParameters2D(scaleX, scaleY, skewness);
        transformation = new ProjectiveTransformation2D(params, rotation);

        // check correctness
        assertEquals(theta, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], 0.0);
        assertEquals(0.0, transformation.getTranslation()[1], 0.0);
        assertEquals(0.0, transformation.getTranslationX(), 0.0);
        assertEquals(0.0, transformation.getTranslationY(), 0.0);
        assertEquals(scaleX, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(skewness, transformation.getAffineParameters().getSkewness(), ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(null, rotation));

        // Test constructor with translation
        final var translation = new double[AffineParameters2D.INHOM_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        transformation = new ProjectiveTransformation2D(translation);

        // check correctness
        assertEquals(0.0, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewness(),
                ABSOLUTE_ERROR);
        assertNotNull(transformation.getA());
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D((double[]) null));

        final var badTranslation = new double[ProjectiveTransformation2D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation2D(badTranslation));

        // Test constructor with matrix A and translation
        final var a = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.INHOM_COORDS,
                ProjectiveTransformation2D.INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        transformation = new ProjectiveTransformation2D(a, translation);

        // check correctness
        assertNotNull(transformation.getRotation());
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertNotNull(transformation.getAffineParameters());
        assertTrue(a.equals(transformation.getA(), ABSOLUTE_ERROR));
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D((Matrix) null, translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(a, null));

        // Force IllegalArgumentException
        final var badA = new Matrix(ProjectiveTransformation2D.INHOM_COORDS + 1,
                ProjectiveTransformation2D.INHOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation2D(badA, translation));
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation2D(a, badTranslation));

        // Test constructor with scale and translation
        transformation = new ProjectiveTransformation2D(scale, translation);

        // Check correctness
        assertEquals(0.0, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewness(),
                ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(scale, (double[]) null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation2D(scale, badTranslation));

        // Test constructor with rotation and translation
        transformation = new ProjectiveTransformation2D(rotation, translation);

        // check correctness
        assertEquals(theta, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewness(),
                ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D((Rotation2D) null, translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(rotation, null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation2D(rotation, badTranslation));

        // Test constructor with scale, rotation and translation
        transformation = new ProjectiveTransformation2D(scale, rotation, translation);

        // check correctness
        assertEquals(theta, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewness(),
                ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class,
                () -> new ProjectiveTransformation2D(scale, null, translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(scale, rotation, null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new ProjectiveTransformation2D(scale, rotation, badTranslation));

        // Test constructor with scale, rotation, translation and projective
        // parameters
        final var projectiveParameters2 = new double[ProjectiveTransformation2D.HOM_COORDS];
        do {
            // repeat to ensure that last element in projective parameters is
            // large enough (we use positive values to ensure that rotations
            // don't change sign
            randomizer.fill(projectiveParameters2, 0, MAX_RANDOM_VALUE);
        } while (Math.abs(projectiveParameters2[ProjectiveTransformation2D.HOM_COORDS - 1]) < ABSOLUTE_ERROR);
        transformation = new ProjectiveTransformation2D(scale, rotation, translation, projectiveParameters2);

        // check correctness
        assertEquals(theta, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewness(),
                ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        norm = Utils.normF(projectiveParameters2);
        ArrayUtils.multiplyByScalar(projectiveParameters2, 1.0 / norm, projectiveParameters2);
        assertArrayEquals(projectiveParameters, projectiveParameters2, ABSOLUTE_ERROR);

        // Force NullPointerException
        final var finalProjectiveParameters = projectiveParameters;
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(scale, null, translation,
                finalProjectiveParameters));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(scale, rotation,
                null, finalProjectiveParameters));
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(scale, rotation,
                translation, null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation2D(scale, rotation,
                badTranslation, finalProjectiveParameters));
        final var badProjectiveParameters = new double[ProjectiveTransformation2D.HOM_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation2D(scale, rotation, translation,
                badProjectiveParameters));

        // Test constructor with affine parameters, rotation and translation
        transformation = new ProjectiveTransformation2D(params, rotation, translation);

        // check correctness
        assertEquals(theta, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(scaleX, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(skewness, transformation.getAffineParameters().getSkewness(), ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(null, rotation,
                translation));
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(params, null,
                translation));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(params, rotation,
                null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation2D(params, rotation,
                badTranslation));

        // Test constructor with affine parameters, rotation, translation and projective parameters
        transformation = new ProjectiveTransformation2D(params, rotation, translation, projectiveParameters2);

        // check correctness
        assertEquals(theta, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
        assertEquals(AffineTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
        assertEquals(scaleX, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(skewness, transformation.getAffineParameters().getSkewness(), ABSOLUTE_ERROR);
        // check projective parameters are equal up to scale
        projectiveParameters = transformation.getProjectiveParameters();
        norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        norm = Utils.normF(projectiveParameters2);
        ArrayUtils.multiplyByScalar(projectiveParameters2, 1.0 / norm, projectiveParameters2);
        assertArrayEquals(projectiveParameters, projectiveParameters2, ABSOLUTE_ERROR);

        // Force NullPointerException
        final var finalProjectiveParameter2 = projectiveParameters;
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(null, rotation,
                translation, finalProjectiveParameter2));
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(params, null,
                translation, finalProjectiveParameter2));
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(params, rotation,
                null, finalProjectiveParameter2));
        assertThrows(NullPointerException.class, () -> new ProjectiveTransformation2D(params, rotation,
                translation, null));

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation2D(params, rotation,
                badTranslation, finalProjectiveParameter2));
        assertThrows(IllegalArgumentException.class, () -> new ProjectiveTransformation2D(params, rotation,
                translation, badProjectiveParameters));
    }

    @Test
    void testGetSetT() throws WrongSizeException {
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation2D();

        // check default value (identity up to scale
        assertTrue(transformation.getT().equals(Matrix.identity(
                ProjectiveTransformation2D.HOM_COORDS, ProjectiveTransformation2D.HOM_COORDS).
                        multiplyByScalarAndReturnNew(1.0 / Math.sqrt(ProjectiveTransformation2D.HOM_COORDS)),
                ABSOLUTE_ERROR));

        // set new value
        transformation.setT(t);

        // check correctness
        assertTrue(transformation.getT().equals(t, ABSOLUTE_ERROR));

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> transformation.setT(null));

        // Force IllegalArgumentException
        final var badT = new Matrix(ProjectiveTransformation2D.HOM_COORDS + 1,
                ProjectiveTransformation2D.HOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> transformation.setT(badT));
    }

    @Test
    void testIsDegenerate() throws WrongSizeException, LockedException, NotReadyException, DecomposerException,
            NotAvailableException {
        // create non singular matrix
        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(
                    ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS,
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        assertFalse(ProjectiveTransformation2D.isDegenerate(t));

        final var transformation = new ProjectiveTransformation2D(t);
        transformation.setT(t);

        assertFalse(transformation.isDegenerate());

        // make matrix singular
        final var row = new double[ProjectiveTransformation2D.HOM_COORDS];
        Arrays.fill(row, 0.0);
        t.setSubmatrix(ProjectiveTransformation2D.HOM_COORDS - 1, 0,
                ProjectiveTransformation2D.HOM_COORDS - 1,
                ProjectiveTransformation2D.HOM_COORDS - 1, row);

        decomposer.setInputMatrix(t);
        decomposer.decompose();
        assertTrue(decomposer.isSingular());

        // check correctness
        ProjectiveTransformation2D.isDegenerate(t);

        transformation.setT(t);

        assertTrue(transformation.isDegenerate());

        // Force IllegalArgumentException
        final var wrongT = new Matrix(ProjectiveTransformation2D.HOM_COORDS + 1,
                ProjectiveTransformation2D.HOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> ProjectiveTransformation2D.isDegenerate(wrongT));
    }

    @Test
    void testGetSetA() throws WrongSizeException {
        final var a = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.INHOM_COORDS,
                ProjectiveTransformation2D.INHOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation2D();

        // check default value
        assertTrue(transformation.getA().equals(Matrix.identity(AffineParameters2D.INHOM_COORDS,
                AffineParameters2D.INHOM_COORDS), ABSOLUTE_ERROR));

        // set matrix A
        transformation.setA(a);

        // check correctness
        assertTrue(transformation.getA().equals(a, ABSOLUTE_ERROR));

        // Force NullPointerException
        assertThrows(NullPointerException.class, () -> transformation.setA(null));

        // Force IllegalArgumentException
        final var badA = new Matrix(ProjectiveTransformation2D.INHOM_COORDS + 1,
                ProjectiveTransformation2D.INHOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> transformation.setA(badA));
    }

    @Test
    void testNormalize() throws WrongSizeException {
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var norm = Utils.normF(t);
        final var normT = t.multiplyByScalarAndReturnNew(1.0 / norm);

        final var transformation = new ProjectiveTransformation2D();

        transformation.setT(t);

        assertTrue(transformation.getT().equals(t, ABSOLUTE_ERROR));

        // normalize
        transformation.normalize();

        // check equal-ness with normalized T matrix
        assertTrue(transformation.getT().equals(normT, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetRotation() throws AlgebraException {
        final var transformation = new ProjectiveTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var theta = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final var rotation = new Rotation2D(theta);

        // test default values
        assertEquals(0.0, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);

        // set new value
        transformation.setRotation(rotation);

        // check correctness
        assertEquals(theta, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);

        // Force NullPointerException
        //noinspection DataFlowIssue
        assertThrows(NullPointerException.class, () -> transformation.setRotation(null));
    }

    @Test
    void testAddRotation() throws AlgebraException {
        final var transformation = new ProjectiveTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var theta1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var theta2 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        var combinedTheta = theta1 + theta2;
        combinedTheta += Math.PI;
        final var times = (int) Math.floor(combinedTheta / (2.0 * Math.PI));
        combinedTheta -= times * 2.0 * Math.PI;
        combinedTheta -= Math.PI;

        final var rotation1 = new Rotation2D(theta1);
        final var rotation2 = new Rotation2D(theta2);

        // set rotation1
        transformation.setRotation(rotation1);

        // check correctness
        assertEquals(theta1, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);

        // add second rotation
        transformation.addRotation(rotation2);

        // check correctness
        assertEquals(combinedTheta, transformation.getRotation().getTheta(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetAffineParameters() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var scaleX = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var scaleY = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);
        final var skewness = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var params = new AffineParameters2D(scaleX, scaleY, skewness);

        // instantiate transformation
        final var transformation = new ProjectiveTransformation2D();

        // check default values
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleY(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, transformation.getAffineParameters().getSkewness(),
                ABSOLUTE_ERROR);

        final var defaultParams = new AffineParameters2D();
        transformation.getAffineParameters(defaultParams);

        assertEquals(AffineParameters2D.DEFAULT_SCALE, defaultParams.getScaleX(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, defaultParams.getScaleY(), ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SKEWNESS, defaultParams.getSkewness(), ABSOLUTE_ERROR);

        // set parameters
        transformation.setAffineParameters(params);

        // check correctness
        assertEquals(scaleX, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
        assertEquals(skewness, transformation.getAffineParameters().getSkewness(), ABSOLUTE_ERROR);

        final var params2 = new AffineParameters2D();
        transformation.getAffineParameters(params2);

        assertEquals(scaleX, params2.getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scaleY, params2.getScaleY(), ABSOLUTE_ERROR);
        assertEquals(skewness, params2.getSkewness(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetProjectiveParameters() {
        final var transformation = new ProjectiveTransformation2D();

        // check projective parameters are equal up to scale
        var projectiveParameters = transformation.getProjectiveParameters();
        final var norm = Utils.normF(projectiveParameters);
        ArrayUtils.multiplyByScalar(projectiveParameters, 1.0 / norm, projectiveParameters);
        assertArrayEquals(new double[]{0.0, 0.0, 1.0}, projectiveParameters, ABSOLUTE_ERROR);

        // set new value
        projectiveParameters = new double[ProjectiveTransformation2D.HOM_COORDS];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(projectiveParameters);

        // set new value
        transformation.setProjectiveParameters(projectiveParameters);

        // check correctness
        assertArrayEquals(transformation.getProjectiveParameters(), projectiveParameters, ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslation() {
        final var transformation = new ProjectiveTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translation = new double[AffineTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        final var translation2 = transformation.getTranslation();
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, translation2.length);
        assertArrayEquals(translation, translation2, ABSOLUTE_ERROR);
        assertEquals(translation[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation[1], transformation.getTranslationY(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var badTranslation = new double[ProjectiveTransformation2D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> transformation.setTranslation(badTranslation));
        assertThrows(WrongSizeException.class, () -> transformation.getTranslation(badTranslation));
    }

    @Test
    void testAddTranslation() {
        final var transformation = new ProjectiveTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translation1 = new double[ProjectiveTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation2 = new double[ProjectiveTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        final var translationCopy = Arrays.copyOf(translation1, ProjectiveTransformation2D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslationY(), ABSOLUTE_ERROR);

        // add translation
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslationY(), ABSOLUTE_ERROR);

        // Force IllegalArgumentException
        final var badTranslation = new double[ProjectiveTransformation2D.NUM_TRANSLATION_COORDS + 1];
        assertThrows(IllegalArgumentException.class, () -> transformation.addTranslation(badTranslation));
    }

    @Test
    void testAddTranslation2() {
        final var transformation = new ProjectiveTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translation1 = new double[ProjectiveTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation1, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation2 = new double[ProjectiveTransformation2D.NUM_TRANSLATION_COORDS];
        randomizer.fill(translation2, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        final var translationCopy = Arrays.copyOf(translation1, ProjectiveTransformation2D.NUM_TRANSLATION_COORDS);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1], transformation.getTranslationY(), ABSOLUTE_ERROR);

        // add translation
        transformation.addTranslation(translation2[0], translation2[1]);

        // check correctness
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1[0] + translation2[0], transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1[1] + translation2[1], transformation.getTranslationY(), ABSOLUTE_ERROR);
    }

    @Test
    void testAddTranslation3() {
        final var transformation = new ProjectiveTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translation1 = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        translation1.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));
        final var translation2 = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES);
        translation2.setInhomogeneousCoordinates(
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE));

        // check default value
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        final var translationCopy = new InhomogeneousPoint2D(translation1);
        transformation.setTranslation(translationCopy);

        // check correctness
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1.getInhomX(), transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomY(), transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomX(), transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomY(), transformation.getTranslationY(), ABSOLUTE_ERROR);

        // add translation
        transformation.addTranslation(translation2);

        // check correctness
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translation1.getInhomX() + translation2.getInhomX(), transformation.getTranslation()[0],
                ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomY() + translation2.getInhomY(), transformation.getTranslation()[1],
                ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomX() + translation2.getInhomX(), transformation.getTranslationX(),
                ABSOLUTE_ERROR);
        assertEquals(translation1.getInhomY() + translation2.getInhomY(), transformation.getTranslationY(),
                ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslationX() {
        final var transformation = new ProjectiveTransformation2D();

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
        final var transformation = new ProjectiveTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslationY(translationY);

        // check correctness
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);
    }

    @Test
    void testSetTranslationCoordinates() {
        final var transformation = new ProjectiveTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        // check default value
        assertEquals(AffineTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translationX, translationY);

        // check correctness
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translationX, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translationX, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);
    }

    @Test
    void testGetSetTranslationPoint() {
        final var transformation = new ProjectiveTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var translationX = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translationY = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var translation = new InhomogeneousPoint2D(translationX, translationY);

        // check default value
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(0.0, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(0.0, transformation.getTranslationY(), ABSOLUTE_ERROR);

        // set new value
        transformation.setTranslation(translation);

        // check correctness
        assertEquals(ProjectiveTransformation2D.NUM_TRANSLATION_COORDS, transformation.getTranslation().length);
        assertEquals(translationX, transformation.getTranslation()[0], ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslation()[1], ABSOLUTE_ERROR);
        assertEquals(translationX, transformation.getTranslationX(), ABSOLUTE_ERROR);
        assertEquals(translationY, transformation.getTranslationY(), ABSOLUTE_ERROR);

        final var translation2 = transformation.getTranslationPoint();
        final var translation3 = Point2D.create();
        transformation.getTranslationPoint(translation3);

        // check correctness
        assertEquals(translation, translation2);
        assertEquals(translation, translation3);
    }

    @Test
    void testAddTranslationX() {
        final var transformation = new ProjectiveTransformation2D();

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
        final var transformation = new ProjectiveTransformation2D();

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
    void testGetSetScale() throws AlgebraException {
        final var transformation = new ProjectiveTransformation2D();

        final var randomizer = new UniformRandomizer();
        final var scale = randomizer.nextDouble(MIN_SCALE, MAX_SCALE);

        // check default value
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleX(),
                ABSOLUTE_ERROR);
        assertEquals(AffineParameters2D.DEFAULT_SCALE, transformation.getAffineParameters().getScaleY(),
                ABSOLUTE_ERROR);

        // set value
        transformation.setScale(scale);

        // check correctness
        assertEquals(scale, transformation.getAffineParameters().getScaleX(), ABSOLUTE_ERROR);
        assertEquals(scale, transformation.getAffineParameters().getScaleY(), ABSOLUTE_ERROR);
    }

    @Test
    void testAsMatrix() throws WrongSizeException {
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation2D();

        transformation.setT(t);

        // check correctness
        assertTrue(transformation.asMatrix().equals(t, ABSOLUTE_ERROR));

        final var t2 = new Matrix(ProjectiveTransformation2D.HOM_COORDS, ProjectiveTransformation2D.HOM_COORDS);
        transformation.asMatrix(t2);

        assertTrue(t2.equals(t, ABSOLUTE_ERROR));

        // Force IllegalArgumentException
        final var badT = new Matrix(ProjectiveTransformation2D.HOM_COORDS + 1,
                ProjectiveTransformation2D.HOM_COORDS + 1);
        assertThrows(IllegalArgumentException.class, () -> transformation.asMatrix(badT));
    }

    @Test
    void testTransformPoint() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var t = Matrix.createWithUniformRandomValues(
                ProjectiveTransformation2D.HOM_COORDS, ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation2D(t);

        final var point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

        final var expectedPoint = Point2D.create();
        transformPoint(point, expectedPoint, transformation);

        final var outPoint1 = transformation.transformAndReturnNew(point);
        final var outPoint2 = Point2D.create();
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

        final var t = Matrix.createWithUniformRandomValues(
                ProjectiveTransformation2D.HOM_COORDS, ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE,
                MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation2D(t);

        final var inputPoints = new ArrayList<Point2D>(size);
        final var expectedPoints = new ArrayList<Point2D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, transformation);

            expectedPoints.add(expectedPoint);
        }

        final var outPoints1 = transformation.transformPointsAndReturnNew(inputPoints);
        final var outPoints2 = new ArrayList<Point2D>();
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

        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation2D(t);

        final var inputPoints = new ArrayList<Point2D>(size);
        final var expectedPoints = new ArrayList<Point2D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point2D.create();
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
    void testTransformConic() throws AlgebraException, NonSymmetricMatrixException {

        final var randomizer = new UniformRandomizer();

        // create input conic
        // Constructor with params
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var conic = new Conic(a, b, c, d, e, f);

        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation2D(t);

        // compute expected value
        final var expectedConic = new Conic();
        transformConic(conic, expectedConic, transformation);
        expectedConic.normalize();

        // make transformation
        final var outConic1 = transformation.transformAndReturnNew(conic);
        final var outConic2 = new Conic();
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
    void testTransformConicAndPoints() throws AlgebraException, GeometryException {
        // create Conic from 5 points
        Conic conic = null;
        Point2D point1;
        Point2D point2;
        Point2D point3;
        Point2D point4;
        Point2D point5;
        do {
            final var m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

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
                // if points are not valid, ignore and continue
            }
        } while (conic == null);

        // check that points belong to conic
        assertTrue(conic.isLocus(point1, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point2, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point3, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point4, ABSOLUTE_ERROR));
        assertTrue(conic.isLocus(point5, ABSOLUTE_ERROR));

        // create transformation
        Matrix t;
        final var luDecomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            luDecomposer.setInputMatrix(t);
            luDecomposer.decompose();
        } while (luDecomposer.isSingular());

        final var transformation = new ProjectiveTransformation2D(t);

        // compute expected value
        final var expectedConic = new Conic();
        transformConic(conic, expectedConic, transformation);
        expectedConic.normalize();

        // transform conic and points
        final var outConic = transformation.transformAndReturnNew(conic);
        final var outPoint1 = transformation.transformAndReturnNew(point1);
        final var outPoint2 = transformation.transformAndReturnNew(point2);
        final var outPoint3 = transformation.transformAndReturnNew(point3);
        final var outPoint4 = transformation.transformAndReturnNew(point4);
        final var outPoint5 = transformation.transformAndReturnNew(point5);

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
    void tesTransformDualConic() throws NonSymmetricMatrixException, AlgebraException {

        final var randomizer = new UniformRandomizer();

        // create input conic
        // Constructor with params
        final var a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var dualConic = new DualConic(a, b, c, d, e, f);

        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation2D(t);

        // compute expected value
        final var expectedDualConic = new DualConic();
        transformDualConic(dualConic, expectedDualConic, transformation);
        expectedDualConic.normalize();

        // make transformation
        final var outDualConic1 = transformation.transformAndReturnNew(dualConic);
        final var outDualConic2 = new DualConic();
        transformation.transform(dualConic, outDualConic2);

        // check correctness
        outDualConic1.normalize();
        outDualConic2.normalize();

        assertEquals(expectedDualConic.getA(), outDualConic1.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), outDualConic1.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), outDualConic1.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), outDualConic1.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), outDualConic1.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), outDualConic1.getF(), ABSOLUTE_ERROR);

        assertEquals(expectedDualConic.getA(), outDualConic2.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), outDualConic2.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), outDualConic2.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), outDualConic2.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), outDualConic2.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), outDualConic2.getF(), ABSOLUTE_ERROR);

        transformation.transform(dualConic);

        // check correctness
        dualConic.normalize();

        assertEquals(expectedDualConic.getA(), dualConic.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getB(), dualConic.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getC(), dualConic.getC(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getD(), dualConic.getD(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getE(), dualConic.getE(), ABSOLUTE_ERROR);
        assertEquals(expectedDualConic.getF(), dualConic.getF(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformDualConicAndLines() throws AlgebraException, GeometryException {
        // create dual conic from 5 lines
        DualConic dualConic = null;
        Line2D line1;
        Line2D line2;
        Line2D line3;
        Line2D line4;
        Line2D line5;
        do {
            final var m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
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
                // if lines are not valid, ignore and continue
            }
        } while (dualConic == null);

        // check that lines belong to dual conic
        assertTrue(dualConic.isLocus(line1, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line2, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line3, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line4, ABSOLUTE_ERROR));
        assertTrue(dualConic.isLocus(line5, ABSOLUTE_ERROR));

        // create transformation
        Matrix t;
        final var luDecomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            luDecomposer.setInputMatrix(t);
            luDecomposer.decompose();
        } while (luDecomposer.isSingular());

        final var transformation = new ProjectiveTransformation2D(t);

        // compute expected value
        final var expectedDualConic = new DualConic();
        transformDualConic(dualConic, expectedDualConic, transformation);
        expectedDualConic.normalize();

        // transform dual conic and lines
        final var outDualConic = transformation.transformAndReturnNew(dualConic);
        final var outLine1 = transformation.transformAndReturnNew(line1);
        final var outLine2 = transformation.transformAndReturnNew(line2);
        final var outLine3 = transformation.transformAndReturnNew(line3);
        final var outLine4 = transformation.transformAndReturnNew(line4);
        final var outLine5 = transformation.transformAndReturnNew(line5);

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
    void testTransformLine() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var params = new double[Line2D.LINE_NUMBER_PARAMS];
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var line = new Line2D(params);

        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation2D(t);

        final var expectedLine = new Line2D();
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();

        final var outLine1 = transformation.transformAndReturnNew(line);
        final var outLine2 = new Line2D();
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
    void testTransformLineAndPoints() throws AlgebraException {
        // create line from 2 points
        var m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var decomposer = new SingularValueDecomposer(m);
        decomposer.decompose();

        // ensure we create a matrix with 2 non-linear dependent rows
        while (decomposer.getRank() < 2) {
            m = Matrix.createWithUniformRandomValues(2, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(m);
            decomposer.decompose();
        }

        final var point1 = new HomogeneousPoint2D(m.getElementAt(0, 0),
                m.getElementAt(0, 1),
                m.getElementAt(0, 2));
        final var point2 = new HomogeneousPoint2D(m.getElementAt(1, 0),
                m.getElementAt(1, 1),
                m.getElementAt(1, 2));

        point1.normalize();
        point2.normalize();

        final var line = new Line2D(point1, point2);

        // check that points belong to the line
        assertTrue(line.isLocus(point1));
        assertTrue(line.isLocus(point2));

        // create transformation
        Matrix t;
        final var luDecomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(
                    ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE,
                    MAX_RANDOM_VALUE);
            luDecomposer.setInputMatrix(t);
            luDecomposer.decompose();
        } while (luDecomposer.isSingular());

        final var transformation = new ProjectiveTransformation2D(t);

        final var expectedLine = new Line2D();
        transformLine(line, expectedLine, transformation);
        expectedLine.normalize();

        // transform line and points
        final var outLine = transformation.transformAndReturnNew(line);
        final var outPoint1 = transformation.transformAndReturnNew(point1);
        final var outPoint2 = transformation.transformAndReturnNew(point2);

        // check that transformed points still belong to transformed line
        assertTrue(outLine.isLocus(outPoint1, ABSOLUTE_ERROR));
        assertTrue(outLine.isLocus(outPoint2, ABSOLUTE_ERROR));

        // check line correctness
        outLine.normalize();

        assertEquals(expectedLine.getA(), outLine.getA(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getB(), outLine.getB(), ABSOLUTE_ERROR);
        assertEquals(expectedLine.getC(), outLine.getC(), ABSOLUTE_ERROR);
    }

    @Test
    void testTransformLines() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation2D(t);

        final var inputLines = new ArrayList<Line2D>(size);
        final var expectedLines = new ArrayList<Line2D>(size);
        for (var i = 0; i < size; i++) {
            final var params = new double[Line2D.LINE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var line = new Line2D(params);
            inputLines.add(line);

            final var expectedLine = new Line2D();
            transformLine(line, expectedLine, transformation);

            expectedLines.add(expectedLine);
        }


        final var outLines1 = transformation.transformLinesAndReturnNew(inputLines);
        final var outLines2 = new ArrayList<Line2D>();
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
            assertEquals(expectedLine.getA(), outLine1.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), outLine1.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), outLine1.getC(), ABSOLUTE_ERROR);

            assertEquals(expectedLine.getA(), outLine2.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), outLine2.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), outLine2.getC(), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testTransformAndOverwriteLines() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation2D(t);

        final var inputLines = new ArrayList<Line2D>(size);
        final var expectedLines = new ArrayList<Line2D>(size);
        for (var i = 0; i < size; i++) {
            final var params = new double[Line2D.LINE_NUMBER_PARAMS];
            randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var line = new Line2D(params);
            inputLines.add(line);

            final var expectedLine = new Line2D();
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
            assertEquals(expectedLine.getA(), line.getA(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getB(), line.getB(), ABSOLUTE_ERROR);
            assertEquals(expectedLine.getC(), line.getC(), ABSOLUTE_ERROR);
        }
    }

    @Test
    void testTransformPolygon() throws NotEnoughVerticesException, AlgebraException {

        final var randomizer = new UniformRandomizer();
        final var size = randomizer.nextInt(MIN_POINTS, MAX_POINTS);

        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation2D(t);

        final var inputPoints = new ArrayList<Point2D>(size);
        final var expectedPoints = new ArrayList<Point2D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, transformation);

            expectedPoints.add(expectedPoint);
        }

        final var inputPolygon = new Polygon2D(inputPoints);
        final var expectedPolygon = new Polygon2D(expectedPoints);

        final var outPolygon1 = transformation.transformAndReturnNew(inputPolygon);
        final var outPolygon2 = new Polygon2D(inputPoints);
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
    }

    @Test
    void testTransformTriangle() throws AlgebraException {
        final var randomizer = new UniformRandomizer();
        final var size = Triangle2D.NUM_VERTICES;

        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation = new ProjectiveTransformation2D(t);

        final var inputPoints = new ArrayList<Point2D>(size);
        final var expectedPoints = new ArrayList<Point2D>(size);
        for (var i = 0; i < size; i++) {
            final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
            randomizer.fill(coords, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            final var point = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);
            inputPoints.add(point);

            final var expectedPoint = Point2D.create();
            transformPoint(point, expectedPoint, transformation);

            expectedPoints.add(expectedPoint);
        }

        final var inputTriangle = new Triangle2D(inputPoints.get(0), inputPoints.get(1), inputPoints.get(2));
        final var expectedTriangle = new Triangle2D(expectedPoints.get(0), expectedPoints.get(1),
                expectedPoints.get(2));

        final var outTriangle1 = transformation.transformAndReturnNew(inputTriangle);
        final var outTriangle2 = new Triangle2D(
                new InhomogeneousPoint2D(inputPoints.get(0)),
                new InhomogeneousPoint2D(inputPoints.get(1)),
                new InhomogeneousPoint2D(inputPoints.get(2)));
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
    void testInverse() throws AlgebraException {
        // generate invertible matrix
        Matrix t;
        final var decomposer = new LUDecomposer();
        do {
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            decomposer.setInputMatrix(t);
            decomposer.decompose();
        } while (decomposer.isSingular());

        final var transformation = new ProjectiveTransformation2D(t);

        final var invTransformation1 = transformation.inverseAndReturnNew();
        final var invTransformation2 = new ProjectiveTransformation2D();
        transformation.inverse(invTransformation2);

        // check that inverse transformation matrix is the inverse matrix of
        // current transformation
        assertTrue(invTransformation1.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(MetricTransformation2D.HOM_COORDS,
                MetricTransformation2D.HOM_COORDS), ABSOLUTE_ERROR));

        assertTrue(invTransformation2.asMatrix().multiplyAndReturnNew(
                transformation.asMatrix()).equals(Matrix.identity(MetricTransformation2D.HOM_COORDS,
                MetricTransformation2D.HOM_COORDS), ABSOLUTE_ERROR));

        // test transforming a random point by transformation and then by its
        // inverse to ensure it remains the same
        final var params = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        final var randomizer = new UniformRandomizer();
        randomizer.fill(params, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var inputPoint = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, params);

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

        // Force AlgebraException
        // by using non-invertible matrix
        final var row = new double[ProjectiveTransformation2D.HOM_COORDS];
        Arrays.fill(row, 0.0);
        t.setSubmatrix(ProjectiveTransformation2D.HOM_COORDS - 1, 0,
                ProjectiveTransformation2D.HOM_COORDS - 1,
                ProjectiveTransformation2D.HOM_COORDS - 1, row);
        transformation.setT(t);
        assertThrows(AlgebraException.class, transformation::inverse);
        assertThrows(AlgebraException.class, transformation::inverseAndReturnNew);
        assertThrows(AlgebraException.class, () -> transformation.inverse(invTransformation2));
    }

    @Test
    void testCombine() throws AlgebraException {
        final var t1 = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        final var t2 = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        final var transformation1 = new ProjectiveTransformation2D(t1);

        final var transformation2 = new ProjectiveTransformation2D(t2);

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

        var numValid = 0;
        for (var times = 0; times < TIMES; times++) {

            Matrix t;
            do {
                // ensure transformation matrix is invertible
                t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                        ProjectiveTransformation2D.HOM_COORDS, -1.0, 1.0);
                t.setElementAt(ProjectiveTransformation2D.HOM_COORDS - 1,
                        ProjectiveTransformation2D.HOM_COORDS - 1, 1.0);
                final var norm = Utils.normF(t);
                // normalize T to increase accuracy
                t.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(t) < ProjectiveTransformation2D.HOM_COORDS);

            final var transformation1 = new ProjectiveTransformation2D(t);

            // generate 4 non coincident random points
            Point2D inputPoint1;
            Point2D inputPoint2;
            Point2D inputPoint3;
            Point2D inputPoint4;
            Point2D outputPoint1;
            Point2D outputPoint2;
            Point2D outputPoint3;
            Point2D outputPoint4;
            // build matrix initialized to zero
            final var m = new Matrix(8, 9);
            do {
                final var coordsMatrix = Matrix.createWithUniformRandomValues(4, 2, -1.0,
                        1.0);

                var coords = coordsMatrix.getSubmatrixAsArray(0, 0, 0,
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint1 = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

                coords = coordsMatrix.getSubmatrixAsArray(1, 0, 1,
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint2 = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

                coords = coordsMatrix.getSubmatrixAsArray(2, 0, 2,
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint3 = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

                coords = coordsMatrix.getSubmatrixAsArray(3, 0, 3,
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint4 = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

                // Transform points using transformation
                outputPoint1 = transformation1.transformAndReturnNew(inputPoint1);
                outputPoint2 = transformation1.transformAndReturnNew(inputPoint2);
                outputPoint3 = transformation1.transformAndReturnNew(inputPoint3);
                outputPoint4 = transformation1.transformAndReturnNew(inputPoint4);

                // 1st pair of points
                var iX = inputPoint1.getHomX();
                var iY = inputPoint1.getHomY();
                var iW = inputPoint1.getHomW();

                var oX = outputPoint1.getHomX();
                var oY = outputPoint1.getHomY();
                var oW = outputPoint1.getHomW();

                var oWiX = oW * iX;
                var oWiY = oW * iY;
                var oWiW = oW * iW;

                var oXiX = oX * iX;
                var oXiY = oX * iY;
                var oXiW = oX * iW;

                var oYiX = oY * iX;
                var oYiY = oY * iY;
                var oYiW = oY * iW;

                var norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

                m.setElementAt(0, 0, oWiX / norm);
                m.setElementAt(0, 1, oWiY / norm);
                m.setElementAt(0, 2, oWiW / norm);

                m.setElementAt(0, 6, -oXiX / norm);
                m.setElementAt(0, 7, -oXiY / norm);
                m.setElementAt(0, 8, -oXiW / norm);

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

                m.setElementAt(1, 3, oWiX / norm);
                m.setElementAt(1, 4, oWiY / norm);
                m.setElementAt(1, 5, oWiW / norm);

                m.setElementAt(1, 6, -oYiX / norm);
                m.setElementAt(1, 7, -oYiY / norm);
                m.setElementAt(1, 8, -oYiW / norm);

                // 2nd pair of points
                iX = inputPoint2.getHomX();
                iY = inputPoint2.getHomY();
                iW = inputPoint2.getHomW();

                oX = outputPoint2.getHomX();
                oY = outputPoint2.getHomY();
                oW = outputPoint2.getHomW();

                oWiX = oW * iX;
                oWiY = oW * iY;
                oWiW = oW * iW;

                oXiX = oX * iX;
                oXiY = oX * iY;
                oXiW = oX * iW;

                oYiX = oY * iX;
                oYiY = oY * iY;
                oYiW = oY * iW;

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

                m.setElementAt(2, 0, oWiX / norm);
                m.setElementAt(2, 1, oWiY / norm);
                m.setElementAt(2, 2, oWiW / norm);

                m.setElementAt(2, 6, -oXiX / norm);
                m.setElementAt(2, 7, -oXiY / norm);
                m.setElementAt(2, 8, -oXiW / norm);

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

                m.setElementAt(3, 3, oWiX / norm);
                m.setElementAt(3, 4, oWiY / norm);
                m.setElementAt(3, 5, oWiW / norm);

                m.setElementAt(3, 6, -oYiX / norm);
                m.setElementAt(3, 7, -oYiY / norm);
                m.setElementAt(3, 8, -oYiW / norm);

                // 3rd pair of points
                iX = inputPoint3.getHomX();
                iY = inputPoint3.getHomY();
                iW = inputPoint3.getHomW();

                oX = outputPoint3.getHomX();
                oY = outputPoint3.getHomY();
                oW = outputPoint3.getHomW();

                oWiX = oW * iX;
                oWiY = oW * iY;
                oWiW = oW * iW;

                oXiX = oX * iX;
                oXiY = oX * iY;
                oXiW = oX * iW;

                oYiX = oY * iX;
                oYiY = oY * iY;
                oYiW = oY * iW;

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

                m.setElementAt(4, 0, oWiX / norm);
                m.setElementAt(4, 1, oWiY / norm);
                m.setElementAt(4, 2, oWiW / norm);

                m.setElementAt(4, 6, -oXiX / norm);
                m.setElementAt(4, 7, -oXiY / norm);
                m.setElementAt(4, 8, -oXiW / norm);

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

                m.setElementAt(5, 3, oWiX / norm);
                m.setElementAt(5, 4, oWiY / norm);
                m.setElementAt(5, 5, oWiW / norm);

                m.setElementAt(5, 6, -oYiX / norm);
                m.setElementAt(5, 7, -oYiY / norm);
                m.setElementAt(5, 8, -oYiW / norm);

                // 4th pair of points
                iX = inputPoint4.getHomX();
                iY = inputPoint4.getHomY();
                iW = inputPoint4.getHomW();

                oX = outputPoint4.getHomX();
                oY = outputPoint4.getHomY();
                oW = outputPoint4.getHomW();

                oWiX = oW * iX;
                oWiY = oW * iY;
                oWiW = oW * iW;

                oXiX = oX * iX;
                oXiY = oX * iY;
                oXiW = oX * iW;

                oYiX = oY * iX;
                oYiY = oY * iY;
                oYiW = oY * iW;

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

                m.setElementAt(6, 0, oWiX / norm);
                m.setElementAt(6, 1, oWiY / norm);
                m.setElementAt(6, 2, oWiW / norm);

                m.setElementAt(6, 6, -oXiX / norm);
                m.setElementAt(6, 7, -oXiY / norm);
                m.setElementAt(6, 8, -oXiW / norm);

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

                m.setElementAt(7, 3, oWiX / norm);
                m.setElementAt(7, 4, oWiY / norm);
                m.setElementAt(7, 5, oWiW / norm);

                m.setElementAt(7, 6, -oYiX / norm);
                m.setElementAt(7, 7, -oYiY / norm);
                m.setElementAt(7, 8, -oYiW / norm);
            } while (Utils.rank(m) < 8);


            // Now build another transformation
            final var transformation2 = new ProjectiveTransformation2D();

            // estimate transformation from corresponding points
            transformation2.setTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3, inputPoint4,
                    outputPoint1, outputPoint2, outputPoint3, outputPoint4);

            // check correctness of transformation by checking transformed points
            if (!outputPoint1.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint1)),
                    LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(outputPoint1.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint1)),
                    LARGE_ABSOLUTE_ERROR));
            if (!outputPoint2.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint2)),
                    LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(outputPoint2.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint2)),
                    LARGE_ABSOLUTE_ERROR));
            if (!outputPoint3.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint3)),
                    LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(outputPoint3.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint3)),
                    LARGE_ABSOLUTE_ERROR));
            if (!outputPoint4.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint4)),
                    LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(outputPoint4.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint4)),
                    LARGE_ABSOLUTE_ERROR));

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

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testConstructorFromPoints() throws WrongSizeException, DecomposerException, CoincidentPointsException {

        for (var times = 0; times < TIMES; times++) {

            Matrix t;
            do {
                // ensure transformation matrix is invertible
                t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                        ProjectiveTransformation2D.HOM_COORDS, -1.0, 1.0);
                t.setElementAt(ProjectiveTransformation2D.HOM_COORDS - 1,
                        ProjectiveTransformation2D.HOM_COORDS - 1, 1.0);
                final var norm = Utils.normF(t);
                // normalize T to increase accuracy
                t.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(t) < ProjectiveTransformation2D.HOM_COORDS);

            final var transformation1 = new ProjectiveTransformation2D(t);

            // generate 4 non coincident random points
            Point2D inputPoint1;
            Point2D inputPoint2;
            Point2D inputPoint3;
            Point2D inputPoint4;
            Point2D outputPoint1;
            Point2D outputPoint2;
            Point2D outputPoint3;
            Point2D outputPoint4;
            // build matrix initialized to zero
            final var m = new Matrix(8, 9);
            do {
                final var coordsMatrix = Matrix.createWithUniformRandomValues(4, 2,
                        -1.0, 1.0);

                var coords = coordsMatrix.getSubmatrixAsArray(0, 0, 0,
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint1 = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

                coords = coordsMatrix.getSubmatrixAsArray(1, 0, 1,
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint2 = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

                coords = coordsMatrix.getSubmatrixAsArray(2, 0, 2,
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint3 = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

                coords = coordsMatrix.getSubmatrixAsArray(3, 0, 3,
                        Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH - 1);
                inputPoint4 = Point2D.create(CoordinatesType.INHOMOGENEOUS_COORDINATES, coords);

                // Transform points using transformation
                outputPoint1 = transformation1.transformAndReturnNew(inputPoint1);
                outputPoint2 = transformation1.transformAndReturnNew(inputPoint2);
                outputPoint3 = transformation1.transformAndReturnNew(inputPoint3);
                outputPoint4 = transformation1.transformAndReturnNew(inputPoint4);

                // 1st pair of points
                var iX = inputPoint1.getHomX();
                var iY = inputPoint1.getHomY();
                var iW = inputPoint1.getHomW();

                var oX = outputPoint1.getHomX();
                var oY = outputPoint1.getHomY();
                var oW = outputPoint1.getHomW();

                var oWiX = oW * iX;
                var oWiY = oW * iY;
                var oWiW = oW * iW;

                var oXiX = oX * iX;
                var oXiY = oX * iY;
                var oXiW = oX * iW;

                var oYiX = oY * iX;
                var oYiY = oY * iY;
                var oYiW = oY * iW;

                var norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

                m.setElementAt(0, 0, oWiX / norm);
                m.setElementAt(0, 1, oWiY / norm);
                m.setElementAt(0, 2, oWiW / norm);

                m.setElementAt(0, 6, -oXiX / norm);
                m.setElementAt(0, 7, -oXiY / norm);
                m.setElementAt(0, 8, -oXiW / norm);

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

                m.setElementAt(1, 3, oWiX / norm);
                m.setElementAt(1, 4, oWiY / norm);
                m.setElementAt(1, 5, oWiW / norm);

                m.setElementAt(1, 6, -oYiX / norm);
                m.setElementAt(1, 7, -oYiY / norm);
                m.setElementAt(1, 8, -oYiW / norm);

                // 2nd pair of points
                iX = inputPoint2.getHomX();
                iY = inputPoint2.getHomY();
                iW = inputPoint2.getHomW();

                oX = outputPoint2.getHomX();
                oY = outputPoint2.getHomY();
                oW = outputPoint2.getHomW();

                oWiX = oW * iX;
                oWiY = oW * iY;
                oWiW = oW * iW;

                oXiX = oX * iX;
                oXiY = oX * iY;
                oXiW = oX * iW;

                oYiX = oY * iX;
                oYiY = oY * iY;
                oYiW = oY * iW;

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

                m.setElementAt(2, 0, oWiX / norm);
                m.setElementAt(2, 1, oWiY / norm);
                m.setElementAt(2, 2, oWiW / norm);

                m.setElementAt(2, 6, -oXiX / norm);
                m.setElementAt(2, 7, -oXiY / norm);
                m.setElementAt(2, 8, -oXiW / norm);

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

                m.setElementAt(3, 3, oWiX / norm);
                m.setElementAt(3, 4, oWiY / norm);
                m.setElementAt(3, 5, oWiW / norm);

                m.setElementAt(3, 6, -oYiX / norm);
                m.setElementAt(3, 7, -oYiY / norm);
                m.setElementAt(3, 8, -oYiW / norm);

                // 3rd pair of points
                iX = inputPoint3.getHomX();
                iY = inputPoint3.getHomY();
                iW = inputPoint3.getHomW();

                oX = outputPoint3.getHomX();
                oY = outputPoint3.getHomY();
                oW = outputPoint3.getHomW();

                oWiX = oW * iX;
                oWiY = oW * iY;
                oWiW = oW * iW;

                oXiX = oX * iX;
                oXiY = oX * iY;
                oXiW = oX * iW;

                oYiX = oY * iX;
                oYiY = oY * iY;
                oYiW = oY * iW;

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

                m.setElementAt(4, 0, oWiX / norm);
                m.setElementAt(4, 1, oWiY / norm);
                m.setElementAt(4, 2, oWiW / norm);

                m.setElementAt(4, 6, -oXiX / norm);
                m.setElementAt(4, 7, -oXiY / norm);
                m.setElementAt(4, 8, -oXiW / norm);

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

                m.setElementAt(5, 3, oWiX / norm);
                m.setElementAt(5, 4, oWiY / norm);
                m.setElementAt(5, 5, oWiW / norm);

                m.setElementAt(5, 6, -oYiX / norm);
                m.setElementAt(5, 7, -oYiY / norm);
                m.setElementAt(5, 8, -oYiW / norm);

                // 4th pair of points
                iX = inputPoint4.getHomX();
                iY = inputPoint4.getHomY();
                iW = inputPoint4.getHomW();

                oX = outputPoint4.getHomX();
                oY = outputPoint4.getHomY();
                oW = outputPoint4.getHomW();

                oWiX = oW * iX;
                oWiY = oW * iY;
                oWiW = oW * iW;

                oXiX = oX * iX;
                oXiY = oX * iY;
                oXiW = oX * iW;

                oYiX = oY * iX;
                oYiY = oY * iY;
                oYiW = oY * iW;

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oXiX * oXiX + oXiY * oXiY + oXiW * oXiW);

                m.setElementAt(6, 0, oWiX / norm);
                m.setElementAt(6, 1, oWiY / norm);
                m.setElementAt(6, 2, oWiW / norm);

                m.setElementAt(6, 6, -oXiX / norm);
                m.setElementAt(6, 7, -oXiY / norm);
                m.setElementAt(6, 8, -oXiW / norm);

                norm = Math.sqrt(oWiX * oWiX + oWiY * oWiY + oWiW * oWiW + oYiX * oYiX + oYiY * oYiY + oYiW * oYiW);

                m.setElementAt(7, 3, oWiX / norm);
                m.setElementAt(7, 4, oWiY / norm);
                m.setElementAt(7, 5, oWiW / norm);

                m.setElementAt(7, 6, -oYiX / norm);
                m.setElementAt(7, 7, -oYiY / norm);
                m.setElementAt(7, 8, -oYiW / norm);
            } while (Utils.rank(m) < 8);


            // Now build another transformation
            var transformation2 = new ProjectiveTransformation2D(inputPoint1, inputPoint2, inputPoint3, inputPoint4,
                    outputPoint1, outputPoint2, outputPoint3, outputPoint4);

            // check correctness of transformation by checking transformed points
            assertTrue(outputPoint1.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint1)),
                    LARGE_ABSOLUTE_ERROR));
            assertTrue(outputPoint2.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint2)),
                    LARGE_ABSOLUTE_ERROR));
            assertTrue(outputPoint3.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint3)),
                    LARGE_ABSOLUTE_ERROR));
            assertTrue(outputPoint4.equals(new InhomogeneousPoint2D(transformation2.transformAndReturnNew(inputPoint4)),
                    LARGE_ABSOLUTE_ERROR));

            // Force CoincidentPointsException
            final var finalInputPoint1 = inputPoint1;
            final var finalInputPoint3 = inputPoint3;
            final var finalInputPoint4 = inputPoint4;
            final var finalOutputPoint1 = outputPoint1;
            final var finalOutputPoint3 = outputPoint3;
            final var finalOutputPoint4 = outputPoint4;
            assertThrows(CoincidentPointsException.class, () -> new ProjectiveTransformation2D(finalInputPoint1,
                    finalInputPoint1, finalInputPoint3, finalInputPoint4, finalOutputPoint1, finalOutputPoint1,
                    finalOutputPoint3, finalOutputPoint4));
        }
    }

    @Test
    void testSetTransformationFromLines() throws CoincidentLinesException, AlgebraException {

        Matrix t;
        do {
            // ensure transformation matrix is invertible
            t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                    ProjectiveTransformation2D.HOM_COORDS, -1.0, 1.0);
            t.setElementAt(ProjectiveTransformation2D.HOM_COORDS - 1,
                    ProjectiveTransformation2D.HOM_COORDS - 1, 1.0);
            final var norm = Utils.normF(t);
            // normalize T to increase accuracy
            t.multiplyByScalar(1.0 / norm);
        } while (Utils.rank(t) < ProjectiveTransformation2D.HOM_COORDS);

        final var transformation1 = new ProjectiveTransformation2D(t);

        // generate 4 non coincident random lines
        Line2D inputLine1;
        Line2D inputLine2;
        Line2D inputLine3;
        Line2D inputLine4;
        Line2D outputLine1;
        Line2D outputLine2;
        Line2D outputLine3;
        Line2D outputLine4;
        // build matrix initialized to zero
        final var m = new Matrix(8, 9);
        do {
            final var coordsMatrix = Matrix.createWithUniformRandomValues(4, 3,
                    -1.0, 1.0);

            var coords = coordsMatrix.getSubmatrixAsArray(0, 0, 0,
                    Line2D.LINE_NUMBER_PARAMS - 1);
            inputLine1 = new Line2D(coords);

            coords = coordsMatrix.getSubmatrixAsArray(1, 0, 1,
                    Line2D.LINE_NUMBER_PARAMS - 1);
            inputLine2 = new Line2D(coords);

            coords = coordsMatrix.getSubmatrixAsArray(2, 0, 2,
                    Line2D.LINE_NUMBER_PARAMS - 1);
            inputLine3 = new Line2D(coords);

            coords = coordsMatrix.getSubmatrixAsArray(3, 0, 3,
                    Line2D.LINE_NUMBER_PARAMS - 1);
            inputLine4 = new Line2D(coords);

            // Transform points using transformation
            outputLine1 = transformation1.transformAndReturnNew(inputLine1);
            outputLine2 = transformation1.transformAndReturnNew(inputLine2);
            outputLine3 = transformation1.transformAndReturnNew(inputLine3);
            outputLine4 = transformation1.transformAndReturnNew(inputLine4);

            outputLine1.normalize();
            outputLine2.normalize();
            outputLine3.normalize();
            outputLine4.normalize();

            // 1st pair of lines
            var iA = inputLine1.getA();
            var iB = inputLine1.getB();
            var iC = inputLine1.getC();

            var oA = outputLine1.getA();
            var oB = outputLine1.getB();
            var oC = outputLine1.getC();

            var oCiA = oC * iA;
            var oCiB = oC * iB;
            var oCiC = oC * iC;

            var oAiA = oA * iA;
            var oAiB = oA * iB;
            var oAiC = oA * iC;

            var oBiA = oB * iA;
            var oBiB = oB * iB;
            var oBiC = oB * iC;

            var norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

            m.setElementAt(0, 0, oCiA / norm);
            m.setElementAt(0, 1, oCiB / norm);
            m.setElementAt(0, 2, oCiC / norm);

            m.setElementAt(0, 6, -oAiA / norm);
            m.setElementAt(0, 7, -oAiB / norm);
            m.setElementAt(0, 8, -oAiC / norm);

            norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

            m.setElementAt(1, 3, oCiA / norm);
            m.setElementAt(1, 4, oCiB / norm);
            m.setElementAt(1, 5, oCiC / norm);

            m.setElementAt(1, 6, -oBiA / norm);
            m.setElementAt(1, 7, -oBiB / norm);
            m.setElementAt(1, 8, -oBiC / norm);

            // 2nd pair of points
            iA = inputLine2.getA();
            iB = inputLine2.getB();
            iC = inputLine2.getC();

            oA = outputLine2.getA();
            oB = outputLine2.getB();
            oC = outputLine2.getC();

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;

            norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

            m.setElementAt(2, 0, oCiA / norm);
            m.setElementAt(2, 1, oCiB / norm);
            m.setElementAt(2, 2, oCiC / norm);

            m.setElementAt(2, 6, -oAiA / norm);
            m.setElementAt(2, 7, -oAiB / norm);
            m.setElementAt(2, 8, -oAiC / norm);

            norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

            m.setElementAt(3, 3, oCiA / norm);
            m.setElementAt(3, 4, oCiB / norm);
            m.setElementAt(3, 5, oCiC / norm);

            m.setElementAt(3, 6, -oBiA / norm);
            m.setElementAt(3, 7, -oBiB / norm);
            m.setElementAt(3, 8, -oBiC / norm);

            // 3rd pair of points
            iA = inputLine3.getA();
            iB = inputLine3.getB();
            iC = inputLine3.getC();

            oA = outputLine3.getA();
            oB = outputLine3.getB();
            oC = outputLine3.getC();

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;

            norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

            m.setElementAt(4, 0, oCiA / norm);
            m.setElementAt(4, 1, oCiB / norm);
            m.setElementAt(4, 2, oCiC / norm);

            m.setElementAt(4, 6, -oAiA / norm);
            m.setElementAt(4, 7, -oAiB / norm);
            m.setElementAt(4, 8, -oAiC / norm);

            norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

            m.setElementAt(5, 3, oCiA / norm);
            m.setElementAt(5, 4, oCiB / norm);
            m.setElementAt(5, 5, oCiC / norm);

            m.setElementAt(5, 6, -oBiA / norm);
            m.setElementAt(5, 7, -oBiB / norm);
            m.setElementAt(5, 8, -oBiC / norm);

            // 4th pair of points
            iA = inputLine4.getA();
            iB = inputLine4.getB();
            iC = inputLine4.getC();

            oA = outputLine4.getA();
            oB = outputLine4.getB();
            oC = outputLine4.getC();

            oCiA = oC * iA;
            oCiB = oC * iB;
            oCiC = oC * iC;

            oAiA = oA * iA;
            oAiB = oA * iB;
            oAiC = oA * iC;

            oBiA = oB * iA;
            oBiB = oB * iB;
            oBiC = oB * iC;

            norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

            m.setElementAt(6, 0, oCiA / norm);
            m.setElementAt(6, 1, oCiB / norm);
            m.setElementAt(6, 2, oCiC / norm);

            m.setElementAt(6, 6, -oAiA / norm);
            m.setElementAt(6, 7, -oAiB / norm);
            m.setElementAt(6, 8, -oAiC / norm);

            norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

            m.setElementAt(7, 3, oCiA / norm);
            m.setElementAt(7, 4, oCiB / norm);
            m.setElementAt(7, 5, oCiC / norm);

            m.setElementAt(7, 6, -oBiA / norm);
            m.setElementAt(7, 7, -oBiB / norm);
            m.setElementAt(7, 8, -oBiC / norm);
        } while (Utils.rank(m) < 8);

        // Now build another transformation
        final var transformation2 = new ProjectiveTransformation2D();

        // estimate transformation from corresponding points
        transformation2.setTransformationFromLines(inputLine1, inputLine2, inputLine3, inputLine4, outputLine1,
                outputLine2, outputLine3, outputLine4);

        // check correctness of lines (up to scale)
        var l = transformation2.transformAndReturnNew(inputLine1);
        l.normalize();

        assertEquals(Math.abs(outputLine1.getA()), Math.abs(l.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getB()), Math.abs(l.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine1.getC()), Math.abs(l.getC()), LARGE_ABSOLUTE_ERROR);

        l = transformation2.transformAndReturnNew(inputLine2);
        l.normalize();

        assertEquals(Math.abs(outputLine2.getA()), Math.abs(l.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getB()), Math.abs(l.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine2.getC()), Math.abs(l.getC()), LARGE_ABSOLUTE_ERROR);

        l = transformation2.transformAndReturnNew(inputLine3);
        l.normalize();

        assertEquals(Math.abs(outputLine3.getA()), Math.abs(l.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getB()), Math.abs(l.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine3.getC()), Math.abs(l.getC()), LARGE_ABSOLUTE_ERROR);

        l = transformation2.transformAndReturnNew(inputLine4);
        l.normalize();

        assertEquals(Math.abs(outputLine4.getA()), Math.abs(l.getA()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine4.getB()), Math.abs(l.getB()), LARGE_ABSOLUTE_ERROR);
        assertEquals(Math.abs(outputLine4.getC()), Math.abs(l.getC()), LARGE_ABSOLUTE_ERROR);

        // Force CoincidentLinesException
        final var finalInputLine1 = inputLine1;
        final var finalInputLine3 = inputLine3;
        final var finalInputLine4 = inputLine4;
        final var finalOutputLine1 = outputLine1;
        final var finalOutputLine3 = outputLine3;
        final var finalOutputLine4 = outputLine4;
        assertThrows(CoincidentLinesException.class, () -> transformation2.setTransformationFromLines(finalInputLine1,
                finalInputLine1, finalInputLine3, finalInputLine4, finalOutputLine1, finalOutputLine1, finalOutputLine3,
                finalOutputLine4));
    }

    @Test
    void testConstructorFromLines() throws CoincidentLinesException, AlgebraException {

        var numValid = 0;
        for (var times = 0; times < TIMES; times++) {

            Matrix t;
            do {
                // ensure transformation matrix is invertible
                t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                        ProjectiveTransformation2D.HOM_COORDS, -1.0, 1.0);
                t.setElementAt(ProjectiveTransformation2D.HOM_COORDS - 1,
                        ProjectiveTransformation2D.HOM_COORDS - 1, 1.0);
                final var norm = Utils.normF(t);
                // normalize T to increase accuracy
                t.multiplyByScalar(1.0 / norm);
            } while (Utils.rank(t) < ProjectiveTransformation2D.HOM_COORDS);

            final var transformation1 = new ProjectiveTransformation2D(t);

            // generate 4 non coincident random lines
            Line2D inputLine1;
            Line2D inputLine2;
            Line2D inputLine3;
            Line2D inputLine4;
            Line2D outputLine1;
            Line2D outputLine2;
            Line2D outputLine3;
            Line2D outputLine4;
            // build matrix initialized to zero
            final var m = new Matrix(8, 9);
            do {
                final var coordsMatrix = Matrix.createWithUniformRandomValues(4, 3,
                        -1.0, 1.0);

                var coords = coordsMatrix.getSubmatrixAsArray(0, 0, 0,
                        Line2D.LINE_NUMBER_PARAMS - 1);
                inputLine1 = new Line2D(coords);

                coords = coordsMatrix.getSubmatrixAsArray(1, 0, 1,
                        Line2D.LINE_NUMBER_PARAMS - 1);
                inputLine2 = new Line2D(coords);

                coords = coordsMatrix.getSubmatrixAsArray(2, 0, 2,
                        Line2D.LINE_NUMBER_PARAMS - 1);
                inputLine3 = new Line2D(coords);

                coords = coordsMatrix.getSubmatrixAsArray(3, 0, 3,
                        Line2D.LINE_NUMBER_PARAMS - 1);
                inputLine4 = new Line2D(coords);

                // Transform points using transformation
                outputLine1 = transformation1.transformAndReturnNew(inputLine1);
                outputLine2 = transformation1.transformAndReturnNew(inputLine2);
                outputLine3 = transformation1.transformAndReturnNew(inputLine3);
                outputLine4 = transformation1.transformAndReturnNew(inputLine4);

                outputLine1.normalize();
                outputLine2.normalize();
                outputLine3.normalize();
                outputLine4.normalize();

                // 1st pair of lines
                var iA = inputLine1.getA();
                var iB = inputLine1.getB();
                var iC = inputLine1.getC();

                var oA = outputLine1.getA();
                var oB = outputLine1.getB();
                var oC = outputLine1.getC();

                var oCiA = oC * iA;
                var oCiB = oC * iB;
                var oCiC = oC * iC;

                var oAiA = oA * iA;
                var oAiB = oA * iB;
                var oAiC = oA * iC;

                var oBiA = oB * iA;
                var oBiB = oB * iB;
                var oBiC = oB * iC;

                var norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

                m.setElementAt(0, 0, oCiA / norm);
                m.setElementAt(0, 1, oCiB / norm);
                m.setElementAt(0, 2, oCiC / norm);

                m.setElementAt(0, 6, -oAiA / norm);
                m.setElementAt(0, 7, -oAiB / norm);
                m.setElementAt(0, 8, -oAiC / norm);

                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

                m.setElementAt(1, 3, oCiA / norm);
                m.setElementAt(1, 4, oCiB / norm);
                m.setElementAt(1, 5, oCiC / norm);

                m.setElementAt(1, 6, -oBiA / norm);
                m.setElementAt(1, 7, -oBiB / norm);
                m.setElementAt(1, 8, -oBiC / norm);

                // 2nd pair of points
                iA = inputLine2.getA();
                iB = inputLine2.getB();
                iC = inputLine2.getC();

                oA = outputLine2.getA();
                oB = outputLine2.getB();
                oC = outputLine2.getC();

                oCiA = oC * iA;
                oCiB = oC * iB;
                oCiC = oC * iC;

                oAiA = oA * iA;
                oAiB = oA * iB;
                oAiC = oA * iC;

                oBiA = oB * iA;
                oBiB = oB * iB;
                oBiC = oB * iC;

                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

                m.setElementAt(2, 0, oCiA / norm);
                m.setElementAt(2, 1, oCiB / norm);
                m.setElementAt(2, 2, oCiC / norm);

                m.setElementAt(2, 6, -oAiA / norm);
                m.setElementAt(2, 7, -oAiB / norm);
                m.setElementAt(2, 8, -oAiC / norm);

                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

                m.setElementAt(3, 3, oCiA / norm);
                m.setElementAt(3, 4, oCiB / norm);
                m.setElementAt(3, 5, oCiC / norm);

                m.setElementAt(3, 6, -oBiA / norm);
                m.setElementAt(3, 7, -oBiB / norm);
                m.setElementAt(3, 8, -oBiC / norm);

                // 3rd pair of points
                iA = inputLine3.getA();
                iB = inputLine3.getB();
                iC = inputLine3.getC();

                oA = outputLine3.getA();
                oB = outputLine3.getB();
                oC = outputLine3.getC();

                oCiA = oC * iA;
                oCiB = oC * iB;
                oCiC = oC * iC;

                oAiA = oA * iA;
                oAiB = oA * iB;
                oAiC = oA * iC;

                oBiA = oB * iA;
                oBiB = oB * iB;
                oBiC = oB * iC;

                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

                m.setElementAt(4, 0, oCiA / norm);
                m.setElementAt(4, 1, oCiB / norm);
                m.setElementAt(4, 2, oCiC / norm);

                m.setElementAt(4, 6, -oAiA / norm);
                m.setElementAt(4, 7, -oAiB / norm);
                m.setElementAt(4, 8, -oAiC / norm);

                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

                m.setElementAt(5, 3, oCiA / norm);
                m.setElementAt(5, 4, oCiB / norm);
                m.setElementAt(5, 5, oCiC / norm);

                m.setElementAt(5, 6, -oBiA / norm);
                m.setElementAt(5, 7, -oBiB / norm);
                m.setElementAt(5, 8, -oBiC / norm);

                // 4th pair of points
                iA = inputLine4.getA();
                iB = inputLine4.getB();
                iC = inputLine4.getC();

                oA = outputLine4.getA();
                oB = outputLine4.getB();
                oC = outputLine4.getC();

                oCiA = oC * iA;
                oCiB = oC * iB;
                oCiC = oC * iC;

                oAiA = oA * iA;
                oAiB = oA * iB;
                oAiC = oA * iC;

                oBiA = oB * iA;
                oBiB = oB * iB;
                oBiC = oB * iC;

                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC);

                m.setElementAt(6, 0, oCiA / norm);
                m.setElementAt(6, 1, oCiB / norm);
                m.setElementAt(6, 2, oCiC / norm);

                m.setElementAt(6, 6, -oAiA / norm);
                m.setElementAt(6, 7, -oAiB / norm);
                m.setElementAt(6, 8, -oAiC / norm);

                norm = Math.sqrt(oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC);

                m.setElementAt(7, 3, oCiA / norm);
                m.setElementAt(7, 4, oCiB / norm);
                m.setElementAt(7, 5, oCiC / norm);

                m.setElementAt(7, 6, -oBiA / norm);
                m.setElementAt(7, 7, -oBiB / norm);
                m.setElementAt(7, 8, -oBiC / norm);
            } while (Utils.rank(m) < 8);


            // Now build another transformation
            var transformation2 = new ProjectiveTransformation2D(inputLine1, inputLine2, inputLine3, inputLine4,
                    outputLine1, outputLine2, outputLine3, outputLine4);

            // check correctness of lines (up to scale)
            var l = transformation2.transformAndReturnNew(inputLine1);
            l.normalize();

            if (Math.abs(Math.abs(outputLine1.getA()) - Math.abs(l.getA())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine1.getA()), Math.abs(l.getA()), ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(outputLine1.getB()) - Math.abs(l.getB())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine1.getB()), Math.abs(l.getB()), ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(outputLine1.getC()) - Math.abs(l.getC())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine1.getC()), Math.abs(l.getC()), ABSOLUTE_ERROR);

            l = transformation2.transformAndReturnNew(inputLine2);
            l.normalize();

            if (Math.abs(Math.abs(outputLine2.getA()) - Math.abs(l.getA())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine2.getA()), Math.abs(l.getA()), ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(outputLine2.getB()) - Math.abs(l.getB())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine2.getB()), Math.abs(l.getB()), ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(outputLine2.getC()) - Math.abs(l.getC())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine2.getC()), Math.abs(l.getC()), ABSOLUTE_ERROR);

            l = transformation2.transformAndReturnNew(inputLine3);
            l.normalize();

            if (Math.abs(Math.abs(outputLine3.getA()) - Math.abs(l.getA())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine3.getA()), Math.abs(l.getA()), ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(outputLine3.getB()) - Math.abs(l.getB())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine3.getB()), Math.abs(l.getB()), ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(outputLine3.getC()) - Math.abs(l.getC())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine3.getC()), Math.abs(l.getC()), ABSOLUTE_ERROR);

            l = transformation2.transformAndReturnNew(inputLine4);
            l.normalize();

            if (Math.abs(Math.abs(outputLine4.getA()) - Math.abs(l.getA())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine4.getA()), Math.abs(l.getA()), ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(outputLine4.getB()) - Math.abs(l.getB())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine4.getB()), Math.abs(l.getB()), ABSOLUTE_ERROR);
            if (Math.abs(Math.abs(outputLine4.getC()) - Math.abs(l.getC())) > ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(Math.abs(outputLine4.getC()), Math.abs(l.getC()), ABSOLUTE_ERROR);

            // Force CoincidentLinesException
            final var finalInputLine1 = inputLine1;
            final var finalInputLine3 = inputLine3;
            final var finalInputLine4 = inputLine4;
            final var finalOutputLine1 = outputLine1;
            final var finalOutputLine3 = outputLine3;
            final var finalOutputLine4 = outputLine4;
            assertThrows(CoincidentLinesException.class, () -> new ProjectiveTransformation2D(finalInputLine1,
                    finalInputLine1, finalInputLine3, finalInputLine4, finalOutputLine1, finalOutputLine1,
                    finalOutputLine3, finalOutputLine4));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSerializeAndDeserialize() throws WrongSizeException, IOException, ClassNotFoundException {
        final var t = Matrix.createWithUniformRandomValues(ProjectiveTransformation2D.HOM_COORDS,
                ProjectiveTransformation2D.HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        // ensure last element is not zero
        t.setElementAt(ProjectiveTransformation2D.HOM_COORDS - 1, ProjectiveTransformation2D.HOM_COORDS - 1,
                1.0);
        final var transformation1 = new ProjectiveTransformation2D(t);

        // check
        assertSame(t, transformation1.getT());

        // serialize and deserialize
        final var bytes = SerializationHelper.serialize(transformation1);
        final var transformation2 = SerializationHelper.<ProjectiveTransformation2D>deserialize(bytes);

        // check
        assertEquals(transformation1.getT(), transformation2.getT());
    }

    private static void transformPoint(
            final Point2D inputPoint, final Point2D outputPoint, final ProjectiveTransformation2D transformation)
            throws AlgebraException {

        inputPoint.normalize();
        transformation.normalize();

        final var t = transformation.asMatrix();

        final var coords = new double[Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH];
        coords[0] = inputPoint.getHomX();
        coords[1] = inputPoint.getHomY();
        coords[2] = inputPoint.getHomW();

        final var p = Matrix.newFromArray(coords, true);

        t.multiply(p);

        outputPoint.setHomogeneousCoordinates(t.getElementAtIndex(0), t.getElementAtIndex(1), t.getElementAtIndex(2));
    }

    private static void transformLine(
            final Line2D inputLine, final Line2D outputLine, final ProjectiveTransformation2D transformation)
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException {

        inputLine.normalize();
        transformation.normalize();

        final var t = transformation.asMatrix();
        var norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final var invT = Utils.inverse(t);
        norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final var transInvT = invT.transposeAndReturnNew();
        final var l = Matrix.newFromArray(inputLine.asArray(), true);

        outputLine.setParameters(transInvT.multiplyAndReturnNew(l).toArray());
    }

    private static void transformConic(
            final Conic inputConic, final Conic outputConic, final ProjectiveTransformation2D transformation)
            throws AlgebraException, NonSymmetricMatrixException {

        transformation.normalize();
        final var t = transformation.asMatrix();
        final var invT = Utils.inverse(t);
        var norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);
        final var transInvT = invT.transposeAndReturnNew();

        inputConic.normalize();
        final var c = inputConic.asMatrix();

        final var transC = transInvT.multiplyAndReturnNew(c.multiplyAndReturnNew(invT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transC);
        transC.multiplyByScalar(1.0 / norm);

        outputConic.setParameters(transC);
    }

    private static void transformDualConic(
            final DualConic inputDualConic, final DualConic outputDualConic,
            final ProjectiveTransformation2D transformation) throws WrongSizeException, NonSymmetricMatrixException {

        transformation.normalize();
        final var t = transformation.asMatrix();
        var norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final var transT = t.transposeAndReturnNew();

        inputDualConic.normalize();
        final var dualC = inputDualConic.asMatrix();

        final var transDualC = t.multiplyAndReturnNew(dualC.multiplyAndReturnNew(transT));
        // normalize to increase accuracy to ensure that matrix remains symmetric
        norm = Utils.normF(transDualC);
        transDualC.multiplyByScalar(1.0 / norm);

        outputDualConic.setParameters(transDualC);
    }
}
