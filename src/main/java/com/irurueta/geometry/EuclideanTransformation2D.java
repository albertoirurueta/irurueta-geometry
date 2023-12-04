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

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.estimators.EuclideanTransformation2DEstimator;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * This class performs Euclidean transformations on 2D space.
 * Euclidean transformations include transformations related to rotations and
 * translations.
 * Scale cannot be modified on Euclidean scale.
 */
@SuppressWarnings("DuplicatedCode")
public class EuclideanTransformation2D extends Transformation2D
        implements Serializable {

    /**
     * Constant indicating number of coordinates required in translation arrays.
     */
    public static final int NUM_TRANSLATION_COORDS = 2;

    /**
     * Constant defining number of homogeneous coordinates in 2D space.
     */
    public static final int HOM_COORDS = 3;

    /**
     * 2D rotation to be performed on geometric objects.
     */
    private Rotation2D rotation;

    /**
     * 2D translation to be performed on geometric objects.
     * Translation is specified using inhomogeneous coordinates.
     */
    private double[] translation;

    /**
     * Empty constructor.
     * Creates transformation that has no effect.
     */
    public EuclideanTransformation2D() {
        rotation = new Rotation2D();
        translation = new double[NUM_TRANSLATION_COORDS];
    }

    /**
     * Creates transformation with provided rotation.
     *
     * @param rotation a 2D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public EuclideanTransformation2D(final Rotation2D rotation) {
        if (rotation == null) {
            throw new NullPointerException();
        }

        this.rotation = rotation;
        translation = new double[NUM_TRANSLATION_COORDS];
    }

    /**
     * Creates transformation with provided 2D translation.
     *
     * @param translation Array indicating 2D translation using inhomogeneous
     *                    coordinates.
     * @throws NullPointerException     raised if provided array is null.
     * @throws IllegalArgumentException raised if length of array is not equal
     *                                  to NUM_TRANSLATION_COORDS.
     */
    public EuclideanTransformation2D(final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        rotation = new Rotation2D();
        this.translation = translation;
    }

    /**
     * Creates transformation with provided 2D rotation and translation.
     *
     * @param rotation    a 2D rotation.
     * @param translation array indicating 2D translation using inhomogeneous
     *                    coordinates.
     * @throws NullPointerException     raised if provided array is null.
     * @throws IllegalArgumentException Raised if length of array is not equal
     *                                  to NUM_TRANSLATION_COORDS.
     */
    public EuclideanTransformation2D(final Rotation2D rotation, final double[] translation) {

        if (rotation == null) {
            throw new NullPointerException();
        }
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        this.rotation = rotation;
        this.translation = translation;
    }

    /**
     * Creates transformation by estimating its internal values using provided 3
     * corresponding original and transformed points.
     *
     * @param inputPoint1  1st input point.
     * @param inputPoint2  2nd input point.
     * @param inputPoint3  3rd input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input
     *                     point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input
     *                     point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     *                     point.
     * @throws CoincidentPointsException raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    public EuclideanTransformation2D(
            final Point2D inputPoint1, final Point2D inputPoint2,
            final Point2D inputPoint3, final Point2D outputPoint1,
            final Point2D outputPoint2, final Point2D outputPoint3)
            throws CoincidentPointsException {
        internalSetTransformationFromPoints(inputPoint1, inputPoint2,
                inputPoint3, outputPoint1, outputPoint2, outputPoint3);
    }

    /**
     * Returns 2D rotation assigned to this transformation.
     *
     * @return 2D rotation.
     */
    public Rotation2D getRotation() {
        return rotation;
    }

    /**
     * Sets 2D rotation for this transformation.
     *
     * @param rotation a 2D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public void setRotation(final Rotation2D rotation) {
        if (rotation == null) {
            throw new NullPointerException();
        }
        this.rotation = rotation;
    }

    /**
     * Adds provided rotation to current rotation assigned to this
     * transformation.
     *
     * @param rotation 2D rotation to be added.
     */
    public void addRotation(final Rotation2D rotation) {
        this.rotation.combine(rotation);
    }

    /**
     * Returns 2D translation assigned to this transformation as an array
     * expressed in inhomogeneous coordinates.
     *
     * @return 2D translation array.
     */
    public double[] getTranslation() {
        return translation;
    }

    /**
     * Sets 2D translation assigned to this transformation as an array expressed
     * in inhomogeneous coordinates.
     *
     * @param translation 2D translation array.
     * @throws IllegalArgumentException Raised if provided array does not have
     *                                  length equal to NUM_TRANSLATION_COORDS.
     */
    public void setTranslation(final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        this.translation = translation;
    }

    /**
     * Adds provided translation to current translation on this transformation.
     * Provided translation must be expressed as an array of inhomogeneous
     * coordinates.
     *
     * @param translation 2D translation array.
     * @throws IllegalArgumentException Raised if provided array does not have
     *                                  length equal to NUM_TRANSLATION_COORDS.
     */
    public void addTranslation(final double[] translation) {
        ArrayUtils.sum(this.translation, translation, this.translation);
    }

    /**
     * Returns current x coordinate translation assigned to this transformation.
     *
     * @return X coordinate translation.
     */
    public double getTranslationX() {
        return translation[0];
    }

    /**
     * Sets x coordinate translation to be made by this transformation.
     *
     * @param translationX X coordinate translation to be set.
     */
    public void setTranslationX(final double translationX) {
        translation[0] = translationX;
    }

    /**
     * Returns current y coordinate translation assigned to this transformation.
     *
     * @return Y coordinate translation.
     */
    public double getTranslationY() {
        return translation[1];
    }

    /**
     * Sets y coordinate translation to be made by this transformation.
     *
     * @param translationY Y coordinate translation to be set.
     */
    public void setTranslationY(final double translationY) {
        translation[1] = translationY;
    }

    /**
     * Sets x, y coordinates of translation to be made by this transformation.
     *
     * @param translationX translation x coordinate to be set.
     * @param translationY translation y coordinate to be set.
     */
    public void setTranslation(final double translationX, final double translationY) {
        translation[0] = translationX;
        translation[1] = translationY;
    }

    /**
     * Sets x, y coordinates of translation to be made by this transformation.
     *
     * @param translation translation to be set.
     */
    public void setTranslation(final Point2D translation) {
        setTranslation(translation.getInhomX(), translation.getInhomY());
    }

    /**
     * Gets x, y coordinates of translation to be made by this transformation
     * as a new point.
     *
     * @return a new point containing translation coordinates.
     */
    public Point2D getTranslationPoint() {
        final Point2D out = Point2D.create();
        getTranslationPoint(out);
        return out;
    }

    /**
     * Gets x, y coordinates of translation to be made by this transformation
     * and stores them into provided point.
     *
     * @param out point where translation coordinates will be stored.
     */
    public void getTranslationPoint(final Point2D out) {
        out.setInhomogeneousCoordinates(translation[0], translation[1]);
    }

    /**
     * Adds provided x coordinate to current translation assigned to this
     * transformation.
     *
     * @param translationX X coordinate to be added to current translation.
     */
    public void addTranslationX(final double translationX) {
        translation[0] += translationX;
    }

    /**
     * Adds provided y coordinate to current translation assigned to this
     * transformation.
     *
     * @param translationY Y coordinate to be added to current translation.
     */
    public void addTranslationY(final double translationY) {
        translation[1] += translationY;
    }

    /**
     * Adds provided coordinates to current translation assigned to this
     * transformation.
     *
     * @param translationX x coordinate to be added to current translation.
     * @param translationY y coordinate to be added to current translation.
     */
    public void addTranslation(final double translationX, final double translationY) {
        translation[0] += translationX;
        translation[1] += translationY;
    }

    /**
     * Adds provided coordinates to current translation assigned to this
     * transformation.
     *
     * @param translation x, y coordinates to be added to current translation.
     */
    public void addTranslation(final Point2D translation) {
        addTranslation(translation.getInhomX(), translation.getInhomY());
    }

    /**
     * Represents this transformation as a 3x3 matrix.
     * A point can be transformed as T * p, where T is the transformation matrix
     * and p is a point expressed as an homogeneous vector.
     *
     * @return This transformation in matrix form.
     */
    @Override
    public Matrix asMatrix() {
        Matrix m = null;
        try {
            m = new Matrix(HOM_COORDS, HOM_COORDS);
            asMatrix(m);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        return m;
    }

    /**
     * Represents this transformation as a 3x3 matrix and stores the result in
     * provided instance.
     *
     * @param m instance where transformation matrix will be stored.
     * @throws IllegalArgumentException raised if provided instance is not a 3x3
     *                                  matrix.
     */
    @Override
    public void asMatrix(final Matrix m) {
        if (m.getRows() != HOM_COORDS || m.getColumns() != HOM_COORDS) {
            throw new IllegalArgumentException();
        }

        m.initialize(0.0);

        // set rotation
        m.setSubmatrix(0, 0,
                Rotation2D.ROTATION2D_INHOM_MATRIX_ROWS - 1,
                Rotation2D.ROTATION2D_INHOM_MATRIX_COLS - 1,
                rotation.asInhomogeneousMatrix());

        // set translation
        m.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                HOM_COORDS - 1, translation);

        // set last element
        m.setElementAt(HOM_COORDS - 1, HOM_COORDS - 1, 1.0);
    }

    /**
     * Transforms input point using this transformation and stores the result in
     * provided output points.
     *
     * @param inputPoint  point to be transformed.
     * @param outputPoint instance where transformed point data will be stored.
     */
    @Override
    public void transform(final Point2D inputPoint, final Point2D outputPoint) {

        inputPoint.normalize();
        rotation.rotate(inputPoint, outputPoint);
        outputPoint.setInhomogeneousCoordinates(
                outputPoint.getInhomX() + translation[0],
                outputPoint.getInhomY() + translation[1]);
    }

    /**
     * Transforms a conic using this transformation and stores the result into
     * provided output conic.
     *
     * @param inputConic  conic to be transformed.
     * @param outputConic instance where data of transformed conic will be
     *                    stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output conic matrix is not considered to be symmetric.
     */
    @Override
    public void transform(final Conic inputConic, final Conic outputConic)
            throws NonSymmetricMatrixException {
        // point' * conic * point = 0
        // point' * T' * transformedConic * T * point = 0
        // where:
        // - transformedPoint = T * point

        // Hence:
        // transformedConic = T^-1' * conic * T^-1

        inputConic.normalize();

        final Matrix c = inputConic.asMatrix();
        final Matrix invT = inverseAndReturnNew().asMatrix();
        // normalize transformation matrix T to increase accuracy
        double norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);

        final Matrix m = invT.transposeAndReturnNew();
        try {
            m.multiply(c);
            m.multiply(invT);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        // normalize resulting m matrix to increase accuracy so that it can be
        // considered symmetric
        norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);

        outputConic.setParameters(m);
    }

    /**
     * Transforms a dual conic using this transformation and stores the result
     * into provided output dual conic.
     *
     * @param inputDualConic  dual conic to be transformed.
     * @param outputDualConic instance where data of transformed dual conic will
     *                        be stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output dual conic matrix is not considered to be symmetric.
     */
    @Override
    public void transform(final DualConic inputDualConic, final DualConic outputDualConic)
            throws NonSymmetricMatrixException {
        // line' * dualConic * line = 0
        // line' * T^-1 * T * dualConic * T' * T^-1'* line

        // Hence:
        // transformed plane: T^-1'* line
        // transformed dual quadric: T * dualQuadric * T'

        inputDualConic.normalize();

        final Matrix dualC = inputDualConic.asMatrix();
        final Matrix t = asMatrix();
        // normalize transformation matrix T to increase accuracy
        double norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final Matrix transT = t.transposeAndReturnNew();
        try {
            t.multiply(dualC);
            t.multiply(transT);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        // normalize resulting m matrix to increase accuracy so that it can be
        // considered symmetric
        norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        outputDualConic.setParameters(t);
    }

    /**
     * Transforms provided input line using this transformation and stores the
     * result into provided output line instance.
     *
     * @param inputLine  line to be transformed.
     * @param outputLine instance where data of transformed line will be stored.
     */
    @Override
    public void transform(final Line2D inputLine, final Line2D outputLine) {
        // line' * point = 0 --> line' * T^-1 * T * point
        // (line' * T^-1)*(T*point) = (T^-1'*line)'*(T*point)
        // where:
        // - transformedLine = T^-1'*line
        // - transformedPoint = T*point

        inputLine.normalize();

        final Matrix invT = inverseAndReturnNew().asMatrix();
        final Matrix l = Matrix.newFromArray(inputLine.asArray());

        // normalize transformation matrix T to increase accuracy
        final double norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);

        invT.transpose();
        try {
            invT.multiply(l);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        outputLine.setParameters(invT.toArray());
    }

    /**
     * Converts this transformation into a metric transformation.
     *
     * @return this transformation converted into a metric transformation.
     */
    public MetricTransformation2D toMetric() {
        return new MetricTransformation2D(rotation, translation,
                MetricTransformation2D.DEFAULT_SCALE);
    }

    /**
     * Inverses this transformation.
     */
    public void inverse() {
        inverse(this);
    }

    /**
     * Computes the inverse of this transformation and returns the result as a
     * new transformation instance.
     *
     * @return inverse transformation.
     */
    public Transformation2D inverseAndReturnNew() {
        final EuclideanTransformation2D result = new EuclideanTransformation2D();
        inverse(result);
        return result;
    }

    /**
     * Combines this transformation with provided transformation.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix of provided transformation.
     *
     * @param transformation transformation to be combined with.
     */
    public void combine(final EuclideanTransformation2D transformation) {
        combine(transformation, this);
    }

    /**
     * Combines this transformation with provided transformation and returns
     * the result as a new transformation instance.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix of provided transformation.
     *
     * @param transformation transformation to be combined with.
     * @return a new transformation resulting of the combination with this
     * transformation and provided transformation.
     */
    public EuclideanTransformation2D combineAndReturnNew(
            final EuclideanTransformation2D transformation) {

        final EuclideanTransformation2D result = new EuclideanTransformation2D();
        combine(transformation, result);
        return result;
    }

    /**
     * Estimates this transformation internal parameters by using 3
     * corresponding original and transformed points.
     *
     * @param inputPoint1  1st input point.
     * @param inputPoint2  2nd input point.
     * @param inputPoint3  3rd input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input
     *                     point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input
     *                     point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     *                     point.
     * @throws CoincidentPointsException raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    public void setTransformationFromPoints(
            final Point2D inputPoint1, final Point2D inputPoint2,
            final Point2D inputPoint3, final Point2D outputPoint1,
            final Point2D outputPoint2, final Point2D outputPoint3)
            throws CoincidentPointsException {
        internalSetTransformationFromPoints(inputPoint1, inputPoint2,
                inputPoint3, outputPoint1, outputPoint2, outputPoint3);
    }

    /**
     * Computes the inverse of this transformation and stores the result in
     * provided instance.
     *
     * @param result instance where inverse transformation will be stored.
     */
    protected void inverse(final EuclideanTransformation2D result) {
        // Transformation is as follows: x' = R* x + t
        // Then inverse transformation is: R* x' = R' * R * x + R'*t = x + R'*t
        // --> x = R'*x' - R'*t

        // reverse rotation
        result.rotation = rotation.inverseRotation();

        // reverse translation
        final Matrix t = Matrix.newFromArray(translation, true);
        t.multiplyByScalar(-1.0);
        final Matrix invRot = result.rotation.asInhomogeneousMatrix();
        try {
            invRot.multiply(t);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        result.translation = invRot.toArray();
    }

    /**
     * Combines this transformation with provided input transformation and
     * stores the result into provided output transformation.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix of provided input transformation.
     *
     * @param inputTransformation  transformation to be combined with.
     * @param outputTransformation transformation where result will be stored.
     */
    private void combine(final EuclideanTransformation2D inputTransformation,
                         final EuclideanTransformation2D outputTransformation) {
        // combination in matrix representation is:
        // [R1 t1] * [R2 t2] = [R1*R2 + t1*0T  R1*t2 + t1*1] = [R1*R2 R1*t2 + t1]
        // [0T 1 ]   [0T 1 ]   [0T*R2 + 1*0T   0T*t2 + 1*1 ]   [0T    1         ]

        try {
            // we do translation first, because this.rotation might change later
            final Matrix r1 = this.rotation.asInhomogeneousMatrix();
            final Matrix t2 = Matrix.newFromArray(inputTransformation.translation,
                    true);
            // this is R1 * t2
            r1.multiply(t2);

            ArrayUtils.sum(r1.toArray(), this.translation,
                    outputTransformation.translation);

            outputTransformation.rotation = this.rotation.combineAndReturnNew(
                    inputTransformation.rotation);

        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Estimates this transformation internal parameters by using 3
     * corresponding original and transformed points.
     *
     * @param inputPoint1  1st input point.
     * @param inputPoint2  2nd input point.
     * @param inputPoint3  3rd input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input
     *                     point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input
     *                     point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     *                     point.
     * @throws CoincidentPointsException raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    private void internalSetTransformationFromPoints(
            final Point2D inputPoint1, final Point2D inputPoint2,
            final Point2D inputPoint3, final Point2D outputPoint1,
            final Point2D outputPoint2, final Point2D outputPoint3)
            throws CoincidentPointsException {
        final List<Point2D> inputPoints = new ArrayList<>();
        inputPoints.add(inputPoint1);
        inputPoints.add(inputPoint2);
        inputPoints.add(inputPoint3);

        final List<Point2D> outputPoints = new ArrayList<>();
        outputPoints.add(outputPoint1);
        outputPoints.add(outputPoint2);
        outputPoints.add(outputPoint3);

        final EuclideanTransformation2DEstimator estimator =
                new EuclideanTransformation2DEstimator(inputPoints,
                        outputPoints);

        try {
            estimator.estimate(this);
        } catch (final LockedException | NotReadyException ignore) {
            // never thrown
        }
    }
}
