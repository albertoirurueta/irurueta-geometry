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
import com.irurueta.geometry.estimators.EuclideanTransformation3DEstimator;
import com.irurueta.geometry.estimators.LockedException;
import com.irurueta.geometry.estimators.NotReadyException;

import java.io.Serializable;
import java.util.ArrayList;

/**
 * This class performs Euclidean transformations on 3D space.
 * Euclidean transformations include transformations related to rotations and
 * translations.
 * Scale cannot be modified on Euclidean transformation.
 */
@SuppressWarnings("DuplicatedCode")
public class EuclideanTransformation3D extends Transformation3D implements Serializable {

    /**
     * Constant indicating number of coordinates required in translation arrays.
     */
    public static final int NUM_TRANSLATION_COORDS = 3;

    /**
     * Constant defining number of homogeneous coordinates in 3D space.
     */
    public static final int HOM_COORDS = 4;

    /**
     * 3D rotation to be performed on geometric objects.
     */
    private Rotation3D rotation;

    /**
     * 3D translation to be performed on geometric objects.
     * Translation is specified using inhomogeneous coordinates.
     */
    private double[] translation;

    /**
     * Empty constructor.
     * Creates transformation that has no effect.
     */
    public EuclideanTransformation3D() {
        rotation = Rotation3D.create();
        translation = new double[NUM_TRANSLATION_COORDS];
    }

    /**
     * Creates transformation with provided rotation.
     *
     * @param rotation A 2D rotation.
     * @throws NullPointerException Raised if provided rotation is null.
     */
    public EuclideanTransformation3D(final Rotation3D rotation) {
        if (rotation == null) {
            throw new NullPointerException();
        }

        this.rotation = rotation;
        translation = new double[NUM_TRANSLATION_COORDS];
    }

    /**
     * Creates transformation with provided 3D translation.
     *
     * @param translation Array indicating 3D translation using in-homogenous
     *                    coordinates.
     * @throws NullPointerException     Raised if provided array is null.
     * @throws IllegalArgumentException Raised if length of array is not equal
     *                                  to NUM_TRANSLATION_COORDS.
     */
    public EuclideanTransformation3D(final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        rotation = Rotation3D.create();
        this.translation = translation;
    }

    /**
     * Creates transformation with provided 3D rotation and translation.
     *
     * @param rotation    A 3D rotation.
     * @param translation Array indicating 3D translation using inhomogeneous
     *                    coordinates.
     * @throws NullPointerException     Raised if provided array is null.
     * @throws IllegalArgumentException Raised if length of array is not equal
     *                                  to NUM_TRANSLATION_COORDS.
     */
    public EuclideanTransformation3D(final Rotation3D rotation, final double[] translation) {
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
     * Creates transformation by estimating its internal values using provided 4
     * corresponding original and transformed points.
     *
     * @param inputPoint1  1st input point.
     * @param inputPoint2  2nd input point.
     * @param inputPoint3  3rd input point.
     * @param inputPoint4  4th input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input
     *                     point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input
     *                     point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     *                     point.
     * @param outputPoint4 4th transformed point corresponding to 4th input
     *                     point.
     * @throws CoincidentPointsException raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    public EuclideanTransformation3D(
            final Point3D inputPoint1, final Point3D inputPoint2, final Point3D inputPoint3, final Point3D inputPoint4,
            final Point3D outputPoint1, final Point3D outputPoint2, final Point3D outputPoint3,
            final Point3D outputPoint4) throws CoincidentPointsException {
        internalSetTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3, inputPoint4, outputPoint1,
                outputPoint2, outputPoint3, outputPoint4);
    }

    /**
     * Returns 3D rotation assigned to this transformation.
     *
     * @return 3D rotation.
     */
    public Rotation3D getRotation() {
        return rotation;
    }

    /**
     * Sets 3D rotation for this transformation.
     *
     * @param rotation A 3D rotation.
     * @throws NullPointerException Raised if provided rotation is null.
     */
    public void setRotation(final Rotation3D rotation) {
        if (rotation == null) {
            throw new NullPointerException();
        }
        this.rotation = rotation;
    }

    /**
     * Adds provided rotation to current rotation assigned to this
     * transformation.
     *
     * @param rotation 3D rotation to be added.
     */
    public void addRotation(final Rotation3D rotation) {
        this.rotation.combine(rotation);
    }

    /**
     * Returns 3D translation assigned to this transformation as an array
     * expressed in inhomogeneous coordinates.
     *
     * @return 3D translation array.
     */
    public double[] getTranslation() {
        return translation;
    }

    /**
     * Sets 3D translation assigned to this transformation as an array expressed
     * in inhomogeneous coordinates.
     *
     * @param translation 3D translation array.
     * @throws IllegalArgumentException raised if provided array does not have
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
     * @param translation 3D translation array.
     * @throws IllegalArgumentException raised if provided array does not have
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
     * Returns current z coordinate translation assigned to this transformation.
     *
     * @return Z coordinate translation.
     */
    public double getTranslationZ() {
        return translation[2];
    }

    /**
     * Sets z coordinate translation to be made by this transformation.
     *
     * @param translationZ Z coordinate translation to be set.
     */
    public void setTranslationZ(final double translationZ) {
        translation[2] = translationZ;
    }

    /**
     * Sets x, y, z coordinates of translation to be made by this
     * transformation.
     *
     * @param translationX translation x coordinate to be set.
     * @param translationY translation y coordinate to be set.
     * @param translationZ translation z coordinate to be set.
     */
    public void setTranslation(
            final double translationX, final double translationY, final double translationZ) {
        translation[0] = translationX;
        translation[1] = translationY;
        translation[2] = translationZ;
    }

    /**
     * Sets x, y, z coordinates of translation to be made by this
     * transformation.
     *
     * @param translation translation to be set.
     */
    public void setTranslation(final Point3D translation) {
        setTranslation(translation.getInhomX(), translation.getInhomY(), translation.getInhomZ());
    }

    /**
     * Gets x, y, z coordinates of translation to be made by this transformation
     * as a new point.
     *
     * @return a new point containing translation coordinates.
     */
    public Point3D getTranslationPoint() {
        final var out = Point3D.create();
        getTranslationPoint(out);
        return out;
    }

    /**
     * Gets x, y, z coordinates of translation to be made by this transformation
     * and stores them into provided point.
     *
     * @param out point where translation coordinates will be stored.
     */
    public void getTranslationPoint(final Point3D out) {
        out.setInhomogeneousCoordinates(translation[0], translation[1], translation[2]);
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
     * Adds provided z coordinate to current translation assigned to this
     * transformation.
     *
     * @param translationZ Z coordinate to be added to current translation.
     */
    public void addTranslationZ(final double translationZ) {
        translation[2] += translationZ;
    }

    /**
     * Adds provided coordinates to current translation assigned to this
     * transformation.
     *
     * @param translationX x coordinate to be added to current translation.
     * @param translationY y coordinate to be added to current translation.
     * @param translationZ z coordinate to be added to current translation.
     */
    public void addTranslation(
            final double translationX, final double translationY, final double translationZ) {
        translation[0] += translationX;
        translation[1] += translationY;
        translation[2] += translationZ;
    }

    /**
     * Adds provided coordinates to current translation assigned to this
     * transformation.
     *
     * @param translation x, y, z coordinates to be added to current
     *                    translation.
     */
    public void addTranslation(final Point3D translation) {
        addTranslation(translation.getInhomX(), translation.getInhomY(), translation.getInhomZ());
    }

    /**
     * Represents this transformation as a 4x4 matrix.
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
     * Represents this transformation as a 4x4 matrix and stores the result in
     * provided instance.
     *
     * @param m instance where transformation matrix will be stored.
     * @throws IllegalArgumentException raised if provided instance is not a 4x4
     *                                  matrix.
     */
    @Override
    public void asMatrix(final Matrix m) {
        if (m.getRows() != HOM_COORDS || m.getColumns() != HOM_COORDS) {
            throw new IllegalArgumentException();
        }

        m.initialize(0.0);

        // set rotation
        m.setSubmatrix(0, 0, Rotation3D.INHOM_COORDS - 1,
                Rotation3D.INHOM_COORDS - 1, rotation.asInhomogeneousMatrix());

        // set translation
        m.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                HOM_COORDS - 1, translation);

        // set last element
        m.setElementAt(HOM_COORDS - 1, HOM_COORDS - 1, 1.0);
    }

    /**
     * Transforms input point using this transformation and stores the result
     * in provided output points.
     *
     * @param inputPoint  point to be transformed.
     * @param outputPoint instance where transformed point data will be stored.
     */
    @Override
    public void transform(final Point3D inputPoint, final Point3D outputPoint) {
        inputPoint.normalize();
        rotation.rotate(inputPoint, outputPoint);
        outputPoint.setInhomogeneousCoordinates(outputPoint.getInhomX() + translation[0],
                outputPoint.getInhomY() + translation[1], outputPoint.getInhomZ() + translation[2]);
    }

    /**
     * Transforms a quadric using this transformation and stores the result into
     * provided output quadric.
     *
     * @param inputQuadric  Quadric to be transformed.
     * @param outputQuadric instance where data of transformed quadric will be
     *                      stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output quadric matrix is not considered to be symmetric.
     */
    @Override
    public void transform(final Quadric inputQuadric, final Quadric outputQuadric) throws NonSymmetricMatrixException {
        // point' * quadric * point = 0
        // point' * T' * transformedQuadric * T * point = 0
        // where:
        // - transformedPoint = T * point

        // Hence:
        // transformedQuadric = T^-1' * quadric * T^-1

        inputQuadric.normalize();

        final var q = inputQuadric.asMatrix();
        final var invT = inverseAndReturnNew().asMatrix();
        // normalize transformation matrix invT to increase accuracy
        var norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);

        final var m = invT.transposeAndReturnNew();
        try {
            m.multiply(q);
            m.multiply(invT);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        // normalize resulting m matrix to increase accuracy so that it can be
        // considered symmetric
        norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);

        outputQuadric.setParameters(m);
    }

    /**
     * Transforms a dual quadric using this transformation and stores the result
     * into provided output dual quadric.
     *
     * @param inputDualQuadric  dual quadric to be transformed.
     * @param outputDualQuadric instance where data of transformed dual quadric
     *                          will be stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision.
     *                                     the resulting output dual quadric matrix is not considered to be
     *                                     symmetric.
     */
    @Override
    public void transform(final DualQuadric inputDualQuadric, final DualQuadric outputDualQuadric)
            throws NonSymmetricMatrixException {
        // plane' * dualQuadric * plane = 0
        // plane' * T^-1 * T * dualQuadric * T' * T^-1'*plane

        // Hence:
        // transformed plane: T^-1'*plane
        // transformed dual quadric: T * dualQuadric * T'

        inputDualQuadric.normalize();

        final var dualQ = inputDualQuadric.asMatrix();
        final var t = asMatrix();
        // normalize transformation matrix T to increase accuracy
        var norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final var transT = t.transposeAndReturnNew();
        try {
            t.multiply(dualQ);
            t.multiply(transT);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        // normalize resulting m matrix to increase accuracy so that it can be
        // considered symmetric
        norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        outputDualQuadric.setParameters(t);
    }

    /**
     * Transforms provided input plane using this transformation and stores the
     * result into provided output plane instance.
     *
     * @param inputPlane  plane to be transformed.
     * @param outputPlane instance where data of transformed plane will be
     *                    stored.
     */
    @Override
    public void transform(final Plane inputPlane, final Plane outputPlane) {
        // plane' * point = 0 --> plane' * T^-1 * T * point
        // (plane' * T^-1)*(T*point) = (T^-1'*plane)'*(T*point)
        // where:
        // - transformedPlane = T^-1'*plane
        // - transformedPoint = T*point

        inputPlane.normalize();

        final var invT = inverseAndReturnNew().asMatrix();
        final var plane = Matrix.newFromArray(inputPlane.asArray());

        // normalize transformation matrix T to increase accuracy
        final var norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);

        invT.transpose();
        try {
            invT.multiply(plane);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        outputPlane.setParameters(invT.getBuffer());
    }

    /**
     * Transforms a camera using this transformation and stores the result into
     * provided output camera.
     *
     * @param inputCamera  camera to be transformed.
     * @param outputCamera instance where data of transformed camera will be
     *                     stored.
     */
    @Override
    public void transform(final PinholeCamera inputCamera, final PinholeCamera outputCamera) {
        inputCamera.normalize();

        final var invT = inverseAndReturnNew().asMatrix();
        final var c = inputCamera.getInternalMatrix();
        try {
            c.multiply(invT);
            outputCamera.setInternalMatrix(c);
        } catch (final WrongSizeException ignore) {
            // never thrown
        }
    }

    /**
     * Converts this transformation into a metric transformation.
     *
     * @return this transformation converted into a metric transformation.
     */
    public MetricTransformation3D toMetric() {
        return new MetricTransformation3D(rotation, translation, MetricTransformation3D.DEFAULT_SCALE);
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
    public Transformation3D inverseAndReturnNew() {
        final var result = new EuclideanTransformation3D();
        inverse(result);
        return result;
    }

    /**
     * Combines this transformation with provided transformation.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix of provided transformation.
     *
     * @param transformation Transformation to be combined with.
     */
    public void combine(final EuclideanTransformation3D transformation) {
        combine(transformation, this);
    }

    /**
     * Combines this transformation with provided transformation and returns
     * the result as a new transformation instance.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix pf provided transformation.
     *
     * @param transformation Transformation to be combined with.
     * @return A new transformation resulting of the combination with this
     * transformation and provided transformation.
     */
    public EuclideanTransformation3D combineAndReturnNew(final EuclideanTransformation3D transformation) {
        final var result = new EuclideanTransformation3D();
        combine(transformation, result);
        return result;
    }

    /**
     * Estimates this transformation internal parameters by using 4
     * corresponding original and transformed points.
     *
     * @param inputPoint1  1st input point.
     * @param inputPoint2  2nd input point.
     * @param inputPoint3  3rd input point.
     * @param inputPoint4  4th input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input
     *                     point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input
     *                     point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     *                     point.
     * @param outputPoint4 4th transformed point corresponding to 4th input
     *                     point.
     * @throws CoincidentPointsException raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    public void setTransformationFromPoints(
            final Point3D inputPoint1, final Point3D inputPoint2, final Point3D inputPoint3, final Point3D inputPoint4,
            final Point3D outputPoint1, final Point3D outputPoint2, final Point3D outputPoint3,
            final Point3D outputPoint4) throws CoincidentPointsException {
        internalSetTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3, inputPoint4, outputPoint1,
                outputPoint2, outputPoint3, outputPoint4);
    }

    /**
     * Computes the inverse of this transformation and stores the result in
     * provided instance.
     *
     * @param result instance where inverse transformation will be stored.
     */
    protected void inverse(final EuclideanTransformation3D result) {
        // Transformation is as follows: x' = R* x + t
        // Then inverse transformation is: R'* x' = R' * R * x + R'*t = x + R'*t
        // --> x = R'*x' - R'*t

        // reverse rotation
        result.rotation = rotation.inverseRotationAndReturnNew();

        // reverse translation
        final var t = Matrix.newFromArray(translation, true);
        t.multiplyByScalar(-1.0);
        final var invRot = result.rotation.asInhomogeneousMatrix();
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
    private void combine(final EuclideanTransformation3D inputTransformation,
                         final EuclideanTransformation3D outputTransformation) {
        // combination in matrix representation is:
        // [R1 t1] * [R2 t2] = [R1*R2 + t1*0T  R1*t2 + t1*1] = [R1*R2 R1*t2 + t1]
        // [0T 1 ]   [0T 1 ]   [0T*R2 + 1*0T   0T*t2 + 1*1 ]   [0T    1         ]

        try {
            // we do translation first, because this.rotation might change later
            final var r1 = this.rotation.asInhomogeneousMatrix();
            final var t2 = Matrix.newFromArray(inputTransformation.translation, true);
            // this is R1 * t2
            r1.multiply(t2);

            ArrayUtils.sum(r1.toArray(), this.translation, outputTransformation.translation);

            outputTransformation.rotation = this.rotation.combineAndReturnNew(inputTransformation.rotation);

        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Estimates this transformation internal parameters by using 4
     * corresponding original and transformed points.
     *
     * @param inputPoint1  1st input point.
     * @param inputPoint2  2nd input point.
     * @param inputPoint3  3rd input point.
     * @param inputPoint4  4th input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input
     *                     point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input
     *                     point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     *                     point.
     * @param outputPoint4 4th transformed point corresponding to 4th input
     *                     point.
     * @throws CoincidentPointsException raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    private void internalSetTransformationFromPoints(
            final Point3D inputPoint1, final Point3D inputPoint2, final Point3D inputPoint3, final Point3D inputPoint4,
            final Point3D outputPoint1, final Point3D outputPoint2, final Point3D outputPoint3,
            final Point3D outputPoint4) throws CoincidentPointsException {
        final var inputPoints = new ArrayList<Point3D>();
        inputPoints.add(inputPoint1);
        inputPoints.add(inputPoint2);
        inputPoints.add(inputPoint3);
        inputPoints.add(inputPoint4);

        final var outputPoints = new ArrayList<Point3D>();
        outputPoints.add(outputPoint1);
        outputPoints.add(outputPoint2);
        outputPoints.add(outputPoint3);
        outputPoints.add(outputPoint4);

        final var estimator = new EuclideanTransformation3DEstimator(inputPoints, outputPoints);

        try {
            estimator.estimate(this);
        } catch (final LockedException | NotReadyException ignore) {
            // never thrown
        }
    }
}
