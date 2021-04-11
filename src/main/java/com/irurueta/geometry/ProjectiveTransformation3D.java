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
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.LUDecomposer;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.RQDecomposer;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;

import java.io.Serializable;
import java.util.Arrays;

/**
 * This class performs projective transformations on 2D space.
 * Projective transformations include any possible transformation that can be
 * applied to 3D points.
 */
@SuppressWarnings("DuplicatedCode")
public class ProjectiveTransformation3D extends Transformation3D
        implements Serializable {

    /**
     * Constant indicating number of coordinates required in translation arrays.
     */
    public static final int NUM_TRANSLATION_COORDS = 3;

    /**
     * Constant indicating the number of projective parameters that can be set
     * in projective parameters array.
     */
    public static final int NUM_PROJECTIVE_PARAMS = 4;

    /**
     * Constant defining number of inhomogeneous coordinates in 3D space
     */
    public static final int INHOM_COORDS = 3;

    /**
     * Constant defining number of homogeneous coordinates in 3D space
     */
    public static final int HOM_COORDS = 4;

    /**
     * Machine precision
     */
    public static final double EPS = 1e-12;

    /**
     * Constant defining a large threshold to consider a matrix valid as
     * rotation
     */
    private static final double LARGE_ROTATION_MATRIX_THRESHOLD = 1.0;

    /**
     * Internal 4x4 matrix containing transformation.
     */
    private Matrix t;

    /**
     * Indicates whether internal matrix is normalized.
     */
    private boolean normalized;

    /**
     * Empty constructor.
     * Creates transformation that has no effect.
     */
    public ProjectiveTransformation3D() {
        super();
        try {
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        normalize();
    }

    /**
     * Creates transformation with provided internal matrix.
     * Notice that provided matrix should usually be invertible, otherwise the
     * transformation will be degenerate and its inverse will not be available.
     *
     * @param t Internal 4x4 matrix.
     * @throws NullPointerException     raised if provided matrix is null.
     * @throws IllegalArgumentException raised if provided matrix is not 4x4.
     */
    public ProjectiveTransformation3D(final Matrix t) {
        setT(t);
        normalize();
    }

    /**
     * Creates transformation with provided scale value.
     *
     * @param scale Scale value. Values between 0.0 and 1.0 reduce objects,
     *              values greater than 1.0 enlarge objects and negative values reverse
     *              objects.
     */
    public ProjectiveTransformation3D(final double scale) {
        final double[] diag = new double[HOM_COORDS];
        Arrays.fill(diag, scale);
        // set last element to 1.0
        diag[HOM_COORDS - 1] = 1.0;
        t = Matrix.diagonal(diag);
        normalize();
    }

    /**
     * Creates transformation with provided rotation.
     *
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public ProjectiveTransformation3D(final Rotation3D rotation) {
        t = rotation.asHomogeneousMatrix();
        normalize();
    }

    /**
     * Creates transformation with provided scale and rotation.
     *
     * @param scale    Scale value. Values between 0.0 and 1.0 reduce objects,
     *                 values greater than 1.0 enlarge objects and negative values reverse
     *                 objects.
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public ProjectiveTransformation3D(final double scale, final Rotation3D rotation) {
        try {
            final double[] diag = new double[INHOM_COORDS];
            Arrays.fill(diag, scale);
            final Matrix a = Matrix.diagonal(diag);
            a.multiply(rotation.asInhomogeneousMatrix());
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            t.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, a);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        normalize();
    }

    /**
     * Creates transformation with provided affine parameters and rotation.
     *
     * @param params   affine parameters including x,y, z scaling and skewness
     *                 of axes.
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided parameters are null or
     *                              if provided rotation is null.
     */
    public ProjectiveTransformation3D(final AffineParameters3D params,
                                      final Rotation3D rotation) {
        try {
            final Matrix a = params.asMatrix();
            a.multiply(rotation.asInhomogeneousMatrix());
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            t.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, a);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        normalize();
    }

    /**
     * Creates transformation with provided 3D translation.
     *
     * @param translation array indicating 3D translation using inhomogeneous
     *                    coordinates.
     * @throws NullPointerException     raised if provided array is null.
     * @throws IllegalArgumentException raised if length of array is not equal
     *                                  to NUM_TRANSLATION_COORDS.
     */
    public ProjectiveTransformation3D(final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        try {
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            t.setSubmatrix(0, 3, 2, 3, translation);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        normalize();
    }

    /**
     * Creates transformation with provided affine linear mapping and
     * translation.
     *
     * @param a           affine linear mapping.
     * @param translation array indicating 3D translation using inhomogeneous
     *                    coordinates.
     * @throws NullPointerException     raised if provided array is null or if
     *                                  affine linear mapping is null.
     * @throws IllegalArgumentException raised if length of array is not equal
     *                                  to NUM_TRANSLATION_COORDS.
     */
    public ProjectiveTransformation3D(final Matrix a, final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        try {
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            t.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, a);
            t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                    HOM_COORDS - 1, translation);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        normalize();
    }

    /**
     * Creates transformation with provided scale and translation.
     *
     * @param scale       scale value. Values between 0.0 and 1.0 reduce objects,
     *                    values greater than 1.0 enlarge objects and negative values reverse
     *                    objects.
     * @param translation array indicating 3D translation using inhomogeneous
     *                    coordinates.
     * @throws NullPointerException     raised if provided translation is null.
     * @throws IllegalArgumentException Raised if provided translation does not
     *                                  have length 3.
     */
    public ProjectiveTransformation3D(final double scale, final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        final double[] diag = new double[HOM_COORDS];
        Arrays.fill(diag, scale);
        // set last element to 1.0
        diag[HOM_COORDS - 1] = 1.0;
        t = Matrix.diagonal(diag);

        // set translation
        t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                HOM_COORDS - 1, translation);
        normalize();
    }

    /**
     * Creates transformation with provided rotation and translation.
     *
     * @param rotation    a 3D rotation.
     * @param translation array indicating 3D translation using inhomogeneous
     *                    coordinates.
     * @throws NullPointerException     raised if provided rotation or translation
     *                                  is null.
     * @throws IllegalArgumentException raised if provided translation does not
     *                                  have length 3.
     */
    public ProjectiveTransformation3D(final Rotation3D rotation, final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        t = rotation.asHomogeneousMatrix();

        // set translation
        t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                HOM_COORDS - 1, translation);
        normalize();
    }

    /**
     * Creates transformation with provided scale, rotation and translation.
     *
     * @param scale       scale value. Values between 0.0 and 1.0 reduce objects,
     *                    values greater than 1.0 enlarge objects and negative values reverse
     *                    objects.
     * @param rotation    a 3D rotation.
     * @param translation array indicating 3D translation using inhomogeneous
     *                    coordinates.
     * @throws NullPointerException     raised if provided rotation or translation
     *                                  is null.
     * @throws IllegalArgumentException raised if provided translation does not
     *                                  have length 3.
     */
    public ProjectiveTransformation3D(final double scale, final Rotation3D rotation,
                                      final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        try {
            final double[] diag = new double[INHOM_COORDS];
            Arrays.fill(diag, scale);
            final Matrix a = Matrix.diagonal(diag);
            a.multiply(rotation.asInhomogeneousMatrix());

            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            // set A
            t.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, a);
            // set translation
            t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                    HOM_COORDS - 1, translation);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        normalize();
    }

    /**
     * Creates transformation with provided scale, rotation and translation.
     *
     * @param scale                scale value. Values between 0.0 and 1.0 reduce objects,
     *                             values greater than 1.0 enlarge objects and negative values reverse
     *                             objects.
     * @param rotation             a 3D rotation.
     * @param translation          array indicating 3D translation using inhomogeneous
     *                             coordinates.
     * @param projectiveParameters array of length 4 containing projective
     *                             parameters.
     * @throws NullPointerException     raised if provided rotation or translation
     *                                  is null.
     * @throws IllegalArgumentException raised if provided translation does not
     *                                  have length 3 or if projective parameters array doesn't have length 4.
     */
    public ProjectiveTransformation3D(final double scale, final Rotation3D rotation,
                                      final double[] translation, final double[] projectiveParameters) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        if (projectiveParameters.length != HOM_COORDS) {
            throw new IllegalArgumentException();
        }

        try {
            final double value = projectiveParameters[HOM_COORDS - 1];
            final double[] diag = new double[INHOM_COORDS];
            Arrays.fill(diag, scale);
            final Matrix a = Matrix.diagonal(diag);
            a.multiply(rotation.asInhomogeneousMatrix());

            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            // set A
            t.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, a);
            // set translation
            t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                    HOM_COORDS - 1, translation);
            t.multiplyByScalar(value);

            t.setSubmatrix(HOM_COORDS - 1, 0, HOM_COORDS - 1, HOM_COORDS - 1,
                    projectiveParameters);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        normalize();
    }

    /**
     * Creates transformation with provided parameters, rotation and
     * translation.
     *
     * @param params      affine parameters including horizontal scaling, vertical
     *                    scaling and skewness.
     * @param rotation    a 3D rotation.
     * @param translation array indicating 3D translation using inhomogeneous
     *                    coordinates.
     * @throws NullPointerException     raised if provided parameters, rotation or
     *                                  translation is null.
     * @throws IllegalArgumentException raised if provided translation does not
     *                                  have length 3.
     */
    public ProjectiveTransformation3D(final AffineParameters3D params,
                                      final Rotation3D rotation, final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        try {
            final Matrix a = params.asMatrix();
            a.multiply(rotation.asInhomogeneousMatrix());
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            // set A
            t.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, a);
            // set translation
            t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                    HOM_COORDS - 1, translation);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        normalize();
    }

    /**
     * Creates transformation with provided parameters, rotation and
     * translation.
     *
     * @param params               affine parameters including horizontal scaling, vertical
     *                             scaling and skewness.
     * @param rotation             a 3D rotation.
     * @param translation          array indicating 3D translation using inhomogeneous
     *                             coordinates.
     * @param projectiveParameters array of length 4 containing projective
     *                             parameters.
     * @throws NullPointerException     raised if provided parameters, rotation or
     *                                  translation is null.
     * @throws IllegalArgumentException raised if provided translation does not
     *                                  have length 3 or if projective parameters array doesn't have length 4.
     */
    public ProjectiveTransformation3D(final AffineParameters3D params,
                                      final Rotation3D rotation, final double[] translation,
                                      final double[] projectiveParameters) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        if (projectiveParameters.length != HOM_COORDS) {
            throw new IllegalArgumentException();
        }

        try {
            final Matrix a = params.asMatrix();
            a.multiply(rotation.asInhomogeneousMatrix());
            t = Matrix.identity(HOM_COORDS, HOM_COORDS);
            // set A
            t.setSubmatrix(0, 0, INHOM_COORDS - 1, INHOM_COORDS - 1, a);
            // set translation
            t.setSubmatrix(0, HOM_COORDS - 1, translation.length - 1,
                    HOM_COORDS - 1, translation);
            final double value = projectiveParameters[HOM_COORDS - 1];
            t.multiplyByScalar(value);

            t.setSubmatrix(HOM_COORDS - 1, 0, HOM_COORDS - 1, HOM_COORDS - 1,
                    projectiveParameters);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        normalize();
    }

    /**
     * Creates transformation by estimating its internal matrix by providing 5
     * corresponding original and transformed points.
     *
     * @param inputPoint1  1st input point.
     * @param inputPoint2  2nd input point.
     * @param inputPoint3  3rd input point.
     * @param inputPoint4  4th input point.
     * @param inputPoint5  5th input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input
     *                     point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input
     *                     point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     *                     point.
     * @param outputPoint4 4th transformed point corresponding to 4th input
     *                     point.
     * @param outputPoint5 5th transformed point corresponding to 5th input
     *                     point.
     * @throws CoincidentPointsException raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    public ProjectiveTransformation3D(
            final Point3D inputPoint1, final Point3D inputPoint2, final Point3D inputPoint3,
            final Point3D inputPoint4, final Point3D inputPoint5, final Point3D outputPoint1,
            final Point3D outputPoint2, final Point3D outputPoint3, final Point3D outputPoint4,
            final Point3D outputPoint5) throws CoincidentPointsException {
        try {
            t = new Matrix(HOM_COORDS, HOM_COORDS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        setTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3,
                inputPoint4, inputPoint5, outputPoint1, outputPoint2,
                outputPoint3, outputPoint4, outputPoint5);
    }

    /**
     * Creates transformation by estimating its internal matrix by providing 5
     * corresponding original and transformed planes.
     *
     * @param inputPlane1  1st input plane.
     * @param inputPlane2  2nd input plane.
     * @param inputPlane3  3rd input plane.
     * @param inputPlane4  4th input plane.
     * @param inputPlane5  5th input plane.
     * @param outputPlane1 1st transformed plane corresponding to 1st input
     *                     plane.
     * @param outputPlane2 2nd transformed plane corresponding to 2nd input
     *                     plane.
     * @param outputPlane3 3rd transformed plane corresponding to 3rd input
     *                     plane.
     * @param outputPlane4 4th transformed plane corresponding to 4th input
     *                     plane.
     * @param outputPlane5 5th transformed plane corresponding to 5th input
     *                     plane.
     * @throws CoincidentPlanesException raised if transformation cannot be
     *                                   estimated for some reason (plane configuration degeneracy, duplicate
     *                                   planes or numerical instabilities).
     */
    public ProjectiveTransformation3D(
            final Plane inputPlane1, final Plane inputPlane2, final Plane inputPlane3,
            final Plane inputPlane4, final Plane inputPlane5, final Plane outputPlane1,
            final Plane outputPlane2, final Plane outputPlane3, final Plane outputPlane4,
            final Plane outputPlane5) throws CoincidentPlanesException {
        setTransformationFromPlanes(inputPlane1, inputPlane2, inputPlane3,
                inputPlane4, inputPlane5, outputPlane1, outputPlane2,
                outputPlane3, outputPlane4, outputPlane5);
    }

    /**
     * Creates transformation by estimating its internal matrix by providing 3
     * corresponding original and transformed lines.
     *
     * @param inputLine1  1st input line.
     * @param inputLine2  2nd input line.
     * @param inputLine3  3rd input line.
     * @param outputLine1 1st transformed line corresponding to 1st input line.
     * @param outputLine2 2nd transformed line corresponding to 2nd input line.
     * @param outputLine3 3rd transformed line corresponding to 3rd input line.
     * @throws CoincidentLinesException raised if transformation cannot be
     *                                  estimated for some reason (line configuration degeneracy, duplicate lines
     *                                  or numerical instabilities).
     */
    public ProjectiveTransformation3D(
            final Line3D inputLine1, final Line3D inputLine2, final Line3D inputLine3,
            final Line3D outputLine1, final Line3D outputLine2, final Line3D outputLine3)
            throws CoincidentLinesException {
        setTransformationFromLines(inputLine1, inputLine2, inputLine3,
                outputLine1, outputLine2, outputLine3);
    }

    /**
     * Returns internal matrix containing this transformation data.
     * Point transformation is computed as t * x, where x is a 3D point
     * expressed using homogeneous coordinates.
     * Usually the internal transformation matrix will be invertible.
     * When this is not the case, the transformation is considered degenerate
     * and its inverse will not be available.
     *
     * @return internal transformation matrix.
     */
    public Matrix getT() {
        return t;
    }

    /**
     * Sets internal matrix containing this transformation data.
     * Point transformation is computed as t * x, where x is a 3D point
     * expressed using homogeneous coordinates.
     * Usually provided matrix will be invertible, when this is not the case
     * this transformation will become degenerate and its inverse will not be
     * available.
     * This method does not check whether provided matrix is invertible or not.
     *
     * @param t transformation matrix.
     * @throws NullPointerException     raised if provided matrix is null.
     * @throws IllegalArgumentException raised if provided matrix is not 4x4.
     */
    public final void setT(final Matrix t) {
        if (t.getRows() != HOM_COORDS || t.getColumns() != HOM_COORDS) {
            throw new IllegalArgumentException();
        }

        this.t = t;
        normalized = false;
    }

    /**
     * Returns boolean indicating whether provided matrix will produce a
     * degenerate projective transformation or not.
     *
     * @param t a 4x4 matrix to be used as the internal matrix of a projective
     *          transformation.
     * @return true if matrix will produce a degenerate transformation, false
     * otherwise.
     * @throws IllegalArgumentException raised if provided matrix is not 4x4.
     */
    public static boolean isDegenerate(final Matrix t) {
        if (t.getRows() != HOM_COORDS || t.getColumns() != HOM_COORDS) {
            throw new IllegalArgumentException();
        }

        try {
            final LUDecomposer decomposer = new LUDecomposer(t);
            decomposer.decompose();
            return decomposer.isSingular();
        } catch (final AlgebraException e) {
            // if decomposition fails, assume that matrix is degenerate because
            // of numerical instabilities
            return true;
        }
    }

    /**
     * Indicates whether this transformation is degenerate.
     * When a transformation is degenerate, its inverse cannot be computed.
     *
     * @return true if transformation is degenerate, false otherwise.
     */
    public boolean isDegenerate() {
        return isDegenerate(t);
    }

    /**
     * Returns affine linear mapping matrix.
     *
     * @return linear mapping matrix.
     * @see AffineTransformation3D
     */
    public Matrix getA() {
        final Matrix a = t.getSubmatrix(0, 0, INHOM_COORDS - 1,
                INHOM_COORDS - 1);
        a.multiplyByScalar(1.0 / t.getElementAt(HOM_COORDS - 1,
                HOM_COORDS - 1));
        return a;
    }

    /**
     * Sets affine linear mapping matrix.
     *
     * @param a Linear mapping matrix.
     * @throws NullPointerException     raised if provided matrix is null.
     * @throws IllegalArgumentException raised if provided matrix does not have
     *                                  size 3x3.
     * @see AffineTransformation3D
     */
    public final void setA(final Matrix a) {
        if (a == null) {
            throw new NullPointerException();
        }
        if (a.getRows() != INHOM_COORDS || a.getColumns() != INHOM_COORDS) {
            throw new IllegalArgumentException();
        }

        t.setSubmatrix(0, 0, INHOM_COORDS - 1,
                INHOM_COORDS - 1,
                a.multiplyByScalarAndReturnNew(t.getElementAt(
                        HOM_COORDS - 1, HOM_COORDS - 1)));
    }

    /**
     * Normalizes current matrix instance.
     */
    public final void normalize() {
        if (!normalized) {
            final double norm = Utils.normF(t);
            if (norm > EPS) {
                t.multiplyByScalar(1.0 / norm);
            }
            normalized = true;
        }
    }

    /**
     * Returns the 3D rotation component associated to this transformation.
     * Note: if this rotation instance is modified, its changes won't be
     * reflected on this transformation until rotation is set again.
     *
     * @return 3D rotation.
     * @throws AlgebraException if for some reason rotation cannot be estimated
     *                          (usually because of numerical instability).
     */
    public Rotation3D getRotation() throws AlgebraException {
        // Use QR decomposition to retrieve rotation component of this
        // transformation
        normalize();
        final RQDecomposer decomposer = new RQDecomposer(t.getSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1));
        try {
            decomposer.decompose();
            //a large threshold is used because Q matrix is always assumed to be orthonormal
            return new MatrixRotation3D(decomposer.getQ(),
                    LARGE_ROTATION_MATRIX_THRESHOLD);
        } catch (final InvalidRotationMatrixException ignore) {
            return null;
        }
    }

    /**
     * Sets 3D rotation for this transformation.
     *
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     * @throws AlgebraException     raised if for numerical reasons rotation cannot
     *                              be set (usually because of numerical instability in parameters of this
     *                              transformation).
     */
    public void setRotation(final Rotation3D rotation) throws AlgebraException {
        final Matrix rotMatrix = rotation.asInhomogeneousMatrix();

        // Use QR decomposition to retrieve parameters matrix
        final RQDecomposer decomposer = new RQDecomposer(t.getSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1));
        decomposer.decompose();
        // retrieves params matrix
        final Matrix localA = decomposer.getR();
        localA.multiply(rotMatrix);
        t.setSubmatrix(0, 0, INHOM_COORDS - 1,
                INHOM_COORDS - 1, localA);
        normalized = false;
    }

    /**
     * Adds provided rotation to current rotation assigned to this
     * transformation.
     *
     * @param rotation 3D rotation to be added.
     * @throws AlgebraException raised if for numerical reasons rotation cannot
     *                          be set (usually because of numerical instability in parameters of this
     *                          transformation).
     */
    public void addRotation(final Rotation3D rotation) throws AlgebraException {
        final Rotation3D localRotation = getRotation();
        localRotation.combine(rotation);
        setRotation(localRotation);
    }

    /**
     * Sets scale of this transformation.
     *
     * @param scale scale value to be set. A value between 0.0 and 1.0 indicates
     *              that objects will be reduced, a value greater than 1.0 indicates that
     *              objects will be enlarged, and a negative value indicates that objects
     *              will be reversed.
     * @throws AlgebraException Raised if for numerical reasons scale cannot
     *                          be set (usually because of numerical instability in parameters of this
     *                          transformation).
     */
    public void setScale(final double scale) throws AlgebraException {
        normalize();
        final double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        final RQDecomposer decomposer = new RQDecomposer(t.getSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1));
        decomposer.decompose();
        // params
        final Matrix localA = decomposer.getR();
        localA.setElementAt(0, 0, scale * value);
        localA.setElementAt(1, 1, scale * value);
        localA.setElementAt(2, 2, scale * value);
        localA.multiply(decomposer.getQ());
        t.setSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1, localA);
        normalized = false;
    }

    /**
     * Gets affine parameters of associated to this instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     *
     * @return affine parameters.
     * @throws AlgebraException raised if for numerical reasons affine.
     *                          parameters cannot be retrieved (usually because of numerical instability
     *                          of the internal matrix of this instance).
     */
    public AffineParameters3D getAffineParameters() throws AlgebraException {
        final AffineParameters3D parameters = new AffineParameters3D();
        getAffineParameters(parameters);
        return parameters;
    }

    /**
     * Computes affine parameters associated to this instance and stores the
     * result in provided instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     *
     * @param result instance where affine parameters will be stored.
     * @throws AlgebraException raised if for numerical reasons affine
     *                          parameters cannot be retrieved (usually because of numerical instability
     *                          of the internal matrix of this instance).
     */
    public void getAffineParameters(final AffineParameters3D result)
            throws AlgebraException {
        normalize();
        final double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        final RQDecomposer decomposer = new RQDecomposer(t.getSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1));
        decomposer.decompose();
        final Matrix r = decomposer.getR();
        r.multiplyByScalar(1.0 / value);
        result.fromMatrix(r);
    }

    /**
     * Sets affine parameters associated to this instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     *
     * @param parameters affine parameters to be set.
     * @throws AlgebraException raised if for numerical reasons affine
     *                          parameters cannot be set (usually because of numerical instability of
     *                          the internal matrix of this instance).
     */
    public void setAffineParameters(final AffineParameters3D parameters)
            throws AlgebraException {
        normalize();
        final double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        final RQDecomposer decomposer = new RQDecomposer(t.getSubmatrix(0, 0,
                INHOM_COORDS - 1, INHOM_COORDS - 1));
        decomposer.decompose();
        final Matrix params = parameters.asMatrix();
        final Matrix rotation = decomposer.getQ();

        // params is equivalent to A because it
        // has been multiplied by rotation
        params.multiply(rotation);
        // normalize
        params.multiplyByScalar(value);
        t.setSubmatrix(0, 0, INHOM_COORDS - 1,
                INHOM_COORDS - 1, params);
        normalized = false;
    }

    /**
     * Returns the projective parameters associated to this instance.
     * These parameters are the located in the last row of the internal
     * transformation matrix.
     * For affine, metric or euclidean transformations this last row is always
     * [0, 0, 0, 1] (taking into account that transformation matrix is defined
     * up to scale).
     *
     * @return Projective parameters returned as the array containing the values
     * of the last row of the internal transformation matrix.
     */
    public double[] getProjectiveParameters() {
        // return last row of matrix t
        return t.getSubmatrixAsArray(HOM_COORDS - 1, 0, HOM_COORDS - 1,
                HOM_COORDS - 1, true);
    }

    /**
     * Sets the projective parameters associated to this instance.
     * These parameters will be set in the last row of the internal
     * transformation matrix.
     * For affine, matrix or euclidean transformations parameters are always
     * [0, 0, 0, 1] (taking into account that transformation matrix is defined
     * up to scale).
     *
     * @param params projective parameters to be set. It must be an array of
     *               length 4.
     * @throws IllegalArgumentException raised if provided array does not have
     *                                  length 4.
     */
    public final void setProjectiveParameters(final double[] params) {
        if (params.length != HOM_COORDS) {
            throw new IllegalArgumentException();
        }

        t.setSubmatrix(HOM_COORDS - 1, 0, HOM_COORDS - 1,
                HOM_COORDS - 1, params);
        normalized = false;
    }

    /**
     * Returns 3D translation assigned to this transformation as a new array
     * expressed in inhomogeneous coordinates.
     * Note: Updating the values of the returned array will not update the
     * translation of this instance. To do so, translation needs to be set
     * again.
     *
     * @return 3D translation array.
     */
    public double[] getTranslation() {
        normalize();
        final double[] translation = t.getSubmatrixAsArray(0, HOM_COORDS - 1,
                INHOM_COORDS - 1, HOM_COORDS - 1);
        final double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        ArrayUtils.multiplyByScalar(translation, 1.0 / value, translation);
        return translation;
    }

    /**
     * Obtains 3D translation assigned to this transformation and stores result
     * into provided array.
     * Note: Updating the values of the returned array will not update the
     * translation of this instance. To do so, translation needs to be set
     * again.
     *
     * @param out array where translation values will be stored.
     * @throws WrongSizeException if provided array does not have length 3.
     */
    public void getTranslation(final double[] out) throws WrongSizeException {
        t.getSubmatrixAsArray(0, HOM_COORDS - 1, INHOM_COORDS - 1,
                HOM_COORDS - 1, out);
        final double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        ArrayUtils.multiplyByScalar(out, 1.0 / value, out);
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

        final double value = t.getElementAt(HOM_COORDS - 1, HOM_COORDS - 1);
        final double[] translation2 = ArrayUtils.multiplyByScalarAndReturnNew(
                translation, value);
        t.setSubmatrix(0, HOM_COORDS - 1, translation2.length - 1,
                HOM_COORDS - 1, translation2);
        normalized = false;
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
        final double[] currentTranslation = getTranslation();
        ArrayUtils.sum(currentTranslation, translation, currentTranslation);
        setTranslation(currentTranslation);
    }

    /**
     * Returns current x coordinate translation assigned to this transformation.
     *
     * @return X coordinate translation.
     */
    public double getTranslationX() {
        normalize();
        return t.getElementAt(0, HOM_COORDS - 1) / t.getElementAt(
                HOM_COORDS - 1, HOM_COORDS - 1);
    }

    /**
     * Sets x coordinate translation to be made by this transformation.
     *
     * @param translationX X coordinate translation to be set.
     */
    public void setTranslationX(final double translationX) {
        t.setElementAt(0, HOM_COORDS - 1, translationX * t.getElementAt(
                HOM_COORDS - 1, HOM_COORDS - 1));
        normalized = false;
    }

    /**
     * Returns current y coordinate translation assigned to this transformation.
     *
     * @return Y coordinate translation.
     */
    public double getTranslationY() {
        normalize();
        return t.getElementAt(1, HOM_COORDS - 1) / t.getElementAt(
                HOM_COORDS - 1, HOM_COORDS - 1);
    }

    /**
     * Sets y coordinate translation to be made by this transformation.
     *
     * @param translationY Y coordinate translation to be set.
     */
    public void setTranslationY(final double translationY) {
        t.setElementAt(1, HOM_COORDS - 1, translationY * t.getElementAt(
                HOM_COORDS - 1, HOM_COORDS - 1));
        normalized = false;
    }

    /**
     * Returns current z coordinate translation assigned to this transformation.
     *
     * @return Z coordinate translation.
     */
    public double getTranslationZ() {
        normalize();
        return t.getElementAt(2, HOM_COORDS - 1) / t.getElementAt(
                HOM_COORDS - 1, HOM_COORDS - 1);
    }

    /**
     * Sets z coordinate translation to be made by this transformation.
     *
     * @param translationZ z coordinate translation to be set.
     */
    public void setTranslationZ(final double translationZ) {
        t.setElementAt(2, HOM_COORDS - 1, translationZ * t.getElementAt(
                HOM_COORDS - 1, HOM_COORDS - 1));
        normalized = false;
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
        setTranslationX(translationX);
        setTranslationY(translationY);
        setTranslationZ(translationZ);
    }

    /**
     * Sets x, y, z coordinates of translation to be made by this
     * transformation.
     *
     * @param translation translation to be set.
     */
    public void setTranslation(final Point3D translation) {
        setTranslation(translation.getInhomX(), translation.getInhomY(),
                translation.getInhomZ());
    }

    /**
     * Gets x, y, z coordinates of translation to be made by this transformation
     * as a new point.
     *
     * @return a new point containing translation coordinates.
     */
    public Point3D getTranslationPoint() {
        final Point3D out = Point3D.create();
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
        out.setInhomogeneousCoordinates(getTranslationX(), getTranslationY(),
                getTranslationZ());
    }

    /**
     * Adds provided x coordinate to current translation assigned to this
     * transformation.
     *
     * @param translationX X coordinate to be added to current translation.
     */
    public void addTranslationX(final double translationX) {
        setTranslationX(getTranslationX() + translationX);
    }

    /**
     * Adds provided y coordinate to current translation assigned to this
     * transformation.
     *
     * @param translationY Y coordinate to be added to current translation.
     */
    public void addTranslationY(final double translationY) {
        setTranslationY(getTranslationY() + translationY);
    }

    /**
     * Adds provided z coordinate to current translation assigned to this
     * transformation.
     *
     * @param translationZ Z coordinate to be added to current translation.
     */
    public void addTranslationZ(final double translationZ) {
        setTranslationZ(getTranslationZ() + translationZ);
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
        addTranslationX(translationX);
        addTranslationY(translationY);
        addTranslationZ(translationZ);
    }

    /**
     * Adds provided coordinates to current translation assigned to this
     * transformation.
     *
     * @param translation x, y, z coordinates to be added to current
     *                    translation.
     */
    public void addTranslation(final Point3D translation) {
        addTranslation(translation.getInhomX(), translation.getInhomY(),
                translation.getInhomZ());
    }

    /**
     * Represents this transformation as a 4x4 matrix.
     * A point can be transformed as t * p, where t is the transformation matrix
     * and p is a point expressed as an homogeneous vector.
     *
     * @return This transformation in matrix form.
     */
    @Override
    public Matrix asMatrix() {
        return new Matrix(t);
    }

    /**
     * Represents this transformation as a 4x4 matrix and stores the result in
     * provided instance.
     *
     * @param m Instance where transformation matrix will be stored.
     * @throws IllegalArgumentException Raised if provided instance is not a 4x4
     *                                  matrix.
     */
    @Override
    public void asMatrix(final Matrix m) {
        if (m.getRows() != HOM_COORDS || m.getColumns() != HOM_COORDS) {
            throw new IllegalArgumentException();
        }

        m.copyFrom(t);
    }

    /**
     * Transforms input point using this transformation and stores the result in
     * provided output points.
     *
     * @param inputPoint  point to be transformed.
     * @param outputPoint Instance where transformed point data will be stored.
     */
    @Override
    public void transform(final Point3D inputPoint, final Point3D outputPoint) {

        inputPoint.normalize();
        normalize();
        try {
            final Matrix point = new Matrix(
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            point.setElementAtIndex(0, inputPoint.getHomX());
            point.setElementAtIndex(1, inputPoint.getHomY());
            point.setElementAtIndex(2, inputPoint.getHomZ());
            point.setElementAtIndex(3, inputPoint.getHomW());

            final Matrix transformedPoint = t.multiplyAndReturnNew(point);

            outputPoint.setHomogeneousCoordinates(
                    transformedPoint.getElementAtIndex(0),
                    transformedPoint.getElementAtIndex(1),
                    transformedPoint.getElementAtIndex(2),
                    transformedPoint.getElementAtIndex(3));
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Transforms a quadric using this transformation and stores the result into
     * provided output quadric.
     *
     * @param inputQuadric  quadric to be transformed.
     * @param outputQuadric instance where data of transformed quadric will be
     *                      stored.
     * @throws NonSymmetricMatrixException Raised if due to numerical precision
     *                                     the resulting output quadric matrix is not considered to be symmetric.
     * @throws AlgebraException            raised if transform cannot be computed becauseof
     *                                     numerical instabilities.
     */
    @Override
    public void transform(final Quadric inputQuadric, final Quadric outputQuadric)
            throws NonSymmetricMatrixException, AlgebraException {
        // point' * quadric * point = 0
        // point' * t' * transformedQuadric * t * point = 0
        // where:
        // - transformedPoint = t * point

        // Hence:
        // transformedQuadric = t^-1' * quadric * t^-1

        inputQuadric.normalize();

        final Matrix q = inputQuadric.asMatrix();
        normalize();

        final Matrix invT = inverseAndReturnNew().asMatrix();
        // normalize transformation matrix invT to increase accuracy
        double norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);

        final Matrix m = invT.transposeAndReturnNew();
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
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output dual quadric matrix is not considered to be
     *                                     symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because
     *                                     of numerical instabilities.
     */
    @Override
    public void transform(
            final DualQuadric inputDualQuadric, final DualQuadric outputDualQuadric)
            throws NonSymmetricMatrixException, AlgebraException {
        // plane' * dualQuadric * plane = 0
        // plane' * t^-1 * t * dualQuadric * t' * t^-1'*plane

        // Hence:
        // transformed plane: t^-1'*plane
        // transformed dual quadric: t * dualQuadric * t'

        inputDualQuadric.normalize();
        normalize();

        final Matrix dualQ = inputDualQuadric.asMatrix();
        final Matrix transT = t.transposeAndReturnNew();

        final Matrix m = t.multiplyAndReturnNew(dualQ);
        m.multiply(transT);

        // normalize resulting m matrix to increase accuracy so that it can be
        // considered symmetric
        final double norm = Utils.normF(m);
        m.multiplyByScalar(1.0 / norm);

        outputDualQuadric.setParameters(m);
    }

    /**
     * Transforms provided input plane using this transformation and stores the
     * result into provided output plane instance.
     *
     * @param inputPlane  plane to be transformed.
     * @param outputPlane instance where data of transformed plane will be
     *                    stored.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    @Override
    public void transform(final Plane inputPlane, final Plane outputPlane)
            throws AlgebraException {
        // plane' * point = 0 --> plane' * t^-1 * t * point
        // (plane' * t^-1)*(t*point) = (t^-1'*plane)'*(t*point)
        // where:
        // - transformedPlane = t^-1'*plane
        // - transformedPoint = t*point


        inputPlane.normalize();
        normalize();

        final Matrix invT = inverseAndReturnNew().asMatrix();
        final Matrix l = Matrix.newFromArray(inputPlane.asArray());

        invT.transpose();
        invT.multiply(l);

        outputPlane.setParameters(invT.toArray());
    }

    /**
     * Transforms a camera using this transformation and stores the result into
     * provided output camera.
     *
     * @param inputCamera  camera to be transformed.
     * @param outputCamera instance where data of transforeed camera will be
     *                     stored.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    @Override
    public void transform(final PinholeCamera inputCamera,
                          final PinholeCamera outputCamera) throws AlgebraException {

        inputCamera.normalize();
        normalize();

        final Matrix invT = inverseAndReturnNew().asMatrix();
        final Matrix c = inputCamera.getInternalMatrix();
        c.multiply(invT);
        outputCamera.setInternalMatrix(c);
    }

    /**
     * Inverses this transformation.
     *
     * @throws AlgebraException if inverse transform cannot be computed because
     *                          of numerical instabilities.
     */
    public void inverse() throws AlgebraException {
        inverse(this);
    }

    /**
     * Computes the inverse of this transformation and returns the result as a
     * new transformation instance.
     *
     * @return inverse transformation.
     * @throws AlgebraException if inverse transfor cannot be computed because
     *                          of numerical instabilities.
     */
    public Transformation3D inverseAndReturnNew() throws AlgebraException {
        final ProjectiveTransformation3D result = new ProjectiveTransformation3D();
        inverse(result);
        return result;
    }

    /**
     * Computes the inverse of this transformation and stores the result in
     * provided instance.
     *
     * @param result instance where inverse transformation will be stored.
     * @throws AlgebraException if inverse transformAndReturnNew cannot be
     *                          computed because of numerical instabilities.
     */
    protected void inverse(final ProjectiveTransformation3D result)
            throws AlgebraException {

        result.t = Utils.inverse(t);
    }

    /**
     * Combines this transformation with provided transformation.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix of provided transformation.
     *
     * @param transformation transformation to be combined with.
     */
    public void combine(final ProjectiveTransformation3D transformation) {
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
    public ProjectiveTransformation3D combineAndReturnNew(
            final ProjectiveTransformation3D transformation) {

        final ProjectiveTransformation3D result = new ProjectiveTransformation3D();
        combine(transformation, result);
        return result;
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
    private void combine(final ProjectiveTransformation3D inputTransformation,
                         final ProjectiveTransformation3D outputTransformation) {
        // combination in matrix representation is: T1 * T2

        normalize();
        inputTransformation.normalize();


        try {
            outputTransformation.t = this.t.multiplyAndReturnNew(
                    inputTransformation.t);

        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Estimates this transformation internal matrix by providing 5
     * corresponding original and transformed points.
     *
     * @param inputPoint1  1st input point.
     * @param inputPoint2  2nd input point.
     * @param inputPoint3  3rd input point.
     * @param inputPoint4  4th input point.
     * @param inputPoint5  5th input point.
     * @param outputPoint1 1st transformed point corresponding to 1st input
     *                     point.
     * @param outputPoint2 2nd transformed point corresponding to 2nd input
     *                     point.
     * @param outputPoint3 3rd transformed point corresponding to 3rd input
     *                     point.
     * @param outputPoint4 4th transformed point corresponding to 4th input
     *                     point.
     * @param outputPoint5 5th transformed point corresponding to 5th input
     *                     point.
     * @throws CoincidentPointsException Raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    public final void setTransformationFromPoints(
            final Point3D inputPoint1, final Point3D inputPoint2, final Point3D inputPoint3,
            final Point3D inputPoint4, final Point3D inputPoint5, final Point3D outputPoint1,
            final Point3D outputPoint2, final Point3D outputPoint3, final Point3D outputPoint4,
            final Point3D outputPoint5) throws CoincidentPointsException {

        // normalize points to increase accuracy
        inputPoint1.normalize();
        inputPoint2.normalize();
        inputPoint3.normalize();
        inputPoint4.normalize();
        inputPoint5.normalize();

        outputPoint1.normalize();
        outputPoint2.normalize();
        outputPoint3.normalize();
        outputPoint4.normalize();
        outputPoint5.normalize();

        // matrix of homogeneous linear system of equations.
        // There are 16 unknowns and 15 equations (3 for each pair of
        // corresponding points)
        Matrix m = null;
        try {
            // build matrix initialized to zero
            m = new Matrix(15, 16);

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

            double oXiX = oX * iX;
            double oXiY = oX * iY;
            double oXiZ = oX * iZ;
            double oXiW = oX * iW;

            double oYiX = oY * iX;
            double oYiY = oY * iY;
            double oYiZ = oY * iZ;
            double oYiW = oY * iW;

            double oZiX = oZ * iX;
            double oZiY = oZ * iY;
            double oZiZ = oZ * iZ;
            double oZiW = oZ * iW;

            double tmp = oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW;
            double norm = Math.sqrt(tmp + oXiX * oXiX + oXiY * oXiY + oXiZ * oXiZ +
                    oXiW * oXiW);

            m.setElementAt(0, 0, oWiX / norm);
            m.setElementAt(0, 1, oWiY / norm);
            m.setElementAt(0, 2, oWiZ / norm);
            m.setElementAt(0, 3, oWiW / norm);

            m.setElementAt(0, 12, -oXiX / norm);
            m.setElementAt(0, 13, -oXiY / norm);
            m.setElementAt(0, 14, -oXiZ / norm);
            m.setElementAt(0, 15, -oXiW / norm);

            norm = Math.sqrt(tmp + oYiX * oYiX + oYiY * oYiY + oYiZ * oYiZ +
                    oYiW * oYiW);

            m.setElementAt(1, 4, oWiX / norm);
            m.setElementAt(1, 5, oWiY / norm);
            m.setElementAt(1, 6, oWiZ / norm);
            m.setElementAt(1, 7, oWiW / norm);

            m.setElementAt(1, 12, -oYiX / norm);
            m.setElementAt(1, 13, -oYiY / norm);
            m.setElementAt(1, 14, -oYiZ / norm);
            m.setElementAt(1, 15, -oYiW / norm);

            norm = Math.sqrt(tmp + oZiX * oZiX + oZiY * oZiY + oZiZ * oZiZ +
                    oZiW * oZiW);

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

            tmp = oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW;
            norm = Math.sqrt(tmp + oXiX * oXiX + oXiY * oXiY + oXiZ * oXiZ +
                    oXiW * oXiW);

            m.setElementAt(3, 0, oWiX / norm);
            m.setElementAt(3, 1, oWiY / norm);
            m.setElementAt(3, 2, oWiZ / norm);
            m.setElementAt(3, 3, oWiW / norm);

            m.setElementAt(3, 12, -oXiX / norm);
            m.setElementAt(3, 13, -oXiY / norm);
            m.setElementAt(3, 14, -oXiZ / norm);
            m.setElementAt(3, 15, -oXiW / norm);

            norm = Math.sqrt(tmp + oYiX * oYiX + oYiY * oYiY + oYiZ * oYiZ +
                    oYiW * oYiW);

            m.setElementAt(4, 4, oWiX / norm);
            m.setElementAt(4, 5, oWiY / norm);
            m.setElementAt(4, 6, oWiZ / norm);
            m.setElementAt(4, 7, oWiW / norm);

            m.setElementAt(4, 12, -oYiX / norm);
            m.setElementAt(4, 13, -oYiY / norm);
            m.setElementAt(4, 14, -oYiZ / norm);
            m.setElementAt(4, 15, -oYiW / norm);

            norm = Math.sqrt(tmp + oZiX * oZiX + oZiY * oZiY + oZiZ * oZiZ +
                    oZiW * oZiW);

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

            tmp = oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW;
            norm = Math.sqrt(tmp + oXiX * oXiX + oXiY * oXiY + oXiZ * oXiZ +
                    oXiW * oXiW);

            m.setElementAt(6, 0, oWiX / norm);
            m.setElementAt(6, 1, oWiY / norm);
            m.setElementAt(6, 2, oWiZ / norm);
            m.setElementAt(6, 3, oWiW / norm);

            m.setElementAt(6, 12, -oXiX / norm);
            m.setElementAt(6, 13, -oXiY / norm);
            m.setElementAt(6, 14, -oXiZ / norm);
            m.setElementAt(6, 15, -oXiW / norm);

            norm = Math.sqrt(tmp + oYiX * oYiX + oYiY * oYiY + oYiZ * oYiZ +
                    oYiW * oYiW);

            m.setElementAt(7, 4, oWiX / norm);
            m.setElementAt(7, 5, oWiY / norm);
            m.setElementAt(7, 6, oWiZ / norm);
            m.setElementAt(7, 7, oWiW / norm);

            m.setElementAt(7, 12, -oYiX / norm);
            m.setElementAt(7, 13, -oYiY / norm);
            m.setElementAt(7, 14, -oYiZ / norm);
            m.setElementAt(7, 15, -oYiW / norm);

            norm = Math.sqrt(tmp + oZiX * oZiX + oZiY * oZiY + oZiZ * oZiZ +
                    oZiW * oZiW);

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

            tmp = oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW;
            norm = Math.sqrt(tmp + oXiX * oXiX + oXiY * oXiY + oXiZ * oXiZ +
                    oXiW * oXiW);

            m.setElementAt(9, 0, oWiX / norm);
            m.setElementAt(9, 1, oWiY / norm);
            m.setElementAt(9, 2, oWiZ / norm);
            m.setElementAt(9, 3, oWiW / norm);

            m.setElementAt(9, 12, -oXiX / norm);
            m.setElementAt(9, 13, -oXiY / norm);
            m.setElementAt(9, 14, -oXiZ / norm);
            m.setElementAt(9, 15, -oXiW / norm);

            norm = Math.sqrt(tmp + oYiX * oYiX + oYiY * oYiY + oYiZ * oYiZ +
                    oYiW * oYiW);

            m.setElementAt(10, 4, oWiX / norm);
            m.setElementAt(10, 5, oWiY / norm);
            m.setElementAt(10, 6, oWiZ / norm);
            m.setElementAt(10, 7, oWiW / norm);

            m.setElementAt(10, 12, -oYiX / norm);
            m.setElementAt(10, 13, -oYiY / norm);
            m.setElementAt(10, 14, -oYiZ / norm);
            m.setElementAt(10, 15, -oYiW / norm);

            norm = Math.sqrt(tmp + oZiX * oZiX + oZiY * oZiY + oZiZ * oZiZ +
                    oZiW * oZiW);

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

            tmp = oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW;
            norm = Math.sqrt(tmp + oXiX * oXiX + oXiY * oXiY + oXiZ * oXiZ +
                    oXiW * oXiW);

            m.setElementAt(12, 0, oWiX / norm);
            m.setElementAt(12, 1, oWiY / norm);
            m.setElementAt(12, 2, oWiZ / norm);
            m.setElementAt(12, 3, oWiW / norm);

            m.setElementAt(12, 12, -oXiX / norm);
            m.setElementAt(12, 13, -oXiY / norm);
            m.setElementAt(12, 14, -oXiZ / norm);
            m.setElementAt(12, 15, -oXiW / norm);

            norm = Math.sqrt(tmp + oYiX * oYiX + oYiY * oYiY + oYiZ * oYiZ +
                    oYiW * oYiW);

            m.setElementAt(13, 4, oWiX / norm);
            m.setElementAt(13, 5, oWiY / norm);
            m.setElementAt(13, 6, oWiZ / norm);
            m.setElementAt(13, 7, oWiW / norm);

            m.setElementAt(13, 12, -oYiX / norm);
            m.setElementAt(13, 13, -oYiY / norm);
            m.setElementAt(13, 14, -oYiZ / norm);
            m.setElementAt(13, 15, -oYiW / norm);

            norm = Math.sqrt(tmp + oZiX * oZiX + oZiY * oZiY + oZiZ * oZiZ +
                    oZiW * oZiW);

            m.setElementAt(14, 8, oWiX / norm);
            m.setElementAt(14, 9, oWiY / norm);
            m.setElementAt(14, 10, oWiZ / norm);
            m.setElementAt(14, 11, oWiW / norm);

            m.setElementAt(14, 12, -oZiX / norm);
            m.setElementAt(14, 13, -oZiY / norm);
            m.setElementAt(14, 14, -oZiZ / norm);
            m.setElementAt(14, 15, -oZiW / norm);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        // use SVD to decompose matrix m
        Matrix v;
        try {
            final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            // ensure that matrix m has enough rank and there is a unique
            // solution (up to scale)
            if (decomposer.getRank() < 15) {
                throw new CoincidentPointsException();
            }
            v = decomposer.getV(); //V is 16x16

            // last column of V will contain parameters of transformation
            t.setSubmatrix(0, 0, HOM_COORDS - 1, HOM_COORDS - 1,
                    v.getSubmatrix(0, 15, 15, 15).toArray(), false);
            normalized = true; //because columns of V are normalized after SVD

        } catch (final AlgebraException e) {
            throw new CoincidentPointsException(e);
        }
    }

    /**
     * Estimates this transformation internal matrix by providing 4
     * corresponding original and transformed planes.
     *
     * @param inputPlane1  1st input plane.
     * @param inputPlane2  2nd input plane.
     * @param inputPlane3  3rd input plane.
     * @param inputPlane4  4th input plane.
     * @param inputPlane5  5th input plane.
     * @param outputPlane1 1st transformed plane corresponding to 1st input
     *                     plane.
     * @param outputPlane2 2nd transformed plane corresponding to 2nd input
     *                     plane.
     * @param outputPlane3 3rd transformed plane corresponding to 3rd input
     *                     plane.
     * @param outputPlane4 4th transformed plane corresponding to 4th input
     *                     plane.
     * @param outputPlane5 5th transformed plane corresponding to 4th input
     *                     plane.
     * @throws CoincidentPlanesException Raised if transformation cannot be
     *                                   estimated for some reason (plane configuration degeneracy, duplicate
     *                                   plane or numerical instabilities).
     */
    public final void setTransformationFromPlanes(
            final Plane inputPlane1, final Plane inputPlane2, final Plane inputPlane3,
            final Plane inputPlane4, final Plane inputPlane5, final Plane outputPlane1,
            final Plane outputPlane2, final Plane outputPlane3, final Plane outputPlane4,
            final Plane outputPlane5) throws CoincidentPlanesException {

        // normalize lines to increase accuracy
        inputPlane1.normalize();
        inputPlane2.normalize();
        inputPlane3.normalize();
        inputPlane4.normalize();
        inputPlane5.normalize();

        outputPlane1.normalize();
        outputPlane2.normalize();
        outputPlane3.normalize();
        outputPlane4.normalize();
        outputPlane5.normalize();

        // matrix of homogeneous linear system of equations.
        // There are 9 unknowns and 8 equations (2 for each pair of corresponding
        // points)
        Matrix m = null;
        try {
            // build matrix initialized to zero
            m = new Matrix(15, 16);

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

            double oAiA = oA * iA;
            double oAiB = oA * iB;
            double oAiC = oA * iC;
            double oAiD = oA * iD;

            double oBiA = oB * iA;
            double oBiB = oB * iB;
            double oBiC = oB * iC;
            double oBiD = oB * iD;

            double oCiA = oC * iA;
            double oCiB = oC * iB;
            double oCiC = oC * iC;
            double oCiD = oC * iD;

            double tmp = oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD;
            double norm = Math.sqrt(tmp + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC +
                    oAiD * oAiD);

            m.setElementAt(0, 0, oDiA / norm);
            m.setElementAt(0, 1, oDiB / norm);
            m.setElementAt(0, 2, oDiC / norm);
            m.setElementAt(0, 3, oDiD / norm);

            m.setElementAt(0, 12, -oAiA / norm);
            m.setElementAt(0, 13, -oAiB / norm);
            m.setElementAt(0, 14, -oAiC / norm);
            m.setElementAt(0, 15, -oAiD / norm);

            norm = Math.sqrt(tmp + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC +
                    oBiD * oBiD);

            m.setElementAt(1, 4, oDiA / norm);
            m.setElementAt(1, 5, oDiB / norm);
            m.setElementAt(1, 6, oDiC / norm);
            m.setElementAt(1, 7, oDiD / norm);

            m.setElementAt(1, 12, -oBiA / norm);
            m.setElementAt(1, 13, -oBiB / norm);
            m.setElementAt(1, 14, -oBiC / norm);
            m.setElementAt(1, 15, -oBiD / norm);

            norm = Math.sqrt(tmp + oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                    oCiD * oCiD);

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

            tmp = oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD;
            norm = Math.sqrt(tmp + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC +
                    oAiD * oAiD);

            m.setElementAt(3, 0, oDiA / norm);
            m.setElementAt(3, 1, oDiB / norm);
            m.setElementAt(3, 2, oDiC / norm);
            m.setElementAt(3, 3, oDiD / norm);

            m.setElementAt(3, 12, -oAiA / norm);
            m.setElementAt(3, 13, -oAiB / norm);
            m.setElementAt(3, 14, -oAiC / norm);
            m.setElementAt(3, 15, -oAiD / norm);

            norm = Math.sqrt(tmp + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC +
                    oBiD * oBiD);

            m.setElementAt(4, 4, oDiA / norm);
            m.setElementAt(4, 5, oDiB / norm);
            m.setElementAt(4, 6, oDiC / norm);
            m.setElementAt(4, 7, oDiD / norm);

            m.setElementAt(4, 12, -oBiA / norm);
            m.setElementAt(4, 13, -oBiB / norm);
            m.setElementAt(4, 14, -oBiC / norm);
            m.setElementAt(4, 15, -oBiD / norm);

            norm = Math.sqrt(tmp + oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                    oCiD * oCiD);

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

            tmp = oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD;
            norm = Math.sqrt(tmp + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC +
                    oAiD * oAiD);

            m.setElementAt(6, 0, oDiA / norm);
            m.setElementAt(6, 1, oDiB / norm);
            m.setElementAt(6, 2, oDiC / norm);
            m.setElementAt(6, 3, oDiD / norm);

            m.setElementAt(6, 12, -oAiA / norm);
            m.setElementAt(6, 13, -oAiB / norm);
            m.setElementAt(6, 14, -oAiC / norm);
            m.setElementAt(6, 15, -oAiD / norm);

            norm = Math.sqrt(tmp + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC +
                    oBiD * oBiD);

            m.setElementAt(7, 4, oDiA / norm);
            m.setElementAt(7, 5, oDiB / norm);
            m.setElementAt(7, 6, oDiC / norm);
            m.setElementAt(7, 7, oDiD / norm);

            m.setElementAt(7, 12, -oBiA / norm);
            m.setElementAt(7, 13, -oBiB / norm);
            m.setElementAt(7, 14, -oBiC / norm);
            m.setElementAt(7, 15, -oBiD / norm);

            norm = Math.sqrt(tmp + oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                    oCiD * oCiD);

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

            tmp = oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD;
            norm = Math.sqrt(tmp + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC +
                    oAiD * oAiD);

            m.setElementAt(9, 0, oDiA / norm);
            m.setElementAt(9, 1, oDiB / norm);
            m.setElementAt(9, 2, oDiC / norm);
            m.setElementAt(9, 3, oDiD / norm);

            m.setElementAt(9, 12, -oAiA / norm);
            m.setElementAt(9, 13, -oAiB / norm);
            m.setElementAt(9, 14, -oAiC / norm);
            m.setElementAt(9, 15, -oAiD / norm);

            norm = Math.sqrt(tmp + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC +
                    oBiD * oBiD);

            m.setElementAt(10, 4, oDiA / norm);
            m.setElementAt(10, 5, oDiB / norm);
            m.setElementAt(10, 6, oDiC / norm);
            m.setElementAt(10, 7, oDiD / norm);

            m.setElementAt(10, 12, -oBiA / norm);
            m.setElementAt(10, 13, -oBiB / norm);
            m.setElementAt(10, 14, -oBiC / norm);
            m.setElementAt(10, 15, -oBiD / norm);

            norm = Math.sqrt(tmp + oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                    oCiD * oCiD);

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

            tmp = oDiA * oDiA + oDiB * oDiB + oDiC * oDiC + oDiD * oDiD;
            norm = Math.sqrt(tmp + oAiA * oAiA + oAiB * oAiB + oAiC * oAiC +
                    oAiD * oAiD);

            m.setElementAt(12, 0, oDiA / norm);
            m.setElementAt(12, 1, oDiB / norm);
            m.setElementAt(12, 2, oDiC / norm);
            m.setElementAt(12, 3, oDiD / norm);

            m.setElementAt(12, 12, -oAiA / norm);
            m.setElementAt(12, 13, -oAiB / norm);
            m.setElementAt(12, 14, -oAiC / norm);
            m.setElementAt(12, 15, -oAiD / norm);

            norm = Math.sqrt(tmp + oBiA * oBiA + oBiB * oBiB + oBiC * oBiC +
                    oBiD * oBiD);

            m.setElementAt(13, 4, oDiA / norm);
            m.setElementAt(13, 5, oDiB / norm);
            m.setElementAt(13, 6, oDiC / norm);
            m.setElementAt(13, 7, oDiD / norm);

            m.setElementAt(13, 12, -oBiA / norm);
            m.setElementAt(13, 13, -oBiB / norm);
            m.setElementAt(13, 14, -oBiC / norm);
            m.setElementAt(13, 15, -oBiD / norm);

            norm = Math.sqrt(tmp + oCiA * oCiA + oCiB * oCiB + oCiC * oCiC +
                    oCiD * oCiD);

            m.setElementAt(14, 8, oDiA / norm);
            m.setElementAt(14, 9, oDiB / norm);
            m.setElementAt(14, 10, oDiC / norm);
            m.setElementAt(14, 11, oDiD / norm);

            m.setElementAt(14, 12, -oCiA / norm);
            m.setElementAt(14, 13, -oCiB / norm);
            m.setElementAt(14, 14, -oCiC / norm);
            m.setElementAt(14, 15, -oCiD / norm);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        // use SVD to decompose matrix m
        Matrix v;
        try {
            final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            // ensure that matrix m has enough rank and there is a unique
            // solution (up to scale)
            if (decomposer.getRank() < 15) {
                throw new CoincidentPlanesException();
            }
            // V is 16x16
            v = decomposer.getV();

            // last column of V will contain parameters of transformation
            final Matrix transInvT = new Matrix(HOM_COORDS, HOM_COORDS);
            transInvT.setSubmatrix(0, 0, HOM_COORDS - 1,
                    HOM_COORDS - 1,
                    v.getSubmatrix(0, 15, 15, 15).toArray(),
                    false);
            // this is now invT
            transInvT.transpose();
            t = Utils.inverse(transInvT);
            // invT is normalized, but not t
            normalized = false;

        } catch (final AlgebraException e) {
            throw new CoincidentPlanesException(e);
        }
    }

    /**
     * Estimates this transformation internal matrix by providing 3
     * corresponding original and transformed lines.
     *
     * @param inputLine1  1st input line.
     * @param inputLine2  2nd input line.
     * @param inputLine3  3rd input line.
     * @param outputLine1 1st transformed line corresponding to 1st input line.
     * @param outputLine2 2nd transformed line corresponding to 2nd input line.
     * @param outputLine3 3rd transformed line corresponding to 3rd input line.
     * @throws CoincidentLinesException Raised if transformation cannot be
     *                                  estimated for some reason (line configuration degeneracy, duplicate lines
     *                                  or numerical instabilities).
     */
    public final void setTransformationFromLines(
            final Line3D inputLine1, final Line3D inputLine2, final Line3D inputLine3,
            final Line3D outputLine1, final Line3D outputLine2, final Line3D outputLine3)
            throws CoincidentLinesException {
        try {
            setTransformationFromPlanes(inputLine1.getPlane1(),
                    inputLine1.getPlane2(), inputLine2.getPlane1(),
                    inputLine2.getPlane2(), inputLine3.getPlane1(),
                    outputLine1.getPlane1(), outputLine1.getPlane2(),
                    outputLine2.getPlane1(), outputLine2.getPlane2(),
                    outputLine3.getPlane1());
        } catch (final CoincidentPlanesException e) {
            throw new CoincidentLinesException(e);
        }
    }
}
