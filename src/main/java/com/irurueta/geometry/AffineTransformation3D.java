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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.RQDecomposer;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;

import java.io.Serializable;
import java.util.Arrays;

/**
 * This class performs affine transformations on 3D space.
 * Affine transformations include transformations related to rotations,
 * translations, independently scaling horizontal or vertical coordinates
 * or skewing the coordinates axis.
 * This class is not intended to be used on points located at infinity or
 * at very large coordinates, since numerical instabilities may occur. For
 * those cases use a ProjectiveTransformation3D instead.
 */
@SuppressWarnings("DuplicatedCode")
public class AffineTransformation3D extends Transformation3D
        implements Serializable {

    /**
     * Constant indicating number of coordinates required in translation arrays.
     */
    public static final int NUM_TRANSLATION_COORDS = 3;


    /**
     * Constant defining number of inhomogeneous coordinates in 3D space.
     */
    public static final int INHOM_COORDS = 3;

    /**
     * Constant defining number of homogeneous coordinates in 3D space.
     */
    public static final int HOM_COORDS = 4;

    /**
     * Linear mapping.
     */
    private Matrix a;

    /**
     * 2D translation to be performed on geometric objects.
     * Translation is specified using inhomogeneous coordinates.
     */
    private double[] translation;

    /**
     * Empty constructor.
     * Creates transformation that has no effect.
     */
    public AffineTransformation3D() {
        super();
        try {
            a = Matrix.identity(INHOM_COORDS, INHOM_COORDS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        translation = new double[NUM_TRANSLATION_COORDS];
    }

    /**
     * Creates transformation with provided rotation.
     *
     * @param a linear mapping.
     * @throws NullPointerException     raised if provided rotation is null.
     * @throws IllegalArgumentException raised if provided matrix does not have
     *                                  size 3x3.
     */
    public AffineTransformation3D(final Matrix a) {
        setA(a);
        translation = new double[NUM_TRANSLATION_COORDS];
    }

    /**
     * Creates transformation with provided scale value.
     *
     * @param scale scale value. Values between 0.0 and 1.0 reduce objects,
     *              values greater than 1.0 enlarge objects and negative values reverse
     *              objects.
     */
    public AffineTransformation3D(final double scale) {
        final double[] diag = new double[INHOM_COORDS];
        Arrays.fill(diag, scale);
        a = Matrix.diagonal(diag);
        translation = new double[NUM_TRANSLATION_COORDS];
    }

    /**
     * Creates transformation with provided rotation.
     *
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public AffineTransformation3D(final Rotation3D rotation) {
        a = rotation.asInhomogeneousMatrix();
        translation = new double[NUM_TRANSLATION_COORDS];
    }

    /**
     * Creates transformation with provided scale and rotation.
     *
     * @param scale    scale value. Values between 0.0 and 1.0 reduce objects,
     *                 values greater than 1.0 enlarge objects and negative values reverse
     *                 objects.
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided rotation is null.
     */
    public AffineTransformation3D(final double scale, final Rotation3D rotation) {
        final double[] diag = new double[INHOM_COORDS];
        Arrays.fill(diag, scale);
        a = Matrix.diagonal(diag);
        try {
            a.multiply(rotation.asInhomogeneousMatrix());
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        translation = new double[NUM_TRANSLATION_COORDS];
    }

    /**
     * Creates transformation with provided affine parameters and rotation.
     *
     * @param params   Affine parameters including horizontal scaling, vertical
     *                 scaling and skewness.
     * @param rotation a 3D rotation.
     * @throws NullPointerException raised if provided parameters are null or
     *                              if provided rotation is null.
     */
    public AffineTransformation3D(final AffineParameters3D params,
                                  final Rotation3D rotation) {
        a = params.asMatrix();
        try {
            a.multiply(rotation.asInhomogeneousMatrix());
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        translation = new double[NUM_TRANSLATION_COORDS];
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
    public AffineTransformation3D(final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        try {
            a = Matrix.identity(INHOM_COORDS, INHOM_COORDS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        this.translation = translation;
    }

    /**
     * Creates transformation with provided rotation, translation and scale
     * value.
     *
     * @param a           linear mapping.
     * @param translation array indicating 3D translation using inhomogeneous
     *                    coordinates.
     * @throws NullPointerException     raised if provided array is null or if
     *                                  linear mapping is null.
     * @throws IllegalArgumentException Raised if length of array is not equal
     *                                  to NUM_TRANSLATION_COORDS.
     */
    public AffineTransformation3D(final Matrix a, final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }
        this.translation = translation;

        setA(a);
    }

    /**
     * Creates transformation with provided scale and translation.
     *
     * @param scale       Scale value. Values between 0.0 and 1.0 reduce objects,
     *                    values greater than 1.0 enlarge objects and negative values reverse
     *                    objects.
     * @param translation Array indicating 3D translation using inhomogeneous
     *                    coordinates.
     * @throws NullPointerException     raised if provided translation is null.
     * @throws IllegalArgumentException Raised if provided translation does not
     *                                  have length 3.
     */
    public AffineTransformation3D(final double scale, final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        final double[] diag = new double[INHOM_COORDS];
        Arrays.fill(diag, scale);
        a = Matrix.diagonal(diag);

        this.translation = translation;
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
    public AffineTransformation3D(final Rotation3D rotation, final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        a = rotation.asInhomogeneousMatrix();
        this.translation = translation;
    }

    /**
     * Creates transformation with provided scale, rotation and translation.
     *
     * @param scale       Scale value. Values between 0.0 and 1.0 reduce objects,
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
    public AffineTransformation3D(final double scale, final Rotation3D rotation,
                                  final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        final double[] diag = new double[INHOM_COORDS];
        Arrays.fill(diag, scale);
        a = Matrix.diagonal(diag);
        try {
            a.multiply(rotation.asInhomogeneousMatrix());
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        this.translation = translation;
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
    public AffineTransformation3D(final AffineParameters3D params,
                                  final Rotation3D rotation, final double[] translation) {
        if (translation.length != NUM_TRANSLATION_COORDS) {
            throw new IllegalArgumentException();
        }

        a = params.asMatrix();
        try {
            a.multiply(rotation.asInhomogeneousMatrix());
        } catch (final WrongSizeException ignore) {
            // never happens
        }
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
    public AffineTransformation3D(
            final Point3D inputPoint1, final Point3D inputPoint2, final Point3D inputPoint3,
            final Point3D inputPoint4, final Point3D outputPoint1, final Point3D outputPoint2,
            final Point3D outputPoint3, final Point3D outputPoint4) throws CoincidentPointsException {
        try {
            a = new Matrix(INHOM_COORDS, INHOM_COORDS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        translation = new double[NUM_TRANSLATION_COORDS];
        setTransformationFromPoints(inputPoint1, inputPoint2, inputPoint3,
                inputPoint4, outputPoint1, outputPoint2, outputPoint3,
                outputPoint4);
    }

    /**
     * Creates transformation by estimating its internal values using provided 4
     * corresponding original and transformed planes.
     *
     * @param inputPlane1  1st input plane.
     * @param inputPlane2  2nd input plane.
     * @param inputPlane3  3rd input plane.
     * @param inputPlane4  4th input plane.
     * @param outputPlane1 1st transformed plane corresponding to 1st input
     *                     plane.
     * @param outputPlane2 2nd transformed plane corresponding to 2nd input
     *                     plane.
     * @param outputPlane3 3rd transformed plane corresponding to 3rd input
     *                     plane.
     * @param outputPlane4 4th transformed plane corresponding to 4th input
     *                     plane.
     * @throws CoincidentPlanesException raised if transformation cannot be
     *                                   estimated for some reason (plane configuration degeneracy, duplicate
     *                                   planes or numerical instabilities).
     */
    public AffineTransformation3D(
            final Plane inputPlane1, final Plane inputPlane2, final Plane inputPlane3,
            final Plane inputPlane4, final Plane outputPlane1, final Plane outputPlane2,
            final Plane outputPlane3, final Plane outputPlane4) throws CoincidentPlanesException {
        setTransformationFromPlanes(inputPlane1, inputPlane2, inputPlane3,
                inputPlane4, outputPlane1, outputPlane2, outputPlane3,
                outputPlane4);
    }

    /**
     * Creates transformation by estimating internal parameters using provided 2
     * corresponding original and transformed lines.
     *
     * @param inputLine1  1st input line.
     * @param inputLine2  2nd input line.
     * @param outputLine1 1st transformed line corresponding to 1st input line.
     * @param outputLine2 2nd transformed line corresponding to 2nd input line.
     * @throws CoincidentLinesException raised if transformation cannot be
     *                                  estimated for some reason (line configuration degeneracy, duplicate lines
     *                                  or numerical instabilities).
     */
    public AffineTransformation3D(
            final Line3D inputLine1, final Line3D inputLine2, final Line3D outputLine1,
            final Line3D outputLine2) throws CoincidentLinesException {
        setTransformationFromLines(inputLine1, inputLine2, outputLine1,
                outputLine2);
    }

    /**
     * Returns linear mapping matrix to perform affine transformation.
     * Point transformation is computed as a * x + t, where x is a point and t
     * is the amount of translation.
     *
     * @return linear mapping matrix.
     */
    public Matrix getA() {
        return a;
    }

    /**
     * Sets linear mapping matrix to perform affine transformation.
     *
     * @param a linear mapping matrix.
     * @throws NullPointerException     raised if provided matrix is null.
     * @throws IllegalArgumentException raised if provided matrix does not have
     *                                  size 3x3.
     */
    public final void setA(final Matrix a) {
        if (a == null) {
            throw new NullPointerException();
        }
        if (a.getRows() != INHOM_COORDS || a.getColumns() != INHOM_COORDS) {
            throw new IllegalArgumentException();
        }

        this.a = a;
    }

    /**
     * Returns 3D rotation assigned to this transformation.
     * Note: if this rotation instance is modified, its changes won't be
     * reflected on this instance until rotation is set again.
     *
     * @return 3D rotation.
     * @throws AlgebraException if for some reason rotation cannot
     *                          be estimated (usually because of numerical instability).
     */
    public Rotation3D getRotation() throws AlgebraException {
        // Use QR decomposition to retrieve rotation
        final RQDecomposer decomposer = new RQDecomposer(a);
        try {
            decomposer.decompose();
            return new MatrixRotation3D(decomposer.getQ());
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
        final RQDecomposer decomposer = new RQDecomposer(a);
        decomposer.decompose();
        // retrieves params matrix
        final Matrix localA = decomposer.getR();
        localA.multiply(rotMatrix);
        this.a = localA;
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
     * @param scale acale value to be set. a value between 0.0 and 1.0 indicates
     *              that objects will be reduced, a value greater than 1.0 indicates that
     *              objects will be enlarged, and a negative value indicates that objects
     *              will be reversed.
     * @throws AlgebraException raised if for numerical reasons scale cannot
     *                          be set (usually because of numerical instability in parameters of this
     *                          transformation).
     */
    public void setScale(final double scale) throws AlgebraException {

        final RQDecomposer decomposer = new RQDecomposer(a);
        decomposer.decompose();
        // params
        final Matrix localA = decomposer.getR();
        localA.setElementAt(0, 0, scale);
        localA.setElementAt(1, 1, scale);
        // multiply by rotation
        localA.multiply(decomposer.getQ());
        this.a = localA;
    }

    /**
     * Gets affine parameters of this instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     *
     * @return affine parameters.
     * @throws AlgebraException raised if for numerical reasons affine
     *                          parameters cannot be retrieved (usually because of numerical instability
     *                          in matrix a).
     */
    public AffineParameters3D getParameters() throws AlgebraException {
        final AffineParameters3D parameters = new AffineParameters3D();
        getParameters(parameters);
        return parameters;
    }

    /**
     * Computes affine parameters of this instance and stores the result in
     * provided instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     *
     * @param result instance where affine parameters will be stored.
     * @throws AlgebraException raised if for numerical reasons affine
     *                          parameters cannot be retrieved (usually because of numerical instability
     *                          in matrix a).
     */
    public void getParameters(final AffineParameters3D result)
            throws AlgebraException {
        final RQDecomposer decomposer = new RQDecomposer(a);
        decomposer.decompose();
        final Matrix params = decomposer.getR();
        result.fromMatrix(params);
    }

    /**
     * Sets affine parameters of this instance.
     * Affine parameters contain horizontal scale, vertical scale and skewness
     * of axes.
     *
     * @param parameters affine parameters to be set.
     * @throws AlgebraException raised if for numerical reasons affine
     *                          parameters cannot be set (usually because of numerical instability in
     *                          current matrix a).
     */
    public void setParameters(final AffineParameters3D parameters)
            throws AlgebraException {
        final RQDecomposer decomposer = new RQDecomposer(a);
        decomposer.decompose();
        final Matrix params = parameters.asMatrix();
        final Matrix rotation = decomposer.getQ();

        params.multiply(rotation);
        a = params;
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
     * @param translation 3D translation array.
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
     * @param translationZ z coordinate translation to be set.
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
        out.setInhomogeneousCoordinates(translation[0], translation[1],
                translation[2]);
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
        addTranslation(translation.getInhomX(), translation.getInhomY(),
                translation.getInhomZ());
    }

    /**
     * Represents this transformation as a 4x4 matrix.
     * a point can be transformed as T * p, where T is the transformation matrix
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

        // set rotation
        m.setSubmatrix(0, 0, 2, 2,
                a);

        // set translation
        m.setSubmatrix(0, 3, 2, 3,
                translation);

        // set last element
        m.setElementAt(3, 0, 0.0);
        m.setElementAt(3, 1, 0.0);
        m.setElementAt(3, 2, 0.0);
        m.setElementAt(3, 3, 1.0);
    }

    /**
     * Transforms input point using this transformation and stores the result in
     * provided output points.
     *
     * @param inputPoint  point to be transformed.
     * @param outputPoint instance where transformed point data will be stored.
     */
    @Override
    public void transform(final Point3D inputPoint, final Point3D outputPoint) {
        try {
            final double[] coords = new double[
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            coords[0] = inputPoint.getInhomX();
            coords[1] = inputPoint.getInhomY();
            coords[2] = inputPoint.getInhomZ();

            final Matrix p = a.multiplyAndReturnNew(Matrix.newFromArray(coords,
                    true));

            outputPoint.setInhomogeneousCoordinates(
                    p.getElementAtIndex(0) + translation[0],
                    p.getElementAtIndex(1) + translation[1],
                    p.getElementAtIndex(2) + translation[2]);
        } catch (final WrongSizeException ignore) {
            // this exception will never be raised
        }
    }

    /**
     * Transforms a quadric using this transformation and stores the result into
     * provided output quadric.
     *
     * @param inputQuadric  quadric to be transformed.
     * @param outputQuadric instance where data of transformed quadric will be
     *                      stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output conic matrix is not considered to be symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because of
     *                                     numerical instabilities.
     */
    @Override
    public void transform(final Quadric inputQuadric, final Quadric outputQuadric)
            throws NonSymmetricMatrixException, AlgebraException {
        // point' * quadric * point = 0
        // point' * T' * transformedQuadric * T * point = 0
        // where:
        // - transformedPoint = T * point

        // Hence:
        // transformedQuadric = T^-1' * quadric * T^-1

        inputQuadric.normalize();

        final Matrix q = inputQuadric.asMatrix();
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
     *                                     the resulting output dual conic matrix is not considered to be symmetric.
     */
    @Override
    public void transform(final DualQuadric inputDualQuadric,
                          final DualQuadric outputDualQuadric) throws NonSymmetricMatrixException {
        // plane' * dualQuadric * plane = 0
        // plane' * T^-1 * T * dualQuadric * T' * T^-1'*plane

        // Hence:
        // transformed plane: T^-1'*plane
        // transformed dual quadric: T * dualQuadric * T'

        inputDualQuadric.normalize();

        final Matrix dualQ = inputDualQuadric.asMatrix();
        final Matrix t = asMatrix();
        // normalize transformation matrix T to increase accuracy
        double norm = Utils.normF(t);
        t.multiplyByScalar(1.0 / norm);

        final Matrix transT = t.transposeAndReturnNew();
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
     * @throws AlgebraException raised if transformAndReturnNew cannot be
     *                          computed because of numerical instabilities.
     */
    @Override
    public void transform(final Plane inputPlane, final Plane outputPlane)
            throws AlgebraException {
        // plane' * point = 0 --> plane' * T^-1 * T * point
        // (plane' * T^-1)*(T*point) = (T^-1'*plane)'*(T*point)
        // where:
        // - transformedPlane = T^-1'*plane
        // - transformedPoint = T*point

        inputPlane.normalize();

        final Matrix invT = inverseAndReturnNew().asMatrix();
        final Matrix p = Matrix.newFromArray(inputPlane.asArray());

        // normalize transformation matrix T to increase accuracy
        final double norm = Utils.normF(invT);
        invT.multiplyByScalar(1.0 / norm);

        invT.transpose();
        invT.multiply(p);

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
     * @throws AlgebraException if inverse transform cannot be computed because
     *                          of numerical instabilities.
     */
    public Transformation3D inverseAndReturnNew() throws AlgebraException {
        final AffineTransformation3D result = new AffineTransformation3D();
        inverse(result);
        return result;
    }

    /**
     * Computes the inverse of this transformation and stores the result in
     * provided instance.
     *
     * @param result instance where inverse transformation will be stored.
     * @throws AlgebraException if inverse transform cannot be computed because
     *                          of numerical instabilities.
     */
    public void inverse(final AffineTransformation3D result)
            throws AlgebraException {

        // x' = a * x + t -->
        // a^-1 * x' = a^-1 * a * x + a^-1 * t = x + a^-1 * t -->
        // x = a^-1 * x' - a^-1 * t

        try {
            // reverse rotation
            final Matrix invA = Utils.inverse(a);
            result.a = invA;

            // reverse translation
            final Matrix t = Matrix.newFromArray(translation, true);
            t.multiplyByScalar(-1.0);

            final Matrix resultT = invA.multiplyAndReturnNew(t);
            result.translation = resultT.toArray();
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Converts this transformation into a metric transformation.
     *
     * @return This transformation converted into a projective transformation.
     */
    public ProjectiveTransformation3D toProjective() {
        return new ProjectiveTransformation3D(a, translation);
    }

    /**
     * Combines this transformation with provided transformation.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix of provided transformation.
     *
     * @param transformation Transformation to be combined with.
     */
    public void combine(final AffineTransformation3D transformation) {
        combine(transformation, this);
    }

    /**
     * Combines this transformation with provided transformation and returns
     * the result as a new transformation instance.
     * The combination is equivalent to multiplying the matrix of this
     * transformation with the matrix of provided transformation.
     *
     * @param transformation Transformation to be combined with.
     * @return a new transformation resulting of the combination with this
     * transformation and provided transformation.
     */
    public AffineTransformation3D combineAndReturnNew(
            final AffineTransformation3D transformation) {

        final AffineTransformation3D result = new AffineTransformation3D();
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
    private void combine(final AffineTransformation3D inputTransformation,
                         final AffineTransformation3D outputTransformation) {
        // combination in matrix representation is:
        // [A1 t1] * [A2 t2] = [A1*A2 + t1*0T  A1*t2 + t1*1] = [A1*A2 A1*t2 + t1]
        // [0T 1 ]   [0T 1 ]   [0T*A2 + 1*0T   0T*t2 + 1*1 ]   [0T    1         ]

        try {
            // we do translation first, because this.rotation might change later
            final Matrix a1 = new Matrix(this.a);
            final Matrix t2 = Matrix.newFromArray(inputTransformation.translation, true);
            // this is R1 * t2
            a1.multiply(t2);

            ArrayUtils.sum(a1.toArray(), this.translation,
                    outputTransformation.translation);

            outputTransformation.a = this.a.multiplyAndReturnNew(
                    inputTransformation.a);

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
    public final void setTransformationFromPoints(
            final Point3D inputPoint1, final Point3D inputPoint2, final Point3D inputPoint3,
            final Point3D inputPoint4, final Point3D outputPoint1, final Point3D outputPoint2,
            final Point3D outputPoint3, final Point3D outputPoint4) throws CoincidentPointsException {

        // normalize points to increase accuracy
        inputPoint1.normalize();
        inputPoint2.normalize();
        inputPoint3.normalize();
        inputPoint4.normalize();

        outputPoint1.normalize();
        outputPoint2.normalize();
        outputPoint3.normalize();
        outputPoint4.normalize();

        // matrix of homogeneous linear system of equations.
        // There are 13 unknowns and 12 equations (3 for each pair of
        // corresponding points)
        Matrix m = null;
        try {
            // build matrix initialized to zero
            m = new Matrix(12, 13);

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

            double tmp = oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW;
            double norm = Math.sqrt(tmp + oXiW * oXiW);

            m.setElementAt(0, 0, oWiX / norm);
            m.setElementAt(0, 1, oWiY / norm);
            m.setElementAt(0, 2, oWiZ / norm);
            m.setElementAt(0, 9, oWiW / norm);
            m.setElementAt(0, 12, -oXiW / norm);

            norm = Math.sqrt(tmp + oYiW * oYiW);

            m.setElementAt(1, 3, oWiX / norm);
            m.setElementAt(1, 4, oWiY / norm);
            m.setElementAt(1, 5, oWiZ / norm);
            m.setElementAt(1, 10, oWiW / norm);
            m.setElementAt(1, 12, -oYiW / norm);

            norm = Math.sqrt(tmp + oZiW * oZiW);

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

            tmp = oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW;
            norm = Math.sqrt(tmp + oXiW * oXiW);

            m.setElementAt(3, 0, oWiX / norm);
            m.setElementAt(3, 1, oWiY / norm);
            m.setElementAt(3, 2, oWiZ / norm);
            m.setElementAt(3, 9, oWiW / norm);
            m.setElementAt(3, 12, -oXiW / norm);

            norm = Math.sqrt(tmp + oYiW * oYiW);

            m.setElementAt(4, 3, oWiX / norm);
            m.setElementAt(4, 4, oWiY / norm);
            m.setElementAt(4, 5, oWiZ / norm);
            m.setElementAt(4, 10, oWiW / norm);
            m.setElementAt(4, 12, -oYiW / norm);

            norm = Math.sqrt(tmp + oZiW * oZiW);

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

            tmp = oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW;
            norm = Math.sqrt(tmp + oXiW * oXiW);

            m.setElementAt(6, 0, oWiX / norm);
            m.setElementAt(6, 1, oWiY / norm);
            m.setElementAt(6, 2, oWiZ / norm);
            m.setElementAt(6, 9, oWiW / norm);
            m.setElementAt(6, 12, -oXiW / norm);

            norm = Math.sqrt(tmp + oYiW * oYiW);

            m.setElementAt(7, 3, oWiX / norm);
            m.setElementAt(7, 4, oWiY / norm);
            m.setElementAt(7, 5, oWiZ / norm);
            m.setElementAt(7, 10, oWiW / norm);
            m.setElementAt(7, 12, -oYiW / norm);

            norm = Math.sqrt(tmp + oZiW * oZiW);

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

            tmp = oWiX * oWiX + oWiY * oWiY + oWiZ * oWiZ + oWiW * oWiW;
            norm = Math.sqrt(tmp + oXiW * oXiW);

            m.setElementAt(9, 0, oWiX / norm);
            m.setElementAt(9, 1, oWiY / norm);
            m.setElementAt(9, 2, oWiZ / norm);
            m.setElementAt(9, 9, oWiW / norm);
            m.setElementAt(9, 12, -oXiW / norm);

            norm = Math.sqrt(tmp + oYiW * oYiW);

            m.setElementAt(10, 3, oWiX / norm);
            m.setElementAt(10, 4, oWiY / norm);
            m.setElementAt(10, 5, oWiZ / norm);
            m.setElementAt(10, 10, oWiW / norm);
            m.setElementAt(10, 12, -oYiW / norm);

            norm = Math.sqrt(tmp + oZiW * oZiW);

            m.setElementAt(11, 6, oWiX / norm);
            m.setElementAt(11, 7, oWiY / norm);
            m.setElementAt(11, 8, oWiZ / norm);
            m.setElementAt(11, 11, oWiW / norm);
            m.setElementAt(11, 12, -oZiW / norm);
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
            if (decomposer.getRank() < 12) {
                throw new CoincidentPointsException();
            }
            //V is 13x13
            v = decomposer.getV();

            // last column of V will contain parameters of transformation
            final double value = v.getElementAt(12, 12);
            a.setElementAt(0, 0, v.getElementAt(0, 12) / value);
            a.setElementAt(0, 1, v.getElementAt(1, 12) / value);
            a.setElementAt(0, 2, v.getElementAt(2, 12) / value);
            a.setElementAt(1, 0, v.getElementAt(3, 12) / value);
            a.setElementAt(1, 1, v.getElementAt(4, 12) / value);
            a.setElementAt(1, 2, v.getElementAt(5, 12) / value);
            a.setElementAt(2, 0, v.getElementAt(6, 12) / value);
            a.setElementAt(2, 1, v.getElementAt(7, 12) / value);
            a.setElementAt(2, 2, v.getElementAt(8, 12) / value);

            translation[0] = v.getElementAt(9, 12) / value;
            translation[1] = v.getElementAt(10, 12) / value;
            translation[2] = v.getElementAt(11, 12) / value;

        } catch (final AlgebraException e) {
            throw new CoincidentPointsException(e);
        }
    }

    /**
     * Estimates this transformation internal parameters by using 4
     * corresponding original and transformed planes.
     *
     * @param inputPlane1  1st input plane.
     * @param inputPlane2  2nd input plane.
     * @param inputPlane3  3rd input plane.
     * @param inputPlane4  4th input plane.
     * @param outputPlane1 1st transformed plane corresponding to 1st input
     *                     plane.
     * @param outputPlane2 2nd transformed plane corresponding to 2nd input
     *                     plane.
     * @param outputPlane3 3rd transformed plane corresponding to 3rd input
     *                     plane.
     * @param outputPlane4 4th transformed plane corresponding to 4th input
     *                     plane.
     * @throws CoincidentPlanesException raised if transformation cannot be
     *                                   estimated for some reason (plane configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    public final void setTransformationFromPlanes(
            final Plane inputPlane1, final Plane inputPlane2, final Plane inputPlane3,
            final Plane inputPlane4, final Plane outputPlane1, final Plane outputPlane2,
            final Plane outputPlane3, final Plane outputPlane4) throws CoincidentPlanesException {

        // normalize points to increase accuracy
        inputPlane1.normalize();
        inputPlane2.normalize();
        inputPlane3.normalize();
        inputPlane4.normalize();

        outputPlane1.normalize();
        outputPlane2.normalize();
        outputPlane3.normalize();
        outputPlane4.normalize();

        // matrix of homogeneous linear system of equations.
        // There are 13 unknowns and 12 equations (3 for each pair of
        // corresponding points)
        Matrix m = null;
        try {
            // build matrix initialized to zero
            m = new Matrix(12, 13);

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


            double tmp = oDiA * oDiA + oDiB * oDiB + oDiC * oDiC;
            double norm = Math.sqrt(tmp +
                    oAiA * oAiA + oAiB * oAiB + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(0, 0, oDiA / norm);
            m.setElementAt(0, 1, oDiB / norm);
            m.setElementAt(0, 2, oDiC / norm);
            m.setElementAt(0, 9, -oAiA / norm);
            m.setElementAt(0, 10, -oAiB / norm);
            m.setElementAt(0, 11, -oAiC / norm);
            m.setElementAt(0, 12, -oAiD / norm);

            norm = Math.sqrt(tmp +
                    oBiA * oBiA + oBiB * oBiB + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(1, 3, oDiA / norm);
            m.setElementAt(1, 4, oDiB / norm);
            m.setElementAt(1, 5, oDiC / norm);
            m.setElementAt(1, 9, -oBiA / norm);
            m.setElementAt(1, 10, -oBiB / norm);
            m.setElementAt(1, 11, -oBiC / norm);
            m.setElementAt(1, 12, -oBiD / norm);

            norm = Math.sqrt(tmp +
                    oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(2, 6, oDiA / norm);
            m.setElementAt(2, 7, oDiB / norm);
            m.setElementAt(2, 8, oDiC / norm);
            m.setElementAt(2, 9, -oCiA / norm);
            m.setElementAt(2, 10, -oCiB / norm);
            m.setElementAt(2, 11, -oCiC / norm);
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

            tmp = oDiA * oDiA + oDiB * oDiB + oDiC * oDiC;
            norm = Math.sqrt(tmp +
                    oAiA * oAiA + oAiB * oAiB + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(3, 0, oDiA / norm);
            m.setElementAt(3, 1, oDiB / norm);
            m.setElementAt(3, 2, oDiC / norm);
            m.setElementAt(3, 9, -oAiA / norm);
            m.setElementAt(3, 10, -oAiB / norm);
            m.setElementAt(3, 11, -oAiC / norm);
            m.setElementAt(3, 12, -oAiD / norm);

            norm = Math.sqrt(tmp +
                    oBiA * oBiA + oBiB * oBiB + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(4, 3, oDiA / norm);
            m.setElementAt(4, 4, oDiB / norm);
            m.setElementAt(4, 5, oDiC / norm);
            m.setElementAt(4, 9, -oBiA / norm);
            m.setElementAt(4, 10, -oBiB / norm);
            m.setElementAt(4, 11, -oBiC / norm);
            m.setElementAt(4, 12, -oBiD / norm);

            norm = Math.sqrt(tmp +
                    oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(5, 6, oDiA / norm);
            m.setElementAt(5, 7, oDiB / norm);
            m.setElementAt(5, 8, oDiC / norm);
            m.setElementAt(5, 9, -oCiA / norm);
            m.setElementAt(5, 10, -oCiB / norm);
            m.setElementAt(5, 11, -oCiC / norm);
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

            tmp = oDiA * oDiA + oDiB * oDiB + oDiC * oDiC;
            norm = Math.sqrt(tmp +
                    oAiA * oAiA + oAiB * oAiB + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(6, 0, oDiA / norm);
            m.setElementAt(6, 1, oDiB / norm);
            m.setElementAt(6, 2, oDiC / norm);
            m.setElementAt(6, 9, -oAiA / norm);
            m.setElementAt(6, 10, -oAiB / norm);
            m.setElementAt(6, 11, -oAiC / norm);
            m.setElementAt(6, 12, -oAiD / norm);

            norm = Math.sqrt(tmp +
                    oBiA * oBiA + oBiB * oBiB + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(7, 3, oDiA / norm);
            m.setElementAt(7, 4, oDiB / norm);
            m.setElementAt(7, 5, oDiC / norm);
            m.setElementAt(7, 9, -oBiA / norm);
            m.setElementAt(7, 10, -oBiB / norm);
            m.setElementAt(7, 11, -oBiC / norm);
            m.setElementAt(7, 12, -oBiD / norm);

            norm = Math.sqrt(tmp +
                    oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(8, 6, oDiA / norm);
            m.setElementAt(8, 7, oDiB / norm);
            m.setElementAt(8, 8, oDiC / norm);
            m.setElementAt(8, 9, -oCiA / norm);
            m.setElementAt(8, 10, -oCiB / norm);
            m.setElementAt(8, 11, -oCiC / norm);
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

            tmp = oDiA * oDiA + oDiB * oDiB + oDiC * oDiC;
            norm = Math.sqrt(tmp +
                    oAiA * oAiA + oAiB * oAiB + oAiC * oAiC + oAiD * oAiD);

            m.setElementAt(9, 0, oDiA / norm);
            m.setElementAt(9, 1, oDiB / norm);
            m.setElementAt(9, 2, oDiC / norm);
            m.setElementAt(9, 9, -oAiA / norm);
            m.setElementAt(9, 10, -oAiB / norm);
            m.setElementAt(9, 11, -oAiC / norm);
            m.setElementAt(9, 12, -oAiD / norm);

            norm = Math.sqrt(tmp +
                    oBiA * oBiA + oBiB * oBiB + oBiC * oBiC + oBiD * oBiD);

            m.setElementAt(10, 3, oDiA / norm);
            m.setElementAt(10, 4, oDiB / norm);
            m.setElementAt(10, 5, oDiC / norm);
            m.setElementAt(10, 9, -oBiA / norm);
            m.setElementAt(10, 10, -oBiB / norm);
            m.setElementAt(10, 11, -oBiC / norm);
            m.setElementAt(10, 12, -oBiD / norm);

            norm = Math.sqrt(tmp +
                    oCiA * oCiA + oCiB * oCiB + oCiC * oCiC + oCiD * oCiD);

            m.setElementAt(11, 6, oDiA / norm);
            m.setElementAt(11, 7, oDiB / norm);
            m.setElementAt(11, 8, oDiC / norm);
            m.setElementAt(11, 9, -oCiA / norm);
            m.setElementAt(11, 10, -oCiB / norm);
            m.setElementAt(11, 11, -oCiC / norm);
            m.setElementAt(11, 12, -oCiD / norm);
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
            if (decomposer.getRank() < 12) {
                throw new CoincidentPlanesException();
            }
            // V is 13x13
            v = decomposer.getV();

            // last column of V will contain parameters of transformation
            final double value = v.getElementAt(12, 12);

            final Matrix invTransA = new Matrix(AffineParameters3D.INHOM_COORDS,
                    AffineParameters3D.INHOM_COORDS);
            // copy former 9 elements of 13th column of V into a in row order
            invTransA.setSubmatrix(0, 0, 2, 2,
                    v.getSubmatrixAsArray(0, 12, 8, 12),
                    false);
            // normalize by scale value
            invTransA.multiplyByScalar(1.0 / value);

            // initially a contains the inverse of its transpose, so to obtain a we need
            // to transpose it and invert it
            invTransA.transpose();
            final Matrix a1 = Utils.inverse(invTransA);

            final Matrix invt = new Matrix(1, 3);
            invt.setSubmatrix(0, 0, 0, 2,
                    v.getSubmatrixAsArray(9, 12, 11, 12),
                    false);
            // normalize by scale value (we need to change sign as well)
            invt.multiplyByScalar(-1.0 / value);
            invt.transpose();

            final Matrix t = a1.multiplyAndReturnNew(invt);

            this.a = a1;
            this.translation = t.getBuffer();
        } catch (final AlgebraException e) {
            throw new CoincidentPlanesException(e);
        }
    }

    /**
     * Estimates this transformation internal parameters by using provided 2
     * corresponding original and transformed lines.
     *
     * @param inputLine1  1st input line.
     * @param inputLine2  2nd input line.
     * @param outputLine1 1st transformed line corresponding to 1st input line.
     * @param outputLine2 2nd transformed line corresponding to 2nd input line.
     * @throws CoincidentLinesException Raised if transformation cannot be
     *                                  estimated for some reason (line configuration degeneracy, duplicate lines
     *                                  or numerical instabilities).
     */
    public final void setTransformationFromLines(
            final Line3D inputLine1, final Line3D inputLine2,
            final Line3D outputLine1, final Line3D outputLine2)
            throws CoincidentLinesException {
        try {
            setTransformationFromPlanes(inputLine1.getPlane1(),
                    inputLine1.getPlane2(), inputLine2.getPlane1(),
                    inputLine2.getPlane2(), outputLine1.getPlane1(),
                    outputLine1.getPlane2(), outputLine2.getPlane1(),
                    outputLine2.getPlane2());
        } catch (final CoincidentPlanesException e) {
            throw new CoincidentLinesException(e);
        }
    }
}
