/*
 * Copyright (C) 2018 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;

/**
 * Contains methods to convert covariance matrices into ellipsoids representing accuracy
 * with requested confidence.
 */
@SuppressWarnings("WeakerAccess")
public class Accuracy3D extends Accuracy {

    /**
     * Constructor.
     */
    public Accuracy3D() {
        super();
    }

    /**
     * Constructor.
     * @param covarianceMatrix covariance matrix to be set. Must be 3x3 and positive
     *                         definite.
     * @throws IllegalArgumentException if provided matrix is not square (it must also be
     * positive definite to be properly converted to an ellipsoid).
     * @throws NonSymmetricPositiveDefiniteMatrixException if provided matrix is not symmetric and
     * positive definite.
     */
    public Accuracy3D(Matrix covarianceMatrix) throws IllegalArgumentException,
            NonSymmetricPositiveDefiniteMatrixException {
        super(covarianceMatrix);
    }

    /**
     * Constructor.
     * @param confidence confidence of provided accuracy of an estimated position.
     * @throws IllegalArgumentException if provided value is not within 0 and 1.
     */
    public Accuracy3D(double confidence) throws IllegalArgumentException {
        super(confidence);
    }

    /**
     * Constructor.
     * @param covarianceMatrix covariance matrix to be set. Must be 3x3 and positive
     *                         definite.
     * @param confidence confidence of provided accuracy of an estimated position.
     * @throws IllegalArgumentException if provided matrix is not square (it must also be
     * positive definite to be properly converted to an ellipsoid, or if provided
     * confidence value is not within 0 and 1.
     * @throws NonSymmetricPositiveDefiniteMatrixException if provided matrix is not symmetric and
     * positive definite.
     */
    public Accuracy3D(Matrix covarianceMatrix, double confidence)
            throws IllegalArgumentException, NonSymmetricPositiveDefiniteMatrixException {
        super(covarianceMatrix, confidence);
    }

    /**
     * Gets number of dimensions.
     * @return always returns 3.
     */
    @Override
    public int getNumberOfDimensions() {
        return Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH;
    }

    /**
     * Converts provided covariance matrix into a 3D ellipsoid taking into account current
     * confidence and standard deviation factor.
     * @return ellipsoid representing accuracy of covariance matrix with current confidence and
     * standard deviation factor.
     * @throws NullPointerException if covariance matrix has not been provided yet.
     * @throws InvalidRotationMatrixException if rotation cannot be properly determined.
     */
    public Ellipsoid toEllipsoid() throws NullPointerException, InvalidRotationMatrixException {
        return toEllipsoid(mStandardDeviationFactor);
    }

    /**
     * Flattens accuracy representation to 2D by taking into account only x and y coordinates and
     * ignoring variance related to z coordinates.
     * @return flattenned accuracy representation in 2D.
     * @throws NullPointerException if covariance matrix is not defined.
     * @throws GeometryException if intersection cannot be computed.
     */
    public Accuracy2D flattenTo2D() throws NullPointerException,
            GeometryException {
        //get intersected ellipse for unitary standard deviation
        Ellipse ellipse = intersectWithPlane(1.0);

        double semiMajorAxis = ellipse.getSemiMajorAxis();
        double semiMinorAxis = ellipse.getSemiMinorAxis();
        Rotation2D rotation = ellipse.getRotation();

        Matrix u = rotation.asInhomogeneousMatrix();
        Matrix s2 = Matrix.diagonal(new double[] {
                semiMajorAxis * semiMajorAxis,
                semiMinorAxis * semiMinorAxis}) ;

        try {
            //compute covariance as the squared matrix M = U*S*V'
            //Hence: M*M' = U*S*V'*(U*S*V')' = U*S*V'*V'*S*U' = U*S^2*U'

            s2.multiply(u);
            u.multiply(s2);

            return new Accuracy2D(u, mConfidence);
        } catch (AlgebraException e) {
            throw new GeometryException(e);
        }
    }

    /**
     * Intersects ellipsoid representing this accuracy with horizontal xy plane.
     * @return intersected ellipse.
     * @throws NullPointerException if covariance matrix is not defined.
     * @throws GeometryException if intersection cannot be computed.
     */
    public Ellipse intersectWithPlane() throws NullPointerException,
            GeometryException {
        return intersectWithPlane(mStandardDeviationFactor);
    }

    /**
     * Converts provided covariance matrix into a 3D ellipsoid taking into account current
     * confidence and standard deviation factor.
     * @param standardDeviationFactor standard deviation factor.
     * @return ellipsoid representinc accuracy of covariance matrix with provided standard
     * deviation factor.
     * @throws NullPointerException if covariance matrix has not been provided yet.
     * @throws InvalidRotationMatrixException if rotation cannot be properly determined.
     */
    private Ellipsoid toEllipsoid(double standardDeviationFactor) throws NullPointerException,
            InvalidRotationMatrixException {
        double[] semiAxesLengths = ArrayUtils.multiplyByScalarAndReturnNew(
                mSqrtSingularValues, standardDeviationFactor);
        Rotation3D rotation = new MatrixRotation3D(mU);
        return new Ellipsoid(Point3D.create(), semiAxesLengths, rotation);
    }

    /**
     * Intersects ellipsoid representing this accuracy with provided standard
     * deviation factor and with horizontal xy plane.
     * @param standardDeviationFactor standard deviation factor.
     * @return intersected ellipse.
     * @throws NullPointerException if covariance matrix is not defined.
     * @throws GeometryException if intersection cannot be computed.
     */
    private Ellipse intersectWithPlane(double standardDeviationFactor)
            throws NullPointerException, GeometryException {
        Ellipsoid ellipsoid = toEllipsoid(standardDeviationFactor);
        Quadric quadric = ellipsoid.toQuadric();

        //create horizontal xy plane located at ellipsoid center
        Point3D center = ellipsoid.getCenter();
        double[] directorVector = new double[] { 0.0, 0.0, 1.0 };
        Plane plane = new Plane(center, directorVector);

        Conic conic = quadric.intersectWith(plane);

        return new Ellipse(conic);
    }
}