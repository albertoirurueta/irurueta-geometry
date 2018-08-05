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

import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;

/**
 * Contains methods to convert covariance matrices into ellipsoids representing accuracy
 * with requested confidence.
 */
@SuppressWarnings("WeakerAccess")
public class AccuracyPoint3D extends AccuracyPoint {

    /**
     * Constructor.
     */
    public AccuracyPoint3D() {
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
    public AccuracyPoint3D(Matrix covarianceMatrix) throws IllegalArgumentException,
            NonSymmetricPositiveDefiniteMatrixException {
        super(covarianceMatrix);
    }

    /**
     * Constructor.
     * @param confidence confidence of provided accuracy of an estimated position.
     * @throws IllegalArgumentException if provided value is not within 0 and 1.
     */
    public AccuracyPoint3D(double confidence) throws IllegalArgumentException {
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
    public AccuracyPoint3D(Matrix covarianceMatrix, double confidence)
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
        double[] semiAxesLengths = ArrayUtils.multiplyByScalarAndReturnNew(
                mSingularValues, mStandardDeviationFactor);
        Rotation3D rotation = new MatrixRotation3D(mU);
        return new Ellipsoid(Point3D.create(), semiAxesLengths, rotation);
    }

    /**
     * Flattens accuracy representation to 2D by taking into account only x and y coordinates and
     * ignoring variance related to z coordinates.
     * @return flattenned accuracy representation in 2D.
     * @throws NullPointerException if covariance matrix is not defined.
     * @throws NonSymmetricPositiveDefiniteMatrixException if covariance matrix is not symmetric and
     * positive definite.
     */
    public AccuracyPoint2D flattenTo2D() throws NullPointerException,
            NonSymmetricPositiveDefiniteMatrixException {
        //consider as covariance matrix only the topleft submatrix related to x and y coordinates
        Matrix subCovarianceMatrix = mCovarianceMatrix.getSubmatrix(
                0, 0, 1, 1);
        return new AccuracyPoint2D(subCovarianceMatrix, mConfidence);
    }
}
