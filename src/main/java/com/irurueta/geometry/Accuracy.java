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
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.statistics.NormalDist;

import java.io.Serializable;

/**
 * Base class representing the confidence of provided accuracy from a covariance matrix
 * expressed in the distance unit of such matrix.
 * This class contains utility methods to convert covariance matrices into geometric figures
 * with the requested confidence.
 */
@SuppressWarnings("WeakerAccess")
public abstract class Accuracy implements Serializable {

    /**
     * Default standard deviation factor to account for a given accuracy confidence.
     * Typically a factor of 2.0 will be used, which means that accuracy of position will
     * be drawn as an ellipse of size equal to one time the standard deviation. Assuming a
     * Gaussian distribution this is equivalent to providing a 95.44% of confidence on provided
     * accuracy.
     */
    public static final double DEFAULT_STANDARD_DEVIATION_FACTOR = 2.0;

    /**
     * Covariance matrix representing the accuracy of an estimated position.
     */
    protected Matrix mCovarianceMatrix;

    /**
     * Standard deviation factor to account for a given accuracy confidence.
     * Typically a factor of 2.0 will be used, which means that accuracy of a point or
     * measure will be drawn as a geometric figure (either an ellipse in 2D or an
     * ellipsoid in 3D) equal to two times the standard deviation. Assuming a
     * Gaussian distribution this is equivalent to providing a 95.44% of confidence on
     * provided accuracy.
     */
    protected double mStandardDeviationFactor = DEFAULT_STANDARD_DEVIATION_FACTOR;

    /**
     * Confidence of provided accuracy of a point for a value located up to the standard
     * deviation factor distance from the mean.
     * This is expressed as a value between 0 and 1, where 1 indicates a 100% confidence that
     * the real position is within provided accuracy.
     */
    protected double mConfidence = 2.0 * NormalDist.cdf(
            DEFAULT_STANDARD_DEVIATION_FACTOR, 0.0, 1.0) - 1.0;

    /**
     * Singular value decomposer to find principal axes of provided covariance matrix.
     */
    protected SingularValueDecomposer mSvdDecomposer = new SingularValueDecomposer();

    /**
     * Square root of singular values of decomposed covariance matrix.
     */
    protected double[] mSqrtSingularValues;

    /**
     * Orthonormal matrix representing a rotation after decomposing covariance matrix.
     */
    protected Matrix mU;

    /**
     * Minimum square root of singular value of decomposed covariance matrix. Can be used to
     * determine the smallest accuracy on a geometric figure (i.e. the shortest semi-axis on
     * an ellipse or an ellipsoid).
     */
    private double mMinSqrtSingularValue = Double.POSITIVE_INFINITY;

    /**
     * Maximum square root of singular value of decomposed covariance matrix. Can be used to
     * determine the largest accuracy on a geometric figure (i.e. the largest semi-axis on
     * an ellipse or an ellipsoid).
     */
    private double mMaxSqrtSingularValue = Double.POSITIVE_INFINITY;

    /**
     * Average square root of singular value of decomposed covariance matrix. Can be used to
     * determine the average accuracy on a geomtric figure (i.e. the average semi-axis on
     * an ellipse or an ellipsoid).
     */
    private double mAvgSqrtSingularValue = Double.POSITIVE_INFINITY;

    /**
     * Constructor.
     */
    public Accuracy() { }

    /**
     * Constructor.
     * @param covarianceMatrix covariance matrix to be set. Must be NxN where N
     *                         is the number of dimensions and positive definite.
     * @throws IllegalArgumentException if provided matrix is not square (it must also be
     * positive definite to be properly converted to a geometric figure - e.g. an ellipse or
     * an ellipsoid).
     * @throws NonSymmetricPositiveDefiniteMatrixException if provided matrix is not symmetric and
     * positive definite.
     */
    public Accuracy(Matrix covarianceMatrix) throws IllegalArgumentException,
            NonSymmetricPositiveDefiniteMatrixException {
        setCovarianceMatrix(covarianceMatrix);
    }

    /**
     * Constructor.
     * @param confidence confidence of provided accuracy of an estimated position.
     * @throws IllegalArgumentException if provided value is not within 0 and 1.
     */
    public Accuracy(double confidence) throws IllegalArgumentException {
        setConfidence(confidence);
    }

    /**
     * Constructor.
     * @param covarianceMatrix covariance matrix to be set. Must be NxN where N
     *                         is the number of dimensions and positive definite.
     * @param confidence confidence of provided accuracy of an estimated position.
     * @throws IllegalArgumentException if provided matrix is not square (it must also be
     * positive definite to be properly converted to a geometric figure - e.g. an ellipse or
     * an ellipsoid), or if provided confidence value is not within 0 and 1.
     * @throws NonSymmetricPositiveDefiniteMatrixException if provided matrix is not symmetric and
     * positive definite.
     */
    public Accuracy(Matrix covarianceMatrix, double confidence)
            throws IllegalArgumentException, NonSymmetricPositiveDefiniteMatrixException {
        setCovarianceMatrix(covarianceMatrix);
        setConfidence(confidence);
    }

    /**
     * Gets covariance matrix representing the accuracy of an estimated point or measure.
     * @return covariance matrix representing the accuracy of an estimated point or measure.
     */
    public Matrix getCovarianceMatrix() {
        return mCovarianceMatrix;
    }

    /**
     * Sets covariance matrix representing the accuracy of an estimated point or measure.
     * @param covarianceMatrix covariance matrix representing the accuracy of an estimated
     *                         point or measure.
     * @throws IllegalArgumentException if provided matrix is not square (it must also be
     * positive definite to be properly converted to a geometric figure - e.g. an ellipse
     * or an ellipsoid).
     * @throws NonSymmetricPositiveDefiniteMatrixException if provided matrix is not symmetric and
     * positive definite.
     */
    public void setCovarianceMatrix(Matrix covarianceMatrix)
            throws IllegalArgumentException, NonSymmetricPositiveDefiniteMatrixException {
        int dims = getNumberOfDimensions();
        if (covarianceMatrix.getRows() != dims || covarianceMatrix.getColumns() != dims) {
            throw new IllegalArgumentException();
        }

        try {
            mSvdDecomposer.setInputMatrix(covarianceMatrix);
            mSvdDecomposer.decompose();

            double[] singularValues = mSvdDecomposer.getSingularValues();
            double [] sqrtSingularValues = new double[dims];

            double minSqrtSingularValue = Double.MAX_VALUE;
            double maxSqrtSingularValue = -Double.MAX_VALUE;
            double avgSqrtSingularValue = 0.0;
            int i = 0;
            for (double singularValue : singularValues) {
                if (singularValue < 0.0) {
                    //matrix is not positive definite
                    throw new NonSymmetricPositiveDefiniteMatrixException();
                }

                double sqrtSingularValue = Math.sqrt(singularValue);
                if (sqrtSingularValue < minSqrtSingularValue) {
                    minSqrtSingularValue = sqrtSingularValue;
                }
                if (sqrtSingularValue > maxSqrtSingularValue) {
                    maxSqrtSingularValue = sqrtSingularValue;
                }
                avgSqrtSingularValue += sqrtSingularValue / dims;

                sqrtSingularValues[i] = sqrtSingularValue;
                i++;
            }

            mSqrtSingularValues = sqrtSingularValues;
            mU = mSvdDecomposer.getU();

            mMinSqrtSingularValue = minSqrtSingularValue;
            mMaxSqrtSingularValue = maxSqrtSingularValue;
            mAvgSqrtSingularValue = avgSqrtSingularValue;

            mCovarianceMatrix = covarianceMatrix;
        } catch (AlgebraException e) {
            throw new NonSymmetricPositiveDefiniteMatrixException(e);
        }
    }

    /**
     * Gets standard deviation factor to account for a given accuracy confidence.
     * Typically a factor of 2.0 will be used, which means that accuracy can be drawn as
     * a geometric figure of size equal to 2 times the standard deviation. Assuming a
     * Gaussian distribution this is equivalent to providing a 95.44% confidence on provided
     * accuracy.
     * @return standard deviation factor.
     */
    public double getStandardDeviationFactor() {
        return mStandardDeviationFactor;
    }

    /**
     * Sets standard deviation factor to account for a given accuracy confidence.
     * Typically a factor of 2.0 will be used, which means that accuracy can be drawn as
     * a geometric figure of size equal to 2 times the standard deviation. Assuming a
     * Gaussian distribution this is equivalent to providing a 95.44% confidence on provided
     * accuracy.
     * @param standardDeviationFactor standard deviation factor to be set.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setStandardDeviationFactor(double standardDeviationFactor)
            throws IllegalArgumentException {
        if (standardDeviationFactor <= 0.0) {
            throw new IllegalArgumentException();
        }
        mStandardDeviationFactor = standardDeviationFactor;
        mConfidence = 2.0 * NormalDist.cdf(mStandardDeviationFactor, 0.0, 1.0) - 1.0;
    }

    /**
     * Gets confidence of provided accuracy of estimated point or measure.
     * This is expressed as a value between 0 and 1, where 1 indicates a 100% confidence
     * that the real point or measure is within provided accuracy.
     * @return confidence of provided accuracy of estimated point or measure.
     */
    public double getConfidence() {
        return mConfidence;
    }

    /**
     * Sets confidence of provided accuracy of estimated point or measure.
     * This is expressed as a value between 0 and 1, where 1 indicates a 100% confidence
     * that the real point or measure is within provided accuracy.
     * @param confidence confidence of provided accuracy of estimated point or measure.
     * @throws IllegalArgumentException if provided value is not within 0 and 1.
     */
    public void setConfidence(double confidence) throws IllegalArgumentException {
        if (confidence < 0.0 || confidence > 1.0) {
            throw new IllegalArgumentException();
        }
        mConfidence = confidence;
        mStandardDeviationFactor = NormalDist.invcdf(
                (confidence + 1.0) / 2.0, 0.0, 1.0);
    }

    /**
     * Gets smallest (best) accuracy in any direction (i.e. either 2D or 3D).
     * This value is represented by the smallest semi axis representing the ellipse or ellipsoid of accuracy.
     * @return smallest accuracy in any direction.
     */
    public double getSmallestAccuracy() {
        return mMinSqrtSingularValue * mStandardDeviationFactor;
    }

    /**
     * Gets largest (worse) accuracy in any direction (i.e. either 2D or 3D).
     * This value is represented by the largest semi axis representing the ellipse or ellipsoid of accuracy.
     * @return largest accuracy in any direction.
     */
    public double getLargestAccuracy() {
        return mMaxSqrtSingularValue * mStandardDeviationFactor;
    }

    /**
     * Gets average accuracy among all directions.
     * This value is equal to the average value of all semi axes representing the ellipse or ellipsoid of
     * accuracy.
     * @return average accuracy among all directions.
     */
    public double getAverageAccuracy() {
        return mAvgSqrtSingularValue * mStandardDeviationFactor;
    }

    /**
     * Gets number of dimensions.
     * This is equal to 2 for 2D, and to 3 for 3D.
     * @return number of dimensions.
     */
    public abstract int getNumberOfDimensions();
}
