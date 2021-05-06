/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.EuclideanTransformation2D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Rotation2D;

import java.util.List;

/**
 * Estimator of a 2D euclidean transformation based on point correspondences.
 * This estimator uses Kabsch algorithm on 2D.
 * A minimum of 3 non-coincident matched 2D input/output points is required for
 * estimation.
 * For some point configurations 2 points are enough to find a valid solution.
 * If more points are provided an LMSE (Least Mean Squared Error) solution will
 * be found.
 * Based on:
 * https://en.wikipedia.org/wiki/Kabsch_algorithm
 */
@SuppressWarnings("DuplicatedCode")
public class EuclideanTransformation2DEstimator {

    /**
     * Minimum required number of matched points.
     */
    public static final int MINIMUM_SIZE = 3;

    /**
     * For some point configurations a solution can be found with only 2 points.
     */
    public static final int WEAK_MINIMUM_SIZE = 2;

    /**
     * 2D input points.
     */
    private List<Point2D> mInputPoints;

    /**
     * 2D output points.
     */
    private List<Point2D> mOutputPoints;

    /**
     * Listener to be notified of events such as when estimation starts or ends.
     */
    private EuclideanTransformation2DEstimatorListener mListener;

    /**
     * Indicates whether estimation can start with only 2 points or not.
     * True allows 2 points, false requires 3.
     */
    private boolean mWeakMinimumSizeAllowed;

    /**
     * Indicates if this estimator is locked because an estimation is being
     * computed.
     */
    private boolean mLocked;

    /**
     * Constructor.
     */
    public EuclideanTransformation2DEstimator() {
    }

    /**
     * Constructor.
     *
     * @param inputPoints  2D input points.
     * @param outputPoints 2D input points.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than 3.
     */
    public EuclideanTransformation2DEstimator(final List<Point2D> inputPoints,
                                              final List<Point2D> outputPoints) {
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts or ends.
     */
    public EuclideanTransformation2DEstimator(
            final EuclideanTransformation2DEstimatorListener listener) {
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts or ends.
     * @param inputPoints  2D input points.
     * @param outputPoints 2D output points.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than 3.
     */
    public EuclideanTransformation2DEstimator(
            final EuclideanTransformation2DEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints) {
        mListener = listener;
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Constructor.
     *
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     */
    public EuclideanTransformation2DEstimator(final boolean weakMinimumSizeAllowed) {
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
    }

    /**
     * Constructor.
     *
     * @param inputPoints            2D input points.
     * @param outputPoints           2D input points.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than 3.
     */
    public EuclideanTransformation2DEstimator(
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final boolean weakMinimumSizeAllowed) {
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Constructor.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts or ends.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     */
    public EuclideanTransformation2DEstimator(
            final EuclideanTransformation2DEstimatorListener listener,
            final boolean weakMinimumSizeAllowed) {
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
        mListener = listener;
    }

    /**
     * Constructor.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts or ends.
     * @param inputPoints            2D input points.
     * @param outputPoints           2D output points.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than 3.
     */
    public EuclideanTransformation2DEstimator(
            final EuclideanTransformation2DEstimatorListener listener,
            final List<Point2D> inputPoints, final List<Point2D> outputPoints,
            final boolean weakMinimumSizeAllowed) {
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
        mListener = listener;
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Returns list of input points to be used to estimate an euclidean 2D
     * transformation.
     * Each point in the list of input points must be matched with the
     * corresponding point in the list of output points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     *
     * @return list of input points to be used to estimate an euclidean
     * transformation.
     */
    public List<Point2D> getInputPoints() {
        return mInputPoints;
    }

    /**
     * Returns list of output points to be used to estimate an euclidean 2D
     * transformation.
     * Each point in the list of output points must be matched with the
     * corresponding point in the list of input points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than MINIMUM_SIZE.
     *
     * @return list of input points to be used to estimate an euclidean
     * transformation.
     */
    public List<Point2D> getOutputPoints() {
        return mOutputPoints;
    }

    /**
     * Sets list of points to be used to estimate an euclidean 2D
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than MINIMUM_SIZE.
     *
     * @param inputPoints  list of input points ot be used ot estimate an
     *                     euclidean 2D transformation.
     * @param outputPoints list of output points ot be used to estimate an
     *                     euclidean 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than MINIMUM_SIZE.
     * @throws LockedException          if estimator is locked because a computation is
     *                                  already in progress.
     */
    public void setPoints(final List<Point2D> inputPoints,
                          final List<Point2D> outputPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Returns reference to listener to be notified of events such as when
     * estimation starts or ends.
     *
     * @return listener to be notified of events.
     */
    public EuclideanTransformation2DEstimatorListener getListener() {
        return mListener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts or
     * ends.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(final EuclideanTransformation2DEstimatorListener listener)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mListener = listener;
    }

    /**
     * Indicates whether estimation can start with only 2 points or not.
     *
     * @return true allows 2 points, false requires 3.
     */
    public boolean isWeakMinimumSizeAllowed() {
        return mWeakMinimumSizeAllowed;
    }

    /**
     * Specifies whether estimation can start with only 2 points or not.
     *
     * @param weakMinimumSizeAllowed true allows 2 points, false requires 3.
     * @throws LockedException if estimator is locked.
     */
    public void setWeakMinimumSizeAllowed(final boolean weakMinimumSizeAllowed)
            throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        mWeakMinimumSizeAllowed = weakMinimumSizeAllowed;
    }

    /**
     * Required minimum number of point correspondences to start the estimation.
     * Can be either 2 or 3.
     *
     * @return minimum number of point correspondences.
     */
    public int getMinimumPoints() {
        return mWeakMinimumSizeAllowed ? WEAK_MINIMUM_SIZE : MINIMUM_SIZE;
    }

    /**
     * Indicates whether listener has been provided and is available for
     * retrieval.
     *
     * @return true if available, false otherwise.
     */
    public boolean isListenerAvailable() {
        return mListener != null;
    }

    /**
     * Indicates if this instance is locked because estimation is being
     * computed.
     *
     * @return true if locked, false otherwise.
     */
    public boolean isLocked() {
        return mLocked;
    }

    /**
     * Indicates if estimator is ready to start the euclidean 2D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched points) are provided
     * and a minimum of MINIMUM_SIZE points are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return mInputPoints != null && mOutputPoints != null &&
                mInputPoints.size() == mOutputPoints.size() &&
                mInputPoints.size() >= getMinimumPoints();
    }

    /**
     * Estimates an euclidean 2D transformation using the list of matched input
     * and output 2D points.
     * A minimum of 3 matched non-coincident points is required. If more points
     * are provided an LMSE (Least Mean Squared Error) solution will be found.
     *
     * @return estimated euclidean 2D transformation.
     * @throws LockedException           if estimator is locked.
     * @throws NotReadyException         if not enough data has been provided.
     * @throws CoincidentPointsException raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    public EuclideanTransformation2D estimate() throws LockedException,
            NotReadyException, CoincidentPointsException {
        final EuclideanTransformation2D result = new EuclideanTransformation2D();
        estimate(result);
        return result;
    }

    /**
     * Estimates an euclidean 2D transformation using the list of matched input
     * and output 2D points.
     * A minimum of 3 matched non-coincident points is required. If more points
     * are provided an LMSE (Least Mean Squared Error) solution will be found.
     *
     * @param result instance where result will be stored.
     * @throws LockedException           if estimator is locked.
     * @throws NotReadyException         if not enough data has been provided.
     * @throws CoincidentPointsException raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    public void estimate(final EuclideanTransformation2D result)
            throws LockedException, NotReadyException,
            CoincidentPointsException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            mLocked = true;

            if (mListener != null) {
                mListener.onEstimateStart(this);
            }

            final Matrix inCentroid = computeCentroid(mInputPoints);
            final Matrix outCentroid = computeCentroid(mOutputPoints);

            final Matrix m = new Matrix(
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);

            final int n = mInputPoints.size();
            Point2D inputPoint;
            Point2D outputPoint;
            final Matrix col = new Matrix(
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, 1);
            final Matrix row = new Matrix(1,
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);
            final Matrix tmp = new Matrix(
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH);
            for (int i = 0; i < n; i++) {
                inputPoint = mInputPoints.get(i);
                outputPoint = mOutputPoints.get(i);

                col.setElementAtIndex(0, inputPoint.getInhomX() -
                        inCentroid.getElementAtIndex(0));
                col.setElementAtIndex(1, inputPoint.getInhomY() -
                        inCentroid.getElementAtIndex(1));

                row.setElementAtIndex(0, outputPoint.getInhomX() -
                        outCentroid.getElementAtIndex(0));
                row.setElementAtIndex(1, outputPoint.getInhomY() -
                        outCentroid.getElementAtIndex(1));

                col.multiply(row, tmp);
                m.add(tmp);
            }

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            if (!mWeakMinimumSizeAllowed && decomposer.getNullity() > 0) {
                throw new CoincidentPointsException();
            }

            final Matrix u = decomposer.getU();
            final Matrix v = decomposer.getV();

            // rotation R = V*U^T
            u.transpose();
            v.multiply(u);

            if (Utils.det(v) < 0.0) {
                // multiply 2nd column of R by -1
                v.setElementAt(0, 1, -v.getElementAt(0, 1));
                v.setElementAt(1, 1, -v.getElementAt(1, 1));
            }

            final Rotation2D rotation = new Rotation2D(v);

            // translation
            final Matrix t = v.multiplyAndReturnNew(inCentroid);
            t.multiplyByScalar(-1.0);
            t.add(outCentroid);

            result.setRotation(rotation);
            result.setTranslation(t.getBuffer());

            if (mListener != null) {
                mListener.onEstimateEnd(this);
            }
        } catch (final AlgebraException | InvalidRotationMatrixException e) {
            throw new CoincidentPointsException(e);
        } finally {
            mLocked = false;
        }
    }

    /**
     * Computes centroid of provided list of points using inhomogeneous
     * coordinates.
     *
     * @param points list of points to compute centroid.
     * @return centroid.
     * @throws AlgebraException never thrown.
     */
    private static Matrix computeCentroid(final List<Point2D> points)
            throws AlgebraException {
        double x = 0.0;
        double y = 0.0;
        final double n = points.size();
        for (final Point2D p : points) {
            x += p.getInhomX() / n;
            y += p.getInhomY() / n;
        }

        final Matrix result = new Matrix(
                Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH, 1);
        result.setElementAtIndex(0, x);
        result.setElementAtIndex(1, y);
        return result;
    }

    /**
     * Internal method to set lists of points to be used to estimate an
     * euclidean 2D transformation.
     * This method does not check whether estimator is locked or not.
     *
     * @param inputPoints  list of input points to be used to estimate an
     *                     euclidean 2D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     euclidean 2D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than #getMinimumPoints.
     */
    private void internalSetPoints(final List<Point2D> inputPoints,
                                   final List<Point2D> outputPoints) {
        if (inputPoints.size() < getMinimumPoints()) {
            throw new IllegalArgumentException();
        }
        if (inputPoints.size() != outputPoints.size()) {
            throw new IllegalArgumentException();
        }
        mInputPoints = inputPoints;
        mOutputPoints = outputPoints;
    }
}
