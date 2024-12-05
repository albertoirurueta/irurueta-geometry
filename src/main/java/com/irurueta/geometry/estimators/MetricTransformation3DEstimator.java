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
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.CoincidentPointsException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.MetricTransformation3D;
import com.irurueta.geometry.Point3D;

import java.util.List;

/**
 * Estimator of a 3D metric transformation based on point correspondences.
 * A minimum of 4 non-coincident matched 3D input/output points is required for
 * estimation.
 * For some point configurations 3 points are enough to find a valid solution.
 * If more points are provided an LMSE (Least Mean Squared Error) solution will
 * be found.
 * Based on:
 * <a href="http://stackoverflow.com/questions/13432805/finding-translation-and-scale-on-two-sets-of-points-to-get-least-square-error-in">
 *     http://stackoverflow.com/questions/13432805/finding-translation-and-scale-on-two-sets-of-points-to-get-least-square-error-in
 * </a>
 */
@SuppressWarnings("DuplicatedCode")
public class MetricTransformation3DEstimator {

    /**
     * Minimum required number of matched points.
     */
    public static final int MINIMUM_SIZE = 4;

    /**
     * For some point configurations a solution can be found with only 3 points.
     */
    public static final int WEAK_MINIMUM_SIZE = 3;

    /**
     * 3D input points.
     */
    private List<Point3D> inputPoints;

    /**
     * 3D output points.
     */
    private List<Point3D> outputPoints;

    /**
     * Listener to be notified of events such as when estimation starts or ends.
     */
    private MetricTransformation3DEstimatorListener listener;

    /**
     * Indicates whether estimation can start with only 3 points or not.
     * True allows 3 points, false requires 4.
     */
    private boolean weakMinimumSizeAllowed;

    /**
     * Indicates if this estimator is locked because an estimation is being
     * computed.
     */
    private boolean locked;

    /**
     * Constructor.
     */
    public MetricTransformation3DEstimator() {
    }

    /**
     * Constructor.
     *
     * @param inputPoints  3D input points.
     * @param outputPoints 3D output points.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than 4.
     */
    public MetricTransformation3DEstimator(final List<Point3D> inputPoints, final List<Point3D> outputPoints) {
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Constructor.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts or ends.
     */
    public MetricTransformation3DEstimator(final MetricTransformation3DEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param listener     listener to be notified of events such as when estimation
     *                     starts or ends.
     * @param inputPoints  3D input points.
     * @param outputPoints 3D output points.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than 4.
     */
    public MetricTransformation3DEstimator(
            final MetricTransformation3DEstimatorListener listener,
            final List<Point3D> inputPoints, final List<Point3D> outputPoints) {
        this.listener = listener;
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Constructor.
     *
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     */
    public MetricTransformation3DEstimator(final boolean weakMinimumSizeAllowed) {
        this.weakMinimumSizeAllowed = weakMinimumSizeAllowed;
    }

    /**
     * Constructor.
     *
     * @param inputPoints            3D input points.
     * @param outputPoints           3D output points.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than 4.
     */
    public MetricTransformation3DEstimator(
            final List<Point3D> inputPoints, final List<Point3D> outputPoints, final boolean weakMinimumSizeAllowed) {
        this.weakMinimumSizeAllowed = weakMinimumSizeAllowed;
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Constructor.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts or ends.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     */
    public MetricTransformation3DEstimator(
            final MetricTransformation3DEstimatorListener listener, final boolean weakMinimumSizeAllowed) {
        this.weakMinimumSizeAllowed = weakMinimumSizeAllowed;
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param listener               listener to be notified of events such as when estimation
     *                               starts or ends.
     * @param inputPoints            3D input points.
     * @param outputPoints           3D output points.
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than 4.
     */
    public MetricTransformation3DEstimator(
            final MetricTransformation3DEstimatorListener listener,
            final List<Point3D> inputPoints, final List<Point3D> outputPoints, final boolean weakMinimumSizeAllowed) {
        this.weakMinimumSizeAllowed = weakMinimumSizeAllowed;
        this.listener = listener;
        internalSetPoints(inputPoints, outputPoints);
    }

    /**
     * Returns list of input points to be used to estimate a metric 3D
     * transformation.
     * Each point in the list of input points must be matched with the
     * corresponding point in the list of output points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than #getMinimumPoints.
     *
     * @return list of input points to be used to estimate a metric 3D
     * transformation.
     */
    public List<Point3D> getInputPoints() {
        return inputPoints;
    }

    /**
     * Returns list of output points to be used to estimate a metric 3D
     * transformation.
     * Each point in the list of output points must be matched with the
     * corresponding point in the list of input points located at the same
     * position. Hence, both input points and output points must have the same
     * size, and their size must be greater or equal than #getMinimumPoints.
     *
     * @return list of output points to be used to estimate a metric 3D
     * transformation.
     */
    public List<Point3D> getOutputPoints() {
        return outputPoints;
    }

    /**
     * Sets list of points to be used to estimate a metric 3D
     * transformation.
     * Points in the list located at the same position are considered to be
     * matched. Hence, both lists must have the same size, and their size must
     * be greater or equal than #getMinimumPoints.
     *
     * @param inputPoints  list of input points to be used to estimate an
     *                     Euclidean 3D transformation.
     * @param outputPoints list of output points to be used to estimate an
     *                     Euclidean 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than #getMinimumPoints.
     * @throws LockedException          if estimator is locked because a computation is
     *                                  already in progress.
     */
    public void setPoints(final List<Point3D> inputPoints, final List<Point3D> outputPoints) throws LockedException {
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
    public MetricTransformation3DEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to be notified of events such as when estimation starts or
     * ends.
     *
     * @param listener listener to be notified of events.
     * @throws LockedException if estimator is locked.
     */
    public void setListener(final MetricTransformation3DEstimatorListener listener) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.listener = listener;
    }

    /**
     * Indicates whether estimation can start with only 3 points or not.
     *
     * @return true allows 3 points, false requires 4.
     */
    public boolean isWeakMinimumSizeAllowed() {
        return weakMinimumSizeAllowed;
    }

    /**
     * Specifies whether estimation can start with only 3 points or not.
     *
     * @param weakMinimumSizeAllowed true allows 3 points, false requires 4.
     * @throws LockedException if estimator is locked.
     */
    public void setWeakMinimumSizeAllowed(final boolean weakMinimumSizeAllowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        this.weakMinimumSizeAllowed = weakMinimumSizeAllowed;
    }

    /**
     * Required minimum number of point correspondences to start the estimation.
     * Can be either 3 or 4.
     *
     * @return minimum number of point correspondences.
     */
    public int getMinimumPoints() {
        return weakMinimumSizeAllowed ? WEAK_MINIMUM_SIZE : MINIMUM_SIZE;
    }

    /**
     * Indicates whether listener has been provided and is available for
     * retrieval.
     *
     * @return true if available, false otherwise.
     */
    public boolean isListenerAvailable() {
        return listener != null;
    }

    /**
     * Indicates if this instance is locked because estimation is being
     * computed.
     *
     * @return true if locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Indicates if estimator is ready to start the metric 3D transformation
     * estimation.
     * This is true when input data (i.e. lists of matched points) are provided
     * and a minimum of MINIMUM_SIZE points are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    public boolean isReady() {
        return inputPoints != null && outputPoints != null && inputPoints.size() == outputPoints.size()
                && inputPoints.size() >= getMinimumPoints();
    }

    /**
     * Estimates a metric 3D transformation using the list of matched input
     * and output 3D points.
     * A minimum of 4 matched non-coincident points is required. If more points
     * are provided an LMSE (Least Mean Squared Error) solution will be found.
     *
     * @return estimated metric 3D transformation.
     * @throws LockedException           if estimator is locked.
     * @throws NotReadyException         if not enough data has been provided.
     * @throws CoincidentPointsException raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    public MetricTransformation3D estimate() throws LockedException, NotReadyException, CoincidentPointsException {
        final var result = new MetricTransformation3D();
        estimate(result);
        return result;
    }

    /**
     * Estimates a metric 3D transformation using the list of matched input
     * and output 3D points.
     * A minimum of 4 matched non-coincident points is required. If more points
     * are provided an LMSE (Least Mean Squared Error) solution will be found.
     *
     * @param result instance where result will be stored.
     * @throws LockedException           if estimator is locked.
     * @throws NotReadyException         if not enough data has been provided.
     * @throws CoincidentPointsException raised if transformation cannot be
     *                                   estimated for some reason (point configuration degeneracy, duplicate
     *                                   points or numerical instabilities).
     */
    public void estimate(final MetricTransformation3D result) throws LockedException, NotReadyException,
            CoincidentPointsException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            locked = true;

            if (listener != null) {
                listener.onEstimateStart(this);
            }

            final var inCentroid = computeCentroid(inputPoints);
            final var outCentroid = computeCentroid(outputPoints);

            final var m = new Matrix(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);

            final var n = inputPoints.size();
            final var col = new Matrix(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 1);
            final var row = new Matrix(1, Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            final var tmp = new Matrix(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            var inCov = 0.0;
            for (int i = 0; i < n; i++) {
                final var inputPoint = inputPoints.get(i);
                final var outputPoint = outputPoints.get(i);

                col.setElementAtIndex(0, inputPoint.getInhomX() - inCentroid.getElementAtIndex(0));
                col.setElementAtIndex(1, inputPoint.getInhomY() - inCentroid.getElementAtIndex(1));
                col.setElementAtIndex(2, inputPoint.getInhomZ() - inCentroid.getElementAtIndex(2));

                row.setElementAtIndex(0, outputPoint.getInhomX() - outCentroid.getElementAtIndex(0));
                row.setElementAtIndex(1, outputPoint.getInhomY() - outCentroid.getElementAtIndex(1));
                row.setElementAtIndex(2, outputPoint.getInhomZ() - outCentroid.getElementAtIndex(2));

                // compute covariances of input and output points
                inCov += Math.pow(Utils.normF(col), 2.0);

                col.multiply(row, tmp);
                m.add(tmp);
            }

            if (inCov == 0.0) {
                throw new CoincidentPointsException();
            }

            final var decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            if (!weakMinimumSizeAllowed && decomposer.getNullity() > 0) {
                throw new CoincidentPointsException();
            }

            final var u = decomposer.getU();
            final var v = decomposer.getV();

            final var s = decomposer.getSingularValues();


            // rotation R = V*U^T
            final var r = v.multiplyAndReturnNew(u.transposeAndReturnNew());

            final var e = new double[]{1.0, 1.0, 1.0};

            if (Utils.det(r) < 0.0) {
                // ideally rotation has 3 unitary singular values.
                // Because of reflection, we must change sign of last singular
                // value and reconstruct rotation matrix

                e[2] = -1.0;

                // because rotation matrix can be seen as V*e*U^T, we can
                // simply multiply 3rd column of R by -1
                r.setElementAt(0, 2, -r.getElementAt(0, 2));
                r.setElementAt(1, 2, -r.getElementAt(1, 2));
                r.setElementAt(2, 2, -r.getElementAt(2, 2));
            }

            final var rotation = new MatrixRotation3D(r);

            // scale
            final var dot = ArrayUtils.dotProduct(s, e);
            final var invScale = dot / inCov;

            // translation
            final var t = r.multiplyAndReturnNew(inCentroid);
            t.multiplyByScalar(-invScale);
            t.add(outCentroid);

            result.setRotation(rotation);
            result.setTranslation(t.getBuffer());
            result.setScale(invScale);

            if (listener != null) {
                listener.onEstimateEnd(this);
            }

        } catch (final AlgebraException | InvalidRotationMatrixException e) {
            throw new CoincidentPointsException(e);
        } finally {
            locked = false;
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
    private static Matrix computeCentroid(final List<Point3D> points) throws AlgebraException {
        var x = 0.0;
        var y = 0.0;
        var z = 0.0;
        final var n = points.size();
        for (final var p : points) {
            x += p.getInhomX() / n;
            y += p.getInhomY() / n;
            z += p.getInhomZ() / n;
        }

        final var result = new Matrix(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH, 1);
        result.setElementAtIndex(0, x);
        result.setElementAtIndex(1, y);
        result.setElementAtIndex(2, z);
        return result;
    }

    /**
     * Internal method to set lists of points to be used to estimate a
     * metric 3D transformation.
     * This method does not check whether estimator is locked or not.
     *
     * @param inputPoints  list of input points to be used to estimate a
     *                     metric 3D transformation.
     * @param outputPoints list of output points to be used to estimate a
     *                     metric 3D transformation.
     * @throws IllegalArgumentException if provided lists of points don't have
     *                                  the same size or their size is smaller than #getMinimumPoints.
     */
    private void internalSetPoints(final List<Point3D> inputPoints, final List<Point3D> outputPoints) {
        if (inputPoints.size() < getMinimumPoints()) {
            throw new IllegalArgumentException();
        }
        if (inputPoints.size() != outputPoints.size()) {
            throw new IllegalArgumentException();
        }
        this.inputPoints = inputPoints;
        this.outputPoints = outputPoints;
    }
}
