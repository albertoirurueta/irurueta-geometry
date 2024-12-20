/*
 * Copyright (C) 2013 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.numerical.robust.WeightSelection;

import java.util.Iterator;
import java.util.List;

/**
 * This class implements pinhole camera estimator using a weighted algorithm and
 * point correspondences.
 */
@SuppressWarnings("DuplicatedCode")
public class WeightedPointCorrespondencePinholeCameraEstimator extends PointCorrespondencePinholeCameraEstimator {

    /**
     * Default number of points (i.e. correspondences) to be weighted and taken
     * into account.
     */
    public static final int DEFAULT_MAX_POINTS = 50;

    /**
     * Indicates if weights are sorted by default so that largest weighted
     * correspondences are used first.
     */
    public static final boolean DEFAULT_SORT_WEIGHTS = true;

    /**
     * Maximum number of points (i.e. correspondences) to be weighted and taken
     * into account.
     */
    private int maxPoints;

    /**
     * Indicates if weights are sorted by default so that largest weighted
     * correspondences are used first.
     */
    private boolean sortWeights;

    /**
     * Array containing weights for all point correspondences.
     */
    private double[] weights;

    /**
     * Constructor.
     */
    public WeightedPointCorrespondencePinholeCameraEstimator() {
        super();
        maxPoints = DEFAULT_MAX_POINTS;
        sortWeights = DEFAULT_SORT_WEIGHTS;
        weights = null;
    }

    /**
     * Constructor with listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or estimation progress changes.
     */
    public WeightedPointCorrespondencePinholeCameraEstimator(final PinholeCameraEstimatorListener listener) {
        super(listener);
        maxPoints = DEFAULT_MAX_POINTS;
        sortWeights = DEFAULT_SORT_WEIGHTS;
        weights = null;
    }

    /**
     * Constructor.
     *
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException  if provided lists of points don't have
     *                                  the same size and enough points.
     */
    public WeightedPointCorrespondencePinholeCameraEstimator(
            final List<Point3D> points3D, final List<Point2D> points2D) throws WrongListSizesException {
        super(points3D, points2D);
        maxPoints = DEFAULT_MAX_POINTS;
        sortWeights = DEFAULT_SORT_WEIGHTS;
        weights = null;
    }

    /**
     * Constructor.
     *
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException  if provided lists of points don't have
     *                                  the same size and enough points.
     */
    public WeightedPointCorrespondencePinholeCameraEstimator(
            final List<Point3D> points3D, final List<Point2D> points2D, final PinholeCameraEstimatorListener listener)
            throws WrongListSizesException {
        super(points3D, points2D, listener);
        maxPoints = DEFAULT_MAX_POINTS;
        sortWeights = DEFAULT_SORT_WEIGHTS;
        weights = null;
    }

    /**
     * Constructor.
     *
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @param weights  array containing a weight amount for each correspondence.
     *                 The larger the value of a weight, the most significant the
     *                 correspondence will be.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException  if provided lists of points don't have
     *                                  the same size and enough points.
     */
    public WeightedPointCorrespondencePinholeCameraEstimator(
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] weights)
            throws WrongListSizesException {
        super();
        maxPoints = DEFAULT_MAX_POINTS;
        sortWeights = DEFAULT_SORT_WEIGHTS;
        this.weights = null;
        internalSetListsAndWeights(points3D, points2D, weights);
    }

    /**
     * Constructor.
     *
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @param weights  array containing a weight amount for each correspondence.
     *                 The larger the value of a weight, the most significant the
     *                 correspondence will be.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException  if provided lists of points don't have
     *                                  the same size and enough points.
     */
    public WeightedPointCorrespondencePinholeCameraEstimator(
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] weights,
            final PinholeCameraEstimatorListener listener) throws WrongListSizesException {
        super(listener);
        maxPoints = DEFAULT_MAX_POINTS;
        sortWeights = DEFAULT_SORT_WEIGHTS;
        this.weights = null;
        internalSetListsAndWeights(points3D, points2D, weights);
    }

    /**
     * Internal method to set list of corresponding points (it does not check
     * if estimator is locked).
     *
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @param weights  array containing a weight amount for each correspondence.
     *                 The larger the value of a weight, the most significant the
     *                 correspondence will be.
     * @throws IllegalArgumentException if any of the lists or arrays are null.
     * @throws WrongListSizesException  if provided lists of points don't have
     *                                  the same size and enough points or if the length of the weights array
     *                                  is not equal to the number of point correspondences.
     */
    private void internalSetListsAndWeights(
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] weights)
            throws WrongListSizesException {

        if (points3D == null || points2D == null || weights == null) {
            throw new IllegalArgumentException();
        }

        if (!areValidListsAndWeights(points3D, points2D, weights)) {
            throw new WrongListSizesException();
        }

        this.points3D = points3D;
        this.points2D = points2D;
        this.weights = weights;
    }

    /**
     * Sets list of corresponding points.
     *
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @param weights  array containing a weight amount for each correspondence.
     *                 The larger the value of a weight, the most significant the
     *                 correspondence will be.
     * @throws LockedException          if estimator is locked.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException  if provided lists of points don't have
     *                                  the same size and enough points.
     */
    public void setListsAndWeights(
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] weights) throws LockedException,
            WrongListSizesException {
        if (isLocked()) {
            throw new LockedException();
        }

        internalSetListsAndWeights(points3D, points2D, weights);
    }

    /**
     * Indicates if lists of corresponding 2D/3D points are valid.
     * Lists are considered valid if they have the same number of points and
     * both have more than the required minimum of correspondences (which is 6).
     *
     * @param points3D list of corresponding 3D points.
     * @param points2D list of corresponding 2D points.
     * @param weights  array containing a weight amount for each correspondence.
     *                 The larger the value of a weight, the most significant the
     *                 correspondence will be.
     * @return true if corresponding 2D/3D points are valid, false otherwise.
     */
    public static boolean areValidListsAndWeights(
            final List<Point3D> points3D, final List<Point2D> points2D, final double[] weights) {
        if (points3D == null || points2D == null || weights == null) {
            return false;
        }
        return points3D.size() == points2D.size() && points2D.size() == weights.length
                && points3D.size() >= MIN_NUMBER_OF_POINT_CORRESPONDENCES;
    }

    /**
     * Returns array containing a weight amount for each correspondence.
     * The larger the value of a weight, the most significant the
     * correspondence will be.
     *
     * @return array containing weights for each correspondence.
     * @throws NotAvailableException if weights are not available.
     */
    public double[] getWeights() throws NotAvailableException {
        if (!areWeightsAvailable()) {
            throw new NotAvailableException();
        }
        return weights;
    }

    /**
     * Returns boolean indicating whether weights have been provided and are
     * available for retrieval.
     *
     * @return true if weights are available, false otherwise.
     */
    public boolean areWeightsAvailable() {
        return weights != null;
    }

    /**
     * Returns maximum number of points (i.e. correspondences) to be weighted
     * and taken into account.
     *
     * @return maximum number of points to be weighted.
     */
    public int getMaxPoints() {
        return maxPoints;
    }

    /**
     * Sets maximum number of points (i.e. correspondences) to be weighted and
     * taken into account.
     *
     * @param maxPoints maximum number of points to be weighted.
     * @throws IllegalArgumentException if provided value is less than the
     *                                  minimum allowed number of point correspondences.
     * @throws LockedException          if this instance is locked.
     */
    public void setMaxPoints(final int maxPoints) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        if (maxPoints < MIN_NUMBER_OF_POINT_CORRESPONDENCES) {
            throw new IllegalArgumentException();
        }

        this.maxPoints = maxPoints;
    }

    /**
     * Indicates if weights are sorted by so that largest weighted
     * correspondences are used first.
     *
     * @return true if weights are sorted, false otherwise.
     */
    public boolean isSortWeightsEnabled() {
        return sortWeights;
    }

    /**
     * Specifies whether weights are sorted by so that largest weighted
     * correspondences are used first.
     *
     * @param sortWeights true if weights are sorted, false otherwise.
     * @throws LockedException if this instance is locked.
     */
    public void setSortWeightsEnabled(final boolean sortWeights) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }

        this.sortWeights = sortWeights;
    }

    /**
     * Indicates if this estimator is ready to start the estimation.
     * Estimator will be ready once both lists and weights are available.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return areListsAvailable() && areWeightsAvailable();
    }

    /**
     * Internal method that actually computes the normalized pinhole camera
     * internal matrix.
     * Returned matrix must have norm equal to one and might be estimated using
     * any convenient algorithm (i.e. DLT or weighted DLT).
     *
     * @param points3D list of 3D points. Points might or might not be
     *                 normalized.
     * @param points2D list of 2D points. Points might or might not be
     *                 normalized.
     * @return matrix of estimated pinhole camera.
     * @throws PinholeCameraEstimatorException if estimation fails for some
     *                                         reason (i.e. numerical instability or geometric degeneracy).
     */
    @Override
    protected Matrix internalEstimate(final List<Point3D> points3D, final List<Point2D> points2D)
            throws PinholeCameraEstimatorException {

        try {
            final var selection = WeightSelection.selectWeights(weights, sortWeights, maxPoints);
            final var selected = selection.getSelected();

            final var a = new Matrix(12, 12);
            final var row = new Matrix(2, 12);
            final var transRow = new Matrix(12, 2);
            final var tmp = new Matrix(12, 12);

            final var iterator2D = points2D.iterator();
            final var iterator3D = points3D.iterator();

            var index = 0;
            var nMatches = 0;
            var previousNorm = 1.0;
            double rowNorm;
            while (iterator2D.hasNext() && iterator3D.hasNext()) {
                final var point2D = iterator2D.next();
                final var point3D = iterator3D.next();

                if (selected[index]) {
                    final var weight = weights[index];

                    if (Math.abs(weight) < EPS) {
                        // skip, because weight is too small
                        index++;
                        continue;
                    }

                    // normalize points to increase accuracy
                    point2D.normalize();
                    point3D.normalize();

                    final var homImageX = point2D.getHomX();
                    final var homImageY = point2D.getHomY();
                    final var homImageW = point2D.getHomW();

                    final var homWorldX = point3D.getHomX();
                    final var homWorldY = point3D.getHomY();
                    final var homWorldZ = point3D.getHomZ();
                    final var homWorldW = point3D.getHomW();

                    // first row
                    row.setElementAt(0, 0, homImageW * homWorldX * weight);
                    row.setElementAt(0, 1, homImageW * homWorldY * weight);
                    row.setElementAt(0, 2, homImageW * homWorldZ * weight);
                    row.setElementAt(0, 3, homImageW * homWorldW * weight);

                    // columns 4, 5, 6, 7 are left with zero values

                    row.setElementAt(0, 8, -homImageX * homWorldX * weight);
                    row.setElementAt(0, 9, -homImageX * homWorldY * weight);
                    row.setElementAt(0, 10, -homImageX * homWorldZ * weight);
                    row.setElementAt(0, 11, -homImageX * homWorldW * weight);

                    // normalize row
                    rowNorm = Math.sqrt(Math.pow(row.getElementAt(0, 0), 2.0)
                            + Math.pow(row.getElementAt(0, 1), 2.0)
                            + Math.pow(row.getElementAt(0, 2), 2.0)
                            + Math.pow(row.getElementAt(0, 3), 2.0)
                            + Math.pow(row.getElementAt(0, 8), 2.0)
                            + Math.pow(row.getElementAt(0, 9), 2.0)
                            + Math.pow(row.getElementAt(0, 10), 2.0)
                            + Math.pow(row.getElementAt(0, 11), 2.0));

                    row.setElementAt(0, 0, row.getElementAt(0, 0) / rowNorm);
                    row.setElementAt(0, 1, row.getElementAt(0, 1) / rowNorm);
                    row.setElementAt(0, 2, row.getElementAt(0, 2) / rowNorm);
                    row.setElementAt(0, 3, row.getElementAt(0, 3) / rowNorm);
                    row.setElementAt(0, 8, row.getElementAt(0, 8) / rowNorm);
                    row.setElementAt(0, 9, row.getElementAt(0, 9) / rowNorm);
                    row.setElementAt(0, 10, row.getElementAt(0, 10) / rowNorm);
                    row.setElementAt(0, 11, row.getElementAt(0, 11) / rowNorm);

                    // second row

                    // columns 0, 1, 2, 3 are left with zero values

                    row.setElementAt(1, 4, homImageW * homWorldX * weight);
                    row.setElementAt(1, 5, homImageW * homWorldY * weight);
                    row.setElementAt(1, 6, homImageW * homWorldZ * weight);
                    row.setElementAt(1, 7, homImageW * homWorldW * weight);

                    row.setElementAt(1, 8, -homImageY * homWorldX * weight);
                    row.setElementAt(1, 9, -homImageY * homWorldY * weight);
                    row.setElementAt(1, 10, -homImageY * homWorldZ * weight);
                    row.setElementAt(1, 11, -homImageY * homWorldW * weight);

                    // normalize row
                    rowNorm = Math.sqrt(Math.pow(row.getElementAt(1, 4), 2.0)
                            + Math.pow(row.getElementAt(1, 5), 2.0)
                            + Math.pow(row.getElementAt(1, 6), 2.0)
                            + Math.pow(row.getElementAt(1, 7), 2.0)
                            + Math.pow(row.getElementAt(1, 8), 2.0)
                            + Math.pow(row.getElementAt(1, 9), 2.0)
                            + Math.pow(row.getElementAt(1, 10), 2.0)
                            + Math.pow(row.getElementAt(1, 11), 2.0));

                    row.setElementAt(1, 4, row.getElementAt(1, 4) / rowNorm);
                    row.setElementAt(1, 5, row.getElementAt(1, 5) / rowNorm);
                    row.setElementAt(1, 6, row.getElementAt(1, 6) / rowNorm);
                    row.setElementAt(1, 7, row.getElementAt(1, 7) / rowNorm);
                    row.setElementAt(1, 8, row.getElementAt(1, 8) / rowNorm);
                    row.setElementAt(1, 9, row.getElementAt(1, 9) / rowNorm);
                    row.setElementAt(1, 10, row.getElementAt(1, 10) / rowNorm);
                    row.setElementAt(1, 11, row.getElementAt(1, 11) / rowNorm);

                    // transRow = row'
                    row.transpose(transRow);
                    // tmp = row' * row
                    transRow.multiply(row, tmp);

                    tmp.multiplyByScalar(1.0 / previousNorm);

                    // a += 1.0 / previousNorm * tmp
                    a.add(tmp);
                    // normalize
                    previousNorm = Utils.normF(a);
                    a.multiplyByScalar(1.0 / previousNorm);

                    nMatches++;
                }
                index++;
            }

            if (nMatches < MIN_NUMBER_OF_POINT_CORRESPONDENCES) {
                throw new PinholeCameraEstimatorException();
            }

            final var decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();

            if (decomposer.getNullity() > 1) {
                // point configuration is degenerate and exists a linear
                // combination of possible pinhole cameras (i.e. solution is not
                // unique up to scale)
                throw new PinholeCameraEstimatorException();
            }

            final var v = decomposer.getV();

            // use last column of V as pinhole camera vector

            // the last column of V contains pinhole camera matrix ordered by
            // rows as: P11, P12, P13, P14, P21, P22, P23, P24, P31, P32, P33,
            // P34, hence we reorder p
            final var pinholeCameraMatrix = new Matrix(PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS,
                    PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);

            pinholeCameraMatrix.setElementAt(0, 0, v.getElementAt(0, 11));
            pinholeCameraMatrix.setElementAt(0, 1, v.getElementAt(1, 11));
            pinholeCameraMatrix.setElementAt(0, 2, v.getElementAt(2, 11));
            pinholeCameraMatrix.setElementAt(0, 3, v.getElementAt(3, 11));

            pinholeCameraMatrix.setElementAt(1, 0, v.getElementAt(4, 11));
            pinholeCameraMatrix.setElementAt(1, 1, v.getElementAt(5, 11));
            pinholeCameraMatrix.setElementAt(1, 2, v.getElementAt(6, 11));
            pinholeCameraMatrix.setElementAt(1, 3, v.getElementAt(7, 11));

            pinholeCameraMatrix.setElementAt(2, 0, v.getElementAt(8, 11));
            pinholeCameraMatrix.setElementAt(2, 1, v.getElementAt(9, 11));
            pinholeCameraMatrix.setElementAt(2, 2, v.getElementAt(10, 11));
            pinholeCameraMatrix.setElementAt(2, 3, v.getElementAt(11, 11));

            // because pinholeCameraMatrix has been obtained as the last column
            // of V, then its Frobenius norm will be 1 because SVD already
            // returns normalized singular vector

            return pinholeCameraMatrix;

        } catch (final PinholeCameraEstimatorException e) {
            throw e;
        } catch (final Exception e) {
            throw new PinholeCameraEstimatorException(e);
        }
    }

    /**
     * Returns type of pinhole camera estimator.
     *
     * @return type of pinhole camera estimator.
     */
    @Override
    public PinholeCameraEstimatorType getType() {
        return PinholeCameraEstimatorType.WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR;
    }
}
