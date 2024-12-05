/*
 * Copyright (C) 2015 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.ProjectiveTransformation2D;

import java.util.List;

/**
 * This class takes a collection of points and computes its average
 * inhomogeneous coordinates and their scale so that a metric transformation is
 * computed to transform points and normalize them.
 * Normalized points are useful in many algorithms because due to the nature
 * of floating point quantization, accuracy of the computations increase for
 * normalized values between -1.0 and 1.0.
 * This implementation uses point inhomogeneous coordinates, hence, it should
 * not be used for points located at or close to infinity (i.e. very large
 * inhomogeneous values).
 */
public class Point2DNormalizer {

    /**
     * Minimum amount of points required to perform normalization.
     */
    public static final int MIN_POINTS = 2;

    /**
     * Collection of points used to compute normalization.
     */
    private List<Point2D> points;

    /**
     * Flag indicating that this instance is locked because computation is
     * in progress.
     */
    private boolean locked;

    /**
     * Minimum x inhomogeneous coordinate found in provided points.
     */
    private double minInhomX;

    /**
     * Minimum y inhomogeneous coordinate found in provided points.
     */
    private double minInhomY;

    /**
     * Maximum x inhomogeneous coordinate found in provided points.
     */
    private double maxInhomX;

    /**
     * Maximum y inhomogeneous coordinate found in provided points.
     */
    private double maxInhomY;

    /**
     * Computed scale on x coordinates to normalize points.
     */
    private double scaleX;

    /**
     * Computed scale on y coordinates to normalize points.
     */
    private double scaleY;

    /**
     * Computed x coordinate of centroid of points.
     */
    private double centroidX;

    /**
     * Computed y coordinate of centroid of points.
     */
    private double centroidY;

    /**
     * Transformation to normalize points.
     */
    private ProjectiveTransformation2D transformation;

    /**
     * Transformation to denormalize points, which corresponds to the inverse
     * transformation.
     */
    private ProjectiveTransformation2D inverseTransformation;

    /**
     * Constructor.
     *
     * @param points collection of points to be used to compute normalization.
     * @throws IllegalArgumentException if provided collection of points does
     *                                  not contain enough points, which is MIN_POINTS.
     */
    public Point2DNormalizer(final List<Point2D> points) {
        internalSetPoints(points);
        reset();
    }

    /**
     * Returns collection of points used to compute normalization.
     *
     * @return collection of points used to compute normalization.
     */
    public List<Point2D> getPoints() {
        return points;
    }

    /**
     * Sets collection of points used to compute normalization.
     *
     * @param points collection of points used to compute normalization.
     * @throws LockedException          if instance is locked because another computation
     *                                  is already in progress.
     * @throws IllegalArgumentException if provided collection of points does
     *                                  not contain enough points, which is MIN_POINTS.
     */
    public void setPoints(final List<Point2D> points) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        internalSetPoints(points);
        reset();
    }

    /**
     * Indicates whether this instance is ready (i.e. has enough data) to
     * start the computation.
     *
     * @return true if this instance is ready, false otherwise.
     */
    public boolean isReady() {
        return points != null && points.size() >= MIN_POINTS;
    }

    /**
     * Indicates whether this instance is locked because computation is
     * in progress.
     * While an instance is in progress, no parameter can be modified and
     * no further computations can be done until instance becomes unlocked.
     *
     * @return true if instance is locked, false otherwise.
     */
    public boolean isLocked() {
        return locked;
    }

    /**
     * Returns minimum x inhomogeneous coordinate found in provided points.
     *
     * @return minimum x inhomogeneous coordinate found in provided points.
     */
    public double getMinInhomX() {
        return minInhomX;
    }

    /**
     * Returns minimum y inhomogeneous coordinate found in provided points.
     *
     * @return minimum y inhomogeneous coordinate found in provided points.
     */
    public double getMinInhomY() {
        return minInhomY;
    }

    /**
     * Returns maximum x inhomogeneous coordinate found in provided points.
     *
     * @return maximum x inhomogeneous coordinate found in provided points.
     */
    public double getMaxInhomX() {
        return maxInhomX;
    }

    /**
     * Returns maximum y inhomogeneous coordinate found in provided points.
     *
     * @return maximum y inhomogeneous coordinate found in provided points.
     */
    public double getMaxInhomY() {
        return maxInhomY;
    }

    /**
     * Returns computed scale to normalize points on x coordinate.
     *
     * @return computed scale to normalize points on x coordinate.
     */
    public double getScaleX() {
        return scaleX;
    }

    /**
     * Returns computed scale to normalize points on y coordinate.
     *
     * @return computed scale to normalize points on y coordinate.
     */
    public double getScaleY() {
        return scaleY;
    }

    /**
     * Returns computed x coordinate of centroid of points.
     *
     * @return computed x coordinate of centroid of points.
     */
    public double getCentroidX() {
        return centroidX;
    }

    /**
     * Returns computed y coordinate of centroid of points.
     *
     * @return computed y coordinate of centroid of points.
     */
    public double getCentroidY() {
        return centroidY;
    }

    /**
     * Returns transformation to normalize points.
     *
     * @return transformation to normalize points.
     */
    public ProjectiveTransformation2D getTransformation() {
        return transformation;
    }

    /**
     * Returns transformation to denormalize points, which corresponds to the
     * inverse transformation.
     *
     * @return transformation to denormalize points.
     */
    public ProjectiveTransformation2D getInverseTransformation() {
        return inverseTransformation;
    }

    /**
     * Indicates whether result (i.e. transformation and inverse transformation)
     * are available or not.
     *
     * @return true if result is available, false otherwise.
     */
    public boolean isResultAvailable() {
        return transformation != null;
    }

    /**
     * Computes normalization and de-normalization transformations.
     *
     * @throws NotReadyException   if not enough data has been provided to
     *                             compute normalization.
     * @throws LockedException     if instance is locked because another computation
     *                             is already in progress.
     * @throws NormalizerException if normalization failed due to numerical
     *                             degeneracy. This usually happens when all provided points are located too
     *                             close to each other, which results in a singularity when computing proper
     *                             normalization scale.
     */
    public void compute() throws NotReadyException, LockedException, NormalizerException {
        if (!isReady()) {
            throw new NotReadyException();
        }
        if (isLocked()) {
            throw new LockedException();
        }
        try {
            locked = true;

            reset();
            computeLimits();

            // compute scale and centroids
            final var width = maxInhomX - minInhomX;
            final var height = maxInhomY - minInhomY;

            if (width < Double.MIN_VALUE || height < Double.MIN_VALUE) {
                // numerical degeneracy
                throw new NormalizerException();
            }

            scaleX = 1.0 / width;
            scaleY = 1.0 / height;

            // centroids of points
            centroidX = (minInhomX + maxInhomX) / 2.0;
            centroidY = (minInhomY + maxInhomY) / 2.0;

            // transformation to normalize points
            final var t = new Matrix(ProjectiveTransformation2D.HOM_COORDS, ProjectiveTransformation2D.HOM_COORDS);

            // X' = s * X + s * t -->
            // s * X = X' - s * t -->
            // X = 1/s*X' - t
            t.setElementAt(0, 0, scaleX);
            t.setElementAt(1, 1, scaleY);
            t.setElementAt(0, 2, -scaleX * centroidX);
            t.setElementAt(1, 2, -scaleY * centroidY);
            t.setElementAt(2, 2, 1.0);

            transformation = new ProjectiveTransformation2D(t);
            transformation.normalize();

            // transformation to denormalize points
            final var invT = new Matrix(ProjectiveTransformation2D.HOM_COORDS, ProjectiveTransformation2D.HOM_COORDS);

            invT.setElementAt(0, 0, width);
            invT.setElementAt(1, 1, height);
            invT.setElementAt(0, 2, centroidX);
            invT.setElementAt(1, 2, centroidY);
            invT.setElementAt(2, 2, 1.0);

            inverseTransformation = new ProjectiveTransformation2D(invT);
            inverseTransformation.normalize();
        } catch (final Exception e) {
            throw new NormalizerException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Computes minimum and maximum inhomogeneous point coordinates from the
     * list of provided 2D points.
     */
    @SuppressWarnings("DuplicatedCode")
    private void computeLimits() {
        for (final var point : points) {
            final var inhomX = point.getInhomX();
            final var inhomY = point.getInhomY();
            if (inhomX < minInhomX) {
                minInhomX = inhomX;
            }
            if (inhomY < minInhomY) {
                minInhomY = inhomY;
            }

            if (inhomX > maxInhomX) {
                maxInhomX = inhomX;
            }
            if (inhomY > maxInhomY) {
                maxInhomY = inhomY;
            }
        }
    }

    /**
     * Sets list of points.
     *
     * @param points list of points to be set.
     * @throws IllegalArgumentException if not enough points are provided, which
     *                                  is MIN_POINTS.
     */
    private void internalSetPoints(final List<Point2D> points) {
        if (points.size() < MIN_POINTS) {
            throw new IllegalArgumentException();
        }
        this.points = points;
    }

    /**
     * Resets internal values.
     */
    private void reset() {
        // reset result
        transformation = inverseTransformation = null;
        // reset limits
        minInhomX = minInhomY = Double.MAX_VALUE;
        maxInhomX = maxInhomY = -Double.MAX_VALUE;
        scaleX = scaleY = 1.0;
        centroidX = centroidY = 0.0;
    }
}
