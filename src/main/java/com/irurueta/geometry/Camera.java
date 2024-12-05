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

import java.util.ArrayList;
import java.util.List;

/**
 * A camera defines relations between 3D and 2D worlds.
 */
public abstract class Camera {

    /**
     * Constant defining default camera type.
     */
    public static final CameraType DEFAULT_CAMERA_TYPE = CameraType.PINHOLE_CAMERA;

    /**
     * Projects a 3D point into a 2D point in a retinal plane.
     *
     * @param point 3D point to be projected.
     * @return 2D projected point.
     */
    public Point2D project(final Point3D point) {
        final var projected = Point2D.create(CoordinatesType.HOMOGENEOUS_COORDINATES);
        project(point, projected);
        return projected;
    }

    /**
     * Projects a 3D point into a 2D point in a retinal plane and stores the
     * result into provided instance.
     *
     * @param inputPoint 3D point to be projected.
     * @param result     Instance where 2D projected point result is stored.
     */
    public abstract void project(final Point3D inputPoint, final Point2D result);

    /**
     * Projects 3D points into 2D points in a retinal plane.
     *
     * @param points 3D points to be projected.
     * @return 2D projected points.
     */
    public List<Point2D> project(final List<Point3D> points) {
        final var projected = new ArrayList<Point2D>(points.size());
        project(points, projected);
        return projected;
    }

    /**
     * Projects 3D points into 2D points in a retinal plane and stores the
     * result into provided list.
     * Note that if result list is not empty, it will be cleared when calling
     * this method and then the estimated 2D points will be stored in it.
     *
     * @param inputPoints 3D points to be projected.
     * @param result      Instance where 2D projected points are stored.
     */
    public void project(final List<Point3D> inputPoints, final List<Point2D> result) {
        result.clear();
        for (final var point : inputPoints) {
            result.add(project(point));
        }
    }

    /**
     * Back-projects a 2D line into a 3D plane.
     *
     * @param line 2D line to be back-projected.
     * @return 3D plane that has been back-projected.
     */
    public Plane backProject(final Line2D line) {
        final var plane = new Plane();
        backProject(line, plane);
        return plane;
    }

    // M^T * PLANE = 0
    // m^T * l = 0
    // m = P * M
    // (P * M)^T * l = 0 --> M^T * (P^T * l) = 0
    // PLANE = P^T * l

    /**
     * Back-projects a line into a plane and stores the result into provided
     * instance.
     *
     * @param line   2D line to be back-projected.
     * @param result Instance where computed back-projected 3D plane data is
     *               stored.
     */
    public abstract void backProject(final Line2D line, final Plane result);

    /**
     * Back-projects provided 2D lines into their corresponding 3D planes.
     *
     * @param lines 2D lines to be back-projected.
     * @return 3D planes that have been back-projected.
     */
    public List<Plane> backProjectLines(final List<Line2D> lines) {
        final var planes = new ArrayList<Plane>(lines.size());
        backProjectLines(lines, planes);
        return planes;
    }

    /**
     * Back-projects provided 2D lines into their corresponding 3D planes and
     * stores the result into provided list.
     * Note that if result list is not empty, its contents will be cleared when
     * calling this method and then the estimated 3D planes will be stored in it.
     *
     * @param lines  2D lines to be back-projected.
     * @param result 3D planes that have been back-projected.
     */
    public void backProjectLines(final List<Line2D> lines, final List<Plane> result) {
        result.clear();
        for (final var line : lines) {
            result.add(backProject(line));
        }
    }

    // Note: multiple point3D points can be back-projected (indeed is a ray of
    // light). In other words, solution is not unique

    /**
     * Back-projects provided 2D point into a 3D point.
     * Notice that estimated solution is not unique, since back-projecting a 2D
     * point results in an infinite number of 3D points located in the same
     * ray of light.
     * This method only returns one possible solution. Any possible solution can
     * be computed as a linear combination between the camera center and the
     * estimated point.
     *
     * @param point 2D point to be back-projected.
     * @return A back-projected 3D point.
     * @throws CameraException thrown if 2D point cannot be back-projected
     *                         because camera is degenerate.
     */
    public Point3D backProject(final Point2D point) throws CameraException {
        final var result = Point3D.create();
        backProject(point, result);
        return result;
    }

    /**
     * Back-projects provided 2D point into a 3D point and stores the result into
     * provided instance.
     * Notice that estimated solution is not unique, since back-projecting a 2D
     * point results in an infinite number of 3D points located in the same
     * ray of light.
     * This method only computes one possible solution. Any other solution can
     * be computed as a linear combination between the camera center and the
     * estimated back-projected point.
     *
     * @param point  2D point to be back-projected.
     * @param result Instance where back-projected 3D point data will be stored
     * @throws CameraException thrown if 2D point cannot be back-projected
     *                         because camera is degenerate.
     */
    public abstract void backProject(final Point2D point, final Point3D result) throws CameraException;

    /**
     * Back-projects provided 2D points into their corresponding 3D points.
     *
     * @param points 2D points to be back-projected.
     * @return 3D points that have been back-projected.
     * @throws CameraException thrown if 2D point cannot be back-projected
     *                         because camera is degenerate.
     */
    public List<Point3D> backProjectPoints(final List<Point2D> points) throws CameraException {
        final var result = new ArrayList<Point3D>(points.size());
        backProjectPoints(points, result);
        return result;
    }

    /**
     * Back-projects provided 2D points into their corresponding 3D points and
     * stores the result into provided list.
     * Note that if result list is not empty, its contents will be cleared when
     * calling this method and then the estimated 3D points will be stored in it.
     *
     * @param points 2D points to be back-projected.
     * @param result 3D points that have been back-projected.
     * @throws CameraException thrown if 2D point cannot be back-projected
     *                         because camera is degenerate.
     */
    public void backProjectPoints(final List<Point2D> points, final List<Point3D> result) throws CameraException {

        result.clear();
        for (final var point : points) {
            result.add(backProject(point));
        }
    }

    /**
     * Back-projects a 2D conic into a 3D quadric.
     *
     * @param conic 2D conic to be back-projected.
     * @return A back-projected 3D quadric.
     */
    public Quadric backProject(final Conic conic) {
        final var quadric = new Quadric();
        backProject(conic, quadric);
        return quadric;
    }

    /**
     * Back-projects a 2D conic into a 3D quadric and stores the result into
     * provided instance.
     *
     * @param conic  2D conic to be back-projected.
     * @param result Instance where data of back-projected 3D quadric will be
     *               stored.
     */
    public abstract void backProject(final Conic conic, final Quadric result);

    /**
     * Projects a 3D dual quadric into a 2D dual conic.
     *
     * @param dualQuadric 3D dual quadric to be projected.
     * @return A 2D dual conic.
     */
    public DualConic project(final DualQuadric dualQuadric) {
        final var dualConic = new DualConic();
        project(dualQuadric, dualConic);
        return dualConic;
    }

    /**
     * Projects a 3D dual quadric into a 2D dual conic and stores the result
     * into provided instance.
     *
     * @param dualQuadric 3D dual quadric to be projected.
     * @param result      Instance where data of projected 2D dual conic will be
     *                    stored.
     */
    public abstract void project(final DualQuadric dualQuadric, final DualConic result);

    /**
     * Projects a 3D quadric into a 2D conic.
     * The internal implementation of this method needs to compute the dual
     * quadric of provided quadric, and also needs to compute the resulting
     * conic from an internal dual conic, for that reason a CameraException might
     * be raised if provided quadric is degenerate or if internal estimated dual
     * conic is degenerate and cannot be converted into a conic.
     *
     * @param quadric 3D quadric to be projected.
     * @return A projected 2D conic.
     * @throws CameraException thrown if there are geometric degeneracies.
     */
    public Conic project(final Quadric quadric) throws CameraException {
        final var conic = new Conic();
        project(quadric, conic);
        return conic;
    }

    /**
     * Projects a 3D quadric into a 2D conic and stores the result into provided
     * conic instance.
     * The internal implementation of this method needs to compute the dual
     * quadric of provided quadric, and also needs to compute the resulting
     * conic from an internal dual conic, for that reason a
     * CameraException might be raised if provided quadric is degenerate or
     * if internal estimated dual conic is degenerate and cannot be converted
     * into a conic.
     *
     * @param quadric 3D quadric to be projected.
     * @param result  Instance where data of projected 2D conic will be stored.
     * @throws CameraException thrown if there are geometric degeneracies.
     */
    public void project(final Quadric quadric, final Conic result) throws CameraException {
        quadric.normalize();
        try {
            final var dualQuadric = quadric.getDualQuadric();
            final var dualConic = new DualConic();
            project(dualQuadric, dualConic);
            dualConic.conic(result);
        } catch (final GeometryException e) {
            throw new CameraException(e);
        }
    }

    /**
     * Back-projects a 2D dual conic into a 3D dual quadric.
     * The internal implementation of this method needs to compute a conic from
     * provided dual conic, and also needs to compute the resulting dual quadric
     * from an internal quadric. For that reason a CameraException might be
     * raised if provided dual conic is degenerate or if internal estimated
     * quadric is degenerate and cannot be converted into a dual quadric.
     *
     * @param dualConic 2D dual conic to be back-projected.
     * @return A back-projected 3D dual quadric.
     * @throws CameraException thrown if there are geometric degeneracies.
     */
    public DualQuadric backProject(final DualConic dualConic) throws CameraException {
        final var dualQuadric = new DualQuadric();
        backProject(dualConic, dualQuadric);
        return dualQuadric;
    }

    /**
     * Back-projects a 2D dual conic into a 3D dual quadric and stores the result
     * into provided dual quadric instance.
     * The internal implementation of this method needs to compute a conic from
     * provided dual conic, and also needs to compute the resulting dual quadric
     * from an internal quadric. For that reason a CameraException might be
     * raised if provided dual conic is degenerate or if internal estimated
     * quadric is degenerate and cannot be converted into a dual quadric.
     *
     * @param dualConic 2D dual conic to be back-projected.
     * @param result    Instance where data of back-projected 3D dual quadric will
     *                  be stored.
     * @throws CameraException thrown if there are geometric degeneracies.
     */
    public void backProject(final DualConic dualConic, final DualQuadric result) throws CameraException {
        try {
            dualConic.normalize();
            final var conic = dualConic.getConic();
            final var quadric = new Quadric();
            backProject(conic, quadric);
            quadric.dualQuadric(result);
        } catch (final GeometryException e) {
            throw new CameraException(e);
        }
    }

    /**
     * Returns the type of this camera.
     *
     * @return Type of this camera.
     */
    public abstract CameraType getType();

    /**
     * Creates an instance of a camera using default type.
     *
     * @return A newly instantiated camera.
     */
    public static Camera create() {
        return new PinholeCamera();
    }
}
