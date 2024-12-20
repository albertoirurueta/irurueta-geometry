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
import com.irurueta.algebra.Matrix;

import java.util.ArrayList;
import java.util.List;

/**
 * This class performs transformations on 3D space.
 * Transformations can be applied to any 3D geometric figure.
 */
public abstract class Transformation3D {

    /**
     * Empty constructor.
     */
    protected Transformation3D() {
    }

    /**
     * Transforms provided point using this transformation and returns a new
     * one.
     *
     * @param inputPoint point to be transformed.
     * @return a new transformed point.
     */
    public Point3D transformAndReturnNew(final Point3D inputPoint) {
        final var outputPoint = Point3D.create();
        transform(inputPoint, outputPoint);
        return outputPoint;
    }

    /**
     * Transforms and updates provided point.
     *
     * @param point point to be transformed and updated.
     */
    public void transform(final Point3D point) {
        transform(point, point);
    }

    /**
     * Transforms input point using this transformation and stores the result in
     * provided output points.
     *
     * @param inputPoint  point to be transformed.
     * @param outputPoint instance where transformed point data will be stored.
     */
    public abstract void transform(final Point3D inputPoint, final Point3D outputPoint);

    /**
     * Transforms provided list of points using this transformation.
     *
     * @param inputPoints points to be transformed.
     * @return new transformed points.
     */
    public List<Point3D> transformPointsAndReturnNew(final List<Point3D> inputPoints) {
        final var outputPoints = new ArrayList<Point3D>(inputPoints.size());
        transformPoints(inputPoints, outputPoints);
        return outputPoints;
    }

    /**
     * Transforms provided list of points using this transformation and stores
     * the result in provided output list of points.
     * Notice that any previous content in output list will be removed when
     * calling this method.
     *
     * @param inputPoints  points to be transformed.
     * @param outputPoints transformed points.
     */
    public void transformPoints(final List<Point3D> inputPoints, final List<Point3D> outputPoints) {
        outputPoints.clear();
        for (final var point : inputPoints) {
            outputPoints.add(transformAndReturnNew(point));
        }
    }

    /**
     * Transforms provided list of points using this transformation and
     * overwriting their previous values.
     *
     * @param points points to be transformed and overwritten.
     */
    public void transformAndOverwritePoints(final List<Point3D> points) {
        for (final var point : points) {
            transform(point, point);
        }
    }

    /**
     * Transforms a quadric using this transformation and returns a new one.
     *
     * @param inputQuadric quadric to be transformed.
     * @return a new transformed quadric.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output quadric matrix is not considered to be symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because of
     *                                     numerical instabilities.
     */
    public Quadric transformAndReturnNew(final Quadric inputQuadric) throws NonSymmetricMatrixException,
            AlgebraException {
        final var outputQuadric = new Quadric();
        transform(inputQuadric, outputQuadric);
        return outputQuadric;
    }

    /**
     * Transforms and updates provided quadric.
     *
     * @param quadric quadric to be transformed.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting quadric matrix is not considered to be symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because of
     *                                     numerical instabilities.
     */
    public void transform(final Quadric quadric) throws NonSymmetricMatrixException, AlgebraException {
        transform(quadric, quadric);
    }

    /**
     * Transforms a quadric using this transformation and stores the result into
     * provided output quadric.
     *
     * @param inputQuadric  quadric to be transformed.
     * @param outputQuadric instance where data of transformed quadric will be
     *                      stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output quadric matrix is not considered to be symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because of
     *                                     numerical instabilities.
     */
    public abstract void transform(final Quadric inputQuadric, final Quadric outputQuadric)
            throws NonSymmetricMatrixException, AlgebraException;

    /**
     * Transforms a dual quadric using this transformation and returns a new
     * one.
     *
     * @param inputDualQuadric dual quadric to be transformed.
     * @return a new transformed dual quadric.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output dual quadric matrix is not considered to be
     *                                     symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because
     *                                     of numerical instabilities.
     */
    public DualQuadric transformAndReturnNew(final DualQuadric inputDualQuadric)
            throws NonSymmetricMatrixException, AlgebraException {
        final var outputDualQuadric = new DualQuadric();
        transform(inputDualQuadric, outputDualQuadric);
        return outputDualQuadric;
    }

    /**
     * Transforms and updates a dual quadric using this transformation.
     *
     * @param dualQuadric dual quadric to be transformed.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output dual quadric matrix is not considered to be
     *                                     symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because
     *                                     of numerical instabilities.
     */
    public void transform(final DualQuadric dualQuadric) throws NonSymmetricMatrixException, AlgebraException {
        transform(dualQuadric, dualQuadric);
    }

    /**
     * Transforms a dual quadric using this transformation and stores the result
     * into provided output dual quadric.
     *
     * @param inputDualQuadric  dual quadric to be transformed.
     * @param outputDualQuadric instance where data of transformed dual quadric
     *                          will be stored.
     * @throws NonSymmetricMatrixException Raised if due to numerical precision
     *                                     the resulting output dual quadric matrix is not considered to be
     *                                     symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because
     *                                     of numerical instabilities.
     */
    public abstract void transform(final DualQuadric inputDualQuadric, final DualQuadric outputDualQuadric)
            throws NonSymmetricMatrixException, AlgebraException;

    /**
     * Transforms provided plane using this transformation and returns a new
     * one.
     *
     * @param inputPlane plane to be transformed.
     * @return a new transformed plane.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public Plane transformAndReturnNew(final Plane inputPlane) throws AlgebraException {
        final var outputPlane = new Plane();
        transform(inputPlane, outputPlane);
        return outputPlane;
    }

    /**
     * Transforms and updates provided plane using this transformation.
     *
     * @param plane plane to be transformed.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public void transform(final Plane plane) throws AlgebraException {
        transform(plane, plane);
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
    public abstract void transform(final Plane inputPlane, final Plane outputPlane) throws AlgebraException;

    /**
     * Transforms provided list of planes using this transformation.
     *
     * @param inputPlanes planes to be transformed.
     * @return transformed planes.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public List<Plane> transformPlanesAndReturnNew(final List<Plane> inputPlanes) throws AlgebraException {
        final var outputPlanes = new ArrayList<Plane>(inputPlanes.size());
        transformPlanes(inputPlanes, outputPlanes);
        return outputPlanes;
    }

    /**
     * Transforms provided list of planes using this transformation and stores
     * the result in provided output list of planes.
     * Notice that any previous content in output list will be removed when
     * calling this method.
     *
     * @param inputPlanes  planes to be transformed.
     * @param outputPlanes transformed planes.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public void transformPlanes(final List<Plane> inputPlanes, final List<Plane> outputPlanes) throws AlgebraException {
        outputPlanes.clear();
        for (final var plane : inputPlanes) {
            outputPlanes.add(transformAndReturnNew(plane));
        }
    }

    /**
     * Transforms provided list of planes using this transformation and
     * overwriting their previous values.
     *
     * @param planes planes to be transformed and overwritten.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public void transformAndOverwritePlanes(final List<Plane> planes) throws AlgebraException {
        for (final var plane : planes) {
            transform(plane, plane);
        }
    }

    /**
     * Transforms provided line using this transformation and returns a new one.
     *
     * @param inputLine line to be transformed.
     * @return transformed line.
     * @throws CoincidentPlanesException raised if transformation is degenerate
     *                                   and results in planes forming a line being coincident.
     * @throws AlgebraException          raised if transform cannot be computed because
     *                                   of numerical instabilities.
     */
    public Line3D transformAndReturnNew(final Line3D inputLine) throws CoincidentPlanesException, AlgebraException {
        final var plane1 = transformAndReturnNew(inputLine.getPlane1());
        final var plane2 = transformAndReturnNew(inputLine.getPlane2());

        return new Line3D(plane1, plane2);
    }

    /**
     * Transforms and updates provided line using this transformation.
     *
     * @param line line to be transformed.
     * @throws CoincidentPlanesException raised if transformation is degenerate
     *                                   and results in planes forming a line being coincident.
     * @throws AlgebraException          raised if transform cannot be computed because
     *                                   of numerical instabilities.
     */
    public void transform(final Line3D line) throws CoincidentPlanesException, AlgebraException {
        transform(line, line);
    }

    /**
     * Transforms provided input line using this transformation and stores the
     * result into provided output line instance.
     *
     * @param inputLine  line to be transformed.
     * @param outputLine instance where data of transformed line will be stored.
     * @throws CoincidentPlanesException Raised if transformation is degenerate
     *                                   and results in planes forming a line being coincident.
     * @throws AlgebraException          raised if transform cannot be computed because
     *                                   of numerical instabilities.
     */
    public void transform(final Line3D inputLine, final Line3D outputLine) throws CoincidentPlanesException,
            AlgebraException {

        final var plane1 = transformAndReturnNew(inputLine.getPlane1());
        final var plane2 = transformAndReturnNew(inputLine.getPlane2());

        outputLine.setPlanes(plane1, plane2);
    }

    /**
     * Transforms provided list of lines using this transformation.
     *
     * @param inputLines lines to be transformed.
     * @return transformed lines.
     * @throws CoincidentPlanesException raised if transformation is degenerate
     *                                   and results in planes forming a line being coincident.
     * @throws AlgebraException          raised if transform cannot be computed because
     *                                   of numerical instabilities.
     */
    public List<Line3D> transformLines(final List<Line3D> inputLines) throws CoincidentPlanesException,
            AlgebraException {
        final var outputLines = new ArrayList<Line3D>(inputLines.size());
        transformLines(inputLines, outputLines);
        return outputLines;
    }

    /**
     * Transforms provided list of lines using this transformation and stores
     * the result in provided output list of lines.
     * Notice that any previous content in output list will be removed when
     * calling this method.
     *
     * @param inputLines  lines to be transformed.
     * @param outputLines transformed lines.
     * @throws CoincidentPlanesException raised if transformation is degenerate
     *                                   and results in planes forming a line being coincident.
     * @throws AlgebraException          raised if transform cannot be computed because
     *                                   of numerical instabilities.
     */
    public void transformLines(
            final List<Line3D> inputLines, final List<Line3D> outputLines) throws CoincidentPlanesException,
            AlgebraException {

        outputLines.clear();
        for (final var line : inputLines) {
            outputLines.add(transformAndReturnNew(line));
        }
    }

    /**
     * Transforms provided list of lines using this transformation and
     * overwriting their previous values.
     *
     * @param lines lines to be transformed and overwritten.
     * @throws CoincidentPlanesException raised if transformation is degenerate
     *                                   and results in planes forming a line being coincident.
     * @throws AlgebraException          raised if transform cannot be computed because
     *                                   of numerical instabilities.
     */
    public void transformAndOverwriteLines(final List<Line3D> lines) throws CoincidentPlanesException,
            AlgebraException {

        for (final var line : lines) {
            transform(line, line);
        }
    }

    /**
     * Transforms provided polygon using this transformation and returns a new
     * one.
     *
     * @param inputPolygon polygon to be transformed.
     * @return a new transformed polygon.
     */
    public Polygon3D transformAndReturnNew(final Polygon3D inputPolygon) {
        final var outVertices = transformPointsAndReturnNew(inputPolygon.getVertices());
        try {
            return new Polygon3D(outVertices);
        } catch (final NotEnoughVerticesException ignore) {
            // this will never happen because all existing polygons have enough
            // vertices
            return null;
        }
    }

    /**
     * Transforms and updates provided polygon using this transformation.
     *
     * @param polygon polygon to be transformed.
     */
    public void transform(final Polygon3D polygon) {
        transformAndOverwritePoints(polygon.getVertices());
    }

    /**
     * Transforms provided input polygon using this transformation and stores
     * the result into provided output polygon instance.
     *
     * @param inputPolygon  polygon to be transformed.
     * @param outputPolygon Instance where transformed polygon data will be
     *                      stored.
     */
    public void transform(final Polygon3D inputPolygon, final Polygon3D outputPolygon) {
        try {
            outputPolygon.setVertices(transformPointsAndReturnNew(inputPolygon.getVertices()));
        } catch (final NotEnoughVerticesException ignore) {
            // this will never happen because all existing polygons have enough
            // vertices
        }
    }

    /**
     * Transforms provided triangle using this transformation and returns a new
     * one.
     *
     * @param inputTriangle triangle to be transformed.
     * @return a new transformed triangle.
     */
    public Triangle3D transformAndReturnNew(final Triangle3D inputTriangle) {
        final var vertex1 = transformAndReturnNew(inputTriangle.getVertex1());
        final var vertex2 = transformAndReturnNew(inputTriangle.getVertex2());
        final var vertex3 = transformAndReturnNew(inputTriangle.getVertex3());
        return new Triangle3D(vertex1, vertex2, vertex3);
    }

    /**
     * Transforms and updates provided input triangle using this transformation.
     *
     * @param triangle triangle to be transformed.
     */
    public void transform(final Triangle3D triangle) {
        transform(triangle, triangle);
    }

    /**
     * Transforms provided input triangle using this transformation and stores
     * the result into provided output triangle instance.
     *
     * @param inputTriangle  triangle to be transformed.
     * @param outputTriangle instance where transformed triangle data will be
     *                       stored.
     */
    public void transform(final Triangle3D inputTriangle, final Triangle3D outputTriangle) {
        transform(inputTriangle.getVertex1(), outputTriangle.getVertex1());
        transform(inputTriangle.getVertex2(), outputTriangle.getVertex2());
        transform(inputTriangle.getVertex3(), outputTriangle.getVertex3());
    }

    /**
     * Represents this transformation as a 4x4 matrix.
     * A point can be transformed as T * p, where T is the transformation matrix
     * and p is a point expressed as an homogeneous vector.
     *
     * @return this transformation in matrix form.
     */
    public abstract Matrix asMatrix();

    /**
     * Represents this transformation as a 4x4 matrix and stores the result in
     * provided instance.
     *
     * @param m instance where transformation matrix will be stored.
     * @throws IllegalArgumentException raised if provided instance is not a 3x3
     *                                  matrix.
     */
    public abstract void asMatrix(final Matrix m);

    /**
     * Transforms a camera using this transformation.
     *
     * @param camera camera to be transformed.
     * @return transformed quadric.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public PinholeCamera transformAndReturnNew(final PinholeCamera camera) throws AlgebraException {
        final var outputCamera = new PinholeCamera();
        transform(camera, outputCamera);
        return outputCamera;
    }

    /**
     * Transforms and updates provided camera using this transformation.
     *
     * @param camera camera to be transformed.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public void transform(final PinholeCamera camera) throws AlgebraException {
        transform(camera, camera);
    }

    /**
     * Transforms a camera using this transformation and stores the result into
     * provided output camera.
     *
     * @param inputCamera  camera to be transformed.
     * @param outputCamera instance where data of transformed camera will be
     *                     stored.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public abstract void transform(final PinholeCamera inputCamera, final PinholeCamera outputCamera)
            throws AlgebraException;
}
