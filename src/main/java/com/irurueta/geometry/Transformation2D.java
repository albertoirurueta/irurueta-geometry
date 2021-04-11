/*
 * @file
 * This file contains implementation of
 * com.irurueta.geometry.Transformation2D
 *
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 25, 2012
 */
package com.irurueta.geometry;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;

import java.util.ArrayList;
import java.util.List;

/**
 * This class performs transformations on 2D space.
 * Transformations can be applied to any 2D geometric figure.
 */
public abstract class Transformation2D {

    /**
     * Empty constructor.
     */
    protected Transformation2D() {
    }

    /**
     * Transforms provided point using this transformation.
     *
     * @param inputPoint point to be transformed.
     * @return transformed point.
     */
    public Point2D transformAndReturnNew(final Point2D inputPoint) {
        final Point2D outputPoint = Point2D.create();
        transform(inputPoint, outputPoint);
        return outputPoint;
    }

    /**
     * Transforms and updates provided point.
     *
     * @param point point to be transformed and updated.
     */
    public void transform(final Point2D point) {
        transform(point, point);
    }

    /**
     * Transforms input point using this transformation and stores the result in
     * provided output points.
     *
     * @param inputPoint  point to be transformed.
     * @param outputPoint instance where transformed point data will be stored.
     */
    public abstract void transform(final Point2D inputPoint, final Point2D outputPoint);

    /**
     * Transforms provided list of points using this transformation and returns
     * a new one.
     *
     * @param inputPoints points to be transformed.
     * @return transformed points.
     */
    public List<Point2D> transformPointsAndReturnNew(
            final List<Point2D> inputPoints) {
        final List<Point2D> outputPoints = new ArrayList<>(inputPoints.size());
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
    public void transformPoints(final List<Point2D> inputPoints,
                                final List<Point2D> outputPoints) {

        outputPoints.clear();
        for (final Point2D point : inputPoints) {
            outputPoints.add(Transformation2D.this.transformAndReturnNew(point));
        }
    }

    /**
     * Transforms provided list of points using this transformation and
     * overwriting their previous values.
     *
     * @param points points to be transformed and overwritten.
     */
    public void transformAndOverwritePoints(final List<Point2D> points) {

        for (final Point2D point : points) {
            transform(point, point);
        }
    }

    /**
     * Transforms a conic using this transformation and returns a new one.
     *
     * @param inputConic conic to be transformed.
     * @return transformed conic.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output conic matrix is not considered to be symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because of
     *                                     numerical instabilities.
     */
    public Conic transformAndReturnNew(final Conic inputConic)
            throws NonSymmetricMatrixException, AlgebraException {
        final Conic outputConic = new Conic();
        transform(inputConic, outputConic);
        return outputConic;
    }

    /**
     * Transforms and updates provided conic.
     *
     * @param conic conic to be transformed.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting conic matrix is not considered to be symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because of
     *                                     numerical instabilities.
     */
    public void transform(final Conic conic) throws NonSymmetricMatrixException,
            AlgebraException {
        transform(conic, conic);
    }

    /**
     * Transforms a conic using this transformation and stores the result into
     * provided output conic.
     *
     * @param inputConic  conic to be transformed.
     * @param outputConic instance where data of transformed conic will be
     *                    stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output conic matrix is not considered to be symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because of
     *                                     numerical instabilities.
     */
    public abstract void transform(final Conic inputConic, final Conic outputConic)
            throws NonSymmetricMatrixException, AlgebraException;

    /**
     * Transforms a dual conic using this transformation and returns a new one.
     *
     * @param inputDualConic dual conic to be transformed.
     * @return transformed dual conic.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output conic matrix is not considered to be symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because
     *                                     of numerical instabilities.
     */
    public DualConic transformAndReturnNew(final DualConic inputDualConic)
            throws NonSymmetricMatrixException, AlgebraException {
        final DualConic outputDualConic = new DualConic();
        transform(inputDualConic, outputDualConic);
        return outputDualConic;
    }

    /**
     * Transforms and updates a dual conic using this transformation.
     *
     * @param dualConic dual conic to be transformed.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting conic matrix is not considered to be symmetric.
     * @throws AlgebraException            raised if transform cannot be computed because
     *                                     of numerical instabilities.
     */
    public void transform(final DualConic dualConic)
            throws NonSymmetricMatrixException, AlgebraException {
        transform(dualConic, dualConic);
    }

    /**
     * Transforms a dual conic using this transformation and stores the result
     * into provided output dual conic.
     *
     * @param inputDualConic  dual conic to be transformed.
     * @param outputDualConic instance where data of transformed dual conic will
     *                        be stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     *                                     the resulting output dual conic matrix is not considered to be symmetric.
     * @throws AlgebraException            Raised if transform cannot be computed because
     *                                     of numerical instabilities.
     */
    public abstract void transform(
            final DualConic inputDualConic, final DualConic outputDualConic)
            throws NonSymmetricMatrixException, AlgebraException;

    /**
     * Transforms provided line using this transformation and returns a new one.
     *
     * @param inputLine line to be transformed.
     * @return transformed line.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public Line2D transformAndReturnNew(final Line2D inputLine)
            throws AlgebraException {
        final Line2D outputLine = new Line2D();
        transform(inputLine, outputLine);
        return outputLine;
    }

    /**
     * Transforms and updates provided line using this transformation.
     *
     * @param line line to be transformed and updated.
     * @throws AlgebraException if transform cannot be computed because of
     *                          numerical instabilities.
     */
    public void transform(final Line2D line) throws AlgebraException {
        transform(line, line);
    }

    /**
     * Transforms provided input line using this transformation and stores the
     * result into provided output line instance.
     *
     * @param inputLine  line to be transformed.
     * @param outputLine instance where data of transformed line will be stored.
     * @throws AlgebraException Raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public abstract void transform(final Line2D inputLine, final Line2D outputLine)
            throws AlgebraException;

    /**
     * Transforms provided list of lines using this transformation and returns
     * a new one.
     *
     * @param inputLines lines to be transformed.
     * @return transformed lines.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public List<Line2D> transformLinesAndReturnNew(final List<Line2D> inputLines)
            throws AlgebraException {
        final List<Line2D> outputLines = new ArrayList<>(inputLines.size());
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
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public void transformLines(final List<Line2D> inputLines,
                               final List<Line2D> outputLines) throws AlgebraException {

        outputLines.clear();
        for (final Line2D line : inputLines) {
            outputLines.add(Transformation2D.this.transformAndReturnNew(line));
        }
    }

    /**
     * Transforms provided list of lines using this transformation and
     * overwriting their previous values.
     *
     * @param lines lines to be transformed and overwritten.
     * @throws AlgebraException raised if transform cannot be computed because
     *                          of numerical instabilities.
     */
    public void transformAndOverwriteLines(final List<Line2D> lines)
            throws AlgebraException {

        for (final Line2D line : lines) {
            transform(line, line);
        }
    }

    /**
     * Transforms provided polygon using this transformation and returns a new
     * one.
     *
     * @param inputPolygon polygon to be transformed.
     * @return Transformed polygon.
     */
    public Polygon2D transformAndReturnNew(final Polygon2D inputPolygon) {
        final List<Point2D> outVertices = transformPointsAndReturnNew(
                inputPolygon.getVertices());
        try {
            return new Polygon2D(outVertices);
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
    public void transform(final Polygon2D polygon) {
        transformAndOverwritePoints(polygon.getVertices());
    }

    /**
     * Transforms provided input polygon using this transformation and stores
     * the result into provided output polygon instance.
     *
     * @param inputPolygon  polygon to be transformed.
     * @param outputPolygon instance where transformed polygon data will be
     *                      stored.
     */
    public void transform(final Polygon2D inputPolygon, final Polygon2D outputPolygon) {
        try {
            outputPolygon.setVertices(transformPointsAndReturnNew(
                    inputPolygon.getVertices()));
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
     * @return transformed triangle.
     */
    public Triangle2D transformAndReturnNew(final Triangle2D inputTriangle) {
        final Point2D vertex1 = Transformation2D.this.transformAndReturnNew(
                inputTriangle.getVertex1());
        final Point2D vertex2 = Transformation2D.this.transformAndReturnNew(
                inputTriangle.getVertex2());
        final Point2D vertex3 = Transformation2D.this.transformAndReturnNew(
                inputTriangle.getVertex3());
        return new Triangle2D(vertex1, vertex2, vertex3);
    }

    /**
     * Transforms and updates provided input triangle using this transformation.
     *
     * @param triangle triangle to be transformed.
     */
    public void transform(final Triangle2D triangle) {
        transform(triangle, triangle);
    }

    /**
     * Transforms provided input triangle using this transformation and stores
     * the result into provided output triangle instance.
     *
     * @param inputTriangle  triangle to be transformed.
     * @param outputTriangle instance where transformed triangle data will be
     *                       stored
     */
    public void transform(final Triangle2D inputTriangle, final Triangle2D outputTriangle) {
        outputTriangle.setVertex1(transformAndReturnNew(
                inputTriangle.getVertex1()));
        outputTriangle.setVertex2(transformAndReturnNew(
                inputTriangle.getVertex2()));
        outputTriangle.setVertex3(transformAndReturnNew(
                inputTriangle.getVertex3()));
    }

    /**
     * Represents this transformation as a 3x3 matrix.
     * A point can be transformed as T * p, where T is the transformation matrix
     * and p is a point expressed as an homogeneous vector.
     *
     * @return This transformation in matrix form.
     */
    public abstract Matrix asMatrix();

    /**
     * Represents this transformation as a 3x3 matrix and stores the result in
     * provided instance.
     *
     * @param m Instance where transformation matrix will be stored.
     * @throws IllegalArgumentException Raised if provided instance is not a 3x3
     *                                  matrix.
     */
    public abstract void asMatrix(final Matrix m);
}
