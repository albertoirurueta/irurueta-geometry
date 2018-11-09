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
    public Transformation3D() { }
    
    /**
     * Transforms provided point using this transformation and returns a new 
     * one.
     * @param inputPoint point to be transformed.
     * @return a new transformed point.
     */
    public Point3D transformAndReturnNew(Point3D inputPoint) {
        Point3D outputPoint = Point3D.create();
        transform(inputPoint, outputPoint);
        return outputPoint;
    }
    
    /**
     * Transforms and updates provided point.
     * @param point point to be transformed and updated.
     */
    public void transform(Point3D point) {
        transform(point, point);
    }
    
    /**
     * Transforms input point using this transformation and stores the result in
     * provided output points.
     * @param inputPoint point to be transformed.
     * @param outputPoint instance where transformed point data will be stored.
     */
    public abstract void transform(Point3D inputPoint, Point3D outputPoint);
    
    /**
     * Transforms provided list of points using this transformation.
     * @param inputPoints points to be transformed.
     * @return new transformed points.
     */
    public List<Point3D> transformPointsAndReturnNew(
            List<Point3D> inputPoints) {
        List<Point3D> outputPoints = new ArrayList<>(inputPoints.size());
        transformPoints(inputPoints, outputPoints);
        return outputPoints;
    }
    
    /**
     * Transforms provided list of points using this transformation and stores
     * the result in provided output list of points.
     * Notice that any previous content in output list will be removed when
     * calling this method.
     * @param inputPoints points to be transformed.
     * @param outputPoints transformed points.
     */
    public void transformPoints(List<Point3D> inputPoints,
            List<Point3D> outputPoints) {
        
        outputPoints.clear();
        for (Point3D point : inputPoints) {
            outputPoints.add(transformAndReturnNew(point));
        }
    }
    
    /**
     * Transforms provided list of points using this transformation and
     * overwriting their previous values.
     * @param points points to be transformed and overwritten.
     */
    public void transformAndOverwritePoints(List<Point3D> points) {
        
        for (Point3D point: points) {
            transform(point, point);
        }
    }
    
    /**
     * Transforms a quadric using this transformation and returns a new one.
     * @param inputQuadric quadric to be transformed.
     * @return a new transformed quadric.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     * the resulting output quadric matrix is not considered to be symmetric.
     * @throws AlgebraException raised if transform cannot be computed becauseof
     * numerical instabilities.
     */
    public Quadric transformAndReturnNew(Quadric inputQuadric) 
            throws NonSymmetricMatrixException, AlgebraException {
        Quadric outputQuadric = new Quadric();
        transform(inputQuadric, outputQuadric);
        return outputQuadric;
    }
    
    /**
     * Transforms and updates provided quadric.
     * @param quadric quadric to be transformed.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     * the resulting quadric matrix is not considered to be symmetric.
     * @throws AlgebraException raised if transform cannot be computed because of
     * numerical instabilities.
     */
    public void transform(Quadric quadric) throws NonSymmetricMatrixException,
            AlgebraException {
        transform(quadric, quadric);
    }
    
    /**
     * Transforms a quadric using this transformation and stores the result into
     * provided output quadric.
     * @param inputQuadric quadric to be transformed.
     * @param outputQuadric instance where data of transformed quadric will be
     * stored.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     * the resulting output quadric matrix is not considered to be symmetric.
     * @throws AlgebraException raised if transform cannot be computed because of
     * numerical instabilities.
     */
    public abstract void transform(Quadric inputQuadric, Quadric outputQuadric)
            throws NonSymmetricMatrixException, AlgebraException;
    
    /**
     * Transforms a dual quadric using this transformation and returns a new 
     * one.
     * @param inputDualQuadric dual quadric to be transformed.
     * @return a new transformed dual quadric.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     * the resulting output dual quadric matrix is not considered to be 
     * symmetric.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public DualQuadric transformAndReturnNew(DualQuadric inputDualQuadric)
            throws NonSymmetricMatrixException, AlgebraException {
        DualQuadric outputDualQuadric = new DualQuadric();
        transform(inputDualQuadric, outputDualQuadric);
        return outputDualQuadric;
    }
    
    /**
     * Transforms and updates a dual quadric using this transformation.
     * @param dualQuadric dual quadric to be transformed.
     * @throws NonSymmetricMatrixException raised if due to numerical precision
     * the resulting output dual quadric matrix is not considered to be 
     * symmetric.
     * @throws AlgebraException raised if transform cannot be computed because
     * of numerical instabilities.
     */
    public void transform(DualQuadric dualQuadric) 
            throws NonSymmetricMatrixException, AlgebraException {
        transform(dualQuadric, dualQuadric);
    }
    
    /**
     * Transforms a dual quadric using this transformation and stores the result
     * into provided output dual quadric.
     * @param inputDualQuadric dual quadric to be transformed.
     * @param outputDualQuadric instance where data of transformed dual quadric
     * will be stored.
     * @throws NonSymmetricMatrixException Raised if due to numerical precision
     * the resulting output dual quadric matrix is not considered to be 
     * symmetric.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public abstract void transform(DualQuadric inputDualQuadric, 
            DualQuadric outputDualQuadric) throws NonSymmetricMatrixException,
            AlgebraException;
    
    /**
     * Transforms provided plane using this transformation and returns a new 
     * one.
     * @param inputPlane plane to be transformed.
     * @return a new transformed plane.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public Plane transformAndReturnNew(Plane inputPlane) 
            throws AlgebraException {
        Plane outputPlane = new Plane();
        transform(inputPlane, outputPlane);
        return outputPlane;
    }
    
    /**
     * Transforms and updates provided plane using this transformation.
     * @param plane plane to be transformed.
     * @throws AlgebraException raised if transform cannot be computed because
     * of numerical instabilities.
     */
    public void transform(Plane plane) throws AlgebraException {
        transform(plane, plane);
    }
    
    /**
     * Transforms provided input plane using this transformation and stores the
     * result into provided output plane instance.
     * @param inputPlane plane to be transformed.
     * @param outputPlane instance where data of transformed plane will be 
     * stored.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public abstract void transform(Plane inputPlane, Plane outputPlane) 
            throws AlgebraException;
    
    /**
     * Transforms provided list of planes using this transformation.
     * @param inputPlanes planes to be transformed.
     * @return transformed planes.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public List<Plane> transformPlanesAndReturnNew(List<Plane> inputPlanes) 
            throws AlgebraException {
        List<Plane> outputPlanes = new ArrayList<>(inputPlanes.size());
        transformPlanes(inputPlanes, outputPlanes);
        return outputPlanes;
    }
    
    /**
     * Transforms provided list of planes using this transformation and stores
     * the result in provided output list of planes.
     * Notice that any previous content in output list will be removed when
     * calling this method.
     * @param inputPlanes planes to be transformed.
     * @param outputPlanes transformed planes.
     * @throws AlgebraException raised if transform cannot be computed because
     * of numerical instabilities.
     */
    public void transformPlanes(List<Plane> inputPlanes,
            List<Plane> outputPlanes) throws AlgebraException {
        
        outputPlanes.clear();
        for (Plane plane : inputPlanes) {
            outputPlanes.add(transformAndReturnNew(plane));
        }
    }
    
    /**
     * Transforms provided list of planes using this transformation and
     * overwriting their previous values.
     * @param planes planes to be transformed and overwritten.
     * @throws AlgebraException raised if transform cannot be computed because
     * of numerical instabilities.
     */
    public void transformAndOverwritePlanes(List<Plane> planes) 
            throws AlgebraException {
        
        for (Plane plane : planes) {
            transform(plane, plane);
        }
    }
    
    /**
     * Transforms provided line using this transformation and returns a new one.
     * @param inputLine line to be transformed.
     * @return transformed line.
     * @throws CoincidentPlanesException raised if transformation is degenerate
     * and results in planes forming a line being coincident.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public Line3D transformAndReturnNew(Line3D inputLine) 
            throws CoincidentPlanesException, AlgebraException {
        Plane plane1 = transformAndReturnNew(inputLine.getPlane1());
        Plane plane2 = transformAndReturnNew(inputLine.getPlane2());
        
        return new Line3D(plane1, plane2);
    }
    
    /**
     * Transforms and updates provided line using this transformation.
     * @param line line to be transformed.
     * @throws CoincidentPlanesException raised if transformation is degenerate
     * and results in planes forming a line being coincident.
     * @throws AlgebraException raised if transform cannot be computed because
     * of numerical instabilities.
     */
    public void transform(Line3D line) throws CoincidentPlanesException,
            AlgebraException {
        transform(line, line);
    }
    
    /**
     * Transforms provided input line using this transformation and stores the
     * result into provided output line instance.
     * @param inputLine line to be transformed.
     * @param outputLine instance where data of transformed line will be stored.
     * @throws CoincidentPlanesException Raised if transformation is degenerate
     * and results in planes forming a line being coincident.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public void transform(Line3D inputLine, Line3D outputLine) 
            throws CoincidentPlanesException, AlgebraException {

        Plane plane1 = transformAndReturnNew(inputLine.getPlane1());
        Plane plane2 = transformAndReturnNew(inputLine.getPlane2());

        outputLine.setPlanes(plane1, plane2);
    }
    
    /**
     * Transforms provided list of lines using this transformation.
     * @param inputLines lines to be transformed.
     * @return transformed lines.
     * @throws CoincidentPlanesException taised if transformation is degenerate
     * and results in planes forming a line being coincident.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public List<Line3D> transformLines(List<Line3D> inputLines) 
            throws CoincidentPlanesException, AlgebraException {
        List<Line3D> outputLines = new ArrayList<>(inputLines.size());
        transformLines(inputLines, outputLines);
        return outputLines;
    }
    
    /**
     * Transforms provided list of lines using this transformation and stores
     * the result in provided output list of lines.
     * Notice that any previous content in output list will be removed when
     * calling this method.
     * @param inputLines lines to be transformed.
     * @param outputLines transformed lines.
     * @throws CoincidentPlanesException raised if transformation is degenerate
     * and results in planes forming a line being coincident.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public void transformLines(List<Line3D> inputLines, 
            List<Line3D> outputLines) throws CoincidentPlanesException, 
            AlgebraException {
        
        outputLines.clear();
        for (Line3D line : inputLines) {
            outputLines.add(transformAndReturnNew(line));
        }
    }
    
    /**
     * Transforms provided list of lines using this transformation and
     * overwriting their previous values.
     * @param lines lines to be transformed and overwritten.
     * @throws CoincidentPlanesException raised if transformation is degenerate
     * and results in planes forming a line being coincident.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public void transformAndOverwriteLines(List<Line3D> lines) 
            throws CoincidentPlanesException, AlgebraException {
        
        for (Line3D line : lines) {
            transform(line, line);
        }
    }
    
    /**
     * Transforms provided polygon using this transformation and returns a new 
     * one.
     * @param inputPolygon polygon to be transformed.
     * @return a new transformed polygon.
     */
    public Polygon3D transformAndReturnNew(Polygon3D inputPolygon){
        List<Point3D> outVertices = transformPointsAndReturnNew(
                inputPolygon.getVertices());
        try {
            return new Polygon3D(outVertices);
        } catch (NotEnoughVerticesException ignore) {
            //this will never happen because all existing polygons have enough
            //vertices
            return null;
        }
    }
    
    /**
     * Transforms and updates provided polygon using this transformation.
     * @param polygon polygon to be transformed.
     */
    public void transform(Polygon3D polygon) {
        transformAndOverwritePoints(polygon.getVertices());
    }
    
    /**
     * Transforms provided input polygon using this transformation and stores
     * the result into provided output polygon instance.
     * @param inputPolygon polygon to be transformed.
     * @param outputPolygon Instance where transformed polygon data will be
     * stored.
     */
    public void transform(Polygon3D inputPolygon, Polygon3D outputPolygon){
        try {
            outputPolygon.setVertices(transformPointsAndReturnNew(
                    inputPolygon.getVertices()));
        } catch (NotEnoughVerticesException ignore) {
            //this will never happen because all existing polygons have enough
            //vertices
        }
    }
    
    /**
     * Transforms provided triangle using this transformation and returns a new 
     * one.
     * @param inputTriangle triangle to be transformed.
     * @return a new transformed triangle.
     */
    public Triangle3D transformAndReturnNew(Triangle3D inputTriangle) {
        Point3D vertex1 = transformAndReturnNew(inputTriangle.getVertex1());
        Point3D vertex2 = transformAndReturnNew(inputTriangle.getVertex2());
        Point3D vertex3 = transformAndReturnNew(inputTriangle.getVertex3());
        return new Triangle3D(vertex1, vertex2, vertex3);
    }
    
    /**
     * Transforms and updates provided input triangle using this transformation.
     * @param triangle triangle to be transformed.
     */
    public void transform(Triangle3D triangle) {
        transform(triangle, triangle);
    }
    
    /**
     * Transforms provided input triangle using this transformation and stores
     * the result into provided output triangle instance.
     * @param inputTriangle triangle to be transformed.
     * @param outputTriangle instance where transformed triangle data will be
     * stored.
     */
    public void transform(Triangle3D inputTriangle, Triangle3D outputTriangle) {
        transform(inputTriangle.getVertex1(), outputTriangle.getVertex1());
        transform(inputTriangle.getVertex2(), outputTriangle.getVertex2());
        transform(inputTriangle.getVertex3(), outputTriangle.getVertex3());
    }
    
    /**
     * Represents this transformation as a 4x4 matrix.
     * A point can be transformed as T * p, where T is the transformation matrix
     * and p is a point expressed as an homogeneous vector.
     * @return this transformation in matrix form.
     */
    public abstract Matrix asMatrix();
    
    /**
     * Represents this transformation as a 4x4 matrix and stores the result in
     * provided instance.
     * @param m instance where transformation matrix will be stored.
     * @throws IllegalArgumentException raised if provided instance is not a 3x3
     * matrix.
     */
    public abstract void asMatrix(Matrix m) throws IllegalArgumentException;
        
    /**
     * Transforms a camera using this transformation.
     * @param camera camera to be transformed.
     * @return transformed quadric.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public PinholeCamera transformAndReturnNew(PinholeCamera camera) 
            throws AlgebraException {
        PinholeCamera outputCamera = new PinholeCamera();
        transform(camera, outputCamera);
        return outputCamera;
    }
    
    /**
     * Transforms and updates provided camera using this transformation.
     * @param camera camera to be transformed.
     * @throws AlgebraException raised if transform cannot be computed because
     * of numerical instabilities.
     */
    public void transform(PinholeCamera camera) throws AlgebraException {
        transform(camera, camera);
    }
    
    /**
     * Transforms a camera using this transformation and stores the result into
     * provided output camera.
     * @param inputCamera camera to be transformed.
     * @param outputCamera instance where data of transformed camera will be 
     * stored.
     * @throws AlgebraException raised if transform cannot be computed because 
     * of numerical instabilities.
     */
    public abstract void transform(PinholeCamera inputCamera, 
            PinholeCamera outputCamera) throws AlgebraException;
}
