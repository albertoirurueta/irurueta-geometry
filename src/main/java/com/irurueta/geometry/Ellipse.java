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
package com.irurueta.geometry;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;

import java.io.Serializable;

/**
 * This class defines an ellipse.
 * This class uses formulas as defined at:
 * <a href="https://en.wikipedia.org/wiki/Ellipse">https://en.wikipedia.org/wiki/Ellipse</a>
 */
@SuppressWarnings("DuplicatedCode")
public class Ellipse implements Serializable {

    /**
     * Constant defining default threshold value used when none is provided.
     */
    public static final double DEFAULT_THRESHOLD = 1e-9;

    /**
     * Constant defining minimum allowed threshold.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Center of ellipse.
     */
    private Point2D center;

    /**
     * Semi-major axis length (a).
     */
    private double semiMajorAxis;

    /**
     * Semi-minor axis length (b).
     */
    private double semiMinorAxis;

    /**
     * Rotation angle.
     */
    private double rotationAngle;

    /**
     * Empty constructor.
     * Creates an ellipse equal to a circle located at space origin (0,0) with
     * radius 1.0.
     */
    public Ellipse() {
        center = Point2D.create();
        semiMajorAxis = semiMinorAxis = 1.0;
        rotationAngle = 0.0;
    }

    /**
     * Constructor.
     *
     * @param center        center of ellipse.
     * @param semiMajorAxis semi-major axis length.
     * @param semiMinorAxis semi-minor axis length.
     * @param rotationAngle rotation angle expressed in radians.
     */
    public Ellipse(final Point2D center, final double semiMajorAxis, final double semiMinorAxis,
                   final double rotationAngle) {
        setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, rotationAngle);
    }

    /**
     * Constructor.
     *
     * @param center        center of ellipse.
     * @param semiMajorAxis semi-major axis length.
     * @param semiMinorAxis semi-minor axis length.
     * @param rotation      2D rotation.
     */
    public Ellipse(final Point2D center, final double semiMajorAxis, final double semiMinorAxis,
                   final Rotation2D rotation) {
        setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, rotation);
    }

    /**
     * Constructor from 2 points, ellipse center and rotation.
     *
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param center center of ellipse.
     * @param theta  rotation angle expressed in radians.
     * @throws ColinearPointsException if points are in a co-linear or degenerate
     *                                 configuration.
     */
    public Ellipse(final Point2D point1, final Point2D point2, final Point2D center, final double theta)
            throws ColinearPointsException {
        setParametersFromPointsCenterAndRotation(point1, point2, center, theta);
    }

    /**
     * Constructor from 5 points.
     *
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param point3 3rd point.
     * @param point4 4th point.
     * @param point5 5th point.
     * @throws ColinearPointsException if points are in a co-linear or degenerate
     *                                 configuration.
     */
    public Ellipse(final Point2D point1, final Point2D point2, final Point2D point3, final Point2D point4,
                   final Point2D point5) throws ColinearPointsException {
        setParametersFromPoints(point1, point2, point3, point4, point5);
    }

    /**
     * Constructor from 5 points.
     *
     * @param point1    1st point.
     * @param point2    2nd point.
     * @param point3    3rd point.
     * @param point4    4th point.
     * @param point5    5th point.
     * @param threshold threshold to determine whether points form an ellipse.
     *                  This is usually a very small value
     * @throws ColinearPointsException if points are in a co-linear or degenerate
     *                                 configuration.
     */
    public Ellipse(final Point2D point1, final Point2D point2, final Point2D point3, final Point2D point4,
                   final Point2D point5, double threshold) throws ColinearPointsException {
        setParametersFromPoints(point1, point2, point3, point4, point5, threshold);
    }

    /**
     * Constructor setting parameters of canonical equation of an ellipse, which
     * is:
     * a*x^2 + b*x*y + c*y^2 + d*x + e*y + f = 0
     *
     * @param a a parameter.
     * @param b b parameter.
     * @param c c parameter.
     * @param d d parameter.
     * @param e e parameter.
     * @param f f parameter.
     * @throws IllegalArgumentException if parameters do not follow
     *                                  b^2 - 4*a*c &lt; 0.0
     */
    public Ellipse(final double a, final double b, final double c, final double d, final double e, final double f) {
        setParameters(a, b, c, d, e, f);
    }

    /**
     * Constructor setting parameters of canonical equation of an ellipse, which
     * is:
     * a*x^2 + b*x*y + c*y^2 + d*x + e*y + f = 0
     *
     * @param a         a parameter.
     * @param b         b parameter.
     * @param c         c parameter.
     * @param d         d parameter.
     * @param e         e parameter.
     * @param f         f parameter.
     * @param threshold threshold to determine whether parameters are valid due
     *                  to machine precision.
     * @throws IllegalArgumentException if parameters do not follow
     *                                  b^2 - 4*A*c &lt; threshold
     */
    public Ellipse(final double a, final double b, final double c, final double d, final double e, final double f,
                   final double threshold) {
        setParameters(a, b, c, d, e, f, threshold);
    }

    /**
     * Constructor.
     *
     * @param conic conic to build ellipse from.
     * @throws IllegalArgumentException if provided conic is not an ellipse.
     */
    public Ellipse(final Conic conic) {
        setFromConic(conic);
    }

    /**
     * Constructor.
     *
     * @param circle a circle to set parameters from.
     */
    public Ellipse(final Circle circle) {
        setFromCircle(circle);
    }


    /**
     * Returns center of ellipse.
     *
     * @return center of ellipse.
     */
    public Point2D getCenter() {
        return center;
    }

    /**
     * Sets center of ellipse.
     *
     * @param center center of ellipse.
     * @throws NullPointerException raised if provided center is null.
     */
    public void setCenter(final Point2D center) {
        if (center == null) {
            throw new NullPointerException();
        }
        this.center = center;
    }

    /**
     * Gets semi-major axis length.
     *
     * @return semi-major axis length.
     */
    public double getSemiMajorAxis() {
        return semiMajorAxis;
    }

    /**
     * Sets semi-major axis length.
     *
     * @param semiMajorAxis semi-major axis length.
     */
    public void setSemiMajorAxis(final double semiMajorAxis) {
        this.semiMajorAxis = semiMajorAxis;
    }

    /**
     * Gets semi-minor axis length.
     *
     * @return semi-minor axis length.
     */
    public double getSemiMinorAxis() {
        return semiMinorAxis;
    }

    /**
     * Sets semi-minor axis length.
     *
     * @param semiMinorAxis semi-minor axis length.
     */
    public void setSemiMinorAxis(final double semiMinorAxis) {
        this.semiMinorAxis = semiMinorAxis;
    }

    /**
     * Gets rotation angle expressed in radians.
     *
     * @return rotation angle expressed in radians.
     */
    public double getRotationAngle() {
        return rotationAngle;
    }

    /**
     * Sets rotation angle expressed in radians.
     *
     * @param rotationAngle rotation angle expressed in radians.
     */
    public void setRotationAngle(final double rotationAngle) {
        this.rotationAngle = rotationAngle;
    }

    /**
     * Gets 2D rotation.
     *
     * @return 2D rotation.
     */
    public Rotation2D getRotation() {
        return new Rotation2D(rotationAngle);
    }

    /**
     * Sets 2D rotation.
     *
     * @param rotation 2D rotation to be set.
     */
    public void setRotation(final Rotation2D rotation) {
        rotationAngle = rotation.getTheta();
    }

    /**
     * Gets parameter A of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @return parameter A.
     */
    public double getA() {
        final var sint = Math.sin(rotationAngle);
        final var cost = Math.cos(rotationAngle);

        final var a = semiMajorAxis;
        final var b = semiMinorAxis;

        final var sint2 = sint * sint;
        final var cost2 = cost * cost;

        final var a2 = a * a;
        final var b2 = b * b;

        return a2 * sint2 + b2 * cost2;
    }

    /**
     * Sets parameter A of canonical ellipse equation.
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @param a parameter A to be set.
     */
    public void setA(final double a) {
        setParameters(a, getB(), getC(), getD(), getE(), getF());
    }

    /**
     * Gets parameter B of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @return parameter B.
     */
    public double getB() {
        final var sint = Math.sin(rotationAngle);
        final var cost = Math.cos(rotationAngle);
        final var a = semiMajorAxis;
        final var b = semiMinorAxis;
        final var a2 = a * a;
        final var b2 = b * b;

        return 2.0 * (b2 - a2) * sint * cost;
    }

    /**
     * Sets parameter B of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @param b parameter B to be set.
     */
    public void setB(final double b) {
        setParameters(getA(), b, getC(), getD(), getE(), getF());
    }

    /**
     * Gets parameter C of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @return parameter C.
     */
    public double getC() {
        final var sint = Math.sin(rotationAngle);
        final var cost = Math.cos(rotationAngle);

        final var a = semiMajorAxis;
        final var b = semiMinorAxis;

        final var sint2 = sint * sint;
        final var cost2 = cost * cost;

        final var a2 = a * a;
        final var b2 = b * b;

        return a2 * cost2 + b2 * sint2;
    }

    /**
     * Sets parameter C of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @param c parameter C to be set.
     */
    public void setC(final double c) {
        setParameters(getA(), getB(), c, getD(), getE(), getF());
    }

    /**
     * Gets parameter D of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @return parameter D.
     */
    public double getD() {
        final var xc = center.getInhomX();
        final var yc = center.getInhomY();

        return -2.0 * getA() * xc - getB() * yc;
    }

    /**
     * Sets parameter D of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @param d parameter D to be set.
     */
    public void setD(final double d) {
        setParameters(getA(), getB(), getC(), d, getE(), getF());
    }

    /**
     * Gets parameter E of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @return parameter E.
     */
    public double getE() {
        final var xc = center.getInhomX();
        final var yc = center.getInhomY();

        return -getB() * xc - 2.0 * getC() * yc;
    }

    /**
     * Sets parameter E of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @param e parameter E to be set.
     */
    public void setE(final double e) {
        setParameters(getA(), getB(), getC(), getD(), e, getF());
    }

    /**
     * Gets parameter F of canonical ellipse equation:
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @return parameter F.
     */
    public double getF() {
        final var xc = center.getInhomX();
        final var yc = center.getInhomY();

        final var a = semiMajorAxis;
        final var b = semiMinorAxis;

        final var xc2 = xc * xc;
        final var yc2 = yc * yc;

        final var a2 = a * a;
        final var b2 = b * b;

        return getA() * xc2 + getB() * xc * yc + getC() * yc2 - a2 * b2;
    }

    /**
     * Sets parameter F of canonical ellipse equation.
     * A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0
     *
     * @param f parameter F to be set.
     */
    public void setF(final double f) {
        setParameters(getA(), getB(), getC(), getD(), getE(), f);
    }

    /**
     * Sets ellipse parameters.
     *
     * @param center        center of ellipse.
     * @param semiMajorAxis semi-major axis length.
     * @param semiMinorAxis semi-minor axis length.
     * @param rotationAngle rotation angle expressed in radians.
     */
    public final void setCenterAxesAndRotation(
            final Point2D center, final double semiMajorAxis, final double semiMinorAxis, final double rotationAngle) {
        this.center = center;
        this.semiMajorAxis = Math.max(semiMajorAxis, semiMinorAxis);
        this.semiMinorAxis = Math.min(semiMinorAxis, semiMajorAxis);
        this.rotationAngle = rotationAngle;
    }

    /**
     * Sets ellipse parameters.
     *
     * @param center        center of ellipse.
     * @param semiMajorAxis semi-major axis length.
     * @param semiMinorAxis semi-minor axis length.
     * @param rotation      2D rotation.
     */
    public final void setCenterAxesAndRotation(
            final Point2D center, final double semiMajorAxis, final double semiMinorAxis, final Rotation2D rotation) {
        setCenterAxesAndRotation(center, semiMajorAxis, semiMinorAxis, rotation.getTheta());
    }

    /**
     * Sets parameters from 2 points, ellipse center and rotation.
     *
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param center center of ellipse.
     * @param theta  rotation angle expressed in radians.
     * @throws ColinearPointsException if points are in a co-linear or degenerate
     *                                 configuration.
     */
    public final void setParametersFromPointsCenterAndRotation(
            final Point2D point1, final Point2D point2, final Point2D center, final double theta)
            throws ColinearPointsException {
        // unknowns: semi-major axis (a) and semi-minor axis (b)

        // equation of an ellipse follows:
        // A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0

        // where
        // sint = sin(theta)
        // cost = cos(theta)

        // A = a^2*sint^2 + b^2*cost^2
        // B = 2*(b^2 - a^2)*sint*cost
        // C = a^2*cost^2 + b^2*sint^2
        // D = -2*A*xc - B*yc = -2*(a^2*sint^2 + b^2*cost^2)*xc - 2*(b^2 - a^2)*sint*cost*yc
        // E = -B*xc - 2*C*yc = -2*(b^2 - a^2)*sint*cost*xc - 2*(a^2*cost^2 + b^2*sint^2)*yc
        // F = A*xc^2 + B*xc*yc + C*yc^2 - a^2*b^2 = (a^2*sint^2 + b^2*cost^2)*xc^2 + 2*(b^2 - a^2)*sint*cost*xc*yc + (a^2*cost^2 + b^2*sint^2)*yc^2 - a^2*b^2

        // (a^2*sint^2 + b^2*cost^2)*x^2 +
        // 2*(b^2 - a^2)*sint*cost*x*y +
        // (a^2*cost^2 + b^2*sint^2)*y^2 +
        // (-2*(a^2*sint^2 + b^2*cost^2)*xc - 2*(b^2 - a^2)*sint*cost*yc)*x +
        // (-2*(b^2 - a^2)*sint*cost*xc - 2*(a^2*cost^2 + b^2*sint^2)*yc)*y +
        // (a^2*sint^2 + b^2*cost^2)*xc^2 + 2*(b^2 - a^2)*sint*cost*xc*yc + (a^2*cost^2 + b^2*sint^2)*yc^2 - a^2*b^2 = 0

        // a^2*sint^2*x^2 + b^2*cost^2*x^2 +
        // 2*b^2*sint*cost*x*y - 2*a^2*sint*cost*x*y +
        // a^2*cost^2*y^2 + b^2*sint^2*y^2 +
        // -2*a^2*sint^2*xc*x -2*b^2*cost^2*xc*x - 2*b^2*sint*cost*yc*x + 2*a^2*sint*cost*yc*x +
        // -2*b^2*sint*cost*xc*y + 2*a^2*sint*cost*xc*y -2*a^2*cost^2*yc*y -2*b^2*sint^2*yc*y +
        // a^2*sint^2*xc^2 + b^2*cost^2*xc^2 + 2*b^2*sint*cost*xc*yc -2*a^2*sint*cost*xc*yc + a^2*cost^2*yc^2 + b^2*sint^2*yc^2 - a^2*b^2 = 0

        // Divide by b^2

        // (a^2/b^2)*sint^2*x^2 + cost^2*x^2 +
        // 2*sint*cost*x*y - (a^2/b^2)*2*sint*cost*x*y +
        // (a^2/b^2)*cost^2*y^2 + sint^2*y^2 +
        // -(a^2/b^2)*2*sint^2*xc*x - 2*cost^2*xc*x - 2*sint*cost*yc*x + (a^2/b^2)*2*sint*cost*yc*x +
        // -2*sint*cost*xc*y + (a^2/b^2)*2*sint*cost*xc*y - (a^2/b^2)*2*cost^2*yc*y - 2*sint^2*yc*y +
        // (a^2/b^2)*sint^2*xc^2 + cost^2*xc^2 + 2*sint*cost*xc*yc - (a^2/b^2)*2*sint*cost*xc*yc + (a^2/b^2)*cost^2*yc^2 + sint^2*yc^2 - a^2 = 0

        // (a^2/b^2)*(sint^2*x^2 - 2*sint*cost*x*y + cost^2*y^2 - 2*sint^2*xc*x + 2*sint*cost*yc*x + 2*sint*cost*xc*y - 2*cost^2*yc*y + sint^2*xc^2 - 2*sint*cost*xc*yc + cost^2*yc^2)
        // - a^2
        // = - cost^2*x^2 - 2*sint*cost*x*y - sint^2*y^2 + 2*cost^2*xc*x + 2*sint*cost*yc*x + 2*sint*cost*xc*y + 2*sint^2*yc*y - cost^2*xc^2 - 2*sint*cost*xc*yc - sint^2*yc^2

        // Unknowns are:
        // a^2/b^2 and a^2

        try {
            final var sint = Math.sin(theta);
            final var cost = Math.cos(theta);

            final var sint2 = sint * sint;
            final var cost2 = cost * cost;
            final var sintcost = sint * cost;

            final var xc = center.getInhomX();
            final var yc = center.getInhomY();
            final var xc2 = xc * xc;
            final var yc2 = yc * yc;

            final var m = new Matrix(2, 2);
            final var b = new double[2];

            final var x = new double[]{
                    point1.getInhomX(),
                    point2.getInhomX(),
            };

            final var y = new double[]{
                    point1.getInhomY(),
                    point2.getInhomY(),
            };

            double x2;
            double y2;
            double rowNorm;
            for (var i = 0; i < 2; i++) {
                x2 = x[i] * x[i];
                y2 = y[i] * y[i];

                final var tmp = 2.0 * sintcost * x[i] * y[i];

                m.setElementAt(i, 0, sint2 * x2 - tmp + cost2 * y2 - 2.0 * sint2 * xc * x[i]
                        + 2.0 * sintcost * yc * x[i] + 2.0 * sintcost * xc * y[i] - 2.0 * cost2 * yc * y[i]
                        + sint2 * xc2 - 2.0 * sintcost * xc * yc + cost2 * yc2);
                m.setElementAt(i, 1, -1.0);

                b[i] = -cost2 * x2 - tmp - sint2 * y2 + 2.0 * cost2 * xc * x[i] + 2.0 * sintcost * yc * x[i]
                        + 2.0 * sintcost * xc * y[i] + 2.0 * sint2 * yc * y[i] - cost2 * xc2 - 2.0 * sintcost * xc * yc
                        - sint2 * yc2;

                // normalize row to increase accuracy
                rowNorm = 0.0;
                for (var j = 0; j < 2; j++) {
                    rowNorm += Math.pow(m.getElementAt(i, j), 2.0);
                }
                rowNorm = Math.sqrt(rowNorm);

                for (var j = 0; j < 2; j++) {
                    m.setElementAt(i, j, m.getElementAt(i, j) / rowNorm);
                }

                b[i] /= rowNorm;
            }

            final var params = com.irurueta.algebra.Utils.solve(m, b);

            // params[1] = a^2
            final var sMajorAxis = Math.sqrt(Math.abs(params[1]));

            // params[0] = a^2/b^2 --> b^2 = a^2/params[0] = params[1]/params[0]
            final var sMinorAxis = Math.sqrt(Math.abs(params[1] / params[0]));

            setCenterAxesAndRotation(center, sMajorAxis, sMinorAxis, theta);
        } catch (final AlgebraException e) {
            throw new ColinearPointsException(e);
        }
    }

    /**
     * Sets parameters from 5 points.
     *
     * @param point1 1st point.
     * @param point2 2nd point.
     * @param point3 3rd point.
     * @param point4 4th point.
     * @param point5 5th point.
     * @throws ColinearPointsException if points are in a co-linear or degenerate
     *                                 configuration.
     */
    public final void setParametersFromPoints(
            final Point2D point1, final Point2D point2, final Point2D point3, final Point2D point4,
            final Point2D point5) throws ColinearPointsException {
        setParametersFromPoints(point1, point2, point3, point4, point5, 0.0);
    }

    /**
     * Sets parameters from 5 points.
     *
     * @param point1    1st point.
     * @param point2    2nd point.
     * @param point3    3rd point.
     * @param point4    4th point.
     * @param point5    5th point.
     * @param threshold threshold to determine whether points form an ellipse.
     *                  This is usually a very small value
     * @throws ColinearPointsException if points are in a co-linear or degenerate
     *                                 configuration.
     */
    public final void setParametersFromPoints(
            final Point2D point1, final Point2D point2, final Point2D point3, final Point2D point4,
            final Point2D point5, final double threshold) throws ColinearPointsException {
        // normalize points to increase accuracy
        point1.normalize();
        point2.normalize();
        point3.normalize();
        point4.normalize();
        point5.normalize();

        try {
            // each point belonging to a conic follows equation:
            // p' * C * p = 0 ==>
            // x^2 + y^2 + w^2 + 2*x*y + 2*x*w + 2*y*w = 0
            final var m = new Matrix(5, 6);
            var x = point1.getHomX();
            var y = point1.getHomY();
            var w = point1.getHomW();
            m.setElementAt(0, 0, x * x);
            m.setElementAt(0, 1, 2.0 * x * y);
            m.setElementAt(0, 2, y * y);
            m.setElementAt(0, 3, 2.0 * x * w);
            m.setElementAt(0, 4, 2.0 * y * w);
            m.setElementAt(0, 5, w * w);
            x = point2.getHomX();
            y = point2.getHomY();
            w = point2.getHomW();
            m.setElementAt(1, 0, x * x);
            m.setElementAt(1, 1, 2.0 * x * y);
            m.setElementAt(1, 2, y * y);
            m.setElementAt(1, 3, 2.0 * x * w);
            m.setElementAt(1, 4, 2.0 * y * w);
            m.setElementAt(1, 5, w * w);
            x = point3.getHomX();
            y = point3.getHomY();
            w = point3.getHomW();
            m.setElementAt(2, 0, x * x);
            m.setElementAt(2, 1, 2.0 * x * y);
            m.setElementAt(2, 2, y * y);
            m.setElementAt(2, 3, 2.0 * x * w);
            m.setElementAt(2, 4, 2.0 * y * w);
            m.setElementAt(2, 5, w * w);
            x = point4.getHomX();
            y = point4.getHomY();
            w = point4.getHomW();
            m.setElementAt(3, 0, x * x);
            m.setElementAt(3, 1, 2.0 * x * y);
            m.setElementAt(3, 2, y * y);
            m.setElementAt(3, 3, 2.0 * x * w);
            m.setElementAt(3, 4, 2.0 * y * w);
            m.setElementAt(3, 5, w * w);
            x = point5.getHomX();
            y = point5.getHomY();
            w = point5.getHomW();
            m.setElementAt(4, 0, x * x);
            m.setElementAt(4, 1, 2.0 * x * y);
            m.setElementAt(4, 2, y * y);
            m.setElementAt(4, 3, 2.0 * x * w);
            m.setElementAt(4, 4, 2.0 * y * w);
            m.setElementAt(4, 5, w * w);

            // normalize each row to increase accuracy
            final var row = new double[6];
            double rowNorm;

            for (var j = 0; j < 5; j++) {
                m.getSubmatrixAsArray(j, 0, j, 5, row);
                rowNorm = com.irurueta.algebra.Utils.normF(row);
                for (var i = 0; i < 6; i++) {
                    m.setElementAt(j, i, m.getElementAt(j, i) / rowNorm);
                }
            }

            final var decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            if (decomposer.getRank() < 5) {
                throw new ColinearPointsException();
            }

            // the right null-space of m contains the parameters a, b, c, d, e ,f
            // of the conic
            final var v = decomposer.getV();

            final var aPrime = v.getElementAt(0, 5);
            final var bPrime = v.getElementAt(1, 5);
            final var cPrime = v.getElementAt(2, 5);
            final var dPrime = v.getElementAt(3, 5);
            final var ePrime = v.getElementAt(4, 5);
            final var fPrime = v.getElementAt(5, 5);

            // an ellipse follows the generic conic equation
            // A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0

            // Or in matrix form
            // [A    B/2 D/2 ]   [A' B'  D']
            // [B/2  C   E/2 ] = [B' C'  E']
            // [D/2  E/2 F   ]   [D' E'  F']

            final var b = 2.0 * bPrime;
            final var d = 2.0 * dPrime;
            final var e = 2.0 * ePrime;

            setParameters(aPrime, b, cPrime, d, e, fPrime, threshold);
        } catch (final AlgebraException | IllegalArgumentException ex) {
            throw new ColinearPointsException(ex);
        }
    }

    /**
     * Sets parameters of canonical equation of an ellipse, which is:
     * a*x^2 + b*x*y + c*y^2 + d*x + e*y + f = 0
     *
     * @param a a parameter.
     * @param b b parameter.
     * @param c c parameter.
     * @param d d parameter.
     * @param e e parameter.
     * @param f f parameter.
     * @throws IllegalArgumentException if parameters do not follow
     *                                  b^2 - 4*a*c &lt; 0.0.
     */
    public final void setParameters(
            final double a, final double b, final double c, final double d, final double e, final double f) {
        setParameters(a, b, c, d, e, f, 0.0);
    }

    /**
     * Sets parameters of canonical equation of an ellipse, which is:
     * a*x^2 + b*x*y + c*y^2 + d*x + e*y + f = 0
     *
     * @param a         a parameter.
     * @param b         b parameter.
     * @param c         c parameter.
     * @param d         d parameter.
     * @param e         e parameter.
     * @param f         f parameter.
     * @param threshold threshold to determine whether parameters are valid due
     *                  to machine precision.
     * @throws IllegalArgumentException if parameters do not follow
     *                                  b^2 - 4*A*c &lt; threshold.
     */
    public final void setParameters(
            final double a, final double b, final double c, final double d, final double e, final double f,
            final double threshold) {
        final var discriminant = b * b - 4.0 * a * c;
        if (discriminant >= threshold) {
            // not an ellipse
            throw new IllegalArgumentException();
        }

        final var tmp1 = a * e * e + c * d * d - b * d * e + discriminant * f;
        final var tmp2 = Math.sqrt((a - c) * (a - c) + b * b);
        final var sMajorAxis = -Math.sqrt(2.0 * tmp1 * (a + c + tmp2)) / discriminant;

        final var sMinorAxis = -Math.sqrt(2.0 * tmp1 * (a + c - tmp2)) / discriminant;

        final var xc = (2.0 * c * d - b * e) / discriminant;
        final var yc = (2.0 * a * e - b * d) / discriminant;
        final var centerPoint = new InhomogeneousPoint2D(xc, yc);

        double alpha;
        if (Math.abs(b) <= threshold && a < c) {
            alpha = 0.0;
        } else {
            alpha = Math.atan2(c - a - tmp2, b);
        }

        setCenterAxesAndRotation(centerPoint, sMajorAxis, sMinorAxis, alpha);
    }

    /**
     * Gets focus distance of ellipse.
     * Focus determines the distance respect to the center of the ellipse
     * where the two focus points are located.
     * The sum of the distances from any point P = P(x,y) on the ellipse to
     * those two foci is constant and equal to the major axis length.
     *
     * @return focus distance.
     */
    public double getFocus() {
        return Math.sqrt(Math.pow(semiMajorAxis, 2.0) - Math.pow(semiMinorAxis, 2.0));
    }

    /**
     * Gets semi major axis x,y coordinates.
     *
     * @param coords array where x, y coordinates of semi major axis will be
     *               stored.
     */
    public void getSemiMajorAxisCoordinates(final double[] coords) {
        coords[0] = semiMajorAxis * Math.cos(rotationAngle);
        coords[1] = semiMajorAxis * Math.sin(rotationAngle);
    }

    /**
     * Gets semi major axis x,y coordinates.
     *
     * @return array containing x, y coordinates of semi major axis.
     */
    public double[] getSemiMajorAxisCoordinates() {
        final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        getSemiMajorAxisCoordinates(coords);
        return coords;
    }

    /**
     * Sets semi major axis coordinates.
     * This method updates rotation angle and semi major axis length.
     *
     * @param coords coordinates of semi major axis.
     */
    public void setSemiMajorAxisCoordinates(final double[] coords) {
        rotationAngle = Math.atan2(coords[1], coords[0]);
        semiMajorAxis = com.irurueta.algebra.Utils.normF(coords);
    }

    /**
     * Gets semi minor axis x,y coordinates.
     *
     * @param coords array where x, y coordinates of semi minor axis will be
     *               stored.
     */
    public void getSemiMinorAxisCoordinates(final double[] coords) {
        coords[0] = -semiMinorAxis * Math.sin(rotationAngle);
        coords[1] = semiMinorAxis * Math.cos(rotationAngle);
    }

    /**
     * Gets semi minor axis x,y coordinates.
     *
     * @return array containing x,y coordinates of semi minor axis.
     */
    public double[] getSemiMinorAxisCoordinates() {
        final var coords = new double[Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH];
        getSemiMinorAxisCoordinates(coords);
        return coords;
    }

    /**
     * Sets semi minor axis coordinates.
     * This method updates rotation angle and semi minor axis length.
     *
     * @param coords coordinates of semi minor axis.
     */
    public void setSemiMinorAxisCoordinates(final double[] coords) {
        rotationAngle = Math.atan2(-coords[0], coords[1]);
        semiMinorAxis = com.irurueta.algebra.Utils.normF(coords);
    }

    /**
     * Gets 1st focus point.
     * The sum of the distances from any point P = P(x,y) on the ellipse to
     * this focus is constant and equal to the major axis length.
     *
     * @param focusPoint1 1st focus point.
     */
    public void getFocusPoint1(final Point2D focusPoint1) {
        final var focus = getFocus();

        focusPoint1.setInhomogeneousCoordinates(center.getInhomX() - focus * Math.cos(rotationAngle),
                center.getInhomY() - focus * Math.sin(rotationAngle));
    }

    /**
     * Gets 1st focus point.
     * The sum of the distances from any point P = P(x,y) on the ellipse to
     * this focus is constant and equal to the major axis length.
     *
     * @return 1st focus point.
     */
    public Point2D getFocusPoint1() {
        final var result = Point2D.create();
        getFocusPoint1(result);
        return result;
    }

    /**
     * Gets 2nd focus point.
     * The sum of the distances from any point P = P(x,y) on the ellipse to
     * this focus is constant and equal to the major axis length.
     *
     * @param focusPoint2 2nd focus point.
     */
    public void getFocusPoint2(final Point2D focusPoint2) {
        final var focus = getFocus();

        focusPoint2.setInhomogeneousCoordinates(center.getInhomX() + focus * Math.cos(rotationAngle),
                center.getInhomY() + focus * Math.sin(rotationAngle));
    }

    /**
     * Gets 2nd focus point.
     * The sum of the distances from any point P = P(x,y) on the ellipse to
     * this focus is constant and equal to the major axis length.
     *
     * @return 2nd focus point.
     */
    public Point2D getFocusPoint2() {
        final var result = Point2D.create();
        getFocusPoint2(result);
        return result;
    }

    /**
     * Sets focus points.
     *
     * @param focusPoint1       1st focus point.
     * @param focusPoint2       2nd focus point.
     * @param keepSemiMinorAxis true indicates that semi-minor axis is kept,
     *                          false indicates that semi-major axis is kept instead.
     */
    public void setFocusPoints(final Point2D focusPoint1, final Point2D focusPoint2, final boolean keepSemiMinorAxis) {
        final var c = new InhomogeneousPoint2D((focusPoint1.getInhomX() + focusPoint2.getInhomX()) / 2.0,
                (focusPoint1.getInhomY() + focusPoint2.getInhomY()) / 2.0);
        final var alpha = Math.atan2(focusPoint2.getInhomY() - focusPoint1.getInhomY(),
                focusPoint2.getInhomX() - focusPoint1.getInhomX());

        final var f = focusPoint1.distanceTo(c);
        final var f2 = f * f;
        final double a2;
        final double b2;
        if (keepSemiMinorAxis) {
            b2 = semiMinorAxis * semiMinorAxis;
            a2 = f2 + b2;
        } else {
            a2 = semiMajorAxis * semiMajorAxis;
            b2 = a2 - f2;
        }

        final var sMajorAxis = Math.sqrt(a2);
        final var sMinorAxis = Math.sqrt(b2);

        setCenterAxesAndRotation(c, sMajorAxis, sMinorAxis, alpha);
    }

    /**
     * Gets eccentricity of ellipsis.
     *
     * @return eccentricity of ellipsis.
     */
    public double getEccentricity() {
        return getFocus() / semiMajorAxis;
    }

    /**
     * Returns area of this ellipse.
     *
     * @return area of this ellipse.
     */
    public double getArea() {
        return Math.PI * semiMajorAxis * semiMinorAxis;
    }

    /**
     * Returns perimeter of this ellipse.
     *
     * @return Perimeter of this ellipse.
     */
    public double getPerimeter() {
        final var a = semiMajorAxis;
        final var b = semiMinorAxis;

        return Math.PI * (3.0 * (a + b) - Math.sqrt((3.0 * a + b) * (a + 3.0 * b)));
    }

    /**
     * Gets curvature of ellipse at provided point.
     *
     * @param point point to be checked.
     * @return curvature of ellipse.
     */
    public double getCurvature(final Point2D point) {
        final var a = semiMajorAxis;
        final var b = semiMinorAxis;

        final var a2 = a * a;
        final var b2 = b * b;

        final var a4 = a2 * a2;
        final var b4 = b2 * b2;

        final var x = point.getInhomX() - center.getInhomX();
        final var y = point.getInhomY() - center.getInhomY();

        final var x2 = x * x;
        final var y2 = y * y;

        return Math.pow(x2 / a4 + y2 / b4, -3.0 / 2.0) / (a2 * b2);
    }

    /**
     * Converts this circle into a conic.
     * Conics are a more general representation of circles.
     *
     * @return A conic representing this circle
     */
    public Conic toConic() {
        center.normalize();
        // use inhomogeneous center coordinates
        final var xc = center.getInhomX();
        final var yc = center.getInhomY();

        final var sint = Math.sin(rotationAngle);
        final var cost = Math.cos(rotationAngle);

        final var a = semiMajorAxis;
        final var b = semiMinorAxis;

        final var xc2 = xc * xc;
        final var yc2 = yc * yc;

        final var sint2 = sint * sint;
        final var cost2 = cost * cost;

        final var a2 = a * a;
        final var b2 = b * b;

        final var aParam = a2 * sint2 + b2 * cost2;
        final var bParam = 2.0 * (b2 - a2) * sint * cost;
        final var cParam = a2 * cost2 + b2 * sint2;
        final var dParam = -2.0 * aParam * xc - bParam * yc;
        final var eParam = -bParam * xc - 2.0 * cParam * yc;
        final var fParam = aParam * xc2 + bParam * xc * yc + cParam * yc2 - a2 * b2;

        final var bConic = bParam / 2.0;
        final var dConic = dParam / 2.0;
        final var eConic = eParam / 2.0;

        return new Conic(aParam, bConic, cParam, dConic, eConic, fParam);
    }

    /**
     * Set parameters of this circle from a valid conic corresponding to a
     * circle.
     *
     * @param conic conic to set parameters from.
     * @throws IllegalArgumentException if provided conic is not an ellipse.
     */
    public final void setFromConic(final Conic conic) {
        if (conic.getConicType() != ConicType.ELLIPSE_CONIC_TYPE
                && conic.getConicType() != ConicType.CIRCLE_CONIC_TYPE) {
            throw new IllegalArgumentException();
        }

        conic.normalize();

        final var aConic = conic.getA();
        final var bConic = conic.getB();
        final var cConic = conic.getC();
        final var dConic = conic.getD();
        final var eConic = conic.getE();
        final var fConic = conic.getF();

        // an ellipse follows the generic conic equation
        // A*x^2 + B*x*y + C*y^2 + D*x + E*y + F = 0

        // Or in matrix form
        // [A    B/2 D/2 ]   [A' B'  D']
        // [B/2  C   E/2 ] = [B' C'  E']
        // [D/2  E/2 F   ]   [D' E'  F']

        final var b = 2.0 * bConic;
        final var d = 2.0 * dConic;
        final var e = 2.0 * eConic;

        setParameters(aConic, b, cConic, d, e, fConic);
    }

    /**
     * Sets parameters of this ellipse from a circle.
     *
     * @param circle a circle to set parameters from.
     */
    public final void setFromCircle(final Circle circle) {
        center = circle.getCenter();
        semiMajorAxis = semiMinorAxis = circle.getRadius();
        rotationAngle = 0.0;
    }

    /**
     * Determines if provided point is inside this ellipse or not up to a
     * certain threshold.
     * If provided threshold is positive, the ellipse behaves as if it was a
     * larger ellipse increased by threshold amount, if provided threshold is
     * negative, the ellipse behaves as if it was a smaller ellipse decreased by
     * threshold amount in radius.
     *
     * @param point     Point to be checked.
     * @param threshold Threshold to determine if point is inside or not.
     * @return True if point is considered to be inside this circle, false
     * otherwise.
     */
    public boolean isInside(final Point2D point, final double threshold) {
        center.normalize();
        // use inhomogeneous center coordinates
        final var xc = center.getInhomX();
        final var yc = center.getInhomY();

        final var sint = Math.sin(rotationAngle);
        final var cost = Math.cos(rotationAngle);

        final var a = semiMajorAxis;
        final var b = semiMinorAxis;

        final var xc2 = xc * xc;
        final var yc2 = yc * yc;

        final var sint2 = sint * sint;
        final var cost2 = cost * cost;

        final var a2 = a * a;
        final var b2 = b * b;

        final var aParam = a2 * sint2 + b2 * cost2;
        final var bParam = 2.0 * (b2 - a2) * sint * cost;
        final var cParam = a2 * cost2 + b2 * sint2;
        final var dParam = -2.0 * aParam * xc - bParam * yc;
        final var eParam = -bParam * xc - 2.0 * cParam * yc;
        final var fParam = aParam * xc2 + bParam * xc * yc + cParam * yc2 - a2 * b2;

        final var x = point.getInhomX();
        final var y = point.getInhomY();

        return aParam * x * x + bParam * x * y + cParam * y * y + dParam * x + eParam * y + fParam <= threshold;
    }

    /**
     * Determines if provided point is inside this circle or not.
     *
     * @param point Point to be checked.
     * @return True if point is considered to be inside this circle, false
     * otherwise.
     */
    public boolean isInside(final Point2D point) {
        return isInside(point, 0.0);
    }

    /**
     * Determines whether provided point lies at ellipse boundary or not up to
     * a certain threshold.
     *
     * @param point     Point to be checked.
     * @param threshold A small threshold to determine whether point lies at
     *                  ellipse boundary.
     * @return True if point lies at ellipse boundary, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is
     *                                  negative.
     */
    public boolean isLocus(final Point2D point, final double threshold) {
        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        center.normalize();
        // use inhomogeneous center coordinates
        final var xc = center.getInhomX();
        final var yc = center.getInhomY();

        final var sint = Math.sin(rotationAngle);
        final var cost = Math.cos(rotationAngle);

        final var a = semiMajorAxis;
        final var b = semiMinorAxis;

        final var xc2 = xc * xc;
        final var yc2 = yc * yc;

        final var sint2 = sint * sint;
        final var cost2 = cost * cost;

        final var a2 = a * a;
        final var b2 = b * b;

        final var aParam = a2 * sint2 + b2 * cost2;
        final var bParam = 2.0 * (b2 - a2) * sint * cost;
        final var cParam = a2 * cost2 + b2 * sint2;
        final var dParam = -2.0 * aParam * xc - bParam * yc;
        final var eParam = -bParam * xc - 2.0 * cParam * yc;
        final var fParam = aParam * xc2 + bParam * xc * yc + cParam * yc2 - a2 * b2;

        final var x = point.getInhomX();
        final var y = point.getInhomY();

        return Math.abs(aParam * x * x + bParam * x * y + cParam * y * y + dParam * x + eParam * y + fParam) <= threshold;
    }

    /**
     * Determines whether provided point lies at ellipse boundary or not.
     *
     * @param point Point to be checked.
     * @return True if point lies at ellipse boundary, false otherwise.
     */
    public boolean isLocus(final Point2D point) {
        return isLocus(point, DEFAULT_THRESHOLD);
    }

    /**
     * Returns a line tangent to this ellipse at provided point. Provided point
     * must be locus of this ellipse, otherwise a NotLocusException will be
     * thrown.
     *
     * @param point a locus point of this ellipse.
     * @return a 2D line tangent to this ellipse at provided point.
     * @throws NotLocusException if provided point is not locus of this ellipse
     *                           up to DEFAULT_THRESHOLD.
     */
    public Line2D getTangentLineAt(final Point2D point) throws NotLocusException {
        return getTangentLineAt(point, DEFAULT_THRESHOLD);
    }

    /**
     * Returns a line tangent to this ellipse at provided point. Provided point
     * must be locus of this ellipse, otherwise a NotLocusException will be
     * thrown.
     *
     * @param point     a locus point of this circle.
     * @param threshold threshold to determine if provided point is locus.
     * @return a 2D line tangent to this circle at provided point.
     * @throws NotLocusException        if provided point is not locus of this circle
     *                                  up to provided threshold.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public Line2D getTangentLineAt(final Point2D point, final double threshold) throws NotLocusException {
        final var line = new Line2D();
        tangentLineAt(point, line, threshold);
        return line;
    }

    /**
     * Computes a line tangent to this circle at provided point. Provided point
     * must be locus of this circle, otherwise a NotLocusException will be
     * thrown.
     *
     * @param point     a locus point of this circle.
     * @param line      instance of a 2D line where result will be stored.
     * @param threshold threshold to determine if provided point is locus.
     * @throws NotLocusException        if provided point is not locus of this circle
     *                                  up to provided threshold.
     * @throws IllegalArgumentException if provided threshold is negative.
     */
    public void tangentLineAt(final Point2D point, final Line2D line, final double threshold) throws NotLocusException {
        if (!isLocus(point, threshold)) {
            throw new NotLocusException();
        }

        final var c = toConic();
        c.tangentLineAt(point, line, threshold);
    }
}
