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

import java.io.Serializable;
import java.util.Arrays;

/**
 * This class defines an ellipsoid.
 */
public class Ellipsoid implements Serializable {

    /**
     * Number of dimensions.
     */
    public static final int DIMENSIONS = 3;

    /**
     * Constant to compute approximate surface area.
     */
    private static final double P = 1.6075;

    /**
     * Center of ellipse.
     */
    private Point3D center;


    /**
     * Lengths of all three semi-axes.
     */
    private double[] semiAxesLengths;

    /**
     * Rotation.
     */
    private Rotation3D rotation;

    /**
     * Empty constructor.
     * Creates an ellipsoid equal to a sphere located at space origin (0,0,0)
     * with radius 1.0.
     */
    public Ellipsoid() {
        center = Point3D.create();
        semiAxesLengths = new double[DIMENSIONS];
        Arrays.fill(semiAxesLengths, 1.0);
        rotation = Rotation3D.create();
    }

    /**
     * Sets ellipsoid parameters.
     *
     * @param center          center of ellipsoid.
     * @param semiAxesLengths lengths of all three semi-axes.
     * @param rotation        rotation.
     * @throws IllegalArgumentException if length of provided array is not
     *                                  three.
     */
    public Ellipsoid(final Point3D center, final double[] semiAxesLengths, final Rotation3D rotation) {
        setCenterAxesAndRotation(center, semiAxesLengths, rotation);
    }

    /**
     * Returns center of ellipsoid.
     *
     * @return center of ellipsoid.
     */
    public Point3D getCenter() {
        return center;
    }

    /**
     * Sets center of ellipsoid.
     *
     * @param center center of ellipsoid.
     * @throws NullPointerException raised if provided center is null.
     */
    public void setCenter(final Point3D center) {
        if (center == null) {
            throw new NullPointerException();
        }
        this.center = center;
    }

    /**
     * Gets lengths of all three semi-axes.
     *
     * @return lengths of all three semi-axes.
     */
    public double[] getSemiAxesLengths() {
        return semiAxesLengths;
    }

    /**
     * Sets lengths of all three semi-axes.
     *
     * @param semiAxesLengths lengths of all three semi-axes.
     * @throws IllegalArgumentException if length of provided array is not
     *                                  three.
     */
    public void setSemiAxesLengths(final double[] semiAxesLengths) {
        if (semiAxesLengths.length != DIMENSIONS) {
            throw new IllegalArgumentException();
        }
        this.semiAxesLengths = semiAxesLengths;
    }

    /**
     * Gets rotation.
     *
     * @return rotation.
     */
    public Rotation3D getRotation() {
        return rotation;
    }

    /**
     * Sets rotation.
     *
     * @param rotation rotation.
     */
    public void setRotation(final Rotation3D rotation) {
        this.rotation = rotation;
    }

    /**
     * Sets ellipsoid parameters.
     *
     * @param center          center of ellipsoid.
     * @param semiAxesLengths lengths of all three semi-axes.
     * @param rotation        rotation.
     * @throws IllegalArgumentException if length of provided array is not
     *                                  three.
     */
    public final void setCenterAxesAndRotation(
            final Point3D center, final double[] semiAxesLengths, final Rotation3D rotation) {
        if (semiAxesLengths.length != DIMENSIONS) {
            throw new IllegalArgumentException();
        }
        this.center = center;
        this.semiAxesLengths = semiAxesLengths;
        this.rotation = rotation;
    }

    /**
     * Returns volume of this ellipsoid.
     *
     * @return volume of this ellipsoid.
     */
    public double getVolume() {
        final var a = semiAxesLengths[0];
        final var b = semiAxesLengths[1];
        final var c = semiAxesLengths[2];
        return 4.0 / 3.0 * Math.PI * a * b * c;
    }

    /**
     * Returns surface of this ellipsoid.
     *
     * @return surface of this ellipsoid.
     */
    public double getSurface() {
        final var a = semiAxesLengths[0];
        final var b = semiAxesLengths[1];
        final var c = semiAxesLengths[2];

        return 4.0 * Math.PI * Math.pow((Math.pow(a * b, P) + Math.pow(a * c, P) + Math.pow(b * c, P)) / 3.0, 1.0 / P);
    }

    /**
     * Converts this ellipsoid into a quadric.
     * Quadrics a re a more general representation of ellipsoids.
     *
     * @return a quadric representing this ellipsoid.
     * @throws GeometryException if quadric cannot be determined due to
     *                           numerical instabilities.
     */
    public Quadric toQuadric() throws GeometryException {
        // A quadric has the following matrix form:
        // Q =   [A  D   F   G]
        //       [D  B   E   H]
        //       [F  E   C   I]
        //       [G  H   I   J]
        // [x y z w][A D F G][x] = [x y z w][A*x + D*y + F*z + G*w] =
        //          [D B E H][y]            [D*x + B*y + E*z + H*w]
        //          [F E C I][z]            [F*x + E*y + C*z + I*w]
        //          [G H I J][w]            [G*x + H*y + I*z + J*w]
        // = A*x^2 + D*x*y + F*x*z + G*x*w + D*x*y + B*y^2 + E*y*z + H*y*w +
        // F*x*z + E*y*z + C*z^2 + I*z*w + G*x*w + H*y*w + I*z*w + J*w^2 =
        // = A*x^2 + B*y^2 + C*z^2 + 2*D*x*y + 2*E*y*z + 2*F*x*z + 2*G*x*w + 2*H*y*w + 2*I*z*w + J*w^2
        // which follows expression:
        // A*x^2 + B*y^2 + C*z^2 + 2*D*x*y + 2*E*y*z + 2*F*x*z + 2*G*x*w + 2*H*y*w + 2*I*z*w + J*w^2 = 0

        // An Ellipsoid is a particular form of a quadric following expression:
        // x^2 / a^2 + y^2 / b^2 + z^2 / c^2 = 1
        // when centered at origin and having no rotation

        // if we take an arbitrary center position:
        final var a = 1.0 / Math.pow(semiAxesLengths[0], 2.0);
        final var b = 1.0 / Math.pow(semiAxesLengths[1], 2.0);
        final var c = 1.0 / Math.pow(semiAxesLengths[2], 2.0);
        final var d = 0.0;
        final var e = 0.0;
        final var f = 0.0;
        final var g = 0.0;
        final var h = 0.0;
        final var i = 0.0;
        final var j = -1;

        final var q = new Quadric(a, b, c, d, e, f, g, h, i, j);
        final var t = new EuclideanTransformation3D(rotation, new double[]{
                center.getInhomX(), center.getInhomY(), center.getInhomZ()});

        try {
            t.transform(q);
        } catch (final NonSymmetricMatrixException | AlgebraException ex) {
            throw new GeometryException(ex);
        }

        return q;
    }

    /**
     * Sets parameters of this ellipsoid from a sphere.
     *
     * @param sphere a sphere to set parameters from.
     */
    public final void setFromSphere(final Sphere sphere) {
        center = sphere.getCenter();
        Arrays.fill(semiAxesLengths, sphere.getRadius());
        rotation = Rotation3D.create();
    }
}
