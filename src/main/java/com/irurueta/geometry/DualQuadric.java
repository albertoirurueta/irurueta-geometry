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
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.WrongSizeException;

import java.io.Serializable;

/**
 * This class contains implementation of a dual quadric.
 */
@SuppressWarnings("DuplicatedCode")
public class DualQuadric extends BaseQuadric implements Serializable {

    /**
     * Constructor.
     */
    public DualQuadric() {
        super();
    }

    /**
     * Constructor of this class. This constructor accepts every parameter
     * describing a dual quadric (parameters a, b, c, d, e, f, g, h, i, j).
     *
     * @param a Parameter A of the quadric.
     * @param b Parameter B of the quadric.
     * @param c Parameter C of the quadric.
     * @param d Parameter D of the quadric.
     * @param e Parameter E of the quadric.
     * @param f Parameter F of the quadric.
     * @param g Parameter G of the quadric.
     * @param h Parameter H of the quadric.
     * @param i Parameter I of the quadric.
     * @param j Parameter J of the quadric.
     */
    public DualQuadric(
            final double a, final double b, final double c,
            final double d, final double e, final double f,
            final double g, final double h, final double i,
            final double j) {
        super(a, b, c, d, e, f, g, h, i, j);
    }

    /**
     * This method sets the matrix used to describe a dual quadric.
     * This matrix must be 4x4 and symmetric.
     *
     * @param m 4x4 Matrix describing the quadric.
     * @throws IllegalArgumentException    Raised when the size of the matrix is
     *                                     not 4x4.
     * @throws NonSymmetricMatrixException Raised when the quadric matrix is not
     *                                     symmetric.
     */
    public DualQuadric(final Matrix m) throws NonSymmetricMatrixException {
        super(m);
    }

    /**
     * Creates a dual matrix where provided planes are its locus, or in other
     * words, provided planes are tangent to the quadric corresponding to the
     * created dual quadric.
     *
     * @param plane1 1st plane.
     * @param plane2 2nd plane.
     * @param plane3 3rd plane.
     * @param plane4 4th plane.
     * @param plane5 5th plane.
     * @param plane6 6th plane.
     * @param plane7 7th plane.
     * @param plane8 8th plane.
     * @param plane9 9th plane.
     * @throws CoincidentPlanesException if provided planes are in a
     *                                   configuration where more than one plane is coincident, creating a
     *                                   degeneracy.
     */
    public DualQuadric(
            final Plane plane1, final Plane plane2, final Plane plane3,
            final Plane plane4, final Plane plane5, final Plane plane6,
            final Plane plane7, final Plane plane8, final Plane plane9)
            throws CoincidentPlanesException {
        setParametersFromPlanes(plane1, plane2, plane3, plane4, plane5, plane6,
                plane7, plane8, plane9);
    }

    /**
     * Checks if provided plane is locus of this dual quadric, or in other
     * words, checks whether provided plane lies within this quadric, or whether
     * provided plane is tangent to the quadric corresponding to this dual
     * quadric.
     *
     * @param plane     Plane to be tested.
     * @param threshold Threshold of tolerance to determine whether this plane
     *                  is locus or not. This is needed because of limited machine precision. If
     *                  threshold is not provided, then DEFAULT_LOCUS_THRESHOLD is used instead.
     * @return True if provided plane is locus of this dual quadric, false
     * otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public boolean isLocus(final Plane plane, final double threshold) {

        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        try {
            normalize();
            final Matrix dualQ = asMatrix();
            final Matrix homPlane = new Matrix(Plane.PLANE_NUMBER_PARAMS, 1);
            plane.normalize();
            homPlane.setElementAt(0, 0, plane.getA());
            homPlane.setElementAt(1, 0, plane.getB());
            homPlane.setElementAt(2, 0, plane.getC());
            homPlane.setElementAt(3, 0, plane.getD());
            final Matrix locusMatrix = homPlane.transposeAndReturnNew();
            locusMatrix.multiply(dualQ);
            locusMatrix.multiply(homPlane);

            return Math.abs(locusMatrix.getElementAt(0, 0)) < threshold;
        } catch (final WrongSizeException ignore) {
            return false;
        }
    }

    /**
     * Checks if provided plane is locus of this dual quadric, or in other
     * words, checks whether provided plane lies within this quadric, or whether
     * provided plane is tangent to the quadric corresponding to this dual
     * quadric.
     *
     * @param plane Plane to be tested.
     * @return True if provided plane is locus of this dual quadric, false
     * otherwise.
     * @see #isLocus(Plane, double)
     */
    public boolean isLocus(final Plane plane) {
        return isLocus(plane, DEFAULT_LOCUS_THRESHOLD);
    }

    /**
     * Computes the angle between two planes in radians.
     *
     * @param planeA First plane to be tested.
     * @param planeB Second plane to be tested.
     * @return Angle between the two provided planes in radians. This angle is
     * equal to the angle of their corresponding director vectors in an
     * euclidean geometry, but it might not be the case for the geometry defined
     * by this dual quadric.
     */
    public double angleBetweenPlanes(final Plane planeA, final Plane planeB) {
        try {
            // retrieve quadric as matrix
            normalize();
            final Matrix dualQ = asMatrix();
            final Matrix transHomPlaneA = new Matrix(1, Plane.PLANE_NUMBER_PARAMS);
            planeA.normalize();
            transHomPlaneA.setElementAt(0, 0, planeA.getA());
            transHomPlaneA.setElementAt(0, 1, planeA.getB());
            transHomPlaneA.setElementAt(0, 2, planeA.getC());
            transHomPlaneA.setElementAt(0, 3, planeA.getD());

            final Matrix tmp = transHomPlaneA.multiplyAndReturnNew(dualQ);
            tmp.multiply(transHomPlaneA.transposeAndReturnNew()); //This is
            // homPlaneA' * dualQ * homPlaneA

            final double normA = tmp.getElementAt(0, 0);

            final Matrix homPlaneB = new Matrix(Plane.PLANE_NUMBER_PARAMS, 1);
            planeB.normalize();
            homPlaneB.setElementAt(0, 0, planeB.getA());
            homPlaneB.setElementAt(1, 0, planeB.getB());
            homPlaneB.setElementAt(2, 0, planeB.getC());
            homPlaneB.setElementAt(3, 0, planeB.getD());

            homPlaneB.transpose(tmp);
            tmp.multiply(dualQ);
            tmp.multiply(homPlaneB);

            final double normB = tmp.getElementAt(0, 0);

            transHomPlaneA.multiply(dualQ);
            transHomPlaneA.multiply(homPlaneB);
            // This is homPlaneA' * dualQ * homPlaneB

            final double angleNumerator = transHomPlaneA.getElementAt(0, 0);

            final double cosTheta = angleNumerator / Math.sqrt(normA * normB);
            return Math.acos(cosTheta);
        } catch (final WrongSizeException ignore) {
            // This will never happen
            return 0.0;
        }
    }

    /**
     * Checks if two planes are perpendicular attending to the geometry defined
     * by this dual quadric, or in other words, if lA' * dualQ * lB is zero.
     *
     * @param planeA    First plane to be checked.
     * @param planeB    Second plane to be checked.
     * @param threshold Threshold of tolerance to determine whether the planes
     *                  are perpendicular or not. This is needed because of limited machine
     *                  precision. If threshold is not provided, then
     *                  DEFAULT_PERPENDICULAR_THREHSOLD is used instead.
     * @return True if provided planes are perpendicular, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is
     *                                  negative.
     */
    public boolean arePerpendicularPlanes(
            final Plane planeA, final Plane planeB, final double threshold) {
        try {
            // retrieve quadric as matrix
            final Matrix transHomPlaneA = new Matrix(1, Plane.PLANE_NUMBER_PARAMS);
            planeA.normalize();
            transHomPlaneA.setElementAt(0, 0, planeA.getA());
            transHomPlaneA.setElementAt(0, 1, planeA.getB());
            transHomPlaneA.setElementAt(0, 2, planeA.getC());
            transHomPlaneA.setElementAt(0, 3, planeA.getD());

            final Matrix homPlaneB =
                    new Matrix(Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH,
                            1);
            planeB.normalize();
            homPlaneB.setElementAt(0, 0, planeB.getA());
            homPlaneB.setElementAt(1, 0, planeB.getB());
            homPlaneB.setElementAt(2, 0, planeB.getC());
            homPlaneB.setElementAt(3, 0, planeB.getD());

            normalize();
            final Matrix dualQ = asMatrix();
            transHomPlaneA.multiply(dualQ);
            transHomPlaneA.multiply(homPlaneB);
            // This is homPlaneA' * dualQ * homPlaneB

            final double perpend = transHomPlaneA.getElementAt(0, 0);

            return Math.abs(perpend) < threshold;
        } catch (final WrongSizeException ignore) {
            // This will never happen
            return false;
        }
    }

    /**
     * Checks if two planes are perpendicular attending to the geometry defined
     * by this dual quadric, or in other words, if lA' * dualQ * lB is zero.
     *
     * @param planeA First plane to be checked.
     * @param planeB Second plane to be checked.
     * @return True if provided planes are perpendicular, false otherwise.
     */
    public boolean arePerpendicularPlanes(final Plane planeA, final Plane planeB) {
        return arePerpendicularPlanes(planeA, planeB,
                DEFAULT_PERPENDICULAR_THRESHOLD);
    }

    /**
     * Computes the quadric corresponding to this dual quadric.
     *
     * @return A new quadric instance of this dual quadric.
     * @throws QuadricNotAvailableException Raised if the rank of the dual
     *                                      quadric matrix is not complete due to wrong parameters or numerical
     *                                      instability.
     */
    public Quadric getQuadric() throws QuadricNotAvailableException {
        final Quadric q = new Quadric();
        quadric(q);
        return q;
    }

    /**
     * Computes the quadric correpsonding to this dual quadric and stores the
     * result in provided instance.
     *
     * @param quadric Quadric where result is stored.
     * @throws QuadricNotAvailableException Raised if the rank of the dual
     *                                      quadric matrix is not complete due to wrong parameters or numerical
     *                                      instability.
     */
    public void quadric(final Quadric quadric) throws QuadricNotAvailableException {

        final Matrix dualQuadricMatrix = asMatrix();
        try {
            final Matrix invMatrix = com.irurueta.algebra.Utils.inverse(
                    dualQuadricMatrix);

            final double a = invMatrix.getElementAt(0, 0);
            final double b = invMatrix.getElementAt(1, 1);
            final double c = invMatrix.getElementAt(2, 2);
            final double d = 0.5 * (invMatrix.getElementAt(0, 1) +
                    invMatrix.getElementAt(1, 0));
            final double e = 0.5 * (invMatrix.getElementAt(2, 1) +
                    invMatrix.getElementAt(1, 2));
            final double f = 0.5 * (invMatrix.getElementAt(2, 0) +
                    invMatrix.getElementAt(0, 2));
            final double g = 0.5 * (invMatrix.getElementAt(3, 0) +
                    invMatrix.getElementAt(0, 3));
            final double h = 0.5 * (invMatrix.getElementAt(3, 1) +
                    invMatrix.getElementAt(1, 3));
            final double i = 0.5 * (invMatrix.getElementAt(3, 2) +
                    invMatrix.getElementAt(2, 3));
            final double j = invMatrix.getElementAt(3, 3);
            quadric.setParameters(a, b, c, d, e, f, g, h, i, j);
        } catch (final AlgebraException e) {
            throw new QuadricNotAvailableException(e);
        }
    }

    /**
     * Sets parameters of this dual quadric so that provided planes lie within
     * it (are locus).
     *
     * @param plane1 1st plane.
     * @param plane2 2nd plane.
     * @param plane3 3rd plane.
     * @param plane4 4th plane.
     * @param plane5 5th plane.
     * @param plane6 6th plane.
     * @param plane7 7th plane.
     * @param plane8 8th plane.
     * @param plane9 9th plane.
     * @throws CoincidentPlanesException Raised if planes are coincident or
     *                                   produce a degenerated configuration.
     */
    public final void setParametersFromPlanes(
            final Plane plane1, final Plane plane2, final Plane plane3,
            final Plane plane4, final Plane plane5, final Plane plane6,
            final Plane plane7, final Plane plane8, final Plane plane9)
            throws CoincidentPlanesException {

        // normalize planes to increase accuracy
        plane1.normalize();
        plane2.normalize();
        plane3.normalize();
        plane4.normalize();
        plane5.normalize();
        plane6.normalize();
        plane7.normalize();
        plane8.normalize();
        plane9.normalize();

        try {
            // each plane belonging to a dual quadric follows equation:
            // p' * Q * p = 0 ==>
            // pA^2 + pB^2 + pC^2 + 2*pA*pB + 2*pA*pC + 2*pB*pC + 2*pA*pD +
            // 2*pB*pD + 2*pC*pD + pD^2 = 0

            final Matrix m = new Matrix(9, 10);
            double pA = plane1.getA();
            double pB = plane1.getB();
            double pC = plane1.getC();
            double pD = plane1.getD();
            m.setElementAt(0, 0, pA * pA);
            m.setElementAt(0, 1, pB * pB);
            m.setElementAt(0, 2, pC * pC);
            m.setElementAt(0, 3, 2.0 * pA * pB);
            m.setElementAt(0, 4, 2.0 * pA * pC);
            m.setElementAt(0, 5, 2.0 * pB * pC);
            m.setElementAt(0, 6, 2.0 * pA * pD);
            m.setElementAt(0, 7, 2.0 * pB * pD);
            m.setElementAt(0, 8, 2.0 * pC * pD);
            m.setElementAt(0, 9, pD * pD);
            pA = plane2.getA();
            pB = plane2.getB();
            pC = plane2.getC();
            pD = plane2.getD();
            m.setElementAt(1, 0, pA * pA);
            m.setElementAt(1, 1, pB * pB);
            m.setElementAt(1, 2, pC * pC);
            m.setElementAt(1, 3, 2.0 * pA * pB);
            m.setElementAt(1, 4, 2.0 * pA * pC);
            m.setElementAt(1, 5, 2.0 * pB * pC);
            m.setElementAt(1, 6, 2.0 * pA * pD);
            m.setElementAt(1, 7, 2.0 * pB * pD);
            m.setElementAt(1, 8, 2.0 * pC * pD);
            m.setElementAt(1, 9, pD * pD);
            pA = plane3.getA();
            pB = plane3.getB();
            pC = plane3.getC();
            pD = plane3.getD();
            m.setElementAt(2, 0, pA * pA);
            m.setElementAt(2, 1, pB * pB);
            m.setElementAt(2, 2, pC * pC);
            m.setElementAt(2, 3, 2.0 * pA * pB);
            m.setElementAt(2, 4, 2.0 * pA * pC);
            m.setElementAt(2, 5, 2.0 * pB * pC);
            m.setElementAt(2, 6, 2.0 * pA * pD);
            m.setElementAt(2, 7, 2.0 * pB * pD);
            m.setElementAt(2, 8, 2.0 * pC * pD);
            m.setElementAt(2, 9, pD * pD);
            pA = plane4.getA();
            pB = plane4.getB();
            pC = plane4.getC();
            pD = plane4.getD();
            m.setElementAt(3, 0, pA * pA);
            m.setElementAt(3, 1, pB * pB);
            m.setElementAt(3, 2, pC * pC);
            m.setElementAt(3, 3, 2.0 * pA * pB);
            m.setElementAt(3, 4, 2.0 * pA * pC);
            m.setElementAt(3, 5, 2.0 * pB * pC);
            m.setElementAt(3, 6, 2.0 * pA * pD);
            m.setElementAt(3, 7, 2.0 * pB * pD);
            m.setElementAt(3, 8, 2.0 * pC * pD);
            m.setElementAt(3, 9, pD * pD);
            pA = plane5.getA();
            pB = plane5.getB();
            pC = plane5.getC();
            pD = plane5.getD();
            m.setElementAt(4, 0, pA * pA);
            m.setElementAt(4, 1, pB * pB);
            m.setElementAt(4, 2, pC * pC);
            m.setElementAt(4, 3, 2.0 * pA * pB);
            m.setElementAt(4, 4, 2.0 * pA * pC);
            m.setElementAt(4, 5, 2.0 * pB * pC);
            m.setElementAt(4, 6, 2.0 * pA * pD);
            m.setElementAt(4, 7, 2.0 * pB * pD);
            m.setElementAt(4, 8, 2.0 * pC * pD);
            m.setElementAt(4, 9, pD * pD);
            pA = plane6.getA();
            pB = plane6.getB();
            pC = plane6.getC();
            pD = plane6.getD();
            m.setElementAt(5, 0, pA * pA);
            m.setElementAt(5, 1, pB * pB);
            m.setElementAt(5, 2, pC * pC);
            m.setElementAt(5, 3, 2.0 * pA * pB);
            m.setElementAt(5, 4, 2.0 * pA * pC);
            m.setElementAt(5, 5, 2.0 * pB * pC);
            m.setElementAt(5, 6, 2.0 * pA * pD);
            m.setElementAt(5, 7, 2.0 * pB * pD);
            m.setElementAt(5, 8, 2.0 * pC * pD);
            m.setElementAt(5, 9, pD * pD);
            pA = plane7.getA();
            pB = plane7.getB();
            pC = plane7.getC();
            pD = plane7.getD();
            m.setElementAt(6, 0, pA * pA);
            m.setElementAt(6, 1, pB * pB);
            m.setElementAt(6, 2, pC * pC);
            m.setElementAt(6, 3, 2.0 * pA * pB);
            m.setElementAt(6, 4, 2.0 * pA * pC);
            m.setElementAt(6, 5, 2.0 * pB * pC);
            m.setElementAt(6, 6, 2.0 * pA * pD);
            m.setElementAt(6, 7, 2.0 * pB * pD);
            m.setElementAt(6, 8, 2.0 * pC * pD);
            m.setElementAt(6, 9, pD * pD);
            pA = plane8.getA();
            pB = plane8.getB();
            pC = plane8.getC();
            pD = plane8.getD();
            m.setElementAt(7, 0, pA * pA);
            m.setElementAt(7, 1, pB * pB);
            m.setElementAt(7, 2, pC * pC);
            m.setElementAt(7, 3, 2.0 * pA * pB);
            m.setElementAt(7, 4, 2.0 * pA * pC);
            m.setElementAt(7, 5, 2.0 * pB * pC);
            m.setElementAt(7, 6, 2.0 * pA * pD);
            m.setElementAt(7, 7, 2.0 * pB * pD);
            m.setElementAt(7, 8, 2.0 * pC * pD);
            m.setElementAt(7, 9, pD * pD);
            pA = plane9.getA();
            pB = plane9.getB();
            pC = plane9.getC();
            pD = plane9.getD();
            m.setElementAt(8, 0, pA * pA);
            m.setElementAt(8, 1, pB * pB);
            m.setElementAt(8, 2, pC * pC);
            m.setElementAt(8, 3, 2.0 * pA * pB);
            m.setElementAt(8, 4, 2.0 * pA * pC);
            m.setElementAt(8, 5, 2.0 * pB * pC);
            m.setElementAt(8, 6, 2.0 * pA * pD);
            m.setElementAt(8, 7, 2.0 * pB * pD);
            m.setElementAt(8, 8, 2.0 * pC * pD);
            m.setElementAt(8, 9, pD * pD);

            // normalize each row to increase accuracy
            final double[] row = new double[10];
            double rowNorm;

            for (int j = 0; j < 9; j++) {
                m.getSubmatrixAsArray(j, 0, j, 9, row);
                rowNorm = com.irurueta.algebra.Utils.normF(row);
                for (int i = 0; i < 10; i++)
                    m.setElementAt(j, i, m.getElementAt(j, i) / rowNorm);
            }

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            if (decomposer.getRank() < 9) {
                throw new CoincidentPlanesException();
            }

            // the right null-space of m contains the parameters a, b, c, d, e ,f
            // of the conic
            final Matrix v = decomposer.getV();

            final double a = v.getElementAt(0, 9);
            final double b = v.getElementAt(1, 9);
            final double c = v.getElementAt(2, 9);
            final double d = v.getElementAt(3, 9);

            final double f = v.getElementAt(4, 9);
            final double e = v.getElementAt(5, 9);

            final double g = v.getElementAt(6, 9);
            final double h = v.getElementAt(7, 9);
            final double i = v.getElementAt(8, 9);
            final double j = v.getElementAt(9, 9);

            setParameters(a, b, c, d, e, f, g, h, i, j);
        } catch (final AlgebraException ex) {
            throw new CoincidentPlanesException(ex);
        }
    }

    /**
     * Creates a canonical instance of the dual absolute quadric in the metric
     * stratum.
     * In an ideal metric stratum, in order to preserve orthogonality, the dual
     * absolute quadric is defined as a degenerate dual quadric (i.e. cannot be
     * inverted to obtain a quadric) containing the canonical dual absolute
     * conic (i.e. the identity) in its top left submatrix
     *
     * @return a canonical instance of the dual absolute quadric
     */
    public static DualQuadric createCanonicalDualAbsoluteQuadric() {
        return new DualQuadric(1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0);
    }
}
