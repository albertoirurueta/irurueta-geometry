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
 * This class contains implementation of a dual conic.
 */
@SuppressWarnings("DuplicatedCode")
public class DualConic extends BaseConic implements Serializable {

    /**
     * Constructor.
     */
    public DualConic() {
        super();
    }

    /**
     * Constructor of this class. This constructor accepts every parameter
     * describing a dual conic (parameters a, b, c, d, e, f).
     *
     * @param a Parameter A of the conic.
     * @param b Parameter B of the conic.
     * @param c Parameter C of the conic.
     * @param d Parameter D of the conic.
     * @param e Parameter E of the conic.
     * @param f Parameter F of the conic.
     */
    public DualConic(final double a, final double b, final double c, final double d, final double e,
                     final double f) {
        super(a, b, c, d, e, f);
    }

    /**
     * This method sets the matrix used to describe a dual conic.
     * This matrix must be 3x3 and symmetric.
     *
     * @param m 3x3 Matrix describing the conic.
     * @throws IllegalArgumentException    Raised when the size of the matrix is
     *                                     not 3x3.
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not
     *                                     symmetric.
     */
    public DualConic(final Matrix m) throws NonSymmetricMatrixException {
        super(m);
    }

    /**
     * Instantiates a dual conic where provided lines belong to its locus.
     *
     * @param line1 1st line.
     * @param line2 2nd line.
     * @param line3 3rd line.
     * @param line4 4th line.
     * @param line5 5th line.
     * @throws CoincidentLinesException Raised if provided lines are coincident
     *                                  (more than one line is equal) or produce a degenerate configuration.
     */
    public DualConic(final Line2D line1, final Line2D line2, final Line2D line3,
                     final Line2D line4, final Line2D line5)
            throws CoincidentLinesException {
        setParametersFromLines(line1, line2, line3, line4, line5);
    }

    /**
     * Checks if provided line is locus of this dual conic, or in other words,
     * checks whether provided line lies within this conic, or whether provided
     * line is tangent to the conic corresponding to this dual conic.
     *
     * @param line      Line2D to be tested.
     * @param threshold Threshold of tolerance to determine whether the line is
     *                  locus or not. This is needed because of limited machine precision. If
     *                  threshold is not provided, then DEFAULT_LOCUS_THRESHOLD is used instead.
     * @return True if provided line is locus of this dual conic, false
     * otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public boolean isLocus(final Line2D line, final double threshold) {

        if (threshold < MIN_THRESHOLD) {
            throw new IllegalArgumentException();
        }

        try {
            normalize();
            final Matrix dualC = asMatrix();
            final Matrix homLine =
                    new Matrix(Line2D.LINE_NUMBER_PARAMS, 1);
            line.normalize();
            homLine.setElementAt(0, 0, line.getA());
            homLine.setElementAt(1, 0, line.getB());
            homLine.setElementAt(2, 0, line.getC());
            final Matrix locusMatrix = homLine.transposeAndReturnNew();
            locusMatrix.multiply(dualC);
            locusMatrix.multiply(homLine);

            return Math.abs(locusMatrix.getElementAt(0, 0)) < threshold;
        } catch (final WrongSizeException ignore) {
            return false;
        }
    }

    /**
     * Checks if provided line is locus of this dual conic, or in other words,
     * checks whether provided line lies within this conic, or whether provided
     * line is tangent to the conic corresponding to this dual conic.
     *
     * @param line Line2D to be tested.
     * @return True if provided line is locus of this dual conic, false
     * otherwise.
     * @see #isLocus(Line2D, double)
     */
    public boolean isLocus(final Line2D line) {
        return isLocus(line, DEFAULT_LOCUS_THRESHOLD);
    }

    /**
     * Computes the angle between two lines in radians.
     *
     * @param lineA First line to be tested.
     * @param lineB Second line to be tested.
     * @return Angle between the two provided lines in radians.
     */
    public double angleBetweenLines(final Line2D lineA, final Line2D lineB) {
        try {
            // retrieve conic as matrix
            normalize();
            final Matrix dualC = asMatrix();
            final Matrix transHomLineA = new Matrix(1, Line2D.LINE_NUMBER_PARAMS);
            lineA.normalize();
            transHomLineA.setElementAt(0, 0, lineA.getA());
            transHomLineA.setElementAt(0, 1, lineA.getB());
            transHomLineA.setElementAt(0, 2, lineA.getC());


            final Matrix tmp = transHomLineA.multiplyAndReturnNew(dualC);
            tmp.multiply(transHomLineA.transposeAndReturnNew()); //This is 
            // homLineA' * dualC * homLineA

            final double normA = tmp.getElementAt(0, 0);

            final Matrix homLineB = new Matrix(Line2D.LINE_NUMBER_PARAMS, 1);
            lineB.normalize();
            homLineB.setElementAt(0, 0, lineB.getA());
            homLineB.setElementAt(1, 0, lineB.getB());
            homLineB.setElementAt(2, 0, lineB.getC());

            homLineB.transpose(tmp);
            tmp.multiply(dualC);
            tmp.multiply(homLineB);

            final double normB = tmp.getElementAt(0, 0);

            transHomLineA.multiply(dualC);
            transHomLineA.multiply(homLineB);
            // This is homLineA' * dualC * homLineB

            final double angleNumerator = transHomLineA.getElementAt(0, 0);

            final double cosTheta = angleNumerator / Math.sqrt(normA * normB);
            return Math.acos(cosTheta);
        } catch (final WrongSizeException ignore) {
            // This will never happen
            return 0.0;
        }
    }

    /**
     * Checks if two lines are perpendicular attending to the geometry defined
     * by this dual conic, or in other words, if lA' * dualC* * lB is zero.
     *
     * @param lineA     First line to be checked.
     * @param lineB     Second line to be checked.
     * @param threshold Threshold of tolerance to determine whether the lines
     *                  are perpendicular or not. This is needed because of limited machine
     *                  precision. If threshold is not provided, then
     *                  DEFAULT_PERPENDICULAR_THRESHOLD is used instead.
     * @return True if provided lines are perpendicular, false otherwise.
     * @throws IllegalArgumentException Raised if provided threshold is negative.
     */
    public boolean arePerpendicularLines(final Line2D lineA, final Line2D lineB,
                                         final double threshold) {
        try {
            // retrieve conic as matrix
            final Matrix transHomLineA = new Matrix(1, Line2D.LINE_NUMBER_PARAMS);
            lineA.normalize();
            transHomLineA.setElementAt(0, 0, lineA.getA());
            transHomLineA.setElementAt(0, 1, lineA.getB());
            transHomLineA.setElementAt(0, 2, lineA.getC());

            final Matrix homLineB =
                    new Matrix(Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH,
                            1);
            lineB.normalize();
            homLineB.setElementAt(0, 0, lineB.getA());
            homLineB.setElementAt(1, 0, lineB.getB());
            homLineB.setElementAt(2, 0, lineB.getC());

            normalize();
            final Matrix dualC = asMatrix();
            transHomLineA.multiply(dualC);
            transHomLineA.multiply(homLineB);
            // This is homLineA' * dualC * homLineB

            final double perpend = transHomLineA.getElementAt(0, 0);

            return Math.abs(perpend) < threshold;
        } catch (final WrongSizeException ignore) {
            // This will never happen
            return false;
        }
    }

    /**
     * Checks if two lines are perpendicular attending to the geometry defined
     * by this dual conic, or in other words, if lA' * dualC* * lB is zero.
     *
     * @param lineA First line to be checked.
     * @param lineB Second line to be checked.
     * @return True if provided lines are perpendicular, false otherwise.
     */
    public boolean arePerpendicularLines(final Line2D lineA, final Line2D lineB) {
        return arePerpendicularLines(lineA, lineB,
                DEFAULT_PERPENDICULAR_THRESHOLD);
    }

    /**
     * Computes the conic corresponding to this dual conic.
     *
     * @return A new conic instance of this dual conic.
     * @throws ConicNotAvailableException Raised if the rank of the dual conic
     *                                    matrix is not complete due to wrong parameters or numerical instability.
     */
    public Conic getConic() throws ConicNotAvailableException {
        final Conic c = new Conic();
        conic(c);
        return c;
    }

    /**
     * Computes the conic corresponding to this dual conic and stores the result
     * in provided instance.
     *
     * @param conic Conic where result is stored.
     * @throws ConicNotAvailableException Raised if the rank of the dual conic
     *                                    matrix is not complete due to wrong parameters or numerical instability.
     */
    public void conic(final Conic conic) throws ConicNotAvailableException {

        final Matrix dualConicMatrix = asMatrix();
        try {
            final Matrix invMatrix = com.irurueta.algebra.Utils.inverse(
                    dualConicMatrix);

            // ensure that resulting matrix after inversion is symmetric
            // by computing the mean of off-diagonal elements
            final double a = invMatrix.getElementAt(0, 0);
            final double b = 0.5 * (invMatrix.getElementAt(0, 1) +
                    invMatrix.getElementAt(1, 0));
            final double c = invMatrix.getElementAt(1, 1);
            final double d = 0.5 * (invMatrix.getElementAt(0, 2) +
                    invMatrix.getElementAt(2, 0));
            final double e = 0.5 * (invMatrix.getElementAt(1, 2) +
                    invMatrix.getElementAt(2, 1));
            final double f = invMatrix.getElementAt(2, 2);
            conic.setParameters(a, b, c, d, e, f);
        } catch (final AlgebraException e) {
            throw new ConicNotAvailableException(e);
        }
    }

    /**
     * Sets parameters of this dual conic so that provided lines lie within it
     * (are locus).
     *
     * @param line1 1st line.
     * @param line2 2nd line.
     * @param line3 3rd line.
     * @param line4 4th line.
     * @param line5 5th line.
     * @throws CoincidentLinesException Raised if lines are coincident or
     *                                  produce a degenerated configuration.
     */
    public final void setParametersFromLines(
            final Line2D line1, final Line2D line2, final Line2D line3,
            final Line2D line4, final Line2D line5) throws CoincidentLinesException {

        try {
            line1.normalize();
            line2.normalize();
            line3.normalize();
            line4.normalize();
            line5.normalize();


            // estimate dual conic that lines inside provided 5 lines
            final Matrix m = new Matrix(5, 6);

            double l1 = line1.getA();
            double l2 = line1.getB();
            double l3 = line1.getC();
            m.setElementAt(0, 0, l1 * l1);
            m.setElementAt(0, 1, 2.0 * l1 * l2);
            m.setElementAt(0, 2, l2 * l2);
            m.setElementAt(0, 3, 2.0 * l1 * l3);
            m.setElementAt(0, 4, 2.0 * l2 * l3);
            m.setElementAt(0, 5, l3 * l3);

            l1 = line2.getA();
            l2 = line2.getB();
            l3 = line2.getC();
            m.setElementAt(1, 0, l1 * l1);
            m.setElementAt(1, 1, 2.0 * l1 * l2);
            m.setElementAt(1, 2, l2 * l2);
            m.setElementAt(1, 3, 2.0 * l1 * l3);
            m.setElementAt(1, 4, 2.0 * l2 * l3);
            m.setElementAt(1, 5, l3 * l3);

            l1 = line3.getA();
            l2 = line3.getB();
            l3 = line3.getC();
            m.setElementAt(2, 0, l1 * l1);
            m.setElementAt(2, 1, 2.0 * l1 * l2);
            m.setElementAt(2, 2, l2 * l2);
            m.setElementAt(2, 3, 2.0 * l1 * l3);
            m.setElementAt(2, 4, 2.0 * l2 * l3);
            m.setElementAt(2, 5, l3 * l3);

            l1 = line4.getA();
            l2 = line4.getB();
            l3 = line4.getC();
            m.setElementAt(3, 0, l1 * l1);
            m.setElementAt(3, 1, 2.0 * l1 * l2);
            m.setElementAt(3, 2, l2 * l2);
            m.setElementAt(3, 3, 2.0 * l1 * l3);
            m.setElementAt(3, 4, 2.0 * l2 * l3);
            m.setElementAt(3, 5, l3 * l3);

            l1 = line5.getA();
            l2 = line5.getB();
            l3 = line5.getC();
            m.setElementAt(4, 0, l1 * l1);
            m.setElementAt(4, 1, 2.0 * l1 * l2);
            m.setElementAt(4, 2, l2 * l2);
            m.setElementAt(4, 3, 2.0 * l1 * l3);
            m.setElementAt(4, 4, 2.0 * l2 * l3);
            m.setElementAt(4, 5, l3 * l3);

            // normalize each row to increase accuracy
            final double[] row = new double[6];
            double rowNorm;

            for (int j = 0; j < 5; j++) {
                m.getSubmatrixAsArray(j, 0, j, 5, row);
                rowNorm = com.irurueta.algebra.Utils.normF(row);
                for (int i = 0; i < 6; i++) {
                    m.setElementAt(j, i, m.getElementAt(j, i) / rowNorm);
                }
            }

            final SingularValueDecomposer decomposer = new SingularValueDecomposer(m);
            decomposer.decompose();

            if (decomposer.getRank() < 5) {
                throw new CoincidentLinesException();
            }

            // the right null-space of m contains the parameters a, b, c, d, e ,f
            // of the conic
            final Matrix v = decomposer.getV();

            // l1^ + 2*l1*l2 + l2^2 + 2*l1*l3 + 2*l2*l3 + l3^2 = 0
            final double a = v.getElementAt(0, 5);
            final double b = v.getElementAt(1, 5);
            final double c = v.getElementAt(2, 5);
            final double d = v.getElementAt(3, 5);
            final double e = v.getElementAt(4, 5);
            final double f = v.getElementAt(5, 5);

            setParameters(a, b, c, d, e, f);
        } catch (final AlgebraException ex) {
            throw new CoincidentLinesException(ex);
        }
    }

    /**
     * Creates a canonical instance of the dual absolute conic in the metric
     * stratum.
     * The intersection of the plane at infinity with the set of planes tangent
     * to the dual absolute quadric produce the dual absolute conic.
     * In other words, The dual absolute conic in the metric stratum is the set
     * of lines tangent to the absolute conic that also lie in the plane at
     * infinity.
     * Both the absolute conic and the dual absolute conic define orthogonality
     * in the metric stratum, and in a purely metric stratum (i.e. when camera
     * is correctly calibrated), their canonical value is equal to the identity.
     *
     * @return a canonical instance of the dual absolute conic.
     */
    public static DualConic createCanonicalDualAbsoluteConic() {
        return new DualConic(1.0, 0.0, 1.0, 0.0, 0.0, 1.0);
    }
}
