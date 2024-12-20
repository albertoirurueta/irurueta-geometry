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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;

import java.io.Serializable;

/**
 * Class defining the base interface of any possible conic.
 * Appropriate subclasses should be used for each conic type: pure conics and
 * dual conics.
 */
public abstract class BaseConic implements Serializable {
    /**
     * Number of rows of one matrix that contains conic parameters.
     */
    public static final int BASECONIC_MATRIX_ROW_SIZE = 3;

    /**
     * Number of columns of one matrix that contains conic parameters.
     */
    public static final int BASECONIC_MATRIX_COLUMN_SIZE = 3;

    /**
     * Number of parameters on a conic or dual conic.
     */
    public static final int N_PARAMS = 6;

    /**
     * Threshold above zero used to determine whether a point lies inside
     * (is locus of) the given conic or not.
     */
    public static final double DEFAULT_LOCUS_THRESHOLD = 1e-12;

    /**
     * Threshold above zero used to determine whether two points of a conic
     * are perpendicular or not.
     */
    public static final double DEFAULT_PERPENDICULAR_THRESHOLD = 1e-12;

    /**
     * Minimum allowed threshold.
     */
    public static final double MIN_THRESHOLD = 0.0;

    /**
     * Threshold above zero to determine whether one matrix is symmetric or not.
     */
    private static final double DEFAULT_SYMMETRIC_THRESHOLD = 1e-12;

    /**
     * Machine precision.
     */
    private static final double PRECISION = 1e-12;

    /**
     * "A" element of the matrix defining a conic.
     */
    protected double a;

    /**
     * B element of the matrix defining a conic.
     */
    protected double b;

    /**
     * C element of the matrix defining a conic.
     */
    protected double c;

    /**
     * D element of the matrix defining a conic.
     */
    private double d;

    /**
     * E element of the matrix defining a conic.
     */
    private double e;

    /**
     * F element of the matrix defining a conic.
     */
    private double f;

    /**
     * Determines whether this instance is already mNormalized.
     */
    private boolean normalized;

    /**
     * Constructor of this class.
     */
    protected BaseConic() {
        a = b = c = d = e = f = 0.0;
        normalized = false;
    }

    /**
     * Constructor of this class. This constructor accepts every parameter
     * describing a base conic (parameters a, b, c, d, e, f).
     *
     * @param a Parameter A of the base conic
     * @param b Parameter B of the base conic.
     * @param c Parameter C of the base conic.
     * @param d Parameter D of the base conic.
     * @param e Parameter E of the base conic.
     * @param f Parameter F of the base conic.
     */
    protected BaseConic(final double a, final double b, final double c, final double d, final double e,
                        final double f) {
        setParameters(a, b, c, d, e, f);
    }

    /**
     * Constructor. This constructor accepts a Matrix describing a base conic.
     *
     * @param m 3x3 matrix describing a base conic.
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not
     *                                     symmetric.
     * @throws IllegalArgumentException    Raised when the size of the matrix is
     *                                     not 3x3.
     */
    protected BaseConic(final Matrix m) throws NonSymmetricMatrixException {
        setParameters(m);
    }

    /**
     * Returns parameter A of the given base conic.
     *
     * @return Parameter A of a matrix describing a base conic.
     */
    public double getA() {
        return a;
    }

    /**
     * Returns parameter B of the given base conic.
     *
     * @return Parameter B of a matrix describing a base conic.
     */
    public double getB() {
        return b;
    }

    /**
     * Returns parameter C of the given base conic.
     *
     * @return Parameter C of a matrix describing a base conic.
     */
    public double getC() {
        return c;
    }

    /**
     * Returns parameter D of the given base conic.
     *
     * @return Parameter D of a matrix describing a base conic.
     */
    public double getD() {
        return d;
    }

    /**
     * Returns parameter E of the given base conic.
     *
     * @return Parameter E of a matrix describing a base conic.
     */
    public double getE() {
        return e;
    }

    /**
     * Returns parameter F of the given base conic.
     *
     * @return Parameter F of a matrix describing a base conic.
     */
    public double getF() {
        return f;
    }

    /**
     * This method accepts every parameter describing a base conic (parameters
     * a, b, c, d, e, f).
     *
     * @param a Parameter A of the base conic.
     * @param b Parameter B of the base conic.
     * @param c Parameter C of the base conic.
     * @param d Parameter D of the base conic.
     * @param e Parameter E of the base conic.
     * @param f Parameter F of the base conic.
     */
    public final void setParameters(final double a, final double b, final double c, final double d, final double e,
                                    final double f) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.e = e;
        this.f = f;
        normalized = false;
    }

    /**
     * This method sets the matrix used for describing a base conic.
     * This matrix must be 3x3 and symmetric.
     *
     * @param m                  3x3 Matrix describing a base conic.
     * @param symmetricThreshold Grade of tolerance to determine whether a
     *                           matrix is symmetric or not. Because of machine precision,
     *                           the values may not be exactly equal. (by default:
     *                           DEFAULT_SYMMETRIC_THRESHOLD is used).
     * @throws IllegalArgumentException    Raised when the size of the matrix is
     *                                     not 3x3.
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not
     *                                     symmetric.
     */
    @SuppressWarnings("DuplicatedCode")
    public final void setParameters(final Matrix m, final double symmetricThreshold)
            throws NonSymmetricMatrixException {
        if (m.getRows() != BASECONIC_MATRIX_ROW_SIZE || m.getColumns() != BASECONIC_MATRIX_COLUMN_SIZE) {
            throw new IllegalArgumentException();
        } else {
            if (!Utils.isSymmetric(m, symmetricThreshold)) {
                throw new NonSymmetricMatrixException();
            } else {
                a = m.getElementAt(0, 0);
                b = m.getElementAt(0, 1);
                c = m.getElementAt(1, 1);
                d = m.getElementAt(0, 2);
                e = m.getElementAt(1, 2);
                f = m.getElementAt(2, 2);
                normalized = false;
            }
        }
    }

    /**
     * This method sets the matrix used for describing a base conic.
     * This matrix must be 3x3 and symmetric.
     *
     * @param m 3x3 Matrix describing a base conic.
     * @throws IllegalArgumentException    Raised when the size of the matrix is
     *                                     not 3x3.
     * @throws NonSymmetricMatrixException Raised when the conic matrix is not
     *                                     symmetric.
     */
    public final void setParameters(final Matrix m) throws NonSymmetricMatrixException {
        setParameters(m, DEFAULT_SYMMETRIC_THRESHOLD);
    }

    /**
     * This method sets the A parameter of a base conic.
     *
     * @param a Parameter A of the given base conic.
     */
    public void setA(final double a) {
        this.a = a;
        normalized = false;
    }

    /**
     * This method sets the B parameter of a base conic.
     *
     * @param b Parameter B of the given base conic.
     */
    public void setB(final double b) {
        this.b = b;
        normalized = false;
    }

    /**
     * This method sets the C parameter of a base conic.
     *
     * @param c Parameter C of the given base conic.
     */
    public void setC(final double c) {
        this.c = c;
        normalized = false;
    }

    /**
     * This method sets the D parameter of a base conic.
     *
     * @param d Parameter D of the given base conic.
     */
    public void setD(final double d) {
        this.d = d;
        normalized = false;
    }

    /**
     * This method sets the E parameter of a base conic.
     *
     * @param e Parameter E of the given base conic.
     */
    public void setE(final double e) {
        this.e = e;
        normalized = false;
    }

    /**
     * This method sets the F parameter of a base conic.
     *
     * @param f Parameter F of the given base conic.
     */
    public void setF(final double f) {
        this.f = f;
        normalized = false;
    }

    /**
     * Returns the matrix that describes this base conic.
     *
     * @return 3x3 matrix describing this base conic.
     */
    public Matrix asMatrix() {
        Matrix out = null;
        try {
            out = new Matrix(BASECONIC_MATRIX_ROW_SIZE, BASECONIC_MATRIX_COLUMN_SIZE);
            asMatrix(out);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        return out;
    }

    /**
     * Sets the values in provided matrix corresponding to this base conic.
     *
     * @param m Provided matrix where values will be stored
     * @throws IllegalArgumentException Raised if provided matrix is not 3x3
     */
    public void asMatrix(final Matrix m) {

        if (m.getRows() != BASECONIC_MATRIX_ROW_SIZE || m.getColumns() != BASECONIC_MATRIX_ROW_SIZE) {
            throw new IllegalArgumentException();
        }

        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);
    }

    /**
     * Normalizes the Conic params using its norm
     */
    public void normalize() {
        if (!normalized) {
            final var m = asMatrix();
            final var norm = Utils.normF(m);

            if (!Double.isNaN(norm) && norm > PRECISION) {
                a /= norm;
                b /= norm;
                c /= norm;
                d /= norm;
                e /= norm;
                f /= norm;
                normalized = true;
            }
        }
    }

    /**
     * Returns boolean indicating whether this base conic has already been
     * normalized.
     *
     * @return True if normalized, false otherwise.
     */
    public boolean isNormalized() {
        return normalized;
    }
}
