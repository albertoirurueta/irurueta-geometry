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
 * Class defining the base interface of any possible quadric.
 * Appropriate subclasses should be used for each quadric type: pure quadrics
 * and dual quadrics.
 */
public abstract class BaseQuadric implements Serializable {
    /**
     * Number of rows of one matrix that contains quadric parameters.
     */
    public static final int BASEQUADRIC_MATRIX_ROW_SIZE = 4;

    /**
     * Number of columns of one matrix that contains quadric parameters.
     */
    public static final int BASEQUADRIC_MATRIX_COLUMN_SIZE = 4;

    /**
     * Number of parameters on a quadric or dual quadric.
     */
    public static final int N_PARAMS = 10;

    /**
     * Threshold above zero to determine whether a point lies inside (is locus
     * of) the given quadric or not.
     */
    public static final double DEFAULT_LOCUS_THRESHOLD = 1e-12;

    /**
     * Threshold above zero to determine whether two points of a quadric are
     * perpendicular or not.
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
     * A element of the matrix defining a quadric.
     */
    private double mA;

    /**
     * B element of the matrix defining a quadric.
     */
    private double mB;

    /**
     * C element of the matrix defining a quadric.
     */
    private double mC;

    /**
     * D element of the matrix defining a quadric.
     */
    private double mD;

    /**
     * E element of the matrix defining a quadric.
     */
    private double mE;

    /**
     * F element of the matrix defining a quadric.
     */
    private double mF;

    /**
     * G element of the matrix defining a quadric.
     */
    private double mG;

    /**
     * H element of the matrix defining a quadric.
     */
    private double mH;

    /**
     * I element of the matrix defining a quadric.
     */
    private double mI;

    /**
     * J element of the matrix defining a quadric.
     */
    private double mJ;

    /**
     * Determines whether this instance is already normalized.
     */
    private boolean mNormalized;

    /**
     * Constructor of this class.
     */
    protected BaseQuadric() {
        mA = mB = mC = mD = mE = mF = mG = mH = mI = mJ = 0.0;
        mNormalized = false;
    }

    /**
     * Constructor of this class. This constructor accepts every parameter
     * describing a base quadric (parameters a, b, c, d, e, f, g, h, i).
     *
     * @param a Parameter A of the base quadric.
     * @param b Parameter B of the base quadric.
     * @param c Parameter C of the base quadric.
     * @param d Parameter D of the base quadric.
     * @param e Parameter E of the base quadric.
     * @param f Parameter F of the base quadric.
     * @param g Parameter G of the base quadric.
     * @param h Parameter H of the base quadric.
     * @param i Parameter I of the base quadric.
     * @param j Parameter J of the base quadric.
     */
    protected BaseQuadric(final double a, final double b, final double c, final double d, final double e,
                          final double f, final double g, final double h, final double i, final double j) {
        setParameters(a, b, c, d, e, f, g, h, i, j);
    }

    /**
     * Constructor. This constructor accepts a Matrix describing a base quadric.
     *
     * @param m 4x4 matrix describing a base quadric.
     * @throws NonSymmetricMatrixException Raised when the quadric matrix is not
     *                                     symmetric.
     * @throws IllegalArgumentException    Raised when the size of the matrix is
     *                                     not 4x4.
     */
    protected BaseQuadric(final Matrix m) throws NonSymmetricMatrixException {
        setParameters(m);
    }

    /**
     * Returns parameter A of the given base quadric.
     *
     * @return Parameter B of a matrix describing a base quadric.
     */
    public double getA() {
        return mA;
    }

    /**
     * Returns parameter B of the given base quadric.
     *
     * @return Parameter B of a matrix describing a base quadric.
     */
    public double getB() {
        return mB;
    }

    /**
     * Returns parameter C of the given base quadric.
     *
     * @return Parameter C of a matrix describing a base quadric.
     */
    public double getC() {
        return mC;
    }

    /**
     * Returns parameter D of the given base quadric.
     *
     * @return Parameter D of a matrix describing a base quadric.
     */
    public double getD() {
        return mD;
    }

    /**
     * Returns parameter E of the given base quadric.
     *
     * @return Parameter E of a matrix describing a base quadric.
     */
    public double getE() {
        return mE;
    }

    /**
     * Returns parameter F of the given base quadric.
     *
     * @return Parameter F of a matrix describing a base quadric.
     */
    public double getF() {
        return mF;
    }

    /**
     * Returns parameter G of the given base quadric.
     *
     * @return Parameter G of a matrix describing a base quadric.
     */
    public double getG() {
        return mG;
    }

    /**
     * Returns parameter H of the given base quadric.
     *
     * @return Parameter H of a matrix describing a base quadric.
     */
    public double getH() {
        return mH;
    }

    /**
     * Returns parameter I of the given base quadric.
     *
     * @return Parameter I of a matrix describing a base quadric.
     */
    public double getI() {
        return mI;
    }

    /**
     * Returns parameter J of the given base quadric.
     *
     * @return Parameter J of a matrix describing a base quadric.
     */
    public double getJ() {
        return mJ;
    }

    /**
     * This method accepts every parameter describing a base quadric (parameters
     * a, b, c, d, e, f, g, h, i,, j).
     *
     * @param a Parameter A of the base quadric.
     * @param b Parameter B of the base quadric.
     * @param c Parameter C of the base quadric.
     * @param d Parameter D of the base quadric.
     * @param e Parameter E of the base quadric.
     * @param f Parameter F of the base quadric.
     * @param g Parameter G of the base quadric.
     * @param h Parameter H of the base quadric.
     * @param i Parameter I of the base quadric.
     * @param j Parameter J of the base quadric.
     */
    public final void setParameters(final double a, final double b, final double c, final double d,
                                    final double e, final double f, final double g, final double h,
                                    final double i, final double j) {
        mA = a;
        mB = b;
        mC = c;
        mD = d;
        mE = e;
        mF = f;
        mG = g;
        mH = h;
        mI = i;
        mJ = j;
        mNormalized = false;
    }

    /**
     * This method sets the matrix used for describing a base quadric.
     * This matrix must be 4x4 and symmetric.
     *
     * @param m                  4x4 Matrix describing a base quadric.
     * @param symmetricThreshold Grade of tolerance to determine whether a
     *                           matrix is symmetric or not. It is used because due to machine precision
     *                           a matrix might not be considered exactly symmetric (by default:
     *                           DEFAULT_SYMMETRIC_THRESHOLD is used).
     * @throws IllegalArgumentException    Raised when the size of the matrix is
     *                                     not 4x4.
     * @throws NonSymmetricMatrixException Raised when the quadric matrix is not
     *                                     symmetric.
     */
    public final void setParameters(final Matrix m, final double symmetricThreshold)
            throws NonSymmetricMatrixException {
        if (m.getRows() != BASEQUADRIC_MATRIX_ROW_SIZE ||
                m.getColumns() != BASEQUADRIC_MATRIX_COLUMN_SIZE) {
            throw new IllegalArgumentException();
        } else {
            if (!Utils.isSymmetric(m, symmetricThreshold)) {
                throw new NonSymmetricMatrixException();
            } else {
                mA = m.getElementAt(0, 0);
                mB = m.getElementAt(1, 1);
                mC = m.getElementAt(2, 2);
                mD = m.getElementAt(0, 1);
                mE = m.getElementAt(2, 1);
                mF = m.getElementAt(2, 0);
                mG = m.getElementAt(3, 0);
                mH = m.getElementAt(3, 1);
                mI = m.getElementAt(3, 2);
                mJ = m.getElementAt(3, 3);
                mNormalized = false;
            }
        }
    }

    /**
     * This method sets the matrix used for describing a base quadric.
     * This matrix must be 4x4 and symmetric.
     *
     * @param m 4x4 matrix describing a base quadric.
     * @throws IllegalArgumentException    Raised when the size of the matrix is
     *                                     not 4x4.
     * @throws NonSymmetricMatrixException Raised when the quadric matrix is not
     *                                     symmetric.
     */
    public final void setParameters(final Matrix m) throws NonSymmetricMatrixException {
        setParameters(m, DEFAULT_SYMMETRIC_THRESHOLD);
    }

    /**
     * This method sets the A parameter of a base quadric.
     *
     * @param a Parameter A of the given base quadric.
     */
    public void setA(final double a) {
        mA = a;
        mNormalized = false;
    }

    /**
     * This method sets the B parameter of a base quadric.
     *
     * @param b Parameter B of the given base quadric.
     */
    public void setB(final double b) {
        mB = b;
        mNormalized = false;
    }

    /**
     * This method sets the C parameter of a base quadric.
     *
     * @param c Parameter C of the given base quadric.
     */
    public void setC(final double c) {
        mC = c;
        mNormalized = false;
    }

    /**
     * This method sets the D parameter of a base quadric.
     *
     * @param d Parameter D of the given base quadric.
     */
    public void setD(final double d) {
        mD = d;
        mNormalized = false;
    }

    /**
     * This method sets the E parameter of a base quadric.
     *
     * @param e Parameter E of the given base quadric.
     */
    public void setE(final double e) {
        mE = e;
        mNormalized = false;
    }

    /**
     * This method sets the F parameter of a base quadric.
     *
     * @param f Parameter F of the given base quadric.
     */
    public void setF(final double f) {
        mF = f;
        mNormalized = false;
    }

    /**
     * This method sets the G parameter of a base quadric.
     *
     * @param g Parameter G of the given base quadric.
     */
    public void setG(final double g) {
        mG = g;
        mNormalized = false;
    }

    /**
     * This method sets the H parameter of a base quadric.
     *
     * @param h Parameter H of the given base quadric.
     */
    public void setH(final double h) {
        mH = h;
        mNormalized = false;
    }

    /**
     * This method sets the "I" parameter of a base quadric.
     *
     * @param i Parameter "I" of the given base quadric.
     */
    public void setI(final double i) {
        mI = i;
        mNormalized = false;
    }

    /**
     * This method sets the J parameter of a base quadric.
     *
     * @param j Parameter J of the given base quadric.
     */
    public void setJ(final double j) {
        mJ = j;
        mNormalized = false;
    }

    /**
     * Returns the matrix that describes this base quadric.
     *
     * @return 4x4 matrix describing this base quadric.
     */
    public Matrix asMatrix() {
        Matrix out = null;
        try {
            out = new Matrix(BASEQUADRIC_MATRIX_ROW_SIZE,
                    BASEQUADRIC_MATRIX_COLUMN_SIZE);
            asMatrix(out);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        return out;
    }

    /**
     * Sets the values in provided matrix corresponding to this base quadric.
     *
     * @param m Provided matrix where values will be stored.
     * @throws IllegalArgumentException Raised if provided matrix is not 4x4.
     */
    public void asMatrix(final Matrix m) {

        if (m.getRows() != BASEQUADRIC_MATRIX_ROW_SIZE ||
                m.getColumns() != BASEQUADRIC_MATRIX_ROW_SIZE) {
            throw new IllegalArgumentException();
        }

        m.setElementAt(0, 0, mA);
        m.setElementAt(1, 1, mB);
        m.setElementAt(2, 2, mC);
        m.setElementAt(3, 3, mJ);
        m.setElementAt(1, 0, mD);
        m.setElementAt(0, 1, mD);
        m.setElementAt(2, 1, mE);
        m.setElementAt(1, 2, mE);
        m.setElementAt(2, 0, mF);
        m.setElementAt(0, 2, mF);
        m.setElementAt(3, 0, mG);
        m.setElementAt(0, 3, mG);
        m.setElementAt(3, 1, mH);
        m.setElementAt(1, 3, mH);
        m.setElementAt(3, 2, mI);
        m.setElementAt(2, 3, mI);
    }

    /**
     * Normalizes the Quadric params using its norm.
     */
    public void normalize() {
        if (!mNormalized) {
            final Matrix m = asMatrix();
            final double norm = Utils.normF(m);

            if (norm > PRECISION) {
                mA /= norm;
                mB /= norm;
                mC /= norm;
                mD /= norm;
                mE /= norm;
                mF /= norm;
                mG /= norm;
                mH /= norm;
                mI /= norm;
                mJ /= norm;
                mNormalized = true;
            }
        }
    }

    /**
     * Returns boolean indicating whether this base quadric has already been
     * normalized.
     *
     * @return True if normalized, false otherwise.
     */
    public boolean isNormalized() {
        return mNormalized;
    }
}
