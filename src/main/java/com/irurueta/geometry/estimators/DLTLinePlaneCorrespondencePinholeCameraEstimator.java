/*
 * Copyright (C) 2013 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.Plane;

import java.util.List;

/**
 * This class implements an algorithm to estimate pinhole camera using the DLT
 * algorithm and point correspondences.
 */
@SuppressWarnings("DuplicatedCode")
public class DLTLinePlaneCorrespondencePinholeCameraEstimator extends LinePlaneCorrespondencePinholeCameraEstimator {

    /**
     * Minimum number of required equations to estimate a pinhole camera.
     */
    public static final int MIN_NUMBER_OF_EQUATIONS = 11;

    /**
     * Indicates if by default an LMSE (Least Mean Square Error) solution is
     * allowed if more correspondences than the minimum are provided.
     */
    public static final boolean DEFAULT_ALLOW_LMSE_SOLUTION = false;

    /**
     * Defines tiny value considered as machine precision.
     */
    public static final double EPS = 1e-8;

    /**
     * Indicates if an LMSE (Least Mean Square Error) solution is allowed if
     * more correspondences than the minimum are provided. If false, the
     * exceeding correspondences will be ignored and only the 6 first
     * correspondences will be used.
     */
    private boolean allowLMSESolution;

    /**
     * Constructor.
     */
    public DLTLinePlaneCorrespondencePinholeCameraEstimator() {
        super();
        allowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }

    /**
     * Constructor with listener.
     *
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or estimation progress changes.
     */
    public DLTLinePlaneCorrespondencePinholeCameraEstimator(final PinholeCameraEstimatorListener listener) {
        super(listener);
        allowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }

    /**
     * Constructor.
     *
     * @param planes  list of corresponding 3D planes.
     * @param lines2D list of corresponding 2D lines.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException  if provided lists of correspondences
     *                                  don't have the same size and enough correspondences.
     */
    public DLTLinePlaneCorrespondencePinholeCameraEstimator(
            final List<Plane> planes, final List<Line2D> lines2D) throws WrongListSizesException {
        super(planes, lines2D);
        allowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }

    /**
     * Constructor.
     *
     * @param planes   list of corresponding 3D planes.
     * @param lines2D  list of corresponding 2D lines.
     * @param listener listener to be notified of events such as when estimation
     *                 starts, ends or estimation progress changes.
     * @throws IllegalArgumentException if any of the lists are null.
     * @throws WrongListSizesException  if provided lists of correspondences
     *                                  don't have the same size and enough correspondences.
     */
    public DLTLinePlaneCorrespondencePinholeCameraEstimator(
            final List<Plane> planes, final List<Line2D> lines2D, final PinholeCameraEstimatorListener listener)
            throws WrongListSizesException {
        super(planes, lines2D, listener);
        allowLMSESolution = DEFAULT_ALLOW_LMSE_SOLUTION;
    }

    /**
     * Indicates if an LMSE (Least Mean Square Error) solution is allowed if
     * more correspondences than the minimum are provided. If false, the
     * exceeding correspondences will be ignored and only the 6 first
     * correspondences will be used.
     *
     * @return true if LMSE solution is allowed, false otherwise.
     */
    public boolean isLMSESolutionAllowed() {
        return allowLMSESolution;
    }

    /**
     * Specifies if an LMSE (Least Mean Square Error) solution is allowed if
     * more correspondences than the minimum are provided. If false, the
     * exceeding correspondences will be ignored and only the 6 first
     * correspondences will be used.
     *
     * @param allowed true if LMSE solution is allowed, false otherwise
     * @throws LockedException if estimator is locked.
     */
    public void setLMSESolutionAllowed(final boolean allowed) throws LockedException {
        if (isLocked()) {
            throw new LockedException();
        }
        allowLMSESolution = allowed;
    }

    /**
     * Indicates if this estimator is ready to start the estimation.
     *
     * @return true if estimator is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return areListsAvailable();
    }

    /**
     * Estimates a pinhole camera.
     *
     * @return estimated pinhole camera.
     * @throws LockedException                 if estimator is locked.
     * @throws NotReadyException               if input has not yet been provided.
     * @throws PinholeCameraEstimatorException if an error occurs during
     *                                         estimation, usually because input data is not valid.
     */
    @Override
    public PinholeCamera estimate() throws LockedException, NotReadyException, PinholeCameraEstimatorException {

        if (isLocked()) {
            throw new LockedException();
        }
        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            final var nLines = lines2D.size();

            locked = true;
            if (listener != null) {
                listener.onEstimateStart(this);
            }

            final Matrix a;
            if (isLMSESolutionAllowed()) {
                // initialize new matrix to zero when LMSE is enabled
                a = new Matrix(3 * nLines, 12);
            } else {
                // When LMSE is disabled, initialize new matrix to zero only with
                // 11 equations
                a = new Matrix(MIN_NUMBER_OF_EQUATIONS, 12);
            }

            final var iterator2D = lines2D.iterator();
            final var iterator3D = planes.iterator();

            Line2D line2D;
            Plane plane;
            var counter = 0;
            double la;
            double lb;
            double lc;
            double pA;
            double pB;
            double pC;
            double pD;
            double rowNorm;
            while (iterator2D.hasNext() && iterator3D.hasNext()) {
                line2D = iterator2D.next();
                plane = iterator3D.next();

                // normalize lines and planes to increase accuracy
                line2D.normalize();
                plane.normalize();

                la = line2D.getA();
                lb = line2D.getB();
                lc = line2D.getC();

                pA = plane.getA();
                pB = plane.getB();
                pC = plane.getC();
                pD = plane.getD();

                // first row
                a.setElementAt(counter, 0, -pD * la);
                a.setElementAt(counter, 1, -pD * lb);
                a.setElementAt(counter, 2, -pD * lc);

                // columns 3, 4, 5, 6, 7, 8 are left with zero values

                a.setElementAt(counter, 9, pA * la);
                a.setElementAt(counter, 10, pA * lb);
                a.setElementAt(counter, 11, pA * lc);

                // normalize row
                rowNorm = Math.sqrt(Math.pow(a.getElementAt(counter, 0), 2.0)
                        + Math.pow(a.getElementAt(counter, 1), 2.0)
                        + Math.pow(a.getElementAt(counter, 2), 2.0)
                        + Math.pow(a.getElementAt(counter, 9), 2.0)
                        + Math.pow(a.getElementAt(counter, 10), 2.0)
                        + Math.pow(a.getElementAt(counter, 11), 2.0));

                a.setElementAt(counter, 0, a.getElementAt(counter, 0) / rowNorm);
                a.setElementAt(counter, 1, a.getElementAt(counter, 1) / rowNorm);
                a.setElementAt(counter, 2, a.getElementAt(counter, 2) / rowNorm);
                a.setElementAt(counter, 9, a.getElementAt(counter, 9) / rowNorm);
                a.setElementAt(counter, 10, a.getElementAt(counter, 10) / rowNorm);
                a.setElementAt(counter, 11, a.getElementAt(counter, 11) / rowNorm);
                counter++;

                // second row

                // columns 0, 1, 2 are left with zero values

                a.setElementAt(counter, 3, -pD * la);
                a.setElementAt(counter, 4, -pD * lb);
                a.setElementAt(counter, 5, -pD * lc);

                // columns 6, 7, 8 are left with zero values

                a.setElementAt(counter, 9, pB * la);
                a.setElementAt(counter, 10, pB * lb);
                a.setElementAt(counter, 11, pB * lc);

                // normalize row
                rowNorm = Math.sqrt(Math.pow(a.getElementAt(counter, 3), 2.0)
                        + Math.pow(a.getElementAt(counter, 4), 2.0)
                        + Math.pow(a.getElementAt(counter, 5), 2.0)
                        + Math.pow(a.getElementAt(counter, 9), 2.0)
                        + Math.pow(a.getElementAt(counter, 10), 2.0)
                        + Math.pow(a.getElementAt(counter, 11), 2.0));

                a.setElementAt(counter, 3, a.getElementAt(counter, 3) / rowNorm);
                a.setElementAt(counter, 4, a.getElementAt(counter, 4) / rowNorm);
                a.setElementAt(counter, 5, a.getElementAt(counter, 5) / rowNorm);
                a.setElementAt(counter, 9, a.getElementAt(counter, 9) / rowNorm);
                a.setElementAt(counter, 10, a.getElementAt(counter, 10) / rowNorm);
                a.setElementAt(counter, 11, a.getElementAt(counter, 11) / rowNorm);
                counter++;


                // in case we want an exact solution (up to scale) when LMSE is
                // disabled, we stop after 11 equations
                if (!isLMSESolutionAllowed() && (counter >= MIN_NUMBER_OF_EQUATIONS)) {
                    break;
                }

                // third row

                // columns 0, 1, 2, 3, 4, 5 are left with zero values

                a.setElementAt(counter, 6, -pD * la);
                a.setElementAt(counter, 7, -pD * lb);
                a.setElementAt(counter, 8, -pD * lc);

                a.setElementAt(counter, 9, pC * la);
                a.setElementAt(counter, 10, pC * lb);
                a.setElementAt(counter, 11, pC * lc);

                // normalize row
                rowNorm = Math.sqrt(Math.pow(a.getElementAt(counter, 6), 2.0)
                        + Math.pow(a.getElementAt(counter, 7), 2.0)
                        + Math.pow(a.getElementAt(counter, 8), 2.0)
                        + Math.pow(a.getElementAt(counter, 9), 2.0)
                        + Math.pow(a.getElementAt(counter, 10), 2.0)
                        + Math.pow(a.getElementAt(counter, 11), 2.0));

                a.setElementAt(counter, 6, a.getElementAt(counter, 6) / rowNorm);
                a.setElementAt(counter, 7, a.getElementAt(counter, 7) / rowNorm);
                a.setElementAt(counter, 8, a.getElementAt(counter, 8) / rowNorm);
                a.setElementAt(counter, 9, a.getElementAt(counter, 9) / rowNorm);
                a.setElementAt(counter, 10, a.getElementAt(counter, 10) / rowNorm);
                a.setElementAt(counter, 11, a.getElementAt(counter, 11) / rowNorm);
                counter++;
            }

            final var decomposer = new SingularValueDecomposer(a);
            decomposer.decompose();

            if (decomposer.getNullity() > 1) {
                // line/plane configuration is degenerate and exists a linear
                // combination of possible pinhole cameras (i.e. solution is not
                // unique up to scale)
                throw new PinholeCameraEstimatorException();
            }

            final var v = decomposer.getV();

            // use last column of V as pinhole camera vector

            // the last column of V contains pinhole camera matrix ordered by
            // columns as: P11, P21, P31, P12, P22, P32, P13, P23, P33, P14, P24,
            // P34, hence we reorder p
            final var pinholeCameraMatrix = new Matrix(
                    PinholeCamera.PINHOLE_CAMERA_MATRIX_ROWS, PinholeCamera.PINHOLE_CAMERA_MATRIX_COLS);

            pinholeCameraMatrix.setElementAt(0, 0, v.getElementAt(0, 11));
            pinholeCameraMatrix.setElementAt(1, 0, v.getElementAt(1, 11));
            pinholeCameraMatrix.setElementAt(2, 0, v.getElementAt(2, 11));

            pinholeCameraMatrix.setElementAt(0, 1, v.getElementAt(3, 11));
            pinholeCameraMatrix.setElementAt(1, 1, v.getElementAt(4, 11));
            pinholeCameraMatrix.setElementAt(2, 1, v.getElementAt(5, 11));

            pinholeCameraMatrix.setElementAt(0, 2, v.getElementAt(6, 11));
            pinholeCameraMatrix.setElementAt(1, 2, v.getElementAt(7, 11));
            pinholeCameraMatrix.setElementAt(2, 2, v.getElementAt(8, 11));

            pinholeCameraMatrix.setElementAt(0, 3, v.getElementAt(9, 11));
            pinholeCameraMatrix.setElementAt(1, 3, v.getElementAt(10, 11));
            pinholeCameraMatrix.setElementAt(2, 3, v.getElementAt(11, 11));

            // because pinholeCameraMatrix has been obtained as the last column
            // of V, then its Frobenius norm will be 1 because SVD already
            // returns normalized singular vector

            final var camera = new PinholeCamera(pinholeCameraMatrix);

            if (listener != null) {
                listener.onEstimateEnd(this);
            }

            return attemptRefine(camera);

        } catch (final PinholeCameraEstimatorException e) {
            throw e;
        } catch (final Exception e) {
            throw new PinholeCameraEstimatorException(e);
        } finally {
            locked = false;
        }
    }

    /**
     * Returns type of pinhole camera estimator.
     *
     * @return type of pinhole camera estimator.
     */
    @Override
    public PinholeCameraEstimatorType getType() {
        return PinholeCameraEstimatorType.DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR;
    }
}
