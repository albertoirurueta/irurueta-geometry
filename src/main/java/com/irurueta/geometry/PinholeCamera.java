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
import com.irurueta.algebra.ArrayUtils;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.RQDecomposer;
import com.irurueta.algebra.SingularValueDecomposer;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.estimators.DLTLinePlaneCorrespondencePinholeCameraEstimator;
import com.irurueta.geometry.estimators.DLTPointCorrespondencePinholeCameraEstimator;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;

/**
 * This class implements the behavior of a pinhole camera.
 * A pinhole camera is a linear mapping between 3D and 2D worlds.
 * Pinhole cameras only take into account translation, rotation and camera
 * intrinsic parameters such as focal length, aspect ratio, skewness and
 * principal point.
 * Pinhole cameras perform projective mappings between 3D and 2D worlds,
 * in other words, the farther an object is, the smaller is represented or
 * parallel lines converge into vanishing points.
 * Pinhole cameras cannot be used for orthographic projections (where
 * parallelism between lines is preserved and there are no vanishing points).
 */
@SuppressWarnings("DuplicatedCode")
public class PinholeCamera extends Camera implements Serializable {

    /**
     * Defines the number of rows of a pinhole camera.
     */
    public static final int PINHOLE_CAMERA_MATRIX_ROWS = 3;

    /**
     * Defines the number of columns of a pinhole camera.
     */
    public static final int PINHOLE_CAMERA_MATRIX_COLS = 4;

    /**
     * Constant defining the number of inhomogeneous coordinates.
     */
    public static final int INHOM_COORDS = 3;

    /**
     * Constant defining a tiny value close to machine precision.
     */
    public static final double EPS = 1e-12;

    /**
     * Indicates if camera should be decomposed into intrinsic parameters and
     * rotation by default after creation or setting new parameters.
     */
    private static final boolean DEFAULT_DECOMPOSE_INTRINSICS_AND_ROTATION =
            true;

    /**
     * Indicates if camera should be decomposed to obtain its center after
     * creation or setting new parameters.
     */
    private static final boolean DEFAULT_DECOMPOSE_CAMERA_CENTER = true;

    /**
     * Threshold to determine whether a point is in front or behind the camera.
     */
    private static final double FRONT_THRESHOLD = 0.0;

    /**
     * Threshold to determine camera sign. If camera sign is negative, its sign
     * must be fixed (multiplying its matrix by -1) so that point cheirality
     * to determine whether points are in front or behind the camera can be
     * correctly determined. When sign is reversed, cheirality gets reversed
     * too. Notice that camera matrix is expressed in homogeneous coordinates,
     * hence multiplying it by -1.0 has no effect on point projection.
     */
    private static final double SIGN_THRESHOLD = 0.0;

    /**
     * Internal matrix defining this camera.
     */
    private Matrix mInternalMatrix;

    /**
     * Boolean indicating whether this camera has already been normalized.
     * Normalization can help to increase numerical accuracy on camera
     * computations.
     */
    private boolean mNormalized;

    /**
     * Boolean indicating whether camera sign has been fixed (it is 1.0).
     * When camera sign is negative, cheirality is reversed, hence it cannot be
     * correctly determined whether points are located in front or behind the
     * camera.
     */
    private boolean mCameraSignFixed;

    /**
     * Intrinsic parameters of the camera after decomposition.
     */
    private PinholeCameraIntrinsicParameters mIntrinsicParameters;

    /**
     * 3D rotation of the camera after decomposition.
     */
    private Rotation3D mCameraRotation;

    /**
     * Camera center after decomposition.
     */
    private Point3D mCameraCenter;

    /**
     * Constructor.
     * Creates a canonical camera, which is equal to the identity 3x4 matrix.
     */
    public PinholeCamera() {
        super();
        mNormalized = false;
        mCameraSignFixed = false;

        mIntrinsicParameters = null;
        mCameraRotation = null;
        mCameraCenter = null;

        try {
            mInternalMatrix = Matrix.identity(PINHOLE_CAMERA_MATRIX_ROWS,
                    PINHOLE_CAMERA_MATRIX_COLS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     * Creates a camera using provided matrix.
     *
     * @param internalMatrix matrix to create the camera from.
     * @throws WrongSizeException If provided matrix is not 3x4.
     */
    public PinholeCamera(final Matrix internalMatrix) throws WrongSizeException {
        super();
        mNormalized = false;
        mCameraSignFixed = false;

        mIntrinsicParameters = null;
        mCameraRotation = null;
        mCameraCenter = null;

        setInternalMatrix(internalMatrix);
    }

    /**
     * Constructor.
     * Creates a camera using provided intrinsic parameters, 3D rotation and
     * 2D coordinates of the world origin.
     *
     * @param intrinsicParameters    Intrinsic parameters of the camera.
     * @param rotation               3D rotation of the camera.
     * @param originImageCoordinates 2D coordinates of the world origin.
     */
    public PinholeCamera(final PinholeCameraIntrinsicParameters intrinsicParameters,
                         final Rotation3D rotation, final Point2D originImageCoordinates) {
        super();
        mNormalized = false;
        mCameraSignFixed = false;

        mIntrinsicParameters = null;
        mCameraRotation = null;
        mCameraCenter = null;
        try {
            mInternalMatrix = Matrix.identity(PINHOLE_CAMERA_MATRIX_ROWS,
                    PINHOLE_CAMERA_MATRIX_COLS);
        } catch (final WrongSizeException ignore) {
            // this will never happen
        }

        setIntrinsicAndExtrinsicParameters(intrinsicParameters, rotation,
                originImageCoordinates);
    }

    /**
     * Constructor.
     * Creates a camera using provided intrinsic parameters, 3D rotation and
     * 3D coordinates of the camera center.
     *
     * @param intrinsicParameters Intrinsic parameters of the camera.
     * @param rotation            3D rotation of the camera.
     * @param cameraCenter        3D coordinates of the camera center.
     */
    public PinholeCamera(final PinholeCameraIntrinsicParameters intrinsicParameters,
                         final Rotation3D rotation, final Point3D cameraCenter) {
        super();
        mNormalized = false;
        mCameraSignFixed = false;

        mIntrinsicParameters = null;
        mCameraRotation = null;
        mCameraCenter = null;
        try {
            mInternalMatrix = Matrix.identity(PINHOLE_CAMERA_MATRIX_ROWS,
                    PINHOLE_CAMERA_MATRIX_COLS);
        } catch (final WrongSizeException ignore) {
            // this will never happen
        }

        setIntrinsicAndExtrinsicParameters(intrinsicParameters, rotation,
                cameraCenter);
    }

    /**
     * Creates an instance of a pinhole camera by estimating its parameters from
     * 2D-3D point correspondences.
     *
     * @param point3D1 1st 3D point.
     * @param point3D2 2nd 3D point.
     * @param point3D3 3rd 3D point.
     * @param point3D4 4th 3D point.
     * @param point3D5 5th 3D point.
     * @param point3D6 6th 3D point.
     * @param point2D1 1st 2D point corresponding to the projection of 1st 3D
     *                 point.
     * @param point2D2 2nd 2D point corresponding to the projection of 2nd 3D
     *                 point.
     * @param point2D3 3rd 2D point corresponding to the projection of 3rd 3D
     *                 point.
     * @param point2D4 4th 2D point corresponding to the projection of 4th 3D
     *                 point.
     * @param point2D5 5th 2D point corresponding to the projection of 5th 3D
     *                 point.
     * @param point2D6 6th 2D point corresponding to the projection of 6th 3D
     *                 point.
     * @throws CameraException if camera cannot be estimated using provided
     *                         points because of a degeneracy.
     */
    public PinholeCamera(
            final Point3D point3D1, final Point3D point3D2, final Point3D point3D3,
            final Point3D point3D4, final Point3D point3D5, final Point3D point3D6,
            final Point2D point2D1, final Point2D point2D2, final Point2D point2D3,
            final Point2D point2D4, final Point2D point2D5, final Point2D point2D6) throws CameraException {
        super();
        mNormalized = false;
        mCameraSignFixed = false;

        mIntrinsicParameters = null;
        mCameraRotation = null;
        mCameraCenter = null;

        try {
            mInternalMatrix = Matrix.identity(PINHOLE_CAMERA_MATRIX_ROWS,
                    PINHOLE_CAMERA_MATRIX_COLS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        setFromPointCorrespondences(point3D1, point3D2, point3D3, point3D4,
                point3D5, point3D6, point2D1, point2D2, point2D3, point2D4,
                point2D5, point2D6);
    }

    /**
     * Creates an instance of a pinhole camera by estimating its parameters from
     * line/plane correspondences.
     *
     * @param plane1 1st 3D plane.
     * @param plane2 2nd 3D plane.
     * @param plane3 3rd 3D plane.
     * @param plane4 4th 3D plane.
     * @param line1  1st 2D line corresponding to 1st 3D plane.
     * @param line2  2nd 2D line corresponding to 2nd 3D plane.
     * @param line3  3rd 2D line corresponding to 3rd 3D plane.
     * @param line4  4th 2D line corresponding to 4th 3D plane.
     * @throws CameraException if camera cannot be estimated using provided
     *                         lines and planes because of a degeneracy.
     */
    public PinholeCamera(
            final Plane plane1, final Plane plane2, final Plane plane3, final Plane plane4,
            final Line2D line1, final Line2D line2, final Line2D line3, final Line2D line4)
            throws CameraException {
        super();
        mNormalized = false;
        mCameraSignFixed = false;

        mIntrinsicParameters = null;
        mCameraRotation = null;
        mCameraCenter = null;

        try {
            mInternalMatrix = Matrix.identity(PINHOLE_CAMERA_MATRIX_ROWS,
                    PINHOLE_CAMERA_MATRIX_COLS);
        } catch (final WrongSizeException ignore) {
            // never happens
        }

        setFromLineAndPlaneCorrespondences(plane1, plane2, plane3, plane4,
                line1, line2, line3, line4);
    }


    /**
     * Projects a 3D point into a 2D point in a retinal plane.
     *
     * @param inputPoint 3D point to be projected.
     * @param result     2D projected point.
     */
    @Override
    public void project(final Point3D inputPoint, final Point2D result) {
        // convert input point to homogeneous and normalize to increase accuracy
        final HomogeneousPoint3D point3D = new HomogeneousPoint3D(inputPoint);
        point3D.normalize();
        // normalize this camera to increase accuracy
        normalize();

        try {
            // copy point3D coordinates in a matrix
            final Matrix m3D = new Matrix(
                    Point3D.POINT3D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            m3D.setElementAtIndex(0, point3D.getHomX());
            m3D.setElementAtIndex(1, point3D.getHomY());
            m3D.setElementAtIndex(2, point3D.getHomZ());
            m3D.setElementAtIndex(3, point3D.getHomW());

            // make product of 3D point column matrix with pinhole camera
            // internal matrix
            final Matrix m = mInternalMatrix.multiplyAndReturnNew(m3D);

            result.setHomogeneousCoordinates(m.getElementAtIndex(0),
                    m.getElementAtIndex(1), m.getElementAtIndex(2));
            // to increase accuracy
            result.normalize();
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Back-projects a line into a plane and stores the result into provided
     * instance.
     *
     * @param line   2D line to be back-projected.
     * @param result Instance where computed back-projected 3D plane data is
     *               stored.
     */
    @Override
    public void backProject(final Line2D line, final Plane result) {

        // normalize input line and camera to increase accuracy
        line.normalize();
        normalize();

        try {
            final Matrix l = new Matrix(Line2D.LINE_NUMBER_PARAMS, 1);
            l.setElementAtIndex(0, line.getA());
            l.setElementAtIndex(1, line.getB());
            l.setElementAtIndex(2, line.getC());

            // Compute transposed of pinhole camera matrix
            final Matrix m = mInternalMatrix.transposeAndReturnNew();
            // PLANE = P^T * l
            m.multiply(l);

            // set coordinates on plane
            result.setParameters(m.getElementAtIndex(0),
                    m.getElementAtIndex(1), m.getElementAtIndex(2),
                    m.getElementAtIndex(3));
            // to increase accuracy
            result.normalize();
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Back-projects provided 2D point into a 3D point and stores the result into
     * provided instance.
     * Notice that estimated solution is not unique, since back-projecting a 2D
     * point results in an infinite number of 3D points located in the same
     * ray of light.
     * This method only computes one possible solution. Any other solution can
     * be computed as a linear combination between the camera center and the
     * estimated back-projeted point.
     *
     * @param point  2D point to be back-projected.
     * @param result Instance where back-projected 3D point data will be stored.
     * @throws CameraException thrown if 2D point cannot be back-projected
     *                         because camera is degenerate.
     */
    @Override
    public void backProject(final Point2D point, final Point3D result)
            throws CameraException {
        // convert to homogeneous and normalize to increase accuracy
        final Point2D p = new HomogeneousPoint2D(point);
        p.normalize();
        // normalize camera to increase accuracy
        normalize();

        try {
            final Matrix m = new Matrix(
                    Point2D.POINT2D_HOMOGENEOUS_COORDINATES_LENGTH, 1);
            m.setElementAtIndex(0, p.getHomX());
            m.setElementAtIndex(1, p.getHomY());
            m.setElementAtIndex(2, p.getHomW());

            // Compute pseudo-inverse of internal matrix
            final Matrix pseudoInverseInternalMatrix = Utils.pseudoInverse(
                    mInternalMatrix);

            // normalize pseudo-inverse
            final double norm = Utils.normF(pseudoInverseInternalMatrix);
            pseudoInverseInternalMatrix.multiplyByScalar(1.0 / norm);

            // back-projected point (ray of light) is the product of pseudo-
            // inverse of internal matrix with image point matrix
            pseudoInverseInternalMatrix.multiply(m);

            result.setHomogeneousCoordinates(
                    pseudoInverseInternalMatrix.getElementAtIndex(0),
                    pseudoInverseInternalMatrix.getElementAtIndex(1),
                    pseudoInverseInternalMatrix.getElementAtIndex(2),
                    pseudoInverseInternalMatrix.getElementAtIndex(3));
        } catch (final AlgebraException e) {
            throw new CameraException(e);
        }
    }

    /**
     * Back-projects a 2D conic into a 3D quadric and stores the result into
     * provided instance.
     *
     * @param conic  2D conic to be back-rojected.
     * @param result Instance where data of back-projected 3D quadric will be
     *               stored.
     */
    @Override
    public void backProject(final Conic conic, final Quadric result) {
        // We need to compute:
        // Q = P^T * C * P

        // normalize conic and camera to increase accuracy
        conic.normalize();
        normalize();

        // get transposed internal matrix
        final Matrix m = mInternalMatrix.transposeAndReturnNew();
        try {
            // multiply by conic
            m.multiply(conic.asMatrix());
            // and then by internal matrix
            m.multiply(mInternalMatrix);
            // resulting matrix m is the internal matrix of a quadric
            result.setParameters(m);
        } catch (final WrongSizeException | NonSymmetricMatrixException ignore) {
            // never happens
        }
    }

    /**
     * Projects a 3D dual quadric into a 2D dual conic and stores the result
     * into provided instance.
     *
     * @param dualQuadric 3D dual quadric to be projected.
     * @param result      Instance where data of projected 2D dual conic will be
     *                    stored.
     */
    @Override
    public void project(final DualQuadric dualQuadric, final DualConic result) {
        // We need to compute:
        // C^-1 = P * Q^-1 * P^T
        dualQuadric.normalize();
        normalize();

        try {
            Matrix m = mInternalMatrix.multiplyAndReturnNew(
                    dualQuadric.asMatrix());
            m.multiply(mInternalMatrix.transposeAndReturnNew());
            // resulting matrix m is the internal matrix of a dual conic
            result.setParameters(m);
        } catch (final WrongSizeException | NonSymmetricMatrixException ignore) {
            //never happens
        }
    }

    /**
     * Returns the type of this camera, which is always PINHOLE_CAMERA for
     * instance of this class.
     *
     * @return Type of this camera.
     */
    @Override
    public CameraType getType() {
        return CameraType.PINHOLE_CAMERA;
    }

    /**
     * Decomposes current camera matrix to determine its intrinsic and extrinsic
     * parameters (rotation and translation).
     *
     * @throws CameraException thrown if camera matrix is degenerate.
     */
    public void decompose() throws CameraException {
        decompose(DEFAULT_DECOMPOSE_INTRINSICS_AND_ROTATION);
    }

    /**
     * Decomposes current camera matrix to determine its camera center.
     * Intrinsic parameters and rotation will be decomposed as well depending on
     * provided value.
     *
     * @param decomposeIntrinsicsAndRotation if true, intrinsic parameters and
     *                                       rotation are computed as well.
     * @throws CameraException thrown if camera matrix is degenerate.
     */
    public void decompose(final boolean decomposeIntrinsicsAndRotation)
            throws CameraException {
        decompose(decomposeIntrinsicsAndRotation,
                DEFAULT_DECOMPOSE_CAMERA_CENTER);
    }

    /**
     * Decomposes current camera matrix.
     * Intrinsic parameters, rotation and camera center will be computed
     * depending on provided values.
     *
     * @param decomposeIntrinsicsAndRotation if true, intrinsic parameters and
     *                                       rotation are computed as well.
     * @param decomposeCameraCenter          if true, camera center is computed as well.
     * @throws CameraException thrown if camera matrix is degenerate.
     */
    public void decompose(final boolean decomposeIntrinsicsAndRotation,
                          final boolean decomposeCameraCenter) throws CameraException {
        // clean up previous intrinsics, rotation and camera center
        mIntrinsicParameters = null;
        mCameraRotation = null;
        mCameraCenter = null;
        if (decomposeIntrinsicsAndRotation) {
            // compute new intrinsics and rotation
            computeIntrinsicsAndRotation();
        }
        if (decomposeCameraCenter) {
            // compute camera center
            if (mCameraCenter == null) {
                mCameraCenter = Point3D.create(
                        CoordinatesType.HOMOGENEOUS_COORDINATES);
            }
            computeCameraCenterSVD(mCameraCenter);
        }
    }

    /**
     * Normalizes camera matrix.
     * Normalization can help to increase accuracy on camera operations.
     * This method should only called when an increase on accuracy is needed
     * to save the additional computational cost.
     * Notice that affine pinhole cameras are never normalized, since elements
     * to be used for normalization have norm equal to zero in such case.
     */
    public void normalize() {
        if (!mNormalized) {
            // normalize camera matrix using norm of P31, P32 and P33, which is
            // equal to a row of rotation, which in a canonical sense should have
            // unitary norm
            final double p20 = mInternalMatrix.getElementAt(2, 0);
            final double p21 = mInternalMatrix.getElementAt(2, 1);
            final double p22 = mInternalMatrix.getElementAt(2, 2);
            double norm = Math.sqrt(p20 * p20 + p21 * p21 + p22 * p22);
            if (Math.abs(norm) <= EPS) {
                // camera is affine, so we use whole matrix norm
                norm = Utils.normF(mInternalMatrix);
            }
            mInternalMatrix.multiplyByScalar(1.0 / norm);
            mNormalized = true;
        }
    }

    /**
     * Indicates if camera matrix has already been normalized.
     * Notice that this value will be set to false when any camera parameter is
     * modified
     *
     * @return true if camera is normalized, false otherwise
     */
    public boolean isNormalized() {
        return mNormalized;
    }

    /**
     * Fixes the camera sign so that point cheirality can be correctly
     * determined. Cheirality indicates if points are located in front or behind
     * the camera.
     * Sign needs to be fixed if it is negative.
     *
     * @throws CameraException thrown if there are numerical instabilities.
     */
    public void fixCameraSign() throws CameraException {
        if (isCameraSignFixed()) {
            return;
        }

        // use sign of determinant to fix pinhole camera matrix sign
        final double cameraSign = getCameraSign();

        mInternalMatrix.multiplyByScalar(cameraSign);
        mCameraSignFixed = true;
    }

    /**
     * Indicates if camera sign has been fixed.
     * When camera sign has been fixed cheirality can be correctly determined.
     * Cheirality indicates if points are located in front or behind the camera.
     * Notice that when camera parameters are modified, this parameter is set to
     * false again.
     *
     * @return true if camera sign has been fixed, false otherwise.
     */
    public boolean isCameraSignFixed() {
        return mCameraSignFixed;
    }

    /**
     * Returns camera rotation if camera has already been decomposed and
     * rotation is available.
     *
     * @return camera rotation.
     * @throws NotAvailableException if camera rotation is not available yet.
     */
    public Rotation3D getCameraRotation() throws NotAvailableException {
        if (!isCameraRotationAvailable()) {
            throw new NotAvailableException();
        }

        return mCameraRotation;
    }

    /**
     * Returns camera intrinsic parameters if camera has already been decomposed
     * and intrinsic parameters are available.
     * Intrinsic parameters contain information related to internal parameters
     * of a camera, usually related to the camera lens and sensor.
     *
     * @return camera intrinsic parameters.
     * @throws NotAvailableException if camera intrinsic parameters are not
     *                               available yet.
     */
    public PinholeCameraIntrinsicParameters getIntrinsicParameters()
            throws NotAvailableException {
        if (!areIntrinsicParametersAvailable()) {
            throw new NotAvailableException();
        }

        return mIntrinsicParameters;
    }

    /**
     * Returns a copy of internal matrix to avoid malicious modifications.
     *
     * @return a copy of the internal camera matrix.
     */
    public Matrix getInternalMatrix() {
        return new Matrix(mInternalMatrix);
    }

    /**
     * Sets internal matrix of this camera. Internal matrix of a pinhole camera
     * must have size 3x4 (i.e. 3 rows and 4 columns).
     *
     * @param internalMatrix internal matrix to be set
     * @throws WrongSizeException if provided matrix doesn't have size 3x4.
     */
    public final void setInternalMatrix(final Matrix internalMatrix)
            throws WrongSizeException {
        if (internalMatrix.getRows() != PINHOLE_CAMERA_MATRIX_ROWS ||
                internalMatrix.getColumns() != PINHOLE_CAMERA_MATRIX_COLS) {
            throw new WrongSizeException();
        }

        mInternalMatrix = internalMatrix;
        mCameraSignFixed = false;
        mIntrinsicParameters = null;
        mCameraRotation = null;
        mCameraCenter = null;
        mNormalized = false;
    }

    /**
     * Indicates if camera intrinsic parameters are available for retrieval.
     * Intrinsic parameters become available after camera decomposition (if
     * intrinsic parameters are requested). And become unavailable when any
     * camera parameters are modified.
     *
     * @return true if camera intrinsic parameters are available, false
     * otherwise.
     */
    public boolean areIntrinsicParametersAvailable() {
        return mIntrinsicParameters != null;
    }

    /**
     * Indicates if camera rotation is available for retrieval.
     * Camera rotation become available after camera decomposition (if camera
     * rotation is requested). And become unavailable when any camera parameters
     * are modified.
     *
     * @return true if camera rotation is available, false otherwise.
     */
    public boolean isCameraRotationAvailable() {
        return mCameraRotation != null;
    }

    /**
     * Sets camera rotation of this camera.
     * When setting rotation the camera sign becomes unknown and the camera
     * becomes non-normalized.
     *
     * @param cameraRotation Camera 3D rotation to be set.
     * @throws CameraException if there are numerical instabilities.
     */
    public void setCameraRotation(final Rotation3D cameraRotation)
            throws CameraException {
        try {
            if (!areIntrinsicParametersAvailable()) {
                // Find intrinsic parameters
                computeIntrinsicsAndRotation();
            }

            final Matrix k = mIntrinsicParameters.getInternalMatrix();
            final Matrix r = cameraRotation.asInhomogeneousMatrix();

            // compute new left 3x3 sub-matrix of pinhole camera matrix
            final Matrix mp = k.multiplyAndReturnNew(r);

            // set new rotation
            mCameraRotation = cameraRotation;

            // set new rotation on left 3x3 sub-matrix of internal pinhole camera
            // matrix
            mInternalMatrix.setSubmatrix(0, 0, PINHOLE_CAMERA_MATRIX_ROWS - 1,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1, mp);

            // reset camera sign fixed
            mCameraSignFixed = false;
            mNormalized = false;
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Combines current camera rotation with provided rotation.
     *
     * @param cameraRotation Camera 3D rotation to be added to current rotation.
     * @throws CameraException if there are numerical instabilities.
     */
    public void rotate(final Rotation3D cameraRotation) throws CameraException {
        if (!isCameraRotationAvailable()) {
            // compute new intrinsics and rotation
            computeIntrinsicsAndRotation();
        }

        // compute new rotation matrix and set it
        mCameraRotation.combine(cameraRotation);
        // set new camera rotation
        setCameraRotation(mCameraRotation);
    }

    /**
     * Modifies camera so that it points to provided point while keeping the
     * camera center.
     *
     * @param point Point to look at.
     * @throws CameraException thrown if operation cannot be done. This happens
     *                         usually when point is located very close to the camera center. In those
     *                         situations orientation cannot be reliably computed.
     */
    public void pointAt(final Point3D point) throws CameraException {
        try {
            if (!isCameraCenterAvailable()) {
                // compute camera center
                if (mCameraCenter == null) {
                    mCameraCenter = Point3D.create(
                            CoordinatesType.HOMOGENEOUS_COORDINATES);
                }
                computeCameraCenterSVD(mCameraCenter);
            }
            if (!areIntrinsicParametersAvailable()) {
                // Find intrinsics
                computeIntrinsicsAndRotation();
            }

            // difference vector between 3D point and camera center using
            // inhomogeneous coordinates. This vector after normalization should
            // be the new principal vector
            final Matrix mDiff = new Matrix(1, PINHOLE_CAMERA_MATRIX_ROWS);
            mDiff.setElementAtIndex(0,
                    point.getInhomX() - mCameraCenter.getInhomX());
            mDiff.setElementAtIndex(1,
                    point.getInhomY() - mCameraCenter.getInhomY());
            mDiff.setElementAtIndex(2,
                    point.getInhomZ() - mCameraCenter.getInhomZ());

            final double norm = Utils.normF(mDiff);
            mDiff.multiplyByScalar(1.0 / norm);

            // mDiff has to be the lower row of rotation matrix, we need to find
            // the remaining rows as two orthonormal vectors respect to this one,
            // hence we use SVD of mDiff
            final SingularValueDecomposer decomposer =
                    new SingularValueDecomposer(mDiff);
            decomposer.decompose();

            final Matrix v = decomposer.getV();

            // set new rotation matrix by setting mDiff (which is 1st column of
            // V) on the last row of rotation matrix
            final Matrix r = new Matrix(PINHOLE_CAMERA_MATRIX_ROWS,
                    PINHOLE_CAMERA_MATRIX_ROWS);
            r.setElementAt(2, 0, v.getElementAt(0, 0));
            r.setElementAt(2, 1, v.getElementAt(1, 0));
            r.setElementAt(2, 2, v.getElementAt(2, 0));

            r.setElementAt(1, 0, v.getElementAt(0, 1));
            r.setElementAt(1, 1, v.getElementAt(1, 1));
            r.setElementAt(1, 2, v.getElementAt(2, 1));

            r.setElementAt(0, 0, v.getElementAt(0, 2));
            r.setElementAt(0, 1, v.getElementAt(1, 2));
            r.setElementAt(0, 2, v.getElementAt(2, 2));

            // set new rotation
            final Rotation3D rotation = new MatrixRotation3D(r);
            setCameraRotation(rotation);

            // because left 3x3 sub-matrix has been modified, then last column of
            // internal matrix has to be modified to ensure that camera center
            // does not change
            final Matrix mp = mInternalMatrix.getSubmatrix(0, 0, 2, 2);
            final Matrix center = new Matrix(PINHOLE_CAMERA_MATRIX_ROWS, 1);
            center.setElementAtIndex(0, mCameraCenter.getInhomX());
            center.setElementAtIndex(1, mCameraCenter.getInhomY());
            center.setElementAtIndex(2, mCameraCenter.getInhomZ());

            // MP will be p4 (last column)
            mp.multiply(center);
            mp.multiplyByScalar(-1.0);

            // set last column of pinhole camera matrix
            mInternalMatrix.setSubmatrix(0, PINHOLE_CAMERA_MATRIX_COLS - 1,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1,
                    PINHOLE_CAMERA_MATRIX_COLS - 1, mp);

            // because we do not check sign of new director vector, we reset
            // camera sign so that it is checked afterwards when needed
            mCameraSignFixed = false;
        } catch (final WrongSizeException ignore) {
            // never happens
        } catch (final AlgebraException | InvalidRotationMatrixException e) {
            throw new CameraException(e);
        }
    }

    /**
     * Sets camera intrinsic parameters.
     * Intrinsic parameters are related to camera lens and sensor and contain
     * parameters such as focal length, skewness or principal point
     *
     * @param intrinsicParameters intrinsic parameters to be set
     * @throws CameraException if there are numerical instabilities
     */
    public void setIntrinsicParameters(
            final PinholeCameraIntrinsicParameters intrinsicParameters)
            throws CameraException {

        if (!isCameraRotationAvailable()) {
            // Find rotation
            computeIntrinsicsAndRotation();
        }

        // get K and R matrices
        final Matrix k = intrinsicParameters.getInternalMatrix();
        final Matrix r = mCameraRotation.asInhomogeneousMatrix();

        // compute now left 3x3 sub-matrix of pinhole camera matrix
        try {
            k.multiply(r);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
        mIntrinsicParameters = intrinsicParameters;

        // set new intrinsic parameters on left 3x3 sub-matrix of internal
        // pinhole camera matrix
        mInternalMatrix.setSubmatrix(0, 0, PINHOLE_CAMERA_MATRIX_ROWS - 1,
                PINHOLE_CAMERA_MATRIX_ROWS - 1, k);
        mCameraSignFixed = false;
        mNormalized = false;
    }

    /**
     * Returns a 3D point indicating camera center (i.e. location) if center
     * has already been computed and is available for retrieval.
     * If camera center is not available, camera must be decomposed before
     * calling this method.
     *
     * @return camera center.
     * @throws NotAvailableException if camera center is not yet available for
     *                               retrieval.
     */
    public Point3D getCameraCenter() throws NotAvailableException {
        if (!isCameraCenterAvailable()) {
            throw new NotAvailableException();
        }

        return mCameraCenter;
    }

    /**
     * Indicates if camera center has been decomposed and is available for
     * retrieval.
     *
     * @return true if camera center is available, false otherwise.
     */
    public boolean isCameraCenterAvailable() {
        return mCameraCenter != null;
    }

    /**
     * Sets 3D coordinates of camera center.
     * When setting camera center camera becomes not normalized.
     *
     * @param cameraCenter camera center to be set.
     */
    public void setCameraCenter(final Point3D cameraCenter) {
        try {
            normalize();
            // new last column is the product of left 3x3 pinhole camera
            // sub-matrix by the inhomogeneous coordinates of new camera center
            final Matrix mp = mInternalMatrix.getSubmatrix(0, 0, 2, 2);
            final Matrix center = new Matrix(PINHOLE_CAMERA_MATRIX_ROWS, 1);
            center.setElementAtIndex(0, cameraCenter.getInhomX());
            center.setElementAtIndex(1, cameraCenter.getInhomY());
            center.setElementAtIndex(2, cameraCenter.getInhomZ());

            // Mp will be p4 (4th column)
            mp.multiply(center);
            mp.multiplyByScalar(-1.0);

            // set camera center
            mCameraCenter = cameraCenter;

            // set last column of pinhole camera matrix
            mInternalMatrix.setSubmatrix(0, PINHOLE_CAMERA_MATRIX_COLS - 1,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1,
                    PINHOLE_CAMERA_MATRIX_COLS - 1, mp);
            mNormalized = false;
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Sets camera intrinsic parameters and camera 3D rotation.
     * Intrinsic parameters indicate internal camera parameters related to
     * camera lens and camera sensor and rotation indicates camera orientation.
     *
     * @param intrinsicParameters intrinsic camera parameters to be set.
     * @param rotation            3D camera rotation to be set.
     */
    public void setIntrinsicParametersAndRotation(
            final PinholeCameraIntrinsicParameters intrinsicParameters,
            final Rotation3D rotation) {
        try {
            normalize();

            // K and R are obtained
            final Matrix k = intrinsicParameters.getInternalMatrix();
            final Matrix r = rotation.asInhomogeneousMatrix();

            // compute new left 3x3 sub-matrix of pinhole camera matrix (also known as Mp)
            k.multiply(r);

            // set new intrinsic parameters
            mIntrinsicParameters = intrinsicParameters;

            // set new rotation
            mCameraRotation = rotation;

            // set new left 3x3 sub-matrix of internal pinhole camera matrix
            mInternalMatrix.setSubmatrix(0, 0, PINHOLE_CAMERA_MATRIX_ROWS - 1,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1, k);
            mCameraSignFixed = false;
            mNormalized = false;
        } catch (final WrongSizeException ignore) {
            // this will never happen
        }
    }

    /**
     * Sets both intrinsic and extrinsic camera parameters.
     * Intrinsic parameters indicate internal camera parameters related to
     * camera lens and sensor, and extrinsic parameters are parameters that
     * indicate camera location and orientation by providing the projected
     * coordinates of world origin and the camera 3D rotation.
     *
     * @param intrinsicParameters    intrinsic parameters to be set.
     * @param rotation               camera rotation to be set.
     * @param originImageCoordinates projected coordinates of world origin to be
     *                               set.
     */
    public final void setIntrinsicAndExtrinsicParameters(
            final PinholeCameraIntrinsicParameters intrinsicParameters,
            final Rotation3D rotation, final Point2D originImageCoordinates) {
        setIntrinsicParametersAndRotation(intrinsicParameters, rotation);
        setImageOfWorldOrigin(originImageCoordinates);
    }

    /**
     * Sets both intrinsic and extrinsic camera parameters.
     * Intrinsic parameters indicate internal camera parameters related to
     * camera lens and sensor, and extrinsic parameters are parameters that
     * indicate camera location and orientation by providing camera center
     * and the camera 3D rotation.
     *
     * @param intrinsicParameters intrinsic parameters to be set.
     * @param rotation            camera rotation to be set.
     * @param cameraCenter        location of camera center to be set.
     */
    public final void setIntrinsicAndExtrinsicParameters(
            final PinholeCameraIntrinsicParameters intrinsicParameters,
            final Rotation3D rotation, final Point3D cameraCenter) {
        setIntrinsicParametersAndRotation(intrinsicParameters, rotation);
        setCameraCenter(cameraCenter);
    }

    /**
     * Returns the projected 2D coordinates of the x axis, which corresponds to
     * its vanishing point.
     *
     * @return vanishing point of x axis.
     */
    public Point2D getXAxisVanishingPoint() {
        final Point2D result = Point2D.create();
        xAxisVanishingPoint(result);
        return result;
    }

    /**
     * Computes the projected 2D coordinates of the x axis, which corresponds to
     * its vanishing point.
     *
     * @param result 2D point where vanishing point of x axis will be stored.
     */
    public void xAxisVanishingPoint(final Point2D result) {

        // use first camera matrix column to set 2D point
        result.setHomogeneousCoordinates(mInternalMatrix.getElementAt(0, 0),
                mInternalMatrix.getElementAt(1, 0),
                mInternalMatrix.getElementAt(2, 0));
    }

    /**
     * Returns the projected 2D coordinates of the y axis, which corresponds to
     * its vanishing point.
     *
     * @return vanishing point of y axis.
     */
    public Point2D getYAxisVanishingPoint() {
        final Point2D result = Point2D.create();
        yAxisVanishingPoint(result);
        return result;
    }

    /**
     * Computes the projected 2D coordinates of the y axis, which corresponds to
     * its vanishing point.
     *
     * @param result 2D point where vanishing point of y axis will be stored.
     */
    public void yAxisVanishingPoint(final Point2D result) {

        // use second camera matrix column to set 2D point
        result.setHomogeneousCoordinates(mInternalMatrix.getElementAt(0, 1),
                mInternalMatrix.getElementAt(1, 1),
                mInternalMatrix.getElementAt(2, 1));
    }

    /**
     * Returns the projected 2D coordinates of the z axis, which corresponds to
     * its vanishing point.
     *
     * @return vanishing point of z axis.
     */
    public Point2D getZAxisVanishingPoint() {
        final Point2D result = Point2D.create();
        zAxisVanishingPoint(result);
        return result;
    }

    /**
     * Computes the projected 2D coordinates of the z axis, which corresponds to
     * its vanishing point.
     *
     * @param result 2D point where vanishing point of z axis will be stored.
     */
    public void zAxisVanishingPoint(final Point2D result) {

        // use third camera matrix column to set 2D point
        result.setHomogeneousCoordinates(mInternalMatrix.getElementAt(0, 2),
                mInternalMatrix.getElementAt(1, 2),
                mInternalMatrix.getElementAt(2, 2));
    }

    /**
     * Returns the projected 2D coordinates of the world origin (0, 0, 0).
     *
     * @return projected point of world origin.
     */
    public Point2D getImageOfWorldOrigin() {
        final Point2D result = Point2D.create();
        imageOfWorldOrigin(result);
        return result;
    }

    /**
     * Computes the projected 2D coordinates of the world origin (0, 0, 0).
     *
     * @param result 2D point where projected point of world origin will be
     *               stored.
     */
    public void imageOfWorldOrigin(final Point2D result) {

        // use fourth camera matrix column to set 2D point
        result.setHomogeneousCoordinates(mInternalMatrix.getElementAt(0, 3),
                mInternalMatrix.getElementAt(1, 3),
                mInternalMatrix.getElementAt(2, 3));
    }

    /**
     * Sets the projected 2D coordinates of the x axis, which corresponds to its
     * vanishing point.
     *
     * @param xAxisVanishingPoint vanishing point of x axis to be set.
     */
    public void setXAxisVanishingPoint(final Point2D xAxisVanishingPoint) {
        // to increase accuracy
        xAxisVanishingPoint.normalize();

        final double homX = xAxisVanishingPoint.getHomX();
        final double homY = xAxisVanishingPoint.getHomY();
        final double homW = xAxisVanishingPoint.getHomW();

        mInternalMatrix.setElementAt(0, 0, homX);
        mInternalMatrix.setElementAt(1, 0, homY);
        mInternalMatrix.setElementAt(2, 0, homW);

        // set camera sign fixed and normalized
        mCameraSignFixed = false;
        mNormalized = false;
    }

    /**
     * Sets the projected 2D coordinates of the y axis, which corresponds to its
     * vanishing point.
     *
     * @param yAxisVanishingPoint vanishing point of y axis to be set.
     */
    public void setYAxisVanishingPoint(final Point2D yAxisVanishingPoint) {
        // to increase accuracy
        yAxisVanishingPoint.normalize();

        final double homX = yAxisVanishingPoint.getHomX();
        final double homY = yAxisVanishingPoint.getHomY();
        final double homW = yAxisVanishingPoint.getHomW();

        mInternalMatrix.setElementAt(0, 1, homX);
        mInternalMatrix.setElementAt(1, 1, homY);
        mInternalMatrix.setElementAt(2, 1, homW);

        //set camera sign fixed and normalized
        mCameraSignFixed = false;
        mNormalized = false;
    }

    /**
     * Sets the projected 2D coordinates of the z axis, which corresponds to its
     * vanishing point.
     *
     * @param zAxisVanishingPoint vanishing point of z axis to be set.
     */
    public void setZAxisVanishingPoint(final Point2D zAxisVanishingPoint) {
        // to increase accuracy
        zAxisVanishingPoint.normalize();

        final double homX = zAxisVanishingPoint.getHomX();
        final double homY = zAxisVanishingPoint.getHomY();
        final double homW = zAxisVanishingPoint.getHomW();

        mInternalMatrix.setElementAt(0, 2, homX);
        mInternalMatrix.setElementAt(1, 2, homY);
        mInternalMatrix.setElementAt(2, 2, homW);

        // set camera sign fixed and normalized
        mCameraSignFixed = false;
        mNormalized = false;
    }

    /**
     * Sets the projected 2D coordinates of the world origin (0, 0, 0).
     *
     * @param imageOfWorldOrigin projected world origin to be set.
     */
    public void setImageOfWorldOrigin(final Point2D imageOfWorldOrigin) {
        // to increase accuracy
        imageOfWorldOrigin.normalize();

        final double homX = imageOfWorldOrigin.getHomX();
        final double homY = imageOfWorldOrigin.getHomY();
        final double homW = imageOfWorldOrigin.getHomW();

        mInternalMatrix.setElementAt(0, 3, homX);
        mInternalMatrix.setElementAt(1, 3, homY);
        mInternalMatrix.setElementAt(2, 3, homW);

        // set normalized (no need to reset camera sign)
        mNormalized = false;
    }

    /**
     * Returns plane formed by x and z retinal axes. x axis is taken respect the
     * projected camera coordinates (i.e. retinal plane), and z axis just points
     * in the direction that the camera is looking at.
     *
     * @return horizontal plane respect camera retinal plane.
     */
    public Plane getHorizontalAxisPlane() {
        final Plane result = new Plane();
        horizontalAxisPlane(result);
        return result;
    }

    /**
     * Computes the plane formed by x and z retinal axes. x axis is taken
     * respect the projected camera coordinates (i.e. retinal plane), and z axis
     * just points in the direction that the camera is looking at.
     *
     * @param result plane where results will be stored.
     */
    public void horizontalAxisPlane(final Plane result) {

        // use second row of camera matrix to set plane
        result.setParameters(mInternalMatrix.getElementAt(1, 0),
                mInternalMatrix.getElementAt(1, 1),
                mInternalMatrix.getElementAt(1, 2),
                mInternalMatrix.getElementAt(1, 3));
    }

    /**
     * Returns plane formed by y and z retinal axes. y axis is taken respect the
     * projected camera coordinates (i.e. retinal plane), and z axis just points
     * in the direction that the camera is looking at.
     *
     * @return vertical plane respect camera retinal plane.
     */
    public Plane getVerticalAxisPlane() {
        final Plane result = new Plane();
        verticalAxisPlane(result);
        return result;
    }

    /**
     * Computes the plane formed by y and z retinal axes. y axis is taken
     * respect the projected camera coordinates (i.e. retinal plane), and z axis
     * just point in the direction that the camera is looking at.
     *
     * @param result plane where results will be stored.
     */
    public void verticalAxisPlane(final Plane result) {
        // use first row of camera matrix to set plane
        result.setParameters(mInternalMatrix.getElementAt(0, 0),
                mInternalMatrix.getElementAt(0, 1),
                mInternalMatrix.getElementAt(0, 2),
                mInternalMatrix.getElementAt(0, 3));
    }

    /**
     * Returns a plane equivalent to the retinal plane (i.e. the plane where 3D
     * points get projected). The principal plane director vector always points
     * in the direction that the camera is looking at, and the camera center is
     * locus of the principal plane.
     *
     * @return a plane equivalent to the retinal plane.
     */
    public Plane getPrincipalPlane() {
        final Plane result = new Plane();
        principalPlane(result);
        return result;
    }

    /**
     * Computes a plane equivalent to the retinal plane (i.e. the plane where 3D
     * points get projected). The principal plane director vector always points
     * in the direction that the camera is looking at, and the camera center is
     * locus of the principal plane.
     *
     * @param result plane where results will be stored.
     */
    public void principalPlane(final Plane result) {
        // use third row of camera matrix to set plane
        result.setParameters(mInternalMatrix.getElementAt(2, 0),
                mInternalMatrix.getElementAt(2, 1),
                mInternalMatrix.getElementAt(2, 2),
                mInternalMatrix.getElementAt(2, 3));
    }

    /**
     * Sets plane formed by x and z retinal plane. x axis is taken respect the
     * projected camera coordinates (i.e. retinal plane), and z axis just points
     * in the direction that the camera is looking at.
     *
     * @param horizontalAxisPlane horizontal plane respect camera retinal plane
     *                            to be set.
     */
    public void setHorizontalAxisPlane(final Plane horizontalAxisPlane) {
        // to increase accuracy
        horizontalAxisPlane.normalize();

        final double a = horizontalAxisPlane.getA();
        final double b = horizontalAxisPlane.getB();
        final double c = horizontalAxisPlane.getC();
        final double d = horizontalAxisPlane.getD();

        mInternalMatrix.setElementAt(1, 0, a);
        mInternalMatrix.setElementAt(1, 1, b);
        mInternalMatrix.setElementAt(1, 2, c);
        mInternalMatrix.setElementAt(1, 3, d);

        // set camera sign fixed and normalized
        mCameraSignFixed = false;
        mNormalized = false;
    }

    /**
     * Sets plane formed by y and z retinal plane. y axis is taken respect the
     * projected camera coordinates (i.e. retinal plane), and z axis just points
     * in the direction that the camera is looking at.
     *
     * @param verticalAxisPlane vertical plane respect camera retinal plane to
     *                          be set.
     */
    public void setVerticalAxisPlane(final Plane verticalAxisPlane) {
        // to increase accuracy
        verticalAxisPlane.normalize();

        final double a = verticalAxisPlane.getA();
        final double b = verticalAxisPlane.getB();
        final double c = verticalAxisPlane.getC();
        final double d = verticalAxisPlane.getD();

        mInternalMatrix.setElementAt(0, 0, a);
        mInternalMatrix.setElementAt(0, 1, b);
        mInternalMatrix.setElementAt(0, 2, c);
        mInternalMatrix.setElementAt(0, 3, d);

        // set camera sign fixed and normalized
        mCameraSignFixed = false;
        mNormalized = false;
    }

    /**
     * Sets plane equivalent to the retinal plane (i.e. the plane where 3D
     * points get projected). Notice that the principal plane director vector
     * always points in the direction that the camera is looking at, and that
     * the camera center is locus of the principal plane.
     *
     * @param principalPlane principal plane to be set.
     */
    public void setPrincipalPlane(final Plane principalPlane) {
        // to increase accuracy
        principalPlane.normalize();

        final double a = principalPlane.getA();
        final double b = principalPlane.getB();
        final double c = principalPlane.getC();
        final double d = principalPlane.getD();

        mInternalMatrix.setElementAt(2, 0, a);
        mInternalMatrix.setElementAt(2, 1, b);
        mInternalMatrix.setElementAt(2, 2, c);
        mInternalMatrix.setElementAt(2, 3, d);

        // set camera sign fixed and normalized
        mCameraSignFixed = false;
        mNormalized = false;
    }

    /**
     * Returns a 2D point indicating where the camera center (or the principal
     * axis) is projected on the retinal plane.
     * Usually the principal plane is located at the center (i.e. origin of
     * coordinates) of the retinal plane.
     *
     * @return the principal point laying on the retinal plane
     */
    public Point2D getPrincipalPoint() {
        final Point2D result = Point2D.create();
        principalPoint(result);
        return result;
    }

    /**
     * Computes the principal point which is a 2D point indicating where the
     * camera center (or the principal axis) is projected on the retinal plane.
     * Usually the principal plane is located at the center (i.e. origin of
     * coordinates) of the retinal plane.
     *
     * @param result 2D point where computed principal point will be stored
     */
    public void principalPoint(final Point2D result) {

        try {
            // the principal point is retrieved as the product of the 3x3
            // top-left sub-matrix of the internal camera with its second row
            final Matrix mMp = mInternalMatrix.getSubmatrix(0, 0,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1);
            final Matrix m = new Matrix(PINHOLE_CAMERA_MATRIX_ROWS, 1);
            m.setElementAtIndex(0, mMp.getElementAt(2, 0));
            m.setElementAtIndex(1, mMp.getElementAt(2, 1));
            m.setElementAtIndex(2, mMp.getElementAt(2, 2));

            mMp.multiply(m);

            result.setHomogeneousCoordinates(mMp.getElementAtIndex(0),
                    mMp.getElementAtIndex(1), mMp.getElementAtIndex(2));
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Returns the principal axis as an array consisting of the x,y,z
     * coordinates of the director vector of the principal plane. Hence, the
     * principal axis contains the direction that the camera is looking at.
     *
     * @return principal axis as an array.
     * @throws CameraException if there is numerical instability in camera
     *                         parameters.
     */
    public double[] getPrincipalAxisArray() throws CameraException {
        final double[] result = new double[PINHOLE_CAMERA_MATRIX_ROWS];
        principalAxisArray(result);
        return result;
    }

    /**
     * Computes the principal axis as an array consisting of the x,y,z
     * coordinates of the director vector of the principal plane. Hence, the
     * principal axis contains the direction that the camera is looking at.
     *
     * @param result array where principal axis coordinates will be stored.
     * @throws IllegalArgumentException if provided array does not have length 3.
     * @throws CameraException          if there is numerical instability in camera
     *                                  parameters.
     */
    public void principalAxisArray(final double[] result) throws CameraException {
        if (result.length != PINHOLE_CAMERA_MATRIX_ROWS) {
            throw new IllegalArgumentException();
        }

        try {
            // get first 3 elements of last row of camera matrix, which contains
            // director vector of principal plane
            mInternalMatrix.getSubmatrixAsArray(PINHOLE_CAMERA_MATRIX_ROWS - 1,
                    0, PINHOLE_CAMERA_MATRIX_ROWS - 1,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1, result);

            if (!isCameraSignFixed()) {
                final double cameraSign = getCameraSign();

                // fix sign of director vector
                ArrayUtils.multiplyByScalar(result, cameraSign, result);
            }

            // normalize director vector
            final double norm = Utils.normF(result);
            ArrayUtils.multiplyByScalar(result, 1.0 / norm, result);
        } catch (final WrongSizeException ignore) {
            // never happens
        }
    }

    /**
     * Returns camera sign of this camera.
     * Pinhole camera is defined in homogeneous coordinates, hence its internal
     * matrix can theoretically be scaled without affecting results (in practice
     * it can affect accuracy). However, scaling the camera with a different
     * sign can have an impact on determining whether points or objects are
     * located in front or behind the camera.
     * When camera sign is positive (i.e. 1.0), then points are correctly
     * detected whether they are in front or behind the camera (this is called
     * cheirality), when sign is negative, point cheirality is reversed and
     * needs to be fixed.
     *
     * @return 1.0 if camera sign is correct, or -1.0 if camera sign needs to be
     * reversed.
     * @throws CameraException if there is numerical instability.
     */
    public double getCameraSign() throws CameraException {
        return getCameraSign(SIGN_THRESHOLD);
    }

    /**
     * Returns camera sign of this camera up to provided threshold.
     * Pinhole camera is defined in homogeneous coordinates, hence its internal
     * matrix can theoretically be scaled without affecting results (in practice
     * it can affect accuracy). However, scaling the camera with a different
     * sign can have an impact on determining whether points or objects are
     * located in front or behind the camera.
     * When camera sign is positive (i.e. 1.0), then points are correctly
     * detected whether they are in front or behind the camera (this is called
     * cheirality), when sign is negative, point cheirality is reversed and
     * needs to be fixed.
     *
     * @param threshold threshold to determine whether camera sign is positive
     *                  or negative. Usually threshold is a very small value close to zero
     * @return 1.0 if camera sign is correct, or -1.0 if camera sign needs to be
     * reversed.
     * @throws CameraException if there is numerical instability.
     */
    public double getCameraSign(final double threshold) throws CameraException {
        try {
            // pick left 3x3 top-left sub-matrix
            final Matrix mMp = mInternalMatrix.getSubmatrix(0, 0,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1);

            // compute its determinant
            final double det = Utils.det(mMp);

            // get sign of determinant to determine pinhole camera matrix sign
            return (det > threshold) ? 1.0 : -1.0;
        } catch (final AlgebraException e) {
            throw new CameraException(e);
        }
    }

    /**
     * Returns the depth of provided point respect to camera center.
     * A positive value indicates that point is in front of the camera, a
     * negative value indicates that point is behind the camera.
     *
     * @param point point to be checked.
     * @return depth of provided point respect to camera center.
     * @throws CameraException if there is numerical instability.
     */
    public double getDepth(final Point3D point) throws CameraException {
        if (!isCameraCenterAvailable()) {
            // compute camera center
            mCameraCenter = computeCameraCenterSVD();
        }

        // because when computing principal axis vector it is normalized, we can
        // compute depth of world points respect to camera just as the dot
        // product between world points minus camera center and principal axis
        // vector using inhomogeneous coordinates
        final double[] principalAxis = getPrincipalAxisArray();
        final double[] diff = new double[INHOM_COORDS];
        diff[0] = point.getInhomX() - mCameraCenter.getInhomX();
        diff[1] = point.getInhomY() - mCameraCenter.getInhomY();
        diff[2] = point.getInhomZ() - mCameraCenter.getInhomZ();

        return ArrayUtils.dotProduct(principalAxis, diff);
    }

    /**
     * Returns the depth of provided points respect to camera center.
     * A positive value indicates that point is in front of the camera, a
     * negative value indicates that point is behind the camera.
     *
     * @param points points to be checked.
     * @return depth of provided points respect to camera center.
     * @throws CameraException if there is numerical instability.
     */
    public List<Double> getDepths(final List<Point3D> points) throws CameraException {
        final List<Double> depths = new ArrayList<>(points.size());
        depths(points, depths);
        return depths;
    }

    /**
     * Computes the depth of provided points respect to camera center and stores
     * the result in provided result list.
     * A positive value indicates that point is in front of the camera, a
     * negative value indicates that point is behind the camera.
     *
     * @param points points to be checked.
     * @param result list where depths of provided points will be stored.
     * @throws CameraException if there is numerical instability.
     */
    public void depths(final List<Point3D> points, final List<Double> result)
            throws CameraException {
        result.clear();
        for (final Point3D point : points) {
            result.add(getDepth(point));
        }
    }

    /**
     * Computes the cheirality of a point.
     * A positive cheirality indicates that a point is in front of the camera,
     * a negative value indicates that a point is behind the camera.
     * Cheirality is less expensive to compute than point depth, for that reason
     * when trying to determine if a point is in front or behind the camera is
     * preferable to check cheirality sign rather than depth sign.
     *
     * @param point point to be checked.
     * @return a positive value if point is in front of the camera, a negative
     * value otherwise.
     * @throws CameraException if there is numerical instability.
     */
    public double getCheirality(final Point3D point) throws CameraException {

        // normalize input point and camera to increase accuracy
        point.normalize();
        normalize();

        // pick last homogeneous component of 3D and 2D points
        final double hom3DW = point.getHomW();

        // W component of projected point can be computed as the dot product
        // between homogeneous 3D point and last row of pinhole camera matrix
        final double hom2DW = point.getHomX() * mInternalMatrix.getElementAt(2, 0) +
                point.getHomY() * mInternalMatrix.getElementAt(2, 1) +
                point.getHomZ() * mInternalMatrix.getElementAt(2, 2) +
                point.getHomW() * mInternalMatrix.getElementAt(2, 3);

        double cheiral = hom3DW * hom2DW;

        if (!isCameraSignFixed()) {
            cheiral *= getCameraSign();
        }

        return cheiral;
    }

    /**
     * Return the cheirality of the list of provided points.
     * A positive cheirality indicates that a point is in front of the camera,
     * a negative value indicates that a point is behind the camera.
     * Cheirality is less expensive to compute than point depth, for that reason
     * when trying to determine if a point is in front or behind the camera it
     * is preferable to check cheirality sign rather than depth sign.
     *
     * @param points list of points to be checked.
     * @return a list of cheirality values corresponding to provided points.
     * @throws CameraException if there is numerical instability.
     */
    public List<Double> getCheiralities(final List<Point3D> points)
            throws CameraException {
        final List<Double> cheiralities = new ArrayList<>(points.size());
        cheiralities(points, cheiralities);
        return cheiralities;
    }

    /**
     * Computes the cheirality of the list of provided points and stores the
     * result in result list.
     * A positive cheirality indicates that a point is in front of the camera,
     * a negative value indicates that a point is behind the camera.
     * Cheirality is less expensive to compute than point depth, for that reason
     * when trying to determine if a point is in front or behind the camera it
     * is preferable to check cheirality sign rather than depth sign.
     *
     * @param points list of points to be checked.
     * @param result list where cheiralities will be stored.
     * @throws CameraException if there is numerical instability.
     */
    public void cheiralities(final List<Point3D> points, final List<Double> result)
            throws CameraException {
        result.clear();
        for (final Point3D point : points) {
            result.add(getCheirality(point));
        }
    }

    /**
     * Determines if a given point is located in front of the camera.
     *
     * @param point point to be checked.
     * @return true if point is in front of the camera, false otherwise.
     * @throws CameraException if there is numerical instability.
     */
    public boolean isPointInFrontOfCamera(final Point3D point) throws CameraException {
        return isPointInFrontOfCamera(point, FRONT_THRESHOLD);
    }

    /**
     * Determines if a given point is located in front of the camera up to given
     * threshold.
     *
     * @param point     point to be checked.
     * @param threshold a threshold which typically is a small value close to
     *                  zero.
     * @return true if point is in front of the camera, false otherwise.
     * @throws CameraException if there is numerical instability.
     */
    public boolean isPointInFrontOfCamera(final Point3D point, final double threshold)
            throws CameraException {
        return getCheirality(point) > threshold;
    }

    /**
     * Returns list indicating if corresponding provided points are located in
     * front of the camera.
     *
     * @param points    points to be checked.
     * @param threshold a threshold which typically is a small value close to
     *                  zero.
     * @return a list of booleans indicating if the corresponding provided point
     * is located in front of the camera or not.
     * @throws CameraException if there is numerical instability.
     */
    public List<Boolean> arePointsInFrontOfCamera(final List<Point3D> points,
                                                  final double threshold) throws CameraException {
        final List<Boolean> result = new ArrayList<>(points.size());
        arePointsInFrontOfCamera(points, result, threshold);
        return result;
    }

    /**
     * Returns list indicating if corresponding provided points are located in
     * front of the camera.
     *
     * @param points points to be checked.
     * @return a list of booleans indicating if the corresponding provided point
     * is located in front of the camera or not.
     * @throws CameraException if there is numerical instability.
     */
    public List<Boolean> arePointsInFrontOfCamera(final List<Point3D> points)
            throws CameraException {
        return arePointsInFrontOfCamera(points, FRONT_THRESHOLD);
    }

    /**
     * Computes list indicating if corresponding provided points are located in
     * front of the camera or not.
     *
     * @param points    points to be checked.
     * @param result    list where results will be stored.
     * @param threshold a threshold which typically is a small value close to
     *                  zero.
     * @throws CameraException if there is numerical instability.
     */
    public void arePointsInFrontOfCamera(
            final List<Point3D> points, final List<Boolean> result, final double threshold)
            throws CameraException {
        result.clear();
        for (final Point3D point : points) {
            result.add(PinholeCamera.this.isPointInFrontOfCamera(point,
                    threshold));
        }
    }

    /**
     * COmputes list indicating if corresponding provided points are located in
     * front of the camera or not.
     *
     * @param points points to be checked.
     * @param result list where results will be stored.
     * @throws CameraException if there is numerical instability.
     */
    public void arePointsInFrontOfCamera(final List<Point3D> points,
                                         final List<Boolean> result) throws CameraException {
        arePointsInFrontOfCamera(points, result, FRONT_THRESHOLD);
    }

    /**
     * Creates an instance of PinholeCamera. Created instance is a canonical
     * camera equal to the 3x4 identity, which means that camera is located at
     * the origin with no translation or rotation.
     *
     * @return a canonical pinhole camera.
     */
    public static PinholeCamera createCanonicalCamera() {
        return new PinholeCamera();
    }

    /**
     * Decompose camera matrix 3x3 left minor and computes camera intrinsic
     * parameters and rotation.
     *
     * @throws CameraException if there is numerical instability.
     */
    private void computeIntrinsicsAndRotation() throws CameraException {
        try {
            // normalize camera to increase accuracy
            normalize();

            final Matrix mMp = mInternalMatrix.getSubmatrix(0, 0,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1);

            // Use RQ decomposition to obtain intrinsic parameters as R ensuring
            // that elements on the diagonal are positive and element (3, 3) is 1,
            // and Q is an orthogonal matrix
            final RQDecomposer decomposer = new RQDecomposer(mMp);
            decomposer.decompose();

            // Intrinsic parameters
            final Matrix r = decomposer.getR();
            final Matrix q = decomposer.getQ();

            // norm to normalize R
            double norm = r.getElementAt(2, 2);

            // ensure that norm is not too small
            if (Math.abs(norm) < EPS) {
                norm = (norm > 0.0 ? 1.0 : -1.0);
            }

            final double invNorm = 1.0 / norm;

            // build diagonal matrix to normalize R and obtain K
            final double[] vDiag = new double[PINHOLE_CAMERA_MATRIX_ROWS];
            final double[] vDiag2 = new double[PINHOLE_CAMERA_MATRIX_ROWS];

            if (invNorm * r.getElementAt(0, 0) > 0.0) {
                vDiag[0] = invNorm;
                vDiag2[0] = norm;
            } else {
                vDiag[0] = -invNorm;
                vDiag2[0] = -norm;
            }
            if (invNorm * r.getElementAt(1, 1) > 0.0) {
                vDiag[1] = invNorm;
                vDiag2[1] = norm;
            } else {
                vDiag[1] = -invNorm;
                vDiag2[1] = -norm;
            }

            vDiag[2] = invNorm;
            vDiag2[2] = norm;

            final Matrix mDiag = Matrix.diagonal(vDiag);
            final Matrix mDiag2 = Matrix.diagonal(vDiag2);

            r.multiply(mDiag);

            mDiag2.multiply(q);

            // thresholds should not be a problem and so we disable the change of
            // throwing any exception by setting infinity threshold (and ignoring
            // GeometryException)
            mIntrinsicParameters = new PinholeCameraIntrinsicParameters(
                    r, Double.POSITIVE_INFINITY);

            mCameraRotation = new MatrixRotation3D(mDiag2,
                    Double.POSITIVE_INFINITY);
        } catch (final AlgebraException e) {
            throw new CameraException(e);
        } catch (final GeometryException ignore) {
            // never happens
        }
    }

    /**
     * Computes camera center using singular value decomposition. This method
     * is valid even when center is located at infinity (w = 0), although it is
     * computationally more complex than other methods. This is the default
     * method used when decomposing a camera and computing its center.
     *
     * @return camera center.
     * @throws CameraException if there is numerical instability.
     */
    public Point3D computeCameraCenterSVD() throws CameraException {
        final Point3D result = Point3D.create();
        computeCameraCenterSVD(result);
        return result;
    }

    /**
     * Computes camera center using singular value decomposition. This method
     * is valid even when center is located at infinity, although it is
     * computationally more complex than other methods. This is the default
     * method used when decomposing a camera and computing its center.
     *
     * @param result point where camera center will be stored.
     * @throws CameraException if there is numerical instability.
     */
    public void computeCameraCenterSVD(final Point3D result) throws CameraException {
        try {
            normalize(); //to increase accuracy

            // camera center is the null-space of camera matrix
            final SingularValueDecomposer decomposer = new SingularValueDecomposer(
                    mInternalMatrix);

            decomposer.decompose();

            // because camera matrix is at most rank 3, the camera center is the
            // last column of decomposed matrix V
            final Matrix v = decomposer.getV();

            result.setHomogeneousCoordinates(
                    v.getElementAt(0, PINHOLE_CAMERA_MATRIX_COLS - 1),
                    v.getElementAt(1, PINHOLE_CAMERA_MATRIX_COLS - 1),
                    v.getElementAt(2, PINHOLE_CAMERA_MATRIX_COLS - 1),
                    v.getElementAt(3, PINHOLE_CAMERA_MATRIX_COLS - 1));
        } catch (final AlgebraException e) {
            throw new CameraException(e);
        }
    }

    /**
     * Computes camera center using determinants of camera matrix minors. This
     * method also works when center is located at infinity (w = 0) and is less
     * computationally expensive than SVD, however it might also be less
     * accurate.
     *
     * @return camera center.
     * @throws CameraException if there is numerical instabilities.
     */
    public Point3D computeCameraCenterDet() throws CameraException {
        final Point3D result = Point3D.create();
        computeCameraCenterDet(result);
        return result;
    }

    /**
     * Computes camera center using determinants of camera matrix minors. This
     * method also works when center is located at infinity (w = 0) and is less
     * computationally expensive than SVD, however it might also be less
     * accurate.
     *
     * @param result point where camera center will be stored.
     * @throws CameraException if there is numerical instability.
     */
    public void computeCameraCenterDet(final Point3D result) throws CameraException {

        try {
            // to increase accuracy
            normalize();

            final Matrix m = new Matrix(PINHOLE_CAMERA_MATRIX_ROWS,
                    PINHOLE_CAMERA_MATRIX_ROWS);

            // build minor using columns 2, 3 and 4
            m.setElementAt(0, 0, mInternalMatrix.getElementAt(0, 1));
            m.setElementAt(1, 0, mInternalMatrix.getElementAt(1, 1));
            m.setElementAt(2, 0, mInternalMatrix.getElementAt(2, 1));

            m.setElementAt(0, 1, mInternalMatrix.getElementAt(0, 2));
            m.setElementAt(1, 1, mInternalMatrix.getElementAt(1, 2));
            m.setElementAt(2, 1, mInternalMatrix.getElementAt(2, 2));

            m.setElementAt(0, 2, mInternalMatrix.getElementAt(0, 3));
            m.setElementAt(1, 2, mInternalMatrix.getElementAt(1, 3));
            m.setElementAt(2, 2, mInternalMatrix.getElementAt(2, 3));

            final double x = Utils.det(m);

            // build minor using columns 1, 3 and 4
            m.setElementAt(0, 0, mInternalMatrix.getElementAt(0, 0));
            m.setElementAt(1, 0, mInternalMatrix.getElementAt(1, 0));
            m.setElementAt(2, 0, mInternalMatrix.getElementAt(2, 0));

            m.setElementAt(0, 1, mInternalMatrix.getElementAt(0, 2));
            m.setElementAt(1, 1, mInternalMatrix.getElementAt(1, 2));
            m.setElementAt(2, 1, mInternalMatrix.getElementAt(2, 2));

            m.setElementAt(0, 2, mInternalMatrix.getElementAt(0, 3));
            m.setElementAt(1, 2, mInternalMatrix.getElementAt(1, 3));
            m.setElementAt(2, 2, mInternalMatrix.getElementAt(2, 3));

            final double y = -Utils.det(m);

            // build minor using columns 1, 2 and 4
            m.setElementAt(0, 0, mInternalMatrix.getElementAt(0, 0));
            m.setElementAt(1, 0, mInternalMatrix.getElementAt(1, 0));
            m.setElementAt(2, 0, mInternalMatrix.getElementAt(2, 0));

            m.setElementAt(0, 1, mInternalMatrix.getElementAt(0, 1));
            m.setElementAt(1, 1, mInternalMatrix.getElementAt(1, 1));
            m.setElementAt(2, 1, mInternalMatrix.getElementAt(2, 1));

            m.setElementAt(0, 2, mInternalMatrix.getElementAt(0, 3));
            m.setElementAt(1, 2, mInternalMatrix.getElementAt(1, 3));
            m.setElementAt(2, 2, mInternalMatrix.getElementAt(2, 3));

            final double z = Utils.det(m);

            // build minor using columns 1, 2 and 3
            m.setElementAt(0, 0, mInternalMatrix.getElementAt(0, 0));
            m.setElementAt(1, 0, mInternalMatrix.getElementAt(1, 0));
            m.setElementAt(2, 0, mInternalMatrix.getElementAt(2, 0));

            m.setElementAt(0, 1, mInternalMatrix.getElementAt(0, 1));
            m.setElementAt(1, 1, mInternalMatrix.getElementAt(1, 1));
            m.setElementAt(2, 1, mInternalMatrix.getElementAt(2, 1));

            m.setElementAt(0, 2, mInternalMatrix.getElementAt(0, 2));
            m.setElementAt(1, 2, mInternalMatrix.getElementAt(1, 2));
            m.setElementAt(2, 2, mInternalMatrix.getElementAt(2, 2));

            final double w = -Utils.det(m);

            result.setHomogeneousCoordinates(x, y, z, w);
            result.normalize();

        } catch (final AlgebraException e) {
            throw new CameraException(e);
        }
    }

    /**
     * Computes camera center. This method is better suited when camera is
     * finite (its center is not located at the infinity or close to it).
     * Otherwise, because of numerical precision inaccurate results might be
     * obtained, or even a CameraException might be thrown. This is the less
     * computationally expensive method.
     *
     * @return camera center.
     * @throws CameraException if there is numerical instabilities or center is
     *                         located at the infinity or close to it.
     */
    public Point3D computeCameraCenterFiniteCamera()
            throws CameraException {
        final Point3D result = Point3D.create();
        computeCameraCenterFiniteCamera(result);
        return result;
    }

    /**
     * Computes camera center. This method is better suited when camera is
     * finite (its center is not located at the infinity or close to it).
     * Otherwise, because of numerical precision inaccurate results might be
     * obtained, or even a CameraException might be thrown. This is the less
     * computationally expensive method.
     *
     * @param result point where camera center will be stored.
     * @throws CameraException if there is numerical instability.
     */
    public void computeCameraCenterFiniteCamera(final Point3D result)
            throws CameraException {

        try {
            // to increase accuracy
            normalize();

            // get top-left 3x3 sub-matrix
            final Matrix mMp = mInternalMatrix.getSubmatrix(0, 0,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1);

            // make inverse
            final Matrix mInvMp = Utils.inverse(mMp);

            // pick 4th column of camera matrix
            final Matrix mP4 = mInternalMatrix.getSubmatrix(0,
                    PINHOLE_CAMERA_MATRIX_COLS - 1,
                    PINHOLE_CAMERA_MATRIX_ROWS - 1,
                    PINHOLE_CAMERA_MATRIX_COLS - 1);

            // get inhomogeneous coordinates of camera center
            mInvMp.multiply(mP4);

            result.setInhomogeneousCoordinates(-mInvMp.getElementAtIndex(0),
                    -mInvMp.getElementAtIndex(1), -mInvMp.getElementAtIndex(2));
            result.normalize();
        } catch (final AlgebraException e) {
            throw new CameraException(e);
        }
    }

    /**
     * Estimates this camera parameters from 2D-3D point correspondences.
     *
     * @param point3D1 1st 3D point.
     * @param point3D2 2nd 3D point.
     * @param point3D3 3rd 3D point.
     * @param point3D4 4th 3D point.
     * @param point3D5 5th 3D point.
     * @param point3D6 6th 3D point.
     * @param point2D1 1st 2D point corresponding to the projection of 1st 3D
     *                 point.
     * @param point2D2 2nd 2D point corresponding to the projection of 2nd 3D
     *                 point.
     * @param point2D3 3rd 2D point corresponding to the projection of 3rd 3D
     *                 point.
     * @param point2D4 4th 2D point corresponding to the projection of 4th 3D
     *                 point.
     * @param point2D5 5th 2D point corresponding to the projection of 5th 3D
     *                 point.
     * @param point2D6 6th 2D point corresponding to the projection of 6th 3D
     *                 point.
     * @throws CameraException if camera cannot be estimated using provided
     *                         points because of a degeneracy.
     */
    public final void setFromPointCorrespondences(
            final Point3D point3D1, final Point3D point3D2, final Point3D point3D3,
            final Point3D point3D4, final Point3D point3D5, final Point3D point3D6,
            final Point2D point2D1, final Point2D point2D2, final Point2D point2D3,
            final Point2D point2D4, final Point2D point2D5, final Point2D point2D6) throws CameraException {

        final List<Point3D> points3D = new ArrayList<>(
                DLTPointCorrespondencePinholeCameraEstimator.
                        MIN_NUMBER_OF_POINT_CORRESPONDENCES);
        final List<Point2D> points2D = new ArrayList<>(
                DLTPointCorrespondencePinholeCameraEstimator.
                        MIN_NUMBER_OF_POINT_CORRESPONDENCES);

        points3D.add(point3D1);
        points3D.add(point3D2);
        points3D.add(point3D3);
        points3D.add(point3D4);
        points3D.add(point3D5);
        points3D.add(point3D6);

        points2D.add(point2D1);
        points2D.add(point2D2);
        points2D.add(point2D3);
        points2D.add(point2D4);
        points2D.add(point2D5);
        points2D.add(point2D6);

        try {
            final DLTPointCorrespondencePinholeCameraEstimator estimator =
                    new DLTPointCorrespondencePinholeCameraEstimator(points3D,
                            points2D);
            estimator.setLMSESolutionAllowed(false);
            final PinholeCamera camera = estimator.estimate();

            setInternalMatrix(camera.mInternalMatrix);
        } catch (final WrongSizeException | GeometryException e) {
            throw new CameraException(e);
        }
    }

    /**
     * Estimates this camera parameters from line/plane correspondences.
     *
     * @param plane1 1st 3D plane.
     * @param plane2 2nd 3D plane.
     * @param plane3 3rd 3D plane.
     * @param plane4 4th 3D plane.
     * @param line1  1st 2D line corresponding to 1st 3D plane.
     * @param line2  2nd 2D line corresponding to 2nd 3D plane.
     * @param line3  3rd 2D line corresponding to 3rd 3D plane.
     * @param line4  4th 2D line corresponding to 4th 3D plane.
     * @throws CameraException if camera cannot be estimated using provided
     *                         lines and planes because of a degeneracy.
     */
    public final void setFromLineAndPlaneCorrespondences(
            final Plane plane1, final Plane plane2, final Plane plane3, final Plane plane4,
            final Line2D line1, final Line2D line2, final Line2D line3, final Line2D line4)
            throws CameraException {

        final List<Plane> planes = new ArrayList<>(
                DLTLinePlaneCorrespondencePinholeCameraEstimator.
                        MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES);
        final List<Line2D> lines2D = new ArrayList<>(
                DLTLinePlaneCorrespondencePinholeCameraEstimator.
                        MIN_NUMBER_OF_LINE_PLANE_CORRESPONDENCES);

        planes.add(plane1);
        planes.add(plane2);
        planes.add(plane3);
        planes.add(plane4);

        lines2D.add(line1);
        lines2D.add(line2);
        lines2D.add(line3);
        lines2D.add(line4);

        try {
            final DLTLinePlaneCorrespondencePinholeCameraEstimator estimator =
                    new DLTLinePlaneCorrespondencePinholeCameraEstimator(planes,
                            lines2D);
            estimator.setLMSESolutionAllowed(false);
            final PinholeCamera camera = estimator.estimate();

            setInternalMatrix(camera.mInternalMatrix);
        } catch (final WrongSizeException | GeometryException e) {
            throw new CameraException(e);
        }
    }
}
