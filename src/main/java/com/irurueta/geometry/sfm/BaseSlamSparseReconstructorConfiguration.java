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

package com.irurueta.geometry.sfm;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.slam.BaseCalibrationData;

import java.io.Serializable;

/**
 * Contains base configuration for a multiple view sparse reconstructor using SLAM (Simultaneous
 * Location And Mapping) to determine the scale of the scene (i.e. the baseline or separation
 * between cameras) by fusing both camera data and data from sensors like an accelerometer or
 * gyroscope.
 * @param <C> type defining calibration data.
 * @param <T> an actual implementation of a configuration class.
 */
public class BaseSlamSparseReconstructorConfiguration<C extends BaseCalibrationData,
        T extends BaseSlamSparseReconstructorConfiguration> extends BaseSparseReconstructorConfiguration<T>
        implements Serializable {

    /**
     * Default variance for coordinates of estimated camera positions.
     */
    public static final double DEFAULT_CAMERA_POSITION_VARIANCE = 1e-6;

    /**
     * Calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If this is null, no calibration data will be used.
     */
    private C mCalibrationData;

    /**
     * Matrix containing covariance of measured camera positions.
     * This should usually be an "almost" diagonal matrix, where diagonal elements
     * are close to the position estimation error variance.
     * Values of this matrix are device specific and depends on factors such as
     * resolution of images, pictures quality, gyroscope and accelerometer accuracy.
     * This matrix must be a 3x3 symmetric positive definite matrix.
     */
    private Matrix mCameraPositionCovariance;

    /**
     * Constructor.
     */
    public BaseSlamSparseReconstructorConfiguration() {
        //initialize default covariance
        try {
            mCameraPositionCovariance = Matrix.identity(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            mCameraPositionCovariance.multiplyByScalar(DEFAULT_CAMERA_POSITION_VARIANCE);
        } catch (AlgebraException ignore) { }
    }

    /**
     * Gets calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If this is null, no calibration data will be used.
     * @return calibration data or null.
     */
    public C getCalibrationData() {
        return mCalibrationData;
    }

    /**
     * Specifies calibration data for accelerometer and gyroscope.
     * This data is usually captured and estimated in an offline step previous
     * to the actual scene reconstruction.
     * Calibration data is usually obtained by keeping the system in a constant
     * state of motion (e.g. acceleration and rotation).
     * If set to null, no calibration data will be used.
     * @param calibrationData calibration data or null.
     * @return this instance so that method can be easily chained.
     */
    public T setCalibrationData(C calibrationData) {
        mCalibrationData = calibrationData;
        return (T)this;
    }

    /**
     * Gets matrix containing covariance of measured camera positions.
     * This should usually be an "almost" diagonal matrix, where diagonal elements
     * are close to the position estimation error variance.
     * Values of this matrix are device specific and depends on factors such as
     * resolution of images, pictures quality, gyroscope and accelerometer accuracy.
     * This matrix must be a 3x3 symmetric positive definite matrix.
     * @return covariance of measured camera positions.
     */
    public Matrix getCameraPositionCovariance() {
        return mCameraPositionCovariance;
    }

    /**
     * Sets matrix containing covariance of measured camera positions.
     * This should usually be an "almost" diagonal matrix, where diagonal elements
     * are close to the position estimation error variance.
     * Values of this matrix are device specific and depends on factors such as
     * resolution of images, pictures quality, gyroscope and accelerometer accuracy.
     * This matrix must be a 3x3 symmetric positive definite matrix.
     * @param cameraPositionCovariance covariance of measured camera positions.
     * @return this instance so that method can be easily chained.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public T setCameraPositionCovariance(Matrix cameraPositionCovariance)
            throws IllegalArgumentException {
        if (cameraPositionCovariance.getRows() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH ||
                cameraPositionCovariance.getColumns() != Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
            throw new IllegalArgumentException();
        }

        mCameraPositionCovariance = cameraPositionCovariance;
        return (T)this;
    }

    public T setCameraPositionVariance(double variance) {
        try {
            mCameraPositionCovariance = Matrix.identity(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH,
                    Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH);
            mCameraPositionCovariance.multiplyByScalar(variance);
        } catch (AlgebraException ignore) { }
        return (T)this;
    }
}
