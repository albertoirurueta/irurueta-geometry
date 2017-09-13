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

import com.irurueta.geometry.*;
import com.irurueta.geometry.slam.BaseCalibrationData;
import com.irurueta.geometry.slam.BaseSlamEstimator;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from sparse
 * image point correspondences in multiple views and also in charge of estimating overall
 * scene scale by means of SLAM (Simultaneous Location And Mapping) using data obtained
 * from sensors like accelerometers or gyroscopes.
 * @param <C> type of configuration.
 * @param <R> type of reconstructor.
 * @param <S> type of SLAM estimator.
 */
public abstract class BaseSlamSparseReconstructor<
        C extends BaseSlamSparseReconstructorConfiguration,
        R extends BaseSparseReconstructor,
        S extends BaseSlamEstimator> extends BaseSparseReconstructor<C, R> {

    /**
     * Slam estimator to estimate position, speed, orientation using
     * accelerometer and gyroscope data.
     */
    protected S mSlamEstimator;

    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     * provided.
     */
    public BaseSlamSparseReconstructor(C configuration,
            BaseSparseReconstructorListener<R> listener) throws NullPointerException {
        super(configuration, listener);
    }

    /**
     * Provides a new accelerometer sample to update SLAM estimation.
     * This method must be called whenever the accelerometer sensor receives new
     * data.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of accelerometer sample since epoch time and
     * expressed in nanoseconds.
     * @param accelerationX linear acceleration along x-axis expressed in meters
     * per squared second (m/s^2).
     * @param accelerationY linear acceleration along y-axis expressed in meters
     * per squared second (m/s^2).
     * @param accelerationZ linear acceleration along z-axis expressed in meters
     * per squared second (m/s^2).
     */
    public void updateAccelerometerSample(long timestamp, float accelerationX,
                                          float accelerationY, float accelerationZ) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateAccelerometerSample(timestamp, accelerationX,
                    accelerationY, accelerationZ);
        }
    }

    /**
     * Provides a new accelerometer sample to update SLAM estimation.
     * This method must be called whenever the accelerometer sensor receives new
     * data.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of accelerometer sample since epoch time and
     * expressed in nanoseconds.
     * @param data array containing x,y,z components of linear acceleration
     * expressed in meters per squared second (m/s^2).
     * @throws IllegalArgumentException if provided array does not have length
     * 3.
     */
    public void updateAccelerometerSample(long timestamp, float[] data)
            throws IllegalArgumentException {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateAccelerometerSample(timestamp, data);
        }
    }

    /**
     * Provides a new gyroscope sample to update SLAM estimation.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of gyroscope sample since epoch time and
     * expressed in nanoseconds.
     * @param angularSpeedX angular speed of rotation along x-axis expressed in
     * radians per second (rad/s).
     * @param angularSpeedY angular speed of rotation along y-axis expressed in
     * radians per second (rad/s).
     * @param angularSpeedZ angular speed of rotation along z-axis expressed in
     * radians per second (rad/s).
     */
    public void updateGyroscopeSample(long timestamp, float angularSpeedX,
                                      float angularSpeedY, float angularSpeedZ) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateGyroscopeSample(timestamp, angularSpeedX,
                    angularSpeedY, angularSpeedZ);
        }
    }

    /**
     * Provies a new gyroscope sample to update SLAM estimation.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of gyroscope sample since epoch time and
     * expressed in nanoseconds.
     * @param data angular speed of rotation along x,y,z axes expressed in
     * radians per second (rad/s).
     * @throws IllegalArgumentException if provided array does not have length
     * 3.
     */
    public void updateGyroscopeSample(long timestamp, float[] data)
            throws IllegalArgumentException {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateGyroscopeSample(timestamp, data);
        }
    }

    /**
     * Set ups calibration data on SLAM estimator if available.
     */
    protected void setUpCalibrationData() {
        BaseCalibrationData calibrationData =
                mConfiguration.getCalibrationData();
        if (calibrationData != null) {
            mSlamEstimator.setCalibrationData(calibrationData);
        }
    }

    /**
     * Update scene scale using SLAM data.
     * @return true if scale was successfully updated, false otherwise.
     */
    protected boolean updateScale() {
        //obtain baseline (camera separation from slam estimator data
        double posX = mSlamEstimator.getStatePositionX();
        double posY = mSlamEstimator.getStatePositionY();
        double posZ = mSlamEstimator.getStatePositionZ();

        InhomogeneousPoint3D slamPosition = new InhomogeneousPoint3D(posX, posY, posZ);

        try {
            PinholeCamera metricCamera1 = mPreviousMetricEstimatedCamera.getCamera();
            PinholeCamera metricCamera2 = mCurrentMetricEstimatedCamera.getCamera();

            metricCamera1.decompose();
            metricCamera2.decompose();

            Point3D center1 = metricCamera1.getCameraCenter();
            Point3D center2 = metricCamera2.getCameraCenter();

            double baseline = center1.distanceTo(slamPosition);
            double estimatedBaseline = center1.distanceTo(center2);

            double scale = mCurrentScale = baseline / estimatedBaseline;
            double sqrScale = scale * scale;

            MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);

            //update scale of cameras
            PinholeCamera euclideanCamera1 = scaleTransformation.transformAndReturnNew(metricCamera1);
            PinholeCamera euclideanCamera2 = scaleTransformation.transformAndReturnNew(metricCamera2);

            mPreviousEuclideanEstimatedCamera = new EstimatedCamera();
            mPreviousEuclideanEstimatedCamera.setCamera(euclideanCamera1);
            mPreviousEuclideanEstimatedCamera.setId(mPreviousMetricEstimatedCamera.getId());
            mPreviousEuclideanEstimatedCamera.setQualityScore(mPreviousMetricEstimatedCamera.getQualityScore());
            if (mPreviousMetricEstimatedCamera.getCovariance() != null) {
                mPreviousEuclideanEstimatedCamera.setCovariance(
                        mPreviousMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            mCurrentEuclideanEstimatedCamera = new EstimatedCamera();
            mCurrentEuclideanEstimatedCamera.setCamera(euclideanCamera2);
            mCurrentEuclideanEstimatedCamera.setId(mCurrentMetricEstimatedCamera.getId());
            mCurrentEuclideanEstimatedCamera.setQualityScore(mCurrentMetricEstimatedCamera.getQualityScore());
            if (mCurrentMetricEstimatedCamera.getCovariance() != null) {
                mCurrentEuclideanEstimatedCamera.setCovariance(
                        mCurrentMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            //update scale of reconstructed points
            int numPoints = mActiveMetricReconstructedPoints.size();
            List<Point3D> metricReconstructedPoints3D = new ArrayList<Point3D>();
            for (int i = 0; i < numPoints; i++) {
                metricReconstructedPoints3D.add(mActiveMetricReconstructedPoints.get(i).
                        getPoint());
            }

            List<Point3D> euclideanReconstructedPoints3D = scaleTransformation.transformPointsAndReturnNew(
                    metricReconstructedPoints3D);

            //set scaled points into result
            mActiveEuclideanReconstructedPoints = new ArrayList<>();
            ReconstructedPoint3D euclideanPoint;
            ReconstructedPoint3D metricPoint;
            for (int i = 0; i < numPoints; i++) {
                metricPoint = mActiveMetricReconstructedPoints.get(i);

                euclideanPoint = new ReconstructedPoint3D();
                euclideanPoint.setId(metricPoint.getId());
                euclideanPoint.setPoint(euclideanReconstructedPoints3D.get(i));
                euclideanPoint.setInlier(metricPoint.isInlier());
                euclideanPoint.setQualityScore(metricPoint.getQualityScore());
                if (metricPoint.getCovariance() != null) {
                    euclideanPoint.setCovariance(metricPoint.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
                }
                euclideanPoint.setColorData(metricPoint.getColorData());

                mActiveEuclideanReconstructedPoints.add(euclideanPoint);
            }

            return true;
        } catch (Exception e) {
            mFailed = true;
            mListener.onFail((R)this);

            return false;
        }
    }
}
