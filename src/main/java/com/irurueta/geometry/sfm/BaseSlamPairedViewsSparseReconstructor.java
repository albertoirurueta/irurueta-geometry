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
import com.irurueta.geometry.*;
import com.irurueta.geometry.slam.BaseCalibrationData;
import com.irurueta.geometry.slam.BaseSlamEstimator;

import java.util.ArrayList;

public abstract class BaseSlamPairedViewsSparseReconstructor<
        C extends BaseSlamPairedViewsSparseReconstructorConfiguration,
        R extends BaseSlamPairedViewsSparseReconstructor,
        L extends BaseSlamPairedViewsSparseReconstructorListener<R>,
        S extends BaseSlamEstimator> extends BasePairedViewsSparseReconstructor<C, R, L> {

    /**
     * Slam estimator to estimate position, speed, orientation using
     * accelerometer and gyroscope data.
     */
    protected S mSlamEstimator;

    /**
     * Position estimated by means of SLAM. It is reused each time it is notified.
     */
    protected InhomogeneousPoint3D mSlamPosition = new InhomogeneousPoint3D();

    /**
     * Camera estimated by means of SLAM. It is reused each time it is notified.
     */
    private PinholeCamera mSlamCamera = new PinholeCamera();

    /**
     * Camera rotation estimated by means of SLAM. It is reused each time it is notified.
     */
    private Quaternion mSlamRotation = new Quaternion();

    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     * provided.
     */
    public BaseSlamPairedViewsSparseReconstructor(C configuration, L listener)
            throws NullPointerException {
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
     * Configures calibration data on SLAM estimator if available.
     */
    protected void setUpCalibrationData() {
        BaseCalibrationData calibrationData =
                mConfiguration.getCalibrationData();
        if (calibrationData != null) {
            //noinspection all
            mSlamEstimator.setCalibrationData(calibrationData);
        }
    }

    /**
     * Configures listener of SLAM estimator
     */
    protected void setUpSlamEstimatorListener() {
        mSlamEstimator.setListener(new BaseSlamEstimator.BaseSlamEstimatorListener() {
            @Override
            public void onFullSampleReceived(BaseSlamEstimator estimator) { /* not used */ }

            @Override
            public void onFullSampleProcessed(BaseSlamEstimator estimator) {
                notifySlamStateIfNeeded();
                notifySlamCameraIfNeeded();
            }

            @Override
            public void onCorrectWithPositionMeasure(BaseSlamEstimator estimator) { /* not used */ }

            @Override
            public void onCorrectedWithPositionMeasure(BaseSlamEstimator estimator) {
                notifySlamStateIfNeeded();
                notifySlamCameraIfNeeded();
            }
        });
    }

    /**
     * Transforms metric cameras on current pair of views so that they are referred to
     * last kept location and rotation and upgrades cameras from metric stratum to
     * euclidean stratum.
     * @param isInitialPairOfViews true if initial pair of views is being processed, false otherwise.
     * @return true if cameras were successfully transformed.
     */
    @Override
    protected boolean transformPairOfCamerasAndPoints(boolean isInitialPairOfViews) {
        if (!super.transformPairOfCamerasAndPoints(isInitialPairOfViews)) {
            return false;
        }

        PinholeCamera previousMetricCamera = mPreviousMetricEstimatedCamera.getCamera();
        PinholeCamera currentMetricCamera = mCurrentMetricEstimatedCamera.getCamera();
        if (previousMetricCamera == null || currentMetricCamera == null) {
            return false;
        }

        mCurrentScale = estimateCurrentScale(isInitialPairOfViews);
        if (mFailed) {
            return false;
        }

        double sqrScale = mCurrentScale * mCurrentScale;

//        if (isInitialPairOfViews) {
            //first pair of views does not require setting translation and rotation
            mReferenceEuclideanTransformation = new MetricTransformation3D(mCurrentScale);
/*        } else {
            //additional pairs also need to translate and rotate
            Rotation3D invRot = mLastEuclideanCameraRotation.inverseRotationAndReturnNew();
            double[] translation = new double[Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH];
            translation[0] = mLastEuclideanCameraCenter.getInhomX();
            translation[1] = mLastEuclideanCameraCenter.getInhomY();
            translation[2] = mLastEuclideanCameraCenter.getInhomZ();
            mReferenceEuclideanTransformation = new MetricTransformation3D(invRot, translation, mCurrentScale);
        }*/

        try {
            //transform cameras
            PinholeCamera previousEuclideanCamera = mReferenceEuclideanTransformation.
                    transformAndReturnNew(previousMetricCamera);
            PinholeCamera currentEuclideanCamera = mReferenceEuclideanTransformation.
                    transformAndReturnNew(currentMetricCamera);

            mPreviousEuclideanEstimatedCamera = new EstimatedCamera();
            mPreviousEuclideanEstimatedCamera.setCamera(previousEuclideanCamera);
            mPreviousEuclideanEstimatedCamera.setViewId(mPreviousMetricEstimatedCamera.getViewId());
            mPreviousEuclideanEstimatedCamera.setQualityScore(mPreviousMetricEstimatedCamera.getQualityScore());
            if (mPreviousMetricEstimatedCamera.getCovariance() != null) {
                mPreviousEuclideanEstimatedCamera.setCovariance(
                        mPreviousMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            mCurrentEuclideanEstimatedCamera = new EstimatedCamera();
            mCurrentEuclideanEstimatedCamera.setCamera(currentEuclideanCamera);
            mCurrentEuclideanEstimatedCamera.setViewId(mCurrentMetricEstimatedCamera.getViewId());
            mCurrentEuclideanEstimatedCamera.setQualityScore(mCurrentMetricEstimatedCamera.getQualityScore());
            if (mCurrentMetricEstimatedCamera.getCovariance() != null) {
                mCurrentEuclideanEstimatedCamera.setCovariance(
                        mCurrentMetricEstimatedCamera.getCovariance().multiplyByScalarAndReturnNew(sqrScale));
            }

            //transform points
            mEuclideanReconstructedPoints = new ArrayList<>();
            ReconstructedPoint3D euclideanReconstructedPoint;
            Point3D metricPoint, euclideanPoint;
            for (ReconstructedPoint3D metricReconstructedPoint : mMetricReconstructedPoints) {
                metricPoint = metricReconstructedPoint.getPoint();
                euclideanPoint = mReferenceEuclideanTransformation.transformAndReturnNew(
                        metricPoint);
                euclideanReconstructedPoint = new ReconstructedPoint3D();
                euclideanReconstructedPoint.setPoint(euclideanPoint);
                euclideanReconstructedPoint.setInlier(metricReconstructedPoint.isInlier());
                euclideanReconstructedPoint.setId(metricReconstructedPoint.getId());
                euclideanReconstructedPoint.setColorData(metricReconstructedPoint.getColorData());
                if (metricReconstructedPoint.getCovariance() != null) {
                    euclideanReconstructedPoint.setCovariance(metricReconstructedPoint.getCovariance().
                            multiplyByScalarAndReturnNew(sqrScale));
                }
                euclideanReconstructedPoint.setQualityScore(metricReconstructedPoint.getQualityScore());
                mEuclideanReconstructedPoints.add(euclideanReconstructedPoint);
            }

        } catch (AlgebraException e) {
            return false;
        }

        return true;
    }

    /**
     * Estimates current scale using SLAM data.
     * @param isInitialPairOfViews true if initial pair of views is being processed, false otherwise.
     * @return estimated scale.
     */
    private double estimateCurrentScale(boolean isInitialPairOfViews) {

        try {
            PinholeCamera metricCamera1 = mPreviousMetricEstimatedCamera.getCamera();
            PinholeCamera metricCamera2 = mCurrentMetricEstimatedCamera.getCamera();

            if (!metricCamera2.isCameraCenterAvailable()) {
                metricCamera2.decompose(false, true);
            }
            Point3D metricCenter2 = metricCamera2.getCameraCenter();
            Point3D euclideanCenter2;

            double slamPosX, slamPosY, slamPosZ;
            double scale;
            if (isInitialPairOfViews) {
                //obtain baseline (camera separation from slam estimator data)
                slamPosX = mSlamEstimator.getStatePositionX();
                slamPosY = mSlamEstimator.getStatePositionY();
                slamPosZ = mSlamEstimator.getStatePositionZ();

                mSlamPosition.setInhomogeneousCoordinates(slamPosX, slamPosY, slamPosZ);

                if (!metricCamera1.isCameraCenterAvailable()) {
                    metricCamera1.decompose(false, true);
                }

                Point3D metricCenter1 = metricCamera1.getCameraCenter();

                double euclideanBaseline = mLastEuclideanCameraCenter.distanceTo(mSlamPosition);
//                double euclideanBaseline = metricCenter1.distanceTo(mSlamPosition);
                double metricBaseline = metricCenter1.distanceTo(metricCenter2);

                scale = mCurrentScale =  euclideanBaseline / metricBaseline;
            } else {
                scale = mCurrentScale;
            }

            MetricTransformation3D scaleTransformation =
                    new MetricTransformation3D(scale);
            euclideanCenter2 = scaleTransformation.transformAndReturnNew(metricCenter2);
            mSlamEstimator.correctWithPositionMeasure(euclideanCenter2,
                    mConfiguration.getCameraPositionCovariance());

            if (!isInitialPairOfViews) {
                slamPosX = mSlamEstimator.getStatePositionX();
                slamPosY = mSlamEstimator.getStatePositionY();
                slamPosZ = mSlamEstimator.getStatePositionZ();
                mSlamPosition.setInhomogeneousCoordinates(slamPosX, slamPosY, slamPosZ);

                //adjust scale of current camera
                double euclideanPosX = euclideanCenter2.getInhomX();
                double euclideanPosY = euclideanCenter2.getInhomY();
                double euclideanPosZ = euclideanCenter2.getInhomZ();

                double scaleVariationX = euclideanPosX / slamPosX;
                double scaleVariationY = euclideanPosY / slamPosY;
                double scaleVariationZ = euclideanPosZ / slamPosZ;

                double scaleVariation = (scaleVariationX + scaleVariationY + scaleVariationZ) / 3.0;
                scale *= scaleVariation;
                mCurrentScale = scale;
            }

            return scale;

        } catch (Exception e) {
            mFailed = true;
            //noinspection all
            mListener.onFail((R)this);
            return DEFAULT_SCALE;
        }

    }

    /**
     * Notifies SLAM state if notification is enabled at configuration time.
     */
    private void notifySlamStateIfNeeded() {
        if (!mConfiguration.isNotifyAvailableSlamDataEnabled()) {
            return;
        }

        double positionX = mSlamEstimator.getStatePositionX();
        double positionY = mSlamEstimator.getStatePositionY();
        double positionZ = mSlamEstimator.getStatePositionZ();

        double velocityX = mSlamEstimator.getStateVelocityX();
        double velocityY = mSlamEstimator.getStateVelocityY();
        double velocityZ = mSlamEstimator.getStateVelocityZ();

        double accelerationX = mSlamEstimator.getStateAccelerationX();
        double accelerationY = mSlamEstimator.getStateAccelerationY();
        double accelerationZ = mSlamEstimator.getStateAccelerationZ();

        double quaternionA = mSlamEstimator.getStateQuaternionA();
        double quaternionB = mSlamEstimator.getStateQuaternionB();
        double quaternionC = mSlamEstimator.getStateQuaternionC();
        double quaternionD = mSlamEstimator.getStateQuaternionD();

        double angularSpeedX = mSlamEstimator.getStateAngularSpeedX();
        double angularSpeedY = mSlamEstimator.getStateAngularSpeedY();
        double angularSpeedZ = mSlamEstimator.getStateAngularSpeedZ();

        //noinspection all
        mListener.onSlamDataAvailable((R)this, positionX, positionY, positionZ,
                velocityX, velocityY, velocityZ,
                accelerationX, accelerationY, accelerationZ,
                quaternionA, quaternionB, quaternionC, quaternionD,
                angularSpeedX, angularSpeedY, angularSpeedZ, mSlamEstimator.getStateCovariance());
    }

    /**
     * Notifies estimated camera by means of SLAM if notification is enabled at
     * configuration time and intrinsics are already available.
     */
    private void notifySlamCameraIfNeeded() {
        if (!mConfiguration.isNotifyEstimatedSlamCameraEnabled()) {
            return;
        }

        //try with current camera
        PinholeCamera camera = mCurrentEuclideanEstimatedCamera != null ?
                mCurrentEuclideanEstimatedCamera.getCamera() : null;
        if (camera == null) {
            //if not available try with previous camera
            camera = mPreviousEuclideanEstimatedCamera != null ?
                    mPreviousEuclideanEstimatedCamera.getCamera() : null;
        }

        try {
            PinholeCameraIntrinsicParameters intrinsicParameters = null;
            if (camera != null) {
                if (!camera.areIntrinsicParametersAvailable()) {
                    //decompose camera to obtain intrinsic parameters
                    camera.decompose();
                }

                intrinsicParameters = camera.getIntrinsicParameters();
            } else if (mConfiguration.areIntrinsicParametersKnown()) {
                //noinspection unchecked
                intrinsicParameters = mListener.onIntrinsicParametersRequested((R)this, mCurrentViewId);
            }

            if (intrinsicParameters == null) {
                return;
            }

            double positionX = mSlamEstimator.getStatePositionX();
            double positionY = mSlamEstimator.getStatePositionY();
            double positionZ = mSlamEstimator.getStatePositionZ();
            mSlamPosition.setInhomogeneousCoordinates(positionX, positionY, positionZ);

            double quaternionA = mSlamEstimator.getStateQuaternionA();
            double quaternionB = mSlamEstimator.getStateQuaternionB();
            double quaternionC = mSlamEstimator.getStateQuaternionC();
            double quaternionD = mSlamEstimator.getStateQuaternionD();
            mSlamRotation.setA(quaternionA);
            mSlamRotation.setB(quaternionB);
            mSlamRotation.setC(quaternionC);
            mSlamRotation.setD(quaternionD);

            mSlamCamera.setIntrinsicAndExtrinsicParameters(intrinsicParameters, mSlamRotation,
                    mSlamPosition);

            //noinspection all
            mListener.onSlamCameraEstimated((R)this, mSlamCamera);

        } catch (GeometryException ignore) { /* do nothing */ }
    }
}
