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
import com.irurueta.geometry.slam.AbsoluteOrientationBaseSlamEstimator;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class in charge of estimating cameras and 3D reconstructed points from sparse
 * image point correspondences in multiple views and also in charge of estimating overall
 * scene scale and absolute orientation by means of SLAM (Simultaneous Location And Mapping)
 * using data obtained from sensors like accelerometers or gyroscopes.
 * NOTE: absolute orientation slam estimators are not very accurate during estimation of
 * the orientation state, for that reason we take into account the initial orientation.
 * @param <C> type of configuration.
 * @param <R> type of reconstructor.
 * @param <S> type of SLAM estimator.
 */
public abstract class BaseAbsoluteOrientationSlamSparseReconstructor<
        C extends BaseSlamSparseReconstructorConfiguration,
        R extends BaseSlamSparseReconstructor,
        S extends AbsoluteOrientationBaseSlamEstimator> extends
        BaseSlamSparseReconstructor<C, R, S> {

    /**
     * First sample of orientation received.
     */
    protected Rotation3D mFirstOrientation;

    /**
     * Constructor.
     * @param configuration configuration for this reconstructor.
     * @param listener listener in charge of handling events.
     * @throws NullPointerException if listener or configuration is not
     * provided.
     */
    public BaseAbsoluteOrientationSlamSparseReconstructor(
            C configuration,
            BaseSparseReconstructorListener<R> listener)
            throws NullPointerException {
        super(configuration, listener);
    }

    /**
     * Provides a new orientation sample to update SLAM estimator.
     * If reconstructor is not running, calling this method has no effect.
     * @param timestamp timestamp of accelerometer sample since epoch time and
     * expressed in nanoseconds.
     * @param orientation new orientation.
     */
    public void updateOrientationSample(long timestamp,
                                        Rotation3D orientation) {
        if (mSlamEstimator != null) {
            mSlamEstimator.updateOrientationSample(timestamp, orientation);
        }
        if (mFirstOrientation == null) {
            //make a copy of orientation
            mFirstOrientation = orientation.toQuaternion();
        }
    }

    /**
     * Updates scene scale and orientation using SLAM data.
     * @return true if scale was successfully updated, false otherwise.
     */
    protected boolean updateScaleAndOrientation() {

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

            //R1' = R1*Rdiff
            //Rdiff = R1^T*R1'

            //where R1' is the desired orientation (obtained by sampling a
            //sensor)
            //and R1 is always the identity for the 1st camera.
            //Hence R1' = Rdiff

            //t1' is the desired translation which is zero for the 1st
            //camera.

            //We want: P1' = K*[R1' t1'] = K*[R1' 0]
            //And we have P1 = K[I 0]

            //We need a transformation T so that:
            //P1' = P1*T^-1 = K[I 0][R1' 0]
            //                      [0   1]

            //Hence: T^-1 = [R1' 0]
            //              [0   1]

            //or T = [R1'^T 0]
            //       [0     1]

            //because we are also applying a transformation of scale s,
            //the combination of both transformations is
            //T = [s*R1'^T 0]
            //    [0       1]

            Rotation3D r = mFirstOrientation.inverseRotationAndReturnNew();

            double baseline = center1.distanceTo(slamPosition);
            double estimatedBaseline = center1.distanceTo(center2);

            double scale = mCurrentScale = baseline / estimatedBaseline;
            double sqrScale = scale * scale;

            MetricTransformation3D scaleAndOrientationTransformation =
                    new MetricTransformation3D(scale);
            scaleAndOrientationTransformation.setRotation(r);

            //update scale of cameras
            PinholeCamera euclideanCamera1 = scaleAndOrientationTransformation.transformAndReturnNew(metricCamera1);
            PinholeCamera euclideanCamera2 = scaleAndOrientationTransformation.transformAndReturnNew(metricCamera2);

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

            List<Point3D> euclideanReconstructedPoints3D =
                    scaleAndOrientationTransformation.transformPointsAndReturnNew(metricReconstructedPoints3D);

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
