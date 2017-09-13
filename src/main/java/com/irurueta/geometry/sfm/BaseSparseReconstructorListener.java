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

import java.util.List;

/**
 * Listener to retrieve and store required data to compute a 3D reconstruction from
 * sparse image point correspondences in multiple views.
 * @param <R> type of reconstructor.
 */
public interface BaseSparseReconstructorListener<R extends BaseSparseReconstructor> {

    /**
     * Called to determine whether there are more views available to attempt to use for
     * the reconstruction.
     * @param reconstructor reconstructor raising this event.
     * @return true if there are more views available, false otherwise.
     */
    boolean hasMoreViewsAvailable(R reconstructor);

    /**
     * Called when samples containing points of interest for current view must be retrieved.
     * @param reconstructor reconstructor raising this event.
     * @param viewId id of view where points will be used.
     * @param samples samples containing points of interest for current view to test.
     */
    void onRequestSamplesForCurrentView(R reconstructor, int viewId, List<Sample2D> samples);

    /**
     * Called when requested samples have been accepted.
     * This method can be used to determine whether samples can be stored or
     * not.
     * @param reconstructor reconstructor raising this event.
     * @param viewId id of view whose samples have been accepted.
     * @param samples accepted samples.
     */
    void onSamplesAccepted(R reconstructor, int viewId, List<Sample2D> samples);

    /**
     * Called when requested samples have been rejected.
     * This method can be used to remove provided samples.
     * @param reconstructor reconstructor raising this event.
     * @param viewId id of view whose samples have been rejected.
     * @param samples rejected samples.
     */
    void onSamplesRejected(R reconstructor, int viewId, List<Sample2D> samples);

    /**
     * Finds matches for provided samples.
     * @param reconstructor reconstructor raising this event.
     * @param samples1 samples on first view.
     * @param samples2 samples on second view.
     * @param viewId1 id of first view.
     * @param viewId2 id of second view.
     * @param matches instance where matches must be stored.
     */
    void onRequestMatches(R reconstructor, List<Sample2D> samples1,
                          List<Sample2D> samples2, int viewId1, int viewId2,
                          List<MatchedSamples> matches);

    /**
     * Called when a fundamental matrix relating two views has been estimated.
     * This event can be used to store estimated fundamental matrix relating
     * two views.
     * @param reconstructor reconstructor raising this event.
     * @param estimatedFundamentalMatrix estimated fundamental matrix.
     */
    void onFundamentalMatrixEstimated(R reconstructor,
                                      EstimatedFundamentalMatrix estimatedFundamentalMatrix);

    /**
     * Notifies when cameras for provided matched pair of views have been
     * estimated. Cameras returned on this event are defined in a metric stratum (i.e. up to scale).
     * This event can be used to store cameras associated to such view.
     * @param reconstructor reconstructor raising this event.
     * @param previousViewId id of previous view (i.e. first view).
     * @param currentViewId id of current view (i.e. second view).
     * @param previousCamera estimated camera for previous view.
     * @param currentCamera estimated camera for current view.
     */
    void onMetricCameraEstimated(R reconstructor, int previousViewId, int currentViewId,
                            EstimatedCamera previousCamera, EstimatedCamera currentCamera);

    /**
     * Called when reconstructed points have been estimated from a series of 2D
     * matches. Reconstructed points returned on this event are defined in a metric stratum (i.e. up to scale).
     * This event can be used to store reconstructed points and their associated data.
     * @param reconstructor reconstructor raising this event.
     * @param matches 2D matches associated to estimated reconstructed points.
     * @param points reconstructed 3D points.
     */
    void onMetricReconstructedPointsEstimated(R reconstructor, List<MatchedSamples> matches,
                                        List<ReconstructedPoint3D> points);

    /**
     * Called when cameras for provided matched pair of views have been estimated in a metric stratum (when possible and
     * up to a certain accuracy).
     * Except SparseReconstructor, which can only make estimations in a metric stratum, other reconstructor
     * implementations either have calibration knowledge to estimate scale, or use SLAM techniques by mixing additional
     * sensor data (i.e. gyroscope and accelerometer) to estimate such scale.
     * @param reconstructor reconstructor raising this event.
     * @param previousViewId id of previous view (i.e. first view).
     * @param currentViewId id of current view (i.e. second view).
     * @param scale estimated scale. This will typically converge to 1.0 as more views are processed. The closer this
     *              value is to one, the more likely the scale of estimated cameras is accurate.
     * @param previousCamera estimated camera for previous view.
     * @param currentCamera estimated camera for current view.
     */
    void onEuclideanCameraEstimated(R reconstructor, int previousViewId, int currentViewId, double scale,
                                    EstimatedCamera previousCamera, EstimatedCamera currentCamera);

    /**
     * Called when reconstructed points have been estimated from a series of 2D matches.
     * Except SparseReconstructor, which can only make estimations in a metric stratum, other reconstructor
     * implementations either have calibration knowledge to estimate scale, or use SLAM techniques by mixing additional
     * sensor data (i.e. gyroscope and accelerometer) to estimate such scale.
     * @param reconstructor reconstructor raising this event.
     * @param scale estimated scale. This will typically converge to 1.0 as more views are processed. The closer this
     *              value is to one, the more likely the scale of estimated cameras is accurate.
     * @param points reconstructed 3D points.
     */
    void onEuclideanReconstructedPointsEstimated(R reconstructor, double scale, List<ReconstructedPoint3D> points);

    /**
     * Called when reconstruction starts.
     * @param reconstructor reconstructor raising this event.
     */
    void onStart(R reconstructor);

    /**
     * Called when reconstruction stops.
     * @param reconstructor reconstructor raising this event.
     */
    void onFinish(R reconstructor);

    /**
     * Called when reconstruction is cancelled before it has finished.
     * @param reconstructor reconstructor raising this event.
     */
    void onCancel(R reconstructor);

    /**
     * Called when reconstruction fails.
     * @param reconstructor reconstructor raising this event.
     */
    void onFail(R reconstructor);

}
