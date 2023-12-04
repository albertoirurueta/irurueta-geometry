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
package com.irurueta.geometry.estimators;

import org.junit.Test;

import static org.junit.Assert.*;

public class PinholeCameraEstimatorTest {

    @Test
    public void testCreate() {
        // create with type
        PinholeCameraEstimator estimator = PinholeCameraEstimator.create(
                PinholeCameraEstimatorType.DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertEquals(PinholeCameraEstimatorType.DLT_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR,
                estimator.getType());

        estimator = PinholeCameraEstimator.create(
                PinholeCameraEstimatorType.WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR,
                estimator.getType());

        estimator = PinholeCameraEstimator.create(
                PinholeCameraEstimatorType.WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR);
        assertEquals(PinholeCameraEstimatorType.WEIGHTED_POINT_PINHOLE_CAMERA_ESTIMATOR,
                estimator.getType());

        estimator = PinholeCameraEstimator.create(
                PinholeCameraEstimatorType.DLT_POINT_PINHOLE_CAMERA_ESTIMATOR);
        assertEquals(PinholeCameraEstimatorType.DLT_POINT_PINHOLE_CAMERA_ESTIMATOR,
                estimator.getType());


        estimator = PinholeCameraEstimator.create();

        // check correctness
        assertNull(estimator.getListener());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED,
                estimator.isSuggestSkewnessValueEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE,
                estimator.getSuggestedSkewnessValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestHorizontalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedHorizontalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED,
                estimator.isSuggestVerticalFocalLengthEnabled());
        assertEquals(0.0, estimator.getSuggestedVerticalFocalLengthValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED,
                estimator.isSuggestAspectRatioEnabled());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE,
                estimator.getSuggestedAspectRatioValue(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED,
                estimator.isSuggestPrincipalPointEnabled());
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED,
                estimator.isSuggestRotationEnabled());
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED,
                estimator.isSuggestCenterEnabled());
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT,
                estimator.getMinSuggestionWeight(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT,
                estimator.getMaxSuggestionWeight(), 0.0);
        assertEquals(PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP,
                estimator.getSuggestionWeightStep(), 0.0);
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(PinholeCameraEstimatorType.DLT_POINT_PINHOLE_CAMERA_ESTIMATOR,
                estimator.getType());
    }
}
