/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PinholeCameraEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 8, 2017.
 */
package com.irurueta.geometry.estimators;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class PinholeCameraEstimatorTest {
    
    public PinholeCameraEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testCreate() {
        PinholeCameraEstimator estimator = PinholeCameraEstimator.create();
        
        //check correctness
        assertNull(estimator.getListener());
        assertEquals(estimator.isSuggestSkewnessValueEnabled(), 
                PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        assertEquals(estimator.getSuggestedSkewnessValue(), 
                PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(), 
                PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0, 
                0.0);
        assertEquals(estimator.isSuggestAspectRatioEnabled(), 
                PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        assertEquals(estimator.getSuggestedAspectRatioValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 
                0.0);
        assertEquals(estimator.isSuggestPrincipalPointEnabled(), 
                PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        assertNull(estimator.getSuggestedPrincipalPointValue());
        assertEquals(estimator.isSuggestRotationEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        assertNull(estimator.getSuggestedRotationValue());
        assertEquals(estimator.isSuggestCenterEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        assertNull(estimator.getSuggestedCenterValue());
        assertEquals(estimator.getMinSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getSuggestionWeightStep(),
                PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getType(), 
                PinholeCameraEstimatorType.DLT_POINT_PINHOLE_CAMERA_ESTIMATOR);
    }
}
