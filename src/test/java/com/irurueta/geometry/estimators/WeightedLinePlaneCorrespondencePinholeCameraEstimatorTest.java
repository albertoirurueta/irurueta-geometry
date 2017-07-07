/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.WeightedLinePlaneCorrespondencePinholeCameraEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 3, 2013
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Line2D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class WeightedLinePlaneCorrespondencePinholeCameraEstimatorTest 
    implements PinholeCameraEstimatorListener {
    
    public static final double ABSOLUTE_ERROR = 1e-5;
    public static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    public static final double VERY_LARGE_ABSOLUTE_ERROR = 2.0;
    
    public static final double MIN_RANDOM_VALUE = 0.0;
    public static final double MAX_RANDOM_VALUE = 1.0;
    
    public static final double MIN_FOCAL_LENGTH = 110.0;
    public static final double MAX_FOCAL_LENGTH = 130.0;
    
    public static final double MIN_SKEWNESS = -0.001;
    public static final double MAX_SKEWNESS = 0.001;
    
    public static final double MIN_PRINCIPAL_POINT = 90.0;
    public static final double MAX_PRINCIPAL_POINT = 100.0;
    
    public static final double MIN_ANGLE_DEGREES = 10.0;
    public static final double MAX_ANGLE_DEGREES = 15.0;

    public static final int INHOM_3D_COORDS = 3;
    public static final int LINE_PARAMS = 3;
    public static final int PLANE_PARAMS = 4;
    
    public static final int N_CORRESPONDENCES = 4;
    public static final int MIN_NUMBER_CORRESPONDENCES = 5;
    public static final int MAX_NUMBER_CORRESPONDENCES = 100;
    
    public static final int TIMES = 10;
    
    public static final double MIN_WEIGHT_VALUE = 0.75;
    public static final double MAX_WEIGHT_VALUE = 1.0;
    
    public static final double ERROR_STD = 1e-5;
    
    private int startCount = 0;
    private int endCount = 0;
    private int progressCount = 0;
    
    public WeightedLinePlaneCorrespondencePinholeCameraEstimatorTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructor() throws WrongListSizesException, 
        NotAvailableException {
        
        //testing constructor without parameters
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();
        
        //check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.
                WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getMaxCorrespondences(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_MAX_CORRESPONDENCES);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_SORT_WEIGHTS);
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        try {
            estimator.getLines2D();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        try {
            estimator.getPlanes();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        try {
            estimator.getWeights();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
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
        
        
        //testing constructor with listener
        estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                this);
        
        //check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.
                WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getMaxCorrespondences(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_MAX_CORRESPONDENCES);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_SORT_WEIGHTS);
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        try {
            estimator.getLines2D();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        try {
            estimator.getPlanes();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        try {
            estimator.getWeights();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        assertEquals(estimator.getListener(), this);
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
        
        //testing constructor with lists
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        List<Plane> planes = new ArrayList<Plane>(N_CORRESPONDENCES);
        List<Line2D> lines2D = new ArrayList<Line2D>(N_CORRESPONDENCES);
        for (int i = 0; i < N_CORRESPONDENCES; i++) {
            planes.add(new Plane(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            lines2D.add(new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
        }
        
        estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, lines2D);
        //check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.
                WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getMaxCorrespondences(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_MAX_CORRESPONDENCES);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_SORT_WEIGHTS);
        assertTrue(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getLines2D(), lines2D);
        assertEquals(estimator.getPlanes(), planes);
        try {
            estimator.getWeights();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
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
        
        //Force WrongListSizesException
        List<Plane> wrongPlanes = new ArrayList<Plane>();
        List<Line2D> wrongLines2D = new ArrayList<Line2D>();
        estimator = null;
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    wrongPlanes, lines2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, wrongLines2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    null, lines2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //testing constructor with lists and listener
        estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, lines2D, this);
        //check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.
                WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getMaxCorrespondences(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_MAX_CORRESPONDENCES);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_SORT_WEIGHTS);
        assertTrue(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertEquals(estimator.getLines2D(), lines2D);
        assertEquals(estimator.getPlanes(), planes);
        try {
            estimator.getWeights();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        assertEquals(estimator.getListener(), this);
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
        
        //Force WrongListSizesException
        wrongPlanes = new ArrayList<Plane>();
        wrongLines2D = new ArrayList<Line2D>();
        estimator = null;
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    wrongPlanes, lines2D, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, wrongLines2D, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    null, lines2D, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        //testing constructor with lists and weights
        double[] weights = new double[N_CORRESPONDENCES];
        randomizer.fill(weights, MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);
        
        estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, lines2D, weights);
        //check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.
                WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getMaxCorrespondences(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_MAX_CORRESPONDENCES);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_SORT_WEIGHTS);
        assertEquals(estimator.getPlanes(), planes);
        assertEquals(estimator.getLines2D(), lines2D);
        assertArrayEquals(estimator.getWeights(), weights, 0.0);
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
        
        //Force WrongListSizesException
        double[] wrongWeights = new double[1];
        estimator = null;
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    wrongPlanes, lines2D, weights);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, wrongLines2D, weights);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, wrongWeights);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    null, lines2D, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, null, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, (double[])null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        
        //testing constructor with lists, weights and listener
        estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                planes, lines2D, weights, this);
        //check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.
                WEIGHTED_LINE_PLANE_PINHOLE_CAMERA_ESTIMATOR);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getMaxCorrespondences(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_MAX_CORRESPONDENCES);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.getMaxCorrespondences(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_MAX_CORRESPONDENCES);
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_SORT_WEIGHTS);
        assertEquals(estimator.getLines2D(), lines2D);
        assertEquals(estimator.getPlanes(), planes);
        assertArrayEquals(estimator.getWeights(), weights, 0.0);
        assertEquals(estimator.getListener(), this);
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
        
        //Force WrongListSizesException
        estimator = null;
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    wrongPlanes, lines2D, weights, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, wrongLines2D, weights, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, wrongWeights, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    null, lines2D, weights, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, null, weights, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
    }    
    
    @Test
    public void testGetSetListsValidityAndAvailability() throws LockedException,
        WrongListSizesException, NotAvailableException {
        
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();
        
        //check that lists are not available
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.isReady());
        try {
            estimator.getLines2D();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        try {
            estimator.getPlanes();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        List<Plane> planes = new ArrayList<Plane>(N_CORRESPONDENCES);
        List<Line2D> lines2D = new ArrayList<Line2D>(N_CORRESPONDENCES);
        for (int i = 0; i < N_CORRESPONDENCES; i++) {
            planes.add(new Plane(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            lines2D.add(new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
        }
        
        //set lists
        assertTrue(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                areValidLists(planes, lines2D));
        
        estimator.setLists(planes, lines2D);
        
        //check correctness
        assertEquals(planes, estimator.getPlanes());
        assertEquals(lines2D, estimator.getLines2D());
        assertTrue(estimator.areListsAvailable());
        
        //Force WrongListSizesException
        List<Plane> wrongPlanes = new ArrayList<Plane>();
        List<Line2D> wrongLines = new ArrayList<Line2D>();
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                areValidLists(wrongPlanes, lines2D));
        try {
            estimator.setLists(wrongPlanes, lines2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                areValidLists(planes, wrongLines));
        try {
            estimator.setLists(planes, wrongLines);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                areValidLists(null, lines2D));
        try {
            estimator.setLists(null, lines2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertFalse(DLTLinePlaneCorrespondencePinholeCameraEstimator.
                areValidLists(planes, null));
        try {
            estimator.setLists(planes, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }    

    @Test
    public void testGetSetListsAndWeightsValidityAndAvailability() 
            throws LockedException, IllegalArgumentException, 
            WrongListSizesException, NotAvailableException {
        
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();
        
        //check that lists are not available
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.areWeightsAvailable());
        assertFalse(estimator.isReady());
        try {
            estimator.getLines2D();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        try {
            estimator.getPlanes();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        try {
            estimator.getWeights();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        List<Plane> planes = new ArrayList<Plane>(N_CORRESPONDENCES);
        List<Line2D> lines2D = new ArrayList<Line2D>(N_CORRESPONDENCES);        
        double[] weights = new double[N_CORRESPONDENCES];
        for (int i = 0; i < N_CORRESPONDENCES; i++) {
            planes.add(new Plane(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            lines2D.add(new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
        }
        randomizer.fill(weights, MIN_WEIGHT_VALUE, MAX_WEIGHT_VALUE);

        //set lists
        assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                areValidListsAndWeights(planes, lines2D, weights));
        
        estimator.setListsAndWeights(planes, lines2D, weights);        
        
        //check correctness
        assertEquals(planes, estimator.getPlanes());
        assertEquals(lines2D, estimator.getLines2D());
        assertArrayEquals(weights, estimator.getWeights(), 0.0);
        assertTrue(estimator.areListsAvailable());
        assertTrue(estimator.areWeightsAvailable());
        
        //Force WrongListSizesException
        List<Plane> wrongPlanes = new ArrayList<Plane>();
        List<Line2D> wrongLines2D = new ArrayList<Line2D>();
        double[] wrongWeights = new double[1];
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                areValidListsAndWeights(wrongPlanes, lines2D, weights));
        try {
            estimator.setListsAndWeights(wrongPlanes, lines2D, weights);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                areValidListsAndWeights(planes, wrongLines2D, weights));
        try {
            estimator.setListsAndWeights(planes, wrongLines2D, weights);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                areValidListsAndWeights(planes, lines2D, wrongWeights));
        try {
            estimator.setListsAndWeights(planes, lines2D, wrongWeights);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                areValidListsAndWeights(null, lines2D, weights));
        try {
            estimator.setListsAndWeights(null, lines2D, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                areValidListsAndWeights(planes, null, weights));
        try {
            estimator.setListsAndWeights(planes, null, weights);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertFalse(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                areValidListsAndWeights(planes, lines2D, null));
        try {
            estimator.setListsAndWeights(planes, lines2D, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }      
    
    @Test
    public void testGetSetListener() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();
        
        assertNull(estimator.getListener());
        
        //set listener
        estimator.setListener(this);
        
        //check correctness
        assertEquals(estimator.getListener(), this);
    }   
    
    @Test
    public void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        
        //set new value
        estimator.setSuggestSkewnessValueEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestSkewnessValueEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_SKEWNESS_VALUE_ENABLED);
    }
    
    @Test
    public void testGetSetSuggestedSkewnessValue() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.getSuggestedSkewnessValue(),
                PinholeCameraEstimator.DEFAULT_SUGGESTED_SKEWNESS_VALUE, 0.0);
        
        //set new value
        estimator.setSuggestedSkewnessValue(1e-3);
        
        //check correctness
        assertEquals(estimator.getSuggestedSkewnessValue(), 1e-3, 0.0);
    }    
    
    @Test
    public void testIsSetSuggestHorizontalFocalLengthEnabled() 
            throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        
        //set new value
        estimator.setSuggestHorizontalFocalLengthEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestHorizontalFocalLengthEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_HORIZONTAL_FOCAL_LENGTH_ENABLED);
    }    
    
    @Test
    public void testGetSetSuggestedHorizontalFocalLengthValue() 
            throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 0.0, 
                0.0);
        
        //set new value
        estimator.setSuggestedHorizontalFocalLengthValue(100.0);
        
        //check correctness
        assertEquals(estimator.getSuggestedHorizontalFocalLengthValue(), 100.0, 
                0.0);
    }        
    
    @Test
    public void testIsSetSuggestedVerticalFocalLengthEnabled() 
            throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        
        //set new value
        estimator.setSuggestVerticalFocalLengthEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestVerticalFocalLengthEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_VERTICAL_FOCAL_LENGTH_ENABLED);
    }    
    
    @Test
    public void testGetSetSuggestedVerticalFocalLengthValue() 
            throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 0.0, 
                0.0);
        
        //set new value
        estimator.setSuggestedVerticalFocalLengthValue(100.0);
        
        //check correctness
        assertEquals(estimator.getSuggestedVerticalFocalLengthValue(), 100.0, 
                0.0);
    }       
    
    @Test
    public void testIsSetSuggestAspectRatioEnabled() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        
        //set new value
        estimator.setSuggestAspectRatioEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestAspectRatioEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_ASPECT_RATIO_ENABLED);        
    }    
    
    @Test
    public void testGetSetSuggestedAspectRatioValue() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.getSuggestedAspectRatioValue(), 
                PinholeCameraEstimator.DEFAULT_SUGGESTED_ASPECT_RATIO_VALUE, 
                0.0);
        
        //set new value
        estimator.setSuggestedAspectRatioValue(-1.0);
        
        //check correctness
        assertEquals(estimator.getSuggestedAspectRatioValue(), -1.0, 0.0);
    }    
    
    @Test
    public void testIsSetSuggestPrincipalPointEnabled() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        
        //set new value
        estimator.setSuggestPrincipalPointEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestPrincipalPointEnabled(),
                !PinholeCameraEstimator.DEFAULT_SUGGEST_PRINCIPAL_POINT_ENABLED);        
    }   
    
    @Test
    public void testGetSetSuggestedPrincipalPointValue() 
            throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertNull(estimator.getSuggestedPrincipalPointValue());
        
        //set new value
        InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();
        estimator.setSuggestedPrincipalPointValue(principalPoint);
        
        //check correctness
        assertSame(estimator.getSuggestedPrincipalPointValue(), principalPoint);
    }    
    
    @Test
    public void testIsSetSuggestRotationEnabled() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.isSuggestRotationEnabled(), 
                PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        
        //set new value
        estimator.setSuggestRotationEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestRotationEnabled(), 
                !PinholeCameraEstimator.DEFAULT_SUGGEST_ROTATION_ENABLED);        
    }    
    
    @Test
    public void testGetSetSuggestedRotationValue() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertNull(estimator.getSuggestedRotationValue());
        
        //set new value
        Quaternion q = new Quaternion();
        estimator.setSuggestedRotationValue(q);
        
        //check correctness
        assertSame(estimator.getSuggestedRotationValue(), q);
    }    
    
    @Test
    public void testIsSetSuggestCenterEnabled() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.isSuggestCenterEnabled(), 
                PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        
        //set new value
        estimator.setSuggestCenterEnabled(
                !PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);
        
        //check correctness
        assertEquals(estimator.isSuggestCenterEnabled(), 
                !PinholeCameraEstimator.DEFAULT_SUGGEST_CENTER_ENABLED);        
    }    
    
    @Test
    public void testGetSetSuggestedCenterValue() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertNull(estimator.getSuggestedCenterValue());
        
        //set new value
        InhomogeneousPoint3D center = new InhomogeneousPoint3D();
        estimator.setSuggestedCenterValue(center);
        
        //check correctness
        assertSame(estimator.getSuggestedCenterValue(), center);
    }        
    
    @Test
    public void testGetSetMinSuggestionWeight() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.getMinSuggestionWeight(), 
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        
        //set new value
        estimator.setMinSuggestionWeight(1.0);
        
        //check correctness
        assertEquals(estimator.getMinSuggestionWeight(), 1.0, 0.0);
    }    
    
    @Test
    public void testGetSetMaxSuggestionWeight() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        
        //set new value
        estimator.setMaxSuggestionWeight(1.0);
        
        //check correctness
        assertEquals(estimator.getMaxSuggestionWeight(), 1.0, 0.0);
    }        
    
    @Test
    public void testSetMinMaxSuggestionWeight() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.getMinSuggestionWeight(), 
                PinholeCameraEstimator.DEFAULT_MIN_SUGGESTION_WEIGHT, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(),
                PinholeCameraEstimator.DEFAULT_MAX_SUGGESTION_WEIGHT, 0.0);
        
        //set new value
        estimator.setMinMaxSuggestionWeight(10.0, 20.0);
        
        //check correctness
        assertEquals(estimator.getMinSuggestionWeight(), 10.0, 0.0);
        assertEquals(estimator.getMaxSuggestionWeight(), 20.0, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setMinMaxSuggestionWeight(10.0, 10.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }    
    
    @Test
    public void testGetSetSuggestionWeightStep() throws LockedException {
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();

        //check default value
        assertEquals(estimator.getSuggestionWeightStep(), 
                PinholeCameraEstimator.DEFAULT_SUGGESTION_WEIGHT_STEP, 0.0);
        
        //set new value
        estimator.setSuggestionWeightStep(1.0);
        
        //check correctness
        assertEquals(estimator.getSuggestionWeightStep(), 1.0, 0.0);
        
        //Force IllegalArgumentException
        try {
            estimator.setSuggestionWeightStep(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }        
    
    @Test
    public void testGetSetMaxCorrespondences() throws LockedException{
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();
        
        assertEquals(estimator.getMaxCorrespondences(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_MAX_CORRESPONDENCES);
        
        //set new value
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        int maxCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, 
                MAX_NUMBER_CORRESPONDENCES);
        estimator.setMaxCorrespondences(maxCorrespondences);
        
        //check correctness
        assertEquals(estimator.getMaxCorrespondences(), maxCorrespondences);
    }    
    
    @Test
    public void testIsSetSortWeightsEnabled() throws LockedException{
        WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator =
                new WeightedLinePlaneCorrespondencePinholeCameraEstimator();
        
        assertEquals(estimator.isSortWeightsEnabled(),
                WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                DEFAULT_SORT_WEIGHTS);
        
        //disable
        estimator.setSortWeightsEnabled(false);
        
        //check correctness
        assertFalse(estimator.isSortWeightsEnabled());
        
        //enable
        estimator.setSortWeightsEnabled(true);
        
        //check correctness
        assertTrue(estimator.isSortWeightsEnabled());
    }
    
    @Test
    public void testEstimateNoSuggestion() throws WrongListSizesException, LockedException, 
        NotReadyException, PinholeCameraEstimatorException, 
        NotAvailableException, CameraException {
        
       boolean passedAtLeastOnce = false; //to account for random degeneracies
       for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);

            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);

            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();   
            
            //test the case where there are exactly 4 correspondences
            int nCorrespondences = N_CORRESPONDENCES;
            List<Line2D> lines2D = new ArrayList<Line2D>(nCorrespondences);
            double[] weights = new double[nCorrespondences];
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines2D.add(line2D);
                
                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                        MAX_WEIGHT_VALUE);                
            }

            List<Plane> planes = camera.backProjectLines(lines2D);

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidListsAndWeights(planes, lines2D, weights));

            WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, weights, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            List<Plane> estimatedPlanes;
            Iterator<Plane> planesIt;
            Iterator<Plane> estimatedPlanesIt;
            Plane plane, estimatedPlane;
            PinholeCameraIntrinsicParameters estimatedIntrinsic;
            Rotation3D estimatedRotation;
            MatrixRotation3D estimatedRotation2;
            double estimatedAlphaEuler, estimatedBetaEuler, estimatedGammaEuler;
            boolean validAlphaEuler, validBetaEuler, validGammaEuler;
            Point3D estimatedCameraCenter;
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();

                assertFalse(estimator.isLocked());        
                assertEquals(startCount, 1);
                assertEquals(endCount, 1);
                assertEquals(progressCount, 0);
                
                assertNotNull(estimatedCamera);

                //project original points using estimated camera
                estimatedPlanes = estimatedCamera.backProjectLines(lines2D);

                //check that planes and estimated planes are equal
                planesIt = planes.iterator();
                estimatedPlanesIt = estimatedPlanes.iterator();
            
                while (planesIt.hasNext() && estimatedPlanesIt.hasNext()) {
                    plane = planesIt.next();
                    estimatedPlane = estimatedPlanesIt.next();

                    assertTrue(plane.equals(estimatedPlane, ABSOLUTE_ERROR));
                }

                //decompose estimated camera and check its parameters
                estimatedCamera.decompose();
            


                //Comparing camera intrinsic parameters
                estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

                if (Math.abs(horizontalFocalLength - estimatedIntrinsic.getHorizontalFocalLength()) > 
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(verticalFocalLength - estimatedIntrinsic.getVerticalFocalLength()) > 
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(horizontalPrincipalPoint - estimatedIntrinsic.getHorizontalPrincipalPoint()) > 
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(verticalPrincipalPoint - estimatedIntrinsic.getVerticalPrincipalPoint()) > 
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    VERY_LARGE_ABSOLUTE_ERROR);
                if (Math.abs(skewness - estimatedIntrinsic.getSkewness()) > 
                        VERY_LARGE_ABSOLUTE_ERROR) {
                    continue;
                }
                assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    VERY_LARGE_ABSOLUTE_ERROR);

            
                //Comparing estimated rotation
                estimatedRotation = estimatedCamera.getCameraRotation();
            
                estimatedRotation2 = (MatrixRotation3D)estimatedRotation;
                estimatedAlphaEuler = estimatedRotation2.getAlphaEulerAngle();
                estimatedBetaEuler = estimatedRotation2.getBetaEulerAngle();
                estimatedGammaEuler = estimatedRotation2.getGammaEulerAngle();
            
                if (Math.abs(alphaEuler - estimatedAlphaEuler) <= 
                        LARGE_ABSOLUTE_ERROR) {
                    validAlphaEuler = true;
                } else if ((Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) - 
                        Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                    validAlphaEuler = true;
                } else {
                    validAlphaEuler = false;
                }

                if (Math.abs(betaEuler - estimatedBetaEuler) <= 
                        LARGE_ABSOLUTE_ERROR) {
                    validBetaEuler = true;
                } else if ((Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) - 
                        Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                    validBetaEuler = true;
                } else {
                    validBetaEuler = false;
                }

                if (Math.abs(gammaEuler - estimatedGammaEuler) <= 
                        LARGE_ABSOLUTE_ERROR) {
                    validGammaEuler = true;
                } else if ((Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) - 
                        Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                    validGammaEuler = true;
                } else {
                    validGammaEuler = false;
                }
            
            
                assertTrue(validAlphaEuler);
                assertTrue(validBetaEuler);
                assertTrue(validGammaEuler);
            
            
                //comparing estimated camera center
                estimatedCameraCenter = estimatedCamera.getCameraCenter();
                assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    VERY_LARGE_ABSOLUTE_ERROR));
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            
            
            //Testing the case where there are more than four correspondences
            nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, 
                    MAX_NUMBER_CORRESPONDENCES);
            lines2D.clear();
            weights = new double[nCorrespondences];
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines2D.add(line2D);
                
                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                        MAX_WEIGHT_VALUE);
            }

            planes = camera.backProjectLines(lines2D);

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidListsAndWeights(planes, lines2D, weights));

            estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, weights, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            try {
                estimatedCamera = estimator.estimate();

                assertFalse(estimator.isLocked());        
                assertEquals(startCount, 1);
                assertEquals(endCount, 1);
                assertEquals(progressCount, 0);
                
                assertNotNull(estimatedCamera);
            
                //project original points using estimated camera
                estimatedPlanes = estimatedCamera.backProjectLines(lines2D);

                //check that points2D and estimated points2D are equal
                planesIt = planes.iterator();
                estimatedPlanesIt = estimatedPlanes.iterator();

                while (planesIt.hasNext() && estimatedPlanesIt.hasNext()) {
                    plane = planesIt.next();
                    estimatedPlane = estimatedPlanesIt.next();

                    assertTrue(plane.equals(estimatedPlane, ABSOLUTE_ERROR));
                }

                //decompose estimated camera and check its parameters
                estimatedCamera.decompose();

                //Comparing camera intrinsic parameters
                estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

                assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    VERY_LARGE_ABSOLUTE_ERROR);
                assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    VERY_LARGE_ABSOLUTE_ERROR);

            
                //Comparing estimated rotation
                estimatedRotation = estimatedCamera.getCameraRotation();
            
                estimatedRotation2 = (MatrixRotation3D)estimatedRotation;
                estimatedAlphaEuler = estimatedRotation2.getAlphaEulerAngle();
                estimatedBetaEuler = estimatedRotation2.getBetaEulerAngle();
                estimatedGammaEuler = estimatedRotation2.getGammaEulerAngle();
            
                if (Math.abs(alphaEuler - estimatedAlphaEuler) <= 
                        LARGE_ABSOLUTE_ERROR) {
                    validAlphaEuler = true;
                } else if ((Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) - 
                        Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                    validAlphaEuler = true;
                } else {
                    validAlphaEuler = false;
                }

                if (Math.abs(betaEuler - estimatedBetaEuler) <= 
                        LARGE_ABSOLUTE_ERROR) {
                    validBetaEuler = true;
                } else if ((Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) - 
                        Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                    validBetaEuler = true;
                } else {
                    validBetaEuler = false;
                }

                if (Math.abs(gammaEuler - estimatedGammaEuler) <= 
                        LARGE_ABSOLUTE_ERROR) {
                    validGammaEuler = true;
                } else if ((Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) - 
                        Math.PI) <= LARGE_ABSOLUTE_ERROR) {
                    validGammaEuler = true;
                } else {
                    validGammaEuler = false;
                }
            
            
                assertTrue(validAlphaEuler);
                assertTrue(validBetaEuler);
                assertTrue(validGammaEuler);
            
        
                //comparing estimated camera center
                estimatedCameraCenter = estimatedCamera.getCameraCenter();
                assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    VERY_LARGE_ABSOLUTE_ERROR));
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            //Force PinholeCameraEstimatorException
            nCorrespondences = N_CORRESPONDENCES;
            lines2D.clear();
            weights = new double[nCorrespondences];
            for (int i = 0; i < nCorrespondences - 2; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                lines2D.add(line2D);
                
                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                        MAX_WEIGHT_VALUE);
            }
            line2D = new Line2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE));                        
            lines2D.add(line2D);
            lines2D.add(line2D);

            planes = camera.backProjectLines(lines2D);            
            
            estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, weights, this);

            reset();
            estimatedCamera = null;
            try {
                estimatedCamera = estimator.estimate();
                fail("PinholeCameraEstimatorException expected but not thrown");
            } catch (PinholeCameraEstimatorException e) { }
            
            //Force NotReadyException
            estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator();
            try {
                estimatedCamera = estimator.estimate();
                fail("NotReadyException expected but not thrown");
            } catch (NotReadyException e) { }
            assertNull(estimatedCamera);
            
            passedAtLeastOnce = true;
            break;
        }               
        assertTrue(passedAtLeastOnce);
    }    

    @Test
    public void testEstimateSuggestedSkewness() throws WrongListSizesException, 
            LockedException, NotReadyException, PinholeCameraEstimatorException, 
            NotAvailableException, CameraException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);

            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);

            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();   
            
            //Testing the case where there are more than six correspondences and
            //the LMSE solution is allowed
            int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, 
                    MAX_NUMBER_CORRESPONDENCES);
            List<Line2D> lines2D = new ArrayList<Line2D>(nCorrespondences);
            double[] weights = new double[nCorrespondences];
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                line2D.normalize();
                lines2D.add(line2D);
                
                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                        MAX_WEIGHT_VALUE);
            }

            List<Plane> planes = camera.backProjectLines(lines2D);
            
            //add error to lines
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (Line2D line : lines2D) {
                double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), 
                        line.getC() + errorC);
            }                        

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidListsAndWeights(planes, lines2D, weights));

            WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, weights, this);
            
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestedSkewnessValue(skewness);            

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            
            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            
            assertFalse(estimator.isLocked());
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);
            
            assertNotNull(estimatedCamera);
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            double estimatedSkewness = estimatedIntrinsic.getSkewness();
            
            //estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }         

            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedSkewnessNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getSkewness();
            
            //check that skewness has become closer to suggested value
            if (Math.abs(skewness - estimatedSkewnessNoSuggestion) > 
                    Math.abs(skewness - estimatedSkewness)) {            
                passedAtLeastOnce = true;
            }           
            
            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }    

    @Test
    public void testEstimateSuggestedHorizontalFocalLengthEnabled() 
            throws WrongListSizesException, LockedException, NotReadyException, 
            PinholeCameraEstimatorException, NotAvailableException, 
            CameraException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);

            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);

            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();   
            
            //Testing the case where there are more than six correspondences and
            //the LMSE solution is allowed
            int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, 
                    MAX_NUMBER_CORRESPONDENCES);
            List<Line2D> lines2D = new ArrayList<Line2D>(nCorrespondences);
            double[] weights = new double[nCorrespondences];
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                line2D.normalize();
                lines2D.add(line2D);
                
                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                        MAX_WEIGHT_VALUE);                
            }

            List<Plane> planes = camera.backProjectLines(lines2D);
            
            //add error to lines
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (Line2D line : lines2D) {
                double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), 
                        line.getC() + errorC);
            }                        

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidListsAndWeights(planes, lines2D, weights));

            WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, weights, this);
            
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            estimator.setSuggestedHorizontalFocalLengthValue(
                    horizontalFocalLength);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            
            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            
            assertFalse(estimator.isLocked());
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);
            
            assertNotNull(estimatedCamera);
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            double estimatedHorizontalFocalLength = 
                    estimatedIntrinsic.getHorizontalFocalLength();
            
            //estimate without suggestion
            estimator.setSuggestHorizontalFocalLengthEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }         

            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedHorizontalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getHorizontalFocalLength();
            
            //check that horizontal focal length has become closer to suggested 
            //value
            if (Math.abs(horizontalFocalLength - estimatedHorizontalFocalLengthNoSuggestion) >
                    Math.abs(horizontalFocalLength - estimatedHorizontalFocalLength)) {
                passedAtLeastOnce = true;
            }
            
            if (passedAtLeastOnce) {
                break;
            }            
        }
        assertTrue(passedAtLeastOnce);
    }    

    @Test
    public void testEstimateSuggestedVerticalFocalLengthEnabled() 
            throws WrongListSizesException, LockedException, NotReadyException, 
            PinholeCameraEstimatorException, NotAvailableException, 
            CameraException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);

            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);

            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();   
            
            //Testing the case where there are more than six correspondences and
            //the LMSE solution is allowed
            int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, 
                    MAX_NUMBER_CORRESPONDENCES);
            List<Line2D> lines2D = new ArrayList<Line2D>(nCorrespondences);
            double[] weights = new double[nCorrespondences];
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                line2D.normalize();
                lines2D.add(line2D);
                
                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                        MAX_WEIGHT_VALUE);                
            }

            List<Plane> planes = camera.backProjectLines(lines2D);
            
            //add error to lines
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (Line2D line : lines2D) {
                double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), 
                        line.getC() + errorC);
            }                        

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidListsAndWeights(planes, lines2D, weights));

            WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, weights, this);
            
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            estimator.setSuggestedVerticalFocalLengthValue(
                    verticalFocalLength);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            
            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            
            assertFalse(estimator.isLocked());
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);
            
            assertNotNull(estimatedCamera);
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            double estimatedVerticalFocalLength = 
                    estimatedIntrinsic.getVerticalFocalLength();
            
            //estimate without suggestion
            estimator.setSuggestVerticalFocalLengthEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }         

            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedVerticalFocalLengthNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getVerticalFocalLength();
            
            //check that horizontal focal length has become closer to suggested 
            //value
            if (Math.abs(horizontalFocalLength - estimatedVerticalFocalLengthNoSuggestion) >
                    Math.abs(horizontalFocalLength - estimatedVerticalFocalLength)) {
                passedAtLeastOnce = true;
            }
            
            if (passedAtLeastOnce) {
                break;
            }            
        }
        assertTrue(passedAtLeastOnce);
    }    

    @Test
    public void testEstimateSuggestedAspectRatioEnabled() 
            throws WrongListSizesException, LockedException, NotReadyException, 
            PinholeCameraEstimatorException, NotAvailableException, 
            CameraException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            double aspectRatio = intrinsic.getAspectRatio();

            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);

            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();   
            
            //Testing the case where there are more than six correspondences and
            //the LMSE solution is allowed
            int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, 
                    MAX_NUMBER_CORRESPONDENCES);
            List<Line2D> lines2D = new ArrayList<Line2D>(nCorrespondences);
            double[] weights = new double[nCorrespondences];
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                line2D.normalize();
                lines2D.add(line2D);
                
                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                        MAX_WEIGHT_VALUE);                
            }

            List<Plane> planes = camera.backProjectLines(lines2D);
            
            //add error to lines
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (Line2D line : lines2D) {
                double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), 
                        line.getC() + errorC);
            }                        

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidListsAndWeights(planes, lines2D, weights));

            WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, weights, this);
            
            estimator.setSuggestAspectRatioEnabled(true);
            estimator.setSuggestedAspectRatioValue(aspectRatio);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            
            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            
            assertFalse(estimator.isLocked());
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);
            
            assertNotNull(estimatedCamera);
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();
            
            //estimate without suggestion
            estimator.setSuggestAspectRatioEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }         

            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedAspectRatioNoSuggestion =
                    estimatedIntrinsicNoSuggestion.getAspectRatio();
            
            //check that aspect ratio has become closer to suggested
            //value
            if (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >
                    Math.abs(aspectRatio - estimatedAspectRatio)) {
                passedAtLeastOnce = true;
            }
            
            if (passedAtLeastOnce) {
                break;
            }            
        }
        assertTrue(passedAtLeastOnce);
    }    

    @Test
    public void testEstimateSuggestedPrincipalPointEnabled() 
            throws WrongListSizesException, LockedException, NotReadyException, 
            PinholeCameraEstimatorException, NotAvailableException, 
            CameraException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);            

            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);

            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();   
            
            //Testing the case where there are more than six correspondences and
            //the LMSE solution is allowed
            int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, 
                    MAX_NUMBER_CORRESPONDENCES);
            List<Line2D> lines2D = new ArrayList<Line2D>(nCorrespondences);
            double[] weights = new double[nCorrespondences];
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                line2D.normalize();
                lines2D.add(line2D);
                
                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                        MAX_WEIGHT_VALUE);                
            }

            List<Plane> planes = camera.backProjectLines(lines2D);
            
            //add error to lines
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (Line2D line : lines2D) {
                double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), 
                        line.getC() + errorC);
            }                        

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidListsAndWeights(planes, lines2D, weights));

            WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, weights, this);
            
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestedPrincipalPointValue(new InhomogeneousPoint2D(
                    horizontalPrincipalPoint, verticalPrincipalPoint));

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            
            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            
            assertFalse(estimator.isLocked());
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);
            
            assertNotNull(estimatedCamera);
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();
            InhomogeneousPoint2D estimatedPrincipalPoint = 
                    new InhomogeneousPoint2D(
                            estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsic.getVerticalPrincipalPoint());
            
            //estimate without suggestion
            estimator.setSuggestPrincipalPointEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }         

            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            InhomogeneousPoint2D estimatedPrincipalPointNoSuggestion =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());
            
            //check that principal point has become closer to suggested value
            if (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >
                    principalPoint.distanceTo(estimatedPrincipalPoint)) {
                passedAtLeastOnce = true;
            }
            
            if (passedAtLeastOnce) {
                break;
            }            
        }
        assertTrue(passedAtLeastOnce);
    }    

    @Test
    public void testEstimateSuggestedRotationEnabled() 
            throws WrongListSizesException, LockedException, NotReadyException, 
            PinholeCameraEstimatorException, NotAvailableException, 
            CameraException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);            

            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);
            Quaternion q = rotation.toQuaternion();
            q.normalize();                        

            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();   
            
            //Testing the case where there are more than six correspondences and
            //the LMSE solution is allowed
            int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, 
                    MAX_NUMBER_CORRESPONDENCES);
            List<Line2D> lines2D = new ArrayList<Line2D>(nCorrespondences);
            double[] weights = new double[nCorrespondences];
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                line2D.normalize();
                lines2D.add(line2D);
                
                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                        MAX_WEIGHT_VALUE);                
            }

            List<Plane> planes = camera.backProjectLines(lines2D);
            
            //add error to lines
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (Line2D line : lines2D) {
                double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), 
                        line.getC() + errorC);
            }                        

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidListsAndWeights(planes, lines2D, weights));

            WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, weights, this);
            
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(q);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            
            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            
            assertFalse(estimator.isLocked());
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);
            
            assertNotNull(estimatedCamera);
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing rotation
            Quaternion estimatedQ = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedQ.normalize();
            
            //estimate without suggestion
            estimator.setSuggestRotationEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }         

            estimatedCameraNoSuggestion.decompose();
            
            Quaternion estimatedQNoSuggestion = estimatedCameraNoSuggestion.
                    getCameraRotation().toQuaternion();
            
            //check that rotation has become closer to suggested value
            double diffEstimatedNoSuggestion = 
                    Math.pow(q.getA() - estimatedQNoSuggestion.getA(), 2.0) +
                    Math.pow(q.getB() - estimatedQNoSuggestion.getB(), 2.0) +
                    Math.pow(q.getC() - estimatedQNoSuggestion.getC(), 2.0) +
                    Math.pow(q.getD() - estimatedQNoSuggestion.getD(), 2.0);
            double diffEstimated =
                    Math.pow(q.getA() - estimatedQ.getA(), 2.0) +
                    Math.pow(q.getB() - estimatedQ.getB(), 2.0) +
                    Math.pow(q.getC() - estimatedQ.getC(), 2.0) +
                    Math.pow(q.getD() - estimatedQ.getD(), 2.0);
            
            if (diffEstimatedNoSuggestion > diffEstimated) {
                passedAtLeastOnce = true;
            }
            
            if (passedAtLeastOnce) {
                break;
            }            
        }
        assertTrue(passedAtLeastOnce);
    }    

    @Test
    public void testEstimateSuggestedCenterEnabled() 
            throws WrongListSizesException, LockedException, NotReadyException, 
            PinholeCameraEstimatorException, NotAvailableException, 
            CameraException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);            

            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);

            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();   
            
            //Testing the case where there are more than six correspondences and
            //the LMSE solution is allowed
            int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, 
                    MAX_NUMBER_CORRESPONDENCES);
            List<Line2D> lines2D = new ArrayList<Line2D>(nCorrespondences);
            double[] weights = new double[nCorrespondences];
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                line2D.normalize();
                lines2D.add(line2D);
                
                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                        MAX_WEIGHT_VALUE);                
            }

            List<Plane> planes = camera.backProjectLines(lines2D);
            
            //add error to lines
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (Line2D line : lines2D) {
                double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), 
                        line.getC() + errorC);
            }                        

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidListsAndWeights(planes, lines2D, weights));

            WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, weights, this);
            
            estimator.setSuggestCenterEnabled(true);
            estimator.setSuggestedCenterValue(cameraCenter);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            
            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            
            assertFalse(estimator.isLocked());
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);
            
            assertNotNull(estimatedCamera);
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing center
            Point3D estimatedCenter = estimatedCamera.getCameraCenter();
            
            //estimate without suggestion
            estimator.setSuggestCenterEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }         

            estimatedCameraNoSuggestion.decompose();
            
            Point3D estimatedCenterNoSuggestion = 
                    estimatedCameraNoSuggestion.getCameraCenter();
            
            //check that camera center has become closer to suggested value
            if (cameraCenter.distanceTo(estimatedCenterNoSuggestion) >
                    cameraCenter.distanceTo(estimatedCenter)) {
                passedAtLeastOnce = true;
            }
            
            if (passedAtLeastOnce) {
                break;
            }            
        }
        assertTrue(passedAtLeastOnce);
    }    

    @Test
    public void testEstimateZeroSkewnesssZeroPrincipalPointAndEqualFocalLength() 
            throws WrongListSizesException, LockedException, NotReadyException, 
            PinholeCameraEstimatorException, NotAvailableException, 
            CameraException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = horizontalFocalLength;
            
            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;
            InhomogeneousPoint2D principalPoint = new InhomogeneousPoint2D();

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(horizontalFocalLength,
                    verticalFocalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            double aspectRatio = intrinsic.getAspectRatio();

            //create rotation parameters
            double alphaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double betaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double gammaEuler = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;

            Rotation3D rotation = new MatrixRotation3D(alphaEuler, betaEuler, 
                    gammaEuler);

            //create camera center
            double[] cameraCenterArray = new double[INHOM_3D_COORDS];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);

            //normalize the camera to improve accuracy
            camera.normalize();   
            
            //Testing the case where there are more than six correspondences and
            //the LMSE solution is allowed
            int nCorrespondences = randomizer.nextInt(MIN_NUMBER_CORRESPONDENCES, 
                    MAX_NUMBER_CORRESPONDENCES);
            List<Line2D> lines2D = new ArrayList<Line2D>(nCorrespondences);
            double[] weights = new double[nCorrespondences];
            Line2D line2D;
            for (int i = 0; i < nCorrespondences; i++) {
                line2D = new Line2D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));                        
                line2D.normalize();
                lines2D.add(line2D);
                
                weights[i] = randomizer.nextDouble(MIN_WEIGHT_VALUE, 
                        MAX_WEIGHT_VALUE);                
            }

            List<Plane> planes = camera.backProjectLines(lines2D);
            
            //add error to lines
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            for (Line2D line : lines2D) {
                double errorC = errorRandomizer.nextDouble();
                line.setParameters(line.getA(), line.getB(), 
                        line.getC() + errorC);
            }                        

            assertTrue(WeightedLinePlaneCorrespondencePinholeCameraEstimator.
                    areValidListsAndWeights(planes, lines2D, weights));

            WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator = 
                    new WeightedLinePlaneCorrespondencePinholeCameraEstimator(
                    planes, lines2D, weights, this);
            
            estimator.setSuggestSkewnessValueEnabled(true);
            estimator.setSuggestPrincipalPointEnabled(true);
            estimator.setSuggestAspectRatioEnabled(true);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            
            reset();
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            
            assertFalse(estimator.isLocked());
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);
            
            assertNotNull(estimatedCamera);
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();
            
            //comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();    
            double estimatedSkewness = estimatedIntrinsic.getSkewness();
            InhomogeneousPoint2D estimatedPrincipalPoint = 
                    new InhomogeneousPoint2D(
                            estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                            estimatedIntrinsic.getVerticalPrincipalPoint());
            double estimatedAspectRatio = estimatedIntrinsic.getAspectRatio();
            
            //estimate without suggestion
            estimator.setSuggestSkewnessValueEnabled(false);
            estimator.setSuggestPrincipalPointEnabled(false);
            estimator.setSuggestAspectRatioEnabled(false);
            
            PinholeCamera estimatedCameraNoSuggestion;
            try {
                estimatedCameraNoSuggestion = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }         

            estimatedCameraNoSuggestion.decompose();
            
            PinholeCameraIntrinsicParameters estimatedIntrinsicNoSuggestion =
                    estimatedCameraNoSuggestion.getIntrinsicParameters();
            double estimatedSkewnessNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getSkewness();
            InhomogeneousPoint2D estimatedPrincipalPointNoSuggestion =
                    new InhomogeneousPoint2D(
                            estimatedIntrinsicNoSuggestion.getHorizontalPrincipalPoint(),
                            estimatedIntrinsicNoSuggestion.getVerticalPrincipalPoint());
            double estimatedAspectRatioNoSuggestion = 
                    estimatedIntrinsicNoSuggestion.getAspectRatio();
            
            //check that intrinsic values have become closer to suggested ones
            if ((Math.abs(skewness - estimatedSkewnessNoSuggestion) >
                    Math.abs(skewness - estimatedSkewness)) &&
                    (principalPoint.distanceTo(estimatedPrincipalPointNoSuggestion) >
                    principalPoint.distanceTo(estimatedPrincipalPoint)) &&
                    (Math.abs(aspectRatio - estimatedAspectRatioNoSuggestion) >
                    Math.abs(aspectRatio - estimatedAspectRatio))) {
                passedAtLeastOnce = true;
            }
            
            if (passedAtLeastOnce) {
                break;
            }            
        }
        assertTrue(passedAtLeastOnce);
    }    
    
    @Override
    public void onEstimateStart(PinholeCameraEstimator estimator) {
        startCount++;
        checkIsLocked((WeightedLinePlaneCorrespondencePinholeCameraEstimator)
                estimator);
    }

    @Override
    public void onEstimateEnd(PinholeCameraEstimator estimator) {
        endCount++;
        checkIsLocked((WeightedLinePlaneCorrespondencePinholeCameraEstimator)
                estimator);        
    }

    @Override
    public void onEstimationProgressChange(PinholeCameraEstimator estimator, float progress) {
        progressCount++;
        checkIsLocked((WeightedLinePlaneCorrespondencePinholeCameraEstimator)
                estimator);        
    }
    
    private void reset() {
        startCount = endCount = progressCount = 0;
    }
    
    private void checkIsLocked(
            WeightedLinePlaneCorrespondencePinholeCameraEstimator estimator) {
        assertTrue(estimator.isLocked());
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        
        try {
            estimator.setLists(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) {
        } catch (Throwable t) {
            fail("LockedException expected but not thrown");
        }
        
        try {
            estimator.setListsAndWeights(null, null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) {
        } catch (Throwable t) {
            fail("LockedException expected but not thrown");
        }
        
        try {
            estimator.setMaxCorrespondences(0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        
        try {
            estimator.setSortWeightsEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        
        try {
            estimator.setSuggestSkewnessValueEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestedSkewnessValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestHorizontalFocalLengthEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestedHorizontalFocalLengthValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestVerticalFocalLengthEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestedVerticalFocalLengthValue(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestAspectRatioEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestedAspectRatioValue(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestPrincipalPointEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestedPrincipalPointValue(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestRotationEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestedRotationValue(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestCenterEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestedCenterValue(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setMinSuggestionWeight(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setMaxSuggestionWeight(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestionWeightStep(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }        
    }
}