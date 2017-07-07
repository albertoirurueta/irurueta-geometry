/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.DLTPointCorrespondencePinholeCameraEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 19, 2013
 */
package com.irurueta.geometry.estimators;

import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.MatrixRotation3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.RotationException;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class DLTPointCorrespondencePinholeCameraEstimatorTest implements 
        PinholeCameraEstimatorListener {
    
    public static final double ABSOLUTE_ERROR = 1e-5;
    public static final double LARGE_ABSOLUTE_ERROR = 5e-1;
    
    public static final double MIN_RANDOM_VALUE = 50.0;
    public static final double MAX_RANDOM_VALUE = 100.0;
    
    public static final double MIN_FOCAL_LENGTH = 110.0;
    public static final double MAX_FOCAL_LENGTH = 130.0;
    
    public static final double MIN_SKEWNESS = -0.001;
    public static final double MAX_SKEWNESS = 0.001;
    
    public static final double MIN_PRINCIPAL_POINT = 90.0;
    public static final double MAX_PRINCIPAL_POINT = 100.0;
    
    public static final double MIN_ANGLE_DEGREES = 10.0;
    public static final double MAX_ANGLE_DEGREES = 15.0;
    
    public static final int INHOM_3D_COORDS = 3;
        
    public static final int MIN_NUMBER_POINTS = 10;
    public static final int MAX_NUMBER_POINTS = 100;
    
    public static final double MIN_DEPTH = 0.5;
    public static final double MAX_DEPTH = 100.0;
    
    public static final int N_POINTS = 6;
    public static final int MIN_POINTS = 7;
    public static final int MAX_POINTS = 100;
    
    public static final double MIN_RANDOM_ERROR = 0.99;
    public static final double MAX_RANDOM_ERROR = 1.01;
    
    public static final int TIMES = 100;
    
    public static final double ERROR_STD = 1e-5;
    
    private int startCount = 0;
    private int endCount = 0;
    private int progressCount = 0;
    
    public DLTPointCorrespondencePinholeCameraEstimatorTest() { }
    
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
        
        //testing contructor without parameters
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();
        
        //check correctness
        assertEquals(estimator.getType(), 
                PinholeCameraEstimatorType.DLT_POINT_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.isLMSESolutionAllowed(), 
                DLTPointCorrespondencePinholeCameraEstimator.
                DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(estimator.arePointCorrespondencesNormalized(),
                DLTPointCorrespondencePinholeCameraEstimator.
                DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);
        try {
            estimator.getPoints2D();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        try {
            estimator.getPoints3D();
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
        estimator = new DLTPointCorrespondencePinholeCameraEstimator(this);
        
        //check correctness
        assertEquals(estimator.getType(), 
                PinholeCameraEstimatorType.DLT_POINT_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.isLMSESolutionAllowed(), 
                DLTPointCorrespondencePinholeCameraEstimator.
                DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(estimator.arePointCorrespondencesNormalized(),
                DLTPointCorrespondencePinholeCameraEstimator.
                DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);
        try {
            estimator.getPoints2D();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        try {
            estimator.getPoints3D();
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
        List<Point3D> points3D = new ArrayList<Point3D>(N_POINTS);
        List<Point2D> points2D = new ArrayList<Point2D>(N_POINTS);        
        for (int i = 0; i < N_POINTS; i++) {
            points3D.add(new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            points2D.add(new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
        }
        
        estimator = new DLTPointCorrespondencePinholeCameraEstimator(points3D, 
                points2D);
        //check correctness
        assertEquals(estimator.getType(), 
                PinholeCameraEstimatorType.DLT_POINT_PINHOLE_CAMERA_ESTIMATOR);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.isLMSESolutionAllowed(), 
                DLTPointCorrespondencePinholeCameraEstimator.
                DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(estimator.arePointCorrespondencesNormalized(),
                DLTPointCorrespondencePinholeCameraEstimator.
                DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);
        assertEquals(estimator.getPoints2D(), points2D);
        assertEquals(estimator.getPoints3D(), points3D);
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
        List<Point3D> wrong3D = new ArrayList<Point3D>();        
        List<Point2D> wrong2D = new ArrayList<Point2D>();
        estimator = null;
        try {
            estimator = new DLTPointCorrespondencePinholeCameraEstimator(
                    wrong3D, points2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        try {
            estimator = new DLTPointCorrespondencePinholeCameraEstimator(
                    points3D, wrong2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        try {
            estimator = new DLTPointCorrespondencePinholeCameraEstimator(null,
                    points2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new DLTPointCorrespondencePinholeCameraEstimator(
                    points3D, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);
        
        
        //testing constructor with lists and listener
        estimator = new DLTPointCorrespondencePinholeCameraEstimator(points3D, 
                points2D, this);
        //check correctness
        assertEquals(estimator.getType(), 
                PinholeCameraEstimatorType.DLT_POINT_PINHOLE_CAMERA_ESTIMATOR);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertEquals(estimator.isLMSESolutionAllowed(), 
                DLTPointCorrespondencePinholeCameraEstimator.
                DEFAULT_ALLOW_LMSE_SOLUTION);
        assertEquals(estimator.arePointCorrespondencesNormalized(),
                DLTPointCorrespondencePinholeCameraEstimator.
                DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);
        assertEquals(estimator.getPoints2D(), points2D);
        assertEquals(estimator.getPoints3D(), points3D);
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
        
        //Force WrongListSizesExcpetion
        estimator = null;
        try {
            estimator = new DLTPointCorrespondencePinholeCameraEstimator(
                    wrong3D, points2D, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        try {
            estimator = new DLTPointCorrespondencePinholeCameraEstimator(
                    points3D, wrong2D, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        try {
            estimator = new DLTPointCorrespondencePinholeCameraEstimator(null,
                    points2D, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new DLTPointCorrespondencePinholeCameraEstimator(
                    points3D, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);        
    }
    
    @Test
    public void testIsSetLMSESolutionAllowed() throws LockedException {
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();
        
        assertEquals(estimator.isLMSESolutionAllowed(),
                DLTPointCorrespondencePinholeCameraEstimator.
                DEFAULT_ALLOW_LMSE_SOLUTION);
        
        //disable
        estimator.setLMSESolutionAllowed(false);
        
        //check correctness
        assertFalse(estimator.isLMSESolutionAllowed());
        
        //enable
        estimator.setLMSESolutionAllowed(true);
        
        //check correctness
        assertTrue(estimator.isLMSESolutionAllowed());
    }
    
    @Test
    public void testAreSetPointCorrespondencesNormalized() 
            throws LockedException {
        
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();
        
        assertEquals(estimator.arePointCorrespondencesNormalized(),
                DLTPointCorrespondencePinholeCameraEstimator.
                DEFAULT_NORMALIZE_POINT_CORRESPONDENCES);
        
        //disable
        estimator.setPointCorrespondencesNormalized(false);
        
        //check correctness
        assertFalse(estimator.arePointCorrespondencesNormalized());
        
        //enable
        estimator.setPointCorrespondencesNormalized(true);
        
        //check correctness
        assertTrue(estimator.arePointCorrespondencesNormalized());
    }
    
    @Test
    public void testGetSetListsValidityAndAvailability() throws LockedException, 
        IllegalArgumentException, WrongListSizesException, 
        NotAvailableException {
        
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();
        
        //check that lists are not available
        assertFalse(estimator.areListsAvailable());
        assertFalse(estimator.isReady());
        try {
            estimator.getPoints2D();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        try {
            estimator.getPoints3D();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        List<Point3D> points3D = new ArrayList<Point3D>(N_POINTS);
        List<Point2D> points2D = new ArrayList<Point2D>(N_POINTS);        
        for (int i = 0; i < N_POINTS; i++) {
            points3D.add(new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE), 
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
            points2D.add(new HomogeneousPoint2D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE)));
        }

        //set lists
        assertTrue(DLTPointCorrespondencePinholeCameraEstimator.areValidLists(
                points3D, points2D));
        
        estimator.setLists(points3D, points2D);        
        
        //check correctness
        assertEquals(points3D, estimator.getPoints3D());
        assertEquals(points2D, estimator.getPoints2D());
        assertTrue(estimator.areListsAvailable());
        assertTrue(estimator.isReady());
        
        //Force WrongListSizesException
        List<Point3D> wrong3D = new ArrayList<Point3D>();        
        List<Point2D> wrong2D = new ArrayList<Point2D>();
        assertFalse(DLTPointCorrespondencePinholeCameraEstimator.areValidLists(
                wrong3D, points2D));
        try {
            estimator.setLists(wrong3D, points2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        assertFalse(DLTPointCorrespondencePinholeCameraEstimator.areValidLists(
                points3D, wrong2D));
        try {
            estimator.setLists(points3D, wrong2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        assertFalse(DLTPointCorrespondencePinholeCameraEstimator.areValidLists(
                null, points2D));
        try {
            estimator.setLists(null, points2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertFalse(DLTPointCorrespondencePinholeCameraEstimator.areValidLists(
                points3D, null));
        try {
            estimator.setLists(points3D, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();
        
        assertNull(estimator.getListener());
        
        //set listener
        estimator.setListener(this);
        
        //check correctness
        assertEquals(estimator.getListener(), this);
    }
    
    @Test
    public void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
        DLTPointCorrespondencePinholeCameraEstimator estimator =
                new DLTPointCorrespondencePinholeCameraEstimator();

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
    public void testEstimateNoSuggestion() throws WrongListSizesException, 
        LockedException, NotReadyException, PinholeCameraEstimatorException, 
        CameraException, NotAvailableException, RotationException {
        
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

            //testing the case where there are more than six points without 
            //allowing an LMSE solution        
            int nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);
            }

            List<Point2D> points2D = camera.project(points3D);

            assertTrue(DLTPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            DLTPointCorrespondencePinholeCameraEstimator estimator = 
                    new DLTPointCorrespondencePinholeCameraEstimator(points3D, 
                    points2D, this);

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

            //test the case where there are exactly six points
            nPoints = N_POINTS;
            points3D.clear();
            for (int i = 0; i < nPoints; i++) {
                point3D = new InhomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);            
            }

            points2D = camera.project(points3D);

            assertTrue(DLTPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            estimator = new DLTPointCorrespondencePinholeCameraEstimator(
                    points3D, points2D, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());        
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);

            //project original points using estimated camera
            List<Point2D> estimatedPoints2D = estimatedCamera.project(points3D);

            //check that points2D and estimated points2D are equal
            Iterator<Point2D> points2DIt = points2D.iterator();
            Iterator<Point2D> estimatedPoints2DIt = 
                    estimatedPoints2D.iterator();

            Point2D point2D, estimatedPoint2D;
            while (points2DIt.hasNext() && estimatedPoints2DIt.hasNext()) {
                point2D = points2DIt.next();
                estimatedPoint2D = estimatedPoints2DIt.next();

                assertTrue(point2D.equals(estimatedPoint2D, ABSOLUTE_ERROR));
            }

            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            //Comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    LARGE_ABSOLUTE_ERROR);

            
            //Comparing estimated rotation
            Rotation3D estimatedRotation = 
                    estimatedCamera.getCameraRotation();
            
            MatrixRotation3D estimatedRotation2 = 
                    (MatrixRotation3D)estimatedRotation;
            double estimatedAlphaEuler = 
                    estimatedRotation2.getAlphaEulerAngle();
            double estimatedBetaEuler =
                    estimatedRotation2.getBetaEulerAngle();
            double estimatedGammaEuler =
                    estimatedRotation2.getGammaEulerAngle();
            boolean validAlphaEuler, validBetaEuler, 
                    validGammaEuler;
            
            if (Math.abs(alphaEuler - estimatedAlphaEuler) <= 
                    ABSOLUTE_ERROR) {
                validAlphaEuler = true;
            } else if ((Math.abs(alphaEuler) + Math.abs(estimatedAlphaEuler) - 
                    Math.PI) <= ABSOLUTE_ERROR) {
                validAlphaEuler = true;
            } else {
                validAlphaEuler = false;
            }

            if (Math.abs(betaEuler - estimatedBetaEuler) <= 
                    ABSOLUTE_ERROR) {
                validBetaEuler = true;
            } else if ((Math.abs(betaEuler) + Math.abs(estimatedBetaEuler) - 
                    Math.PI) <= ABSOLUTE_ERROR) {
                validBetaEuler = true;
            } else {
                validBetaEuler = false;
            }

            if (Math.abs(gammaEuler - estimatedGammaEuler) <= 
                    ABSOLUTE_ERROR) {
                validGammaEuler = true;
            } else if ((Math.abs(gammaEuler) + Math.abs(estimatedGammaEuler) - 
                    Math.PI) <= ABSOLUTE_ERROR) {
                validGammaEuler = true;
            } else {
                validGammaEuler = false;
            }
            
            
            assertTrue(validAlphaEuler);
            assertTrue(validBetaEuler);
            assertTrue(validGammaEuler);
            
            
            //comparing estimated camera center
            Point3D estimatedCameraCenter = estimatedCamera.getCameraCenter();
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    LARGE_ABSOLUTE_ERROR));

            
            
            //Testing the case where there are more than six points and the LMSE 
            //solution is allowed
            estimator.setLMSESolutionAllowed(true);
            nPoints = randomizer.nextInt(MIN_POINTS, MAX_POINTS);
            points3D.clear();
            for (int i = 0; i < nPoints; i++) {
                point3D = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);            
            }

            points2D = camera.project(points3D);

            assertTrue(DLTPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            estimator = new DLTPointCorrespondencePinholeCameraEstimator(
                    points3D, points2D, this);

            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            try {
                estimatedCamera = estimator.estimate();
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }

            assertFalse(estimator.isLocked());        
            assertEquals(startCount, 1);
            assertEquals(endCount, 1);
            assertEquals(progressCount, 0);
            

            //project original points using estimated camera
            estimatedPoints2D = estimatedCamera.project(points3D);

            //check that points2D and estimated points2D are equal
            points2DIt = points2D.iterator();
            estimatedPoints2DIt = estimatedPoints2D.iterator();

            while (points2DIt.hasNext() && estimatedPoints2DIt.hasNext()) {
                point2D = points2DIt.next();
                estimatedPoint2D = estimatedPoints2DIt.next();

                assertTrue(point2D.equals(estimatedPoint2D, ABSOLUTE_ERROR));
            }

            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();

            //Comparing camera intrinsic parameters
            estimatedIntrinsic = estimatedCamera.getIntrinsicParameters();

            assertEquals(horizontalFocalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalFocalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    LARGE_ABSOLUTE_ERROR);

            
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
                    LARGE_ABSOLUTE_ERROR));   
            
            //Force PinholeCameraEstimatorException
            nPoints = N_POINTS;
            points3D.clear();
            for (int i = 0; i < nPoints - 2; i++) {
                point3D = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);            
            }
            point3D = new HomogeneousPoint3D(
                    randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE),
                    randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE));
            points3D.add(point3D);            
            points3D.add(point3D);            

            points2D = camera.project(points3D);

            estimator = new DLTPointCorrespondencePinholeCameraEstimator(
                    points3D, points2D, this);

            reset();
            estimatedCamera = null;
            try {
                estimatedCamera = estimator.estimate();
                fail("PinholeCameraEstimatorException expected but not thrown");
            } catch (PinholeCameraEstimatorException e) { }
            
            //Force NotReadyException
            estimator = new DLTPointCorrespondencePinholeCameraEstimator();
            try {
                estimatedCamera = estimator.estimate();
                fail("NotReadyException expected but not thrown");
            } catch (NotReadyException e) { }
            assertNull(estimatedCamera);            
            
            passedAtLeastOnce = true;
            
            if (passedAtLeastOnce) {
                break;
            }
        }
        assertTrue(passedAtLeastOnce);
    }

    @Test
    public void testEstimateSuggestedSkewness() throws WrongListSizesException,
            LockedException, NotReadyException, PinholeCameraEstimatorException,
            CameraException, NotAvailableException, RotationException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            
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
            
            //testing the case where there are exactly six points
            int nPoints = N_POINTS;
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);                
            }
            
            List<Point2D> points2D = camera.project(points3D);
            
            //add error to projected points
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            
            for (Point2D point2D : points2D) {
                double errorX = errorRandomizer.nextDouble();
                double errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX, 
                        point2D.getInhomY() + errorY);
            }
            
            assertTrue(DLTPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            DLTPointCorrespondencePinholeCameraEstimator estimator = 
                    new DLTPointCorrespondencePinholeCameraEstimator(points3D, 
                    points2D, this);
            estimator.setLMSESolutionAllowed(false);
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
    public void testEstimatedSuggestedHorizontalFocalLengthEnabled() 
            throws WrongListSizesException, LockedException, NotReadyException, 
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException, RotationException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            
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
            
            //testing the case where there are exactly six points
            int nPoints = N_POINTS;
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);                
            }
            
            List<Point2D> points2D = camera.project(points3D);
            
            //add error to projected points
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            
            for(Point2D point2D : points2D) {
                double errorX = errorRandomizer.nextDouble();
                double errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX, 
                        point2D.getInhomY() + errorY);
            }
            
            assertTrue(DLTPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            DLTPointCorrespondencePinholeCameraEstimator estimator = 
                    new DLTPointCorrespondencePinholeCameraEstimator(points3D, 
                    points2D, this);
            estimator.setLMSESolutionAllowed(false);
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
    public void testEstimatedSuggestedVerticalFocalLengthEnabled() 
            throws WrongListSizesException, LockedException, NotReadyException, 
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException, RotationException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            
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
            
            //testing the case where there are exactly six points
            int nPoints = N_POINTS;
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);                
            }
            
            List<Point2D> points2D = camera.project(points3D);
            
            //add error to projected points
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            
            for(Point2D point2D : points2D) {
                double errorX = errorRandomizer.nextDouble();
                double errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX, 
                        point2D.getInhomY() + errorY);
            }
            
            assertTrue(DLTPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));

            DLTPointCorrespondencePinholeCameraEstimator estimator = 
                    new DLTPointCorrespondencePinholeCameraEstimator(points3D, 
                    points2D, this);
            estimator.setLMSESolutionAllowed(false);
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
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException, RotationException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            
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
            
            //testing the case where there are exactly six points
            int nPoints = N_POINTS;
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);                
            }
            
            List<Point2D> points2D = camera.project(points3D);
            
            //add error to projected points
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            
            for(Point2D point2D : points2D) {
                double errorX = errorRandomizer.nextDouble();
                double errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX, 
                        point2D.getInhomY() + errorY);
            }
            
            assertTrue(DLTPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));
            
            DLTPointCorrespondencePinholeCameraEstimator estimator = 
                    new DLTPointCorrespondencePinholeCameraEstimator(points3D, 
                    points2D, this);
            estimator.setLMSESolutionAllowed(false);
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
            PinholeCameraEstimatorException, CameraException,
            NotAvailableException, RotationException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            
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
            
            //testing the case where there are exactly six points
            int nPoints = N_POINTS;
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);                
            }
            
            List<Point2D> points2D = camera.project(points3D);
            
            //add error to projected points
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            
            for(Point2D point2D : points2D) {
                double errorX = errorRandomizer.nextDouble();
                double errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX, 
                        point2D.getInhomY() + errorY);
            }
            
            assertTrue(DLTPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));
            
            DLTPointCorrespondencePinholeCameraEstimator estimator = 
                    new DLTPointCorrespondencePinholeCameraEstimator(points3D, 
                    points2D, this);
            estimator.setLMSESolutionAllowed(false);
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
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException, RotationException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            
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
            
            //testing the case where there are exactly six points
            int nPoints = N_POINTS;
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);                
            }
            
            List<Point2D> points2D = camera.project(points3D);
            
            //add error to projected points
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            
            for(Point2D point2D : points2D) {
                double errorX = errorRandomizer.nextDouble();
                double errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX, 
                        point2D.getInhomY() + errorY);
            }
            
            assertTrue(DLTPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));  
            
            DLTPointCorrespondencePinholeCameraEstimator estimator = 
                    new DLTPointCorrespondencePinholeCameraEstimator(points3D, 
                    points2D, this);
            estimator.setLMSESolutionAllowed(false);
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
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException, RotationException {
        
        boolean passedAtLeastOnce = false; //to account for random degeneracies
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            double horizontalFocalLength = randomizer.nextDouble(
                    MIN_FOCAL_LENGTH, MAX_FOCAL_LENGTH);
            double verticalFocalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH,
                    MAX_FOCAL_LENGTH);
            
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
            
            //testing the case where there are exactly six points
            int nPoints = N_POINTS;
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);                
            }
            
            List<Point2D> points2D = camera.project(points3D);
            
            //add error to projected points
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            
            for(Point2D point2D : points2D) {
                double errorX = errorRandomizer.nextDouble();
                double errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX, 
                        point2D.getInhomY() + errorY);
            }
            
            assertTrue(DLTPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));  
            
            DLTPointCorrespondencePinholeCameraEstimator estimator = 
                    new DLTPointCorrespondencePinholeCameraEstimator(points3D, 
                    points2D, this);
            estimator.setLMSESolutionAllowed(false);
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
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException, RotationException {
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
            
            //testing the case where there are exactly six points
            int nPoints = N_POINTS;
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                point3D = new HomogeneousPoint3D(
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE),
                        randomizer.nextDouble(MIN_RANDOM_VALUE, 
                        MAX_RANDOM_VALUE));
                points3D.add(point3D);                
            }
            
            List<Point2D> points2D = camera.project(points3D);
            
            //add error to projected points
            GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, ERROR_STD);
            
            for(Point2D point2D : points2D) {
                double errorX = errorRandomizer.nextDouble();
                double errorY = errorRandomizer.nextDouble();
                point2D.setInhomogeneousCoordinates(
                        point2D.getInhomX() + errorX, 
                        point2D.getInhomY() + errorY);
            }
            
            assertTrue(DLTPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));  
            
            DLTPointCorrespondencePinholeCameraEstimator estimator = 
                    new DLTPointCorrespondencePinholeCameraEstimator(points3D, 
                    points2D, this);
            estimator.setLMSESolutionAllowed(false);
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
            
            //estimate without suggestions
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
        checkIsLocked((DLTPointCorrespondencePinholeCameraEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(PinholeCameraEstimator estimator) {
        endCount++;
        checkIsLocked((DLTPointCorrespondencePinholeCameraEstimator)estimator);
    }

    @Override
    public void onEstimationProgressChange(PinholeCameraEstimator estimator, 
        float progress) {
        progressCount++;
        checkIsLocked((DLTPointCorrespondencePinholeCameraEstimator)estimator);
    }
    
    private void reset(){
        startCount = endCount = progressCount = 0;
    }
    
    private void checkIsLocked(
            DLTPointCorrespondencePinholeCameraEstimator estimator) {
        assertTrue(estimator.isLocked());
        try {
            estimator.setLMSESolutionAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        
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
            estimator.setPointCorrespondencesNormalized(true);
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
