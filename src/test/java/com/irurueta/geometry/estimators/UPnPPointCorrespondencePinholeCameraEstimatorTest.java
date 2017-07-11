/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.UPnPPointCorrespondencePinholeCameraEstimator
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 5, 2017.
 */
package com.irurueta.geometry.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.AxisRotation3D;
import com.irurueta.geometry.CameraException;
import com.irurueta.geometry.HomogeneousPoint2D;
import com.irurueta.geometry.HomogeneousPoint3D;
import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.NotAvailableException;
import com.irurueta.geometry.PinholeCamera;
import com.irurueta.geometry.PinholeCameraIntrinsicParameters;
import com.irurueta.geometry.Plane;
import com.irurueta.geometry.Point2D;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class UPnPPointCorrespondencePinholeCameraEstimatorTest implements 
        PinholeCameraEstimatorListener {
    
    public static final double ABSOLUTE_ERROR = 1e-6;
    public static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    public static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-1;
    public static final double ULTRA_LARGE_ABSOLUTE_ERROR = 1.0;
    
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
    
    public static final int N_POINTS = 6;
    
    public static final int TIMES = 100;
    
    public static final double ERROR_STD = 1e-5;    
    
    public static final double PLANE_SLANT_DEGREES = 45.0;
    
    private int estimateStart;
    private int estimateEnd;
    private int estimationProgressChange;
        
    public UPnPPointCorrespondencePinholeCameraEstimatorTest() { }
    
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
        //check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.UPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD, 
                0.0);
        assertEquals(estimator.getSkewness(), 
                UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_SKEWNESS, 
                0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_HORIZONTAL_PRINCIPAL_POINT, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_VERTICAL_PRINCIPAL_POINT, 0.0);
        assertFalse(estimator.isPlanar());
        
        
        //testing constructor with listener
        estimator = new UPnPPointCorrespondencePinholeCameraEstimator(this);
        
        //check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.UPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        try {
            estimator.getPoints2D();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        try {
            estimator.getPoints3D();
            fail("NotAvailableException expected but not thrown");
        } catch (NotAvailableException e) { }
        assertSame(estimator.getListener(), this);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD, 
                0.0);
        assertEquals(estimator.getSkewness(), 
                UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_SKEWNESS, 
                0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_HORIZONTAL_PRINCIPAL_POINT, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_VERTICAL_PRINCIPAL_POINT, 0.0);
        assertFalse(estimator.isPlanar());


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
        
        estimator = new UPnPPointCorrespondencePinholeCameraEstimator(points3D, 
                points2D);
        
        //check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.UPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD, 
                0.0);
        assertEquals(estimator.getSkewness(), 
                UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_SKEWNESS, 
                0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_HORIZONTAL_PRINCIPAL_POINT, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_VERTICAL_PRINCIPAL_POINT, 0.0);
        assertFalse(estimator.isPlanar());
        
        //Force WrongListSizesException
        List<Point3D> wrong3D = new ArrayList<Point3D>();
        List<Point2D> wrong2D = new ArrayList<Point2D>();
        estimator = null;
        try {
            estimator = new UPnPPointCorrespondencePinholeCameraEstimator(
                    wrong3D, points2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        try {
            estimator = new UPnPPointCorrespondencePinholeCameraEstimator(
                    points3D, wrong2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        try {
            estimator = new UPnPPointCorrespondencePinholeCameraEstimator(null,
                    points2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new UPnPPointCorrespondencePinholeCameraEstimator(
                    points3D, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);        
        
        
        //testing constructor with lists and listener
        estimator = new UPnPPointCorrespondencePinholeCameraEstimator(points3D, 
                points2D, this);
        
        //check correctness
        assertEquals(estimator.getType(),
                PinholeCameraEstimatorType.UPnP_PINHOLE_CAMERA_ESTIMATOR);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.arePointCorrespondencesNormalized());
        assertEquals(estimator.getPoints2D(), points2D);
        assertEquals(estimator.getPoints3D(), points3D);
        assertSame(estimator.getListener(), this);
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
        assertTrue(estimator.isPlanarConfigurationAllowed());
        assertTrue(estimator.isNullspaceDimension2Allowed());
        assertEquals(estimator.getPlanarThreshold(), 
                EPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_PLANAR_THRESHOLD, 
                0.0);
        assertEquals(estimator.getSkewness(), 
                UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_SKEWNESS, 
                0.0);
        assertEquals(estimator.getHorizontalPrincipalPoint(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_HORIZONTAL_PRINCIPAL_POINT, 0.0);
        assertEquals(estimator.getVerticalPrincipalPoint(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_VERTICAL_PRINCIPAL_POINT, 0.0);
        assertFalse(estimator.isPlanar());
        
        //Force WrongListSizesException
        estimator = null;
        try {
            estimator = new UPnPPointCorrespondencePinholeCameraEstimator(
                    wrong3D, points2D, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        try {
            estimator = new UPnPPointCorrespondencePinholeCameraEstimator(
                    points3D, wrong2D, this);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        try {
            estimator = new UPnPPointCorrespondencePinholeCameraEstimator(null,
                    points2D, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        try {
            estimator = new UPnPPointCorrespondencePinholeCameraEstimator(
                    points3D, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertNull(estimator);                
    }
    
    @Test
    public void testAreSetPointCorrespondencesNormalized() 
            throws LockedException {
        
        UPnPPointCorrespondencePinholeCameraEstimator estimator = 
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
        assertFalse(estimator.arePointCorrespondencesNormalized());
        
        //check it can't be enabled
        estimator.setPointCorrespondencesNormalized(true);
        
        assertFalse(estimator.arePointCorrespondencesNormalized());
    }
    
    @Test
    public void testGetSetListsValidityAndAvailability() throws LockedException,
            IllegalArgumentException, WrongListSizesException,
            NotAvailableException {
        
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(
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
        assertFalse(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(
                wrong3D, points2D));
        try {
            estimator.setLists(wrong3D, points2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        assertFalse(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(
                points3D, wrong2D));
        try {
            estimator.setLists(points3D, wrong2D);
            fail("WrongListSizesException expected but not thrown");
        } catch (WrongListSizesException e) { }
        
        //Force IllegalArgumentException
        assertFalse(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(
                null, points2D));
        try {
            estimator.setLists(null, points2D);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }
        assertFalse(UPnPPointCorrespondencePinholeCameraEstimator.areValidLists(
                points3D, null));
        try {
            estimator.setLists(points3D, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException e) { }        
    }
    
    @Test
    public void testGetSetListener() throws LockedException {
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
        assertNull(estimator.getListener());
        
        //set listener
        estimator.setListener(this);
        
        //check correctness
        assertSame(estimator.getListener(), this);
    }    
    
    @Test
    public void testIsSetSuggestSkewnessValueEnabled() throws LockedException {
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();

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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();

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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();

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
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();

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
    public void testIsSetPlanarConfigurationAllowed() throws LockedException {
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
        //check default value
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        
        //set new value
        estimator.setPlanarConfigurationAllowed(
                !UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);
        
        //check correctness
        assertEquals(estimator.isPlanarConfigurationAllowed(),
                !UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_CONFIGURATION_ALLOWED);        
    }
    
    @Test
    public void testIsSetNullspaceDimension2Allowed() throws LockedException {
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
        //check default value
        assertTrue(estimator.isNullspaceDimension2Allowed());
        
        //set new value
        estimator.setNullspaceDimension2Allowed(false);
        
        //check correctness
        assertFalse(estimator.isNullspaceDimension2Allowed());
    }
    
    @Test
    public void testGetSetPlanarThreshold() throws LockedException {
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
        //check correctness
        assertEquals(estimator.getPlanarThreshold(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_PLANAR_THRESHOLD, 0.0);
        
        //set new value
        estimator.setPlanarThreshold(1e-3);
        
        //check correctness
        assertEquals(estimator.getPlanarThreshold(), 1e-3, 0.0);
    }
    
    @Test
    public void testGetSetSkewness() throws LockedException {
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
        //check correctness
        assertEquals(estimator.getSkewness(),
                UPnPPointCorrespondencePinholeCameraEstimator.DEFAULT_SKEWNESS, 
                0.0);
        
        //set new value
        estimator.setSkewness(1e-3);
        
        //check correctness
        assertEquals(estimator.getSkewness(), 1e-3, 0.0);
    }
    
    @Test
    public void testGetSetHorizontalPrincipalPoint() throws LockedException {
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
        //check correctness
        assertEquals(estimator.getHorizontalPrincipalPoint(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_HORIZONTAL_PRINCIPAL_POINT, 0.0);
        
        //set new value
        estimator.setHorizontalPrincipalPoint(100.0);
        
        //check correctness
        assertEquals(estimator.getHorizontalPrincipalPoint(), 100.0, 0.0);
    }
    
    @Test
    public void testGetSetVerticalPrincipalPoint() throws LockedException {
        UPnPPointCorrespondencePinholeCameraEstimator estimator =
                new UPnPPointCorrespondencePinholeCameraEstimator();
        
        //check correctness
        assertEquals(estimator.getVerticalPrincipalPoint(),
                UPnPPointCorrespondencePinholeCameraEstimator.
                        DEFAULT_VERTICAL_PRINCIPAL_POINT, 0.0);
        
        //set new value
        estimator.setVerticalPrincipalPoint(100.0);
        
        //check correctness
        assertEquals(estimator.getVerticalPrincipalPoint(), 100.0, 0.0);
    }
    
    @Test
    public void testEstimateGeneralNoSuggestion() 
            throws WrongListSizesException, LockedException, NotReadyException,
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                    focalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = UPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;
            
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
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
            
            List<Point2D> points2D = camera.project(points3D);  

            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));
            
            UPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new UPnPPointCorrespondencePinholeCameraEstimator(points3D, 
                            points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
            
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();            
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            assertFalse(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);
            
            assertNotNull(estimatedCamera);  
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();    
            
            //Comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();

            if(Math.abs(focalLength - estimatedIntrinsic.getHorizontalFocalLength()) > 
                    VERY_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(focalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(focalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    ABSOLUTE_ERROR);
            
            //comparing estimated camera center
            Point3D estimatedCameraCenter = new InhomogeneousPoint3D(
                    estimatedCamera.getCameraCenter());
            if(!cameraCenter.equals(estimatedCameraCenter, 
                    VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    VERY_LARGE_ABSOLUTE_ERROR));            
            
            //comparing estimated rotation
            Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();
            
            if (Math.abs(rotation.getA() - estimatedRotation.getA()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getA(), estimatedRotation.getA(), 
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getB() - estimatedRotation.getB()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getB(), estimatedRotation.getB(), 
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getC() - estimatedRotation.getC()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getC(), estimatedRotation.getC(), 
                    LARGE_ABSOLUTE_ERROR);
            if (Math.abs(rotation.getD() - estimatedRotation.getD()) > LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(rotation.getD(), estimatedRotation.getD(), 
                    LARGE_ABSOLUTE_ERROR);  
            
            if (!rotMatrix.equals(estimatedRotMatrix, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(rotMatrix.equals(estimatedRotMatrix, 
                    LARGE_ABSOLUTE_ERROR));

            //project original points using estimated camera
            List<Point2D> estimatedPoints2D = estimatedCamera.project(points3D);
            
            //check that points2D and estimated points2D are equal
            Point2D point2D, estimatedPoint2D;
            for (int i = 0; i < nPoints; i++) {
                point2D = points2D.get(i);
                estimatedPoint2D = estimatedPoints2D.get(i);
                
                assertTrue(point2D.equals(estimatedPoint2D, 
                        VERY_LARGE_ABSOLUTE_ERROR));
            }
            
            numValid++;         
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimatePlanarNoSuggestion() 
            throws WrongListSizesException, LockedException, NotReadyException,
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                    focalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            //generate plane parallel to the camera's principal plane and
            //located at certain distance from it.
            double[] principalAxis = camera.getPrincipalAxisArray();
            double depth = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));
            
              
            //plane is completely parallel to image plane
            Plane plane = new Plane(centroid, principalAxis);
            
            //we rotate plane slightly using a perpendicular axis
            double[] axis = camera.getHorizontalAxisPlane().getDirectorVector();
            double angle = com.irurueta.geometry.Utils.convertToRadians(
                    PLANE_SLANT_DEGREES);
            AxisRotation3D planeRotation = new AxisRotation3D(axis, angle);
            plane = planeRotation.rotate(plane);
            
            axis = camera.getVerticalAxisPlane().getDirectorVector();
            planeRotation = new AxisRotation3D(axis, angle);
            plane = planeRotation.rotate(plane);
            
            int nPoints = UPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;
            
            double a = plane.getA();
            double b = plane.getB();
            double c = plane.getC();
            double d = plane.getD();
            
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane
                //a*x + b*y + c*z + d*w = 0
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = 1.0;
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  

            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));
            
            UPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new UPnPPointCorrespondencePinholeCameraEstimator(points3D, 
                            points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
            
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();            
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            assertTrue(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);
            
            assertNotNull(estimatedCamera);  
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();    
            
            //Comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();

            if(Math.abs(focalLength - estimatedIntrinsic.getHorizontalFocalLength()) > 
                    ULTRA_LARGE_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(focalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(focalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    ABSOLUTE_ERROR);
            
            //comparing estimated camera center
            Point3D estimatedCameraCenter = new InhomogeneousPoint3D(
                    estimatedCamera.getCameraCenter());
            if(!cameraCenter.equals(estimatedCameraCenter, 
                    VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    VERY_LARGE_ABSOLUTE_ERROR));            
            
            //comparing estimated rotation
            Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();
            
            assertEquals(rotation.getA(), estimatedRotation.getA(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), 
                    LARGE_ABSOLUTE_ERROR);  
            
            assertTrue(rotMatrix.equals(estimatedRotMatrix, 
                    LARGE_ABSOLUTE_ERROR));

            //project original points using estimated camera
            List<Point2D> estimatedPoints2D = estimatedCamera.project(points3D);
            
            //check that points2D and estimated points2D are equal
            Point2D point2D, estimatedPoint2D;
            for (int i = 0; i < nPoints; i++) {
                point2D = points2D.get(i);
                estimatedPoint2D = estimatedPoints2D.get(i);
                
                assertTrue(point2D.equals(estimatedPoint2D, 
                        VERY_LARGE_ABSOLUTE_ERROR));
            }
            
            numValid++;    
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimateGeneralSuggestedRotationEnabled() 
            throws WrongListSizesException, LockedException, NotReadyException,
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                    focalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = UPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;
            
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
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
            
            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));
            
            UPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new UPnPPointCorrespondencePinholeCameraEstimator(points3D, 
                            points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
            estimator.setSuggestRotationEnabled(true);
            estimator.setSuggestedRotationValue(rotation);            
            
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();            
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            assertFalse(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);
            
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
                    Math.pow(rotation.getA() - estimatedQNoSuggestion.getA(), 2.0) +
                    Math.pow(rotation.getB() - estimatedQNoSuggestion.getB(), 2.0) +
                    Math.pow(rotation.getC() - estimatedQNoSuggestion.getC(), 2.0) +
                    Math.pow(rotation.getD() - estimatedQNoSuggestion.getD(), 2.0);
            double diffEstimated =
                    Math.pow(rotation.getA() - estimatedQ.getA(), 2.0) +
                    Math.pow(rotation.getB() - estimatedQ.getB(), 2.0) +
                    Math.pow(rotation.getC() - estimatedQ.getC(), 2.0) +
                    Math.pow(rotation.getD() - estimatedQ.getD(), 2.0);
            
            if (diffEstimatedNoSuggestion > diffEstimated) {
                numValid++;
            }
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateGeneralSuggestedCenterEnabled() 
            throws WrongListSizesException, LockedException, NotReadyException,
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = randomizer.nextDouble(MIN_SKEWNESS, MAX_SKEWNESS);
            double horizontalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);
            double verticalPrincipalPoint = randomizer.nextDouble(
                    MIN_PRINCIPAL_POINT, MAX_PRINCIPAL_POINT);

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                    focalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = UPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;
            
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
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
            
            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));
            
            UPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new UPnPPointCorrespondencePinholeCameraEstimator(points3D, 
                            points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
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
            
            assertFalse(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);
            
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
                numValid++;
            }
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimateGeneralZeroSkewnessZeroPrincipalPointNoSuggestion() 
            throws WrongListSizesException, LockedException, NotReadyException,
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                    focalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            int nPoints = UPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;
            
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
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
            
            List<Point2D> points2D = camera.project(points3D);  

            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));
            
            UPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new UPnPPointCorrespondencePinholeCameraEstimator(points3D, 
                            points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
            
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();            
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            assertFalse(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);
            
            assertNotNull(estimatedCamera);  
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();    
            
            //Comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();

            assertEquals(focalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(focalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    ABSOLUTE_ERROR);
            
            //comparing estimated camera center
            Point3D estimatedCameraCenter = new InhomogeneousPoint3D(
                    estimatedCamera.getCameraCenter());
            if(!cameraCenter.equals(estimatedCameraCenter, 
                    VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    VERY_LARGE_ABSOLUTE_ERROR));            
            
            //comparing estimated rotation
            Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();
            
            assertEquals(rotation.getA(), estimatedRotation.getA(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), 
                    LARGE_ABSOLUTE_ERROR);  
            
            assertTrue(rotMatrix.equals(estimatedRotMatrix, 
                    LARGE_ABSOLUTE_ERROR));

            //project original points using estimated camera
            List<Point2D> estimatedPoints2D = estimatedCamera.project(points3D);
            
            //check that points2D and estimated points2D are equal
            Point2D point2D, estimatedPoint2D;
            for (int i = 0; i < nPoints; i++) {
                point2D = points2D.get(i);
                estimatedPoint2D = estimatedPoints2D.get(i);
                
                assertTrue(point2D.equals(estimatedPoint2D, 
                        VERY_LARGE_ABSOLUTE_ERROR));
            }
            
            numValid++;      
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testEstimatePlanarZeroSkewnessZeroPrincipalPointNoSuggestion() 
            throws WrongListSizesException, LockedException, NotReadyException,
            PinholeCameraEstimatorException, CameraException, 
            NotAvailableException {
        int numValid = 0;
        for (int t = 0; t < 5*TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            
            //intrinsic parameters
            double focalLength = randomizer.nextDouble(MIN_FOCAL_LENGTH, 
                    MAX_FOCAL_LENGTH);
            double skewness = 0.0;
            double horizontalPrincipalPoint = 0.0;
            double verticalPrincipalPoint = 0.0;

            PinholeCameraIntrinsicParameters intrinsic =
                    new PinholeCameraIntrinsicParameters(focalLength,
                    focalLength, horizontalPrincipalPoint, 
                    verticalPrincipalPoint, skewness);
            
            //create rotation parameters
            double roll = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double pitch = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            double yaw = randomizer.nextDouble(MIN_ANGLE_DEGREES, 
                    MAX_ANGLE_DEGREES) * Math.PI / 180.0;
            Quaternion rotation = new Quaternion(roll, pitch, yaw);
            rotation.normalize();
            
            //create camera center
            double[] cameraCenterArray = new double[3];
            randomizer.fill(cameraCenterArray, MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            InhomogeneousPoint3D cameraCenter = new InhomogeneousPoint3D(
                    cameraCenterArray);

            //instantiate camera
            PinholeCamera camera = new PinholeCamera(intrinsic, rotation, 
                    cameraCenter);
            
            //normalize the camera to improve accuracy
            camera.normalize();  
            
            //generate plane parallel to the camera's principal plane and
            //located at certain distance from it.
            double[] principalAxis = camera.getPrincipalAxisArray();
            double depth = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                    MAX_RANDOM_VALUE);
            Point3D centroid = new InhomogeneousPoint3D(
                    cameraCenter.getInhomX() + depth * principalAxis[0],
                    cameraCenter.getInhomY() + depth * principalAxis[1],
                    cameraCenter.getInhomZ() + depth * principalAxis[2]);
            assertTrue(camera.isPointInFrontOfCamera(centroid));
            
            //plane is completely parallel to image plane
            Plane plane = new Plane(centroid, principalAxis);
            
            //we rotate plane slightly using a perpendicular axis
            double[] axis = camera.getHorizontalAxisPlane().getDirectorVector();
            double angle = com.irurueta.geometry.Utils.convertToRadians(
                    PLANE_SLANT_DEGREES);
            AxisRotation3D planeRotation = new AxisRotation3D(axis, angle);
            plane = planeRotation.rotate(plane);
            
            axis = camera.getVerticalAxisPlane().getDirectorVector();
            planeRotation = new AxisRotation3D(axis, angle);
            plane = planeRotation.rotate(plane);
            
            int nPoints = UPnPPointCorrespondencePinholeCameraEstimator.
                    MIN_NUMBER_OF_POINT_CORRESPONDENCES;
            
            double a = plane.getA();
            double b = plane.getB();
            double c = plane.getC();
            double d = plane.getD();
            
            List<Point3D> points3D = new ArrayList<Point3D>(nPoints);
            Point3D point3D;
            for (int i = 0; i < nPoints; i++) {
                //get a random point belonging to the plane
                //a*x + b*y + c*z + d*w = 0
                //y = -(a*x + c*z + d*w)/b or x = -(b*y + c*z + d*w)/a
                double homX, homY;
                double homW = 1.0;
                double homZ = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);                
                if(Math.abs(b) > ABSOLUTE_ERROR){
                    homX = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homY = -(a * homX + c * homZ + d * homW) / b;
                }else{
                    homY = randomizer.nextDouble(MIN_RANDOM_VALUE, 
                            MAX_RANDOM_VALUE);
                    homX = -(b * homY + c * homZ + d * homW) / a;
                }
                
                point3D = new HomogeneousPoint3D(homX, homY, homZ, homW);
                
                assertTrue(plane.isLocus(point3D));
                
                points3D.add(point3D);
            }
            
            List<Point2D> points2D = camera.project(points3D);  

            assertTrue(UPnPPointCorrespondencePinholeCameraEstimator.
                    areValidLists(points3D, points2D));
            
            UPnPPointCorrespondencePinholeCameraEstimator estimator =
                    new UPnPPointCorrespondencePinholeCameraEstimator(points3D, 
                            points2D, this);
            estimator.setSkewness(skewness);
            estimator.setHorizontalPrincipalPoint(horizontalPrincipalPoint);
            estimator.setVerticalPrincipalPoint(verticalPrincipalPoint);
            
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());

            reset();
            
            PinholeCamera estimatedCamera;
            try {
                estimatedCamera = estimator.estimate();            
            } catch (PinholeCameraEstimatorException e) {
                continue;
            }
            
            assertTrue(estimator.isPlanar());
            assertFalse(estimator.isLocked());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);
            assertEquals(estimationProgressChange, 0);
            
            assertNotNull(estimatedCamera);  
            
            //decompose estimated camera and check its parameters
            estimatedCamera.decompose();    
            
            //Comparing camera intrinsic parameters
            PinholeCameraIntrinsicParameters estimatedIntrinsic =
                    estimatedCamera.getIntrinsicParameters();

            if(Math.abs(focalLength - estimatedIntrinsic.getHorizontalFocalLength()) > 
                    ULTRA_LARGE_ABSOLUTE_ERROR) {
                continue;
            }            
            assertEquals(focalLength, 
                    estimatedIntrinsic.getHorizontalFocalLength(), 
                    ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(focalLength,
                    estimatedIntrinsic.getVerticalFocalLength(), 
                    ULTRA_LARGE_ABSOLUTE_ERROR);
            assertEquals(horizontalPrincipalPoint,
                    estimatedIntrinsic.getHorizontalPrincipalPoint(), 
                    ABSOLUTE_ERROR);
            assertEquals(verticalPrincipalPoint,
                    estimatedIntrinsic.getVerticalPrincipalPoint(),
                    ABSOLUTE_ERROR);
            assertEquals(skewness, estimatedIntrinsic.getSkewness(), 
                    ABSOLUTE_ERROR);
            
            //comparing estimated camera center
            Point3D estimatedCameraCenter = new InhomogeneousPoint3D(
                    estimatedCamera.getCameraCenter());
            if(!cameraCenter.equals(estimatedCameraCenter, 
                    VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(cameraCenter.equals(estimatedCameraCenter, 
                    VERY_LARGE_ABSOLUTE_ERROR));            
            
            //comparing estimated rotation
            Quaternion estimatedRotation = estimatedCamera.getCameraRotation().
                    toQuaternion();
            estimatedRotation.normalize();
            
            Matrix rotMatrix = rotation.asInhomogeneousMatrix();
            Matrix estimatedRotMatrix = estimatedRotation.
                    asInhomogeneousMatrix();
            
            assertEquals(rotation.getA(), estimatedRotation.getA(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getB(), estimatedRotation.getB(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getC(), estimatedRotation.getC(), 
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(rotation.getD(), estimatedRotation.getD(), 
                    LARGE_ABSOLUTE_ERROR);  
            
            assertTrue(rotMatrix.equals(estimatedRotMatrix, 
                    LARGE_ABSOLUTE_ERROR));

            //project original points using estimated camera
            List<Point2D> estimatedPoints2D = estimatedCamera.project(points3D);
            
            //check that points2D and estimated points2D are equal
            Point2D point2D, estimatedPoint2D;
            for (int i = 0; i < nPoints; i++) {
                point2D = points2D.get(i);
                estimatedPoint2D = estimatedPoints2D.get(i);
                
                assertTrue(point2D.equals(estimatedPoint2D, 
                        VERY_LARGE_ABSOLUTE_ERROR));
            }
            
            numValid++;      
            
            if (numValid > 0) {
                break;
            }
        }
        
        assertTrue(numValid > 0);
    }
    

    @Override
    public void onEstimateStart(PinholeCameraEstimator estimator) {
        estimateStart++;
        checkIsLocked((UPnPPointCorrespondencePinholeCameraEstimator)estimator);
    }

    @Override
    public void onEstimateEnd(PinholeCameraEstimator estimator) {
        estimateEnd++;
        checkIsLocked((UPnPPointCorrespondencePinholeCameraEstimator)estimator);
    }

    @Override
    public void onEstimationProgressChange(PinholeCameraEstimator estimator, 
            float progress) {
        estimationProgressChange++;
        checkIsLocked((UPnPPointCorrespondencePinholeCameraEstimator)estimator);
    }
    
    private void reset() {
        estimateStart = estimateEnd = estimationProgressChange = 0;
    }   
    
    private void checkIsLocked(
            UPnPPointCorrespondencePinholeCameraEstimator estimator) {
        try {
            estimator.setLists(null, null);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setPlanarConfigurationAllowed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setNullspaceDimension2Allowed(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setPlanarThreshold(1e9);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSkewness(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setHorizontalPrincipalPoint(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setVerticalPrincipalPoint(0.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setPointCorrespondencesNormalized(true);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { 
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
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
            estimator.setMinSuggestionWeight(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setMaxSuggestionWeight(1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setMinMaxSuggestionWeight(1.0, 1.0);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }
        try {
            estimator.setSuggestionWeightStep(0.2);
            fail("LockedException expected but not thrown");
        } catch (LockedException e) { }        
        assertTrue(estimator.isLocked());
    }
}
