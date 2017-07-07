/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.estimators.SingleHomographyPinholeCameraEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 23, 2017.
 */
package com.irurueta.geometry.calib3d.estimators;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class SingleHomographyPinholeCameraEstimatorExceptionTest {
    
    public SingleHomographyPinholeCameraEstimatorExceptionTest() { }
    
    @BeforeClass
    public static void setUpClass() { }
    
    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }

    @Test
    public void testConstructor() {
        SingleHomographyPinholeCameraEstimatorException ex;
        assertNotNull(ex = 
                new SingleHomographyPinholeCameraEstimatorException());
        
        ex = null;
        assertNotNull(ex = 
                new SingleHomographyPinholeCameraEstimatorException("message"));
        
        ex = null;
        assertNotNull(ex = new SingleHomographyPinholeCameraEstimatorException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new SingleHomographyPinholeCameraEstimatorException(
                "message", new Exception()));
    }
}
