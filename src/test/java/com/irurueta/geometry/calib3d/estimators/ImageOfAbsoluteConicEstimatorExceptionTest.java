/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.estimators.ImageOfAbsoluteConicEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 1, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class ImageOfAbsoluteConicEstimatorExceptionTest {
    
    public ImageOfAbsoluteConicEstimatorExceptionTest() { }
    
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
        ImageOfAbsoluteConicEstimatorException ex;
        assertNotNull(ex = new ImageOfAbsoluteConicEstimatorException());
        
        ex = null;
        assertNotNull(ex = new ImageOfAbsoluteConicEstimatorException(
                "message"));
        
        ex = null;
        assertNotNull(ex = new ImageOfAbsoluteConicEstimatorException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new ImageOfAbsoluteConicEstimatorException("message", 
                new Exception()));
    }    
}
