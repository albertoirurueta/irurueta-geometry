/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.estimators.DualAbsoluteQuadricEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 22, 2016.
 */
package com.irurueta.geometry.calib3d.estimators;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class DualAbsoluteQuadricEstimatorExceptionTest {
    
    public DualAbsoluteQuadricEstimatorExceptionTest() { }
    
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
        DualAbsoluteQuadricEstimatorException ex;
        assertNotNull(ex = new DualAbsoluteQuadricEstimatorException());
        
        ex = null;
        assertNotNull(ex = new DualAbsoluteQuadricEstimatorException(
                "message"));
        
        ex = null;
        assertNotNull(ex = new DualAbsoluteQuadricEstimatorException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new DualAbsoluteQuadricEstimatorException("message",
        new Exception()));
    }
}
