/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.estimators.KruppaDualImageOfAbsoluteConicEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 26, 2016.
 */
package com.irurueta.geometry.calib3d.estimators;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class KruppaDualImageOfAbsoluteConicEstimatorExceptionTest {
    
    public KruppaDualImageOfAbsoluteConicEstimatorExceptionTest() { }
    
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
        KruppaDualImageOfAbsoluteConicEstimatorException ex;
        assertNotNull(
                ex = new KruppaDualImageOfAbsoluteConicEstimatorException());
        
        ex = null;
        assertNotNull(ex = new KruppaDualImageOfAbsoluteConicEstimatorException(
                        "message"));
        
        ex = null;
        assertNotNull(ex = new KruppaDualImageOfAbsoluteConicEstimatorException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new KruppaDualImageOfAbsoluteConicEstimatorException(
                "message", new Exception()));
    }
}
