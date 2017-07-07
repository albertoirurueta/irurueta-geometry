/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.RadialDistorsionEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 16, 2015
 */
package com.irurueta.geometry.calib3d.estimators;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class RadialDistortionEstimatorExceptionTest {
    
    public RadialDistortionEstimatorExceptionTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testConstructor(){
        RadialDistortionEstimatorException ex;
        assertNotNull(ex = new RadialDistortionEstimatorException());
        
        ex = null;
        assertNotNull(ex = new RadialDistortionEstimatorException("message"));
        
        ex = null;
        assertNotNull(ex = new RadialDistortionEstimatorException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new RadialDistortionEstimatorException("message", 
                new Exception()));
    }    
}
