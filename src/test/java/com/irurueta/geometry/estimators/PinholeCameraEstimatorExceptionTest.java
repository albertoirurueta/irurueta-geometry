/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.PinholeCameraEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 19, 2013
 */
package com.irurueta.geometry.estimators;

import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class PinholeCameraEstimatorExceptionTest {
    
    public PinholeCameraEstimatorExceptionTest() {
    }
    
    @BeforeClass
    public static void setUpClass() {
    }
    
    @AfterClass
    public static void tearDownClass() {
    }
    
    @Before
    public void setUp() {
    }
    
    @After
    public void tearDown() {
    }
    
    @Test
    public void testConstructor(){
        PinholeCameraEstimatorException ex;
        assertNotNull(ex = new PinholeCameraEstimatorException());
        
        ex = null;
        assertNotNull(ex = new PinholeCameraEstimatorException("message"));
        
        ex = null;
        assertNotNull(ex = new PinholeCameraEstimatorException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new PinholeCameraEstimatorException("message", 
                new Exception()));
    }
}
