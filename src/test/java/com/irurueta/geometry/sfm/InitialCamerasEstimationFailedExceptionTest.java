/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.InitialCamerasEstimationFailedException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date January 3, 2017.
 */
package com.irurueta.geometry.sfm;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class InitialCamerasEstimationFailedExceptionTest {
    
    public InitialCamerasEstimationFailedExceptionTest() { }
    
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
        InitialCamerasEstimationFailedException ex;
        assertNotNull(ex = new InitialCamerasEstimationFailedException());
        
        ex = null;
        assertNotNull(ex = new InitialCamerasEstimationFailedException(
                "message"));
        
        ex = null;
        assertNotNull(ex = new InitialCamerasEstimationFailedException(
                "message", new Exception()));
        
        ex = null;
        assertNotNull(ex = new InitialCamerasEstimationFailedException(
                new Exception()));
    }
}
