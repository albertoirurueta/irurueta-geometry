/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.DistortionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 30, 2015
 */
package com.irurueta.geometry.calib3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class DistortionExceptionTest {
    
    public DistortionExceptionTest() {}
    
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
        DistortionException ex;
        assertNotNull(ex = new DistortionException());
        
        ex = null;
        assertNotNull(ex = new DistortionException("message"));
        
        ex = null;
        assertNotNull(ex = new DistortionException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new DistortionException("message", new Exception()));
    }
}
