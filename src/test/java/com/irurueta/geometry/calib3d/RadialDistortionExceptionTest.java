/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.RadialDistortionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date March 23, 2015
 */
package com.irurueta.geometry.calib3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class RadialDistortionExceptionTest {
    
    public RadialDistortionExceptionTest() {}
    
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
        RadialDistortionException ex;
        assertNotNull(ex = new RadialDistortionException());
        
        ex = null;
        assertNotNull(ex = new RadialDistortionException("message"));
        
        ex = null;
        assertNotNull(ex = new RadialDistortionException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new RadialDistortionException("message", 
                new Exception()));
    }
}
