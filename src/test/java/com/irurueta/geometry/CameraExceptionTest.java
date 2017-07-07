/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.CameraException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date November 7, 2012
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class CameraExceptionTest {
    
    public CameraExceptionTest() {
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
        CameraException ex;
        assertNotNull(ex = new CameraException());
        
        ex = null;
        assertNotNull(ex = new CameraException("message"));
        
        ex = null;
        assertNotNull(ex = new CameraException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new CameraException("message", 
                new Exception()));
    }
}
