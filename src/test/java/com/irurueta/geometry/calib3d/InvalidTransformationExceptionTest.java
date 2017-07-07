/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.InvalidTransformationException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 28, 2016.
 */
package com.irurueta.geometry.calib3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class InvalidTransformationExceptionTest {
    
    public InvalidTransformationExceptionTest() { }
    
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
        InvalidTransformationException ex;
        assertNotNull(ex = new InvalidTransformationException());
        
        ex = null;
        assertNotNull(ex = new InvalidTransformationException("message"));
        
        ex = null;
        assertNotNull(ex = new InvalidTransformationException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new InvalidTransformationException("message", 
                new Exception()));
    }
}
