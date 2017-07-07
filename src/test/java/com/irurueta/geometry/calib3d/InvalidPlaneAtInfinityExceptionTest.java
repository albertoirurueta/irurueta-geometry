/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.calib3d.InvalidPlaneAtInfinityException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date October 22, 2016.
 */
package com.irurueta.geometry.calib3d;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class InvalidPlaneAtInfinityExceptionTest {
    
    public InvalidPlaneAtInfinityExceptionTest() { }
    
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
        InvalidPlaneAtInfinityException ex;
        assertNotNull(ex = new InvalidPlaneAtInfinityException());
        
        ex = null;
        assertNotNull(ex = new InvalidPlaneAtInfinityException("message"));
        
        ex = null;
        assertNotNull(ex = new InvalidPlaneAtInfinityException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new InvalidPlaneAtInfinityException("message", 
                new Exception()));
    }
}
