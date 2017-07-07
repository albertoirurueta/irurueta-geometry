/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.RotationException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 9, 2012
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class RotationExceptionTest {
    
    public RotationExceptionTest() {
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
        RotationException ex;
        assertNotNull(ex = new RotationException());
        
        ex = null;
        assertNotNull(ex = new RotationException("message"));
        
        ex = null;
        assertNotNull(ex = new RotationException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new RotationException("message", 
                new Exception()));
    }
}
