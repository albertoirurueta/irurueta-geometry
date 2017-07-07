/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.UndefinedPointException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 14, 2012
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class UndefinedPointExceptionTest {
    
    public UndefinedPointExceptionTest() {
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
        UndefinedPointException ex;
        assertNotNull(ex = new UndefinedPointException());
        
        ex = null;
        assertNotNull(ex = new UndefinedPointException("message"));
        
        ex = null;
        assertNotNull(ex = new UndefinedPointException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new UndefinedPointException("message", 
                new Exception()));
    }
}
