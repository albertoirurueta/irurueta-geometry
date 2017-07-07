/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.NoIntersectionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 13, 2012
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class NoIntersectionExceptionTest {
    
    public NoIntersectionExceptionTest() {
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
        NoIntersectionException ex;
        assertNotNull(ex = new NoIntersectionException());
        
        ex = null;
        assertNotNull(ex = new NoIntersectionException("message"));
        
        ex = null;
        assertNotNull(ex = new NoIntersectionException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new NoIntersectionException("message", 
                new Exception()));
    }    
}
