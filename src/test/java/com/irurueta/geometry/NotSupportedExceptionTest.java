/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.NotSupportedException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com9
 * @date March 16, 2015
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class NotSupportedExceptionTest {
    
    public NotSupportedExceptionTest() {
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
        NotSupportedException ex;
        assertNotNull(ex = new NotSupportedException());
        
        ex = null;
        assertNotNull(ex = new NotSupportedException("message"));
        
        ex = null;
        assertNotNull(ex = new NotSupportedException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new NotSupportedException("message", 
                new Exception()));        
    }
}
