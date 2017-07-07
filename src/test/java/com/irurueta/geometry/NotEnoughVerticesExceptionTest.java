/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.NotEnoughVerticesException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date September 17, 2012
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class NotEnoughVerticesExceptionTest {
    
    public NotEnoughVerticesExceptionTest() {
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
        NotEnoughVerticesException ex;
        assertNotNull(ex = new NotEnoughVerticesException());
        
        ex = null;
        assertNotNull(ex = new NotEnoughVerticesException("message"));
        
        ex = null;
        assertNotNull(ex = new NotEnoughVerticesException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new NotEnoughVerticesException("message", 
                new Exception()));
    }        
}
