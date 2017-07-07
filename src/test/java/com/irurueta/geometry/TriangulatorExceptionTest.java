/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry. TriangulatorException
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

public class TriangulatorExceptionTest {
    
    public TriangulatorExceptionTest() {
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
        TriangulatorException ex;
        assertNotNull(ex = new TriangulatorException());
        
        ex = null;
        assertNotNull(ex = new TriangulatorException("message"));
        
        ex = null;
        assertNotNull(ex = new TriangulatorException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new TriangulatorException("message", 
                new Exception()));
    }    
}
