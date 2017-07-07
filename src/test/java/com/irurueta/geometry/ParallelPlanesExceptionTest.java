/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.ParallelPlanesException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class ParallelPlanesExceptionTest {
    
    public ParallelPlanesExceptionTest() {
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
        ParallelPlanesException ex;
        assertNotNull(ex = new ParallelPlanesException());
        
        ex = null;
        assertNotNull(ex = new ParallelPlanesException("message"));
        
        ex = null;
        assertNotNull(ex = new ParallelPlanesException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new ParallelPlanesException("message", 
                new Exception()));
    }            
}
