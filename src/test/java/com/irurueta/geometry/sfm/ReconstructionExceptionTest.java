/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.ReconstructionException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 28, 2016.
 */
package com.irurueta.geometry.sfm;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class ReconstructionExceptionTest {
    
    public ReconstructionExceptionTest() { }
    
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
        ReconstructionException ex;
        assertNotNull(ex = new ReconstructionException());
        
        ex = null;
        assertNotNull(ex = new ReconstructionException("message"));
        
        ex = null;
        assertNotNull(ex = new ReconstructionException("message", 
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new ReconstructionException(new Exception()));
    }
}
