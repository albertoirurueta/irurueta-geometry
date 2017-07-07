/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.FailedReconstructionException
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

public class FailedReconstructionExceptionTest {
    
    public FailedReconstructionExceptionTest() { }
    
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
        FailedReconstructionException ex;
        assertNotNull(ex = new FailedReconstructionException());
        
        ex = null;
        assertNotNull(ex = new FailedReconstructionException("message"));
        
        ex = null;
        assertNotNull(ex = new FailedReconstructionException("message",
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new FailedReconstructionException(new Exception()));
    }
}
