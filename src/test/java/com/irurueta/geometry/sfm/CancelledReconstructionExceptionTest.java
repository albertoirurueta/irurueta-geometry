/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.CancelledReconstructionException
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

public class CancelledReconstructionExceptionTest {
    
    public CancelledReconstructionExceptionTest() { }
    
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
        CancelledReconstructionException ex;
        assertNotNull(ex = new CancelledReconstructionException());
        
        ex = null;
        assertNotNull(ex = new CancelledReconstructionException("message"));
        
        ex = null;
        assertNotNull(ex = new CancelledReconstructionException("message",
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new CancelledReconstructionException(
                new Exception()));
    }
}
