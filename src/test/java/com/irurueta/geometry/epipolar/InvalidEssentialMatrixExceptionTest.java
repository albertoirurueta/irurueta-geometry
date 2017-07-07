/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.InvalidEssentialMatrixException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 25, 2015
 */
package com.irurueta.geometry.epipolar;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class InvalidEssentialMatrixExceptionTest {
    
    public InvalidEssentialMatrixExceptionTest() {}
    
    @BeforeClass
    public static void setUpClass() {}
    
    @AfterClass
    public static void tearDownClass() {}
    
    @Before
    public void setUp() {}
    
    @After
    public void tearDown() {}

    @Test
    public void testConstructor(){
        InvalidEssentialMatrixException ex;
        assertNotNull(ex = new InvalidEssentialMatrixException());
        
        ex = null;
        assertNotNull(ex = new InvalidEssentialMatrixException(
                "message"));
        
        ex = null;
        assertNotNull(ex = new InvalidEssentialMatrixException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new InvalidEssentialMatrixException(
                "message", new Exception()));
    }
}
