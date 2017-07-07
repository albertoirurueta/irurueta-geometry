/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.InvalidFundamentalMatrixException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 12, 2015
 */
package com.irurueta.geometry.epipolar;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class InvalidFundamentalMatrixExceptionTest {
    
    public InvalidFundamentalMatrixExceptionTest() {}
    
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
        InvalidFundamentalMatrixException ex;
        assertNotNull(ex = new InvalidFundamentalMatrixException());
        
        ex = null;
        assertNotNull(ex = new InvalidFundamentalMatrixException("message"));
        
        ex = null;
        assertNotNull(ex = new InvalidFundamentalMatrixException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new InvalidFundamentalMatrixException("message",
                new Exception()));
    }
}
