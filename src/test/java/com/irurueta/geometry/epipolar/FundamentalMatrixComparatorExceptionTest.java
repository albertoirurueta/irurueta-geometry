/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.FundamentalMatrixComparatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 28, 2015
 */
package com.irurueta.geometry.epipolar;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class FundamentalMatrixComparatorExceptionTest {
    
    public FundamentalMatrixComparatorExceptionTest() {}
    
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
        FundamentalMatrixComparatorException ex;
        assertNotNull(ex = new FundamentalMatrixComparatorException());
        
        ex = null;
        assertNotNull(ex = new FundamentalMatrixComparatorException("message"));
        
        ex = null;
        assertNotNull(ex = new FundamentalMatrixComparatorException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new FundamentalMatrixComparatorException("message",
                new Exception()));
    }
}
