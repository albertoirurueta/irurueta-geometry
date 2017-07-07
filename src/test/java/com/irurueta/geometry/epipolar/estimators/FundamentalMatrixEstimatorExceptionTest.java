/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.estimators.FundamentalMatrixEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date April 12, 2015
 */
package com.irurueta.geometry.epipolar.estimators;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class FundamentalMatrixEstimatorExceptionTest {
    
    public FundamentalMatrixEstimatorExceptionTest() { }
    
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
        FundamentalMatrixEstimatorException ex;
        assertNotNull(ex = new FundamentalMatrixEstimatorException());
        
        ex = null;
        assertNotNull(ex = new FundamentalMatrixEstimatorException("message"));
        
        ex = null;
        assertNotNull(ex = new FundamentalMatrixEstimatorException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new FundamentalMatrixEstimatorException("message",
                new Exception()));
    }
}
