/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.InvalidPairOfIntrinsicParametersException
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

public class InvalidPairOfIntrinsicParametersExceptionTest {
    
    public InvalidPairOfIntrinsicParametersExceptionTest() {}
    
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
        InvalidPairOfIntrinsicParametersException ex;
        assertNotNull(ex = new InvalidPairOfIntrinsicParametersException());
        
        ex = null;
        assertNotNull(ex = new InvalidPairOfIntrinsicParametersException(
                "message"));
        
        ex = null;
        assertNotNull(ex = new InvalidPairOfIntrinsicParametersException(
                new Exception()));
        
        ex = null;
        assertNotNull(ex = new InvalidPairOfIntrinsicParametersException(
                "message", new Exception()));
    }
}
