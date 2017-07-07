/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.epipolar.InvalidPairOfCamerasException
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

public class InvalidPairOfCamerasExceptionTest {
    
    public InvalidPairOfCamerasExceptionTest() {}
    
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
        InvalidPairOfCamerasException ex;
        assertNotNull(ex = new InvalidPairOfCamerasException());
        
        ex = null;
        assertNotNull(ex = new InvalidPairOfCamerasException("message"));
        
        ex = null;
        assertNotNull(ex = new InvalidPairOfCamerasException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new InvalidPairOfCamerasException("message",
                new Exception()));
    }
}
