/**
 * @file
 * This file contains Unit Tests for
 * com.irurueta.geometry.InvalidRotationMatrixException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date August 15, 2012
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class InvalidRotationMatrixExceptionTest {
    
    public InvalidRotationMatrixExceptionTest() {
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
        InvalidRotationMatrixException ex;
        assertNotNull(ex = new InvalidRotationMatrixException());
        
        ex = null;
        assertNotNull(ex = new InvalidRotationMatrixException("message"));
        
        ex = null;
        assertNotNull(ex = new InvalidRotationMatrixException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new InvalidRotationMatrixException("message", 
                new Exception()));
    }        
}
