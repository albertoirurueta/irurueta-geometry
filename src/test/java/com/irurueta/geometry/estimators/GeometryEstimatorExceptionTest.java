/**
 * @file
 * This file contians unit tests for
 * com.irurueta.geometry.estimators.GeometryEstimatorException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 19, 2013
 */
package com.irurueta.geometry.estimators;

import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class GeometryEstimatorExceptionTest {
    
    public GeometryEstimatorExceptionTest() {
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
        GeometryEstimatorException ex;
        assertNotNull(ex = new GeometryEstimatorException());
        
        ex = null;
        assertNotNull(ex = new GeometryEstimatorException("message"));
        
        ex = null;
        assertNotNull(ex = new GeometryEstimatorException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new GeometryEstimatorException("message", 
                new Exception()));
    }
}
