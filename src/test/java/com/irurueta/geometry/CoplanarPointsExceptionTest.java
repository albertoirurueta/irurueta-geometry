/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.CoplanarPointsException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date February 26, 2015
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class CoplanarPointsExceptionTest {
    
    public CoplanarPointsExceptionTest() {
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
        CoplanarPointsException ex;
        assertNotNull(ex = new CoplanarPointsException());
        
        ex = null;
        assertNotNull(ex = new CoplanarPointsException("message"));
        
        ex = null;
        assertNotNull(ex = new CoplanarPointsException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new CoplanarPointsException("message", 
                new Exception()));
    }
}
