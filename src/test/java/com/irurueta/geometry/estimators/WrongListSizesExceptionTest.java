/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.estimators.WrongListSizesException
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

public class WrongListSizesExceptionTest {
    
    public WrongListSizesExceptionTest() {
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
        WrongListSizesException ex;
        assertNotNull(ex = new WrongListSizesException());
        
        ex = null;
        assertNotNull(ex = new WrongListSizesException("message"));
        
        ex = null;
        assertNotNull(ex = new WrongListSizesException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new WrongListSizesException("message", 
                new Exception()));
    }
}
