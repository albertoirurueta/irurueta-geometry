/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.InvalidPinholeCameraIntrinsicParametersException
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date November 5, 2012
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class InvalidPinholeCameraIntrinsicParametersExceptionTest {
    
    public InvalidPinholeCameraIntrinsicParametersExceptionTest() {
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
        InvalidPinholeCameraIntrinsicParametersException ex;
        
        ex = new InvalidPinholeCameraIntrinsicParametersException();
        assertNotNull(ex);
        
        ex = new InvalidPinholeCameraIntrinsicParametersException("message");
        assertNotNull(ex);
        
        ex = new InvalidPinholeCameraIntrinsicParametersException(
                new Exception());
        assertNotNull(ex);
        
        ex = new InvalidPinholeCameraIntrinsicParametersException("message", 
                new Exception());
        assertNotNull(ex);
    }
    // TODO add test methods here.
    // The methods must be annotated with annotation @Test. For example:
    //
    // @Test
    // public void hello() {}
}
