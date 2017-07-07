/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.irurueta.geometry;

import org.junit.After;
import org.junit.AfterClass;
import static org.junit.Assert.*;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

public class NotLocusExceptionTest {
    
    public NotLocusExceptionTest() {
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
        NotLocusException ex;
        assertNotNull(ex = new NotLocusException());
        
        ex = null;
        assertNotNull(ex = new NotLocusException("message"));
        
        ex = null;
        assertNotNull(ex = new NotLocusException(new Exception()));
        
        ex = null;
        assertNotNull(ex = new NotLocusException("message", new Exception()));
    }
}
