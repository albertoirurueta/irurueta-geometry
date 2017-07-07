/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.EstimatedCamera
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 28, 2016.
 */
package com.irurueta.geometry.sfm;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.PinholeCamera;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class EstimatedCameraTest {
    
    public EstimatedCameraTest() { }
    
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
        EstimatedCamera ec = new EstimatedCamera();
        
        //check default values
        assertNull(ec.getId());
        assertNull(ec.getCamera());
        assertEquals(ec.getQualityScore(), 
                EstimatedCamera.DEFAULT_QUALITY_SCORE, 0.0);
        assertNull(ec.getCovariance());
    }
    
    @Test
    public void testGetSetId() {
        EstimatedCamera ec = new EstimatedCamera();
        
        //check default value
        assertNull(ec.getId());
        
        //set new value
        ec.setId("id");
        
        //check correctness
        assertEquals(ec.getId(), "id");
    }

    @Test
    public void testGetSetCamera() {
        EstimatedCamera ec = new EstimatedCamera();
        
        //check default value
        assertNull(ec.getCamera());
        
        //set new value
        PinholeCamera camera = new PinholeCamera();
        ec.setCamera(camera);
        
        //check correctness
        assertSame(ec.getCamera(), camera);
    }
    
    @Test
    public void testGetSetQualityScore() {
        EstimatedCamera ec = new EstimatedCamera();
        
        //check default value
        assertEquals(ec.getQualityScore(), 
                EstimatedCamera.DEFAULT_QUALITY_SCORE, 0.0);
        
        //set new value
        ec.setQualityScore(5.0);
        
        //check correctness
        assertEquals(ec.getQualityScore(), 5.0, 0.0);
    }
    
    @Test
    public void testGetSetCovariance() throws WrongSizeException {
        EstimatedCamera ec = new EstimatedCamera();
        
        //check default value
        assertNull(ec.getCovariance());
        
        //set new value
        Matrix cov = new Matrix(12, 12);
        ec.setCovariance(cov);
        
        //check correctness
        assertSame(ec.getCovariance(), cov);
    }
}
