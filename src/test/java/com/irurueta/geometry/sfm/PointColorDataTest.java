/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.PointColorData
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 28, 2016.
 */
package com.irurueta.geometry.sfm;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class PointColorDataTest {
    
    public PointColorDataTest() { }
    
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
        PointColorData data = new CustomPointColorData();
        
        //check default value
        assertNull(data.getId());
        assertEquals(data.getQualityScore(), 
                PointColorData.DEFAULT_QUALITY_SCORE, 0.0);
    }
    
    @Test
    public void testGetSetId() {
        PointColorData data = new CustomPointColorData();
        
        //check default value
        assertNull(data.getId());
        
        //set new value
        data.setId("id");
        
        //check correctness
        assertEquals(data.getId(), "id");
    }
    
    @Test
    public void testGetSetQualityScore() {
        PointColorData data = new CustomPointColorData();
        
        //check default value
        assertEquals(data.getQualityScore(), 
                PointColorData.DEFAULT_QUALITY_SCORE, 0.0);
        
        //set new value
        data.setQualityScore(5.0);
        
        //check correctness
        assertEquals(data.getQualityScore(), 5.0, 0.0);                
    }
    
    @Test
    public void testAverage() {
        PointColorData data1 = new CustomPointColorData();
        PointColorData data2 = new CustomPointColorData();
        PointColorData data3 = new CustomPointColorData();
        
        data1.average(data2, data3);
        
        assertNotNull(data1);
        assertNotNull(data2);
        assertNotNull(data3);
    }
    
    public static class CustomPointColorData extends PointColorData {

        @Override
        public void average(PointColorData other, PointColorData result) { }        
    }
}
