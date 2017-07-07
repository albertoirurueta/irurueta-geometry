/**
 * @file
 * This file contains unit tests for
 * com.irurueta.geometry.sfm.MatchedSamples
 * 
 * @author Alberto Irurueta (alberto@irurueta.com)
 * @date December 28, 2016.
 */
package com.irurueta.geometry.sfm;

import java.util.BitSet;
import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import static org.junit.Assert.*;

public class MatchedSamplesTest {
    
    public MatchedSamplesTest() { }
    
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
        MatchedSamples samples = new MatchedSamples();
        
        //check default values
        assertNull(samples.getSamples());
        assertNull(samples.getCameras());
        assertNull(samples.getViewIds());
        assertNull(samples.getReconstructedPoint());
        assertEquals(samples.getQualityScore(), 
                MatchedSamples.DEFAULT_QUALITY_SCORE, 0.0);
        assertNull(samples.getInliers());
    }
    
    @Test
    public void testGetSetSamples() {
        MatchedSamples samples = new MatchedSamples();
        
        //check default value
        assertNull(samples.getSamples());
        
        //set new value
        Sample2D[] s = new Sample2D[1];
        samples.setSamples(s);
        
        //check correctness
        assertSame(samples.getSamples(), s);
    }
    
    @Test
    public void testGetSetCameras() {
        MatchedSamples samples = new MatchedSamples();
        
        //check default value
        assertNull(samples.getCameras());
        
        //set new value
        EstimatedCamera[] cams = new EstimatedCamera[1];
        samples.setCameras(cams);
        
        //check correctness
        assertSame(samples.getCameras(), cams);
    }
    
    @Test
    public void testGetSetViewIds() {
        MatchedSamples samples = new MatchedSamples();
        
        //check default value
        assertNull(samples.getViewIds());
        
        //set new value
        int[] ids = new int[1];
        samples.setViewIds(ids);
        
        //check correctness
        assertSame(samples.getViewIds(), ids);
    }
    
    @Test
    public void testGetSetReconstructedPoint() {
        MatchedSamples samples = new MatchedSamples();
        
        //check default value
        assertNull(samples.getReconstructedPoint());
        
        //set new value
        ReconstructedPoint3D rp = new ReconstructedPoint3D();
        samples.setReconstructedPoint(rp);
        
        //check correctness
        assertSame(samples.getReconstructedPoint(), rp);
    }
    
    @Test
    public void testGetSetQualityScore() {
        MatchedSamples samples = new MatchedSamples();
        
        //check default value
        assertEquals(samples.getQualityScore(), 
                MatchedSamples.DEFAULT_QUALITY_SCORE, 0.0);
        
        //set new value
        samples.setQualityScore(20.0);
        
        //check correctness
        assertEquals(samples.getQualityScore(), 20.0, 0.0);
    }
    
    @Test
    public void testGetSetInliers() {
        MatchedSamples samples = new MatchedSamples();
        
        //check default value
        assertNull(samples.getInliers());
        
        //set new value
        BitSet inliers = new BitSet();
        samples.setInliers(inliers);
        
        //check correctness
        assertSame(samples.getInliers(), inliers);
    }
}
