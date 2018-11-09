/*
 * Copyright (C) 2012 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.geometry;

import com.irurueta.algebra.*;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.Random;

import static org.junit.Assert.*;

public class DualConicTest {
    
    private static final double MIN_RANDOM_VALUE = -10.0;
    private static final double MAX_RANDOM_VALUE = 10.0;
    private static final double PRECISION_ERROR = 1e-8;
    private static final double LOCUS_THRESHOLD = 1e-8;
    private static final double PERPENDICULAR_THRESHOLD = 1e-6;
    
    private static final int DUAL_CONIC_ROWS = 3;
    private static final int DUAL_CONIC_COLS = 3;
    private static final int HOM_COORDS = 3;
    
    private static final int TIMES = 10;
    
    public DualConicTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }
    
    @Before
    public void setUp() { }
    
    @After
    public void tearDown() { }
    
    @Test
    public void testConstructor() throws WrongSizeException, 
            IllegalArgumentException, NonSymmetricMatrixException,
            DecomposerException, CoincidentLinesException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        //Constructor
        DualConic dualConic = new DualConic();
        assertEquals(dualConic.getA(), 0.0, 0.0);
        assertEquals(dualConic.getB(), 0.0, 0.0);
        assertEquals(dualConic.getC(), 0.0, 0.0);
        assertEquals(dualConic.getD(), 0.0, 0.0);
        assertEquals(dualConic.getE(), 0.0, 0.0);
        assertEquals(dualConic.getF(), 0.0, 0.0);
        assertFalse(dualConic.isNormalized());
        
        //Constructor with params
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        dualConic = new DualConic(a, b, c, d, e, f);
        assertEquals(dualConic.getA(), a, 0.0);
        assertEquals(dualConic.getB(), b, 0.0);
        assertEquals(dualConic.getC(), c, 0.0);
        assertEquals(dualConic.getD(), d, 0.0);
        assertEquals(dualConic.getE(), e, 0.0);
        assertEquals(dualConic.getF(), f, 0.0);
        assertFalse(dualConic.isNormalized());
        
        //Constructor using matrix
        Matrix m = new Matrix(DUAL_CONIC_ROWS, DUAL_CONIC_COLS);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);
        dualConic = new DualConic(m);
        assertEquals(dualConic.getA(), m.getElementAt(0, 0), 0.0);
        assertEquals(dualConic.getB(), m.getElementAt(0, 1), 0.0);
        assertEquals(dualConic.getC(), m.getElementAt(1, 1), 0.0);
        assertEquals(dualConic.getD(), m.getElementAt(0, 2), 0.0);
        assertEquals(dualConic.getE(), m.getElementAt(1, 2), 0.0);
        assertEquals(dualConic.getF(), m.getElementAt(2, 2), 0.0);
        
        //Constructor using matrix with wrong size
        m = new Matrix(DUAL_CONIC_ROWS + 1, DUAL_CONIC_COLS);
        dualConic = null;
        try {
            dualConic = new DualConic(m);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(dualConic);
        
        //Constructor using non-symmetric matrix
        m = new Matrix(DUAL_CONIC_ROWS, DUAL_CONIC_COLS);
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b + 1.0);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e + 1.0);
        m.setElementAt(2, 0, d + 1.0);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);
        dualConic = null;
        try {
            dualConic = new DualConic(m);
            fail("NonSymmetricMatrixException expected but not thrown");
        } catch (NonSymmetricMatrixException ignore) { }
        assertNull(dualConic);
        
        
        //Constructor from 5 lines
        m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        Line2D line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1), 
                m.getElementAt(0, 2));
        Line2D line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        Line2D line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2));
        Line2D line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2));
        Line2D line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2));
        
        line1.normalize();
        line2.normalize();
        line3.normalize();
        line4.normalize();
        line5.normalize();
                
        
        //estimate dual conic that lies inside of provided 5 lines
        Matrix m2 = new Matrix(5, 6);
        
        double l1 = line1.getA();
        double l2 = line1.getB();
        double l3 = line1.getC();
        m2.setElementAt(0, 0, l1 * l1);
        m2.setElementAt(0, 1, 2.0 * l1 * l2);
        m2.setElementAt(0, 2, l2 * l2);
        m2.setElementAt(0, 3, 2.0 * l1 * l3);
        m2.setElementAt(0, 4, 2.0 * l2 * l3);
        m2.setElementAt(0, 5, l3 * l3);
        
        l1 = line2.getA();
        l2 = line2.getB();
        l3 = line2.getC();
        m2.setElementAt(1, 0, l1 * l1);
        m2.setElementAt(1, 1, 2.0 * l1 * l2);
        m2.setElementAt(1, 2, l2 * l2);
        m2.setElementAt(1, 3, 2.0 * l1 * l3);
        m2.setElementAt(1, 4, 2.0 * l2 * l3);
        m2.setElementAt(1, 5, l3 * l3);
        
        l1 = line3.getA();
        l2 = line3.getB();
        l3 = line3.getC();
        m2.setElementAt(2, 0, l1 * l1);
        m2.setElementAt(2, 1, 2.0 * l1 * l2);
        m2.setElementAt(2, 2, l2 * l2);
        m2.setElementAt(2, 3, 2.0 * l1 * l3);
        m2.setElementAt(2, 4, 2.0 * l2 * l3);
        m2.setElementAt(2, 5, l3 * l3);

        l1 = line4.getA();
        l2 = line4.getB();
        l3 = line4.getC();
        m2.setElementAt(3, 0, l1 * l1);
        m2.setElementAt(3, 1, 2.0 * l1 * l2);
        m2.setElementAt(3, 2, l2 * l2);
        m2.setElementAt(3, 3, 2.0 * l1 * l3);
        m2.setElementAt(3, 4, 2.0 * l2 * l3);
        m2.setElementAt(3, 5, l3 * l3);
        
        l1 = line5.getA();
        l2 = line5.getB();
        l3 = line5.getC();
        m2.setElementAt(4, 0, l1 * l1);
        m2.setElementAt(4, 1, 2.0 * l1 * l2);
        m2.setElementAt(4, 2, l2 * l2);
        m2.setElementAt(4, 3, 2.0 * l1 * l3);
        m2.setElementAt(4, 4, 2.0 * l2 * l3);
        m2.setElementAt(4, 5, l3 * l3);
        
        while (com.irurueta.algebra.Utils.rank(m2) < 5) {
            m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1), 
                    m.getElementAt(0, 2));
            line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2));
        
            line1.normalize();
            line2.normalize();
            line3.normalize();
            line4.normalize();
            line5.normalize();
                
                
            l1 = line1.getA();
            l2 = line1.getB();
            l3 = line1.getC();
            m2.setElementAt(0, 0, l1 * l1);
            m2.setElementAt(0, 1, 2.0 * l1 * l2);
            m2.setElementAt(0, 2, l2 * l2);
            m2.setElementAt(0, 3, 2.0 * l1 * l3);
            m2.setElementAt(0, 4, 2.0 * l2 * l3);
            m2.setElementAt(0, 5, l3 * l3);
        
            l1 = line2.getA();
            l2 = line2.getB();
            l3 = line2.getC();
            m2.setElementAt(1, 0, l1 * l1);
            m2.setElementAt(1, 1, 2.0 * l1 * l2);
            m2.setElementAt(1, 2, l2 * l2);
            m2.setElementAt(1, 3, 2.0 * l1 * l3);
            m2.setElementAt(1, 4, 2.0 * l2 * l3);
            m2.setElementAt(1, 5, l3 * l3);
        
            l1 = line3.getA();
            l2 = line3.getB();
            l3 = line3.getC();
            m2.setElementAt(2, 0, l1 * l1);
            m2.setElementAt(2, 1, 2.0 * l1 * l2);
            m2.setElementAt(2, 2, l2 * l2);
            m2.setElementAt(2, 3, 2.0 * l1 * l3);
            m2.setElementAt(2, 4, 2.0 * l2 * l3);
            m2.setElementAt(2, 5, l3 * l3);

            l1 = line4.getA();
            l2 = line4.getB();
            l3 = line4.getC();
            m2.setElementAt(3, 0, l1 * l1);
            m2.setElementAt(3, 1, 2.0 * l1 * l2);
            m2.setElementAt(3, 2, l2 * l2);
            m2.setElementAt(3, 3, 2.0 * l1 * l3);
            m2.setElementAt(3, 4, 2.0 * l2 * l3);
            m2.setElementAt(3, 5, l3 * l3);
        
            l1 = line5.getA();
            l2 = line5.getB();
            l3 = line5.getC();
            m2.setElementAt(4, 0, l1 * l1);
            m2.setElementAt(4, 1, 2.0 * l1 * l2);
            m2.setElementAt(4, 2, l2 * l2);
            m2.setElementAt(4, 3, 2.0 * l1 * l3);
            m2.setElementAt(4, 4, 2.0 * l2 * l3);
            m2.setElementAt(4, 5, l3 * l3);
        }   
        
        dualConic = new DualConic(line1, line2, line3, line4, line5);
        assertTrue(dualConic.isLocus(line1, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line2, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line3, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line4, PRECISION_ERROR));
        assertTrue(dualConic.isLocus(line5, PRECISION_ERROR));
        
        //Force CoincidentLinesException
        dualConic = null;
        try {
            dualConic = new DualConic(line1, line2, line3, line4, line4);
            fail("CoincidentLinesException expected but not thrown");
        } catch (CoincidentLinesException ignore) { }
        assertNull(dualConic);
    }
    
    @Test
    public void testGettersAndSetters() throws WrongSizeException, 
            IllegalArgumentException, NonSymmetricMatrixException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        DualConic dualConic = new DualConic();
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        dualConic.setA(a);
        dualConic.setB(b);
        dualConic.setC(c);
        dualConic.setD(d);
        dualConic.setE(e);
        dualConic.setF(f);
        assertEquals(dualConic.getA(), a, 0.0);
        assertEquals(dualConic.getB(), b, 0.0);
        assertEquals(dualConic.getC(), c, 0.0);
        assertEquals(dualConic.getD(), d, 0.0);
        assertEquals(dualConic.getE(), e, 0.0);
        assertEquals(dualConic.getF(), f, 0.0);
        
        dualConic = new DualConic();
        Matrix m = new Matrix(DUAL_CONIC_ROWS, DUAL_CONIC_COLS);
        a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);
        dualConic.setParameters(m);
        assertEquals(dualConic.getA(), m.getElementAt(0, 0), 0.0);
        assertEquals(dualConic.getB(), m.getElementAt(0, 1), 0.0);
        assertEquals(dualConic.getC(), m.getElementAt(1, 1), 0.0);
        assertEquals(dualConic.getD(), m.getElementAt(0, 2), 0.0);
        assertEquals(dualConic.getE(), m.getElementAt(1, 2), 0.0);
        assertEquals(dualConic.getF(), m.getElementAt(2, 2), 0.0);        
    }
    
    @Test
    public void testAsMatrix() throws WrongSizeException, 
            IllegalArgumentException, NonSymmetricMatrixException {
        
        UniformRandomizer randomizer = new UniformRandomizer(new Random());
        
        DualConic dualConic = new DualConic();
        Matrix m = new Matrix(DUAL_CONIC_ROWS, DUAL_CONIC_COLS);
        double a = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double b = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double c = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double d = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double e = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        double f = randomizer.nextDouble(MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        m.setElementAt(0, 0, a);
        m.setElementAt(0, 1, b);
        m.setElementAt(0, 2, d);
        m.setElementAt(1, 0, b);
        m.setElementAt(1, 1, c);
        m.setElementAt(1, 2, e);
        m.setElementAt(2, 0, d);
        m.setElementAt(2, 1, e);
        m.setElementAt(2, 2, f);
        dualConic.setParameters(m);
        Matrix m2 = dualConic.asMatrix();
        
        assertTrue(m.equals(m2, PRECISION_ERROR));        
    }
    
    @Test
    public void testIsLocus() throws WrongSizeException, DecomposerException, 
            RankDeficientMatrixException, IllegalArgumentException,
            NonSymmetricMatrixException, NotReadyException, LockedException,
            com.irurueta.algebra.NotAvailableException {
        
        Matrix m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

        Line2D line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1), 
                m.getElementAt(0, 2));
        Line2D line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                m.getElementAt(1, 2));
        Line2D line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                m.getElementAt(2, 2));
        Line2D line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                m.getElementAt(3, 2));
        Line2D line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                m.getElementAt(4, 2));
        
        line1.normalize();
        line2.normalize();
        line3.normalize();
        line4.normalize();
        line5.normalize();
                
        
        //estimate dual conic that lines inside of provided 5 lines
        Matrix systemOfEquationsMatrix = new Matrix(5, 6);
        
        double l1 = line1.getA();
        double l2 = line1.getB();
        double l3 = line1.getC();
        systemOfEquationsMatrix.setElementAt(0, 0, l1 * l1);
        systemOfEquationsMatrix.setElementAt(0, 1, 2.0 * l1 * l2);
        systemOfEquationsMatrix.setElementAt(0, 2, l2 * l2);
        systemOfEquationsMatrix.setElementAt(0, 3, 2.0 * l1 * l3);
        systemOfEquationsMatrix.setElementAt(0, 4, 2.0 * l2 * l3);
        systemOfEquationsMatrix.setElementAt(0, 5, l3 * l3);
        
        l1 = line2.getA();
        l2 = line2.getB();
        l3 = line2.getC();
        systemOfEquationsMatrix.setElementAt(1, 0, l1 * l1);
        systemOfEquationsMatrix.setElementAt(1, 1, 2.0 * l1 * l2);
        systemOfEquationsMatrix.setElementAt(1, 2, l2 * l2);
        systemOfEquationsMatrix.setElementAt(1, 3, 2.0 * l1 * l3);
        systemOfEquationsMatrix.setElementAt(1, 4, 2.0 * l2 * l3);
        systemOfEquationsMatrix.setElementAt(1, 5, l3 * l3);        

        l1 = line3.getA();
        l2 = line3.getB();
        l3 = line3.getC();
        systemOfEquationsMatrix.setElementAt(2, 0, l1 * l1);
        systemOfEquationsMatrix.setElementAt(2, 1, 2.0 * l1 * l2);
        systemOfEquationsMatrix.setElementAt(2, 2, l2 * l2);
        systemOfEquationsMatrix.setElementAt(2, 3, 2.0 * l1 * l3);
        systemOfEquationsMatrix.setElementAt(2, 4, 2.0 * l2 * l3);
        systemOfEquationsMatrix.setElementAt(2, 5, l3 * l3);            
        
        l1 = line4.getA();
        l2 = line4.getB();
        l3 = line4.getC();
        systemOfEquationsMatrix.setElementAt(3, 0, l1 * l1);
        systemOfEquationsMatrix.setElementAt(3, 1, 2.0 * l1 * l2);
        systemOfEquationsMatrix.setElementAt(3, 2, l2 * l2);
        systemOfEquationsMatrix.setElementAt(3, 3, 2.0 * l1 * l3);
        systemOfEquationsMatrix.setElementAt(3, 4, 2.0 * l2 * l3);
        systemOfEquationsMatrix.setElementAt(3, 5, l3 * l3);            

        l1 = line5.getA();
        l2 = line5.getB();
        l3 = line5.getC();
        systemOfEquationsMatrix.setElementAt(4, 0, l1 * l1);
        systemOfEquationsMatrix.setElementAt(4, 1, 2.0 * l1 * l2);
        systemOfEquationsMatrix.setElementAt(4, 2, l2 * l2);
        systemOfEquationsMatrix.setElementAt(4, 3, 2.0 * l1 * l3);
        systemOfEquationsMatrix.setElementAt(4, 4, 2.0 * l2 * l3);
        systemOfEquationsMatrix.setElementAt(4, 5, l3 * l3);
            
        while (com.irurueta.algebra.Utils.rank(systemOfEquationsMatrix) < 5) {
            m = Matrix.createWithUniformRandomValues(5, HOM_COORDS, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            line1 = new Line2D(m.getElementAt(0, 0), m.getElementAt(0, 1), 
                    m.getElementAt(0, 2));
            line2 = new Line2D(m.getElementAt(1, 0), m.getElementAt(1, 1),
                    m.getElementAt(1, 2));
            line3 = new Line2D(m.getElementAt(2, 0), m.getElementAt(2, 1),
                    m.getElementAt(2, 2));
            line4 = new Line2D(m.getElementAt(3, 0), m.getElementAt(3, 1),
                    m.getElementAt(3, 2));
            line5 = new Line2D(m.getElementAt(4, 0), m.getElementAt(4, 1),
                    m.getElementAt(4, 2));
        
            line1.normalize();
            line2.normalize();
            line3.normalize();
            line4.normalize();
            line5.normalize();
                
                
            l1 = line1.getA();
            l2 = line1.getB();
            l3 = line1.getC();
            systemOfEquationsMatrix.setElementAt(0, 0, l1 * l1);
            systemOfEquationsMatrix.setElementAt(0, 1, 2.0 * l1 * l2);
            systemOfEquationsMatrix.setElementAt(0, 2, l2 * l2);
            systemOfEquationsMatrix.setElementAt(0, 3, 2.0 * l1 * l3);
            systemOfEquationsMatrix.setElementAt(0, 4, 2.0 * l2 * l3);
            systemOfEquationsMatrix.setElementAt(0, 5, l3 * l3);
        
            l1 = line2.getA();
            l2 = line2.getB();
            l3 = line2.getC();
            systemOfEquationsMatrix.setElementAt(1, 0, l1 * l1);
            systemOfEquationsMatrix.setElementAt(1, 1, 2.0 * l1 * l2);
            systemOfEquationsMatrix.setElementAt(1, 2, l2 * l2);
            systemOfEquationsMatrix.setElementAt(1, 3, 2.0 * l1 * l3);
            systemOfEquationsMatrix.setElementAt(1, 4, 2.0 * l2 * l3);
            systemOfEquationsMatrix.setElementAt(1, 5, l3 * l3);        

            l1 = line3.getA();
            l2 = line3.getB();
            l3 = line3.getC();
            systemOfEquationsMatrix.setElementAt(2, 0, l1 * l1);
            systemOfEquationsMatrix.setElementAt(2, 1, 2.0 * l1 * l2);
            systemOfEquationsMatrix.setElementAt(2, 2, l2 * l2);
            systemOfEquationsMatrix.setElementAt(2, 3, 2.0 * l1 * l3);
            systemOfEquationsMatrix.setElementAt(2, 4, 2.0 * l2 * l3);
            systemOfEquationsMatrix.setElementAt(2, 5, l3 * l3);            
        
            l1 = line4.getA();
            l2 = line4.getB();
            l3 = line4.getC();
            systemOfEquationsMatrix.setElementAt(3, 0, l1 * l1);
            systemOfEquationsMatrix.setElementAt(3, 1, 2.0 * l1 * l2);
            systemOfEquationsMatrix.setElementAt(3, 2, l2 * l2);
            systemOfEquationsMatrix.setElementAt(3, 3, 2.0 * l1 * l3);
            systemOfEquationsMatrix.setElementAt(3, 4, 2.0 * l2 * l3);
            systemOfEquationsMatrix.setElementAt(3, 5, l3 * l3);            

            l1 = line5.getA();
            l2 = line5.getB();
            l3 = line5.getC();
            systemOfEquationsMatrix.setElementAt(4, 0, l1 * l1);
            systemOfEquationsMatrix.setElementAt(4, 1, 2.0 * l1 * l2);
            systemOfEquationsMatrix.setElementAt(4, 2, l2 * l2);
            systemOfEquationsMatrix.setElementAt(4, 3, 2.0 * l1 * l3);
            systemOfEquationsMatrix.setElementAt(4, 4, 2.0 * l2 * l3);
            systemOfEquationsMatrix.setElementAt(4, 5, l3 * l3);
        }
        
        SingularValueDecomposer decomposer = new SingularValueDecomposer(
                systemOfEquationsMatrix);
        decomposer.decompose();
            
        Matrix V = decomposer.getV();
                    
        double a = V.getElementAt(0, 5);
        double b = V.getElementAt(1, 5);
        double c = V.getElementAt(2, 5);
        double d = V.getElementAt(3, 5);
        double e = V.getElementAt(4, 5);
        double f = V.getElementAt(5, 5);
            
        Matrix dualConicLine = new Matrix(HOM_COORDS, 1);
        dualConicLine.setElementAt(0, 0, line1.getA());
        dualConicLine.setElementAt(1, 0, line1.getB());
        dualConicLine.setElementAt(2, 0, line1.getC());
        
        double norm = com.irurueta.algebra.Utils.normF(dualConicLine);
        dualConicLine.multiplyByScalar(1.0 / norm);            
        
        Matrix dualConicMatrix = new Matrix(DUAL_CONIC_ROWS, DUAL_CONIC_COLS);
        dualConicMatrix.setElementAt(0, 0, a);
        dualConicMatrix.setElementAt(0, 1, b);
        dualConicMatrix.setElementAt(0, 2, d);
        dualConicMatrix.setElementAt(1, 0, b);
        dualConicMatrix.setElementAt(1, 1, c);
        dualConicMatrix.setElementAt(1, 2, e);
        dualConicMatrix.setElementAt(2, 0, d);
        dualConicMatrix.setElementAt(2, 1, e);
        dualConicMatrix.setElementAt(2, 2, f);
        
        norm = com.irurueta.algebra.Utils.normF(dualConicMatrix);
        dualConicMatrix.multiplyByScalar(1.0 / norm);
        
        //find point where line is tangent to conic
        Matrix homPointMatrix = dualConicMatrix.multiplyAndReturnNew(
                dualConicLine);
        
        //add director vector of tangent line to get a point outside of conic
        double directVectorA = dualConicLine.getElementAtIndex(0);
        double directVectorB = dualConicLine.getElementAtIndex(1);
        double directVectorNorm = Math.sqrt(directVectorA * directVectorA +
            directVectorB * directVectorB);
        directVectorA /= directVectorNorm;
        directVectorB /= directVectorNorm;
        homPointMatrix.setElementAtIndex(0, homPointMatrix.getElementAtIndex(0) 
                + directVectorA * homPointMatrix.getElementAtIndex(2));
        homPointMatrix.setElementAtIndex(1, homPointMatrix.getElementAtIndex(1) 
                + directVectorB * homPointMatrix.getElementAtIndex(2));
        
        norm = com.irurueta.algebra.Utils.normF(homPointMatrix);
        homPointMatrix.multiplyByScalar(1.0 / norm);
        
        //get conic matrix by inversing dual conic matrix
        Matrix conicMatrix = com.irurueta.algebra.Utils.inverse(
                dualConicMatrix);
        
        //find line vector outside of dual conic as the product of conic matrix
        //and point outside of conic
        Matrix outsideLineMatrix = conicMatrix.multiplyAndReturnNew(
                homPointMatrix);
        
        //instantiate line outside of dual conic using computed vector
        Line2D outsideLine = new Line2D(outsideLineMatrix.toArray());
        
        //instantiate new dual conic instance
        DualConic dualConic = new DualConic(dualConicMatrix);

        //check that initial 5 lines lie inside the dual conic
        assertTrue(dualConic.isLocus(line1, LOCUS_THRESHOLD));
        assertTrue(dualConic.isLocus(line2, LOCUS_THRESHOLD));
        assertTrue(dualConic.isLocus(line3, LOCUS_THRESHOLD));
        assertTrue(dualConic.isLocus(line4, LOCUS_THRESHOLD));
        assertTrue(dualConic.isLocus(line5, LOCUS_THRESHOLD));
        
        //check line outside of dual conic
        assertFalse(dualConic.isLocus(outsideLine, LOCUS_THRESHOLD));
    }
    
    @Test
    public void testAngleBetweenLines() throws WrongSizeException, 
            IllegalArgumentException, NonSymmetricMatrixException {
        
        //initial lines
        Matrix lineMatrix1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Matrix lineMatrix2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        //transformation matrix
        Matrix transform = Matrix.createWithUniformRandomValues(HOM_COORDS, 
                HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        
        
        //transform lines
        Matrix tLine1 = transform.multiplyAndReturnNew(lineMatrix1);
        Matrix tLine2 = transform.multiplyAndReturnNew(lineMatrix2);
        
        double norm1 = com.irurueta.algebra.Utils.normF(tLine1);
        double norm2 = com.irurueta.algebra.Utils.normF(tLine2);
        
        double numerator = tLine1.transposeAndReturnNew().multiplyAndReturnNew(
                tLine2).getElementAt(0, 0);
        
        double cosAngle = numerator / (norm1 * norm2);
        
        double angle = Math.acos(cosAngle);
        
        //compute dual conic matrix as the product of transposed transform 
        //matrix with itself
        Matrix transposedTransform = transform.transposeAndReturnNew();
        Matrix dualConicMatrix = transposedTransform.multiplyAndReturnNew(
                transform);
        
        //normalize conic matrix
        double normDualConic = com.irurueta.algebra.Utils.normF(dualConicMatrix);
        dualConicMatrix.multiplyByScalar(1.0 / normDualConic);
        
        DualConic dualConic = new DualConic(dualConicMatrix);
        
        Line2D line1 = new Line2D(lineMatrix1.getElementAt(0, 0), 
                lineMatrix1.getElementAt(1, 0), lineMatrix1.getElementAt(2, 0));
        Line2D line2 = new Line2D(lineMatrix2.getElementAt(0, 0), 
                lineMatrix2.getElementAt(1, 0), lineMatrix2.getElementAt(2, 0));
        
        assertEquals(dualConic.angleBetweenLines(line1, line2), angle, 
                PRECISION_ERROR);
    }
    
    @Test
    public void testArePerpendicularLines() throws WrongSizeException, 
            DecomposerException, RankDeficientMatrixException,
            IllegalArgumentException, NonSymmetricMatrixException {
        
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            //trying perpendicular angle
            Matrix matrixLine1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);

            double norm = com.irurueta.algebra.Utils.normF(matrixLine1);
            matrixLine1.multiplyByScalar(1.0 / norm);

            Matrix matrixLine2 = new Matrix(HOM_COORDS, 1);
            matrixLine2.setElementAt(0, 0, matrixLine1.getElementAt(1, 0) + 
                    matrixLine1.getElementAt(2, 0));
            matrixLine2.setElementAt(1, 0, -matrixLine1.getElementAt(0, 0));
            matrixLine2.setElementAt(2, 0, -matrixLine1.getElementAt(0, 0));

            norm = com.irurueta.algebra.Utils.normF(matrixLine2);
            matrixLine2.multiplyByScalar(1.0 / norm);

            Matrix transform = Matrix.createWithUniformRandomValues(HOM_COORDS, 
                    HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            while (com.irurueta.algebra.Utils.rank(transform) < 3) {
                transform = Matrix.createWithUniformRandomValues(HOM_COORDS, 
                        HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            }

            Matrix invTransform = com.irurueta.algebra.Utils.inverse(transform);

            Matrix transformLineMatrix1 = transform.multiplyAndReturnNew(
                    matrixLine1);
            Matrix transformLineMatrix2 = transform.multiplyAndReturnNew(
                    matrixLine2);

            Matrix transInvTransform = invTransform.transposeAndReturnNew();

            Matrix dualConicMatrix = transInvTransform.multiplyAndReturnNew(
                    invTransform);
            norm = com.irurueta.algebra.Utils.normF(dualConicMatrix);
            dualConicMatrix.multiplyByScalar(1.0 / norm);

            Line2D transformLine1 = new Line2D(transformLineMatrix1.toArray());
            Line2D transformLine2 = new Line2D(transformLineMatrix2.toArray());

            DualConic dualConic = new DualConic(dualConicMatrix);

            assertTrue(dualConic.arePerpendicularLines(transformLine1, 
                    transformLine2, PERPENDICULAR_THRESHOLD));

            //trying non-perpendicular points
            double dotProduct;

            matrixLine1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixLine1);
            matrixLine1.multiplyByScalar(1.0 / norm);

            matrixLine2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, 
                    MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
            norm = com.irurueta.algebra.Utils.normF(matrixLine2);
            matrixLine2.multiplyByScalar(1.0 / norm);

            dotProduct = matrixLine1.transposeAndReturnNew().multiplyAndReturnNew(
                    matrixLine2).getElementAt(0, 0);

            //ensure lines are not perpendicular
            while (Math.abs(dotProduct) < PERPENDICULAR_THRESHOLD) {
                matrixLine1 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, 
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixLine1);
                matrixLine1.multiplyByScalar(1.0 / norm);

                matrixLine2 = Matrix.createWithUniformRandomValues(HOM_COORDS, 1, 
                        MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
                norm = com.irurueta.algebra.Utils.normF(matrixLine2);
                matrixLine2.multiplyByScalar(1.0 / norm);

                dotProduct = matrixLine1.transposeAndReturnNew().
                        multiplyAndReturnNew(matrixLine2).getElementAt(0, 0);
            }

            transformLineMatrix1 = transform.multiplyAndReturnNew(matrixLine1);
            transformLineMatrix2 = transform.multiplyAndReturnNew(matrixLine2);

            transformLine1 = new Line2D(transformLineMatrix1.toArray());
            transformLine2 = new Line2D(transformLineMatrix2.toArray());

            if (dualConic.arePerpendicularLines(transformLine1, transformLine2,
                    5.0 * PERPENDICULAR_THRESHOLD)) {
                continue;
            }
            assertFalse(dualConic.arePerpendicularLines(transformLine1,
                    transformLine2, 5.0 * PERPENDICULAR_THRESHOLD));
            numValid++;
        }
        
        assertTrue(numValid > 0);
    }
    
    @Test
    public void testGetConic() throws WrongSizeException, DecomposerException, 
            RankDeficientMatrixException, IllegalArgumentException,
            NonSymmetricMatrixException, ConicNotAvailableException {
        
        Matrix transformMatrix = Matrix.createWithUniformRandomValues(
                HOM_COORDS, HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        while (com.irurueta.algebra.Utils.rank(transformMatrix) != 3) {
            transformMatrix = Matrix.createWithUniformRandomValues(HOM_COORDS, 
                    HOM_COORDS, MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        }
        
        Matrix transformTransposedMatrix = 
                transformMatrix.transposeAndReturnNew();
        
        Matrix dualConicMatrix = transformTransposedMatrix.multiplyAndReturnNew(
                transformMatrix);
        
        Matrix conicMatrix1 = com.irurueta.algebra.Utils.inverse(
                dualConicMatrix);
        
        DualConic dualConic = new DualConic(dualConicMatrix);
        
        Conic conic = dualConic.getConic();
        Matrix conicMatrix2 = conic.asMatrix();
        
        //normalize conic matrices
        double norm = com.irurueta.algebra.Utils.normF(conicMatrix1);
        conicMatrix1.multiplyByScalar(1.0 / norm);
        
        norm = com.irurueta.algebra.Utils.normF(conicMatrix2);
        conicMatrix2.multiplyByScalar(1.0 / norm);
        
        //compute difference of normalized conic matrices
        Matrix diffMatrix = conicMatrix1.subtractAndReturnNew(conicMatrix2);
        
        //ensure that difference matrix is almost zero by checking its norm
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        assertEquals(norm, 0.0, PRECISION_ERROR);
    }
    
    @Test
    public void testNormalize() throws WrongSizeException, 
            IllegalArgumentException, NonSymmetricMatrixException {
        
        Matrix t = Matrix.createWithUniformRandomValues(HOM_COORDS, HOM_COORDS, 
                MIN_RANDOM_VALUE, MAX_RANDOM_VALUE);
        Matrix transT = t.transposeAndReturnNew();
        
        //make symmetric matrix
        Matrix dualConicMatrix = transT.multiplyAndReturnNew(t);
        
        DualConic dualConic = new DualConic(dualConicMatrix);
        
        //normalize dual conic
        dualConic.normalize();
        
        //return dual conic as matrix
        Matrix dualConicMatrix2 = dualConic.asMatrix();
        
        //compare that both matrices are equal up to scale, for that reason we
        //first normalize both matrices
        double norm = com.irurueta.algebra.Utils.normF(dualConicMatrix);
        dualConicMatrix.multiplyByScalar(1.0 / norm);
        
        norm = com.irurueta.algebra.Utils.normF(dualConicMatrix2);
        dualConicMatrix2.multiplyByScalar(1.0 / norm);
        
        //compute their difference
        Matrix diffMatrix = dualConicMatrix.subtractAndReturnNew(
                dualConicMatrix2);
        
        //finally, ensure that the norm of the difference matrix is almost up to
        //machine precision
        norm = com.irurueta.algebra.Utils.normF(diffMatrix);
        
        assertEquals(norm, 0.0, PRECISION_ERROR);
    }
    
    @Test
    public void testCreateCanonicalDualAbsoluteConic() throws WrongSizeException {
        DualConic dac = DualConic.createCanonicalDualAbsoluteConic();
        
        assertEquals(dac.getA(), 1.0, 0.0);
        assertEquals(dac.getB(), 0.0, 0.0);
        assertEquals(dac.getC(), 1.0, 0.0);
        assertEquals(dac.getD(), 0.0, 0.0);
        assertEquals(dac.getE(), 0.0, 0.0);
        assertEquals(dac.getF(), 1.0, 0.0);
        
        assertEquals(dac.asMatrix(), Matrix.identity(3, 3));        
    }
}
