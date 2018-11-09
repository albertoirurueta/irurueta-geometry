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

import org.junit.*;

import static org.junit.Assert.assertNotNull;

public class CoincidentPointsExceptionTest {
    
    public CoincidentPointsExceptionTest() { }

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
        CoincidentPointsException ex = new CoincidentPointsException();
        assertNotNull(ex);

        ex = new CoincidentPointsException("message");
        assertNotNull(ex);

        ex = new CoincidentPointsException(new Exception());
        assertNotNull(ex);

        ex = new CoincidentPointsException("message",
                new Exception());
        assertNotNull(ex);
    }
}
