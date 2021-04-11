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

/**
 * Enumerator that indicates the type of conic depending on the values of its
 * inner parameters.
 */
public enum ConicType {
    /**
     * Conic parameters satisfying b^2 - ac &lt; 0.
     */
    ELLIPSE_CONIC_TYPE,

    /**
     * Conic parameters satisfying b^2 - ac &lt; 0 and a = c, b = 0.
     */
    CIRCLE_CONIC_TYPE,

    /**
     * Conic parameters satisfying b^2 - ac = 0.
     */
    PARABOLA_CONIC_TYPE,

    /**
     * Conic parameters satisfying b^2 - ac &lt; 0.
     */
    HYPERBOLA_CONIC_TYPE,

    /**
     * Conic parameters satisfying b^2 - ac &lt; 0 and a + b = 0.
     */
    RECTANGULAR_HYPERBOLA_CONIC_TYPE
}
