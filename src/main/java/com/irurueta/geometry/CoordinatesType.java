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
 * Enumerator that indicates the type of coordinates used to represent a point.
 */
public enum CoordinatesType {

    /**
     * Homogeneous coordinates: coordinates x, y and w must be given.
     */
    HOMOGENEOUS_COORDINATES,

    /**
     * Inhomogeneous coordinates: coordinates x and y (w is assumed to be 1).
     */
    INHOMOGENEOUS_COORDINATES
}
