/*
 * Copyright (C) 2017 Alberto Irurueta Carro (alberto@irurueta.com)
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

import java.lang.reflect.Array;
import java.util.Collection;

/**
 * Implementation of a k-D tree in an arbitrary dimension.
 * Once a K-D tree is built for a collection of points, it can later be used to efficiently do certain operations
 * such as point location, nearest points searches, etc.
 */
public abstract class KDTree<P extends Point<P>> {

    /**
     * Minimum number of allowed points to be stored in the tree.
     */
    public static final int MIN_PTS = 3;

    /**
     * A very large value to consider as the maximum allowed coordinate value.
     */
    protected static final double BIG = Double.MAX_VALUE;

    /**
     * Number of tasks that can be queued.
     */
    private static final int N_TASKS = 50;

    /**
     * Array of boxes stored in this tree as its nodes.
     */
    protected BoxNode<P>[] boxes;

    /**
     * Indices of points going from boxes in the tree to the input collection of points.
     * Indices are sorted as the tree is built.
     */
    private final int[] ptIndx;

    /**
     * Indices of points going from input collection of points to boxes in the tree.
     * This is the reverse of mPtIndx.
     */
    private final int[] rPtIndx;

    /**
     * Number of points stored by the tree.
     */
    private final int nPts;

    /**
     * Array of points containing input collection of points.
     */
    private P[] pts;

    /**
     * Constructor.
     *
     * @param pts   collection of points to store in the tree.
     * @param clazz class of point implementation to use.
     */
    protected KDTree(final Collection<P> pts, final Class<P> clazz) {
        nPts = pts.size();
        if (nPts < MIN_PTS) {
            throw new IllegalArgumentException("number of points must be at least 3");
        }

        //noinspection unchecked
        this.pts = (P[]) Array.newInstance(clazz, pts.size());
        this.pts = pts.toArray(this.pts);
        ptIndx = new int[nPts];
        rPtIndx = new int[nPts];
        final int dim = getDimensions();

        // build tree
        int ntmp;
        int m;
        int k;
        int kk;
        int j;
        int nowtask;
        int jbox;
        int np;
        int tmom;
        int tdim;
        int ptlo;
        int pthi;
        int hpOffset;
        int cpOffset;
        final var taskmom = new int[N_TASKS];
        final var taskdim = new int[N_TASKS];
        for (k = 0; k < nPts; k++) {
            ptIndx[k] = k;
        }
        m = 1;
        for (ntmp = nPts; ntmp != 0; ntmp >>= 1) {
            m <<= 1;
        }
        var nboxes = 2 * nPts - (m >> 1); // number of boxes to store points
        if (m < nboxes) {
            nboxes = m;
        }
        nboxes--;
        //noinspection unchecked
        boxes = (BoxNode<P>[]) Array.newInstance(BoxNode.class, nboxes);
        final var coords = new double[dim * nPts];
        for (j = 0, kk = 0; j < dim; j++, kk += nPts) {
            for (k = 0; k < nPts; k++) {
                coords[kk + k] = this.pts[k].getInhomogeneousCoordinate(j);
            }
        }
        P lo = createPoint(-BIG);
        P hi = createPoint(BIG);
        boxes[0] = new BoxNode<>(lo, hi, 0, 0, 0, 0, nPts - 1);
        jbox = 0;
        taskmom[1] = 0;
        taskdim[1] = 0;
        nowtask = 1;
        while (nowtask != 0) {
            tmom = taskmom[nowtask];
            tdim = taskdim[nowtask--];
            ptlo = boxes[tmom].ptLo;
            pthi = boxes[tmom].ptHi;
            hpOffset = ptlo;
            cpOffset = tdim * nPts;
            np = pthi - ptlo + 1;
            kk = (np - 1) / 2;
            selecti(kk, hpOffset, ptIndx, np, cpOffset, coords);

            hi = copyPoint(boxes[tmom].getHi());
            lo = copyPoint(boxes[tmom].getLo());

            final var value = coords[tdim * nPts + ptIndx[hpOffset + kk]];
            hi.setInhomogeneousCoordinate(tdim, value);
            lo.setInhomogeneousCoordinate(tdim, value);

            boxes[++jbox] = new BoxNode<>(copyPoint(boxes[tmom].getLo()), hi, tmom, 0, 0, ptlo, ptlo + kk);
            boxes[++jbox] = new BoxNode<>(lo, copyPoint(boxes[tmom].getHi()), tmom, 0, 0, ptlo + kk + 1,
                    pthi);
            boxes[tmom].dau1 = jbox - 1;
            boxes[tmom].dau2 = jbox;
            if (kk > 1) {
                taskmom[++nowtask] = jbox - 1;
                taskdim[nowtask] = (tdim + 1) % dim;
            }
            if (np - kk > 3) {
                taskmom[++nowtask] = jbox;
                taskdim[nowtask] = (tdim + 1) % dim;
            }
        }
        for (j = 0; j < nPts; j++) {
            rPtIndx[ptIndx[j]] = j;
        }
    }

    /**
     * Gets distance between points located at provided positions on input collection.
     *
     * @param jpt index of 1st point.
     * @param kpt index of 2nd point.
     * @return distance between points or BIG if indices are equal.
     */
    public double distance(final int jpt, final int kpt) {
        if (jpt == kpt) {
            return BIG;
        } else {
            return pts[jpt].distanceTo(pts[kpt]);
        }
    }

    /**
     * Gets position of smallest box containing provided point in the input list of points.
     *
     * @param pt point to locate its containing box. Does not need to be contained in input collection.
     * @return position of smallest box containing the point.
     */
    public int locateBoxIndex(final P pt) {
        final var dim = getDimensions();

        var nb = 0;
        int d1;
        var jdim = 0;
        while (boxes[nb].dau1 != 0) {
            d1 = boxes[nb].dau1;
            if (pt.getInhomogeneousCoordinate(jdim) <= boxes[d1].getHi().getInhomogeneousCoordinate(jdim)) {
                nb = d1;
            } else {
                nb = boxes[nb].dau2;
            }
            jdim = ++jdim % dim;
        }
        return nb;
    }

    /**
     * Gets smallest box containing provided point in the input list of points.
     *
     * @param pt point to locate its containing box. Does not need to be contained in input collection.
     * @return smallest box containing the point.
     */
    public BoxNode<P> locateBox(final P pt) {
        return boxes[locateBoxIndex(pt)];
    }

    /**
     * Index in provided input list of points of closest point to provided one.
     *
     * @param pt point to check against. Does not need to be contained in input collection.
     * @return position of closest point.
     */
    public int nearestIndex(final P pt) {
        int i;
        int k;
        int nrst = 0;
        int ntask;
        int pi;
        final var task = new int[N_TASKS];
        var dnrst = BIG;
        double d;

        // find the smallest box index containing point
        k = locateBoxIndex(pt);
        for (i = boxes[k].ptLo; i <= boxes[k].ptHi; i++) {
            pi = ptIndx[i];
            d = pts[pi].distanceTo(pt);
            if (d < dnrst) {
                nrst = pi; // index of nearest point
                dnrst = d; // distance to nearest point
            }
        }

        // check other boxes in case they contain any nearer point
        task[1] = 0;
        ntask = 1;
        while (ntask != 0) {
            k = task[ntask--];
            if (boxes[k].getDistance(pt) < dnrst) {
                if (boxes[k].dau1 != 0) {
                    task[++ntask] = boxes[k].dau1;
                    task[++ntask] = boxes[k].dau2;
                } else {
                    for (i = boxes[k].ptLo; i <= boxes[k].ptHi; i++) {
                        d = pts[ptIndx[i]].distanceTo(pt);
                        if (d < dnrst) {
                            nrst = ptIndx[i];
                            dnrst = d;
                        }
                    }
                }
            }
        }
        return nrst;
    }

    /**
     * Closest point to provided one.
     *
     * @param pt point to be checked. Does not need to be contained in input collection.
     * @return closest point.
     */
    public P nearestPoint(final P pt) {
        return pts[nearestIndex(pt)];
    }

    /**
     * Gets n nearest point indices to a given one in the input collection.
     *
     * @param jpt index of point to search nearest ones for.
     * @param nn  array containing resulting indices of nearest points up to the number of found points.
     * @param dn  array containing resulting distances to nearest points up to the number of found points.
     * @param n   number of nearest points to find.
     * @throws IllegalArgumentException if number of nearest points is invalid or if length of arrays
     *                                  containing results are not valid either.
     */
    public void nNearest(final int jpt, final int[] nn, final double[] dn, final int n) {
        if (n < 0) {
            throw new IllegalArgumentException("no neighbours requested");
        }
        if (n > nPts - 1) {
            throw new IllegalArgumentException("too many neighbours requested");
        }
        if (nn.length != n || dn.length != n) {
            throw new IllegalArgumentException("invalid result array lengths");
        }

        int i;
        int k;
        int ntask;
        int kp;
        final var task = new int[N_TASKS];
        double d;
        for (i = 0; i < n; i++) {
            dn[i] = BIG;
        }
        kp = boxes[locate(jpt)].mom;
        while (boxes[kp].ptHi - boxes[kp].ptLo < n) {
            kp = boxes[kp].mom;
        }
        for (i = boxes[kp].ptLo; i <= boxes[kp].ptHi; i++) {
            if (jpt == ptIndx[i]) {
                continue;
            }
            d = distance(ptIndx[i], jpt);
            if (d < dn[0]) {
                dn[0] = d;
                nn[0] = ptIndx[i];
                if (n > 1) {
                    siftDown(dn, nn, n);
                }
            }
        }
        task[1] = 0;
        ntask = 1;
        while (ntask != 0) {
            k = task[ntask--];
            if (k == kp) {
                continue;
            }
            if (boxes[k].getDistance(pts[jpt]) < dn[0]) {
                if (boxes[k].dau1 != 0) {
                    task[++ntask] = boxes[k].dau1;
                    task[++ntask] = boxes[k].dau2;
                } else {
                    for (i = boxes[k].ptLo; i <= boxes[k].ptHi; i++) {
                        d = distance(ptIndx[i], jpt);
                        if (d < dn[0]) {
                            dn[0] = d;
                            nn[0] = ptIndx[i];
                            if (n > 1) {
                                siftDown(dn, nn, n);
                            }
                        }
                    }
                }
            }
        }
    }

    /**
     * Gets n nearest point indices to a given point in the input collection.
     *
     * @param pt point to search nearest ones for.
     * @param nn array containing resulting indices of nearest points up to the number of found points.
     * @param dn array containing resulting distances to nearest points up to the number of found points.
     * @param n  number of nearest points to find.
     * @throws IllegalArgumentException if number of nearest points is invalid or if length of arrays
     *                                  containing results are not valid either.
     */
    public void nNearest(final P pt, final int[] nn, final double[] dn, final int n) {
        nNearest(nearestIndex(pt), nn, dn, n);
    }

    /**
     * Gets n nearest points to a given point index in the input collection.
     *
     * @param jpt index of point to search nearest ones for.
     * @param pn  array containing nearest points up to the number of found points.
     * @param dn  array containing resulting distances to nearest points up to the number of found points.
     * @param n   number of nearest points to find.
     * @throws IllegalArgumentException if number of nearest points is invalid or if length of arrays
     *                                  containing results are not valid either.
     */
    public void nNearest(final int jpt, final P[] pn, final double[] dn, final int n) {
        if (n < 0) {
            throw new IllegalArgumentException("no neighbours requested");
        }

        final var nn = new int[n];

        nNearest(jpt, nn, dn, n);

        for (var i = 0; i < n; i++) {
            pn[i] = pts[nn[i]];
        }
    }

    /**
     * Gets n nearest points to a given point in the input collection.
     *
     * @param pt point to search nearest ones for.
     * @param pn array containing nearest points up to the number of found points.
     * @param dn array containing resulting distances to nearest points up to the number of found points.
     * @param n  number of nearest points to find.
     * @throws IllegalArgumentException if number of nearest points is invalid or if length of arrays
     *                                  containing results are not valid either.
     */
    public void nNearest(final P pt, final P[] pn, final double[] dn, final int n) {
        nNearest(nearestIndex(pt), pn, dn, n);
    }

    /**
     * Locates some near points to provided one up to a certain radius of search.
     * This method only returns up to nmax results, which means that not all points within required
     * radius are returned if more points than provided nmax value are within such radius.
     *
     * @param pt   point to search nearby.
     * @param r    radius of search.
     * @param list list where indices of found points are stored up to the number of found points.
     * @param nmax maximum number of points to search.
     * @return number of found points.
     * @throws IllegalArgumentException if radius is negative or maximum number of points to search is zero or negative,
     *                                  or list where indices are stored is not large enough.
     */
    public int locateNear(final P pt, final double r, final int[] list, final int nmax) {
        if (r < 0.0) {
            throw new IllegalArgumentException("radius must be non-negative");
        }
        if (nmax <= 0) {
            throw new IllegalArgumentException("number of points to search must be at least 1");
        }
        if (list.length < nmax) {
            throw new IllegalArgumentException("result might not fit into provided list");
        }

        final int dim = getDimensions();

        int k;
        int i;
        int nb;
        int nbold;
        int nret;
        int ntask;
        int jdim;
        int d1;
        int d2;
        final var task = new int[N_TASKS];
        nb = jdim = nret = 0;

        while (boxes[nb].dau1 != 0) {
            nbold = nb;
            d1 = boxes[nb].dau1;
            d2 = boxes[nb].dau2;
            final var coord = pt.getInhomogeneousCoordinate(jdim);
            if (coord + r <= boxes[d1].getHi().getInhomogeneousCoordinate(jdim)) {
                nb = d1;
            } else if (coord - r >= boxes[d2].getLo().getInhomogeneousCoordinate(jdim)) {
                nb = d2;
            }
            jdim = ++jdim % dim;
            if (nb == nbold) {
                break;
            }
        }
        task[1] = nb;
        ntask = 1;
        while (ntask != 0) {
            k = task[ntask--];
            if (boxes[k].getDistance(pt) > r) {
                continue;
            }
            if (boxes[k].dau1 != 0) {
                task[++ntask] = boxes[k].dau1;
                task[++ntask] = boxes[k].dau2;
            } else {
                for (i = boxes[k].ptLo; i <= boxes[k].ptHi; i++) {
                    if (pts[ptIndx[i]].distanceTo(pt) <= r && nret < nmax) {
                        list[nret++] = ptIndx[i];
                    }
                    if (nret == nmax) {
                        return nmax;
                    }
                }
            }
        }
        return nret;
    }

    /**
     * Locates near points to provided one up to a certain radius of search defined in a bounding box.
     *
     * @param pt    point to search nearby.
     * @param r     radius of search defining a bounding box.
     * @param plist list where found points are stored up to the number of found points.
     * @param nmax  maximum number of points to search.
     * @return number of found points.
     * @throws IllegalArgumentException if radius is negative or maximum number of points to search is zero or negative,
     *                                  or list where points are stored is not large enough.
     */
    public int locateNear(final P pt, final double r, final P[] plist, final int nmax) {
        if (r < 0.0) {
            throw new IllegalArgumentException("radius must be non-negative");
        }
        if (nmax <= 0) {
            throw new IllegalArgumentException("number of points to search must be at least 1");
        }
        if (plist.length < nmax) {
            throw new IllegalArgumentException("result might not fit into provided list");
        }

        final var list = new int[nmax];
        final var result = locateNear(pt, r, list, nmax);

        for (int i = 0; i < result; i++) {
            plist[i] = pts[list[i]];
        }

        return result;
    }

    /**
     * Gets number of dimensions supported by this k-D tree implementation on provided list of points.
     *
     * @return number of dimensions.
     */
    public abstract int getDimensions();

    /**
     * Creates a point.
     *
     * @param value value to be set on point coordinates.
     * @return created point.
     */
    protected abstract P createPoint(final double value);

    /**
     * Copies a point.
     *
     * @param point point to be copied.
     * @return copied point.
     */
    protected abstract P copyPoint(final P point);

    /**
     * Gets position of point on input collection for provided internal boxes position.
     *
     * @param jpt internal position in the boxes.
     * @return position in the input collection of points.
     */
    private int locate(final int jpt) {
        var nb = 0;
        int d1;
        final var jh = rPtIndx[jpt];
        while (boxes[nb].dau1 != 0) {
            d1 = boxes[nb].dau1;
            if (jh <= boxes[d1].ptHi) {
                nb = d1;
            } else {
                nb = boxes[nb].dau2;
            }
        }
        return nb;
    }

    /**
     * Makes a selection so that we obtain ordered index at provided k position so that
     * distances are ordered in such a way that resulting array arr contains distances
     * as follows: arr[indx[0 .. k-1]] &lt;= arr[indx[k]] &lt;= arr[indx[k+1 .. n]].
     * So that positions between 0 and k-1 are not in any particular order but is less than
     * k position, and positions between k+1 and n neither have any particular order but is
     * more than k position.
     *
     * @param k          sorted position to retrieve.
     * @param indxOffset offset where indx search starts.
     * @param indx       array to be sorted (i.e. selected).
     * @param n          length of arrays.
     * @param arrOffset  offset of distances array. This is usually equal to indxOffset.
     * @param arr        resulting array containing distances to each selected point.
     * @return index of selected point.
     */
    @SuppressWarnings("UnusedReturnValue")
    private static int selecti(final int k, final int indxOffset, final int[] indx, final int n, final int arrOffset,
                               final double[] arr) {
        int i;
        int ia;
        var ir = n - 1;
        int j;
        var l = 0;
        int mid;
        double a;

        for (; ; ) {
            if (ir <= l + 1) {
                if (ir == l + 1 && arr[arrOffset + indx[indxOffset + ir]] < arr[arrOffset + indx[indxOffset + l]]) {
                    swap(indx, indxOffset + l, indx, indxOffset + ir);
                }
                return indx[indxOffset + k];
            } else {
                mid = (l + ir) >> 1;
                swap(indx, indxOffset + mid, indx, indxOffset + l + 1);
                if (arr[arrOffset + indx[indxOffset + l]] > arr[arrOffset + indx[indxOffset + ir]]) {
                    swap(indx, indxOffset + l, indx, indxOffset + ir);
                }
                if (arr[arrOffset + indx[indxOffset + l + 1]] > arr[arrOffset + indx[indxOffset + ir]]) {
                    swap(indx, indxOffset + l + 1, indx, indxOffset + ir);
                }
                if (arr[arrOffset + indx[indxOffset + l]] > arr[arrOffset + indx[indxOffset + l + 1]]) {
                    swap(indx, indxOffset + l, indx, indxOffset + l + 1);
                }
                i = l + 1;
                j = ir;
                ia = indx[indxOffset + l + 1];
                a = arr[arrOffset + ia];
                for (; ; ) {
                    do {
                        i++;
                    } while (arr[arrOffset + indx[indxOffset + i]] < a);
                    do {
                        j--;
                    } while (arr[arrOffset + indx[indxOffset + j]] > a);
                    if (j < i) {
                        break;
                    }
                    swap(indx, indxOffset + i, indx, indxOffset + j);
                }
                indx[indxOffset + l + 1] = indx[indxOffset + j];
                indx[indxOffset + j] = ia;
                if (j >= k) {
                    ir = j - 1;
                }
                if (j <= k) {
                    l = i;
                }
            }
        }
    }

    /**
     * Moves things around.
     *
     * @param heap array of distances.
     * @param ndx  array of indices.
     * @param nn   number of indices to move.
     */
    private static void siftDown(final double[] heap, final int[] ndx, final int nn) {
        final var n = nn - 1;
        var j = 1;
        var jold = 0;
        final var ia = ndx[0];
        final var a = heap[0];
        while (j <= n) {
            if (j < n && heap[j] < heap[j + 1]) {
                j++;
            }
            if (a >= heap[j]) {
                break;
            }
            heap[jold] = heap[j];
            ndx[jold] = ndx[j];
            jold = j;
            j = 2 * j + 1;
        }
        heap[jold] = a;
        ndx[jold] = ia;
    }

    /**
     * Swaps values.
     *
     * @param a    1st array containing values to swap.
     * @param posA position to be swapped on 1st array.
     * @param b    2nd array containing values to swap.
     * @param posB position to be swapped on 2nd array.
     */
    private static void swap(final int[] a, final int posA, final int[] b, final int posB) {
        final var tmp = a[posA];

        a[posA] = b[posB];
        b[posB] = tmp;
    }

    /**
     * Contains a node of a KD Tree.
     */
    public static class BoxNode<P extends Point<P>> extends Box<P> {

        /**
         * Position of mother node in the list of nodes of a tree.
         */
        private final int mom;

        /**
         * Position of 1st daughter node in the list of nodes of a tree.
         */
        private int dau1;

        /**
         * Position of 2nd daughter node of a tree.
         */
        private int dau2;

        /**
         * Low index of list of points inside this box.
         * mPtLo and mPtHi define the range of points inside the box.
         */
        private final int ptLo;

        /**
         * High index of list of points inside this box.
         * mPtLo and mPtHi define the range of points inside the box.
         */
        private final int ptHi;

        /**
         * Constructor.
         *
         * @param lo   low coordinate values.
         * @param hi   high coordinate values.
         * @param mom  index of mother node.
         * @param d1   index of 1st daughter.
         * @param d2   index of 2nd daughter.
         * @param ptLo low index of list of points inside this box.
         * @param ptHi high index of list of points inside this box.
         */
        public BoxNode(final P lo, final P hi, final int mom, final int d1, final int d2, final int ptLo,
                       final int ptHi) {
            super(lo, hi);
            this.mom = mom;
            dau1 = d1;
            dau2 = d2;
            this.ptLo = ptLo;
            this.ptHi = ptHi;
        }

        /**
         * Gets position of mother node in the list of nodes of a tree.
         *
         * @return position of mother node in the list of nodes of a tree.
         */
        public int getMom() {
            return mom;
        }

        /**
         * Gets position of 1st daughter node in the list of nodes of a tree.
         *
         * @return position of 1st daughter node in the list of nodes of a tree.
         */
        public int getDau1() {
            return dau1;
        }

        /**
         * Gets position of 2nd daughter node of a tree.
         *
         * @return position of 2nd daughter node of a tree.
         */
        public int getDau2() {
            return dau2;
        }

        /**
         * Gets low index of list of points inside this box.
         * getPtLo() and {@link #getPtHi()} define the range of indices of points
         * contained in this box.
         *
         * @return low index of list of points inside this box.
         */
        public int getPtLo() {
            return ptLo;
        }

        /**
         * Gets high index of list of points inside this box.
         * {@link #getPtLo()} and getPtHi() define the range of indices of points
         * contained in this box.
         *
         * @return high index of list of points inside this box.
         */
        public int getPtHi() {
            return ptHi;
        }

        /**
         * Sets low coordinate values.
         *
         * @param lo low coordinate values.
         * @throws IllegalArgumentException always thrown.
         */
        @Override
        public void setLo(final P lo) {
            throw new IllegalArgumentException();
        }

        /**
         * Sets high coordinate values.
         *
         * @param hi high coordinate values.
         * @throws IllegalArgumentException always thrown.
         */
        @Override
        public void setHi(final P hi) {
            throw new IllegalArgumentException();
        }

        /**
         * Sets boundaries.
         *
         * @param lo low coordinate values.
         * @param hi high coordinate values.
         * @throws IllegalArgumentException always thrown.
         */
        @Override
        public void setBounds(final P lo, final P hi) {
            throw new IllegalArgumentException();
        }
    }
}
