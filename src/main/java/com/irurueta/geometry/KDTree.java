package com.irurueta.geometry;

import java.lang.reflect.Array;
import java.util.Collection;

/**
 * Implementation of a k-D tree in an arbitrary dimension.
 * Once a K-D tree is built for a collection of points, it can later be used to efficiently do certain operations
 * such as point location, nearest points searches, etc.
 */
public abstract class KDTree<P extends Point> {

    /**
     * Minimum number of allowed points to be stored in the tree.
     */
    public static final int MIN_PTS = 3;

    /**
     * A very large value to consider as the maximum allowed coordinate value.
     */
    protected static final double BIG = 1e99;

    /**
     * Number of tasks that can be queued.
     */
    private static final int N_TASKS = 50;

    /**
     * Array of boxes stored in this tree as its nodes.
     */
    protected BoxNode[] mBoxes;

    /**
     * Indices of points pointing from boxes in the tree to the input collection of points.
     * Indices are sorted as the tree is built.
     */
    private int[] mPtIndx;

    /**
     * Indices of points pointing from input collection of points to boxes in the tree.
     * This is the reverse of mPtIndx.
     */
    private int[] mRPtIndx;

    /**
     * Number of points stored by the tree.
     */
    private int mNpts;

    /**
     * Array of points containing input collection of points.
     */
    private P[] mPts;

    /**
     * Constructor.
     * @param pts collection of points to store in the tree.
     */
    public KDTree(Collection<P> pts, Class<P> clazz) throws IllegalArgumentException {
        mNpts = pts.size();
        if (mNpts < MIN_PTS) {
            throw new IllegalArgumentException("number of points must be at least 3");
        }

        //noinspection unchecked
        mPts = (P[])Array.newInstance(clazz, pts.size());
        mPts = pts.toArray(mPts);
        mPtIndx = new int[mNpts];
        mRPtIndx = new int[mNpts];
        int dim = getDimensions();

        //build tree
        int ntmp, m, k, kk, j, nowtask, jbox, np, tmom, tdim, ptlo, pthi;
        int hpOffset, cpOffset;
        int[] taskmom = new int[N_TASKS];
        int[] taskdim = new int[N_TASKS];
        for (k = 0; k < mNpts; k++) {
            mPtIndx[k] = k;
        }
        m = 1;
        for (ntmp = mNpts; ntmp != 0; ntmp >>= 1) {
            m <<= 1;
        }
        int nboxes = 2 * mNpts - (m >> 1); //number of boxes to store points
        if (m < nboxes) {
            nboxes = m;
        }
        nboxes--;
        //noinspection unchecked
        mBoxes = (BoxNode[])Array.newInstance(BoxNode.class, nboxes);
        double[] coords = new double[dim * mNpts];
        for (j = 0, kk = 0; j < dim; j++, kk += mNpts) {
            for (k = 0; k < mNpts; k++) {
                coords[kk + k] = mPts[k].getInhomogeneousCoordinate(j);
            }
        }
        P lo = createPoint(-BIG);
        P hi = createPoint(BIG);
        mBoxes[0] = new BoxNode(lo, hi, 0, 0, 0, 0, mNpts - 1);
        jbox = 0;
        taskmom[1] = 0;
        taskdim[1] = 0;
        nowtask = 1;
        while(nowtask != 0) {
            tmom = taskmom[nowtask];
            tdim = taskdim[nowtask--];
            ptlo = mBoxes[tmom].ptLo;
            pthi = mBoxes[tmom].ptHi;
            hpOffset = ptlo;
            cpOffset = tdim * mNpts;
            np = pthi - ptlo + 1;
            kk = (np - 1) / 2;
            selecti(kk, hpOffset, mPtIndx, np, cpOffset, coords);

            hi = copyPoint(mBoxes[tmom].getHi());
            lo = copyPoint(mBoxes[tmom].getLo());

            double value = coords[tdim * mNpts + mPtIndx[hpOffset + kk]];
            hi.setInhomogeneousCoordinate(tdim, value);
            lo.setInhomogeneousCoordinate(tdim, value);

            mBoxes[++jbox] = new BoxNode(copyPoint(mBoxes[tmom].getLo()), hi, tmom, 0, 0, ptlo,
                    ptlo + kk);
            mBoxes[++jbox] = new BoxNode(lo, copyPoint(mBoxes[tmom].getHi()), tmom, 0, 0,
                    ptlo + kk + 1, pthi);
            mBoxes[tmom].dau1 = jbox - 1;
            mBoxes[tmom].dau2 = jbox;
            if (kk > 1) {
                taskmom[++nowtask] = jbox - 1;
                taskdim[nowtask] = (tdim + 1) % dim;
            }
            if (np - kk > 3) {
                taskmom[++nowtask] = jbox;
                taskdim[nowtask] = (tdim + 1) % dim;
            }
        }
        for (j = 0; j < mNpts; j++) {
            mRPtIndx[mPtIndx[j]] = j;
        }
    }

    /**
     * Gets distance between points located at provided positions on input collection.
     * @param jpt index of 1st point.
     * @param kpt index of 2nd point.
     * @return distance between points or BIG if indices are equal.
     */
    public double distance(int jpt, int kpt) {
        if (jpt == kpt) {
            return BIG;
        } else {
            //noinspection unchecked
            return mPts[jpt].distanceTo(mPts[kpt]);
        }
    }

    /**
     * Gets position of smallest box containing provided point in the input list of points.
     * @param pt point to locate its containing box. Does not need to be contained in input collection.
     * @return position of smallest box containing the point.
     */
    public int locateBoxIndex(P pt) {
        int dim = getDimensions();

        int nb, d1, jdim;
        nb = jdim = 0;
        while (mBoxes[nb].dau1 != 0) {
            d1 = mBoxes[nb].dau1;
            if (pt.getInhomogeneousCoordinate(jdim) <= mBoxes[d1].getHi().getInhomogeneousCoordinate(jdim)) {
                nb = d1;
            } else {
                nb = mBoxes[nb].dau2;
            }
            jdim = ++jdim % dim;
        }
        return nb;
    }

    /**
     * Gets smallest box containing provided point in the input list of points.
     * @param pt point to locate its containing box. Does not need to be contained in input collection.
     * @return smallest box containing the point.
     */
    public BoxNode locateBox(P pt) {
        return mBoxes[locateBoxIndex(pt)];
    }

    /**
     * Index in provided input list of points of closest point to provided one.
     * @param pt point to check against. Does not need to be contained in input collection.
     * @return position of closest point.
     */
    public int nearestIndex(P pt) {
        int i, k, nrst = 0, ntask, pi;
        int[] task = new int[N_TASKS];
        double dnrst = BIG, d;

        //find smallest box index containing point
        k = locateBoxIndex(pt);
        for (i = mBoxes[k].ptLo; i <= mBoxes[k].ptHi; i++) {
            pi = mPtIndx[i];
            //noinspection unchecked
            d = mPts[pi].distanceTo(pt);
            if (d < dnrst) {
                nrst = pi; //index of nearest point
                dnrst = d; //distance to nearest point
            }
        }

        //check other boxes in case they contain any nearer point
        task[1] = 0;
        ntask = 1;
        while (ntask != 0) {
            k = task[ntask--];
            if (mBoxes[k].getDistance(pt) < dnrst) {
                if (mBoxes[k].dau1 != 0) {
                    task[++ntask] = mBoxes[k].dau1;
                    task[++ntask] = mBoxes[k].dau2;
                } else {
                    for (i = mBoxes[k].ptLo; i <= mBoxes[k].ptHi; i++) {
                        //noinspection unchecked
                        d = mPts[mPtIndx[i]].distanceTo(pt);
                        if (d < dnrst) {
                            nrst = mPtIndx[i];
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
     * @param pt point to be checked. Does not need to be contained in input collection.
     * @return closest point.
     */
    public P nearestPoint(P pt) {
        return mPts[nearestIndex(pt)];
    }

    /**
     * Gets n nearest point indices to a given one in the input collection.
     * @param jpt index of point to search nearest ones for.
     * @param nn array containing resulting indices of nearest points up to the number of found points.
     * @param dn array containing resulting distances to nearest points up to the number of found points.
     * @param n number of nearest points to find.
     * @throws IllegalArgumentException if number of nearest points is invalid or if length of arrays
     * containing results are not valid either.
     */
    public void nNearest(int jpt, int[] nn, double[] dn, int n) throws IllegalArgumentException {
        if (n < 0) {
            throw new IllegalArgumentException("no neighbours requested");
        }
        if (n > mNpts - 1) {
            throw new IllegalArgumentException("too many neighbours requested");
        }
        if (nn.length != n || dn.length != n) {
            throw new IllegalArgumentException("invalid result array lengths");
        }

        int i, k, ntask, kp;
        int[] task = new int[N_TASKS];
        double d;
        for (i = 0; i < n; i++) {
            dn[i] = BIG;
        }
        kp = mBoxes[locate(jpt)].mom;
        while (mBoxes[kp].ptHi - mBoxes[kp].ptLo < n) {
            kp = mBoxes[kp].mom;
        }
        for (i = mBoxes[kp].ptLo; i <= mBoxes[kp].ptHi; i++) {
            if (jpt == mPtIndx[i]) {
                continue;
            }
            d = distance(mPtIndx[i], jpt);
            if (d < dn[0]) {
                dn[0] = d;
                nn[0] = mPtIndx[i];
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
            if (mBoxes[k].getDistance(mPts[jpt]) < dn[0]) {
                if (mBoxes[k].dau1 != 0) {
                    task[++ntask] = mBoxes[k].dau1;
                    task[++ntask] = mBoxes[k].dau2;
                } else {
                    for (i = mBoxes[k].ptLo; i <= mBoxes[k].ptHi; i++) {
                        d = distance(mPtIndx[i], jpt);
                        if (d < dn[0]) {
                            dn[0] = d;
                            nn[0] = mPtIndx[i];
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
     * @param pt point to search nearest ones for.
     * @param nn array containing resulting indices of nearest points up to the number of found points.
     * @param dn array containing resulting distances to nearest points up to the number of found points.
     * @param n number of nearest points to find.
     * @throws IllegalArgumentException if number of nearest points is invalid or if length of arrays
     * containing results are not valid either.
     */
    public void nNearest(P pt, int[] nn, double[] dn, int n) throws IllegalArgumentException {
        nNearest(nearestIndex(pt), nn, dn, n);
    }

    /**
     * Gets n nearest points to a given point index in the input collection.
     * @param jpt index of point to search nearest ones for.
     * @param pn array containing nearest points up to the number of found points.
     * @param dn array containing resulting distances to nearest points up to the number of found points.
     * @param n number of nearest points to find.
     * @throws IllegalArgumentException if number of nearest points is invalid or if length of arrays
     * containing results are not valid either.
     */
    public void nNearest(int jpt, P[] pn, double[] dn, int n) throws IllegalArgumentException {
        if (n < 0) {
            throw new IllegalArgumentException("no neighbours requested");
        }

        int[] nn = new int[n];

        nNearest(jpt, nn, dn, n);

        for (int i = 0; i < n; i++) {
            pn[i] = mPts[nn[i]];
        }
    }

    /**
     * Gets n nearest points to a given point in the input collection.
     * @param pt point to search nearest ones for.
     * @param pn array containing nearest points up to the number of found points.
     * @param dn array containing resulting distances to nearest points up to the number of found points.
     * @param n number of nearest points to find.
     * @throws IllegalArgumentException if number of nearest points is invalid or if length of arrays
     * containing results are not valid either.
     */
    public void nNearest(P pt, P[] pn, double[] dn, int n) throws IllegalArgumentException {
        nNearest(nearestIndex(pt), pn, dn, n);
    }

    /**
     * Locates some near points to provided one up to a certain radius of search.
     * This method only returns up to nmax results, which means that not all points within required
     * radius are returned if more points than provided nmax value are within such radius.
     * @param pt point to search nearby.
     * @param r radius of search.
     * @param list list where indices of found points are stored up to the number of found points.
     * @param nmax maximum number of points to search.
     * @return number of found points.
     * @throws IllegalArgumentException if radius is negative or maximum number of points to search is zero or negative,
     * or list where indices are stored is not large enough.
     */
    public int locateNear(P pt, double r, int[] list, int nmax) throws IllegalArgumentException {
        if (r < 0.0) {
            throw new IllegalArgumentException("radius must be nonnegative");
        }
        if (nmax <= 0) {
            throw new IllegalArgumentException("number of points to search must be at least 1");
        }
        if (list.length < nmax) {
            throw new IllegalArgumentException("result might not fit into provided list");
        }

        int dim = getDimensions();

        int k, i, nb, nbold, nret, ntask, jdim, d1, d2;
        int[] task = new int[N_TASKS];
        nb = jdim = nret = 0;

        while (mBoxes[nb].dau1 != 0) {
            nbold = nb;
            d1 = mBoxes[nb].dau1;
            d2 = mBoxes[nb].dau2;
            double coord = pt.getInhomogeneousCoordinate(jdim);
            if (coord + r <= mBoxes[d1].getHi().getInhomogeneousCoordinate(jdim)) {
                nb = d1;
            } else if (coord - r >= mBoxes[d2].getLo().getInhomogeneousCoordinate(jdim)) {
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
            if (mBoxes[k].getDistance(pt) > r) {
                continue;
            }
            if (mBoxes[k].dau1 != 0) {
                task[++ntask] = mBoxes[k].dau1;
                task[++ntask] = mBoxes[k].dau2;
            } else {
                for (i = mBoxes[k].ptLo; i <= mBoxes[k].ptHi; i++) {
                    //noinspection unchecked
                    if (mPts[mPtIndx[i]].distanceTo(pt) <= r && nret < nmax) {
                        list[nret++] = mPtIndx[i];
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
     * @param pt point to search nearby.
     * @param r radius of search defining a bounding box.
     * @param plist list where found points are stored up to the number of found points.
     * @param nmax maximum number of points to search.
     * @return number of found points.
     * @throws IllegalArgumentException if radius is negative or maximum number of points to search is zero or negative,
     * or list where points are stored is not large enough.
     */
    public int locateNear(P pt, double r, P[] plist, int nmax) throws IllegalArgumentException {
        if (r < 0.0) {
            throw new IllegalArgumentException("radius must be nonnegative");
        }
        if (nmax <= 0) {
            throw new IllegalArgumentException("number of points to search must be at least 1");
        }
        if (plist.length < nmax) {
            throw new IllegalArgumentException("result might not fit into provided list");
        }

        int[] list = new int[nmax];
        int result = locateNear(pt, r, list, nmax);

        for (int i = 0; i < result; i++) {
            plist[i] = mPts[list[i]];
        }

        return result;
    }

    /**
     * Gets number of dimensions supported by this k-D tree implementation on provided list of points.
     * @return number of dimensions.
     */
    public abstract int getDimensions();

    /**
     * Creates a point.
     * @param value value to be set on point coordinates.
     * @return created point.
     */
    protected abstract P createPoint(double value);

    /**
     * Copies a point.
     * @param point point to be copied.
     * @return copied point.
     */
    protected abstract P copyPoint(P point);

    /**
     * Gets position of point on input collection for provided internal boxes position.
     * @param jpt internal position in the boxes.
     * @return position in the input collection of points.
     */
    private int locate(int jpt) {
        int nb, d1, jh;
        jh = mRPtIndx[jpt];
        nb = 0;
        while (mBoxes[nb].dau1 != 0) {
            d1 = mBoxes[nb].dau1;
            if (jh <= mBoxes[d1].ptHi) {
                nb = d1;
            } else {
                nb = mBoxes[nb].dau2;
            }
        }
        return nb;
    }

    /**
     * Makes a selection so that we obtain ordered index at provided k position so that
     * distances are ordered in such a way that resulting array arr containg distances
     * as follows: arr[indx[0 .. k-1]] <= arr[indx[k]] <= arr[indx[k+1 .. n]].
     * So that positions between 0 and k-1 are not in any particular order but is less than
     * k position, and positions between k+1 and n neither have any particular order but is
     * more than k position.
     * @param k sorted position to retrieve.
     * @param indxOffset offset where indx search starts.
     * @param indx array to be sorted (i.e. selected).
     * @param n length of arrays.
     * @param arrOffset offset of distances array. This is usually equal to indxOffset.
     * @param arr resulting array containing distances to each selected point.
     * @return index of selected point.
     */
    @SuppressWarnings("all")
    private static int selecti(int k, int indxOffset, int[] indx, int n, int arrOffset, double[] arr) {
        int i, ia, ir, j, l, mid;
        double a;

        l = 0;
        ir = n - 1;
        for (;;) {
            if (ir <= l + 1) {
                if (ir == l + 1 && arr[arrOffset + indx[indxOffset + ir]] < arr[arrOffset + indx[indxOffset + l]]) {
                    swap(indx, indxOffset + l, indx, indxOffset + ir);
                }
                return  indx[indxOffset + k];
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
                    swap(indx, indxOffset + l, indx,indxOffset + l + 1);
                }
                i = l + 1;
                j = ir;
                ia = indx[indxOffset + l + 1];
                a = arr[arrOffset + ia];
                for (;;) {
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
     * @param heap array of distances.
     * @param ndx array of indices.
     * @param nn number of indices to move.
     */
    private static void siftDown(double[] heap, int[] ndx, int nn) {
        int n = nn - 1;
        int j, jold, ia;
        double a = heap[0];
        ia = ndx[0];
        jold = 0;
        j = 1;
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
     * @param a 1st array containing values to swap.
     * @param posA position to be swapped on 1st array.
     * @param b 2nd array containing values to swap.
     * @param posB position to be swapped on 2nd array.
     */
    private static void swap(int[] a, int posA, int[] b, int posB) {
        int tmp = a[posA];

        a[posA] = b[posB];
        b[posB] = tmp;
    }

    /**
     * Contains a node of a KD Tree.
     */
    public class BoxNode extends Box<P> {

        /**
         * Position of mother node in the list of nodes of a tree.
         */
        int mom;

        /**
         * Position of 1st daughter node in the list of nodes of a tree.
         */
        int dau1;

        /**
         * Position of 2nd daughter node of a tree.
         */
        int dau2;

        /**
         * Low index of list of points inside this box.
         * mPtLo and mPtHi define the range of points inside the box.
         */
        int ptLo;

        /**
         * High index of list of points inside this box.
         * mPtLo and mPtHi define the range of points inside the box.
         */
        int ptHi;

        /**
         * Constructor.
         * @param lo low coordinate values.
         * @param hi high coordinate values.
         * @param mom index of mother node.
         * @param d1 index of 1st daughter.
         * @param d2 index of 2nd daughter.
         * @param ptLo low index of list of points inside this box.
         * @param ptHi high index of list of points inside this box.
         */
        public BoxNode(P lo, P hi, int mom, int d1, int d2, int ptLo, int ptHi) {
            super(lo, hi);
            this.mom = mom;
            dau1 = d1;
            dau2 = d2;
            this.ptLo = ptLo;
            this.ptHi = ptHi;
        }
    }
}