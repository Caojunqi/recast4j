/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4J copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

This software is provided 'as-is', without any express or implied
warranty.  In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
 claim that you wrote the original software. If you use this software
 in a product, an acknowledgment in the product documentation would be
 appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
 misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
package org.recast4j.recast;

import static org.recast4j.recast.RecastCommon.GetCon;
import static org.recast4j.recast.RecastCommon.GetDirOffsetX;
import static org.recast4j.recast.RecastCommon.GetDirOffsetY;
import static org.recast4j.recast.RecastCommon.rcGetDirForOffset;
import static org.recast4j.recast.RecastConstants.RC_MESH_NULL_IDX;
import static org.recast4j.recast.RecastConstants.RC_MULTIPLE_REGS;
import static org.recast4j.recast.RecastConstants.RC_NOT_CONNECTED;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class RecastMeshDetail {

    static int MAX_VERTS = 127;
    static int MAX_TRIS = 255; // Max tris for delaunay is 2n-2-k (n=num verts, k=num hull verts).
    static int MAX_VERTS_PER_EDGE = 32;

    static int RC_UNSET_HEIGHT = 0xffff;
    static int EV_UNDEF = -1;
    static int EV_HULL = -2;

    private static class HeightPatch {
        int xmin;
        int ymin;
        int width;
        int height;
        int[] data;
    }

    private static float vdot2(float[] a, float[] b) {
        return a[0] * b[0] + a[2] * b[2];
    }

    private static float vdistSq2(float[] verts, int p, int q) {
        float dx = verts[q + 0] - verts[p + 0];
        float dy = verts[q + 2] - verts[p + 2];
        return dx * dx + dy * dy;
    }

    private static float vdist2(float[] verts, int p, int q) {
        return (float) Math.sqrt(vdistSq2(verts, p, q));
    }

    private static float vdistSq2(float[] p, float[] q) {
        float dx = q[0] - p[0];
        float dy = q[2] - p[2];
        return dx * dx + dy * dy;
    }

    private static float vdist2(float[] p, float[] q) {
        return (float) Math.sqrt(vdistSq2(p, q));
    }

    private static float vdistSq2(float[] p, float[] verts, int q) {
        float dx = verts[q + 0] - p[0];
        float dy = verts[q + 2] - p[2];
        return dx * dx + dy * dy;
    }

    private static float vdist2(float[] p, float[] verts, int q) {
        return (float) Math.sqrt(vdistSq2(p, verts, q));
    }

    /**
     * @return p1 p2 p3三个顶点组成的三角形面积的两倍，也即向量p1->p2和向量p1->p3组成的平行四边形的面积
     * 返回值大于0，表示p1 p2 p3三点以逆时针方向排列；
     * 返回值小于0，表示p1 p2 p3三点以顺时针方向排列；
     * 返回0，表示p1 p2 p3三点共线或共点。
     */
    private static float vcross2(float[] verts, int p1, int p2, int p3) {
        float u1 = verts[p2 + 0] - verts[p1 + 0];
        float v1 = verts[p2 + 2] - verts[p1 + 2];
        float u2 = verts[p3 + 0] - verts[p1 + 0];
        float v2 = verts[p3 + 2] - verts[p1 + 2];
        return u1 * v2 - v1 * u2;
    }

    private static float vcross2(float[] p1, float[] p2, float[] p3) {
        float u1 = p2[0] - p1[0];
        float v1 = p2[2] - p1[2];
        float u2 = p3[0] - p1[0];
        float v2 = p3[2] - p1[2];
        return u1 * v2 - v1 * u2;
    }

    private static boolean circumCircle(float[] verts, int p1, int p2, int p3, float[] c, AtomicReference<Float> r) {
        float EPS = 1e-6f;
        // Calculate the circle relative to p1, to avoid some precision issues.
        float v1[] = new float[3];
        float v2[] = new float[3];
        float v3[] = new float[3];
        RecastVectors.sub(v2, verts, p2, p1);
        RecastVectors.sub(v3, verts, p3, p1);

        float cp = vcross2(v1, v2, v3);
        if (Math.abs(cp) > EPS) {
            float v1Sq = vdot2(v1, v1);
            float v2Sq = vdot2(v2, v2);
            float v3Sq = vdot2(v3, v3);
            // 数组c是三角形p1 p2 p3的外接圆的圆心坐标，此坐标是一个相对坐标，是把p1看成(0,0)求出的
            c[0] = (v1Sq * (v2[2] - v3[2]) + v2Sq * (v3[2] - v1[2]) + v3Sq * (v1[2] - v2[2])) / (2 * cp);
            c[1] = 0;
            c[2] = (v1Sq * (v3[0] - v2[0]) + v2Sq * (v1[0] - v3[0]) + v3Sq * (v2[0] - v1[0])) / (2 * cp);
            // r是三角形p1 p2 p3的外接圆的半径
            r.set(vdist2(c, v1));
            // 经此步转换后，c变为一个世界坐标
            RecastVectors.add(c, c, verts, p1);
            return true;
        }
        RecastVectors.copy(c, verts, p1);
        r.set(0f);
        return false;
    }

    /**
     * 近似计算点p与三角形a b c平面之间的高度差
     * 注：说是“近似”，其实也是很精确的，有算法理论支撑
     * 关于barycentric coordinates的理论和计算，
     * 参考资料：https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/barycentric-coordinates
     *
     * @param p     待检测点
     * @param verts
     * @param a     三角形顶点一
     * @param b     三角形顶点二
     * @param c     三角形顶点三
     * @return 返回Float.MAX_VALUE表示点p不在三角形内部
     */
    private static float distPtTri(float[] p, float[] verts, int a, int b, int c) {
        float[] v0 = new float[3];
        float[] v1 = new float[3];
        float[] v2 = new float[3];
        RecastVectors.sub(v0, verts, c, a);
        RecastVectors.sub(v1, verts, b, a);
        RecastVectors.sub(v2, p, verts, a);

        float dot00 = vdot2(v0, v0);
        float dot01 = vdot2(v0, v1);
        float dot02 = vdot2(v0, v2);
        float dot11 = vdot2(v1, v1);
        float dot12 = vdot2(v1, v2);

        // Compute barycentric coordinates
        float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
        float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        // 也可以使用下面这种算法，u和v是点p分隔后的小三角形面积和总三角形面积的比例
        // invDenom = 1.0f/Math.abs(v0[0]*v1[2]-v1[0]*v0[2]);
        // u = Math.abs(v1[0]*v2[2]-v2[0]*v1[2])*invDenom;
        // v = Math.abs(v0[0]*v2[2]-v2[0]*v0[2])*invDenom;

        // If point lies inside the triangle, return interpolated y-coord.
        float EPS = 1e-4f;
        if (u >= -EPS && v >= -EPS && (u + v) <= 1 + EPS) {
            // 这里其实就是，如果u>=0，v>=0，u+v<=1，则表明点p在三角形内部
            float y = verts[a + 1] + v0[1] * u + v1[1] * v;
            return Math.abs(y - p[1]);
        }
        return Float.MAX_VALUE;
    }

    private static float distancePtSeg(float[] verts, int pt, int p, int q) {
        float pqx = verts[q + 0] - verts[p + 0];
        float pqy = verts[q + 1] - verts[p + 1];
        float pqz = verts[q + 2] - verts[p + 2];
        float dx = verts[pt + 0] - verts[p + 0];
        float dy = verts[pt + 1] - verts[p + 1];
        float dz = verts[pt + 2] - verts[p + 2];
        float d = pqx * pqx + pqy * pqy + pqz * pqz;
        float t = pqx * dx + pqy * dy + pqz * dz;
        if (d > 0) {
            t /= d;
        }
        if (t < 0) {
            t = 0;
        } else if (t > 1) {
            t = 1;
        }

        dx = verts[p + 0] + t * pqx - verts[pt + 0];
        dy = verts[p + 1] + t * pqy - verts[pt + 1];
        dz = verts[p + 2] + t * pqz - verts[pt + 2];

        return dx * dx + dy * dy + dz * dz;
    }

    private static float distancePtSeg2d(float[] verts, int pt, float[] poly, int p, int q) {
        float pqx = poly[q + 0] - poly[p + 0];
        float pqz = poly[q + 2] - poly[p + 2];
        float dx = verts[pt + 0] - poly[p + 0];
        float dz = verts[pt + 2] - poly[p + 2];
        float d = pqx * pqx + pqz * pqz;
        float t = pqx * dx + pqz * dz;
        if (d > 0) {
            t /= d;
        }
        if (t < 0) {
            t = 0;
        } else if (t > 1) {
            t = 1;
        }

        dx = poly[p + 0] + t * pqx - verts[pt + 0];
        dz = poly[p + 2] + t * pqz - verts[pt + 2];

        return dx * dx + dz * dz;
    }

    private static float distToTriMesh(float[] p, float[] verts, int nverts, List<Integer> tris, int ntris) {
        float dmin = Float.MAX_VALUE;
        for (int i = 0; i < ntris; ++i) {
            int va = tris.get(i * 4 + 0) * 3;
            int vb = tris.get(i * 4 + 1) * 3;
            int vc = tris.get(i * 4 + 2) * 3;
            float d = distPtTri(p, verts, va, vb, vc);
            if (d < dmin) {
                dmin = d;
            }
        }
        if (dmin == Float.MAX_VALUE) {
            return -1;
        }
        return dmin;
    }

    /**
     * 计算点到多边形各边的最短距离
     *
     * @param nvert
     * @param verts
     * @param p
     * @return 返回正值，表示点在多边形外部；返回负值，表示点在多边形内部。
     */
    private static float distToPoly(int nvert, float[] verts, float[] p) {

        float dmin = Float.MAX_VALUE;
        int i, j;
        // 布尔值c是判断点p是否在多边形verts内部
        boolean c = false;
        for (i = 0, j = nvert - 1; i < nvert; j = i++) {
            int vi = i * 3;
            int vj = j * 3;
            if (((verts[vi + 2] > p[2]) != (verts[vj + 2] > p[2])) && (p[0] < (verts[vj + 0] - verts[vi + 0])
                    * (p[2] - verts[vi + 2]) / (verts[vj + 2] - verts[vi + 2]) + verts[vi + 0])) {
                c = !c;
            }
            dmin = Math.min(dmin, distancePtSeg2d(p, 0, verts, vj, vi));
        }
        return c ? -dmin : dmin;
    }

    private static int getHeight(float fx, float fy, float fz, float cs, float ics, float ch, int radius,
                                 HeightPatch hp) {
        int ix = (int) Math.floor(fx * ics + 0.01f);
        int iz = (int) Math.floor(fz * ics + 0.01f);
        ix = RecastCommon.clamp(ix - hp.xmin, 0, hp.width - 1);
        iz = RecastCommon.clamp(iz - hp.ymin, 0, hp.height - 1);
        int h = hp.data[ix + iz * hp.width];
        if (h == RC_UNSET_HEIGHT) {
            // Special case when data might be bad.
            // Walk adjacent cells in a spiral up to 'radius', and look
            // for a pixel which has a valid height.
            int x = 1, z = 0, dx = 1, dz = 0;
            int maxSize = radius * 2 + 1;
            int maxIter = maxSize * maxSize - 1;

            int nextRingIterStart = 8;
            int nextRingIters = 16;

            float dmin = Float.MAX_VALUE;
            for (int i = 0; i < maxIter; ++i) {
                int nx = ix + x;
                int nz = iz + z;

                if (nx >= 0 && nz >= 0 && nx < hp.width && nz < hp.height) {
                    int nh = hp.data[nx + nz * hp.width];
                    if (nh != RC_UNSET_HEIGHT) {
                        float d = Math.abs(nh * ch - fy);
                        if (d < dmin) {
                            h = nh;
                            dmin = d;
                        }
                    }
                }

                // We are searching in a grid which looks approximately like this:
                // __________
                // |2 ______ 2|
                // | |1 __ 1| |
                // | | |__| | |
                // | |______| |
                // |__________|
                // We want to find the best height as close to the center cell as possible. This means that
                // if we find a height in one of the neighbor cells to the center, we don't want to
                // expand further out than the 8 neighbors - we want to limit our search to the closest
                // of these "rings", but the best height in the ring.
                // For example, the center is just 1 cell. We checked that at the entrance to the function.
                // The next "ring" contains 8 cells (marked 1 above). Those are all the neighbors to the center cell.
                // The next one again contains 16 cells (marked 2). In general each ring has 8 additional cells, which
                // can be thought of as adding 2 cells around the "center" of each side when we expand the ring.
                // Here we detect if we are about to enter the next ring, and if we are and we have found
                // a height, we abort the search.
                if (i + 1 == nextRingIterStart) {
                    if (h != RC_UNSET_HEIGHT) {
                        break;
                    }

                    nextRingIterStart += nextRingIters;
                    nextRingIters += 8;
                }

                if ((x == z) || ((x < 0) && (x == -z)) || ((x > 0) && (x == 1 - z))) {
                    int tmp = dx;
                    dx = -dz;
                    dz = tmp;
                }
                x += dx;
                z += dz;
            }
        }
        return h;
    }

    private static int findEdge(List<Integer> edges, int s, int t) {
        for (int i = 0; i < edges.size() / 4; i++) {
            int e = i * 4;
            if ((edges.get(e + 0) == s && edges.get(e + 1) == t) || (edges.get(e + 0) == t && edges.get(e + 1) == s)) {
                return i;
            }
        }
        return EV_UNDEF;
    }

    private static void addEdge(Context ctx, List<Integer> edges, int maxEdges, int s, int t, int l, int r) {
        if (edges.size() / 4 >= maxEdges) {
            throw new RuntimeException("addEdge: Too many edges (" + edges.size() / 4 + "/" + maxEdges + ").");
        }

        // Add edge if not already in the triangulation.
        int e = findEdge(edges, s, t);
        if (e == EV_UNDEF) {
            edges.add(s);
            edges.add(t);
            edges.add(l);
            edges.add(r);
        }
    }

    private static void updateLeftFace(List<Integer> edges, int e, int s, int t, int f) {
        if (edges.get(e + 0) == s && edges.get(e + 1) == t && edges.get(e + 2) == EV_UNDEF) {
            edges.set(e + 2, f);
        } else if (edges.get(e + 1) == s && edges.get(e + 0) == t && edges.get(e + 3) == EV_UNDEF) {
            edges.set(e + 3, f);
        }
    }

    private static boolean overlapSegSeg2d(float[] verts, int a, int b, int c, int d) {
        float a1 = vcross2(verts, a, b, d);
        float a2 = vcross2(verts, a, b, c);
        // a1*a2<0表示c d两点分布在线段ab的两边
        if (a1 * a2 < 0.0f) {
            float a3 = vcross2(verts, c, d, a);
            float a4 = a3 + a2 - a1;
            // a3*a4<0表示a b两点分布在线段cd的两边
            if (a3 * a4 < 0.0f) {
                return true;
            }
        }
        return false;
    }

    private static boolean overlapEdges(float[] pts, List<Integer> edges, int s1, int t1) {
        for (int i = 0; i < edges.size() / 4; ++i) {
            int s0 = edges.get(i * 4 + 0);
            int t0 = edges.get(i * 4 + 1);
            // Same or connected edges do not overlap.
            if (s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1) {
                continue;
            }
            if (overlapSegSeg2d(pts, s0 * 3, t0 * 3, s1 * 3, t1 * 3)) {
                return true;
            }
        }
        return false;
    }

    static int completeFacet(Context ctx, float[] pts, int npts, List<Integer> edges, int maxEdges, int nfaces, int e) {
        float EPS = 1e-5f;

        int edge = e * 4;

        // Cache s and t.
        int s, t;
        if (edges.get(edge + 2) == EV_UNDEF) {
            s = edges.get(edge + 0);
            t = edges.get(edge + 1);
        } else if (edges.get(edge + 3) == EV_UNDEF) {
            s = edges.get(edge + 1);
            t = edges.get(edge + 0);
        } else {
            // Edge already completed.
            return nfaces;
        }

        // Find best point on left of edge.
        int pt = npts;
        float[] c = new float[3];
        AtomicReference<Float> r = new AtomicReference<>(-1f);
        for (int u = 0; u < npts; ++u) {
            if (u == s || u == t) {
                continue;
            }
            if (vcross2(pts, s * 3, t * 3, u * 3) > EPS) {
                if (r.get() < 0) {
                    // The circle is not updated yet, do it now.
                    pt = u;
                    circumCircle(pts, s * 3, t * 3, u * 3, c, r);
                    continue;
                }
                float d = vdist2(c, pts, u * 3);
                float tol = 0.001f;
                if (d > r.get() * (1 + tol)) {
                    // Outside current circumcircle, skip.
                    continue;
                } else if (d < r.get() * (1 - tol)) {
                    // Inside safe circumcircle, update circle.
                    pt = u;
                    circumCircle(pts, s * 3, t * 3, u * 3, c, r);
                } else {
                    // Inside epsilon circum circle, do extra tests to make sure the edge is valid.
                    // s-u and t-u cannot overlap with s-pt nor t-pt if they exists.
                    if (overlapEdges(pts, edges, s, u)) {
                        continue;
                    }
                    if (overlapEdges(pts, edges, t, u)) {
                        continue;
                    }
                    // Edge is valid.
                    pt = u;
                    circumCircle(pts, s * 3, t * 3, u * 3, c, r);
                }
            }
        }

        // Add new triangle or update edge info if s-t is on hull.
        if (pt < npts) {
            // Update face information of edge being completed.
            updateLeftFace(edges, e * 4, s, t, nfaces);

            // Add new edge or update face info of old edge.
            e = findEdge(edges, pt, s);
            if (e == EV_UNDEF) {
                addEdge(ctx, edges, maxEdges, pt, s, nfaces, EV_UNDEF);
            } else {
                updateLeftFace(edges, e * 4, pt, s, nfaces);
            }

            // Add new edge or update face info of old edge.
            e = findEdge(edges, t, pt);
            if (e == EV_UNDEF) {
                addEdge(ctx, edges, maxEdges, t, pt, nfaces, EV_UNDEF);
            } else {
                updateLeftFace(edges, e * 4, t, pt, nfaces);
            }

            nfaces++;
        } else {
            updateLeftFace(edges, e * 4, s, t, EV_HULL);
        }
        return nfaces;
    }

    private static void delaunayHull(Context ctx, int npts, float[] pts, int nhull, int[] hull, List<Integer> tris) {
        int nfaces = 0;
        int maxEdges = npts * 10;
        List<Integer> edges = new ArrayList<>(64);
        for (int i = 0, j = nhull - 1; i < nhull; j = i++) {
            addEdge(ctx, edges, maxEdges, hull[j], hull[i], EV_HULL, EV_UNDEF);
        }
        int currentEdge = 0;
        while (currentEdge < edges.size() / 4) {
            if (edges.get(currentEdge * 4 + 2) == EV_UNDEF) {
                nfaces = completeFacet(ctx, pts, npts, edges, maxEdges, nfaces, currentEdge);
            }
            if (edges.get(currentEdge * 4 + 3) == EV_UNDEF) {
                nfaces = completeFacet(ctx, pts, npts, edges, maxEdges, nfaces, currentEdge);
            }
            currentEdge++;
        }
        // Create tris
        tris.clear();
        for (int i = 0; i < nfaces * 4; ++i) {
            tris.add(-1);
        }

        for (int i = 0; i < edges.size() / 4; ++i) {
            int e = i * 4;
            if (edges.get(e + 3) >= 0) {
                // Left face
                int t = edges.get(e + 3) * 4;
                if (tris.get(t + 0) == -1) {
                    tris.set(t + 0, edges.get(e + 0));
                    tris.set(t + 1, edges.get(e + 1));
                } else if (tris.get(t + 0) == edges.get(e + 1)) {
                    tris.set(t + 2, edges.get(e + 0));
                } else if (tris.get(t + 1) == edges.get(e + 0)) {
                    tris.set(t + 2, edges.get(e + 1));
                }
            }
            if (edges.get(e + 2) >= 0) {
                // Right
                int t = edges.get(e + 2) * 4;
                if (tris.get(t + 0) == -1) {
                    tris.set(t + 0, edges.get(e + 1));
                    tris.set(t + 1, edges.get(e + 0));
                } else if (tris.get(t + 0) == edges.get(e + 0)) {
                    tris.set(t + 2, edges.get(e + 1));
                } else if (tris.get(t + 1) == edges.get(e + 1)) {
                    tris.set(t + 2, edges.get(e + 0));
                }
            }
        }

        for (int i = 0; i < tris.size() / 4; ++i) {
            int t = i * 4;
            if (tris.get(t + 0) == -1 || tris.get(t + 1) == -1 || tris.get(t + 2) == -1) {
                System.err.println("Dangling! " + tris.get(t) + " " + tris.get(t + 1) + "  " + tris.get(t + 2));
                // ctx.log(RC_LOG_WARNING, "delaunayHull: Removing dangling face %d [%d,%d,%d].", i, t[0],t[1],t[2]);
                tris.set(t + 0, tris.get(tris.size() - 4));
                tris.set(t + 1, tris.get(tris.size() - 3));
                tris.set(t + 2, tris.get(tris.size() - 2));
                tris.set(t + 3, tris.get(tris.size() - 1));
                tris.remove(tris.size() - 1);
                tris.remove(tris.size() - 1);
                tris.remove(tris.size() - 1);
                tris.remove(tris.size() - 1);
                --i;
            }
        }
    }

    // Calculate minimum extend of the polygon.
    private static float polyMinExtent(float[] verts, int nverts) {
        float minDist = Float.MAX_VALUE;
        for (int i = 0; i < nverts; i++) {
            int ni = (i + 1) % nverts;
            int p1 = i * 3;
            int p2 = ni * 3;
            float maxEdgeDist = 0;
            for (int j = 0; j < nverts; j++) {
                if (j == i || j == ni) {
                    continue;
                }
                float d = distancePtSeg2d(verts, j * 3, verts, p1, p2);
                maxEdgeDist = Math.max(maxEdgeDist, d);
            }
            minDist = Math.min(minDist, maxEdgeDist);
        }
        return (float) Math.sqrt(minDist);
    }

    /**
     * 一种三角化的算法，步骤如下：
     * 1.首先从所有的顶点中选出一个周长最小的三角形，例如，选一个顶点i，i的下一个顶点记为left，i的上一个顶点记为right，周长就是right i left这三个顶点组成的三角形的周长。
     * 这里还有一个注意事项，就是顶点i必须要选择多边形原始顶点，也就是要选择这个多边形投射到xz平面后的顶点，由于高度差的原因，可能会在多边形的边上加一些顶点，i不能选这些顶点。
     * 2.以周长最小的顶点i为种子，left和right不断向外扩张，每次left和right都尝试扩张一步，
     * left向下获得nLeft，right向上获得pRight，然后看一下nLeft left right组成的三角形和left right pRight组成的三角形哪个周长更小，选择更小的那个进行扩张。
     * 3.循环第2步，直到所有的顶点都扩张完毕。
     */
    private static void triangulateHull(int nverts, float[] verts, int nhull, int[] hull, int nin, List<Integer> tris) {
        int start = 0, left = 1, right = nhull - 1;

        // Start from an ear with shortest perimeter.
        // This tends to favor well formed triangles as starting point.
        float dmin = Float.MAX_VALUE;
        for (int i = 0; i < nhull; i++) {
            if (hull[i] >= nin) {
                continue; // Ears are triangles with original vertices as middle vertex while others are actually line
            }
            // segments on edges
            int pi = RecastMesh.prev(i, nhull);
            int ni = RecastMesh.next(i, nhull);
            int pv = hull[pi] * 3;
            int cv = hull[i] * 3;
            int nv = hull[ni] * 3;
            float d = vdist2(verts, pv, cv) + vdist2(verts, cv, nv) + vdist2(verts, nv, pv);
            if (d < dmin) {
                start = i;
                left = ni;
                right = pi;
                dmin = d;
            }
        }

        // Add first triangle
        tris.add(hull[start]);
        tris.add(hull[left]);
        tris.add(hull[right]);
        tris.add(0);

        // Triangulate the polygon by moving left or right,
        // depending on which triangle has shorter perimeter.
        // This heuristic was chose emprically, since it seems
        // handle tesselated straight edges well.
        while (RecastMesh.next(left, nhull) != right) {
            // Check to see if se should advance left or right.
            int nleft = RecastMesh.next(left, nhull);
            int nright = RecastMesh.prev(right, nhull);

            int cvleft = hull[left] * 3;
            int nvleft = hull[nleft] * 3;
            int cvright = hull[right] * 3;
            int nvright = hull[nright] * 3;
            float dleft = vdist2(verts, cvleft, nvleft) + vdist2(verts, nvleft, cvright);
            float dright = vdist2(verts, cvright, nvright) + vdist2(verts, cvleft, nvright);

            if (dleft < dright) {
                tris.add(hull[left]);
                tris.add(hull[nleft]);
                tris.add(hull[right]);
                tris.add(0);
                left = nleft;
            } else {
                tris.add(hull[left]);
                tris.add(hull[nright]);
                tris.add(hull[right]);
                tris.add(0);
                right = nright;
            }
        }
    }

    private static float getJitterX(int i) {
        return (((i * 0x8da6b343) & 0xffff) / 65535.0f * 2.0f) - 1.0f;
    }

    private static float getJitterY(int i) {
        return (((i * 0xd8163841) & 0xffff) / 65535.0f * 2.0f) - 1.0f;
    }

    static int buildPolyDetail(Context ctx, float[] in, int nin, float sampleDist, float sampleMaxError,
                               int heightSearchRadius, CompactHeightfield chf, HeightPatch hp, float[] verts, List<Integer> tris) {

        List<Integer> samples = new ArrayList<>(512);

        int nverts = 0;
        float[] edge = new float[(MAX_VERTS_PER_EDGE + 1) * 3];
        // hull数组是和verts数组搭配使用的，在hull数据填充完毕后，verts数据也填充完毕，
        // 此时，verts数组可以分成hull.length块，每一块都存储着一个(x,y,z)坐标信息，也即每一块都存储着一个顶点信息，
        // 而hull数组中就按顺序存储着顶点索引，使用时，从hull数组中按顺序依次拿出索引，然后用该索引去verts数组中获取具体的顶点坐标数据。
        int[] hull = new int[MAX_VERTS];
        int nhull = 0;

        nverts = nin;

        for (int i = 0; i < nin; ++i) {
            RecastVectors.copy(verts, i * 3, in, i * 3);
        }
        tris.clear();

        float cs = chf.cs;
        float ics = 1.0f / cs;

        // Calculate minimum extents of the polygon based on input data.
        float minExtent = polyMinExtent(verts, nverts);

        // Tessellate outlines.
        // This is done in separate pass in order to ensure
        // seamless height values across the ply boundaries.
        if (sampleDist > 0) {
            for (int i = 0, j = nin - 1; i < nin; j = i++) {
                int vj = j * 3;
                int vi = i * 3;
                boolean swapped = false;
                // Make sure the segments are always handled in same order
                // using lexological sort or else there will be seams.
                // 1e-6(也就是0.000001)叫做epslon，用来抵消浮点运算中因为误差造成的相等无法判断的情况.
                // Math.abs(in[vj + 0] - in[vi + 0]) < 1e-6f 相等于 in[vj + 0] == in[vi + 0]
                // 此处进行交换后，保证了vi是一个lower-left点，vj是一个upper-right点
                if (Math.abs(in[vj + 0] - in[vi + 0]) < 1e-6f) {
                    if (in[vj + 2] > in[vi + 2]) {
                        int temp = vi;
                        vi = vj;
                        vj = temp;
                        swapped = true;
                    }
                } else {
                    if (in[vj + 0] > in[vi + 0]) {
                        int temp = vi;
                        vi = vj;
                        vj = temp;
                        swapped = true;
                    }
                }
                // Create samples along the edge.
                float dx = in[vi + 0] - in[vj + 0];
                float dy = in[vi + 1] - in[vj + 1];
                float dz = in[vi + 2] - in[vj + 2];
                float d = (float) Math.sqrt(dx * dx + dz * dz);
                int nn = 1 + (int) Math.floor(d / sampleDist);
                if (nn >= MAX_VERTS_PER_EDGE) {
                    nn = MAX_VERTS_PER_EDGE - 1;
                }
                if (nverts + nn >= MAX_VERTS) {
                    nn = MAX_VERTS - 1 - nverts;
                }

                for (int k = 0; k <= nn; ++k) {
                    float u = (float) k / (float) nn;
                    int pos = k * 3;
                    edge[pos + 0] = in[vj + 0] + dx * u;
                    edge[pos + 1] = in[vj + 1] + dy * u;
                    edge[pos + 2] = in[vj + 2] + dz * u;
                    edge[pos + 1] = getHeight(edge[pos + 0], edge[pos + 1], edge[pos + 2], cs, ics, chf.ch,
                            heightSearchRadius, hp) * chf.ch;
                }
                // Simplify samples.
                int[] idx = new int[MAX_VERTS_PER_EDGE];
                idx[0] = 0;
                idx[1] = nn;
                int nidx = 2;
                for (int k = 0; k < nidx - 1; ) {
                    int a = idx[k];
                    int b = idx[k + 1];
                    int va = a * 3;
                    int vb = b * 3;
                    // Find maximum deviation along the segment.
                    float maxd = 0;
                    int maxi = -1;
                    for (int m = a + 1; m < b; ++m) {
                        float dev = distancePtSeg(edge, m * 3, va, vb);
                        if (dev > maxd) {
                            maxd = dev;
                            maxi = m;
                        }
                    }
                    // If the max deviation is larger than accepted error,
                    // add new point, else continue to next segment.
                    if (maxi != -1 && maxd > sampleMaxError * sampleMaxError) {
                        for (int m = nidx; m > k; --m) {
                            idx[m] = idx[m - 1];
                        }
                        idx[k + 1] = maxi;
                        nidx++;
                    } else {
                        ++k;
                    }
                }

                hull[nhull++] = j;
                // Add new vertices.
                if (swapped) {
                    for (int k = nidx - 2; k > 0; --k) {
                        RecastVectors.copy(verts, nverts * 3, edge, idx[k] * 3);
                        hull[nhull++] = nverts;
                        nverts++;
                    }
                } else {
                    for (int k = 1; k < nidx - 1; ++k) {
                        RecastVectors.copy(verts, nverts * 3, edge, idx[k] * 3);
                        hull[nhull++] = nverts;
                        nverts++;
                    }
                }
            }
        }

        // If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
        if (minExtent < sampleDist * 2) {
            triangulateHull(nverts, verts, nhull, hull, nin, tris);
            return nverts;
        }

        // Tessellate the base mesh.
        // We're using the triangulateHull instead of delaunayHull as it tends to
        // create a bit better triangulation for long thin triangles when there
        // are no internal points.
        triangulateHull(nverts, verts, nhull, hull, nin, tris);

        if (tris.size() == 0) {
            // Could not triangulate the poly, make sure there is some valid data there.
            throw new RuntimeException("buildPolyDetail: Could not triangulate polygon (" + nverts + ") verts).");
        }

        if (sampleDist > 0) {
            // Create sample locations in a grid.
            float[] bmin = new float[3];
            float[] bmax = new float[3];
            RecastVectors.copy(bmin, in, 0);
            RecastVectors.copy(bmax, in, 0);
            for (int i = 1; i < nin; ++i) {
                RecastVectors.min(bmin, in, i * 3);
                RecastVectors.max(bmax, in, i * 3);
            }
            int x0 = (int) Math.floor(bmin[0] / sampleDist);
            int x1 = (int) Math.ceil(bmax[0] / sampleDist);
            int z0 = (int) Math.floor(bmin[2] / sampleDist);
            int z1 = (int) Math.ceil(bmax[2] / sampleDist);
            samples.clear();
            for (int z = z0; z < z1; ++z) {
                for (int x = x0; x < x1; ++x) {
                    float[] pt = new float[3];
                    pt[0] = x * sampleDist;
                    pt[1] = (bmax[1] + bmin[1]) * 0.5f;
                    pt[2] = z * sampleDist;
                    // Make sure the samples are not too close to the edges.
                    // 样例点选择标准：1.点必须在多边形内部；2.点不能距离边太近。
                    if (distToPoly(nin, in, pt) > -sampleDist / 2) {
                        continue;
                    }
                    samples.add(x);
                    samples.add(getHeight(pt[0], pt[1], pt[2], cs, ics, chf.ch, heightSearchRadius, hp));
                    samples.add(z);
                    samples.add(0); // Not added
                }
            }

            // Add the samples starting from the one that has the most
            // error. The procedure stops when all samples are added
            // or when the max error is within treshold.
            int nsamples = samples.size() / 4;
            for (int iter = 0; iter < nsamples; ++iter) {
                if (nverts >= MAX_VERTS) {
                    break;
                }

                // Find sample with most error.
                float[] bestpt = new float[3];
                float bestd = 0;
                int besti = -1;
                for (int i = 0; i < nsamples; ++i) {
                    int s = i * 4;
                    if (samples.get(s + 3) != 0) {
                        continue; // skip added.
                    }
                    float[] pt = new float[3];
                    // The sample location is jittered to get rid of some bad triangulations
                    // which are cause by symmetrical data from the grid structure.
                    pt[0] = samples.get(s + 0) * sampleDist + getJitterX(i) * cs * 0.1f;
                    pt[1] = samples.get(s + 1) * chf.ch;
                    pt[2] = samples.get(s + 2) * sampleDist + getJitterY(i) * cs * 0.1f;
                    float d = distToTriMesh(pt, verts, nverts, tris, tris.size() / 4);
                    if (d < 0) {
                        continue; // did not hit the mesh.
                    }
                    if (d > bestd) {
                        bestd = d;
                        besti = i;
                        bestpt = pt;
                    }
                }
                // If the max error is within accepted threshold, stop tesselating.
                if (bestd <= sampleMaxError || besti == -1) {
                    break;
                }
                // Mark sample as added.
                samples.set(besti * 4 + 3, 1);
                // Add the new sample point.
                RecastVectors.copy(verts, nverts * 3, bestpt, 0);
                nverts++;

                // Create new triangulation.
                // TODO: Incremental add instead of full rebuild.
                delaunayHull(ctx, nverts, verts, nhull, hull, tris);
            }
        }

        int ntris = tris.size() / 4;
        if (ntris > MAX_TRIS) {
            List<Integer> subList = tris.subList(0, MAX_TRIS * 4);
            tris.clear();
            tris.addAll(subList);
            throw new RuntimeException(
                    "rcBuildPolyMeshDetail: Shrinking triangle count from " + ntris + " to max " + MAX_TRIS);
        }
        return nverts;
    }

    static void seedArrayWithPolyCenter(Context ctx, CompactHeightfield chf, int[] meshpoly, int poly, int npoly,
                                        int[] verts, int bs, HeightPatch hp, List<Integer> array) {
        // Note: Reads to the compact heightfield are offset by border size (bs)
        // since border size offset is already removed from the polymesh vertices.

        int offset[] = {0, 0, -1, -1, 0, -1, 1, -1, 1, 0, 1, 1, 0, 1, -1, 1, -1, 0,};

        // Find cell closest to a poly vertex
        int startCellX = 0, startCellY = 0, startSpanIndex = -1;
        int dmin = RC_UNSET_HEIGHT;
        for (int j = 0; j < npoly && dmin > 0; ++j) {
            for (int k = 0; k < 9 && dmin > 0; ++k) {
                int ax = verts[meshpoly[poly + j] * 3 + 0] + offset[k * 2 + 0];
                int ay = verts[meshpoly[poly + j] * 3 + 1];
                int az = verts[meshpoly[poly + j] * 3 + 2] + offset[k * 2 + 1];
                if (ax < hp.xmin || ax >= hp.xmin + hp.width || az < hp.ymin || az >= hp.ymin + hp.height) {
                    continue;
                }

                CompactCell c = chf.cells[(ax + bs) + (az + bs) * chf.width];
                for (int i = c.index, ni = c.index + c.count; i < ni && dmin > 0; ++i) {
                    CompactSpan s = chf.spans[i];
                    int d = Math.abs(ay - s.y);
                    if (d < dmin) {
                        startCellX = ax;
                        startCellY = az;
                        startSpanIndex = i;
                        dmin = d;
                    }
                }
            }
        }

        // Find center of the polygon
        int pcx = 0, pcy = 0;
        for (int j = 0; j < npoly; ++j) {
            pcx += verts[meshpoly[poly + j] * 3 + 0];
            pcy += verts[meshpoly[poly + j] * 3 + 2];
        }
        pcx /= npoly;
        pcy /= npoly;

        array.clear();
        array.add(startCellX);
        array.add(startCellY);
        array.add(startSpanIndex);
        int dirs[] = {0, 1, 2, 3};
        Arrays.fill(hp.data, 0, hp.width * hp.height, 0);
        // DFS to move to the center. Note that we need a DFS here and can not just move
        // directly towards the center without recording intermediate nodes, even though the polygons
        // are convex. In very rare we can get stuck due to contour simplification if we do not
        // record nodes.
        int cx = -1, cy = -1, ci = -1;
        while (true) {
            if (array.size() < 3) {
                ctx.warn("Walk towards polygon center failed to reach center");
                break;
            }
            ci = array.remove(array.size() - 1);
            cy = array.remove(array.size() - 1);
            cx = array.remove(array.size() - 1);

            // Check if close to center of the polygon.
            if (cx == pcx && cy == pcy) {
                break;
            }
            // If we are already at the correct X-position, prefer direction
            // directly towards the center in the Y-axis; otherwise prefer
            // direction in the X-axis
            int directDir;
            if (cx == pcx) {
                directDir = rcGetDirForOffset(0, pcy > cy ? 1 : -1);
            } else {
                directDir = rcGetDirForOffset(pcx > cx ? 1 : -1, 0);
            }

            // Push the direct dir last so we start with this on next iteration
            int tmp = dirs[3];
            dirs[3] = dirs[directDir];
            dirs[directDir] = tmp;

            CompactSpan cs = chf.spans[ci];

            for (int i = 0; i < 4; ++i) {
                int dir = dirs[i];
                if (GetCon(cs, dir) == RC_NOT_CONNECTED) {
                    continue;
                }

                int newX = cx + GetDirOffsetX(dir);
                int newY = cy + GetDirOffsetY(dir);

                int hpx = newX - hp.xmin;
                int hpy = newY - hp.ymin;
                if (hpx < 0 || hpx >= hp.width || hpy < 0 || hpy >= hp.height) {
                    continue;
                }
                if (hp.data[hpx + hpy * hp.width] != 0) {
                    continue;
                }

                hp.data[hpx + hpy * hp.width] = 1;

                array.add(newX);
                array.add(newY);
                array.add(chf.cells[(newX + bs) + (newY + bs) * chf.width].index + GetCon(cs, dir));
            }

            tmp = dirs[3];
            dirs[3] = dirs[directDir];
            dirs[directDir] = tmp;

        }

        array.clear();
        // getHeightData seeds are given in coordinates with borders
        array.add(cx + bs);
        array.add(cy + bs);
        array.add(ci);
        Arrays.fill(hp.data, 0, hp.width * hp.height, RC_UNSET_HEIGHT);
        CompactSpan cs = chf.spans[ci];
        hp.data[cx - hp.xmin + (cy - hp.ymin) * hp.width] = cs.y;
    }

    static final int RETRACT_SIZE = 256;

    static void push3(List<Integer> queue, int v1, int v2, int v3) {
        queue.add(v1);
        queue.add(v2);
        queue.add(v3);
    }

    /**
     * 该函数的作用是把多边形的AABB盒内的span的高度都统计出来
     * 算法分两步：
     * 1.先统计多边形内部的span高度，也即先统计AABB盒内和多边形处于同一个region的span，并顺手记录下多边形的边界。
     * 2.从多边形的边界慢慢向外扩张，统计和多边形连接在一起的span的高度。
     * 这样做的原因是，一个(x,z)坐标内可能会有多个span，针对当前多边形而言，我们所需要的高度信息是和多边形相连接的那个span的高度。
     */
    static void getHeightData(Context ctx, CompactHeightfield chf, int[] meshpolys, int poly, int npoly, int[] verts,
                              int bs, HeightPatch hp, int region) {
        // Note: Reads to the compact heightfield are offset by border size (bs)
        // since border size offset is already removed from the polymesh vertices.

        List<Integer> queue = new ArrayList<>(512);
        Arrays.fill(hp.data, 0, hp.width * hp.height, RC_UNSET_HEIGHT);

        boolean empty = true;

        // We cannot sample from this poly if it was created from polys
        // of different regions. If it was then it could potentially be overlapping
        // with polys of that region and the heights sampled here could be wrong.
        if (region != RC_MULTIPLE_REGS) {
            // Copy the height from the same region, and mark region borders
            // as seed points to fill the rest.
            for (int hz = 0; hz < hp.height; hz++) {
                int z = hp.ymin + hz + bs;
                for (int hx = 0; hx < hp.width; hx++) {
                    int x = hp.xmin + hx + bs;
                    CompactCell c = chf.cells[x + z * chf.width];
                    for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                        CompactSpan s = chf.spans[i];
                        if (s.reg == region) {
                            // Store height
                            hp.data[hx + hz * hp.width] = s.y;
                            empty = false;
                            // If any of the neighbours is not in same region,
                            // add the current location as flood fill start
                            boolean border = false;
                            for (int dir = 0; dir < 4; ++dir) {
                                if (GetCon(s, dir) != RC_NOT_CONNECTED) {
                                    int ax = x + GetDirOffsetX(dir);
                                    int az = z + GetDirOffsetY(dir);
                                    int ai = chf.cells[ax + az * chf.width].index + GetCon(s, dir);
                                    CompactSpan as = chf.spans[ai];
                                    if (as.reg != region) {
                                        border = true;
                                        break;
                                    }
                                }
                            }
                            if (border) {
                                push3(queue, x, z, i);
                            }
                            break;
                        }
                    }
                }
            }
        }

        // if the polygon does not contain any points from the current region (rare, but happens)
        // or if it could potentially be overlapping polygons of the same region,
        // then use the center as the seed point.
        if (empty) {
            seedArrayWithPolyCenter(ctx, chf, meshpolys, poly, npoly, verts, bs, hp, queue);
        }

        int head = 0;

        // We assume the seed is centered in the polygon, so a BFS to collect
        // height data will ensure we do not move onto overlapping polygons and
        // sample wrong heights.
        while (head * 3 < queue.size()) {
            int cx = queue.get(head * 3 + 0);
            int cz = queue.get(head * 3 + 1);
            int ci = queue.get(head * 3 + 2);
            head++;
            if (head >= RETRACT_SIZE) {
                // 这一步的目的是为了节省内存，执行过高度赋值的坐标就没有用了，可以删掉了
                head = 0;
                queue = queue.subList(RETRACT_SIZE * 3, queue.size());
            }

            CompactSpan cs = chf.spans[ci];
            for (int dir = 0; dir < 4; ++dir) {
                if (GetCon(cs, dir) == RC_NOT_CONNECTED) {
                    continue;
                }

                int ax = cx + GetDirOffsetX(dir);
                int az = cz + GetDirOffsetY(dir);
                int hx = ax - hp.xmin - bs;
                int hz = az - hp.ymin - bs;

                if (hx < 0 || hx >= hp.width || hz < 0 || hz >= hp.height) {
                    continue;
                }

                if (hp.data[hx + hz * hp.width] != RC_UNSET_HEIGHT) {
                    continue;
                }

                int ai = chf.cells[ax + az * chf.width].index + GetCon(cs, dir);
                CompactSpan as = chf.spans[ai];

                hp.data[hx + hz * hp.width] = as.y;
                push3(queue, ax, az, ai);
            }
        }
    }

    /**
     * @param vpoly 多边形的初始顶点信息，不包含后续在边上或者多边形内部增加的点
     * @return 返回0表示线段[va-->vb]不在多边形的边界上；返回1表示线段[va-->vb]在多边形的边界上。
     */
    static int getEdgeFlags(float[] verts, int va, int vb, float[] vpoly, int npoly) {
        // The flag returned by this function matches getDetailTriEdgeFlags in Detour.
        // Figure out if edge (va,vb) is part of the polygon boundary.
        float thrSqr = 0.001f * 0.001f;
        for (int i = 0, j = npoly - 1; i < npoly; j = i++) {
            if (distancePtSeg2d(verts, va, vpoly, j * 3, i * 3) < thrSqr
                    && distancePtSeg2d(verts, vb, vpoly, j * 3, i * 3) < thrSqr) {
                return 1;
            }
        }
        return 0;
    }

    static int getTriFlags(float[] verts, int va, int vb, int vc, float[] vpoly, int npoly) {
        int flags = 0;
        flags |= getEdgeFlags(verts, va, vb, vpoly, npoly) << 0;
        flags |= getEdgeFlags(verts, vb, vc, vpoly, npoly) << 2;
        flags |= getEdgeFlags(verts, vc, va, vpoly, npoly) << 4;
        return flags;
    }

    /// @par
    ///
    /// See the #rcConfig documentation for more information on the configuration parameters.
    ///
    /// @see rcAllocPolyMeshDetail, rcPolyMesh, rcCompactHeightfield, rcPolyMeshDetail, rcConfig
    public static PolyMeshDetail buildPolyMeshDetail(Context ctx, PolyMesh mesh, CompactHeightfield chf,
                                                     float sampleDist, float sampleMaxError) {

        ctx.startTimer("BUILD_POLYMESHDETAIL");
        if (mesh.nverts == 0 || mesh.npolys == 0) {
            return null;
        }

        PolyMeshDetail dmesh = new PolyMeshDetail();
        int nvp = mesh.nvp;
        float cs = mesh.cs;
        float ch = mesh.ch;
        float[] orig = mesh.bmin;
        int borderSize = mesh.borderSize;
        int heightSearchRadius = (int) Math.max(1, Math.ceil(mesh.maxEdgeError));

        List<Integer> tris = new ArrayList<>(512);
        float verts[] = new float[256 * 3];
        HeightPatch hp = new HeightPatch();
        int nPolyVerts = 0;
        // maxhw是所有多边形的AABB盒，在x-axis方向上的最大长度。
        // maxhh是所有多边形的AABB盒，在z-axis方向上的最大长度。
        int maxhw = 0, maxhh = 0;

        // bounds数组中4个数值为一组，用来统计每个多边形的AABB信息。
        int[] bounds = new int[mesh.npolys * 4];
        float[] poly = new float[nvp * 3];

        // Find max size for a polygon area.
        for (int i = 0; i < mesh.npolys; ++i) {
            int p = i * nvp * 2;
            bounds[i * 4 + 0] = chf.width;
            bounds[i * 4 + 1] = 0;
            bounds[i * 4 + 2] = chf.height;
            bounds[i * 4 + 3] = 0;
            for (int j = 0; j < nvp; ++j) {
                if (mesh.polys[p + j] == RC_MESH_NULL_IDX) {
                    break;
                }
                int v = mesh.polys[p + j] * 3;
                bounds[i * 4 + 0] = Math.min(bounds[i * 4 + 0], mesh.verts[v + 0]);
                bounds[i * 4 + 1] = Math.max(bounds[i * 4 + 1], mesh.verts[v + 0]);
                bounds[i * 4 + 2] = Math.min(bounds[i * 4 + 2], mesh.verts[v + 2]);
                bounds[i * 4 + 3] = Math.max(bounds[i * 4 + 3], mesh.verts[v + 2]);
                nPolyVerts++;
            }
            bounds[i * 4 + 0] = Math.max(0, bounds[i * 4 + 0] - 1);
            bounds[i * 4 + 1] = Math.min(chf.width, bounds[i * 4 + 1] + 1);
            bounds[i * 4 + 2] = Math.max(0, bounds[i * 4 + 2] - 1);
            bounds[i * 4 + 3] = Math.min(chf.height, bounds[i * 4 + 3] + 1);
            if (bounds[i * 4 + 0] >= bounds[i * 4 + 1] || bounds[i * 4 + 2] >= bounds[i * 4 + 3]) {
                continue;
            }
            maxhw = Math.max(maxhw, bounds[i * 4 + 1] - bounds[i * 4 + 0]);
            maxhh = Math.max(maxhh, bounds[i * 4 + 3] - bounds[i * 4 + 2]);
        }
        hp.data = new int[maxhw * maxhh];

        dmesh.nmeshes = mesh.npolys;
        dmesh.nverts = 0;
        dmesh.ntris = 0;
        dmesh.meshes = new int[dmesh.nmeshes * 4];

        int vcap = nPolyVerts + nPolyVerts / 2;
        int tcap = vcap * 2;

        dmesh.nverts = 0;
        dmesh.verts = new float[vcap * 3];
        dmesh.ntris = 0;
        dmesh.tris = new int[tcap * 4];

        for (int i = 0; i < mesh.npolys; ++i) {
            int p = i * nvp * 2;

            // Store polygon vertices for processing.
            int npoly = 0;
            for (int j = 0; j < nvp; ++j) {
                if (mesh.polys[p + j] == RC_MESH_NULL_IDX) {
                    break;
                }
                int v = mesh.polys[p + j] * 3;
                poly[j * 3 + 0] = mesh.verts[v + 0] * cs;
                poly[j * 3 + 1] = mesh.verts[v + 1] * ch;
                poly[j * 3 + 2] = mesh.verts[v + 2] * cs;
                npoly++;
            }

            // Get the height data from the area of the polygon.
            hp.xmin = bounds[i * 4 + 0];
            hp.ymin = bounds[i * 4 + 2];
            hp.width = bounds[i * 4 + 1] - bounds[i * 4 + 0];
            hp.height = bounds[i * 4 + 3] - bounds[i * 4 + 2];
            getHeightData(ctx, chf, mesh.polys, p, npoly, mesh.verts, borderSize, hp, mesh.regs[i]);

            // Build detail mesh.
            int nverts = buildPolyDetail(ctx, poly, npoly, sampleDist, sampleMaxError, heightSearchRadius, chf, hp,
                    verts, tris);

            // Move detail verts to world space.
            for (int j = 0; j < nverts; ++j) {
                verts[j * 3 + 0] += orig[0];
                verts[j * 3 + 1] += orig[1] + chf.ch; // Is this offset necessary? See
                // https://groups.google.com/d/msg/recastnavigation/UQFN6BGCcV0/-1Ny4koOBpkJ
                verts[j * 3 + 2] += orig[2];
            }
            // Offset poly too, will be used to flag checking.
            for (int j = 0; j < npoly; ++j) {
                poly[j * 3 + 0] += orig[0];
                poly[j * 3 + 1] += orig[1];
                poly[j * 3 + 2] += orig[2];
            }

            // Store detail submesh.
            int ntris = tris.size() / 4;

            dmesh.meshes[i * 4 + 0] = dmesh.nverts;
            dmesh.meshes[i * 4 + 1] = nverts;
            dmesh.meshes[i * 4 + 2] = dmesh.ntris;
            dmesh.meshes[i * 4 + 3] = ntris;

            // Store vertices, allocate more memory if necessary.
            if (dmesh.nverts + nverts > vcap) {
                while (dmesh.nverts + nverts > vcap) {
                    vcap += 256;
                }

                float[] newv = new float[vcap * 3];
                if (dmesh.nverts != 0) {
                    System.arraycopy(dmesh.verts, 0, newv, 0, 3 * dmesh.nverts);
                }
                dmesh.verts = newv;
            }
            for (int j = 0; j < nverts; ++j) {
                dmesh.verts[dmesh.nverts * 3 + 0] = verts[j * 3 + 0];
                dmesh.verts[dmesh.nverts * 3 + 1] = verts[j * 3 + 1];
                dmesh.verts[dmesh.nverts * 3 + 2] = verts[j * 3 + 2];
                dmesh.nverts++;
            }

            // Store triangles, allocate more memory if necessary.
            if (dmesh.ntris + ntris > tcap) {
                while (dmesh.ntris + ntris > tcap) {
                    tcap += 256;
                }
                int[] newt = new int[tcap * 4];
                if (dmesh.ntris != 0) {
                    System.arraycopy(dmesh.tris, 0, newt, 0, 4 * dmesh.ntris);
                }
                dmesh.tris = newt;
            }
            for (int j = 0; j < ntris; ++j) {
                int t = j * 4;
                dmesh.tris[dmesh.ntris * 4 + 0] = tris.get(t + 0);
                dmesh.tris[dmesh.ntris * 4 + 1] = tris.get(t + 1);
                dmesh.tris[dmesh.ntris * 4 + 2] = tris.get(t + 2);
                dmesh.tris[dmesh.ntris * 4 + 3] = getTriFlags(verts, tris.get(t + 0) * 3, tris.get(t + 1) * 3,
                        tris.get(t + 2) * 3, poly, npoly);
                dmesh.ntris++;
            }
        }

        ctx.stopTimer("BUILD_POLYMESHDETAIL");
        return dmesh;

    }

    /// @see rcAllocPolyMeshDetail, rcPolyMeshDetail
    PolyMeshDetail mergePolyMeshDetails(Context ctx, PolyMeshDetail[] meshes, int nmeshes) {
        PolyMeshDetail mesh = new PolyMeshDetail();

        ctx.startTimer("MERGE_POLYMESHDETAIL");

        int maxVerts = 0;
        int maxTris = 0;
        int maxMeshes = 0;

        for (int i = 0; i < nmeshes; ++i) {
            if (meshes[i] == null) {
                continue;
            }
            maxVerts += meshes[i].nverts;
            maxTris += meshes[i].ntris;
            maxMeshes += meshes[i].nmeshes;
        }

        mesh.nmeshes = 0;
        mesh.meshes = new int[maxMeshes * 4];
        mesh.ntris = 0;
        mesh.tris = new int[maxTris * 4];
        mesh.nverts = 0;
        mesh.verts = new float[maxVerts * 3];

        // Merge datas.
        for (int i = 0; i < nmeshes; ++i) {
            PolyMeshDetail dm = meshes[i];
            if (dm == null) {
                continue;
            }
            for (int j = 0; j < dm.nmeshes; ++j) {
                int dst = mesh.nmeshes * 4;
                int src = j * 4;
                mesh.meshes[dst + 0] = mesh.nverts + dm.meshes[src + 0];
                mesh.meshes[dst + 1] = dm.meshes[src + 1];
                mesh.meshes[dst + 2] = mesh.ntris + dm.meshes[src + 2];
                mesh.meshes[dst + 3] = dm.meshes[src + 3];
                mesh.nmeshes++;
            }

            for (int k = 0; k < dm.nverts; ++k) {
                RecastVectors.copy(mesh.verts, mesh.nverts * 3, dm.verts, k * 3);
                mesh.nverts++;
            }
            for (int k = 0; k < dm.ntris; ++k) {
                mesh.tris[mesh.ntris * 4 + 0] = dm.tris[k * 4 + 0];
                mesh.tris[mesh.ntris * 4 + 1] = dm.tris[k * 4 + 1];
                mesh.tris[mesh.ntris * 4 + 2] = dm.tris[k * 4 + 2];
                mesh.tris[mesh.ntris * 4 + 3] = dm.tris[k * 4 + 3];
                mesh.ntris++;
            }
        }
        ctx.stopTimer("MERGE_POLYMESHDETAIL");
        return mesh;
    }

}
