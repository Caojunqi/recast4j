/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4J Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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

import static org.recast4j.recast.RecastConstants.RC_NOT_CONNECTED;
import static org.recast4j.recast.RecastConstants.RC_NULL_AREA;

import java.util.Arrays;

public class RecastArea {

    /// @par
    ///
    /// Basically, any spans that are closer to a boundary or obstruction than the specified radius
    /// are marked as unwalkable.
    ///
    /// This method is usually called immediately after the heightfield has been built.
    ///
    /// @see rcCompactHeightfield, rcBuildCompactHeightfield, rcConfig::walkableRadius
    public static void erodeWalkableArea(Context ctx, int radius, CompactHeightfield chf) {
        int w = chf.width;
        int h = chf.height;
        ctx.startTimer("ERODE_AREA");

        // 为了不使用小数，dist数组中存放的距离的单位其实是半个vx(体素块)
        int[] dist = new int[chf.spanCount];
        Arrays.fill(dist, 255);
        // Mark boundary cells.  不满4个axis-neighbor的span被定义为边界块
        for (int z = 0; z < h; ++z) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + z * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    if (chf.areas[i] == RC_NULL_AREA) {
                        dist[i] = 0;
                    } else {
                        CompactSpan s = chf.spans[i];
                        int nc = 0;
                        for (int dir = 0; dir < 4; ++dir) {
                            if (RecastCommon.GetCon(s, dir) != RC_NOT_CONNECTED) {
                                int nx = x + RecastCommon.GetDirOffsetX(dir);
                                int nz = z + RecastCommon.GetDirOffsetY(dir);
                                int nidx = chf.cells[nx + nz * w].index + RecastCommon.GetCon(s, dir);
                                if (chf.areas[nidx] != RC_NULL_AREA) {
                                    nc++;
                                }
                            }
                        }
                        // At least one missing neighbour.
                        if (nc != 4)
                            dist[i] = 0;
                    }
                }
            }
        }

        int nd;

        // 这种计算距离场（distance field）的算法叫distance transform。
        // 示例图：
        // A-----B-----C-----D
        // |  S1 |  S2 |  S3 |
        // E-----F-----G-----H
        // |  S4 |  S5 |  S6 |
        // I-----J-----K-----L
        // |  S7 |  S8 |  S9 |
        // M-----N-----O-----P
        // 上图中，水平和垂直方向上两格之间的距离是2，斜方向上两格之间的距离是3（近似于2√2），即S1和S2的距离是2，S1和S4的距离是2，S1和S5的距离是3，单位是“半个体素块”
        // Pass 1，先计算span与AM边和MP边的距离，所以span的遍历顺序为 S7->S8->S9->S4->S5->S6->S1->S2->S3
        for (int z = 0; z < h; ++z) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + z * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];

                    if (RecastCommon.GetCon(s, 0) != RC_NOT_CONNECTED) {
                        // (-1,0)
                        int ax = x + RecastCommon.GetDirOffsetX(0);
                        int az = z + RecastCommon.GetDirOffsetY(0);
                        int ai = chf.cells[ax + az * w].index + RecastCommon.GetCon(s, 0);
                        CompactSpan as = chf.spans[ai];
                        nd = Math.min(dist[ai] + 2, 255);
                        if (nd < dist[i])
                            dist[i] = nd;

                        // (-1,-1)
                        if (RecastCommon.GetCon(as, 3) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.GetDirOffsetX(3);
                            int aaz = az + RecastCommon.GetDirOffsetY(3);
                            int aai = chf.cells[aax + aaz * w].index + RecastCommon.GetCon(as, 3);
                            nd = Math.min(dist[aai] + 3, 255);
                            if (nd < dist[i])
                                dist[i] = nd;
                        }
                    }
                    if (RecastCommon.GetCon(s, 3) != RC_NOT_CONNECTED) {
                        // (0,-1)
                        int ax = x + RecastCommon.GetDirOffsetX(3);
                        int az = z + RecastCommon.GetDirOffsetY(3);
                        int ai = chf.cells[ax + az * w].index + RecastCommon.GetCon(s, 3);
                        CompactSpan as = chf.spans[ai];
                        nd = Math.min(dist[ai] + 2, 255);
                        if (nd < dist[i])
                            dist[i] = nd;

                        // (1,-1)
                        if (RecastCommon.GetCon(as, 2) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.GetDirOffsetX(2);
                            int aaz = az + RecastCommon.GetDirOffsetY(2);
                            int aai = chf.cells[aax + aaz * w].index + RecastCommon.GetCon(as, 2);
                            nd = Math.min(dist[aai] + 3, 255);
                            if (nd < dist[i])
                                dist[i] = nd;
                        }
                    }
                }
            }
        }

        // Pass 2，再计算span与AD边和DP边的距离，所以span的遍历顺序为 S3->S2->S1->S6->S5->S4->S9->S8->S7
        for (int z = h - 1; z >= 0; --z) {
            for (int x = w - 1; x >= 0; --x) {
                CompactCell c = chf.cells[x + z * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];

                    if (RecastCommon.GetCon(s, 2) != RC_NOT_CONNECTED) {
                        // (1,0)
                        int ax = x + RecastCommon.GetDirOffsetX(2);
                        int az = z + RecastCommon.GetDirOffsetY(2);
                        int ai = chf.cells[ax + az * w].index + RecastCommon.GetCon(s, 2);
                        CompactSpan as = chf.spans[ai];
                        nd = Math.min(dist[ai] + 2, 255);
                        if (nd < dist[i])
                            dist[i] = nd;

                        // (1,1)
                        if (RecastCommon.GetCon(as, 1) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.GetDirOffsetX(1);
                            int aaz = az + RecastCommon.GetDirOffsetY(1);
                            int aai = chf.cells[aax + aaz * w].index + RecastCommon.GetCon(as, 1);
                            nd = Math.min(dist[aai] + 3, 255);
                            if (nd < dist[i])
                                dist[i] = nd;
                        }
                    }
                    if (RecastCommon.GetCon(s, 1) != RC_NOT_CONNECTED) {
                        // (0,1)
                        int ax = x + RecastCommon.GetDirOffsetX(1);
                        int az = z + RecastCommon.GetDirOffsetY(1);
                        int ai = chf.cells[ax + az * w].index + RecastCommon.GetCon(s, 1);
                        CompactSpan as = chf.spans[ai];
                        nd = Math.min(dist[ai] + 2, 255);
                        if (nd < dist[i])
                            dist[i] = nd;

                        // (-1,1)
                        if (RecastCommon.GetCon(as, 0) != RC_NOT_CONNECTED) {
                            int aax = ax + RecastCommon.GetDirOffsetX(0);
                            int aaz = az + RecastCommon.GetDirOffsetY(0);
                            int aai = chf.cells[aax + aaz * w].index + RecastCommon.GetCon(as, 0);
                            nd = Math.min(dist[aai] + 3, 255);
                            if (nd < dist[i])
                                dist[i] = nd;
                        }
                    }
                }
            }
        }

        // dist数组存放的距离的单位是“半个体素块”，所以此处要对radius乘以2
        int thr = radius * 2;
        for (int i = 0; i < chf.spanCount; ++i)
            if (dist[i] < thr)
                chf.areas[i] = RC_NULL_AREA;

        ctx.stopTimer("ERODE_AREA");
    }

    /// @par
    ///
    /// This filter is usually applied after applying area id's using functions
    /// such as #rcMarkBoxArea, #rcMarkConvexPolyArea, and #rcMarkCylinderArea.
    ///
    /// @see rcCompactHeightfield
    public boolean medianFilterWalkableArea(Context ctx, CompactHeightfield chf) {

        int w = chf.width;
        int h = chf.height;

        ctx.startTimer("MEDIAN_AREA");

        int[] areas = new int[chf.spanCount];

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + y * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    if (chf.areas[i] == RC_NULL_AREA) {
                        areas[i] = chf.areas[i];
                        continue;
                    }

                    int nei[] = new int[9];
                    for (int j = 0; j < 9; ++j)
                        nei[j] = chf.areas[i];

                    for (int dir = 0; dir < 4; ++dir) {
                        if (RecastCommon.GetCon(s, dir) != RC_NOT_CONNECTED) {
                            int ax = x + RecastCommon.GetDirOffsetX(dir);
                            int ay = y + RecastCommon.GetDirOffsetY(dir);
                            int ai = chf.cells[ax + ay * w].index + RecastCommon.GetCon(s, dir);
                            if (chf.areas[ai] != RC_NULL_AREA)
                                nei[dir * 2 + 0] = chf.areas[ai];

                            CompactSpan as = chf.spans[ai];
                            int dir2 = (dir + 1) & 0x3;
                            if (RecastCommon.GetCon(as, dir2) != RC_NOT_CONNECTED) {
                                int ax2 = ax + RecastCommon.GetDirOffsetX(dir2);
                                int ay2 = ay + RecastCommon.GetDirOffsetY(dir2);
                                int ai2 = chf.cells[ax2 + ay2 * w].index + RecastCommon.GetCon(as, dir2);
                                if (chf.areas[ai2] != RC_NULL_AREA)
                                    nei[dir * 2 + 1] = chf.areas[ai2];
                            }
                        }
                    }
                    Arrays.sort(nei);
                    areas[i] = nei[4];
                }
            }
        }
        chf.areas = areas;

        ctx.stopTimer("MEDIAN_AREA");

        return true;
    }

    /// @par
    ///
    /// The value of spacial parameters are in world units.
    ///
    /// @see rcCompactHeightfield, rcMedianFilterWalkableArea
    public void markBoxArea(Context ctx, float[] bmin, float[] bmax, AreaModification areaMod, CompactHeightfield chf) {
        ctx.startTimer("MARK_BOX_AREA");

        int minx = (int) ((bmin[0] - chf.bmin[0]) / chf.cs);
        int miny = (int) ((bmin[1] - chf.bmin[1]) / chf.ch);
        int minz = (int) ((bmin[2] - chf.bmin[2]) / chf.cs);
        int maxx = (int) ((bmax[0] - chf.bmin[0]) / chf.cs);
        int maxy = (int) ((bmax[1] - chf.bmin[1]) / chf.ch);
        int maxz = (int) ((bmax[2] - chf.bmin[2]) / chf.cs);

        if (maxx < 0)
            return;
        if (minx >= chf.width)
            return;
        if (maxz < 0)
            return;
        if (minz >= chf.height)
            return;

        if (minx < 0)
            minx = 0;
        if (maxx >= chf.width)
            maxx = chf.width - 1;
        if (minz < 0)
            minz = 0;
        if (maxz >= chf.height)
            maxz = chf.height - 1;

        for (int z = minz; z <= maxz; ++z) {
            for (int x = minx; x <= maxx; ++x) {
                CompactCell c = chf.cells[x + z * chf.width];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    if (s.y >= miny && s.y <= maxy) {
                        if (chf.areas[i] != RC_NULL_AREA)
                            chf.areas[i] = areaMod.apply(chf.areas[i]);
                    }
                }
            }
        }

        ctx.stopTimer("MARK_BOX_AREA");

    }

    /**
     * 判断指定点是否在多边形内部（xz平面内）
     * @param verts 多边形顶点信息，三个值为一组，表示一个顶点坐标信息
     * @param p 一个数组，拥有三个值，表示点p的三个坐标
     * @return true-点p在多边形内部；false-点p不在多边形内部
     */
    static boolean pointInPoly(float[] verts, float[] p) {
        boolean c = false;
        int i, j;
        // 这里把多边形顶点相邻两两分为一组进行判断
        for (i = 0, j = verts.length - 3; i < verts.length; j = i, i += 3) {
            int vi = i;
            int vj = j;
            // 判断逻辑一：从z轴方向上判断p是否在两个顶点的中间；判断逻辑二：从X轴方向上判断p是否在两个顶点连线的左边，使用了相似三角形。
            // 如果p既在两个顶点中间，又在两个顶点连线的左边，就把上一步的判断结果取反。
            if (((verts[vi + 2] > p[2]) != (verts[vj + 2] > p[2]))
                    && (p[0] < (verts[vj] - verts[vi]) * (p[2] - verts[vi + 2]) / (verts[vj + 2] - verts[vi + 2])
                            + verts[vi]))
                c = !c;
        }
        return c;
    }

    /// @par
    ///
    /// The value of spacial parameters are in world units.
    ///
    /// The y-values of the polygon vertices are ignored. So the polygon is effectively
    /// projected onto the xz-plane at @p hmin, then extruded to @p hmax.
    ///
    /// @see rcCompactHeightfield, rcMedianFilterWalkableArea
    public static void markConvexPolyArea(Context ctx, float[] verts, float hmin, float hmax, AreaModification areaMod,
            CompactHeightfield chf) {
        ctx.startTimer("MARK_CONVEXPOLY_AREA");

        float bmin[] = new float[3], bmax[] = new float[3];
        RecastVectors.copy(bmin, verts, 0);
        RecastVectors.copy(bmax, verts, 0);
        for (int i = 3; i < verts.length; i += 3) {
            RecastVectors.min(bmin, verts, i);
            RecastVectors.max(bmax, verts, i);
        }
        bmin[1] = hmin;
        bmax[1] = hmax;

        int minx = (int) ((bmin[0] - chf.bmin[0]) / chf.cs);
        int miny = (int) ((bmin[1] - chf.bmin[1]) / chf.ch);
        int minz = (int) ((bmin[2] - chf.bmin[2]) / chf.cs);
        int maxx = (int) ((bmax[0] - chf.bmin[0]) / chf.cs);
        int maxy = (int) ((bmax[1] - chf.bmin[1]) / chf.ch);
        int maxz = (int) ((bmax[2] - chf.bmin[2]) / chf.cs);

        if (maxx < 0)
            return;
        if (minx >= chf.width)
            return;
        if (maxz < 0)
            return;
        if (minz >= chf.height)
            return;

        if (minx < 0)
            minx = 0;
        if (maxx >= chf.width)
            maxx = chf.width - 1;
        if (minz < 0)
            minz = 0;
        if (maxz >= chf.height)
            maxz = chf.height - 1;

        // TODO: Optimize.
        for (int z = minz; z <= maxz; ++z) {
            for (int x = minx; x <= maxx; ++x) {
                CompactCell c = chf.cells[x + z * chf.width];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];
                    if (chf.areas[i] == RC_NULL_AREA)
                        continue;
                    if (s.y >= miny && s.y <= maxy) {
                        float p[] = new float[3];
                        p[0] = chf.bmin[0] + (x + 0.5f) * chf.cs;
                        p[1] = 0;
                        p[2] = chf.bmin[2] + (z + 0.5f) * chf.cs;

                        if (pointInPoly(verts, p)) {
                            chf.areas[i] = areaMod.apply(chf.areas[i]);
                        }
                    }
                }
            }
        }

        ctx.stopTimer("MARK_CONVEXPOLY_AREA");
    }

    int offsetPoly(float[] verts, int nverts, float offset, float[] outVerts, int maxOutVerts) {
        float MITER_LIMIT = 1.20f;

        int n = 0;

        for (int i = 0; i < nverts; i++) {
            int a = (i + nverts - 1) % nverts;
            int b = i;
            int c = (i + 1) % nverts;
            int va = a * 3;
            int vb = b * 3;
            int vc = c * 3;
            float dx0 = verts[vb] - verts[va];
            float dy0 = verts[vb + 2] - verts[va + 2];
            float d0 = dx0 * dx0 + dy0 * dy0;
            if (d0 > 1e-6f) {
                d0 = (float) (1.0f / Math.sqrt(d0));
                dx0 *= d0;
                dy0 *= d0;
            }
            float dx1 = verts[vc] - verts[vb];
            float dy1 = verts[vc + 2] - verts[vb + 2];
            float d1 = dx1 * dx1 + dy1 * dy1;
            if (d1 > 1e-6f) {
                d1 = (float) (1.0f / Math.sqrt(d1));
                dx1 *= d1;
                dy1 *= d1;
            }
            float dlx0 = -dy0;
            float dly0 = dx0;
            float dlx1 = -dy1;
            float dly1 = dx1;
            float cross = dx1 * dy0 - dx0 * dy1;
            float dmx = (dlx0 + dlx1) * 0.5f;
            float dmy = (dly0 + dly1) * 0.5f;
            float dmr2 = dmx * dmx + dmy * dmy;
            boolean bevel = dmr2 * MITER_LIMIT * MITER_LIMIT < 1.0f;
            if (dmr2 > 1e-6f) {
                float scale = 1.0f / dmr2;
                dmx *= scale;
                dmy *= scale;
            }

            if (bevel && cross < 0.0f) {
                if (n + 2 >= maxOutVerts)
                    return 0;
                float d = (1.0f - (dx0 * dx1 + dy0 * dy1)) * 0.5f;
                outVerts[n * 3 + 0] = verts[vb] + (-dlx0 + dx0 * d) * offset;
                outVerts[n * 3 + 1] = verts[vb + 1];
                outVerts[n * 3 + 2] = verts[vb + 2] + (-dly0 + dy0 * d) * offset;
                n++;
                outVerts[n * 3 + 0] = verts[vb] + (-dlx1 - dx1 * d) * offset;
                outVerts[n * 3 + 1] = verts[vb + 1];
                outVerts[n * 3 + 2] = verts[vb + 2] + (-dly1 - dy1 * d) * offset;
                n++;
            } else {
                if (n + 1 >= maxOutVerts)
                    return 0;
                outVerts[n * 3 + 0] = verts[vb] - dmx * offset;
                outVerts[n * 3 + 1] = verts[vb + 1];
                outVerts[n * 3 + 2] = verts[vb + 2] - dmy * offset;
                n++;
            }
        }

        return n;
    }

    /// @par
    ///
    /// The value of spacial parameters are in world units.
    ///
    /// @see rcCompactHeightfield, rcMedianFilterWalkableArea
    public void markCylinderArea(Context ctx, float[] pos, float r, float h, AreaModification areaMod,
            CompactHeightfield chf) {

        ctx.startTimer("MARK_CYLINDER_AREA");

        float bmin[] = new float[3], bmax[] = new float[3];
        bmin[0] = pos[0] - r;
        bmin[1] = pos[1];
        bmin[2] = pos[2] - r;
        bmax[0] = pos[0] + r;
        bmax[1] = pos[1] + h;
        bmax[2] = pos[2] + r;
        float r2 = r * r;

        int minx = (int) ((bmin[0] - chf.bmin[0]) / chf.cs);
        int miny = (int) ((bmin[1] - chf.bmin[1]) / chf.ch);
        int minz = (int) ((bmin[2] - chf.bmin[2]) / chf.cs);
        int maxx = (int) ((bmax[0] - chf.bmin[0]) / chf.cs);
        int maxy = (int) ((bmax[1] - chf.bmin[1]) / chf.ch);
        int maxz = (int) ((bmax[2] - chf.bmin[2]) / chf.cs);

        if (maxx < 0)
            return;
        if (minx >= chf.width)
            return;
        if (maxz < 0)
            return;
        if (minz >= chf.height)
            return;

        if (minx < 0)
            minx = 0;
        if (maxx >= chf.width)
            maxx = chf.width - 1;
        if (minz < 0)
            minz = 0;
        if (maxz >= chf.height)
            maxz = chf.height - 1;

        for (int z = minz; z <= maxz; ++z) {
            for (int x = minx; x <= maxx; ++x) {
                CompactCell c = chf.cells[x + z * chf.width];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];

                    if (chf.areas[i] == RC_NULL_AREA)
                        continue;

                    if (s.y >= miny && s.y <= maxy) {
                        float sx = chf.bmin[0] + (x + 0.5f) * chf.cs;
                        float sz = chf.bmin[2] + (z + 0.5f) * chf.cs;
                        float dx = sx - pos[0];
                        float dz = sz - pos[2];

                        if (dx * dx + dz * dz < r2) {
                            chf.areas[i] = areaMod.apply(chf.areas[i]);
                        }
                    }
                }
            }
        }
        ctx.stopTimer("MARK_CYLINDER_AREA");
    }

}
