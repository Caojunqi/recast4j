/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
import static org.recast4j.recast.RecastVectors.copy;

public class Recast {

    void calcBounds(float[] verts, int nv, float[] bmin, float[] bmax) {
        for (int i = 0; i < 3; i++) {
            bmin[i] = verts[i];
            bmax[i] = verts[i];
        }
        for (int i = 1; i < nv; ++i) {
            for (int j = 0; j < 3; j++) {
                bmin[j] = Math.min(bmin[j], verts[i * 3 + j]);
                bmax[j] = Math.max(bmax[j], verts[i * 3 + j]);
            }
        }
        // Calculate bounding box.
    }

    /**
     * 计算地图中的Grid数量
     * 将组成地图的所有三角形用AABB包围盒框起来，投射到xz坐标平面上，然后按照指定的cell大小进行分格
     *
     * @param bmin 地图AABB包围盒的最小值顶点坐标 (x,y,z)
     * @param bmax 地图AABB包围盒的最大值顶点坐标 (x,y,z)
     * @param cs   一个格子的大小，格子是正方形的
     * @return 含有两个值的int数组，表示地图在xz平面划分的格子数量，第一个值是x轴方向上的格子数量；第二个值是z轴方向上的格子数量。
     */
    public static int[] calcGridSize(float[] bmin, float[] bmax, float cs) {
        // 注，此处的计算公式中有个0.5f，作用是，对于不满一个格子的区域块，分两种情况处理：
        // 1.该区域块不满0.5个格子，省略不计；
        // 2.该区域块超过0.5个格子，按一个格子计算。
        return new int[]{(int) ((bmax[0] - bmin[0]) / cs + 0.5f), (int) ((bmax[2] - bmin[2]) / cs + 0.5f)};
    }

    public static int[] calcTileCount(float[] bmin, float[] bmax, float cs, int tileSize) {
        int[] gwh = Recast.calcGridSize(bmin, bmax, cs);
        int gw = gwh[0];
        int gh = gwh[1];
        int ts = tileSize;
        int tw = (gw + ts - 1) / ts;
        int th = (gh + ts - 1) / ts;
        return new int[]{tw, th};
    }

    /**
     * Modifies the area id of all triangles with a slope below the specified value.
     * <p>
     * See the {@link RecastConfig} documentation for more information on the configuration parameters.
     *
     * @param walkableSlopeAngle 可行走的斜坡最大角度
     * @param verts              所有顶点信息，三个值构成一个顶点(x,y,z)
     * @param tris               三角形信息，三个顶点构成一个三角形(v0,v1,v2)
     * @param nt                 三角形数量
     * @see Heightfield
     * @see Recast#clearUnwalkableTriangles,
     * @see RecastRasterization#rasterizeTriangles
     **/
    public static int[] markWalkableTriangles(Context ctx, float walkableSlopeAngle, float[] verts, int[] tris, int nt,
                                              AreaModification areaMod) {
        // 此方法体内的算法逻辑如下：
        // 假设三角形的三个顶点分别为v0 v1 v2；
        // 向量e0为v0->v1；向量e1为v0->v2
        // 计算e0和e1的叉积cross，计算的结果向量方向垂直于e0和e1构成的平面，大小为e0和e1构成的平行四边形面积
        // 对cross进行标准化，使其大小为1，这样一来cross的y坐标值就是cross向量与y轴夹角的cos值
        // cross的y坐标值分正负，正值表示cross与y轴夹角为锐角，负值表示夹角为钝角。
        // cos函数值在[0,180]范围内，随着夹角度数增大，cos值会减小，所以cos值越小表示夹角越大，cross与y轴夹角越大，也就表明e0和e1组成的平面越陡峭
        // 综上所述，cross向量的y坐标值（即norm[1]）超过walkableThr的三角形才是可以行走的区域。
        int[] areas = new int[nt];
        float walkableThr = (float) Math.cos(walkableSlopeAngle / 180.0f * Math.PI);
        float norm[] = new float[3];
        for (int i = 0; i < nt; ++i) {
            int tri = i * 3;
            calcTriNormal(verts, tris[tri], tris[tri + 1], tris[tri + 2], norm);
            // Check if the face is walkable.
            if (norm[1] > walkableThr)
                areas[i] = areaMod.apply(areas[i]);
        }
        return areas;
    }

    static void calcTriNormal(float[] verts, int v0, int v1, int v2, float[] norm) {
        float e0[] = new float[3], e1[] = new float[3];
        RecastVectors.sub(e0, verts, v1 * 3, v0 * 3);
        RecastVectors.sub(e1, verts, v2 * 3, v0 * 3);
        RecastVectors.cross(norm, e0, e1);
        RecastVectors.normalize(norm);
    }

    /// @par
    ///
    /// Only sets the area id's for the unwalkable triangles. Does not alter the
    /// area id's for walkable triangles.
    ///
    /// See the #rcConfig documentation for more information on the configuration parameters.
    ///
    /// @see rcHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
    public static void clearUnwalkableTriangles(Context ctx, float walkableSlopeAngle, float[] verts, int nv,
                                                int[] tris, int nt, int[] areas) {
        float walkableThr = (float) Math.cos(walkableSlopeAngle / 180.0f * Math.PI);

        float norm[] = new float[3];

        for (int i = 0; i < nt; ++i) {
            int tri = i * 3;
            calcTriNormal(verts, tris[tri], tris[tri + 1], tris[tri + 2], norm);
            // Check if the face is walkable.
            if (norm[1] <= walkableThr)
                areas[i] = RC_NULL_AREA;
        }
    }

    /**
     * 统计高度场中的Span数量
     */
    static int getHeightFieldSpanCount(Context ctx, Heightfield hf) {
        int w = hf.width;
        int h = hf.height;
        int spanCount = 0;
        for (int z = 0; z < h; ++z) {
            for (int x = 0; x < w; ++x) {
                for (Span s = hf.spans[x + z * w]; s != null; s = s.next) {
                    if (s.area != RC_NULL_AREA)
                        spanCount++;
                }
            }
        }
        return spanCount;
    }

    /// @par
    ///
    /// This is just the beginning of the process of fully building a compact heightfield.
    /// Various filters may be applied, then the distance field and regions built.
    /// E.g: #rcBuildDistanceField and #rcBuildRegions
    ///
    /// See the #rcConfig documentation for more information on the configuration parameters.
    ///
    /// @see rcAllocCompactHeightfield, rcHeightfield, rcCompactHeightfield, rcConfig

    public static CompactHeightfield buildCompactHeightfield(Context ctx, int walkableHeight, int walkableClimb,
                                                             Heightfield hf) {

        ctx.startTimer("BUILD_COMPACTHEIGHTFIELD");

        CompactHeightfield chf = new CompactHeightfield();
        int w = hf.width;
        int h = hf.height;
        int spanCount = getHeightFieldSpanCount(ctx, hf);

        // Fill in header.
        chf.width = w;
        chf.height = h;
        chf.spanCount = spanCount;
        chf.walkableHeight = walkableHeight;
        chf.walkableClimb = walkableClimb;
        chf.maxRegions = 0;
        copy(chf.bmin, hf.bmin);
        copy(chf.bmax, hf.bmax);
        // CompactHeightfield比SolidHeightfield高出一层walkableHeigh
        // CompactHeightfield和SolidHeightfield是互补的，SolidHeightfield是地图中的实心部分，CompactHeightfield是地图中的空心部分
        chf.bmax[1] += walkableHeight * hf.ch;
        chf.cs = hf.cs;
        chf.ch = hf.ch;
        chf.cells = new CompactCell[w * h];
        chf.spans = new CompactSpan[spanCount];
        chf.areas = new int[spanCount];
        int MAX_HEIGHT = 0xffff;
        for (int i = 0; i < chf.cells.length; i++) {
            chf.cells[i] = new CompactCell();
        }
        for (int i = 0; i < chf.spans.length; i++) {
            chf.spans[i] = new CompactSpan();
        }
        // Fill in cells and spans.
        int idx = 0;
        for (int z = 0; z < h; ++z) {
            for (int x = 0; x < w; ++x) {
                Span s = hf.spans[x + z * w];
                // If there are no spans at this cell, just leave the data to index=0, count=0.
                if (s == null)
                    continue;
                CompactCell c = chf.cells[x + z * w];
                c.index = idx;
                c.count = 0;
                while (s != null) {
                    if (s.area != RC_NULL_AREA) {
                        int bot = s.smax;
                        int top = s.next != null ? (int) s.next.smin : MAX_HEIGHT;
                        chf.spans[idx].y = RecastCommon.clamp(bot, 0, 0xffff);
                        chf.spans[idx].h = RecastCommon.clamp(top - bot, 0, 0xff);
                        chf.areas[idx] = s.area;
                        idx++;
                        c.count++;
                    }
                    s = s.next;
                }
            }
        }

        // Find neighbour connections.
        int MAX_LAYERS = RC_NOT_CONNECTED - 1;
        int tooHighNeighbour = 0;
        for (int z = 0; z < h; ++z) {
            for (int x = 0; x < w; ++x) {
                CompactCell c = chf.cells[x + z * w];
                for (int i = c.index, ni = c.index + c.count; i < ni; ++i) {
                    CompactSpan s = chf.spans[i];

                    for (int dir = 0; dir < 4; ++dir) {
                        RecastCommon.SetCon(s, dir, RC_NOT_CONNECTED);
                        int nx = x + RecastCommon.GetDirOffsetX(dir);
                        int nz = z + RecastCommon.GetDirOffsetY(dir);
                        // First check that the neighbour cell is in bounds.
                        if (nx < 0 || nz < 0 || nx >= w || nz >= h)
                            continue;

                        // Iterate over all neighbour spans and check if any of the is
                        // accessible from current cell.
                        CompactCell nc = chf.cells[nx + nz * w];
                        for (int k = nc.index, nk = nc.index + nc.count; k < nk; ++k) {
                            CompactSpan ns = chf.spans[k];
                            int bot = Math.max(s.y, ns.y);
                            int top = Math.min(s.y + s.h, ns.y + ns.h);

                            // Check that the gap between the spans is walkable,
                            // and that the climb height between the gaps is not too high.
                            if ((top - bot) >= walkableHeight && Math.abs(ns.y - s.y) <= walkableClimb) {
                                // Mark direction as walkable.
                                int lidx = k - nc.index;
                                if (lidx < 0 || lidx > MAX_LAYERS) {
                                    tooHighNeighbour = Math.max(tooHighNeighbour, lidx);
                                    continue;
                                }
                                RecastCommon.SetCon(s, dir, lidx);
                                break;
                            }
                        }

                    }
                }
            }
        }

        if (tooHighNeighbour > MAX_LAYERS) {
            throw new RuntimeException("rcBuildCompactHeightfield: Heightfield has too many layers " + tooHighNeighbour
                    + " (max: " + MAX_LAYERS + ")");
        }
        ctx.stopTimer("BUILD_COMPACTHEIGHTFIELD");
        return chf;
    }
}
