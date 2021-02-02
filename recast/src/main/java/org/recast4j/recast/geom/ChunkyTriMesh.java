/*
Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
recast4j Copyright (c) 2015-2019 Piotr Piastucki piotr@jtilia.org

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
package org.recast4j.recast.geom;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

public class ChunkyTriMesh {

    /**
     * 三角形的AABB包围盒信息
     */
    private static class BoundsItem {
        /**
         * 地图中第i个三角形的三个顶点中，x和z的最小坐标值
         * 即，AABB包围盒的左下角顶点坐标
         */
        private final float[] bmin = new float[2];
        /**
         * 地图中第i个三角形的三个顶点中，x和z的最大坐标值
         * 即，AABB包围盒的右上角顶点坐标
         */
        private final float[] bmax = new float[2];
        /**
         * 索引项，表示当前item是根据地图配置中的第i个三角形信息构建而来的
         */
        private int i;
    }

    /**
     * 由多个三角形组成的多边形信息
     */
    public static class ChunkyTriMeshNode {
        /**
         * 多边形AABB包围盒最小顶点坐标值
         */
        private final float[] bmin = new float[2];
        /**
         * 多边形AABB包围盒最大顶点坐标值
         */
        private final float[] bmax = new float[2];
        /**
         * 多边形索引值
         */
        private int i;
        /**
         * 组成该多边形的所有三角形顶点索引信息
         */
        public int[] tris;
    }

    /**
     * 三角形AABB包围盒数据比较器（X轴方向），最小顶点X坐标值较小的项排在前面
     */
    private class CompareItemX implements Comparator<BoundsItem> {
        @Override
        public int compare(BoundsItem a, BoundsItem b) {
            if (a.bmin[0] < b.bmin[0]) {
                return -1;
            }
            if (a.bmin[0] > b.bmin[0]) {
                return 1;
            }
            return 0;
        }
    }

    /**
     * 三角形AABB包围盒数据比较器（Z轴方向），最小顶点Z坐标值较小的项排在前面
     */
    private class CompareItemZ implements Comparator<BoundsItem> {
        @Override
        public int compare(BoundsItem a, BoundsItem b) {
            if (a.bmin[1] < b.bmin[1]) {
                return -1;
            }
            if (a.bmin[1] > b.bmin[1]) {
                return 1;
            }
            return 0;
        }
    }

    /**
     * 多边形数据集合
     */
    List<ChunkyTriMeshNode> nodes;
    /**
     * 组成{@link ChunkyTriMesh#nodes}的三角形总数量
     */
    int ntris;
    /**
     * {@link ChunkyTriMesh#nodes}中的多边形最大拥有三角形数量
     */
    int maxTrisPerChunk;

    /**
     * 计算一系列三角形在xz面的AABB包围盒信息
     *
     * @param items 单一三角形的AABB包围盒数据集合
     * @param imin  最小索引值，对应于items数组，表示需要考察的三角形数据范围
     * @param imax  最大索引值，对应于items数组，表示需要考察的三角形数据范围
     * @param bmin  imin~imax范围内的三角形组成的多边形的AABB包围盒最小值顶点坐标值
     * @param bmax  imin~imax范围内的三角形组成的多边形的AABB包围盒最大值顶点坐标值
     */
    private void calcExtends(BoundsItem[] items, int imin, int imax, float[] bmin, float[] bmax) {
        bmin[0] = items[imin].bmin[0];
        bmin[1] = items[imin].bmin[1];

        bmax[0] = items[imin].bmax[0];
        bmax[1] = items[imin].bmax[1];

        for (int i = imin + 1; i < imax; ++i) {
            BoundsItem it = items[i];
            if (it.bmin[0] < bmin[0]) {
                bmin[0] = it.bmin[0];
            }
            if (it.bmin[1] < bmin[1]) {
                bmin[1] = it.bmin[1];
            }

            if (it.bmax[0] > bmax[0]) {
                bmax[0] = it.bmax[0];
            }
            if (it.bmax[1] > bmax[1]) {
                bmax[1] = it.bmax[1];
            }
        }
    }

    private int longestAxis(float x, float y) {
        return y > x ? 1 : 0;
    }

    /**
     * 对items中的三角形数据进行处理，将其划分为多边形
     *
     * @param items        三角形数据集合
     * @param imin         最小索引值，对应于items数组，表示需要处理的三角形数据的范围
     * @param imax         最大索引值，对应于items数组，表示需要处理的三角形数据的范围
     * @param trisPerChunk 每个多边形中的三角形数量
     * @param nodes        多边形数据集合，用于存放处理好的多边形数据
     * @param inTris
     */
    private void subdivide(BoundsItem[] items, int imin, int imax, int trisPerChunk, List<ChunkyTriMeshNode> nodes,
                           int[] inTris) {
        // 待处理的三角形数量
        int inum = imax - imin;

        ChunkyTriMeshNode node = new ChunkyTriMeshNode();
        nodes.add(node);

        // 分两种情况进行处理：
        // 1.待处理的三角形数量超出规定的“每个多边形中的三角形数量”；
        // 2.待处理的三角形数量未超出规定的“每个多边形中的三角形数量”；
        if (inum <= trisPerChunk) {
            // Leaf
            calcExtends(items, imin, imax, node.bmin, node.bmax);

            // Copy triangles.
            node.i = nodes.size();
            node.tris = new int[inum * 3];

            int dst = 0;
            for (int i = imin; i < imax; ++i) {
                int src = items[i].i * 3;
                node.tris[dst++] = inTris[src];
                node.tris[dst++] = inTris[src + 1];
                node.tris[dst++] = inTris[src + 2];
            }
        } else {
            // Split
            calcExtends(items, imin, imax, node.bmin, node.bmax);

            int axis = longestAxis(node.bmax[0] - node.bmin[0], node.bmax[1] - node.bmin[1]);

            if (axis == 0) {
                // imin~imax范围内的三角形组成的多边形的AABB包围盒的z轴的跨度<=x轴的跨度
                Arrays.sort(items, imin, imax, new CompareItemX());
                // Sort along x-axis
            } else if (axis == 1) {
                // imin~imax范围内的三角形组成的多边形的AABB包围盒的z轴的跨度>x轴的跨度
                Arrays.sort(items, imin, imax, new CompareItemZ());
                // Sort along z-axis
            }

            // 三角形数量过多，一分为二
            int isplit = imin + inum / 2;

            // Left
            subdivide(items, imin, isplit, trisPerChunk, nodes, inTris);
            // Right
            subdivide(items, isplit, imax, trisPerChunk, nodes, inTris);

            // Negative index means escape.
            node.i = -nodes.size();
        }
    }

    /**
     * 把三角形划分成多边形
     *
     * @param verts        坐标值数据，三个坐标值确定一个坐标点(x,y,z)
     * @param tris         坐标点索引数据，三个坐标点确定一个三角形(v0,v1,v2)
     * @param ntris        地图中三角形的数量
     * @param trisPerChunk 每个多边形中的三角形数量
     */
    public ChunkyTriMesh(float[] verts, int[] tris, int ntris, int trisPerChunk) {
        // 这些三角形能划分成的多边形数量，凑不够一个多边形的按一个多边形算
        // 举例：假如1个多边形由4个三角形组成，如果有8个三角形，恰好可以分为2个多边形；如果有7个三角形，也可以分为2个多边形。
        int nchunks = (ntris + trisPerChunk - 1) / trisPerChunk;

        nodes = new ArrayList<>(nchunks);
        this.ntris = ntris;

        // Build tree
        BoundsItem[] items = new BoundsItem[ntris];

        for (int i = 0; i < ntris; i++) {
            // 当前三角形坐标点的起始索引值
            int t = i * 3;
            BoundsItem it = items[i] = new BoundsItem();
            it.i = i;
            // Calc triangle XZ bounds.
            // verts[tris[t] * 3 + 0] 表示第i个三角形的v0顶点的x坐标值
            // verts[tris[t] * 3 + 2] 表示第i个三角形的v0顶点的z坐标值
            it.bmin[0] = it.bmax[0] = verts[tris[t] * 3 + 0];
            it.bmin[1] = it.bmax[1] = verts[tris[t] * 3 + 2];
            // 遍历三角形的剩余两个顶点v1 v2，维护好bmin和bmax
            for (int j = 1; j < 3; ++j) {
                int v = tris[t + j] * 3;
                if (verts[v] < it.bmin[0]) {
                    it.bmin[0] = verts[v];
                }
                if (verts[v + 2] < it.bmin[1]) {
                    it.bmin[1] = verts[v + 2];
                }

                if (verts[v] > it.bmax[0]) {
                    it.bmax[0] = verts[v];
                }
                if (verts[v + 2] > it.bmax[1]) {
                    it.bmax[1] = verts[v + 2];
                }
            }
        }

        subdivide(items, 0, ntris, trisPerChunk, nodes, tris);

        // Calc max tris per node.
        maxTrisPerChunk = 0;
        for (ChunkyTriMeshNode node : nodes) {
            boolean isLeaf = node.i >= 0;
            if (!isLeaf) {
                continue;
            }
            if (node.tris.length / 3 > maxTrisPerChunk) {
                maxTrisPerChunk = node.tris.length / 3;
            }
        }

    }

    private boolean checkOverlapRect(float[] amin, float[] amax, float[] bmin, float[] bmax) {
        boolean overlap = true;
        overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
        overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
        return overlap;
    }

    public List<ChunkyTriMeshNode> getChunksOverlappingRect(float[] bmin, float[] bmax) {
        // Traverse tree
        List<ChunkyTriMeshNode> ids = new ArrayList<>();
        int i = 0;
        while (i < nodes.size()) {
            ChunkyTriMeshNode node = nodes.get(i);
            boolean overlap = checkOverlapRect(bmin, bmax, node.bmin, node.bmax);
            boolean isLeafNode = node.i >= 0;

            if (isLeafNode && overlap) {
                ids.add(node);
            }

            if (overlap || isLeafNode) {
                i++;
            } else {
                i = -node.i;
            }
        }
        return ids;
    }

}
