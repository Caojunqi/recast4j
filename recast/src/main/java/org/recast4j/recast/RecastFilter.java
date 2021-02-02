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
package org.recast4j.recast;

public class RecastFilter {

    /// @par
    ///
    /// Allows the formation of walkable regions that will flow over low lying
    /// objects such as curbs, and up structures such as stairways.
    ///
    /// Two neighboring spans are walkable if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) < waklableClimb</tt>
    ///
    /// @warning Will override the effect of #rcFilterLedgeSpans. So if both filters are used, call
    /// #rcFilterLedgeSpans after calling this filter.
    ///
    /// 此方法的目的是把高度差小于walkableClimb的不可走span标记为可走
    /// 就像楼梯，两个楼梯阶之间有一块垂直的平面，这个平面从RecastConfig#walkableSlopeAngle的角度來看是不可行走的，
    /// 但是由于它和前一个span之间的高度差低于RecastConfig#walkableClimb，我们认为它是可以直接跨过的
    ///
    /// @see rcHeightfield, rcConfig
    public static void filterLowHangingWalkableObstacles(Context ctx, int walkableClimb, Heightfield solid) {

        ctx.startTimer("FILTER_LOW_OBSTACLES");

        int w = solid.width;
        int h = solid.height;

        for (int z = 0; z < h; ++z) {
            for (int x = 0; x < w; ++x) {
                Span ps = null;
                boolean previousWalkable = false;
                int previousArea = RecastConstants.RC_NULL_AREA;

                for (Span s = solid.spans[x + z * w]; s != null; ps = s, s = s.next) {
                    boolean walkable = s.area != RecastConstants.RC_NULL_AREA;
                    // If current span is not walkable, but there is walkable
                    // span just below it, mark the span above it walkable too.
                    if (!walkable && previousWalkable) {
                        if (Math.abs(s.smax - ps.smax) <= walkableClimb)
                            s.area = previousArea;
                    }
                    // Copy walkable flag so that it cannot propagate
                    // past multiple non-walkable objects.
                    previousWalkable = walkable;
                    previousArea = s.area;
                }
            }
        }

        ctx.stopTimer("FILTER_LOW_OBSTACLES");
    }

    /// @par
    ///
    /// A ledge is a span with one or more neighbors whose maximum is further away than @p walkableClimb
    /// from the current span's maximum.
    /// This method removes the impact of the overestimation of conservative voxelization
    /// so the resulting mesh will not have regions hanging in the air over ledges.
    ///
    /// A span is a ledge if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) > walkableClimb</tt>
    ///
    /// 如果一个span处于悬崖边，或是处在一个陡峭的山坡上，就将此span标记为不可走
    /// 判断一个span处于悬崖边：此span的4方向邻居中（ns1 ns2 ns3 ns4），最"深"的那个邻居与s的表面高度差超过了walkableClimb，就说s处在悬崖边，不可走
    /// 判断一个span处在陡峭的山坡上：此span的4方向邻居中（ns1 ns2 ns3 ns4），最“高”的那个邻居和最“深”的那个邻居的表面高度差超过了walkableClimb，就说s处在陡峭的山坡上，不可走
    ///
    /// @see rcHeightfield, rcConfig
    public static void filterLedgeSpans(Context ctx, int walkableHeight, int walkableClimb, Heightfield solid) {
        ctx.startTimer("FILTER_BORDER");

        int w = solid.width;
        int h = solid.height;
        int MAX_HEIGHT = 0xffff;

        // Mark border spans.
        for (int z = 0; z < h; ++z) {
            for (int x = 0; x < w; ++x) {
                for (Span s = solid.spans[x + z * w]; s != null; s = s.next) {
                    // Skip non walkable spans.
                    if (s.area == RecastConstants.RC_NULL_AREA)
                        continue;

                    int bot = (s.smax);
                    int top = s.next != null ? s.next.smin : MAX_HEIGHT;

                    // Find neighbours minimum height.
                    // 记录与邻居的最大高度差，邻居（ns）的smax减去当前（s）的smax
                    int minh = MAX_HEIGHT;

                    // Min and max height of accessible neighbours.
                    // 记录四个邻居中最高的邻居（ns1）和最深的邻居（ns2）之间的高度差，ns1的smax减去ns2的smax
                    int asmin = s.smax;
                    int asmax = s.smax;

                    for (int dir = 0; dir < 4; ++dir) {
                        int dx = x + RecastCommon.GetDirOffsetX(dir);
                        int dz = z + RecastCommon.GetDirOffsetY(dir);
                        // Skip neighbours which are out of bounds.
                        // 对于超出地图xz平面的位置，想象这个位置上有一个陷入地面walkableClimb深的span，且此span头顶没有其他span
                        if (dx < 0 || dz < 0 || dx >= w || dz >= h) {
                            minh = Math.min(minh, -walkableClimb - bot);
                            continue;
                        }

                        // From minus infinity to the first span.
                        // 此处想象一块陷入底面walkableClimb深的span，此span头顶就是ns，这里比较的就是这个span和s
                        Span ns = solid.spans[dx + dz * w];
                        int nbot = -walkableClimb;
                        int ntop = ns != null ? ns.smin : MAX_HEIGHT;
                        // Skip neightbour if the gap between the spans is too small.
                        if (Math.min(top, ntop) - Math.max(bot, nbot) > walkableHeight)
                            minh = Math.min(minh, nbot - bot);

                        // Rest of the spans.
                        for (ns = solid.spans[dx + dz * w]; ns != null; ns = ns.next) {
                            nbot = ns.smax;
                            ntop = ns.next != null ? ns.next.smin : MAX_HEIGHT;
                            // Skip neightbour if the gap between the spans is too small.
                            if (Math.min(top, ntop) - Math.max(bot, nbot) > walkableHeight) {
                                minh = Math.min(minh, nbot - bot);

                                // Find min/max accessible neighbour height.
                                if (Math.abs(nbot - bot) <= walkableClimb) {
                                    if (nbot < asmin)
                                        asmin = nbot;
                                    if (nbot > asmax)
                                        asmax = nbot;
                                }

                            }
                        }
                    }

                    // The current span is close to a ledge if the drop to any
                    // neighbour span is less than the walkableClimb.
                    if (minh < -walkableClimb)
                        s.area = RecastConstants.RC_NULL_AREA;

                    // If the difference between all neighbours is too large,
                    // we are at steep slope, mark the span as ledge.
                    if ((asmax - asmin) > walkableClimb) {
                        s.area = RecastConstants.RC_NULL_AREA;
                    }
                }
            }
        }

        ctx.stopTimer("FILTER_BORDER");
    }

    /// @par
    ///
    /// For this filter, the clearance above the span is the distance from the span's
    /// maximum to the next higher span's minimum. (Same grid column.)
    ///
    /// 像桌子，桌子面和桌子下面的地板都会被标为可行走区域，但是由于这两个span之间的空间太矮了，AI单位不能通过，所以桌子下面的地板应该被标为不可行走
    ///
    /// @see rcHeightfield, rcConfig
    public static void filterWalkableLowHeightSpans(Context ctx, int walkableHeight, Heightfield solid) {
        ctx.startTimer("FILTER_WALKABLE");

        int w = solid.width;
        int h = solid.height;
        int MAX_HEIGHT = 0xffff;

        // Remove walkable flag from spans which do not have enough
        // space above them for the agent to stand there.
        for (int z = 0; z < h; ++z) {
            for (int x = 0; x < w; ++x) {
                for (Span s = solid.spans[x + z * w]; s != null; s = s.next) {
                    int bot = (s.smax);
                    int top = s.next != null ? s.next.smin : MAX_HEIGHT;
                    if ((top - bot) <= walkableHeight)
                        s.area = RecastConstants.RC_NULL_AREA;
                }
            }
        }
        ctx.stopTimer("FILTER_WALKABLE");
    }
}
