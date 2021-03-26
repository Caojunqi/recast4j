package org.recast4j.polyanya;

import org.apache.commons.lang3.Validate;
import org.recast4j.detour.*;

import static org.recast4j.detour.DetourCommon.*;
import static java.lang.Math.*;

/**
 * Polyanya工具方法
 *
 * @author Caojunqi
 */
public final class PolyanyaHelper {

    private final static double EPSILON = 1e-8;
    private final static float[] PLOY_PICK_EXT = new float[]{2, 4, 2};

    /**
     * @return true-指定Poly的邻接traversable poly数量小于等于1;false-指定Poly的邻接traversable poly数量大于1
     */
    public static boolean isOneWay(Poly poly) {
        boolean foundTrav = false;
        for (int i = 0; i < poly.vertCount; i++) {
            if (poly.neis[i] != 0) {
                if (!foundTrav) {
                    foundTrav = true;
                } else {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Polyanya算法中计算指定SearchNode到终点消耗的启发式函数
     * 算法简述：
     * 我们可以把一个SearchNode的root看作一个手电筒一样的光源，
     * root点和left right可以形成一个圆锥形的光柱，我们把这个光柱往前无限延伸，
     * 此时终点goal相对于这个圆锥形光柱有三种情况：
     * 1.goal和root分列在[left,right]直线的两边，且goal在圆锥形光柱内，那么此h值就是root到goal的直线距离；
     * 2.goal和root分列在[left,right]直线的两边，但goal不在圆锥形光柱内，此时分两种情况：
     * 2.1 goal点在靠近left端点的一边，此h值为distance(root,left)+distance(left,goal)
     * 2.2 goal点在靠近right端点的一边，此h值为distance(root,right)+distance(right,goal)
     * 3.goal和root在[left,right]直线的同一边，此时，我们首先把goal点关于直线[left,right]做一个对称映射，得到goal'点，
     * 然后，再根据情况1和情况2进行h值的计算。
     *
     * @param root  SearchNode的root点坐标信息
     * @param goal  寻路终点的坐标信息
     * @param left  SearchNode的interval左边端点的坐标信息
     * @param right SearchNode的interval右边端点的坐标信息
     * @return 寻路消耗h值
     */
    public static double getHValue(float[] root, float[] goal, float[] left, float[] right) {
        if (vEqual(root, left) || vEqual(root, right)) {
            return vDist2D(root, goal);
        }
        // First, check whether goal and root are on the same side of the interval.
        // If either are collinear with r/l, reflecting does nothing.
        float[] leftRight = vSub(right, left);
        float[] leftRoot = vSub(root, left);
        float[] leftGoal = vSub(goal, left);
        if ((vCross2D(leftRoot, leftRight) > 0) == (vCross2D(leftGoal, leftRight) > 0)) {
            // Need to reflect.
            goal = reflectPoint(goal, left, right);
            leftGoal = vSub(goal, left);
        }
        // Now we do the actual line intersection test.
        // abs(denom)可以认为是以[left,right]线段和[root,goal]线段构成的四边形的面积的两倍，
        // 这里之所以说“可以认为”，是因为[left,right]和[root,goal]可能没有交点。
        double denom = vCross2D(vSub(goal, root), leftRight);
        if (abs(denom) < EPSILON) {
            // Root, goal, L and R are ALL collinear!
            // Take the best one of root-L-goal and root-R-goal.

            // We can be sneaky and use the distance squared as we always want the
            // endpoint which is closest to the root.
            double rootLeftSqr = vDist2DSqr(root, left);
            double rootRightSqr = vDist2DSqr(root, right);

            // If they're the same or within an epsilon we don't care which one we
            // use.

            if (rootLeftSqr < rootRightSqr) {
                // Left is better.
                return sqrt(rootLeftSqr) + vDist2D(left, goal);
            } else {
                // Right is better.
                return sqrt(rootRightSqr) + vDist2D(right, goal);
            }

            // TODO 此处为什么不直接用vDidt2D(root,goal)，既然root left right goal四点共线
        }

        // abs(leftRightNum)是三角形[left,goal,root]的面积的2倍
        double leftRightNum = vCross2D(leftGoal, leftRoot);
        // abs(leftRightNum)/abs(denom)是[left,right]线段和[goal,root]线段的交点与right点之间的相对位置
        ZeroOnePos leftRightPos = lineIntersectBoundCheck(leftRightNum, denom);
        switch (leftRightPos) {
            case LT_ZERO:
                // Too far left.
                // Use left end point.
                return vDist2D(root, left) + vDist2D(left, goal);

            case EQ_ZERO:
            case IN_RANGE:
            case EQ_ONE:
                // Line goes through interval, so just use the direct distance.
                return vDist2D(root, goal);

            case GT_ONE:
                // Too far right.
                // Use right end point.
                return vDist2D(root, right) + vDist2D(right, goal);

            default:
                throw new IllegalStateException("This should not be reachable!!");
        }
    }

    // Returns where num / denom is in the range [0, 1].
    private static ZeroOnePos lineIntersectBoundCheck(double num, double denom) {
        // Check num / denom == 0
        if (abs(num) < EPSILON) {
            return ZeroOnePos.EQ_ZERO;
        }
        // Check num / denom == 1.
        // Note: Checking whether it is accurately near 1 requires us to check
        // |num - denom| < EPSILON^2 * denom
        // which requires a multiplication. Instead, we can assume that denom
        // is less than 1/EPSILON and check
        // |num - denom| < EPSILON
        // instead. This is less accurate but faster.
        if (abs(num - denom) < EPSILON) {
            return ZeroOnePos.EQ_ONE;
        }

        // Now we finally check whether it's greater than 1 or less than 0.
        if (denom > 0) {
            if (num < 0) {
                // strictly less than 0
                return ZeroOnePos.LT_ZERO;
            }
            if (num > denom) {
                // strictly greater than 1
                return ZeroOnePos.GT_ONE;
            }
        } else {
            if (num > 0) {
                // strictly less than 0
                return ZeroOnePos.LT_ZERO;
            }
            if (num < denom) {
                // strictly greater than 1
                return ZeroOnePos.GT_ONE;
            }
        }
        return ZeroOnePos.IN_RANGE;
    }

    /**
     * 将点point对称投射到线段[left,right]的另一边。
     *
     * @return 投射后的点坐标信息
     */
    private static float[] reflectPoint(float[] point, float[] left, float[] right) {
        double denom = vDist2DSqr(right, left);
        if (abs(denom) < EPSILON) {
            // left和right共点
            return vAdd(point, vScale(vSub(left, point), 2));
        }
        double number = vCross2D(vSub(right, point), vSub(left, point));

        // The vector r - l rotated 90 degrees counterclockwise.
        // Can imagine "multiplying" the vector by the imaginary constant.
        // 向量a(m,n)逆时针旋转90°变为(-n,m)
        // 此处是2D平面的旋转，不考虑y坐标，所以deltaRotated[1]的坐标值是不准的。
        float[] deltaRotated = new float[]{left[2] - right[2], left[1] - right[1], right[0] - left[0]};

        // point + (number/denom)*deltaRotated 是point点在线段[left,right]上的投影
        Validate.isTrue(abs(triArea2D(left, vAdd(point, vScale(deltaRotated, (float) (number / denom))), right)) < EPSILON);

        return vAdd(point, vScale(deltaRotated, (float) (2.0 * number / denom)));
    }

    /**
     * 计算指定坐标点在多边形中的位置
     *
     * @param meshTile  所处MeshTile
     * @param polyIndex 多边形索引
     * @param pos       坐标点信息
     * @return
     */
    public static PointLocation getPointLocation2D(MeshTile meshTile, int polyIndex, float[] pos) {
        Poly poly = meshTile.data.polys[polyIndex];
        PolyContainment result = polyContainsPoint(meshTile, poly, pos);
        switch (result.type) {
            case OUTSIDE:
                throw new IllegalStateException("指定坐标不应该处于Poly外部!!");
            case INSIDE:
                return PointLocation.valueOf(PointLocation.Type.IN_POLYGON, polyIndex, -1, -1, -1);
            case ON_EDGE:
                return result.adjacent_poly == -1 ?
                        PointLocation.valueOf(PointLocation.Type.ON_MESH_BORDER, polyIndex, -1, result.vertex1, result.vertex2) :
                        PointLocation.valueOf(PointLocation.Type.ON_EDGE, polyIndex, result.adjacent_poly, result.vertex1, result.vertex2);
            case ON_VERTEX:
                return PointLocation.valueOf(PointLocation.Type.ON_CORNER_VERTEX_UNAMBIG, polyIndex, -1, result.vertex1, -1);
            default:
                throw new IllegalStateException("This should not be reachable!!");
        }
    }

    private static PolyContainment polyContainsPoint(MeshTile meshTile, Poly poly, float[] pos) {
        int nv = poly.vertCount;
        float[] verts = new float[nv * 3];
        for (int i = 0; i < nv; ++i) {
            System.arraycopy(meshTile.data.verts, poly.verts[i] * 3, verts, i * 3, 3);
        }

        if (!pointInPolygon(pos, verts, nv)) {
            return PolyContainment.valueOf(PolyContainment.Type.OUTSIDE, -1, -1, -1);
        }

        float[] last = new float[3];
        vSet(last, verts[(nv - 1) * 3], verts[(nv - 1) * 3 + 1], verts[(nv - 1) * 3 + 2]);
        if (vEqual(pos, last)) {
            return PolyContainment.valueOf(PolyContainment.Type.ON_VERTEX, -1, poly.verts[nv - 1], -1);
        }
        for (int i = 0, j = nv - 1; i < nv; j = i++) {
            last = new float[3];
            vSet(last, verts[j * 3], verts[j * 3 + 1], verts[j * 3 + 2]);
            float[] cur = new float[3];
            vSet(cur, verts[i * 3], verts[i * 3 + 1], verts[i * 3 + 2]);
            if (vEqual(pos, cur)) {
                return PolyContainment.valueOf(PolyContainment.Type.ON_VERTEX, -1, poly.verts[i], -1);
            }
            if (Math.abs(triArea2D(last, pos, cur)) < EPSILON) {
                // 三点共线
                return PolyContainment.valueOf(PolyContainment.Type.ON_EDGE, poly.neis[j] - 1, poly.verts[i], poly.verts[j]);
            }
        }
        return PolyContainment.valueOf(PolyContainment.Type.INSIDE, -1, -1, -1);
    }
}
