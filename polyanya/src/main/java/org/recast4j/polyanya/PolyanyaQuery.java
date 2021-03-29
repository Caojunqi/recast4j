package org.recast4j.polyanya;

import org.apache.commons.lang3.Validate;
import org.recast4j.detour.*;

import java.util.*;
import java.util.function.Predicate;

import static org.recast4j.detour.DetourCommon.*;
import static org.recast4j.polyanya.PolyanyaHelper.*;
import static org.recast4j.detour.NavMesh.*;
import static java.lang.Math.*;

/**
 * 寻路工具，采用Polyanya算法
 * 参考论文：Compromise-free pathfinding on a navigation mesh. / Cui, Michael L.; Harabor, Daniel D.; Grastien, Alban.
 */
public class PolyanyaQuery {
    private final static double EPSILON = 1e-4f;
    private static final float[] PLOY_PICK_EXT = new float[]{2, 4, 2};

    /**
     * 寻路场景
     */
    private final NavMesh navMesh;
    /**
     * 起始点和终点坐标
     */
    private float[] startPos, goalPos;

    private final SearchNodeQueue openList;
    // Best g value for a specific vertex.
    private final double[] rootGValues;
    // Contains the current search id if the root has been reached by the search.
    private final int[] rootSearchIds;
    // TODO searchId和rootSearchIds似乎没啥用，后续酌情删除
    private int searchId;

    /**
     * 起始点和终点所在MeshTile
     * TODO 暂时不考虑多个MeshTile的情况
     */
    private MeshTile meshTile;
    /**
     * 起点和终点所在Poly的索引
     */
    private int startPolyIndex, goalPolyIndex;

    private SearchNode finalNode;

    // Pre-initialised variables to use in search().
    private Successor[] searchSuccessors;
    private SearchNode[] searchNodesToPush;

    public PolyanyaQuery(NavMesh navMesh, float[] startPos, float[] goalPos) {
        this.navMesh = navMesh;
        this.startPos = startPos;
        this.goalPos = goalPos;
        this.openList = new SearchNodeQueue();

        NavMeshQuery query = new NavMeshQuery(navMesh);
        QueryFilter filter = new DefaultQueryFilter();
        long startRef = query.findNearestPoly(startPos, PLOY_PICK_EXT, filter).result.getNearestRef();
        long goalRef = query.findNearestPoly(goalPos, PLOY_PICK_EXT, filter).result.getNearestRef();
        Validate.isTrue(startRef != 0 && goalRef != 0, "起点和终点坐标有误！！");
        int[] startSaltItIp = decodePolyId(startRef);
        int[] goalSaltItIp = decodePolyId(goalRef);
        Validate.isTrue(startSaltItIp[1] == goalSaltItIp[1], "起点和终点不在同一个MeshTile中！！");
        this.meshTile = navMesh.getTile(startSaltItIp[1]);
        this.startPolyIndex = startSaltItIp[2];
        this.goalPolyIndex = goalSaltItIp[2];
        this.rootGValues = new double[this.meshTile.data.header.vertCount];
        this.rootSearchIds = new int[this.meshTile.data.header.vertCount];
        this.searchId++;

        this.searchSuccessors = new Successor[navMesh.getMaxVertsPerPoly() + 2];
        this.searchNodesToPush = new SearchNode[navMesh.getMaxVertsPerPoly() + 2];
    }

    public void search() {
        generateInitialNodes();

        if (finalNode != null) {
            return;
        }

        while (!openList.isEmpty()) {
            SearchNode node = openList.pop();
            int nextPolyIndex = node.nextPolyIndex;
            if (nextPolyIndex == goalPolyIndex) {
                // Make the TRUE final node.
                // (We usually push it onto the open list, but we know it's going
                // to be immediately popped off anyway.)

                // We need to find whether we need to turn left/right to ge
                // to the goal, so we do an orientation check like how we
                // special case triangle successors.

                float[] root = node.root == -1 ? startPos : getVertexPoint(node.root);
                float[] rootGoal = vSub(goalPos, root);
                int finalRoot;
                if (vCross2D(rootGoal, vSub(node.left, root)) < -EPSILON) {
                    finalRoot = node.leftVertex;
                } else if (vCross2D(vSub(node.right, root), rootGoal) < -EPSILON) {
                    finalRoot = node.rightVertex;
                } else {
                    finalRoot = node.root;
                }

                finalNode = SearchNode.valueOf(node, finalRoot, goalPos, goalPos, -1, -1, goalPolyIndex, node.f, node.g);
                return;
            }

            // We will never update our root list here.
            int root = node.root;
            if (root != -1) {
                Validate.isTrue(root >= 0 && root < rootGValues.length);
                if (rootSearchIds[root] == searchId) {
                    // We've been here before!
                    // Check whether we've done better.
                    if (rootGValues[root] + EPSILON < node.g) {
                        continue;
                    }
                }
            }

            int numNodes = 1;
            this.searchNodesToPush[0] = node;

            // We use a do while here because the first iteration is guaranteed
            // to work.
            do {
                SearchNode curNode = searchNodesToPush[0];
                // don't forget this!!!
                if (curNode.nextPolyIndex == goalPolyIndex) {
                    break;
                }
                int numSucc = getSuccessors(curNode);
                numNodes = succToNode(curNode, searchSuccessors, numSucc, searchNodesToPush);
                if (numNodes == 1) {
                    // Did we turn?
                    if (curNode.g != searchNodesToPush[0].g) {
                        // cur_node的g值如果等于其后继者的g值，就表明，cur_node和其后继者共用一个root，不需要设置parent
                        // Turned. Set the parent of this, and set the current
                        // node pointer to this after allocating space for it.
                        searchNodesToPush[0].parent = node;
                        node = searchNodesToPush[0];
                    }
                }
            } while (numNodes == 1);

            for (int i = 0; i < numNodes; i++) {
                SearchNode curNode = searchNodesToPush[i];
                // We need to update the h value before we push!

                // if we turned before, AND we immediately broke out of the
                // do-while loop, then cur_node has a valid parent pointer
                // and cur_node is equivalent to node.
                // so: if cur_node has a parent pointer, then "node"
                // is the thing we want to push.
                SearchNode n;
                if (curNode.parent != null) {
                    n = node;
                } else {
                    n = curNode;
                    n.parent = node;
                }

                float[] nRoot = n.root == -1 ? startPos : getVertexPoint(n.root);
                n.f += getHValue(nRoot, goalPos, n.left, n.right);

                openList.push(n);
            }

        }

    }

    private void generateInitialNodes() {
        PointLocation pl = getPointLocation2D(meshTile, startPolyIndex, startPos);
        double h = vDist2D(startPos, goalPos);
        switch (pl.type) {
            // Don't bother.
            case NOT_ON_MESH:
                break;

            // Generate all in an arbirary polygon.
            case ON_CORNER_VERTEX_AMBIG:
                if (pl.poly1 == -1) {
                    break;
                }
            case ON_CORNER_VERTEX_UNAMBIG:
                // Generate all in the polygon.
            case IN_POLYGON:
            case ON_MESH_BORDER:
                SearchNode startNode = genStartNode(-1, -1, pl.poly1, h);
                pushStartSuccessor(startNode);
                break;

            case ON_EDGE:
                SearchNode startNode1 = genStartNode(pl.vertex1, pl.vertex2, pl.poly2, h);
                pushStartSuccessor(startNode1);
                if (this.finalNode != null) {
                    return;
                }
                SearchNode startNode2 = genStartNode(pl.vertex2, pl.vertex1, pl.poly1, h);
                pushStartSuccessor(startNode2);
                break;

            case ON_NON_CORNER_VERTEX:
                throw new IllegalStateException("This should not be reachable!!");
        }
    }

    public Result<List<float[]>> findStraightPath() {
        if (finalNode == null) {
            return Result.invalidParam("Cannot find straight path!!");
        }
        Stack<float[]> stack = new Stack<>();
        stack.push(goalPos);
        SearchNode curNode = finalNode;
        while (curNode != null) {
            float[] point = curNode.root == -1 ? startPos : getVertexPoint(curNode.root);
            stack.push(point);
            curNode = curNode.parent;
        }

        List<float[]> result = new ArrayList<>(stack.size());
        while (!stack.isEmpty()) {
            result.add(stack.pop());
        }
        return Result.success(result);
    }

    /**
     * 生成起始点对应的SearchNode
     */
    private SearchNode genStartNode(int leftVertex, int rightVertex, int nextPolyIndex, double h) {
        // TODO this.nodePool后续看有没有用
        return SearchNode.valueOf(null, -1, startPos, startPos, leftVertex, rightVertex, nextPolyIndex, h, 0);
    }

    /**
     * 将起始点的successor推入openList
     *
     * @param startNode 起始点对应的搜寻节点
     */
    private void pushStartSuccessor(SearchNode startNode) {
        int polyIndex = startNode.nextPolyIndex;
        if (polyIndex == -1) {
            return;
        }
        if (polyIndex == this.goalPolyIndex) {
            this.finalNode = startNode;
            return;
        }
        Poly poly = getPoly(polyIndex);

        Successor[] successors = new Successor[poly.vertCount];
        int numSucc = 0;
        for (int i = 0, j = poly.vertCount - 1; i < poly.vertCount; j = i++) {
            int vertex = poly.verts[i];
            int lastVertex = poly.verts[j];
            if (vertex == startNode.rightVertex || lastVertex == startNode.leftVertex) {
                continue;
            }
            successors[numSucc++] = Successor.valueOf(Successor.Type.OBSERVABLE, getVertexPoint(vertex), getVertexPoint(lastVertex), j);
        }

        SearchNode[] nodes = new SearchNode[numSucc];
        int numNodes = succToNode(startNode, successors, numSucc, nodes);
        for (int i = 0; i < numNodes; i++) {
            SearchNode node = nodes[i];
            float[] nodeRoot = node.root == -1 ? startPos : getVertexPoint(node.root);
            node.f += getHValue(nodeRoot, goalPos, node.left, node.right);
            node.parent = startNode;

            this.openList.push(node);
        }
    }

    /**
     * 获取指定顶点索引对应的具体顶点坐标信息
     *
     * @param vertex 顶点索引
     * @return 顶点坐标信息 (x,y,z)
     */
    private float[] getVertexPoint(int vertex) {
        float[] point = new float[3];
        vSet(point, meshTile.data.verts[vertex * 3], meshTile.data.verts[vertex * 3 + 1], meshTile.data.verts[vertex * 3 + 2]);
        return point;
    }

    /**
     * 获取多边形poly中顶点索引为index的顶点坐标信息
     *
     * @param poly  顶点所在多边形
     * @param index 顶点的相对索引，0,1,2……
     * @return 顶点的全局坐标信息 (x,y,z)
     */
    private float[] index2Point(Poly poly, int index) {
        Validate.isTrue(index < poly.vertCount);
        return getVertexPoint(poly.verts[normalise(index, poly.vertCount)]);
    }

    /**
     * 将successor转为searchNode
     */
    private int succToNode(SearchNode parent, Successor[] successors, int numSucc, SearchNode[] nodes) {
        Poly poly = getPoly(parent.nextPolyIndex);

        double rightG = -1, leftG = -1;

        int out = 0;
        for (int i = 0; i < numSucc; i++) {
            Successor succ = successors[i];
            int neiPolyIndex = poly.neis[succ.polyRightInd] - 1;
            if (neiPolyIndex == -1) {
                continue;
            }

            Poly neiPoly = getPoly(neiPolyIndex);
            // If the successor we're about to push pushes into a one-way polygon,
            // and the polygon isn't the end polygon, just continue.
            if (isOneWay(neiPoly) && neiPolyIndex != goalPolyIndex) {
                continue;
            }
            int polyLeftInd = succ.polyRightInd == poly.vertCount - 1 ? 0 : succ.polyRightInd + 1;
            int leftVertex = poly.verts[polyLeftInd];
            int rightVertex = poly.verts[succ.polyRightInd];

            switch (succ.type) {
                case RIGHT_NON_OBSERVABLE:
                    if (rightG == -1) {
                        rightG = getG(parent, parent.right);
                    }
                    out = doPushNode(parent.rightVertex, rightG, succ, leftVertex, rightVertex, neiPolyIndex, nodes, out);
                    break;
                case OBSERVABLE:
                    out = doPushNode(parent.root, parent.g, succ, leftVertex, rightVertex, neiPolyIndex, nodes, out);
                    break;
                case LEFT_NON_OBSERVABLE:
                    if (leftG == -1) {
                        leftG = getG(parent, parent.left);
                    }
                    out = doPushNode(parent.leftVertex, leftG, succ, leftVertex, rightVertex, neiPolyIndex, nodes, out);
                    break;
                default:
                    throw new IllegalStateException("This should not be reachable!!");
            }
        }
        return out;
    }

    private double getG(SearchNode parent, float[] newRoot) {
        float[] parentRoot = parent.root == -1 ? startPos : getVertexPoint(parent.root);
        return parent.g + vDist2D(parentRoot, newRoot);
    }

    private int doPushNode(int root, double g, Successor succ, int leftVertex, int rightVertex, int nextPolyIndex, SearchNode[] nodes, int out) {
        if (root != -1) {
            Validate.isTrue(root >= 0 && root < rootGValues.length);
            if (rootSearchIds[root] != searchId) {
                // First time reaching root
                rootSearchIds[root] = searchId;
                rootGValues[root] = g;
            } else {
                // We've been here before!
                // Check whether we've done better.
                if (rootGValues[root] + EPSILON < g) {
                    // We've done better!
                    return out;
                } else {
                    // This is better.
                    rootGValues[root] = g;
                }
            }
        }
        nodes[out++] = SearchNode.valueOf(null, root, succ.left, succ.right, leftVertex, rightVertex, nextPolyIndex, g, g);
        return out;
    }

    private Poly getPoly(int polyIndex) {
        Validate.isTrue(polyIndex >= 0 && polyIndex < meshTile.data.polys.length);
        return meshTile.data.polys[polyIndex];
    }

    private int getSuccessors(SearchNode node) {
        // If the next polygon is -1, we did a bad job at pruning...
        Validate.isTrue(node.nextPolyIndex != -1);

        Poly poly = meshTile.data.polys[node.nextPolyIndex];
        float[] root = node.root == -1 ? startPos : getVertexPoint(node.root);
        int out = 0;

        Validate.isTrue(getOrientation(root, node.left, node.right) != Orientation.CCW);

        // Check collinearity.
        float[] rootLeft = vSub(node.left, root);
        float[] rootRight = vSub(node.right, root);
        boolean rootEqLeft = vEqual2D(root, node.left);
        boolean rootEqRight = vEqual2D(root, node.right);
        if (rootEqLeft || rootEqRight || triArea2D(root, node.right, node.left) < EPSILON) {
            // It's collinear... but we don't know where to turn.
            // Find which endpoint is closer.
            // We can terminate early if we know the root is equal to one
            // of the endpoints.
            // Additionally, we can simply compare the absolute values of
            // the coordinates to find which is closer.
            Successor.Type succType;
            if (rootEqLeft || (!rootEqRight && abs(rootLeft[0] - rootRight[0]) < EPSILON ? abs(rootLeft[2]) < abs(rootRight[2]) : abs(rootLeft[0]) < abs(rootRight[0]))) {
                // root距离left更近
                succType = Successor.Type.LEFT_NON_OBSERVABLE;
            } else {
                succType = Successor.Type.RIGHT_NON_OBSERVABLE;
            }

            for (int i = 0, j = poly.vertCount - 1; i < poly.vertCount; j = i++) {
                int lastVertex = poly.verts[j];
                int thisVertex = poly.verts[i];
                // TODO 这里应该是错了，应该是 this_vertex == node.left_right
                if (thisVertex == node.rightVertex) {
                    continue;
                }
                float[] left = getVertexPoint(thisVertex);
                float[] right = getVertexPoint(lastVertex);
                searchSuccessors[out++] = Successor.valueOf(succType, left, right, j);
            }
            return out;
        }

        if (poly.vertCount == 3) {
            int p1; // V[p1] = t1. Used for poly_right_ind for 1-2 successors.
            int p2; // V[p2] = t2. Used for poly_right_ind for 2-3 successors.
            // Note that p3 is redundant, as that's the polygon we came from.

            // The right point of the triangle.
            float[] t1 = getVertexPoint(node.rightVertex);
            // The middle point of the triangle.
            float[] t2;
            if (poly.verts[0] == node.rightVertex) {
                p1 = 0;
                p2 = 1;
                t2 = getVertexPoint(poly.verts[1]);
            } else if (poly.verts[0] == node.leftVertex) {
                p1 = 1;
                p2 = 2;
                t2 = getVertexPoint(poly.verts[2]);
            } else {
                p1 = 2;
                p2 = 0;
                t2 = getVertexPoint(poly.verts[0]);
            }
            // The left point of the triangle.
            float[] t3 = getVertexPoint(node.leftVertex);

            float[] left = node.left;
            float[] right = node.right;

            // Now we need to check the orientation of root-L-t2.
            // TODO: precompute a shared term for getting orientation,
            // like t2 - root.
            switch (getOrientation(root, left, t2)) {
                case CCW:
                    // LI in (1, 2)
                    // RI in [1, 2)

                    // TODO: precompute shared constants (assuming the compiler
                    // doesn't)
                    float[] leftIntersect = lineIntersect2D(t1, t2, root, left);
                    float[] rightIntersect = vEqual2D(right, t1) ? t1 : lineIntersect2D(t1, t2, root, right);

                    // observable(RI, LI)
                    searchSuccessors[0] = Successor.valueOf(Successor.Type.OBSERVABLE, leftIntersect, rightIntersect, p1);

                    // if we can turn left
                    if (vEqual2D(left, t3)) {
                        // left_non_observable(LI,2)
                        searchSuccessors[1] = Successor.valueOf(Successor.Type.LEFT_NON_OBSERVABLE, t2, leftIntersect, p1);
                        // left_collinear(2,3)
                        searchSuccessors[2] = Successor.valueOf(Successor.Type.LEFT_NON_OBSERVABLE, t3, t2, p2);
                        return 3;
                    }
                    return 1;

                case COLLINEAR:
                    // LI = 2
                    // RI in [1, 2)
                    rightIntersect = vEqual2D(right, t1) ? t1 : lineIntersect2D(t1, t2, root, right);

                    // observable(RI,2)
                    searchSuccessors[0] = Successor.valueOf(Successor.Type.OBSERVABLE, t2, rightIntersect, p1);

                    // if we can turn left
                    if (vEqual2D(left, t3)) {
                        searchSuccessors[1] = Successor.valueOf(Successor.Type.LEFT_NON_OBSERVABLE, t3, t2, p2);
                        return 2;
                    }
                    return 1;

                case CW:
                    // LI in (2,3]
                    leftIntersect = vEqual2D(left, t3) ? t3 : lineIntersect2D(t2, t3, root, left);

                    // Now we need to check th orientation of root-R-t2.
                    switch (getOrientation(root, right, t2)) {
                        case CW:
                            // RI in (2,3)
                            rightIntersect = lineIntersect2D(t2, t3, root, right);

                            // if we can turn right
                            if (vEqual2D(right, t1)) {
                                // right_collinear(1, 2)
                                searchSuccessors[0] = Successor.valueOf(Successor.Type.RIGHT_NON_OBSERVABLE, t2, t1, p1);

                                // right_non_observable(2,RI)
                                searchSuccessors[1] = Successor.valueOf(Successor.Type.RIGHT_NON_OBSERVABLE, rightIntersect, t2, p2);

                                // observable(RI,LI)
                                searchSuccessors[2] = Successor.valueOf(Successor.Type.OBSERVABLE, leftIntersect, rightIntersect, p2);
                                return 3;
                            }

                            // observable(RI,LI)
                            searchSuccessors[0] = Successor.valueOf(Successor.Type.OBSERVABLE, leftIntersect, rightIntersect, p2);
                            return 1;

                        case COLLINEAR:
                            // RI = 2
                            // if we can turn right
                            if (vEqual2D(right, t1)) {
                                // right_collinear(1, 2)
                                searchSuccessors[0] = Successor.valueOf(Successor.Type.RIGHT_NON_OBSERVABLE, t2, t1, p1);

                                // observable(2,LI)
                                searchSuccessors[1] = Successor.valueOf(Successor.Type.OBSERVABLE, leftIntersect, t2, p2);
                                return 2;
                            }

                            // observable(2,LI)
                            searchSuccessors[0] = Successor.valueOf(Successor.Type.OBSERVABLE, leftIntersect, t2, p2);
                            return 1;

                        case CCW:
                            // RI in [1, 2)
                            rightIntersect = vEqual2D(right, t1) ? t1 : lineIntersect2D(t1, t2, root, right);

                            // observable(RI,2)
                            searchSuccessors[0] = Successor.valueOf(Successor.Type.OBSERVABLE, t2, rightIntersect, p1);

                            // observable(2,LI)
                            searchSuccessors[1] = Successor.valueOf(Successor.Type.OBSERVABLE, leftIntersect, t2, p2);
                            return 2;

                        default:
                            throw new IllegalStateException("This should not be reachable!!");
                    }

                default:
                    throw new IllegalStateException("This should not be reachable!!");
            }
        }

        // It is not collinear.
        // Find the starting vertex (the "right" vertex).

        // Note that "_ind" means "index in V/P",
        // "_vertex" means "index of mesh_vertices".
        // "_vertex_obj" means "object of the vertex" and
        // "_p" means "point".
        int rightIndex = -1;
        for (int temp = 0; temp < poly.vertCount; temp++) {
            if (poly.verts[temp] == node.rightVertex) {
                rightIndex = temp;
                break;
            }
        }
        Validate.isTrue(rightIndex != -1);
        // Note that left_ind MUST be greater than right_ind.
        // This will make binary searching easier.
        int leftIndex = poly.vertCount + rightIndex - 1;
        Validate.isTrue(poly.verts[leftIndex] == node.leftVertex);

        // Find whether we can turn at either endpoint
        float[] rightPoint = getVertexPoint(node.rightVertex);
        float[] leftPoint = getVertexPoint(node.leftVertex);
        boolean rightLiesVertex = vEqual2D(rightPoint, node.right);
        boolean leftLiesVertex = vEqual2D(leftPoint, node.left);

        // find the transition between non-observable-right and observable.
        // we will call this A, defined by:
        // "first P such that root-right-p is strictly CCW".
        // lower bound is right+1, as root-right-right is not CCW (it is collinear).
        // upper bound is left.
        // the "transition" will lie in the range [A-1, A)
        final float[] root_right = vSub(node.right, root);
        int A = -1;
        if (rightLiesVertex) {
            // Check whether root-right-right+1 is collinear or CCW.
            float[] right_p1 = index2Point(poly, normalise(rightIndex + 1, poly.vertCount));
            if (vCross2D(root_right, vSub(right_p1, node.right)) > -EPSILON) {
                A = rightIndex + 1;
            }
        } else {
            A = binarySearch(poly, rightIndex + 1, leftIndex, point -> {
                // STRICTLY CCW
                return vCross2D(root_right, vSub(point, node.right)) > EPSILON;
            }, false);
        }
        Validate.isTrue(A != -1);

        int normalised_A = normalise(A, poly.vertCount);
        int normalised_Am1 = normalise(A - 1, poly.vertCount);
        float[] A_p = index2Point(poly, normalised_A);
        float[] Am1_p = index2Point(poly, normalised_Am1);
        float[] rightIntersect = rightLiesVertex && A == rightIndex + 1 ? node.right : lineIntersect2D(A_p, Am1_p, root, node.right);

        // find the transition between observable and non-observable-left.
        // we will call this B, defined by:
        // "first P such that root-left-p is strictly CW".
        // lower-bound is A - 1 (in the same segment as A).
        // upper bound is left-1, as we don't want root-left-left.
        // the "transition" will lie in the range (B, B+1]
        final float[] root_left = vSub(node.left, root);
        int B = -1;
        if (leftLiesVertex) {
            // Check whether root-left-left-1 is collinear or CW.
            float[] left_m1 = index2Point(poly, normalise(leftIndex - 1, poly.vertCount));
            if (vCross2D(root_left, vSub(left_m1, node.left)) < EPSILON) {
                // Intersects at left, so...
                // we should use left_ind-1!
                B = leftIndex - 1;
            }
        } else {
            B = binarySearch(poly, A - 1, leftIndex - 1, point -> {
                // STRICTLY CW.
                return vCross2D(root_left, vSub(point, node.left)) < -EPSILON;
            }, true);
        }
        Validate.isTrue(B != -1);

        int normalised_B = normalise(B, poly.vertCount);
        int normalised_Bp1 = normalise(B + 1, poly.vertCount);
        float[] B_p = index2Point(poly, normalised_B);
        float[] Bp1_p = index2Point(poly, normalised_Bp1);
        float[] leftIntersect = leftLiesVertex && B == leftIndex - 1 ? node.left : lineIntersect2D(B_p, Bp1_p, root, node.left);

        if (rightLiesVertex) {
            // Generate non-observable.

            // Generate non-observable to Am1.
            // Generate non-observable from Am1 to intersect
            // if right_intersect != Am1_p.

            // We always generate successors from last_ind to cur_ind.
            // right_ind should always be normalised.
            Validate.isTrue(normalise(rightIndex, poly.vertCount) == rightIndex);
            int last_ind = rightIndex;
            int cur_ind = normalise(rightIndex + 1, poly.vertCount);

            // Generate non-observable to Am1.
            while (last_ind != normalised_Am1) {
                // Generate last-cur, turning at right.
                searchSuccessors[out++] = Successor.valueOf(Successor.Type.RIGHT_NON_OBSERVABLE, index2Point(poly, cur_ind), index2Point(poly, last_ind), last_ind);

                last_ind = cur_ind++;
                if (cur_ind == poly.vertCount) {
                    cur_ind = 0;
                }
            }
            Validate.isTrue(cur_ind == normalised_A);

            if (!vEqual2D(rightIntersect, Am1_p)) {
                // Generate Am1-right_intersect, turning at right.
                searchSuccessors[out++] = Successor.valueOf(Successor.Type.RIGHT_NON_OBSERVABLE, rightIntersect, Am1_p, normalised_Am1);
            }
        }

        // Start at Am1.
        // last_node = right_intersect
        // If index is normalised_Bp1, go from last_node to left_intersect.
        // (And terminate too!)
        // Else, go to the end and set that as last_node

        // Special case when there are NO observable successors.
        if (A == B + 2) {
            // Do nothing.
        }
        // Special case when there only exists one observable successor.
        // Note that we used the non-normalised indices for this.
        else if (A == B + 1) {
            searchSuccessors[out++] = Successor.valueOf(Successor.Type.OBSERVABLE, leftIntersect, rightIntersect, normalised_Am1);
        } else {
            // Generate first (probably non-maximal) successor
            // (right_intersect-A)
            searchSuccessors[out++] = Successor.valueOf(Successor.Type.OBSERVABLE, A_p, rightIntersect, normalised_Am1);

            // Generate all guaranteed-maximal successors.
            // Should generate B-A of them.
            int last_ind = normalised_A;
            int cur_ind = normalise(A + 1, poly.vertCount);

            int counter = 0;
            while (last_ind != normalised_B) {
                counter++;

                // Generate last-cur.
                searchSuccessors[out++] = Successor.valueOf(Successor.Type.OBSERVABLE, index2Point(poly, cur_ind), index2Point(poly, last_ind), last_ind);

                last_ind = cur_ind++;
                if (cur_ind == poly.vertCount) {
                    cur_ind = 0;
                }
            }

            Validate.isTrue(counter == B - A);

            // Generate last (probably non-maximal) successor
            // (B-left_intersect)
            searchSuccessors[out++] = Successor.valueOf(Successor.Type.OBSERVABLE, leftIntersect, B_p, normalised_B);
        }

        if (leftLiesVertex) {
            // Generate non-observable from left_intersect to Bp1_p
            // if left_intersect != Bp1_p.
            // Generate non-observable up to end.
            // Generate left_intersect-Bp1, turning at left.
            if (!vEqual2D(leftIntersect, Bp1_p)) {
                searchSuccessors[out++] = Successor.valueOf(Successor.Type.LEFT_NON_OBSERVABLE, Bp1_p, leftIntersect, normalised_B);
            }

            int last_ind = normalised_Bp1;
            int cur_ind = normalise(B + 2, poly.vertCount);

            int normalised_left_ind = normalise(leftIndex, poly.vertCount);
            while (last_ind != normalised_left_ind) {
                // Generate last_ind-cur_ind, turning at left.
                searchSuccessors[out++] = Successor.valueOf(Successor.Type.LEFT_NON_OBSERVABLE, index2Point(poly, cur_ind), index2Point(poly, last_ind), last_ind);

                last_ind = cur_ind++;
                if (cur_ind == poly.vertCount) {
                    cur_ind = 0;
                }
            }
        }
        return out;
    }

    // Assume that there exists at least one element within the range which
    // satisifies the predicate.
    public int binarySearch(Poly poly, int lower, int upper, Predicate<float[]> pred, boolean isUpperBound) {
        if (lower == upper) {
            return lower;
        }
        int bestSoFar = -1;
        while (lower <= upper) {
            int mid = lower + (upper - lower) / 2;
            boolean matchesPred = pred.test(index2Point(poly, mid));
            if (matchesPred) {
                bestSoFar = mid;
            }
            // If we're looking for an upper bound:
            //      If we match the predicate, go higher.
            //      If not, go lower.
            // If we're looking for a lower bound:
            //      If we match the predicate, go lower.
            //      If not, go higher.
            if (matchesPred == isUpperBound) {
                // Either "upper bound AND matches pred"
                // or "lower bound AND doesn't match pred"
                // We should go higher, so increase the lower bound.
                lower = mid + 1;
            } else {
                // The opposite.
                // Decrease the upper bound.
                upper = mid - 1;
            }
        }
        return bestSoFar;
    }
}
