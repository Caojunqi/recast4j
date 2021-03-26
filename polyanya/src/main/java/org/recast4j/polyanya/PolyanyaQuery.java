package org.recast4j.polyanya;

import org.apache.commons.lang3.Validate;
import org.recast4j.detour.*;

import static org.recast4j.detour.DetourCommon.*;
import static org.recast4j.polyanya.PolyanyaHelper.*;
import static org.recast4j.detour.NavMesh.*;
import static java.lang.Math.*;

/**
 * 寻路工具，采用Polyanya算法
 * 参考论文：Compromise-free pathfinding on a navigation mesh. / Cui, Michael L.; Harabor, Daniel D.; Grastien, Alban.
 */
public class PolyanyaQuery {
    private final static double EPSILON = 1e-8;
    private static final float[] PLOY_PICK_EXT = new float[]{2, 4, 2};

    /**
     * 寻路场景
     */
    private final NavMesh navMesh;
    /**
     * 起始点和终点坐标
     */
    private float[] startPos, goalPos;

    private final SearchNodePool nodePool;
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
        this.nodePool = new SearchNodePool();
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

        Validate.isTrue(triArea2D(root, node.left, node.right) >= 0);

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

        if(poly.vertCount == 3){
            int p1; // V[p1] = t2. Used for poly_left_ind for 1-2 successors.
            int p2; // V[p2] = t3. Used for poly_left_ind for 2-3 successors.
            // Note that p3 is redundant, as that's the polygon we came from.

            // The right point of the triangle.
            float[] t1 = getVertexPoint(node.rightVertex);
            // The middle point of the triangle.
            float[] t2;
            if(poly.verts[0] == node.rightVertex){
                p1 = 1;
                p2 = 2;
                t2 = getVertexPoint(poly.verts[1]);
            }else if(poly.verts[0] == node.leftVertex){
                p1 = 2;
                p2 = 0;
                t2 = getVertexPoint(poly.verts[2]);
            }else {
                p1 = 0;
                p2 = 1;
                t2 = getVertexPoint(poly.verts[0]);
            }
            // The left point of the triangle.
            float[] t3 = getVertexPoint(node.leftVertex);
        }

    }
}
