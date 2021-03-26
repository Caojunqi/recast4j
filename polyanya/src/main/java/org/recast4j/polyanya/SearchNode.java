package org.recast4j.polyanya;

/**
 * A search node.
 * Only makes sense given a mesh and an endpoint, which the node does not store.
 * This means that the f value needs to be set manually.
 *
 * @author Caojunqi
 * @date 2021-03-24 18:01
 */
public class SearchNode implements Comparable<SearchNode> {
    SearchNode parent;

    // Note that all Points here will be in terms of a Cartesian plane.
    int root; // -1 if start

    // If possible, set the orientation of left / root / right to be
    // "if I'm standing at 'root' and look at 'left', 'right' is on my right"
    float[] left, right;

    // The left vertex of the edge the interval is lying on.
    // When generating the successors of this node, end there.
    int leftVertex;

    // The right vertex of the edge the interval is lying on.
    // When generating the successors of this node, start there.
    int rightVertex;

    // Index of the polygon we're going to "push" into.
    // Every successor must lie within this polygon.
    int nextPolyIndex;

    double f, g;

    private SearchNode() {
        // 私有化构造器
    }

    public static SearchNode valueOf(SearchNode parent, int root, float[] left, float[] right, int leftVertex, int rightVertex, int nextPolyIndex, double f, double g) {
        SearchNode searchNode = new SearchNode();
        searchNode.parent = parent;
        searchNode.root = root;
        searchNode.left = left;
        searchNode.right = right;
        searchNode.leftVertex = leftVertex;
        searchNode.rightVertex = rightVertex;
        searchNode.nextPolyIndex = nextPolyIndex;
        searchNode.f = f;
        searchNode.g = g;
        return searchNode;
    }

    // Comparison.
    // Always take the "smallest" search node in a priority queue.
    @Override
    public int compareTo(SearchNode o) {
        if (this.f == o.f) {
            // If two nodes have the same f, the one with the bigger g
            // is "smaller" to us.
            return this.g > o.g ? -1 : 1;
        }
        return this.f < o.f ? -1 : 1;
    }
}
