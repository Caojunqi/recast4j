package org.recast4j.polyanya;

/**
 * 点在多边形中的包含情况
 *
 * @author Caojunqi
 * @date 2021-03-24 11:47
 */
public class PolyContainment {

    Type type;

    int adjacent_poly;

    // If on edge, vertex1/vertex2 represents the left/right vertices of the
    // edge when looking from a point in the poly.
    int vertex1, vertex2;

    private PolyContainment() {
        // 私有化构造器
    }

    public static PolyContainment valueOf(Type type, int adjacent_poly, int vertex1, int vertex2) {
        PolyContainment polyContainment = new PolyContainment();
        polyContainment.type = type;
        polyContainment.adjacent_poly = adjacent_poly;
        polyContainment.vertex1 = vertex1;
        polyContainment.vertex2 = vertex2;
        return polyContainment;
    }

    enum Type {
        // Does not use any ints.
        OUTSIDE,

        // Does not use any ints.
        INSIDE,

        // Uses adjacent_poly, vertex1 and vertex2.
        ON_EDGE,

        // Uses vertex1.
        ON_VERTEX,
    }
}
