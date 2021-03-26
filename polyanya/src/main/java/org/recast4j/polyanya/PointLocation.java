package org.recast4j.polyanya;

import org.recast4j.detour.MeshTile;

import javax.print.DocFlavor;
import java.util.Objects;

/**
 * 坐标点位置信息
 *
 * @author Caojunqi
 */
public class PointLocation {

    Type type;
    // polyIndex，-1表示障碍物
    int poly1, poly2;
    // If on edge, vertex1/vertex2 represents the left/right vertices of the
    // edge when looking from a point in poly1.
    int vertex1, vertex2;

    private PointLocation() {
        // 私有化构造器
    }

    public static PointLocation valueOf(Type type, int poly1, int poly2, int vertex1, int vertex2) {
        PointLocation pointLocation = new PointLocation();
        pointLocation.type = type;
        pointLocation.poly1 = poly1;
        pointLocation.poly2 = poly2;
        pointLocation.vertex1 = vertex1;
        pointLocation.vertex2 = vertex2;
        return pointLocation;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        PointLocation other = (PointLocation) o;
        if (type != other.type) {
            return false;
        }
        switch (type) {
            case NOT_ON_MESH:
                return true;
            case IN_POLYGON:
                return poly1 == other.poly1;
            case ON_MESH_BORDER:
                if (poly1 != other.poly1) {
                    return false;
                }
                if (vertex1 == other.vertex1 && vertex2 == other.vertex2) {
                    return true;
                }
                if (vertex1 == other.vertex2 && vertex2 == other.vertex1) {
                    return true;
                }
                return false;
            case ON_EDGE:
                if (poly1 == other.poly1 && poly2 == other.poly2 &&
                        vertex1 == other.vertex1 && vertex2 == other.vertex2) {
                    return true;
                }
                if (poly1 == other.poly2 && poly2 == other.poly1 &&
                        vertex1 == other.vertex2 && vertex2 == other.vertex1) {
                    return true;
                }
                return false;
            case ON_CORNER_VERTEX_AMBIG:
            case ON_CORNER_VERTEX_UNAMBIG:
            case ON_NON_CORNER_VERTEX:
                return vertex1 == other.vertex1;
            default:
                assert (false);
                return false;
        }
    }

    @Override
    public int hashCode() {
        return Objects.hash(type, poly1, poly2, vertex1, vertex2);
    }

    enum Type {
        // Does not use any ints.
        NOT_ON_MESH,

        // Uses poly1 (the polygon it is on).
        IN_POLYGON,

        // Uses poly1 (the polygon it is on) and both vertices.
        ON_MESH_BORDER,        // edge: a polygon is not traversable

        // Uses poly1, poly2 and both vertices.
        ON_EDGE,         // edge: both polygons are traversable

        // TODO:不可能事件！！后续删除！！
        // Uses vertex1.
        // Can use poly1 to specify the "grid corrected poly".
        // Will need to manually assign poly1, though.
        ON_CORNER_VERTEX_AMBIG,   // vertex; two+ polygons are not traversable

        // Uses vertex1. Also returns an arbirary traversable adjacent
        // polygon in poly1.
        ON_CORNER_VERTEX_UNAMBIG, // vertex; one polygon is not traversable

        // TODO:不可能事件！！后续删除！！
        // Uses vertex1. Also returns an arbitrary adjacent polygon in poly1.
        ON_NON_CORNER_VERTEX, // vertex: all polygons are traversable
        ;
    }
}
