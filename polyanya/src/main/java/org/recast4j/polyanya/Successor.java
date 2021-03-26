package org.recast4j.polyanya;

/**
 * 后继搜寻点
 *
 * @author Caojunqi
 * @date 2021-03-25 15:16
 */
public class Successor {
    Type type;

    float[] left, right;

    int polyRightInd;

    private Successor() {
        // 私有化构造器
    }

    public static Successor valueOf(Type type, float[] left, float[] right, int polyRightInd) {
        Successor successor = new Successor();
        successor.type = type;
        successor.left = left;
        successor.right = right;
        successor.polyRightInd = polyRightInd;
        return successor;
    }

    enum Type {
        RIGHT_NON_OBSERVABLE,
        OBSERVABLE,
        LEFT_NON_OBSERVABLE,
    }

}
