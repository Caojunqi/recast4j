package org.recast4j.polyanya;

/**
 * @author Caojunqi
 * @date 2021-03-26 12:24
 */
public enum  ZeroOnePos {
    LT_ZERO,  // n < 0
    EQ_ZERO,  // n = 0
    IN_RANGE, // 0 < n < 1
    EQ_ONE,   // n = 1
    GT_ONE,   // n > 1
}
