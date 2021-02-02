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

/**
 * Represents a heightfield layer within a layer set.
 */
public class Heightfield {

    /**
     * The width of the heightfield. (Along the x-axis in cell units.)
     */
    public final int width;
    /**
     * The height of the heightfield. (Along the z-axis in cell units.)
     */
    public final int height;
    /**
     * The minimum bounds in world space. [(x, y, z)]
     */
    public final float[] bmin;
    /**
     * The maximum bounds in world space. [(x, y, z)]
     */
    public final float[] bmax;
    /**
     * The size of each cell. (On the xz-plane.)
     */
    public final float cs;
    /**
     * The height of each cell. (The minimum increment along the y-axis.)
     */
    public final float ch;
    /**
     * Heightfield of spans (width*height).
     * 数据结构解释：将Heightfield在xz平面上的投影进行分格，x轴方向上有width个格子，z轴方向上有height个格子，对每个格子进行编号，编号为x+z*width
     * 每个格子上对应一个处于最底层的Span，所以spans中一共有width*height个格子
     * Span的next是位于同一格上，处于自己正上方的Span
     */
    public final Span[] spans;

    public Heightfield(int width, int height, float[] bmin, float[] bmax, float cs, float ch) {
        this.width = width;
        this.height = height;
        this.bmin = bmin;
        this.bmax = bmax;
        this.cs = cs;
        this.ch = ch;
        spans = new Span[width * height];

    }
}
