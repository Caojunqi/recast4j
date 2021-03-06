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
 * Represents a span of unobstructed space within a compact heightfield.
 */
public class CompactSpan {

    /**
     * The lower extent of the span. (Measured from the heightfield's base.)
     */
    public int y;
    /**
     * The id of the region the span belongs to. (Or zero if not in a region.)
     */
    public int reg;
    /**
     * Packed neighbor connection data.
     * con为int类型，一共有32位，此处用4*6=24位来存储邻居连接信息。
     * 从右往左，每6位表示一个方向的连接信息，一个CompactSpan在一个方向上只会保存一个可连接邻居信息
     */
    public int con;
    /**
     * The height of the span. (Measured from #y.)
     */
    public int h;

}
