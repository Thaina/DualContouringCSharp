﻿/*
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <http://unlicense.org/>
 */

using Unity.Mathematics;

public static class Givens
{
    public static void rot01_post(ref float3x3 m, float c, float s)
    {
        var r = new float2x2(new float2(c,-s),new float2(s,c));
        m = new float3x3(new float3(math.mul(m.c0.xy,r),m[0][2])
                        ,new float3(math.mul(m.c1.xy,r),m[1][2])
                        ,new float3(math.mul(m.c2.xy,r),m[2][2]));
    }

    public static void rot02_post(ref float3x3 m, float c, float s)
    {
        var r = new float2x2(new float2(c,-s),new float2(s,c));
        m = new float3x3(new float3(m[0][1],math.mul(m.c0.xz,r)).yxz
                        ,new float3(m[1][1],math.mul(m.c1.xz,r)).yxz
                        ,new float3(m[2][1],math.mul(m.c2.xz,r)).yxz);
    }

    public static void rot12_post(ref float3x3 m, float c, float s)
    {
        var r = new float2x2(new float2(c,-s),new float2(s,c));
        m = new float3x3(new float3(m[0][0],math.mul(m.c0.yz,r))
                        ,new float3(m[1][0],math.mul(m.c1.yz,r))
                        ,new float3(m[2][0],math.mul(m.c2.yz,r)));
    }
}

