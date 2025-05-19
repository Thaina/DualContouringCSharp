/*
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
using UnityEngine;

public static class SVD
{
    public static void rotate01(in SMat3 vtav,ref float3x3 v)
    {
        if (vtav.m01 == 0)
        {
            return;
        }

        Schur2.rot01(vtav,out float c,out float s);
        Givens.rot01_post(ref v, c, s);
    }

    public static void rotate02(in SMat3 vtav,ref float3x3 v)
    {
        if (vtav.m02 == 0)
        {
            return;
        }

        Schur2.rot02(vtav,out float c,out float s);
        Givens.rot02_post(ref v, c, s);
    }

    public static void rotate12(in SMat3 vtav, ref float3x3 v)
    {
        if (vtav.m12 == 0)
        {
            return;
        }

        Schur2.rot12(vtav,out float c,out float s);
        Givens.rot12_post(ref v, c, s);
    }

    public static void getSymmetricSvd(in SMat3 a,out SMat3 vtav,out float3x3 v, float tol, int max_sweeps)
    {
        vtav = a;
        v = float3x3.identity;
        float delta = tol * vtav.fnorm();

        for (int i = 0; i < max_sweeps && vtav.off() > delta; ++i)
        {
            rotate01(vtav,ref v);
            rotate02(vtav,ref v);
            rotate12(vtav,ref v);
        }
    }

    public static float calcError(in float3x3 A, float3 x, float3 b)
    {
        var vtmp = b - math.mul(A,x);
        return math.dot(vtmp, vtmp);
    }

    public static void pseudoinverse(out float3x3 Out,float3 d, float3x3 v, float tol)
    {
        d = math.select(1 / d,0,math.abs(d) < tol | math.abs(1 / d) < tol);

        Out = v * math.scaleMul(d,float3x3.identity) * v;
    }

    public static float solveSymmetric(in SMat3 A, float3 b,out float3 x, float svd_tol, int svd_sweeps, float pinv_tol)
    {
        getSymmetricSvd(A,out var VTAV,out var V, svd_tol, svd_sweeps);
        pseudoinverse(out var pinv, new float3(VTAV.m00,VTAV.m11,VTAV.m22), V, pinv_tol);
        x = math.mul(pinv, b);
        return calcError(A, x, b);
    }

    public static void calcSymmetricGivensCoefficients(float a_pp, float a_pq, float a_qq,out float c,out float s)
    {
        if (a_pq == 0)
        {
            c = 1;
            s = 0;
            return;
        }

        float tau = (a_qq - a_pp) / (2 * a_pq);
        float stt = Mathf.Sqrt(1.0f + tau * tau);
        float tan = 1.0f / ((tau >= 0) ? (tau + stt) : (tau - stt));
        c = 1.0f / Mathf.Sqrt(1.0f + tan * tan);
        s = tan * c;
    }
}