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

using System;
using UnityEngine;
using Unity.Mathematics;

public ref struct QefSolver
{
    private QefData data;
    private SMat3 ata;
    private float3 atb, massPoint, x;
    private bool hasSolution;

    public Vector3 getMassPoint()
    {
        return massPoint;
    }

    public void add(float3 p,float3 n)
    {
        hasSolution = false;

        n = math.normalize(n);

        data.ata.m00 += n.x * n.x;
        data.ata.m01 += n.x * n.y;
        data.ata.m02 += n.x * n.z;
        data.ata.m11 += n.y * n.y;
        data.ata.m12 += n.y * n.z;
        data.ata.m22 += n.z * n.z;
        float dot = math.dot(p,n);
        data.atb += dot * n;
        data.btb += dot * dot;
        data.massPoint += p;
        ++data.numPoints;
    }

    public void add(in QefData rhs)
    {
        hasSolution = false;
        data.add(rhs);
    }

    public QefData getData()
    {
        return data;
    }

    public float getError()
    {
        if(!hasSolution)
        {
            throw new ArgumentException("Qef Solver does not have a solution!");
        }

        return getError(x);
    }

    public float getError(float3 pos)
    {
        if(!hasSolution)
        {
            ata = data.ata;
            atb = data.atb;
        }

        var atax = ata.vmul_symmetric(pos);
        return math.dot(pos,atax) - 2 * math.dot(pos,atb) + data.btb;
    }

    public float solve(out Vector3 outx,float svd_tol,int svd_sweeps,float pinv_tol)
    {
        if(data.numPoints == 0)
        {
            throw new ArgumentException("...");
        }

        massPoint = data.massPoint / data.numPoints;
        ata = data.ata;
        atb = data.atb;
        var tmpv = ata.vmul_symmetric(massPoint);
        atb = atb - tmpv;
        float result = SVD.solveSymmetric(ata,atb,out var x,svd_tol,svd_sweeps,pinv_tol);
        x += massPoint * 1;
        atb = data.atb;
        outx = x;
        hasSolution = true;
        return result;
    }

    public struct QefData
    {
        public SMat3 ata;
        public float3 atb,massPoint;
        public float btb;
        public int numPoints;

        public void add(in QefData rhs)
        {
            ata.m00 += rhs.ata.m00;
            ata.m01 += rhs.ata.m01;
            ata.m02 += rhs.ata.m02;
            ata.m11 += rhs.ata.m11;
            ata.m12 += rhs.ata.m12;
            ata.m22 += rhs.ata.m22;
            atb += rhs.atb;
            btb += rhs.btb;
            massPoint += rhs.massPoint;
            numPoints += rhs.numPoints;
        }
    }
}