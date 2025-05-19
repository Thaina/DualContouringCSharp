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

    public void add(float3 p,float3 n) => add(new QefData(p,n));
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

    public float solve(out float3 outx,float svd_tol,int svd_sweeps,float pinv_tol)
    {
        if(data.numPoints == 0)
        {
            throw new ArgumentException("...");
        }

        massPoint = data.massPoint / data.numPoints;
        ata = data.ata;
        atb = data.atb;
        float result = SVD.solveSymmetric(ata,atb - ata.vmul_symmetric(massPoint),out x,svd_tol,svd_sweeps,pinv_tol);
        x += massPoint;
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

        public QefData(float3 p,float3 n)
        {
            n = math.normalize(n);
            float dot = math.dot(p,n);
            ata = n * new SMat3(n.x,n.y,n.z,n.y,n.z,n.z);
            atb = dot * n;
            btb = dot * dot;
            massPoint = p;
            numPoints = 1;
        }

        public void add(in QefData rhs)
        {
            ata += rhs.ata;
            atb += rhs.atb;
            btb += rhs.btb;
            massPoint += rhs.massPoint;
            numPoints += rhs.numPoints;
        }
    }
}