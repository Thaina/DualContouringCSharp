/*

Implementations of Octree member functions.

Copyright (C) 2011  Tao Ju

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public License
(LGPL) as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

using UnityEngine;

using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

using Code.Utils;

public static partial class Octree
{
    private static Color[] drawColors = new Color[]
    {
        Color.white,
        Color.yellow,
        Color.grey,
        Color.green,
        Color.blue,
        Color.black,
        Color.red,
        Color.cyan,
    };

    public const float QEF_ERROR = 1e-6f;
    public const int QEF_SWEEPS = 4;

    #region Readonly Variables
    public static readonly float3[] CHILD_MIN_OFFSETS = {
        // needs to match the vertMap from Dual Contouring impl
        new float3( 0, 0, 0 ),
        new float3( 0, 0, 1 ),
        new float3( 0, 1, 0 ),
        new float3( 0, 1, 1 ),
        new float3( 1, 0, 0 ),
        new float3( 1, 0, 1 ),
        new float3( 1, 1, 0 ),
        new float3( 1, 1, 1 ),
    };
    #endregion

    static int2 rshift(in int2 l,in int2 r) => new(l.x >> r.x,l.y >> r.y);

    public interface IGenerator
    {
        float Density_Func(in float3 position);
    }

	[BurstCompile]
    public struct OctreeJob<T> : IJobFor where T : struct,IGenerator
    {
        public float3 min;
        public int size;
        public float threshold;

        public T glm;

        public NativeList<Node> nodes;

        bool SimplifyOctree(ref Node node,float threshold)
        {
            if(node.Type == NodeType.None)
                return false;

            if(node.Type != NodeType.Internal)
                return true;

            var qef = new QefSolver();
            int4x2 signs = -1;
            int midsign = -1;
            int edgeCount = 0;
            bool isCollapsible = true;

            for(int i = 0; i < 8; i++)
            {
                ref var child = ref nodes.ElementAt(node.childIndex + i);
                if(!SimplifyOctree(ref child,threshold))
                    continue;

                if(child.Type == NodeType.Internal)
                    isCollapsible = false;
                else
                {
                    qef.add(child.drawInfo.qef);

                    midsign += (child.drawInfo.corners >> (7 - i)) & 1;
                    signs[i / 4][i % 4] = (child.drawInfo.corners >> i) & 1;

                    edgeCount++;
                }
            }

            if(!isCollapsible)
            {
                // at least one child is an internal node, can't collapse
                return true;
            }

            qef.solve(out var position,QEF_ERROR,QEF_SWEEPS,QEF_ERROR);
            float error = qef.getError();

            // at this point the masspoint will actually be a sum, so divide to make it the average
            if(error > threshold)
            {
                // this collapse breaches the threshold
                return true;
            }

            if(math.any(position < node.min) || math.any(position > (node.min + node.size)))
            {
                position = qef.getMassPoint();
            }

            // change the node from an internal node to a 'psuedo leaf' node
            byte corners = 0;
            for(int i = 0; i < 8; i++)
            {
                int s = signs[i / 4][i % 4];
                if(s == -1)
                    s = edgeCount > 0 ? (int)math.round(midsign / (float)edgeCount) : 0;

                corners |= (byte)(s << i);
            }

            var drawInfo = new DrawInfo();
            drawInfo.corners = corners;
            drawInfo.averageNormal = float3.zero;
            for(int i = 0; i < 8; i++)
            {
                ref var child = ref nodes.ElementAt(node.childIndex + i);
                if(child.Type == NodeType.Internal)
                {
                    drawInfo.averageNormal += child.drawInfo.averageNormal;
                }
            }

            drawInfo.averageNormal = math.normalize(drawInfo.averageNormal);
            drawInfo.position = position;
            drawInfo.qef = qef.getData();

            for(int i = 0; i < 8; i++)
            {
                DestroyOctree(ref nodes.ElementAt(node.childIndex + i));
            }

            node.Type = NodeType.Psuedo;
            node.drawInfo = drawInfo;

            return true;
        }

        public float3 ApproximateZeroCrossingPosition(float3 p0,float3 p1)
        {
            // approximate the zero crossing by finding the min value along the edge
            float minValue = 100000f;
            float t = 0f;
            float currentT = 0f;
            const int steps = 8;
            const float increment = 1f / (float)steps;
            while(currentT <= 1.0f)
            {
                float3 p = p0 + ((p1 - p0) * currentT);
                float density = math.abs(glm.Density_Func(p));
                if(density < minValue)
                {
                    minValue = density;
                    t = currentT;
                }

                currentT += increment;
            }

            return p0 + ((p1 - p0) * t);
        }

        public float3 CalculateSurfaceNormal(in float3 p)
        {
            float H = 0.001f;
            float dx = glm.Density_Func(p + new float3(H,0,0)) - glm.Density_Func(p - new float3(H,0,0));
            float dy = glm.Density_Func(p + new float3(0,H,0)) - glm.Density_Func(p - new float3(0,H,0));
            float dz = glm.Density_Func(p + new float3(0,0,H)) - glm.Density_Func(p - new float3(0,0,H));

            return math.normalize(new float3(dx,dy,dz));
        }

        public bool ConstructLeaf(ref Node leaf)
        {
            if(leaf.Type == NodeType.None || leaf.size != 1)
            {
                return false;
            }

            byte corners = 0;
            for(int i = 0; i < 8; i++)
            {
                float density = glm.Density_Func(leaf.min + CHILD_MIN_OFFSETS[i]);
                if(density < 0)
                    corners |= (byte)(1 << i);
            }

            if(corners == 0 || corners == 255)
            {
                // voxel is full inside or outside the volume
                //delete leaf
                //setting as null isn't required by the GC in C#... but its in the original, so why not!
                leaf = default;
                return false;
            }

            // otherwise the voxel contains the surface, so find the edge intersections
            const int MAX_CROSSINGS = 6;
            int edgeCount = 0;
            float3 averageNormal = float3.zero;
            var qef = new QefSolver();

            for(int i = 0; i < 12 && edgeCount < MAX_CROSSINGS; i++)
            {
                var c = edgevmap[i];
                int2 m = rshift(corners,c) & 1;

                if(m[0] == m[1])
                {
                    // no zero crossing on this edge
                    continue;
                }

                var p1 = leaf.min + CHILD_MIN_OFFSETS[c.x];
                var p2 = leaf.min + CHILD_MIN_OFFSETS[c.y];
                var p = ApproximateZeroCrossingPosition(p1,p2);
                var n = CalculateSurfaceNormal(p);
                qef.add(p,n);

                averageNormal += n;

                edgeCount++;
            }

            var drawInfo = new DrawInfo();

            qef.solve(out drawInfo.position,QEF_ERROR,QEF_SWEEPS,QEF_ERROR);

            drawInfo.qef = qef.getData();

            if(math.any(drawInfo.position < leaf.min) || math.any(drawInfo.position > leaf.min + leaf.size))
            {
                drawInfo.position = qef.getMassPoint();
            }

            drawInfo.averageNormal = math.normalize(averageNormal / edgeCount);
            drawInfo.corners = corners;

            leaf.Type = NodeType.Leaf;
            leaf.drawInfo = drawInfo;

            return true;
        }

        public bool ConstructOctreeNodes(ref Node node)
        {
            if(node.Type == NodeType.None)
                return false;

            if(node.size == 1)
                return ConstructLeaf(ref node);

            int childSize = node.size / 2;
            bool hasChildren = false;

            for(int i = 0; i < 8; i++)
            {
                ref var child = ref nodes.ElementAt(node.childIndex + i);
                child = new Node(nodes,NodeType.Internal);
                child.size = childSize;
                child.min = node.min + (CHILD_MIN_OFFSETS[i] * childSize);

                if(ConstructOctreeNodes(ref child))
                    hasChildren |= true;
            }

            if(!hasChildren)
                node = default;

            return hasChildren;
        }

        public void Execute(int index)
        {
            // Initialize root node
            nodes.Add(default);
            ref var root = ref nodes.ElementAt(0);
            root = new Node(nodes,NodeType.Internal);
            root.min = min;
            root.size = size;

            // Build and simplify octree
            if(ConstructOctreeNodes(ref root))
                SimplifyOctree(ref root,threshold);
        }

        public void DrawOctree(ref Node node,int colorIndex)
        {
            if(node.Type != NodeType.None)
            {
                for(int i = 0; i < 8; i++)
                {
                    DrawOctree(ref nodes.ElementAt(node.childIndex + i),colorIndex + 1);
                }

                DrawCubeInLines.DrawCube(node.min,node.size,drawColors[colorIndex]);
            }
        }

        public void DestroyOctree(ref Node node)
        {
            if(node.Type == NodeType.None)
            {
                return;
            }

            for(int i = 0; i < 8; i++)
            {
                DestroyOctree(ref nodes.ElementAt(node.childIndex + i));
            }

            node = default;
        }
    }

    public static int2 Types(in this NativeList<Node> nodes,in int2 indexes) => new int2((int)nodes.ElementAt(indexes[0]).Type,(int)nodes.ElementAt(indexes[1]).Type);
    public static int4 Types(in this NativeList<Node> nodes,in int4 indexes) => new int4((int)nodes.ElementAt(indexes[0]).Type,(int)nodes.ElementAt(indexes[1]).Type,(int)nodes.ElementAt(indexes[2]).Type,(int)nodes.ElementAt(indexes[3]).Type);
    public static int4x2 Types(in this NativeList<Node> nodes,in int4x2 indexes) => new int4x2(nodes.Types(indexes[0]),nodes.Types(indexes[1]));

    public static int2 ChildIndexes(in this NativeList<Node> nodes,in int2 indexes) => new int2(nodes.ElementAt(indexes[0]).childIndex,nodes.ElementAt(indexes[1]).childIndex);
    public static int4 ChildIndexes(in this NativeList<Node> nodes,in int4 indexes) => new int4((int)nodes.ElementAt(indexes[0]).childIndex,nodes.ElementAt(indexes[1]).childIndex,nodes.ElementAt(indexes[2]).childIndex,nodes.ElementAt(indexes[3]).childIndex);
    public static int4x2 ChildIndexes(in this NativeList<Node> nodes,in int4x2 indexes) => new int4x2(nodes.ChildIndexes(indexes[0]),nodes.ChildIndexes(indexes[1]));

    public struct DrawInfo
    {
        public byte corners;
        public float3 position;
        public float3 averageNormal;
        public QefSolver.QefData qef;
    }

    public enum NodeType : int
    {
        None = 0,
        Internal = 1,
        Psuedo = 0b11,
        Leaf = 0b10
    }

    public struct Node
    {
        public NodeType Type;

        public float3 min;
        public int size;
        public int childIndex;
        public DrawInfo drawInfo;

        public Node(in NativeList<Node> nodes,NodeType _type = NodeType.None)
        {
            Type = _type;
            min = 0;
            size = 0;
            drawInfo = default;

            childIndex = nodes.Length;
            nodes.AddReplicate(default,8);
        }
    }
}