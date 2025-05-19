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

    public const int MATERIAL_AIR = 0;
    public const int MATERIAL_SOLID = 1;

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

    // data from the original DC impl, drives the contouring process

    public static readonly int2[] edgevmap = new int2[] {
        new int2(2,4),new int2(1,5),new int2(2,6),new int2(3,7),	// x-axis
        new int2(0,2),new int2(1,3),new int2(4,6),new int2(5,7),	// y-axis
        new int2(0,1),new int2(2,3),new int2(4,5),new int2(6,7)		// z-axis
    };

    public static readonly int4[] faceMap = new int4[6] {
        new int4(4, 8, 5, 9),
        new int4(6, 10, 7, 11),
        new int4(0, 8, 1, 10),
        new int4(2, 9, 3, 11),
        new int4(0, 4, 2, 6),
        new int4(1, 5, 3, 7)
    };

    public static readonly int2[] cellProcFaceMask = new int2[] {
        new int2(0,4),
        new int2(1,5),
        new int2(2,6),
        new int2(3,7),
        new int2(0,2),
        new int2(4,6),
        new int2(1,3),
        new int2(5,7),
        new int2(0,1),
        new int2(2,3),
        new int2(4,5),
        new int2(6,7)
    };

    public static readonly int4[] cellProcEdgeMask = new int4[] {
        new (0,1,2,3),
        new (4,5,6,7),
        new (0,4,1,5),
        new (2,6,3,7),
        new (0,2,4,6),
        new (1,3,5,7)
    };

    public static readonly int2x4[] faceProcFaceMask = new int2x4[] {
        new ( new (4,0), new (5,1), new (6,2), new (7,3) ),
        new ( new (2,0), new (6,4), new (3,1), new (7,5) ),
        new ( new (1,0), new (3,2), new (5,4), new (7,6) )
    };

    public static readonly (int2x4 control, int4x4 mask)[] faceProcEdgeMask = new (int2x4, int4x4)[] {
        (new (new (1,1),new (1,1),new (0,2),new (0,2)),new (new (4,0,5,1),new (6,2,7,3),new (4,6,0,2),new (5,7,1,3))),
        (new (new (0,0),new (0,0),new (1,2),new (1,2)),new (new (2,3,0,1),new (6,7,4,5),new (2,0,6,4),new (3,1,7,5))),
        (new (new (1,0),new (1,0),new (0,1),new (0,1)),new (new (1,0,3,2),new (5,4,7,6),new (1,5,0,4),new (3,7,2,6)))
    };

    public static readonly int4x2[] edgeProcEdgeMask = new int4x2[] {
        new int4x2(new int4(3,2,1,0),new int4(7,6,5,4)),
        new int4x2(new int4(5,1,4,0),new int4(7,3,6,2)),
        new int4x2(new int4(6,4,2,0),new int4(7,5,3,1))
    };

    public static readonly int4x3 processEdgeMask = new int4x3(
        new int4(3,2,1,0),new int4(7,5,6,4),new int4(11,10,9,8)
    );
    #endregion

    [BurstCompile]
    public struct OctreeJob : IJobFor, INativeDisposable
    {
        public float3 min;
        public int size;
        public float threshold;

        public NativeList<Node> nodes;
        public NativeList<float3x2> vertexBuffer;
        public NativeList<int> indexBuffer;

        bool SimplifyOctree(ref Node node,float threshold)
        {
            if(node.Type == NodeType.None)
                return false;

            if(!node.IsInternal)
                return true;

            var qef = new QefSolver();
            int4x2 signs = -1;
            int midsign = -1;
            int edgeCount = 0;
            bool isCollapsible = true;

            for(int i = 0; i < 8; i++)
            {
                ref var child = ref node.children[nodes,i];
                if(!SimplifyOctree(ref child,threshold))
                    continue;

                if(child.IsInternal)
                    isCollapsible = false;
                else
                {
                    qef.add(child.drawInfo.qef);

                    midsign = (child.drawInfo.corners >> (7 - i)) & 1;
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
            var drawInfo = new DrawInfo();
            drawInfo.corners = 0;
            drawInfo.index = -1;

            for(int i = 0; i < 8; i++)
            {
                int s = signs[i / 4][i % 4];
                if(s == -1)
                    s = midsign;

                drawInfo.corners |= s << i;
            }

            drawInfo.averageNormal = Vector3.zero;
            for(int i = 0; i < 8; i++)
            {
                ref var child = ref node.children[nodes,i];
                if(child.Type == NodeType.None && child.IsPsuedoOrLeaf)
                {
                    drawInfo.averageNormal += child.drawInfo.averageNormal;
                }
            }

            drawInfo.averageNormal = math.normalize(drawInfo.averageNormal);
            drawInfo.position = position;
            drawInfo.qef = qef.getData();

            for(int i = 0; i < 8; i++)
            {
                DestroyOctree(ref node.children[nodes,i]);
            }

            node.Type = NodeType.Psuedo;
            node.drawInfo = drawInfo;

            return true;
        }

        public void GenerateVertexIndices(ref Node node)
        {
            if(node.Type == NodeType.None)
            {
                return;
            }

            if(node.Type != NodeType.Leaf)
            {
                for(int i = 0; i < 8; i++)
                {
                    GenerateVertexIndices(ref node.children[nodes,i]);
                }
            }

            if (!node.IsInternal)
            {
                node.drawInfo.index = vertexBuffer.Length;
                vertexBuffer.Add(new float3x2(node.drawInfo.position, node.drawInfo.averageNormal));
            }
        }

        public void ContourProcessEdge(in Chunk4<Node> node,int dir)
        {
            int minSize = 1000000;      // arbitrary big number
            int minIndex = 0;
            bool flip = false;
            int4 indices = -1;
            bool4 signChange = false;

            for(int i = 0; i < 4; i++)
            {
                int edge = processEdgeMask[dir][i];
                int2 c = edgevmap[edge];

                int m1 = (node[nodes,i].drawInfo.corners >> c.x) & 1;
                int m2 = (node[nodes,i].drawInfo.corners >> c.y) & 1;

                if(node[nodes,i].size < minSize)
                {
                    minSize = node[nodes,i].size;
                    minIndex = i;
                    flip = m1 != MATERIAL_AIR;
                }

                indices[i] = node[nodes,i].drawInfo.index;

                signChange[i] =
                    (m1 == MATERIAL_AIR && m2 != MATERIAL_AIR) ||
                    (m1 != MATERIAL_AIR && m2 == MATERIAL_AIR);
            }

            if(signChange[minIndex])
            {
                if(!flip)
                {
                    indexBuffer.Add(indices[0]);
                    indexBuffer.Add(indices[1]);
                    indexBuffer.Add(indices[3]);

                    indexBuffer.Add(indices[0]);
                    indexBuffer.Add(indices[3]);
                    indexBuffer.Add(indices[2]);
                }
                else
                {
                    indexBuffer.Add(indices[0]);
                    indexBuffer.Add(indices[3]);
                    indexBuffer.Add(indices[1]);

                    indexBuffer.Add(indices[0]);
                    indexBuffer.Add(indices[2]);
                    indexBuffer.Add(indices[3]);
                }
            }
        }

        public void ContourEdgeProc(in Chunk4<Node> node,int dir)
        {
            if(node[nodes,0].Type == NodeType.None || node[nodes,1].Type == NodeType.None || node[nodes,2].Type == NodeType.None || node[nodes,3].Type == NodeType.None)
            {
                return;
            }

            if(!node[nodes,0].IsInternal && !node[nodes,1].IsInternal && !node[nodes,2].IsInternal && !node[nodes,3].IsInternal)
            {
                ContourProcessEdge(node,dir);
                return;
            }

            for(int i = 0; i < 2; i++)
            {
                int4 c = edgeProcEdgeMask[dir][i];

                var edgeNodes = node;

                for(int j = 0; j < 4; j++)
                {
                    if(!node[nodes,j].IsPsuedoOrLeaf)
                        edgeNodes.indexes[j] = node[nodes,j].childIndex + c[j];
                }

                ContourEdgeProc(edgeNodes,dir);
            }
        }

        public void ContourFaceProc(in Chunk2<Node> node,int dir)
        {
            if(node[nodes,0].Type == NodeType.None || node[nodes,1].Type == NodeType.None)
                return;

            if(!node[nodes,0].IsInternal && !node[nodes,1].IsInternal)
                return;

            for(int i = 0; i < 4; i++)
            {
                var faceNodes = node;
                int2 c = faceProcFaceMask[dir][i];

                for(int j = 0; j < 2; j++)
                {
                    if(node[nodes,j].IsInternal)
                        faceNodes.indexes[j] = node[nodes,j].childIndex + c[j];
                }

                ContourFaceProc(faceNodes,dir);
            }

            int4x2 orders = new int4x2(new int4(0,0,1,1),new int4(0,1,0,1));
            ref var masks = ref faceProcEdgeMask[dir];
            for(int i = 0; i < 4; i++)
            {
                ref var control = ref masks.control[i];
                ref var mask = ref masks.mask[i];

                Chunk4<Node> edgeNodes = default;
                int4 order = orders[control[0]];
                for(int j = 0; j < 4; j++)
                {
                    var n = node[nodes,order[j]];
                    if(n.IsPsuedoOrLeaf)
                        edgeNodes.indexes[j] = node.indexes[order[j]];
                    else edgeNodes.indexes[j] = n.childIndex + mask[j];
                }

                ContourEdgeProc(edgeNodes,control[1]);
            }
        }

        void ContourCellProc(ref Node node)
        {
            if(node.Type == NodeType.None || !node.IsInternal)
            {
                return;
            }

            for(int i = 0; i < 8; i++)
            {
                ContourCellProc(ref node.children[nodes,i]);
            }

            for(int i = 0; i < 12; i++)
            {
                var c = cellProcFaceMask[i];
                var faceNodes = new Chunk2<Node>() { indexes = node.childIndex + c };
                ContourFaceProc(faceNodes,i / 4);
            }

            for(int i = 0; i < 6; i++)
            {
                int4 c = cellProcEdgeMask[i];
                var edgeNodes = new Chunk4<Node>() { indexes = node.childIndex + c };
                ContourEdgeProc(edgeNodes,i / 2);
            }
        }

        public static float3 ApproximateZeroCrossingPosition(float3 p0,float3 p1)
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
                float density = Mathf.Abs(glm.Density_Func(p));
                if(density < minValue)
                {
                    minValue = density;
                    t = currentT;
                }

                currentT += increment;
            }

            return p0 + ((p1 - p0) * t);
        }

        public static Vector3 CalculateSurfaceNormal(Vector3 p)
        {
            float H = 0.001f;
            float dx = glm.Density_Func(p + new Vector3(H,0,0)) - glm.Density_Func(p - new Vector3(H,0,0));
            float dy = glm.Density_Func(p + new Vector3(0,H,0)) - glm.Density_Func(p - new Vector3(0,H,0));
            float dz = glm.Density_Func(p + new Vector3(0,0,H)) - glm.Density_Func(p - new Vector3(0,0,H));

            return new Vector3(dx,dy,dz).normalized;
        }

        public static bool ConstructLeaf(ref Node leaf)
        {
            if(leaf.Type == NodeType.None || leaf.size != 1)
            {
                return false;
            }

            int corners = 0;
            for(int i = 0; i < 8; i++)
            {
                float3 cornerPos = leaf.min + CHILD_MIN_OFFSETS[i];
                float density = glm.Density_Func(cornerPos);
                int material = density < 0 ? MATERIAL_SOLID : MATERIAL_AIR;
                corners |= (material << i);
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
            Vector3 averageNormal = Vector3.zero;
            var qef = new QefSolver();

            for(int i = 0; i < 12 && edgeCount < MAX_CROSSINGS; i++)
            {
                int2 c = edgevmap[i];

                int m1 = (corners >> c.x) & 1;
                int m2 = (corners >> c.y) & 1;

                if((m1 == MATERIAL_AIR && m2 == MATERIAL_AIR) || (m1 == MATERIAL_SOLID && m2 == MATERIAL_SOLID))
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

            qef.solve(out var qefPosition,QEF_ERROR,QEF_SWEEPS,QEF_ERROR);

            var drawInfo = new DrawInfo();
            drawInfo.corners = 0;
            drawInfo.index = -1;
            drawInfo.position = qefPosition;
            drawInfo.qef = qef.getData();

            if(math.any(drawInfo.position < leaf.min) || math.any(drawInfo.position > leaf.min + leaf.size))
            {
                drawInfo.position = qef.getMassPoint();
            }

            drawInfo.averageNormal = Vector3.Normalize(averageNormal / edgeCount);
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
                node.children[nodes,i] = new Node(nodes,NodeType.Internal);
                ref var child = ref node.children[nodes,i];
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
            root = new Node(nodes, NodeType.Internal);
            root.min = min;
            root.size = size;

            // Build and simplify octree
            if (ConstructOctreeNodes(ref root))
                SimplifyOctree(ref root, threshold);

            if (root.Type == NodeType.None)
                return;

            // Generate mesh data
            GenerateVertexIndices(ref root);
            ContourCellProc(ref root);
        }

        public void Dispose()
        {
            nodes.Dispose();
            vertexBuffer.Dispose();
            indexBuffer.Dispose();
        }

        public JobHandle Dispose(JobHandle inputDeps)
        {
            return JobHandle.CombineDependencies(
                nodes.Dispose(inputDeps),
                vertexBuffer.Dispose(inputDeps),
                indexBuffer.Dispose(inputDeps)
            );
        }

        public void DrawOctree(ref Node node,int colorIndex)
        {
            if(node.Type != NodeType.None)
            {
                for(int i = 0; i < 8; i++)
                {
                    DrawOctree(ref node.children[nodes,i],colorIndex + 1);
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
                DestroyOctree(ref node.children[nodes,i]);
            }

            node = default;
        }
    }

    public struct Chunk2<T> where T : unmanaged
    {
        public int2 indexes;
        public ref T this[NativeList<T> list,int i] => ref list.ElementAt(indexes[i]);
    }

    public struct Chunk4<T> where T : unmanaged
    {
        public int4 indexes;
        public ref T this[NativeList<T> list,int i] => ref list.ElementAt(indexes[i]);
    }

    public struct Chunk8<T> where T : unmanaged
    {
        public int4x2 indexes;
        public ref T this[NativeList<T> list,int i] => ref list.ElementAt(indexes[i / 4][i % 4]);
    }

    public struct DrawInfo
    {
        public int index;
        public int corners;
        public float3 position;
        public float3 averageNormal;
        public QefSolver.QefData qef;
    }

    public enum NodeType
    {
        None,
        Internal,
        Psuedo,
        Leaf
    }

    public struct Node
    {
        public NodeType Type;
        public bool IsInternal => Type == NodeType.Internal;
        public bool IsPsuedoOrLeaf => Type == NodeType.Psuedo || Type == NodeType.Leaf;

        public float3 min;
        public int size;
        public int childIndex;
        public DrawInfo drawInfo;
        public Chunk8<Node> children;

        public Node(in NativeList<Node> nodes,NodeType _type = NodeType.None)
        {
            Type = _type;
            min = 0;
            size = 0;
            drawInfo = default;

            childIndex = nodes.Length;
            nodes.AddReplicate(default,8);
            children = new Chunk8<Node> { indexes = childIndex + new int4x2(new(0,1,2,3),new(4,5,6,7)) };
        }
    }
}