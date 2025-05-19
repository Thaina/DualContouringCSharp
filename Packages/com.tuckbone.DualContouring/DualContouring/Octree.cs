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

using Code.Utils;
using JetBrains.Annotations;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Profiling;

public class Octree
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

    public static int MATERIAL_AIR = 0;
    public static int MATERIAL_SOLID = 1;

    public static float QEF_ERROR = 1e-6f;
    public static int QEF_SWEEPS = 4;

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


    public static Node SimplifyOctree(Node node,float threshold)
    {
        if(node == null)
        {
            return null;
        }

        if(!node.IsInternal)
            return node;

        var qef = new QefSolver();
        int[] signs = new int[8] { -1,-1,-1,-1,-1,-1,-1,-1 };
        int midsign = -1;
        int edgeCount = 0;
        bool isCollapsible = true;

        for(int i = 0; i < 8; i++)
        {
            var child = node.children[i] = SimplifyOctree(node.children[i],threshold);
            if(child == null)
                continue;

            if(child.IsInternal)
                isCollapsible = false;
            else
            {
                qef.add(child.drawInfo.qef);

                midsign = (child.drawInfo.corners >> (7 - i)) & 1;
                signs[i] = (child.drawInfo.corners >> i) & 1;

                edgeCount++;
            }
        }

        if(!isCollapsible)
        {
            // at least one child is an internal node, can't collapse
            return node;
        }

        qef.solve(out var position,QEF_ERROR,QEF_SWEEPS,QEF_ERROR);
        float error = qef.getError();

        // at this point the masspoint will actually be a sum, so divide to make it the average
        if(error > threshold)
        {
            // this collapse breaches the threshold
            return node;
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
            int s = signs[i] == -1 ? midsign : signs[i];
            drawInfo.corners |= s << i;
        }

        drawInfo.averageNormal = Vector3.zero;
        for(int i = 0; i < 8; i++)
        {
            var child = node.children[i];
            if(child != null && child.IsPsuedoOrLeaf)
            {
                drawInfo.averageNormal += child.drawInfo.averageNormal;
            }
        }

        drawInfo.averageNormal = math.normalize(drawInfo.averageNormal);
        drawInfo.position = position;
        drawInfo.qef = qef.getData();

        for(int i = 0; i < 8; i++)
        {
            DestroyOctree(ref node.children[i]);
        }

        node.Type = NodeType.Psuedo;
        node.drawInfo = drawInfo;

        return node;
    }

    public static void GenerateVertexIndices(Node node,List<float3x2> vertexBuffer)
    {
        if(node == null)
        {
            return;
        }

        if(node.Type != NodeType.Leaf)
        {
            for(int i = 0; i < 8; i++)
            {
                GenerateVertexIndices(node.children[i],vertexBuffer);
            }
        }

        if(!node.IsInternal)
        {
            node.drawInfo.index = vertexBuffer.Count;

            vertexBuffer.Add(new float3x2(node.drawInfo.position,node.drawInfo.averageNormal));
        }
    }

    public static void ContourProcessEdge(Node[] node,int dir,List<int> indexBuffer)
    {
        int minSize = 1000000;		// arbitrary big number
        int minIndex = 0;
        bool flip = false;
        int4 indices = -1;
        bool4 signChange = false;

        for(int i = 0; i < 4; i++)
        {
            int edge = processEdgeMask[dir][i];
            int2 c = edgevmap[edge];

            int m1 = (node[i].drawInfo.corners >> c.x) & 1;
            int m2 = (node[i].drawInfo.corners >> c.y) & 1;

            if(node[i].size < minSize)
            {
                minSize = node[i].size;
                minIndex = i;
                flip = m1 != MATERIAL_AIR;
            }

            indices[i] = node[i].drawInfo.index;

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

    public static void ContourEdgeProc(Node[] node,int dir,List<int> indexBuffer)
    {
        if(node[0] == null || node[1] == null || node[2] == null || node[3] == null)
        {
            return;
        }

        if(!node[0].IsInternal && !node[1].IsInternal && !node[2].IsInternal && !node[3].IsInternal)
        {
            ContourProcessEdge(node,dir,indexBuffer);
            return;
        }

        for(int i = 0; i < 2; i++)
        {
            var edgeNodes = new Node[4];
            int4 c = edgeProcEdgeMask[dir][i];

            int4 nodeIndexes = -1;

            for(int j = 0; j < 4; j++)
            {
                if(node[j].IsPsuedoOrLeaf)
                    edgeNodes[j] = node[j];
                else edgeNodes[j] = node[j].children[c[j]];

                if(!node[j].IsPsuedoOrLeaf)
                    nodeIndexes[j] = c[j];
            }

            ContourEdgeProc(edgeNodes,dir,indexBuffer);
        }
    }

    public static void ContourFaceProc(Node[] node,int dir,List<int> indexBuffer)
    {
        if(node[0] == null || node[1] == null)
            return;

        if(!node[0].IsInternal && !node[1].IsInternal)
            return;

        for(int i = 0; i < 4; i++)
        {
            var faceNodes = new Node[2];
            int2 c = faceProcFaceMask[dir][i];

            for(int j = 0; j < 2; j++)
            {
                if(!node[j].IsInternal)
                    faceNodes[j] = node[j];
                else faceNodes[j] = node[j].children[c[j]];
            }

            ContourFaceProc(faceNodes,dir,indexBuffer);
        }

        int4x2 orders = new int4x2 (new int4(0, 0, 1, 1),new int4(0, 1, 0, 1));
        ref var masks = ref faceProcEdgeMask[dir];
        for(int i = 0; i < 4; i++)
        {
            ref var control = ref masks.control[i];
            ref var mask = ref masks.mask[i];

            var edgeNodes = new Node[4];
            int4 order = orders[control[0]];
            for(int j = 0; j < 4; j++)
            {
                var n = node[order[j]];
                if(n.IsPsuedoOrLeaf)
                    edgeNodes[j] = n;
                else edgeNodes[j] = n.children[mask[j]];
            }

            ContourEdgeProc(edgeNodes,control[1],indexBuffer);
        }
    }

    public static void ContourCellProc(Node node,List<int> indexBuffer)
    {
        if(node == null || !node.IsInternal)
        {
            return;
        }

        for(int i = 0; i < 8; i++)
        {
            ContourCellProc(node.children[i],indexBuffer);
        }

        for(int i = 0; i < 12; i++)
        {
            var faceNodes = new Node[2];
            var c = cellProcFaceMask[i];

            faceNodes[0] = node.children[c[0]];
            faceNodes[1] = node.children[c[1]];

            ContourFaceProc(faceNodes,i / 4,indexBuffer);
        }

        for(int i = 0; i < 6; i++)
        {
            var edgeNodes = new Node[4];

            int4 c = cellProcEdgeMask[i];
            for(int j = 0; j < 4; j++)
            {
                edgeNodes[j] = node.children[c[j]];
            }

            ContourEdgeProc(edgeNodes,i / 2,indexBuffer);
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

    public struct DrawInfo
    {
        public int index;
        public int corners;
        public float3 position;
        public float3 averageNormal;
        public QefSolver.QefData qef;
    }

    public static Node ConstructLeaf(Node leaf)
    {
        if(leaf == null || leaf.size != 1)
        {
            return null;
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
            leaf = null;
            return null;
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

        drawInfo.averageNormal = Vector3.Normalize(averageNormal / (float)edgeCount);
        drawInfo.corners = corners;

        leaf.Type = NodeType.Leaf;
        leaf.drawInfo = drawInfo;

        return leaf;
    }

    public static async Awaitable<Node> ConstructOctreeNodes(Node node)
    {
        await Awaitable.BackgroundThreadAsync();
        if(node == null)
            return null;

        if(node.size == 1)
            return ConstructLeaf(node);

        int childSize = node.size / 2;
        bool hasChildren = false;

        for(int i = 0; i < 8; i++)
        {
            var child = new Node();
            child.size = childSize;
            child.min = node.min + (CHILD_MIN_OFFSETS[i] * childSize);
            child.Type = NodeType.Internal;

            node.children[i] = await ConstructOctreeNodes(child);
            hasChildren |= node.children[i] != null;
        }

        if(!hasChildren)
            node = null;

        return node;
    }

    public static async Awaitable<Node> BuildOctree(float3 min,int size,float threshold)
    {
        Debug.Log(string.Format("Building Octree at {0}, with size of {1} and threshold of {2}",min,size,threshold));

        var root = new Node(NodeType.Internal);
        root.min = min;
        root.size = size;

        root = await ConstructOctreeNodes(root);
        root = SimplifyOctree(root,threshold);
        return root;
    }

    public static void GenerateMeshFromOctree(Node node,ref List<float3x2> vertexBuffer,ref List<int> indexBuffer)
    {
        if(node == null)
            return;

        vertexBuffer ??= new List<float3x2>();
        indexBuffer ??= new List<int>();

        GenerateVertexIndices(node,vertexBuffer);
        ContourCellProc(node,indexBuffer);
    }

    public static void DrawOctree(Node rootNode,int colorIndex)
    {
        if(rootNode != null && rootNode.children.Length > 0)
        {
            for(int i = 0; i < rootNode.children.Length; i++)
            {
                DrawOctree(rootNode.children[i],colorIndex + 1);
            }

            DrawOctreeNode(rootNode,drawColors[colorIndex]);
        }
    }

    public static void DrawOctreeNode(Node node,Color color)
    {
        DrawCubeInLines.DrawCube(node.min,node.size,color);
    }

    public static void DestroyOctree(ref Node node)
    {
        if(node == null)
        {
            return;
        }

        for(int i = 0; i < 8; i++)
        {
            DestroyOctree(ref node.children[i]);
        }

        node = null;
    }

    public enum NodeType
    {
        None,
        Internal,
        Psuedo,
        Leaf
    }

    public class Node
    {
        public NodeType Type;
        public bool IsInternal => Type == NodeType.Internal;
        public bool IsPsuedoOrLeaf => Type == NodeType.Psuedo || Type == NodeType.Leaf;

        public float3 min;
        public int size;
        public Node[] children;
        public DrawInfo drawInfo;

        public Node(NodeType _type = NodeType.None)
        {
            Type = _type;
            min = 0;
            size = 0;
            drawInfo = new();

            children = new Node[8];
            for(int i = 0; i < 8; i++)
            {
                children[i] = null;
            }
        }
    }
}