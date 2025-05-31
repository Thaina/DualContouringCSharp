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

using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;

public static partial class Octree
{
    // data from the original DC impl, drives the contouring process

    public static readonly int2[] edgevmap = new int2[] {
        new int2(2,4),new int2(1,5),new int2(2,6),new int2(3,7),	// x-axis
        new int2(0,2),new int2(1,3),new int2(4,6),new int2(5,7),	// y-axis
        new int2(0,1),new int2(2,3),new int2(4,5),new int2(6,7)		// z-axis
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

    public static readonly (int4 dir,int4x4 mask)[] faceProcEdgeMask = new (int4,int4x4)[] {
        (new (1,1,2,2),new (new (4,0,5,1),new (6,2,7,3),new (4,6,0,2),new (5,7,1,3))),
        (new (0,0,2,2),new (new (2,3,0,1),new (6,7,4,5),new (2,0,6,4),new (3,1,7,5))),
        (new (0,0,1,1),new (new (1,0,3,2),new (5,4,7,6),new (1,5,0,4),new (3,7,2,6)))
    };

    public static readonly int4x2[] edgeProcEdgeMask = new int4x2[] {
        new int4x2(new int4(3,2,1,0),new int4(7,6,5,4)),
        new int4x2(new int4(5,1,4,0),new int4(7,3,6,2)),
        new int4x2(new int4(6,4,2,0),new int4(7,5,3,1))
    };

    [BurstCompile]
	public struct GenMeshJob : IJobFor, INativeDisposable
    {
        public NativeList<Node> nodes;
        public NativeList<int> nodeVertexIndex;
        public NativeList<float3x2> vertexBuffer;
        [WriteOnly]
        public NativeList<int3> indexBuffer;
        public void Execute(int index)
        {
            ref var root = ref nodes.ElementAt(0);
            if(root.Type == NodeType.None)
                return;

            nodeVertexIndex.Resize(nodes.Length,NativeArrayOptions.UninitializedMemory);

            // Generate mesh data
            GenerateVertexIndices(0);
            ContourCellProc(root);
        }

        public void GenerateVertexIndices(int nodeIndex)
        {
            ref var node = ref nodes.ElementAt(nodeIndex);
            if(node.Type == NodeType.None)
                return;

            if(node.Type != NodeType.Leaf)
            {
                for(int i = 0; i < 8; i++)
                {
                    GenerateVertexIndices(node.childIndex + i);
                }
            }

            if(node.Type == NodeType.Internal)
                return;

            nodeVertexIndex[nodeIndex] = vertexBuffer.Length;
            vertexBuffer.Add(new float3x2(node.drawInfo.position,node.drawInfo.averageNormal));
        }

        void ContourCellProc(in Node node)
        {
            if(node.Type == NodeType.None || node.Type != NodeType.Internal)
            {
                return;
            }

            for(int i = 0; i < 8; i++)
            {
                ContourCellProc(nodes.ElementAt(node.childIndex + i));
            }

            for(int i = 0; i < 12; i++)
            {
                ContourFaceProc(cellProcFaceMask[i] + node.childIndex,i / 4);
            }

            for(int i = 0; i < 6; i++)
            {
                ContourEdgeProc(cellProcEdgeMask[i] + node.childIndex,i / 2);
            }
        }

        public void ContourProcessEdge(in int4 node,int dir)
        {
            int minSize = int.MaxValue;
            int minIndex = 0;
            bool flip = false;
            int4 indices = -1;
            bool4 signChange = false;

            for(int i = 0; i < 4; i++)
            {
                ref var child = ref nodes.ElementAt(node[i]);

                int2 m = rshift(child.drawInfo.corners,edgevmap[(4 * dir) + 3 - i]) & 1;

                if(minSize > child.size)
                {
                    minSize = child.size;
                    minIndex = i;
                    flip = m.x != 0;
                }

                indices[i] = nodeVertexIndex[node[i]];

                signChange[i] = m.x != m.y;
            }

            if(signChange[minIndex])
            {
                indexBuffer.Add(flip ? indices.xwy : indices.xyw);
                indexBuffer.Add(flip ? indices.xzw : indices.xwz);
            }
        }

        public void ContourEdgeProc(in int4 node,int dir)
        {
            if(math.any(nodes.Types(node) == (int)NodeType.None))
            {
                return;
            }

            if(math.all(nodes.Types(node) != (int)NodeType.Internal))
            {
                ContourProcessEdge(node,dir);
                return;
            }

            ref var edgeMask = ref edgeProcEdgeMask[dir];
            for(int i = 0; i < 2; i++)
            {
                var edgeNodes = math.select(node,nodes.ChildIndexes(node) + edgeMask[i],nodes.Types(node) == (int)NodeType.Internal);
                ContourEdgeProc(edgeNodes,dir);
            }
        }

        public void ContourFaceProc(in int2 node,int dir)
        {
            var types = nodes.Types(node);
            if(math.any(types == (int)NodeType.None))
                return;

            if(math.all(types != (int)NodeType.Internal))
                return;

            ref var faceMask = ref faceProcFaceMask[dir];
            for(int i = 0; i < 4; i++)
            {
                var faceNodes = math.select(node,nodes.ChildIndexes(node) + faceMask[i],types == (int)NodeType.Internal);
                ContourFaceProc(faceNodes,dir);
            }

            ref var masks = ref faceProcEdgeMask[dir];
            for(int i = 0; i < 4; i++)
            {
                int4 edgeNodes = (dir == 1) == (i < 2) ? node.xxyy : node.xyxy;
                edgeNodes = math.select(edgeNodes,nodes.ChildIndexes(edgeNodes) + masks.mask[i],nodes.Types(edgeNodes) == (int)NodeType.Internal);
                ContourEdgeProc(edgeNodes,masks.dir[i]);
            }
        }

        public void Dispose() => Dispose(default);
        public JobHandle Dispose(JobHandle inputDeps)
        {
            return JobHandle.CombineDependencies(
                nodeVertexIndex.Dispose(inputDeps),
                vertexBuffer.Dispose(inputDeps),
                indexBuffer.Dispose(inputDeps)
            );
        }
    }
}