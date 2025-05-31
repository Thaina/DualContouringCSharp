using System;
using System.Collections.Generic;

using UnityEngine;

using Unity.Jobs;
using Unity.Collections;
using Unity.Mathematics;

using Code.Utils;
using UnityEngine.Serialization;

public class DualContouringGenerator : MonoBehaviour
{
    [Serializable]
    public struct GLM : Octree.IGenerator
    {
        public float3 cubePos;
        public float3 cubeSize;
        public float4 spherePosSize;
        public int octaves;
        public float frequency;
        public float lacunarity;
        public float persistence;
        public float Density_Func(in float3 position)
        {
            float MAX_HEIGHT = 10.0f;
            float noise = FractalNoise(position.xz);
            float terrain = position.y - (MAX_HEIGHT * noise);

            float cube = Cuboid(position,cubePos,cubeSize);
            float sphere = Sphere(position,spherePosSize.xyz,spherePosSize.w);

            return math.max(-cube,math.min(sphere,terrain));
        }

        public static float Sphere(float3 worldPosition,float3 origin,float radius)
        {
            return math.distance(worldPosition,origin) - radius;
        }

        public static float Cuboid(float3 worldPosition,float3 origin,float3 halfDimensions)
        {
            float3 local_pos = worldPosition - origin;
            float3 pos = local_pos;

            float3 d = math.abs(pos) - halfDimensions;
            float m = math.max(d.x,math.max(d.y,d.z));
            return math.min(m,math.length(d));
        }

        public float FractalNoise(float2 position)
        {
            float SCALE = 1.0f / 128.0f;
            float2 p = position * SCALE;
            float value = 0.0f;

            float amplitude = 1.0f;
            p *= frequency;

            for(int i = 0; i < octaves; i++)
            {
                value += noise.cnoise(p) * amplitude;
                p *= lacunarity;
                amplitude *= persistence;
            }

            // move into [0, 1] range
            return 0.5f + (0.5f * value);
        }
    }

	const int MAX_THRESHOLDS = 5;
	static float[] THRESHOLDS = new float[MAX_THRESHOLDS] { -1.0f, 0.1f, 1.0f, 10.0f, 50.0f };
	public int thresholdIndex = -1;

    [FormerlySerializedAs("glm")]
    public GLM generator;

    // octreeSize must be a power of two!
    public int octreeSize = 64;
    public Material material;

    async Awaitable GenMesh(Mesh mesh)
    {
        thresholdIndex = (thresholdIndex + 1) % MAX_THRESHOLDS;

        NativeList<Octree.Node> nodes = default;
        while(!destroyCancellationToken.IsCancellationRequested)
        {
            int nodeSize = 8 * (((int)math.pow(octreeSize,3) * 8) - 1) / (8 - 1);
            if(!nodes.IsCreated || nodes.Capacity != nodeSize)
            {
                nodes.Dispose();
                nodes = new NativeList<Octree.Node>(nodeSize,Allocator.Persistent);
            }

            var octreeJob = new Octree.OctreeJob<GLM>() {
                glm = generator,
                nodes = nodes,
                min = -octreeSize / 2,
                size = octreeSize,
                threshold = THRESHOLDS[thresholdIndex]
            };

            var genHandle = octreeJob.Schedule(1,default);
            while(!genHandle.IsCompleted)
                await Awaitable.EndOfFrameAsync();

            genHandle.Complete();

            using var genMeshJob = new Octree.GenMeshJob() {
                nodes = nodes,
                nodeVertexIndex = new(Allocator.Persistent),
                indexBuffer = new(Allocator.Persistent),
                vertexBuffer = new(Allocator.Persistent),
            };

            genHandle = genMeshJob.Schedule(1,genHandle);
            while(!genHandle.IsCompleted)
                await Awaitable.EndOfFrameAsync();

            genHandle.Complete();

            var vertices = genMeshJob.vertexBuffer.AsArray().ToArray();
            var indices = genMeshJob.indexBuffer.AsArray().Reinterpret<int>(sizeof(int) * 3);

            if(octreeJob.nodes[0].Type == Octree.NodeType.None)
                Debug.Log("root is null");

            var vertArray = new Vector3[vertices.Length];
            var normArray = new Vector3[vertices.Length];
            var uvs = new Vector2[vertices.Length];
            for(int i = 0; i < vertices.Length; i++)
            {
                var pos = vertices[i][0];
                vertArray[i] = pos;
                normArray[i] = vertices[i][1];
                uvs[i] = pos.xz;
            }

            mesh.Clear();
            mesh.vertices = vertArray;
            mesh.uv = uvs;
            mesh.normals = normArray;
            mesh.triangles = indices.ToArray();
            mesh.RecalculateBounds();

            Debug.Log("GenMesh");
        }

        Debug.Log("destroyed");
        nodes.Dispose();
    }

    void Start()
    {
        var go = new GameObject("Mesh");

        go.transform.SetParent(gameObject.transform);

        var meshRenderer = go.AddComponent<MeshRenderer>();
        meshRenderer.sharedMaterial = material;

        var filter = go.AddComponent<MeshFilter>();
        GenMesh(filter.mesh);
	}

	public float moveSpeed = 10;
    void Update()
    {
        var cam = Camera.main.transform;
        PlayerInput.Update(cam,out var move);
        cam.position += ((cam.right * move.x) + (cam.up * move.y) + (cam.forward * move.z)) * moveSpeed * Time.deltaTime;

        generator.spherePosSize.xyz = math.mul(quaternion.Euler(0,0.5f * math.PI * Time.deltaTime,0),generator.spherePosSize.xyz);
	}

    void OnGUI ()
    {
        //You need a killer computer to be able to use this!! (or drop the octree size some.. )
        //Octree.DrawOctree(root, 0);
    }
}
