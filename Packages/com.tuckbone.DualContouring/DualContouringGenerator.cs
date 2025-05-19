using System.Collections.Generic;

using UnityEngine;

using Unity.Mathematics;

using Code.Utils;

public class DualContouringGenerator : MonoBehaviour
{
    const int MAX_THRESHOLDS = 5;
	static float[] THRESHOLDS = new float[MAX_THRESHOLDS] { -1.0f, 0.1f, 1.0f, 10.0f, 50.0f };
	static int thresholdIndex = -1;

    // octreeSize must be a power of two!
	public int octreeSize = 64;
    public Material material;
    void Start()
    {
        thresholdIndex = (thresholdIndex + 1) % MAX_THRESHOLDS;
        var vertices = new List<float3x2>();
        var indices = new List<int>();

        using var root = Octree.BuildOctree(-octreeSize / 2,octreeSize,THRESHOLDS[thresholdIndex],vertices,indices);

        if(root[0].Type == Octree.NodeType.None)
            Debug.Log("root is null");

        var go = new GameObject("Mesh");

        go.transform.SetParent(gameObject.transform);

        var meshRenderer = go.AddComponent<MeshRenderer>();
        var filter = go.AddComponent<MeshFilter>();

        meshRenderer.sharedMaterial = material;

        var vertArray = new Vector3[vertices.Count];
        var normArray = new Vector3[vertices.Count];
        var uvs = new Vector2[vertices.Count];
        for(int i = 0; i < vertices.Count; i++)
        {
            var pos = vertices[i][0];
            vertArray[i] = pos;
            normArray[i] = vertices[i][1];
            uvs[i] = pos.xz;
        }

        var mesh = filter.mesh;
        mesh.vertices = vertArray;
        mesh.uv = uvs;
        mesh.normals = normArray;
        mesh.triangles = indices.ToArray();
        mesh.RecalculateBounds();
	}

    public float moveSpeed = 10;
    void Update()
    {
        var cam = Camera.main.transform;
        PlayerInput.Update(cam,out var move);
        cam.position += ((cam.right * move.x) + (cam.up * move.y) + (cam.forward * move.z)) * moveSpeed * Time.deltaTime;
	}

    void OnGUI ()
    {
        //You need a killer computer to be able to use this!! (or drop the octree size some.. )
        //Octree.DrawOctree(root, 0);
    }
}
