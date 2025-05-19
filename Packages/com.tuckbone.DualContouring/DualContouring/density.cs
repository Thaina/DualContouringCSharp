using Unity.Mathematics;

public static class glm
{
    public static float Sphere(float3 worldPosition, float3 origin, float radius)
    {
        return math.distance(worldPosition,origin) - radius;
    }

    public static float Cuboid(float3 worldPosition, float3 origin, float3 halfDimensions)
    {
        float3 local_pos = worldPosition - origin;
        float3 pos = local_pos;

        float3 d = math.abs(pos) - halfDimensions;
        float m = math.max(d.x, math.max(d.y, d.z));
        return math.min(m, math.length(d));
    }

    public static float FractalNoise(int octaves, float frequency, float lacunarity, float persistence, float2 position)
    {
        float SCALE = 1.0f / 128.0f;
        float2 p = position * SCALE;
        float value = 0.0f;

        float amplitude = 1.0f;
        p *= frequency;

        for (int i = 0; i < octaves; i++)
        {
            value += noise.cnoise(p) * amplitude;
            p *= lacunarity;
            amplitude *= persistence;
        }

        // move into [0, 1] range
        return 0.5f + (0.5f * value);
    }


    public static float Density_Func(float3 worldPosition)
    {
        float MAX_HEIGHT = 10.0f;
        float noise = FractalNoise(4, 0.5343f, 2.2324f, 0.68324f, worldPosition.xz);
        float terrain = worldPosition.y - (MAX_HEIGHT * noise);

        float cube = Cuboid(worldPosition, new float3(-1, 3, -1), new float3(5));
        float sphere = Sphere(worldPosition, new float3(5, 1, 1), 8);

        return math.max(-cube, math.min(sphere, terrain));
    }
}