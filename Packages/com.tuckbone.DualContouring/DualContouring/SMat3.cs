using Unity.Mathematics;

public struct SMat3
{
    public float m00, m01, m02, m11, m12, m22;

    public float3 c0 => new float3(m00,m01,m02);
    public float3 c1 => new float3(m01,m11,m12);
    public float3 c2 => new float3(m02,m12,m22);

    public SMat3(float a00,float a01,float a02,float a11,float a12,float a22)
    {
        m00 = a00;
        m01 = a01;
        m02 = a02;
        m11 = a11;
        m12 = a12;
        m22 = a22;
    }

    public void setSymmetric(float a00,float a01,float a02,float a11,float a12,float a22)
    {
        this = new SMat3(a00,a01,a02,a11,a12,a22);
    }

    public void SetSymmetricTo(ref float3x3 m)
    {
        m.c0 = c0;
        m.c1 = c1;
        m.c2 = c2;
    }

    public static implicit operator float3x3(in SMat3 s)
    {
        var m = float3x3.zero;
        s.SetSymmetricTo(ref m);
        return m;
    }

    public float fnorm()
    {
        return math.sqrt((m00 * m00) + (m11 * m11) + (m22 * m22) + (2 * ((m01 * m01) + (m02 * m02) + (m12 * m12))));
    }

    public float off()
    {
        return math.sqrt(2 * ((m01 * m01) + (m02 * m02) + (m12 * m12)));
    }

    public float3 vmul_symmetric(in float3 v)
    {
        return new float3(math.dot(c0,v),math.dot(c1,v),math.dot(c2,v));
    }
}