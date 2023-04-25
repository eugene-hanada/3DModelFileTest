
struct VertexInput
{
	float4 pos : POSITION;
	float4 normal : NORMAL;
	float2 uv : TEXCOORD;
};

struct VertexOutput
{
	float4 svPos : SV_POSITION;
	float3 normal : NORMAL;
	float2 uv : TEXCOORD0;
};

cbuffer Camera : register(b0)
{
	matrix view;
	matrix projection;
}

VertexOutput main(VertexInput input)
{
	VertexOutput output;
	float4 pos = mul(view, input.pos);

	output.svPos = mul(projection, pos);
	output.normal = mul(view, input.normal).xyz;
	output.uv = input.uv;
	return output;
}