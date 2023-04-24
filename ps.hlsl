struct PixelInput
{
	float4 svPos : SV_POSITION;
	float3 normal : NORMAL;
	float2 uv : TEXCOORD0;
};



float4 main(PixelInput input) : SV_TARGET
{
	return float4(input.normal, 1.0f);
}