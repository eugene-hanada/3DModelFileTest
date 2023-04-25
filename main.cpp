#include <iostream>
#include <string>
#include <memory>
#include <deque>
#include <EugeneLib.h>

#include "EugeneLib/Include/Graphics/IndexView.h"

#include "EugeneLib/Include/Math/Geometry.h"

#define __STDC_LIB_EXT1__
#define TINYGLTF_IMPLEMENTATION
//#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tinygltf/tiny_gltf.h"

struct GltfVertex
{
	Eugene::Vector3 pos;
	Eugene::Vector3 normal;
	Eugene::Vector2 uv;
};

struct Camera
{
	Eugene::Matrix4x4 view;
	Eugene::Matrix4x4 projection;
};


void LoadGltf(std::vector<GltfVertex>& output, std::vector<std::uint16_t>& outIndex, const std::string& path)
{
	tinygltf::TinyGLTF gltf;
	tinygltf::Model model;
	std::string err;
	std::string warn;
	gltf.LoadASCIIFromFile(&model, &err, &warn, path);


	for (auto& mesh : model.meshes)
	{
		for (auto& primitive : mesh.primitives)
		{
			auto& posAccsessor = model.accessors[primitive.attributes["POSITION"]];
			auto& normAccessor = model.accessors[primitive.attributes["NORMAL"]];
			auto& uvAccessor = model.accessors[primitive.attributes["TEXCOORD_0"]];

			auto& posBufferView = model.bufferViews[posAccsessor.bufferView];
			auto& normBufferView = model.bufferViews[normAccessor.bufferView];
			auto& uvBufferView = model.bufferViews[uvAccessor.bufferView];

			auto& posBuffer = model.buffers[posBufferView.buffer];
			auto& normBuffer = model.buffers[normBufferView.buffer];
			auto& uvBuffer = model.buffers[uvBufferView.buffer];


			auto pos = reinterpret_cast<float*>(&posBuffer.data[posBufferView.byteOffset + posAccsessor.byteOffset]);
			auto norm = reinterpret_cast<float*>(&normBuffer.data[normBufferView.byteOffset + normAccessor.byteOffset]);
			auto uv = reinterpret_cast<float*>(&uvBuffer.data[uvBufferView.byteOffset + uvAccessor.byteOffset]);

			output.resize(posAccsessor.count);
			for (int i = 0; i < posAccsessor.count; i++)
			{
				output[i] = GltfVertex{
					Eugene::Vector3{ pos[i * 3 + 0],pos[i * 3 + 1],pos[i * 3 + 2] } ,
					Eugene::Vector3{ norm[i * 3 + 0],norm[i * 3 + 1],norm[i * 3 + 2] },
					Eugene::Vector2{ uv[i * 2 + 0],uv[i * 2 + 1] }
				};
			}

			auto& accessor = model.accessors[primitive.indices];
			auto& bufferView = model.bufferViews[accessor.bufferView];
			auto& buffer = model.buffers[bufferView.buffer];
			auto idxP = reinterpret_cast<std::uint16_t*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
			outIndex.resize(accessor.count);
			for (int i = 0; i < accessor.count; i++)
			{
				outIndex[i] = idxP[i];
			}
			break;
		}
		break;
	}
}


int main()
{
	std::unique_ptr<Eugene::System> system;
	system.reset(Eugene::CreateSystem({ 1280.0f,720.0f }, u8"モデルテスト"));

	std::unique_ptr<Eugene::Graphics> graphics;
	std::unique_ptr<Eugene::GpuEngine> gpuEngine;
	{
		auto [gPtr, gpuPtr] = system->CreateGraphics();
		graphics.reset(gPtr);
		gpuEngine.reset(gpuPtr);
	}
	std::unique_ptr<Eugene::CommandList> cmdList;
	cmdList.reset(graphics->CreateCommandList());

	std::unique_ptr<Eugene::GraphicsPipeline> pipeline;
	// 頂点シェーダの入力のレイアウト
	std::vector<Eugene::ShaderInputLayout> layout
	{
		{ "POSITION", 0, Eugene::Format::R32G32B32_FLOAT },
		{ "NORMAL", 0, Eugene::Format::R32G32B32_FLOAT },
		{ "TEXCOORD", 0, Eugene::Format::R32G32_FLOAT }
	};

	// シェーダー
	std::vector<std::pair<Eugene::Shader, Eugene::ShaderType>> shaders
	{
		{ Eugene::Shader{ "./vs.vso" }, Eugene::ShaderType::Vertex },
		{ Eugene::Shader{ "./ps.pso" }, Eugene::ShaderType::Pixel }
	};

	// レンダーターゲット
	std::vector<Eugene::RendertargetLayout> rendertargets
	{
		{ Eugene::Format::R8G8B8A8_UNORM, Eugene::BlendType::Non }
	};

	std::vector<std::vector<Eugene::ShaderLayout>> shaderLayout
	{
		{ Eugene::ShaderLayout{ Eugene::ViewType::ConstantBuffer, 1,0 } }
	};

	pipeline.reset(
		graphics->CreateGraphicsPipeline(
			layout,
			shaders,
			rendertargets,
			Eugene::TopologyType::Triangle,
			false, false,
			shaderLayout
		));


	std::vector<GltfVertex> vertex_;
	std::vector<std::uint16_t> index;

	LoadGltf(vertex_, index, "untitled.gltf");

	// 頂点バッファとビュー作成
	std::unique_ptr<Eugene::BufferResource> vertexBuffer;
	std::unique_ptr<Eugene::VertexView> vertexView;
	vertexBuffer.reset(graphics->CreateUploadableBufferResource(sizeof(GltfVertex) * vertex_.size()));
	vertexView.reset(graphics->CreateVertexView(sizeof(GltfVertex), vertex_.size(), *vertexBuffer));
	GltfVertex* mapped = reinterpret_cast<GltfVertex*>(vertexBuffer->Map());
	std::copy(vertex_.begin(), vertex_.end(), mapped);
	vertexBuffer->UnMap();

	// インデックスバッファとビュー作成
	std::unique_ptr<Eugene::BufferResource> indexBuffer;
	std::unique_ptr<Eugene::IndexView> indexView;
	indexBuffer.reset(graphics->CreateUploadableBufferResource(sizeof(index[0]) * index.size()));
	indexView.reset(graphics->CreateIndexView(sizeof(index[0]) * index.size(), Eugene::Format::R16_UINT, *indexBuffer));
	std::uint16_t* indexMap = reinterpret_cast<std::uint16_t*>(indexBuffer->Map());
	std::copy(index.begin(), index.end(), indexMap);
	indexBuffer->UnMap();

	// カメラ行列を作成
	std::unique_ptr<Eugene::BufferResource> matrixBuffer;
	std::unique_ptr<Eugene::ShaderResourceViews> matrixViews;
	matrixBuffer.reset(graphics->CreateUploadableBufferResource(256));
	matrixViews.reset(graphics->CreateShaderResourceViews(1));
	matrixViews->CreateConstantBuffer(*matrixBuffer, 0);
	Camera camera;
	Camera* mapCamera = reinterpret_cast<Camera*>(matrixBuffer->Map());
	auto camPos = Eugene::Vector3{ 0.0f,0.5f,-2.0f };;
	Eugene::GetLookAtMatrix(camera.view, camPos, camPos + Eugene::forwardVector3<float> *2.0f, Eugene::upVector3<float>);
	Eugene::GetPerspectiveFovMatrix(camera.projection, Eugene::Deg2Rad(90.0f), 1280.0f / 720.0f);
	*mapCamera = camera;
	matrixBuffer->UnMap();


	float color[]{ 1.0f, 0.0f, 0.0f,1.0f };
	while (system->Update())
	{
		cmdList->Begin();
		cmdList->TransitionRenderTargetBegin(graphics->GetBackBufferResource());
		cmdList->ClearRenderTarget(graphics->GetViews(), color, graphics->GetNowBackBufferIndex());
		cmdList->SetRenderTarget(graphics->GetViews(), graphics->GetNowBackBufferIndex());
		cmdList->SetGraphicsPipeline(*pipeline);

		cmdList->SetPrimitiveType(Eugene::PrimitiveType::Triangle);

		// シザーレクトセット
		cmdList->SetScissorrect({ 0,0 }, { 1280, 720 });

		// ビューポートセット
		cmdList->SetViewPort({ 0.0f,0.0f }, { 1280.0f, 720.0f });

		cmdList->SetShaderResourceView(*matrixViews, 0, 0);

		cmdList->SetVertexView(*vertexView);

		cmdList->SetIndexView(*indexView);

		cmdList->DrawIndexed(index.size());
		//cmdList->Draw(vertex_.size());

		cmdList->TransitionRenderTargetEnd(graphics->GetBackBufferResource());
		cmdList->End();

		gpuEngine->Push(*cmdList);
		gpuEngine->Execute();
		gpuEngine->Wait();

		graphics->Present();
	}
	return 0;
}