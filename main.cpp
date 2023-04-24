#include <iostream>
#include <string>
#include <memory>
#include <deque>
#include <EugeneLib.h>

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


void LoadGltf(std::vector<GltfVertex>& output, std::vector<std::uint16_t>& outIndex, const std::string& path)
{
	tinygltf::TinyGLTF gltf;
	tinygltf::Model model;
	std::string err;
	std::string warn;
	gltf.LoadASCIIFromFile(&model, &err, &warn, path);

	std::deque<Eugene::Vector3> posList;
	std::deque<Eugene::Vector3> normalList;
	std::deque<Eugene::Vector2> uvList;
	std::deque<std::uint16_t> index_;

	for (auto& mesh : model.meshes)
	{
		for (auto& primitive : mesh.primitives)
		{


			auto& accessor = model.accessors[primitive.attributes["POSITION"]];
			auto& bufferView = model.bufferViews[accessor.bufferView];
			auto& buffer = model.buffers[bufferView.buffer];

			auto p = reinterpret_cast<float*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
			for (int i = 0; i < accessor.count; i++)
			{
				posList.emplace_back(Eugene::Vector3{ p[i * 3 + 0],p[i * 3 + 1],p[i * 3 + 2] });
			}

			accessor = model.accessors[primitive.attributes["NORMAL"]];
			bufferView = model.bufferViews[accessor.bufferView];
			buffer = model.buffers[bufferView.buffer];
			p = reinterpret_cast<float*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
			for (int i = 0; i < accessor.count; i++)
			{
				normalList.emplace_back(Eugene::Vector3{ p[i * 3 + 0],p[i * 3 + 1],p[i * 3 + 2] });
			}

			accessor = model.accessors[primitive.attributes["TEXCOORD_0"]];
			bufferView = model.bufferViews[accessor.bufferView];
			buffer = model.buffers[bufferView.buffer];
			p = reinterpret_cast<float*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
			for (int i = 0; i < accessor.count; i++)
			{
				uvList.emplace_back(Eugene::Vector2{ p[i * 2 + 0],p[i * 2 + 1] });
			}

			accessor = model.accessors[primitive.indices];
			bufferView = model.bufferViews[accessor.bufferView];
			buffer = model.buffers[bufferView.buffer];
			auto idxP = reinterpret_cast<std::uint16_t*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
			for (int i = 0; i < accessor.count; i++)
			{
				index_.emplace_back(idxP[i]);
			}
		}
	}

	auto size = posList.size();
	output.resize(size);
	for (int i = 0; i < size; i++)
	{
		output[i] = GltfVertex{ posList[i],normalList[i],uvList[i] };
	}

	outIndex.resize(index_.size());
	for (int i = 0;i < index_.size(); i++)
	{
		outIndex[i] = index_[i];
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
		{ "POSITION", 0, Eugene::Format::R32G32_FLOAT }
	};

	// シェーダー
	std::vector<std::pair<Eugene::Shader, Eugene::ShaderType>> shaders
	{
		{ Eugene::Shader{ "./Asset/vs.vso" }, Eugene::ShaderType::Vertex },
		{ Eugene::Shader{ "./Asset/ps.pso" }, Eugene::ShaderType::Pixel }
	};

	// レンダーターゲット
	std::vector<Eugene::RendertargetLayout> rendertargets
	{
		{ Eugene::Format::R8G8B8A8_UNORM, Eugene::BlendType::Non }
	};

	std::vector<std::vector<Eugene::ShaderLayout>> shaderLayout
	{
		{ Eugene::ShaderLayout{ Eugene::ViewType::ConstantBuffer, 2,0 } }
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

	std::unique_ptr<Eugene::BufferResource> indexBuffer;


	float color[]{ 1.0f, 0.0f, 0.0f,1.0f };
	while (system->Update())
	{
		cmdList->Begin();
		cmdList->TransitionRenderTargetBegin(graphics->GetBackBufferResource());
		cmdList->ClearRenderTarget(graphics->GetViews(), color, graphics->GetNowBackBufferIndex());

		cmdList->TransitionRenderTargetEnd(graphics->GetBackBufferResource());
		cmdList->End();

		gpuEngine->Push(*cmdList);
		gpuEngine->Execute();
		gpuEngine->Wait();

		graphics->Present();
	}
	return 0;
}