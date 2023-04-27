#include <iostream>
#include <string>
#include <memory>
#include <deque>
#include <fstream>
#include <filesystem>
#include <EugeneLib.h>

#include "EugeneLib/Include/Graphics/IndexView.h"

#include "EugeneLib/Include/Math/Geometry.h"

#define __STDC_LIB_EXT1__
#define TINYGLTF_IMPLEMENTATION
//#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "tinygltf/tiny_gltf.h"

/// <summary>
/// 頂点
/// </summary>
struct GltfVertex
{
	Eugene::Vector3 pos;
	Eugene::Vector3 normal;
	Eugene::Vector3 tan;
	Eugene::Vector2 uv;
};

/// <summary>
/// アニメーションあり頂点
/// </summary>
struct SkeletalGltfVertex
{
	Eugene::Vector3 pos;
	Eugene::Vector3 normal;
	Eugene::Vector2 uv;
	Eugene::Vector3 tan;
	std::uint16_t joint[4];
	float weight[4];
};

struct MeshHeader
{
	char sig[4]{ 'm','e','s','h' };
	float version;
	std::uint32_t meshNum;
	std::uint32_t vertexSize;
	std::uint32_t indexSize;
};

struct Material
{
	std::uint32_t baseColorTexture;
	std::uint32_t normalMapTexture;
};

struct Camera
{
	Eugene::Matrix4x4 view;
	Eugene::Matrix4x4 projection;
};

struct Mesh
{
	Mesh(){}
	Mesh(Mesh&& mesh) noexcept
	{
		vertex = std::move(mesh.vertex);
		index = std::move(mesh.index);
		vertexBuffer = std::move(mesh.vertexBuffer);
		vertexView = std::move(mesh.vertexView);
		indexBuffer = std::move(mesh.indexBuffer);
		indexView = std::move(mesh.indexView);
	}

	Mesh& operator=(Mesh&& mesh) noexcept
	{
		vertex = std::move(mesh.vertex);
		index = std::move(mesh.index);
		vertexBuffer = std::move(mesh.vertexBuffer);
		vertexView = std::move(mesh.vertexView);
		indexBuffer = std::move(mesh.indexBuffer);
		indexView = std::move(mesh.indexView);
	}
	std::vector<GltfVertex> vertex;
	std::vector<std::uint16_t> index;
	std::unique_ptr<Eugene::BufferResource> vertexBuffer;
	std::unique_ptr<Eugene::VertexView> vertexView;
	std::unique_ptr<Eugene::BufferResource> indexBuffer;
	std::unique_ptr<Eugene::IndexView> indexView;
};


void LoadGltf(std::list<Mesh>& list, const std::string& path)
{
	tinygltf::TinyGLTF gltf;
	tinygltf::Model model;
	std::string err;
	std::string warn;
	gltf.LoadASCIIFromFile(&model, &err, &warn, path);
	std::vector<std::uint8_t> joints;

	MeshHeader h{};
	std::ofstream file{ "./" + path.substr(0, path.find_last_of(".")) + ".mesh", std::ios::binary };
	h.version = 0.01f;
	for (auto& mesh : model.meshes)
	{
		h.meshNum += static_cast<std::uint32_t>(mesh.primitives.size());
	}
	
	h.vertexSize = static_cast<std::uint32_t>(sizeof(GltfVertex));
	h.indexSize = static_cast<std::uint32_t>(sizeof(std::uint16_t));
	file.write(reinterpret_cast<char*>(&h), sizeof(h));

	//// 画像情報
	//for (auto& img : model.images)
	//{
	//	auto size = img.name.size();
	//	file.write(reinterpret_cast<char*>(&size), sizeof(size));
	//	file.write(img.name.data(), sizeof(img.name[0]) * size);

	//	//stbi_write_png(("./" + img.name + ".png").c_str(), img.width, img.height, img.component, img.image.data(), 0);
	//}

	//Material m{};
	//for (auto& material : model.materials)
	//{
	//	m.baseColorTexture = material.pbrMetallicRoughness.baseColorTexture.index;
	//	m.normalMapTexture = material.normalTexture.index;
	//	file.write(reinterpret_cast<char*>(&m), sizeof(m));
	//}


	for (auto& mesh : model.meshes)
	{
		for (auto& primitive : mesh.primitives)
		{
			auto& posAccsessor = model.accessors[primitive.attributes["POSITION"]];
			auto& normAccessor = model.accessors[primitive.attributes["NORMAL"]];
			auto& tanAccessor = model.accessors[primitive.attributes["TANGENT"]];
			auto& uvAccessor = model.accessors[primitive.attributes["TEXCOORD_0"]];
			

			//auto& boneAccessor = model.accessors[primitive.attributes["JOINTS_0"]];
			//auto& weighisAccessor = model.accessors[primitive.attributes["WEIGHIS_0"]];

			auto& posBufferView = model.bufferViews[posAccsessor.bufferView];
			auto& normBufferView = model.bufferViews[normAccessor.bufferView];
			auto& tanBufferView = model.bufferViews[tanAccessor.bufferView];
			auto& uvBufferView = model.bufferViews[uvAccessor.bufferView];
			//auto& boneBufferView = model.bufferViews[boneAccessor.bufferView];
			//auto& weighisBufferView = model.bufferViews[weighisAccessor.bufferView];

			auto& posBuffer = model.buffers[posBufferView.buffer];
			auto& normBuffer = model.buffers[normBufferView.buffer];
			auto& tanBuffer = model.buffers[tanBufferView.buffer];
			auto& uvBuffer = model.buffers[uvBufferView.buffer];
			//auto& boneBuffer = model.buffers[boneBufferView.buffer];
			//auto& weighisBuffer = model.buffers[weighisBufferView.buffer];

			auto pos = reinterpret_cast<float*>(&posBuffer.data[posBufferView.byteOffset + posAccsessor.byteOffset]);
			auto norm = reinterpret_cast<float*>(&normBuffer.data[normBufferView.byteOffset + normAccessor.byteOffset]);
			auto tan = reinterpret_cast<float*>(&tanBuffer.data[tanBufferView.byteOffset + tanAccessor.byteOffset]);
			auto uv = reinterpret_cast<float*>(&uvBuffer.data[uvBufferView.byteOffset + uvAccessor.byteOffset]);
			//auto joint = reinterpret_cast<std::uint8_t*>(&boneBuffer.data[boneBufferView.byteOffset + boneAccessor.byteOffset]);
			//auto weight = reinterpret_cast<float*>(&weighisBuffer.data[weighisBufferView.byteOffset + weighisAccessor.byteOffset]);

			Mesh mesh;

			mesh.vertex.resize(posAccsessor.count);
			for (int i = 0; i < posAccsessor.count; i++)
			{
				//std::uint8_t j[4];
				mesh.vertex[i].pos = Eugene::Vector3{ -pos[i * 3 + 0] ,pos[i * 3 + 1],pos[i * 3 + 2] };
				mesh.vertex[i].normal = Eugene::Vector3{ -norm[i * 3 + 0],norm[i * 3 + 1],norm[i * 3 + 2] };
				mesh.vertex[i].tan = Eugene::Vector3{ -tan[i * 3 + 0],tan[i * 3 + 1],tan[i * 3 + 2] };
				mesh.vertex[i].uv = Eugene::Vector2{ uv[i * 2 + 0],1.0f - uv[i * 2 + 1] };
				/*mesh.vertex[i].joint[0] = joint[i * 4 + 0];
				mesh.vertex[i].joint[1] = joint[i * 4 + 1];
				mesh.vertex[i].joint[2] = joint[i * 4 + 2];
				mesh.vertex[i].joint[3] = joint[i * 4 + 3];
				
				mesh.vertex[i].weight[0] = weight[i * 4 + 0];
				mesh.vertex[i].weight[1] = weight[i * 4 + 1];
				mesh.vertex[i].weight[2] = weight[i * 4 + 2];
				mesh.vertex[i].weight[3] = weight[i * 4 + 3];*/
			}

			auto& accessor = model.accessors[primitive.indices];
			auto& bufferView = model.bufferViews[accessor.bufferView];
			auto& buffer = model.buffers[bufferView.buffer];
			auto idxP = reinterpret_cast<std::uint16_t*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
			mesh.index.resize(accessor.count);
			for (int i = 0; i < accessor.count; i++)
			{
				mesh.index[i] = idxP[i];
			}

			std::reverse(mesh.index.begin(), mesh.index.end());

			// メッシュ情報
			
			std::uint32_t size = static_cast<std::uint32_t>(mesh.vertex.size());
			file.write(reinterpret_cast<char*>(&size), sizeof(size));
			file.write(reinterpret_cast<char*>(mesh.vertex.data()), sizeof(mesh.vertex[0]) * mesh.vertex.size());

			size = static_cast<std::uint32_t>(mesh.index.size());
			file.write(reinterpret_cast<char*>(&size), sizeof(size));
			file.write(reinterpret_cast<char*>(mesh.index.data()), sizeof(mesh.index[0]) * mesh.index.size());
			
			if (primitive.material == -1)
			{
				size = 0u;
			}
			else
			{
				size = static_cast<std::uint32_t>(model.materials[primitive.material].name.size());
			}
			file.write(reinterpret_cast<char*>(&size), sizeof(size));

			if (size != 0u)
			{
				file.write(
					model.materials[primitive.material].name.data(),
					sizeof(model.materials[primitive.material].name[0]) * model.materials[primitive.material].name.size()
				);
			}

			list.emplace_back(std::move(mesh));
		}
	}

}

void LoadMesh(const std::filesystem::path& path, std::vector<Mesh>& meshs)
{
	std::ifstream file{ path , std::ios::binary};
	MeshHeader h{};
	file.read(reinterpret_cast<char*>(&h), sizeof(h));
	//std::vector<Mesh> meshs{ h.meshNum };
	meshs.resize(h.meshNum);
	for (std::uint32_t i = 0u; i < h.meshNum; i++)
	{
		std::uint32_t size = 0u;
		file.read(reinterpret_cast<char*>(&size), sizeof(size));
		meshs[i].vertex.resize(size);
		file.read(reinterpret_cast<char*>(meshs[i].vertex.data()), h.vertexSize * size);

		file.read(reinterpret_cast<char*>(&size), sizeof(size));
		meshs[i].index.resize(size);
		file.read(reinterpret_cast<char*>(meshs[i].index.data()), h.indexSize * size);

		file.read(reinterpret_cast<char*>(&size), sizeof(size));
		std::string matName;
		matName.resize(size);
		file.read(matName.data(), sizeof(matName[0]) * matName.size());
	}
}

void MeshInit(Eugene::Graphics& graphics, Mesh& mesh)
{
	mesh.vertexBuffer.reset(graphics.CreateUploadableBufferResource(sizeof(GltfVertex) * mesh.vertex.size()));
	mesh.vertexView.reset(graphics.CreateVertexView(sizeof(GltfVertex), mesh.vertex.size(), *mesh.vertexBuffer));
	GltfVertex* mapped = reinterpret_cast<GltfVertex*>(mesh.vertexBuffer->Map());
	std::copy(mesh.vertex.begin(), mesh.vertex.end(), mapped);
	mesh.vertexBuffer->UnMap();

	// インデックスバッファとビュー作成
	mesh.indexBuffer.reset(graphics.CreateUploadableBufferResource(sizeof(mesh.index[0]) * mesh.index.size()));
	mesh.indexView.reset(graphics.CreateIndexView(sizeof(mesh.index[0]) * mesh.index.size(), Eugene::Format::R16_UINT, *mesh.indexBuffer));
	std::uint16_t* indexMap = reinterpret_cast<std::uint16_t*>(mesh.indexBuffer->Map());
	std::copy(mesh.index.begin(), mesh.index.end(), indexMap);
	mesh.indexBuffer->UnMap();
}


int main(int argc, char* argv[])
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

	std::vector<Mesh> meshList;
	std::list<Mesh> mlist;
	LoadGltf(mlist, "Swat.gltf");
	LoadMesh("Swat.mesh", meshList);
	for (auto& mesh : meshList)
	{
		MeshInit(*graphics, mesh);
	}

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

	/*	cmdList->SetVertexView(*vertexView);

		cmdList->SetIndexView(*indexView);*/

		//cmdList->DrawIndexed(index.size());
		
		for (auto& mesh : meshList)
		{
			cmdList->SetVertexView(*mesh.vertexView);
			cmdList->SetIndexView(*mesh.indexView);
			cmdList->DrawIndexed(mesh.index.size());
		}

		cmdList->TransitionRenderTargetEnd(graphics->GetBackBufferResource());
		cmdList->End();

		gpuEngine->Push(*cmdList);
		gpuEngine->Execute();
		gpuEngine->Wait();

		graphics->Present();
	}
	return 0;
}