#include <iostream>
#include <string>
#include <memory>
#include <deque>
#include <fstream>
#include <filesystem>
#include <EugeneLib.h>
#include <Common/Debug.h>
#include <unordered_map>
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
	Eugene::Vector3 tan;
	Eugene::Vector2 uv;
	std::uint16_t joint[4];
	float weight[4];
};

struct Bone
{
	std::string name_;
	std::vector<int> children;
	Eugene::Matrix4x4 matrix_;
};

struct BoneHeader
{
	char sig[4]{ 'b','o','n','e' };
	std::uint32_t version;
	std::uint32_t num;
};

struct MeshHeader
{
	char sig[4]{ 'm','e','s','h' };
	std::uint32_t version;
	std::uint32_t vertexSize;
	std::uint32_t indexSize;
};

struct MaterialHeader
{
	char sig[4]{ 'm','e','s','h' };
	std::uint32_t version;
};

struct Material
{
	std::string colorTexture;
	std::string normalTexture;
	std::string pipelineName;
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
		materialName = std::move(mesh.materialName);
	}

	Mesh& operator=(Mesh&& mesh) noexcept
	{
		vertex = std::move(mesh.vertex);
		index = std::move(mesh.index);
		vertexBuffer = std::move(mesh.vertexBuffer);
		vertexView = std::move(mesh.vertexView);
		indexBuffer = std::move(mesh.indexBuffer);
		indexView = std::move(mesh.indexView);
		materialName = std::move(mesh.materialName);
	}
	std::vector<GltfVertex> vertex;
	std::vector<std::uint16_t> index;
	std::unique_ptr<Eugene::BufferResource> vertexBuffer;
	std::unique_ptr<Eugene::VertexView> vertexView;
	std::unique_ptr<Eugene::BufferResource> indexBuffer;
	std::unique_ptr<Eugene::IndexView> indexView;
	std::string materialName;
};

void ExportMesh(const std::filesystem::path& path, std::vector<GltfVertex>& vert,std::vector<std::uint16_t>& ind)
{
	MeshHeader h{};
	std::ofstream file{ path, std::ios::binary };
	h.version = 0;
	h.vertexSize = static_cast<std::uint32_t>(sizeof(GltfVertex));
	h.indexSize = static_cast<std::uint32_t>(sizeof(std::uint16_t));
	file.write(reinterpret_cast<char*>(&h), sizeof(h));

	std::uint32_t size = static_cast<std::uint32_t>(vert.size());
	file.write(reinterpret_cast<char*>(&size), sizeof(size));
	file.write(reinterpret_cast<char*>(vert.data()), sizeof(vert[0]) * vert.size());

	size = static_cast<std::uint32_t>(ind.size());
	file.write(reinterpret_cast<char*>(&size), sizeof(size));
	file.write(reinterpret_cast<char*>(ind.data()), sizeof(ind[0]) * ind.size());
}

void ExportMesh(const std::filesystem::path& path, std::vector<SkeletalGltfVertex>& vert, std::vector<std::uint16_t>& ind)
{
	MeshHeader h{};
	std::ofstream file{ path, std::ios::binary };
	h.version = 0;
	h.vertexSize = static_cast<std::uint32_t>(sizeof(SkeletalGltfVertex));
	h.indexSize = static_cast<std::uint32_t>(sizeof(std::uint16_t));
	file.write(reinterpret_cast<char*>(&h), sizeof(h));

	std::uint32_t size = static_cast<std::uint32_t>(vert.size());
	file.write(reinterpret_cast<char*>(&size), sizeof(size));
	file.write(reinterpret_cast<char*>(vert.data()), sizeof(vert[0]) * vert.size());

	size = static_cast<std::uint32_t>(ind.size());
	file.write(reinterpret_cast<char*>(&size), sizeof(size));
	file.write(reinterpret_cast<char*>(ind.data()), sizeof(ind[0]) * ind.size());
}

void ExportMaterial(const std::filesystem::path& path, tinygltf::Material& material, tinygltf::Model& model, const std::string& pipeline)
{
	std::ofstream file{ path, std::ios::binary };
	MaterialHeader h{};
	h.version = 0;
	file.write(reinterpret_cast<char*>(&h), sizeof(h));
	auto colorIndex = model.textures[material.pbrMetallicRoughness.baseColorTexture.index].source;
	std::uint32_t size = 0;
	size = model.images[ colorIndex].name.size();
	file.write(reinterpret_cast<char*>(&size), sizeof(size));
	file.write(model.images[colorIndex].name.data(), sizeof(model.images[colorIndex].name[0]) * size);


	auto normalIndex = model.textures[material.normalTexture.index].source;
	size = model.images[normalIndex].name.size();
	file.write(reinterpret_cast<char*>(&size), sizeof(size));
	file.write(model.images[normalIndex].name.data(), sizeof(model.images[normalIndex].name[0]) * size);

	std::string gpipeName = pipeline;
	size = gpipeName.size();
	file.write(reinterpret_cast<char*>(&size), sizeof(size));
	file.write(gpipeName.data(), sizeof(gpipeName[0]) * size);
}

void ExportBone(const std::filesystem::path& path, std::vector<Bone>& bones)
{
	std::ofstream file{ path,std::ios::binary };
	BoneHeader h{};
	h.num = bones.size();

	file.write(reinterpret_cast<char*>(&h), sizeof(h));

	for (auto& bone : bones)
	{
		std::uint32_t size = bone.name_.size();
		file.write(reinterpret_cast<char*>(&size), sizeof(size));
		file.write(reinterpret_cast<char*>(bone.name_.data()), sizeof(bone.name_[0]) * size);

		file.write(reinterpret_cast<char*>(&bone.matrix_), sizeof(bone.matrix_));
		size = static_cast<std::uint32_t>(bone.children.size());
		file.write(reinterpret_cast<char*>(&size), sizeof(size));
		file.write(reinterpret_cast<char*>(bone.children.data()), sizeof(Bone) * size);
	}

}

void LoadGltf(std::vector<Mesh>& list, const std::string& path)
{
	tinygltf::TinyGLTF gltf;
	tinygltf::Model model;
	std::string err;
	std::string warn;
	gltf.LoadASCIIFromFile(&model, &err, &warn, path);


	for (auto& m : model.meshes)
	{
		for (int p = 0; p < m.primitives.size(); p++)
		{
			auto& primitive = m.primitives[p];
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
				mesh.vertex[i].pos = Eugene::Vector3{ -pos[i * 3 + 0] ,pos[i * 3 + 1], pos[i * 3 + 2] };
				mesh.vertex[i].normal = Eugene::Vector3{ -norm[i * 3 + 0],norm[i * 3 + 1],norm[i * 3 + 2] };
				mesh.vertex[i].tan = Eugene::Vector3{ -tan[i * 3 + 0],tan[i * 3 + 1],tan[i * 3 + 2] };
				mesh.vertex[i].uv = Eugene::Vector2{ uv[i * 2 + 0],uv[i * 2 + 1] };
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

			for (int i = 0; i < accessor.count; i++)
			{
				std::swap(mesh.index[i + 0], mesh.index[i + 2]);
				i += 2;
			}

			//std::reverse(mesh.index.begin(), mesh.index.end());

			// メッシュ情報
			
			
			

			if (model.materials[primitive.material].name.size() != 0u)
			{
				mesh.materialName = model.materials[primitive.material].name;
			}
			ExportMesh(path.substr(0, path.find_last_of("."))+ m.name + std::to_string(p) + ".mesh", mesh.vertex,mesh.index);
			list.emplace_back(std::move(mesh));
		}
	}

	
	
	for (auto& material : model.materials)
	{
		ExportMaterial("./" + material.name + ".mat", material, model,"StaticMesh");
	}

}

void LoadBone(tinygltf::Model& model, int idx,std::vector<Bone>& bones, std::unordered_map<std::string, int>& nameTbl)
{
	//bones[idx].children.resize(model.nodes[idx].children.size());
	auto nodeIdx = model.skins[0].joints[idx];
	bones[idx].name_ = model.nodes[nodeIdx].name;
	Eugene::Vector3 pos{ 
		-static_cast<float>(model.nodes[nodeIdx].translation[0]),
		static_cast<float>(model.nodes[nodeIdx].translation[1]),
		static_cast<float>(model.nodes[nodeIdx].translation[2]) 
	};


	Eugene::Quaternion q{ 
		-static_cast<float>(model.nodes[nodeIdx].rotation[0]) ,
		static_cast<float>(model.nodes[nodeIdx].rotation[1] ),
		static_cast<float>(model.nodes[nodeIdx].rotation[2]) ,
		-static_cast<float>(model.nodes[nodeIdx].rotation[3])
	};
	
	Eugene::GetTransformMatrix(bones[idx].matrix_, q, pos, { 1.0f,1.0f,1.0f });

	for (int i = 0; i < model.nodes[nodeIdx].children.size(); i++)
	{
		auto child = model.nodes[nodeIdx].children[i];
		if (nameTbl.contains(model.nodes[child].name))
		{
			bones[idx].children.push_back(nameTbl[model.nodes[child].name]);
			LoadBone(model, bones[idx].children[i], bones, nameTbl);
		}
	}
}

void LoadSkeltalGltf(const std::string& path)
{
	tinygltf::TinyGLTF gltf;
	tinygltf::Model model;
	std::string err;
	std::string warn;
	gltf.LoadASCIIFromFile(&model, &err, &warn, path);


	for (auto& m : model.meshes)
	{
		for (int p = 0; p < m.primitives.size(); p++)
		{
			auto& primitive = m.primitives[p];
			auto& posAccsessor = model.accessors[primitive.attributes["POSITION"]];
			auto& normAccessor = model.accessors[primitive.attributes["NORMAL"]];
			auto& tanAccessor = model.accessors[primitive.attributes["TANGENT"]];
			auto& uvAccessor = model.accessors[primitive.attributes["TEXCOORD_0"]];
			auto& boneAccessor = model.accessors[primitive.attributes["JOINTS_0"]];
			auto& weighisAccessor = model.accessors[primitive.attributes["WEIGHIS_0"]];

			auto& posBufferView = model.bufferViews[posAccsessor.bufferView];
			auto& normBufferView = model.bufferViews[normAccessor.bufferView];
			auto& tanBufferView = model.bufferViews[tanAccessor.bufferView];
			auto& uvBufferView = model.bufferViews[uvAccessor.bufferView];
			auto& boneBufferView = model.bufferViews[boneAccessor.bufferView];
			auto& weighisBufferView = model.bufferViews[weighisAccessor.bufferView];

			auto& posBuffer = model.buffers[posBufferView.buffer];
			auto& normBuffer = model.buffers[normBufferView.buffer];
			auto& tanBuffer = model.buffers[tanBufferView.buffer];
			auto& uvBuffer = model.buffers[uvBufferView.buffer];
			auto& boneBuffer = model.buffers[boneBufferView.buffer];
			auto& weighisBuffer = model.buffers[weighisBufferView.buffer];

			auto pos = reinterpret_cast<float*>(&posBuffer.data[posBufferView.byteOffset + posAccsessor.byteOffset]);
			auto norm = reinterpret_cast<float*>(&normBuffer.data[normBufferView.byteOffset + normAccessor.byteOffset]);
			auto tan = reinterpret_cast<float*>(&tanBuffer.data[tanBufferView.byteOffset + tanAccessor.byteOffset]);
			auto uv = reinterpret_cast<float*>(&uvBuffer.data[uvBufferView.byteOffset + uvAccessor.byteOffset]);
			auto joint = reinterpret_cast<std::uint8_t*>(&boneBuffer.data[boneBufferView.byteOffset + boneAccessor.byteOffset]);
			auto weight = reinterpret_cast<float*>(&weighisBuffer.data[weighisBufferView.byteOffset + weighisAccessor.byteOffset]);

			std::vector<SkeletalGltfVertex> vertex;
			vertex.resize(posAccsessor.count);
			for (int i = 0; i < posAccsessor.count; i++)
			{
				std::uint8_t j[4];
				vertex[i].pos = Eugene::Vector3{ -pos[i * 3 + 0] ,pos[i * 3 + 1], pos[i * 3 + 2] };
				vertex[i].normal = Eugene::Vector3{ -norm[i * 3 + 0],norm[i * 3 + 1],norm[i * 3 + 2] };
				vertex[i].tan = Eugene::Vector3{ -tan[i * 3 + 0],tan[i * 3 + 1],tan[i * 3 + 2] };
				vertex[i].uv = Eugene::Vector2{ uv[i * 2 + 0],uv[i * 2 + 1] };
				vertex[i].joint[0] = joint[i * 4 + 0];
				vertex[i].joint[1] = joint[i * 4 + 1];
				vertex[i].joint[2] = joint[i * 4 + 2];
				vertex[i].joint[3] = joint[i * 4 + 3];

				vertex[i].weight[0] = weight[i * 4 + 0];
				vertex[i].weight[1] = weight[i * 4 + 1];
				vertex[i].weight[2] = weight[i * 4 + 2];
				vertex[i].weight[3] = weight[i * 4 + 3];
			}

			auto& accessor = model.accessors[primitive.indices];
			auto& bufferView = model.bufferViews[accessor.bufferView];
			auto& buffer = model.buffers[bufferView.buffer];
			auto idxP = reinterpret_cast<std::uint16_t*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
			std::vector<std::uint16_t> index;
			index.resize(accessor.count);
			for (int i = 0; i < accessor.count; i++)
			{
				index[i] = idxP[i];
			}

			for (int i = 0; i < accessor.count; i++)
			{
				std::swap(index[i + 0], index[i + 2]);
				i += 2;
			}
			auto name = m.name;
			ExportMesh(path.substr(0, path.find_last_of(".")) + name + std::to_string(p) + ".mesh", vertex, index);
			
		}
	}

	for (auto& material : model.materials)
	{
		ExportMaterial("./" + material.name + ".mat", material, model, "SkeletalMesh");
	}

	for (auto& skin : model.skins)
	{
		DebugLog(skin.name);
		std::unordered_map<std::string, int> map;
		std::vector<Bone> bones;
		bones.resize(skin.joints.size());
		map.reserve(skin.joints.size());
		for (int i = 0; i < skin.joints.size(); i++)
		{
			map.emplace(model.nodes[skin.joints[i]].name,i);
		}

		for (auto& joint : skin.joints)
		{
			LoadBone(model, map[model.nodes[joint].name],bones,map);
		}
		ExportBone(path.substr(0, path.find_last_of(".")) + ".bone", bones);
	}
}

void LoadMesh(const std::filesystem::path& path, std::vector<Mesh>& meshs)
{
	//std::ifstream file{ path , std::ios::binary};
	//MeshHeader h{};
	//file.read(reinterpret_cast<char*>(&h), sizeof(h));
	////std::vector<Mesh> meshs{ h.meshNum };
	//meshs.resize(h.meshNum);
	//for (std::uint32_t i = 0u; i < h.meshNum; i++)
	//{
	//	std::uint32_t size = 0u;
	//	file.read(reinterpret_cast<char*>(&size), sizeof(size));
	//	meshs[i].vertex.resize(size);
	//	file.read(reinterpret_cast<char*>(meshs[i].vertex.data()), h.vertexSize * size);

	//	file.read(reinterpret_cast<char*>(&size), sizeof(size));
	//	meshs[i].index.resize(size);
	//	file.read(reinterpret_cast<char*>(meshs[i].index.data()), h.indexSize * size);

	//	file.read(reinterpret_cast<char*>(&size), sizeof(size));
	//	std::string matName;
	//	matName.resize(size);
	//	file.read(matName.data(), sizeof(matName[0]) * matName.size());
	//}
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
	//mesh.indexView.reset(graphics.CreateIndexView(sizeof(mesh.index[0]) * mesh.index.size(), Eugene::Format::R16_UINT, *mesh.indexBuffer));
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
	std::vector<Mesh> mlist;
	LoadSkeltalGltf("Swat.gltf");
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