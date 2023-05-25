#include <iostream>
#include <string>
#include <memory>
#include <deque>
#include <fstream>
#include <filesystem>
#include <EugeneLib.h>
#include <Common/Debug.h>
#include <unordered_map>

//#include <fbxsdk.h>

#include "EugeneLib/Include/Graphics/IndexView.h"

//#include <DirectXMath.h>

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
	float weight[4]{0.0f,0.0f,0.0f,0.0f};
};

struct Bone
{
	std::string name_;
	int parent_ = -1;
	Eugene::Vector3 offset_;
	Eugene::Quaternion q_;
	Eugene::Matrix4x4 transform_;
	Eugene::Matrix4x4 inverseMatrix;
	std::vector<int> children;
	std::uint32_t index;
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



using Vector3AnimationData = std::vector<std::pair<float, Eugene::Vector3>>;
using RotationAnimationData = std::vector<std::pair<float, Eugene::Quaternion>>;


DirectX::XMMATRIX ConvertYToZFront(const DirectX::XMMATRIX& yRotationMatrix)
{
	// Y軸周りの回転行列からY軸の回転角度を取得
	DirectX::XMFLOAT4X4 yMatrix;
	DirectX::XMStoreFloat4x4(&yMatrix, yRotationMatrix);

	float yRotationAngle = atan2f(yMatrix._13, yMatrix._33);

	// Z軸周りの回転角度を計算
	float zRotationAngle = DirectX::XM_PIDIV2 - yRotationAngle;

	// Z軸周りの回転行列を作成
	DirectX::XMMATRIX zRotationMatrix = DirectX::XMMatrixRotationZ(zRotationAngle);

	// Y軸周りの回転行列とZ軸周りの回転行列を順番に掛け合わせて返す
	return DirectX::XMMatrixMultiply(yRotationMatrix, zRotationMatrix);
}

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
		// 名前を書き込む
		std::uint32_t size = bone.name_.size();
		file.write(reinterpret_cast<char*>(&size), sizeof(size));
		file.write(reinterpret_cast<char*>(bone.name_.data()), sizeof(bone.name_[0]) * size);

		// 親Indexを書き込む
		file.write(reinterpret_cast<char*>(&bone.parent_),sizeof(bone.parent_));

		file.write(reinterpret_cast<char*>(&bone.offset_), sizeof(bone.offset_));

		file.write(reinterpret_cast<char*>(&bone.q_), sizeof(bone.q_));

		file.write(reinterpret_cast<char*>(&bone.transform_), sizeof(bone.transform_));

		file.write(reinterpret_cast<char*>(&bone.inverseMatrix), sizeof(bone.inverseMatrix));


		// 子indexを書き込む
		size = static_cast<std::uint32_t>(bone.children.size());
		file.write(reinterpret_cast<char*>(&size), sizeof(size));
		if (size != 0u)
		{
			file.write(reinterpret_cast<char*>(bone.children.data()), sizeof(bone.children[0]) * size);
		}
	}

}

void ExportImage(tinygltf::Model& model)
{
	for (auto& img : model.images)
	{
		stbi_write_png((img.name + ".png").c_str(), img.width, img.height, img.component, img.image.data(), 0);
	}
}

void LoadGltf(const std::string& path)
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
			//list.emplace_back(std::move(mesh));
		}
	}

	
	
	for (auto& material : model.materials)
	{
		ExportMaterial("./" + material.name + ".mat", material, model,"StaticMesh");
	}
	ExportImage(model);
}

void LoadBone(tinygltf::Model& model,int idx, std::vector<Bone>& bones, std::unordered_map<std::string, int>& nameTbl)
{
	//bones[idx].children.resize(model.nodes[idx].children.size());
	auto nodeIdx = model.skins[0].joints[idx];
	bones[idx].name_ = model.nodes[nodeIdx].name;

	/*bones[idx].offset_ = pos;
	bones[idx].q_ = q;*/
	for (auto& child: model.nodes[nodeIdx].children)
	{
		auto tmp = model.nodes[child].name;
		if (nameTbl.contains(tmp))
		{
			auto itr = std::find(bones[idx].children.begin(), bones[idx].children.end(), nameTbl[model.nodes[child].name]);
			if (itr == bones[idx].children.end())
			{
				bones[idx].children.push_back(nameTbl[model.nodes[child].name]);
				bones[nameTbl[model.nodes[child].name]].parent_ = idx;
				LoadBone(model, nameTbl[tmp], bones, nameTbl);
			}
		}
	}
}

void LoadTestGltf(const std::string& path)
{
	tinygltf::TinyGLTF gltf;
	tinygltf::Model model;
	std::string err;
	std::string warn;
	gltf.LoadASCIIFromFile(&model, &err, &warn, path);

	struct Vertex
	{
		Eugene::Vector3 pos;
		Eugene::Vector3 normal;
		Eugene::Vector2 uv;
	};

	std::vector<Vertex> vert;
	std::vector<std::uint16_t> index;

	std::ofstream file{ path.substr(0,path.find_last_of(".") - 1) + ".mesh", std::ios::binary };

	for (auto& m : model.meshes)
	{
		for (int p = 0; p < m.primitives.size(); p++)
		{
			auto& primitive = m.primitives[p];
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

			vert.resize(posAccsessor.count);
			for (int i = 0; i < posAccsessor.count; i++)
			{
				vert[i].pos = Eugene::Vector3{ -pos[i * 3 + 0] ,pos[i * 3 + 1], pos[i * 3 + 2] };
				vert[i].normal = Eugene::Vector3{ -norm[i * 3 + 0],norm[i * 3 + 1],norm[i * 3 + 2] };
				vert[i].uv = Eugene::Vector2{ uv[i * 2 + 0],uv[i * 2 + 1] };
			}

			auto& accessor = model.accessors[primitive.indices];
			auto& bufferView = model.bufferViews[accessor.bufferView];
			auto& buffer = model.buffers[bufferView.buffer];
			auto idxP = reinterpret_cast<std::uint16_t*>(&buffer.data[bufferView.byteOffset + accessor.byteOffset]);
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

			int vertexNum = vert.size();
			int indexNum = index.size();
			file.write(reinterpret_cast<char*>(&vertexNum), sizeof(vertexNum));
			file.write(reinterpret_cast<char*>(vert.data()), sizeof(Vertex)* vertexNum);
			file.write(reinterpret_cast<char*>(&indexNum), sizeof(indexNum));
			file.write(reinterpret_cast<char*>(index.data()), sizeof(std::uint16_t)* indexNum);
		}
	}
}

void SetOffset(Bone& b, const Eugene::Vector3& offset,std::vector<Bone>& bones)
{
	b.offset_ = offset + b.offset_;
	for (int i = 0; i < b.children.size(); i++)
	{
		SetOffset(bones[b.children[i]], b.offset_, bones);
	}
}

void SetLoacalTransformMatrix(Bone& b, std::vector<Bone>& bones)
{
	DirectX::XMMATRIX parentMatrix = DirectX::XMMatrixIdentity();
	DirectX::XMMATRIX worldMatrix = DirectX::XMMatrixInverse(nullptr, DirectX::XMLoadFloat4x4(&b.inverseMatrix));
	Eugene::Vector3 diff = b.offset_;
	Eugene::Vector3 localOffset = b.offset_;
	Eugene::Vector3 localRot = b.q_.ToEuler();

	if (b.parent_ != -1)
	{
		parentMatrix = DirectX::XMLoadFloat4x4(&bones[b.parent_].inverseMatrix);
		diff = b.offset_ - bones[b.parent_].offset_;
		localOffset = b.offset_ - bones[b.parent_].offset_;
		localRot = b.q_.ToEuler() - bones[b.parent_].q_.ToEuler();
		DirectX::XMStoreFloat4x4(&b.transform_, worldMatrix * parentMatrix);
	}
	else
	{
		DirectX::XMStoreFloat4x4( &b.transform_, worldMatrix);
	}
	
	



	/*DirectX::XMStoreFloat4x4(&b.transform_, 
		DirectX::XMMatrixRotationRollPitchYaw(localRot.x, localRot.y, localRot.z) * DirectX::XMMatrixTranslation(localOffset.x, localOffset.y, localOffset.z));*/


	for (int i = 0; i < b.children.size(); i++)
	{
		SetLoacalTransformMatrix(bones[b.children[i]], bones);
	}

}

void SetInverseBindMatrix(Bone& b, Eugene::Matrix4x4& matrix, std::vector<Bone>& bones)
{
	Eugene::Matrix4x4 worldMatrix;
	Eugene::Mul(worldMatrix, matrix, b.transform_);
	b.inverseMatrix = worldMatrix;
	Eugene::Inverse(b.inverseMatrix);

	for (int i = 0; i < b.children.size(); i++)
	{
		SetInverseBindMatrix(bones[b.children[i]], worldMatrix, bones);
	}
}

void ExportAnimationData(std::vector<std::tuple<Vector3AnimationData, RotationAnimationData, Vector3AnimationData>>& animdata,
	const std::filesystem::path& path)
{
	std::ofstream file{ path,std::ios::binary };
	struct AnimHeader
	{
		char sig[4]{ 's','a','n','i' };
		std::uint32_t version = 1;
	};

	AnimHeader h{};
	file.write(reinterpret_cast<char*>(&h), sizeof(h));

	std::uint32_t size = static_cast<std::uint32_t>(animdata.size());
	file.write(reinterpret_cast<char*>(&size), sizeof(size));
	
	for (auto& anim : animdata)
	{
		auto& move = std::get<0>(anim);
		size = static_cast<std::uint32_t>(move.size());
		file.write(reinterpret_cast<char*>(&size), sizeof(size));
		file.write(reinterpret_cast<char*>(move.data()), size* (sizeof(float) + sizeof(move[0].second)));

		auto& rot = std::get<1>(anim);
		size = static_cast<std::uint32_t>(rot.size());
		file.write(reinterpret_cast<char*>(&size), sizeof(size));
		file.write(reinterpret_cast<char*>(rot.data()), size * (sizeof(float) + sizeof(rot[0].second)));

		auto& scale = std::get<2>(anim);
		size = static_cast<std::uint32_t>(scale.size());
		file.write(reinterpret_cast<char*>(&size), sizeof(size));
		file.write(reinterpret_cast<char*>(scale.data()), size* (sizeof(float) + sizeof(scale[0].second)));
	}

}

struct Motion
{
	float time;
	Eugene::Vector3 location;
	Eugene::Quaternion quaternion;
};

void ExportMotion(const std::filesystem::path& path,
	std::map<std::string, std::list<Motion>>& motiondata, 
	std::unordered_map<std::string, std::string>& boneNameTbl)
{
	std::ofstream file{ path,std::ios::binary };
	struct AnimHeader
	{
		char sig[4]{ 's','a','n','i' };
		std::uint32_t version = 1;
	};

	AnimHeader h{};
	file.write(reinterpret_cast<char*>(&h), sizeof(h));
	
	std::vector<std::vector<Motion>> exportData(boneNameTbl.size());
	
	std::uint32_t boneNum = motiondata.size();
	file.write(reinterpret_cast<char*>(&boneNum), sizeof(boneNum));

	std::uint32_t nameSize{ 0u };
	for (auto& motion : motiondata)
	{
		nameSize = boneNameTbl[motion.first].size();
		file.write(reinterpret_cast<char*>(&nameSize), sizeof(nameSize));
		file.write(boneNameTbl[motion.first].data(),  nameSize);

		std::uint32_t num = static_cast<std::uint32_t>(motion.second.size());
		file.write(reinterpret_cast<char*>(&num), sizeof(num));
		std::vector<Motion> tmpData(num);
		std::copy(motion.second.begin(), motion.second.end(), tmpData.data());
		file.write(reinterpret_cast<char*>(tmpData.data()), sizeof(tmpData[0]) * num);

		//exportData[boneNameTbl[motion.first]].resize(motion.second.size());
		//std::copy(motion.second.begin(), motion.second.end(), exportData[boneNameTbl[motion.first]].data());
	}

	/*for (int i = 0; i < exportData.size(); i++)
	{
		if (exportData[i].size() <= 0)
		{
			continue;
		}
		auto boneIdx = static_cast<std::uint32_t>(i);
		file.write(reinterpret_cast<char*>(&boneIdx), sizeof(boneIdx));
		auto num = static_cast<std::uint32_t>(exportData[i].size());
		file.write(reinterpret_cast<char*>(&num), sizeof(num));
		file.write(reinterpret_cast<char*>(exportData[i].data()), exportData[i].size() * sizeof(exportData[i][0]));
	}*/


}



void LoadVmdFile(const std::filesystem::path& path, std::unordered_map<std::string, int>& boneNameTbl)
{
	std::unordered_map<std::string, std::string> vmdBoneNameTbl(boneNameTbl.size());

	for (auto& boneName : boneNameTbl)
	{
		auto name = boneName.first.substr(boneName.first.find_first_of(":") + 1);
		auto l = name.find("Left");
		if (l  < name.size())
		{
			vmdBoneNameTbl["L_" + name.substr(l + 4)] = boneName.first;
			continue;
		}

		auto r = name.find("Right");
		if (r < name.size())
		{
			vmdBoneNameTbl["R_" + name.substr(r + 5)] = boneName.first;
			continue;
		}

		vmdBoneNameTbl[name] = boneName.first;
	}


	std::ifstream file{ path, std::ios::binary };
#pragma pack(1)
	struct VMDMotion
	{
		VMDMotion()
		{
			std::fill(std::begin(boneName), std::end(boneName), 0);
		}
		char boneName[15];		// ボーン名
		unsigned int frameNo;	// フレーム番号
		Eugene::Vector3 location;		// 位置
		Eugene::Quaternion quaternion;	// クオータニオン(回転)
		unsigned char bezier[64];		// [4][4][4]ベジェ補完パラメーター
	};
#pragma pack()

	file.ignore(50);
	unsigned int motionDataNum{ 0 };
	file.read(reinterpret_cast<char*>(&motionDataNum), sizeof(motionDataNum));

	std::vector<VMDMotion> vmdMotionData(motionDataNum);
	std::vector<std::string> nameVec(vmdBoneNameTbl.size());
	for (auto& boneN : vmdBoneNameTbl)
	{
		nameVec.emplace_back(boneN.first);
	}
	std::sort(nameVec.begin(), nameVec.end(), [](auto& a, auto& b) {return a.size() > b.size(); });

	for (auto& vmd : vmdMotionData)
	{
		file.read(reinterpret_cast<char*>(&vmd.boneName), sizeof(char) * 15);
		file.read(reinterpret_cast<char*>(&vmd.frameNo), sizeof(VMDMotion) - (sizeof(char) * 15));
	}

	auto  checkName = [&nameVec](char name[15])
	{
		bool flag = true;
		std::string tmp = name;

		for (auto& boneN : nameVec)
		{
			if (tmp.starts_with(boneN))
			{
				return boneN;
			}
		}
		return std::string{};
	};

	std::map<std::string,std::list<Motion>> motionData;
	for (auto& vmd : vmdMotionData)
	{
		auto tmp = std::string(vmd.boneName);
		auto t = static_cast<float>( 1.0 / 30.0 * static_cast<double>(vmd.frameNo));
		//DebugLog("Name={}", vmd.boneName);
		motionData[tmp].emplace_back
		(
			Motion{ t, vmd.location / (1250.0f), vmd.quaternion }
		);
	}

	for (auto& motion : motionData)
	{
		if ((*motion.second.begin()).time > 0.0f)
		{
			(*motion.second.begin()).time = 0.0f;
		}
	}

	/*for (auto& m : motionData)
	{
		DebugLog("Bone={}", m.first);
		for (auto& motion : m.second)
		{
			
		}
	}*/


	ExportMotion(path.string().substr(0,path.string().find_last_of(".")) + ".sani", motionData, vmdBoneNameTbl);
}



void LoadSkeltalGltf(const std::string& path)
{
	
	tinygltf::TinyGLTF gltf;
	tinygltf::Model model;
	std::string err;
	std::string warn;
	gltf.LoadASCIIFromFile(&model, &err, &warn, path);
	std::vector<Bone> bones;
	std::unordered_map<std::string, int> nameTbl;
	for (auto& skin : model.skins)
	{
		DebugLog(skin.name);
		
		
		bones.resize(skin.joints.size());
		std::vector<Eugene::Matrix4x4> matrixs(skin.joints.size());
		nameTbl.reserve(skin.joints.size());
		for (int i = 0; i < skin.joints.size(); i++)
		{
			auto tmp = model.nodes[skin.joints[i]].name;
			nameTbl.emplace(tmp, i);
		}

		for (auto& joint : skin.joints)
		{
			auto tmp = model.nodes[joint].name;
			LoadBone(model, nameTbl[tmp], bones, nameTbl);
		}

		auto& accsessor = model.accessors[skin.inverseBindMatrices];
		auto& bufferView = model.bufferViews[accsessor.bufferView];
		auto& buffer = model.buffers[bufferView.buffer];
		auto p = reinterpret_cast<Eugene::Matrix4x4*>(&buffer.data[accsessor.byteOffset + bufferView.byteOffset]);
		
		std::vector<Eugene::Matrix4x4> inverseMat_(accsessor.count);
		std::copy_n(p, accsessor.count, inverseMat_.data());


		
		for (int i = 0; i < skin.joints.size(); i++)
		{
			DirectX::XMVECTOR scale, trans, qrot;
			auto result = DirectX::XMMatrixDecompose(&scale, &qrot, &trans, DirectX::XMMatrixInverse(nullptr, DirectX::XMLoadFloat4x4(&inverseMat_[i]))); 
			
			// 座標系を左手座標系に
			trans.m128_f32[0]  = -trans.m128_f32[0];


			bones[i].offset_ = { trans.m128_f32[0] , trans.m128_f32[1] , trans.m128_f32[2] };
			if (!result)
			{
				throw std::exception{};
			}
			DirectX::XMFLOAT4 q;

			
			//auto rotX = DirectX::XMQuaternionRotationAxis(DirectX::XMVectorSet(1.0f, 0.0f, 0.0f, 0.0f), Eugene::Deg2Rad(-90.0f));
			//qrot = DirectX::XMQuaternionMultiply(qrot,rotX);
			

			auto frontVec = DirectX::XMVector3Rotate(DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f), qrot);
			//qrot = DirectX::XMQuaternionRotationAxis(frontVec, -angle);
			auto lookMatrix = DirectX::XMMatrixLookToRH(DirectX::XMVectorZero(), DirectX::XMVector3Normalize(frontVec), DirectX::XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
			DirectX::XMMatrixDecompose(&scale, &qrot, &trans, lookMatrix);

			//DirectX::XMMatrixDecompose(&scale,&qrot,&trans, ConvertYToZFront(DirectX::XMMatrixRotationQuaternion(qrot)));
			
			// 右手座標系→左手座標系に
			DirectX::XMVECTOR axis;
			float angle;
			DirectX::XMQuaternionToAxisAngle(&axis, &angle, qrot);
			axis.m128_f32[0] = -axis.m128_f32[0];
			qrot = DirectX::XMQuaternionRotationAxis(axis, -angle);


			DirectX::XMStoreFloat4(&q, (qrot));
			bones[i].q_ = { q.x,q.y, q.z, q.w };
			

			// ボーンをワールド座標上に変換する行列
			auto worldMatrix = DirectX::XMMatrixRotationQuaternion(qrot)* DirectX::XMMatrixTranslation(bones[i].offset_.x, bones[i].offset_.y, bones[i].offset_.z);

			// ワールド座標にする行列の逆行列(逆バインド行列敵なの)にする
			DirectX::XMStoreFloat4x4(&bones[i].inverseMatrix, DirectX::XMMatrixInverse(nullptr,worldMatrix));
		}

		//SetOffset(bones[0],Eugene::zeroVector3<float>, bones);
	/*	for (auto& b : bones)
		{
			b.offset_ = b.offset_ / 100.0f;
		}*/

		//SetLoacalTransformMatrix(bones[0], bones);

		
		ExportBone(path.substr(0, path.find_last_of(".")) + ".bone", bones);
	}

	int count = 0;
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
			auto& weightsAccessor = model.accessors[primitive.attributes["WEIGHTS_0"]];

			auto& posBufferView = model.bufferViews[posAccsessor.bufferView];
			auto& normBufferView = model.bufferViews[normAccessor.bufferView];
			auto& tanBufferView = model.bufferViews[tanAccessor.bufferView];
			auto& uvBufferView = model.bufferViews[uvAccessor.bufferView];
			auto& boneBufferView = model.bufferViews[boneAccessor.bufferView];
			auto& weightsBufferView = model.bufferViews[weightsAccessor.bufferView];

			auto& posBuffer = model.buffers[posBufferView.buffer];
			auto& normBuffer = model.buffers[normBufferView.buffer];
			auto& tanBuffer = model.buffers[tanBufferView.buffer];
			auto& uvBuffer = model.buffers[uvBufferView.buffer];
			auto& boneBuffer = model.buffers[boneBufferView.buffer];
			auto& weighisBuffer = model.buffers[weightsBufferView.buffer];

			auto pos = reinterpret_cast<float*>(&posBuffer.data[posBufferView.byteOffset + posAccsessor.byteOffset]);
			auto norm = reinterpret_cast<float*>(&normBuffer.data[normBufferView.byteOffset + normAccessor.byteOffset]);
			auto tan = reinterpret_cast<float*>(&tanBuffer.data[tanBufferView.byteOffset + tanAccessor.byteOffset]);
			auto uv = reinterpret_cast<float*>(&uvBuffer.data[uvBufferView.byteOffset + uvAccessor.byteOffset]);
			auto joint = reinterpret_cast<std::uint8_t*>(&boneBuffer.data[boneBufferView.byteOffset + boneAccessor.byteOffset]);
			auto weight = reinterpret_cast<float*>(&weighisBuffer.data[weightsBufferView.byteOffset + weightsAccessor.byteOffset]);

			std::vector<SkeletalGltfVertex> vertex;
			vertex.resize(posAccsessor.count);
			for (int i = 0; i < posAccsessor.count; i++)
			{
				std::uint8_t j[4];
				vertex[i].pos = Eugene::Vector3{ -pos[i * 3 + 0] ,pos[i * 3 + 1], pos[i * 3 + 2] };
				vertex[i].normal = Eugene::Vector3{ -norm[i * 3 + 0],norm[i * 3 + 1],norm[i * 3 + 2] };
				vertex[i].tan = Eugene::Vector3{ -tan[i * 3 + 0],tan[i * 3 + 1],tan[i * 3 + 2] };
				vertex[i].uv = Eugene::Vector2{ uv[i * 2 + 0],uv[i * 2 + 1] };
				vertex[i].joint[0] = static_cast<std::uint16_t>(joint[i * 4 + 0]);
				vertex[i].joint[1] = static_cast<std::uint16_t>(joint[i * 4 + 1]);
				vertex[i].joint[2] = static_cast<std::uint16_t>(joint[i * 4 + 2]);
				vertex[i].joint[3] = static_cast<std::uint16_t>(joint[i * 4 + 3]);
				for (auto j : vertex[i].joint)
				{
					if (j == 0u)
					{
						count++;
					}
				}
				
				
				vertex[i].weight[0] = weight[i * 4 + 0];
				vertex[i].weight[1] = weight[i * 4 + 1];
				vertex[i].weight[2] = weight[i * 4 + 2];
				vertex[i].weight[3] = weight[i * 4 + 3];

				//DebugLog("bone_x={0:}y={1:}z={2:}w={3:}", vertex[i].joint[0], vertex[i].joint[1], vertex[i].joint[2], vertex[i].joint[3]);
				//DebugLog("weight_x={0:}y={1:}z={2:}w={3:}", vertex[i].weight.x, vertex[i].weight.y, vertex[i].weight.z,1.0f - (vertex[i].weight.x - vertex[i].weight.y - vertex[i].weight.z));
			/*	auto inverseVal = 1.0f / (bones[vertex[i].joint[0]].offset_ - vertex[i].pos).Magnitude();
				inverseVal += 1.0f / (bones[vertex[i].joint[1]].offset_ - vertex[i].pos).Magnitude();
				inverseVal += 1.0f / (bones[vertex[i].joint[2]].offset_ - vertex[i].pos).Magnitude();
				inverseVal += 1.0f / (bones[vertex[i].joint[3]].offset_ - vertex[i].pos).Magnitude();

				vertex[i].weight.x = (1.0f / (bones[vertex[i].joint[0]].offset_ - vertex[i].pos).Magnitude()) / inverseVal;
				vertex[i].weight.y = (1.0f / (bones[vertex[i].joint[1]].offset_ - vertex[i].pos).Magnitude()) / inverseVal;
				vertex[i].weight.z = (1.0f / (bones[vertex[i].joint[2]].offset_ - vertex[i].pos).Magnitude()) / inverseVal;*/
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

	ExportImage(model);

	LoadVmdFile("ZombieDie.vmd", nameTbl);
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
	//LoadSkeltalGltf("ZombieA.gltf");
	LoadGltf("Stage.gltf");
	//LoadSkeletalFbx("Swat.fbx");
	//LoadTestGltf("water.gltf");
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