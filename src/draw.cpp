#ifdef _DEBUG

#include "draw.h"

#include "skse64_common/BranchTrampoline.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <winrt/base.h>

#include "skse64/GameReferences.h"


namespace Draw {
    RelocPtr<RendererShadowState> g_rendererShadowState(0x3180DB0);

    RelocPtr<ID3D11Buffer *[]> g_currentVSConstantBuffers(0x3181640);
    RelocPtr<ID3D11Buffer *[]> g_currentPSConstantBuffers(0x31816A0);

    RelocPtr<RenderGlobals> g_renderGlobals(0x317E790);

    typedef void(*_XMMATRIXFromNiTransform)(XMMATRIX *out, const NiTransform *transform, int eyeIndex);
    RelocAddr<_XMMATRIXFromNiTransform> XMMATRIXFromNiTransform(0x1301660);


    ID3D11Buffer *g_cameraBuffer = nullptr;
    ID3D11Buffer *g_modelBuffer = nullptr;

    struct ShapeBuffers
    {
        Microsoft::WRL::ComPtr<ID3D11Buffer> vertexBuffer;
        Microsoft::WRL::ComPtr<ID3D11Buffer> indexBuffer;
        UINT numIndices = 0;
    };

    struct Vertex
    {
        NiPoint3 pos;
    };

    ShapeBuffers CreateVertexAndIndexBuffers(std::vector<Vertex> &vertices, std::vector<WORD> &indices)
    {
        ShapeBuffers buffers{};

        D3D11_BUFFER_DESC vertexBufferDesc;
        vertexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
        vertexBufferDesc.ByteWidth = sizeof(Vertex) * vertices.size();
        vertexBufferDesc.BindFlags = D3D11_BIND_VERTEX_BUFFER;
        vertexBufferDesc.CPUAccessFlags = 0;
        vertexBufferDesc.MiscFlags = 0;
        vertexBufferDesc.StructureByteStride = 0;

        D3D11_SUBRESOURCE_DATA vertexData;
        vertexData.pSysMem = vertices.data();
        vertexData.SysMemPitch = 0;
        vertexData.SysMemSlicePitch = 0;

        HRESULT result = g_renderGlobals->device->CreateBuffer(&vertexBufferDesc, &vertexData, buffers.vertexBuffer.GetAddressOf());
        if (FAILED(result)) {
            _ERROR("Failed to create vertex buffer");
            return buffers;
        }

        D3D11_BUFFER_DESC indexBufferDesc;
        indexBufferDesc.Usage = D3D11_USAGE_DEFAULT;
        indexBufferDesc.ByteWidth = sizeof(WORD) * indices.size();
        indexBufferDesc.BindFlags = D3D11_BIND_INDEX_BUFFER;
        indexBufferDesc.CPUAccessFlags = 0;
        indexBufferDesc.MiscFlags = 0;
        indexBufferDesc.StructureByteStride = 0;

        D3D11_SUBRESOURCE_DATA indexData;
        indexData.pSysMem = indices.data();
        indexData.SysMemPitch = 0;
        indexData.SysMemSlicePitch = 0;

        result = g_renderGlobals->device->CreateBuffer(&indexBufferDesc, &indexData, buffers.indexBuffer.GetAddressOf());
        if (FAILED(result)) {
            _ERROR("Failed to create index buffer");
            return buffers;
        }

        buffers.numIndices = indices.size();

        return buffers;
    }

    struct PerFrameVSData
    {
        XMMATRIX matProjView[2];
    };

    struct PerObjectVSData
    {
        XMMATRIX matModel[2];
        NiColorA color;
    };

    void CreateConstantBuffers()
    {
        // Create camera buffer
        D3D11_BUFFER_DESC cameraBufferDesc;
        cameraBufferDesc.Usage = D3D11_USAGE_DYNAMIC;
        cameraBufferDesc.ByteWidth = sizeof(PerFrameVSData);
        cameraBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        cameraBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        cameraBufferDesc.MiscFlags = 0;
        cameraBufferDesc.StructureByteStride = 0;

        HRESULT result = g_renderGlobals->device->CreateBuffer(&cameraBufferDesc, nullptr, &g_cameraBuffer);
        if (FAILED(result)) {
            _ERROR("Failed to create camera buffer");
        }

        // Create model buffer
        D3D11_BUFFER_DESC modelBufferDesc;
        modelBufferDesc.Usage = D3D11_USAGE_DYNAMIC;
        modelBufferDesc.ByteWidth = sizeof(PerObjectVSData);
        modelBufferDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
        modelBufferDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
        modelBufferDesc.MiscFlags = 0;
        modelBufferDesc.StructureByteStride = 0;

        result = g_renderGlobals->device->CreateBuffer(&modelBufferDesc, nullptr, &g_modelBuffer);
        if (FAILED(result)) {
            _ERROR("Failed to create model buffer");
        }
    }

    ID3D11RasterizerState *g_rasterizerState = nullptr;

    void CreateRasterizerState(bool wireframe)
    {
        D3D11_RASTERIZER_DESC desc;
        if (wireframe) {
            desc.FillMode = D3D11_FILL_WIREFRAME;
            desc.CullMode = D3D11_CULL_NONE;
        }
        else {
            desc.FillMode = D3D11_FILL_SOLID;
            desc.CullMode = D3D11_CULL_BACK;
        }
        desc.FrontCounterClockwise = true;
        desc.DepthBias = 0.f;
        desc.DepthBiasClamp = -100.f;
        desc.SlopeScaledDepthBias = 0.f;
        desc.DepthClipEnable = true;
        desc.ScissorEnable = false;
        desc.MultisampleEnable = false;
        desc.AntialiasedLineEnable = false;
        HRESULT result = g_renderGlobals->device->CreateRasterizerState(&desc, &g_rasterizerState);
        if (FAILED(result)) {
            _ERROR("Failed to create rasterizer state");
        }
    }

    ID3D11InputLayout *g_inputLayout = nullptr;
    ID3D11VertexShader *g_vertexShader = nullptr;
    ID3D11PixelShader *g_pixelShader = nullptr;

    constexpr const auto vertexShaderSource = R"(
struct VS_INPUT {
    float3 vPos : POS;
    uint instanceId: SV_InstanceID0;
};

struct VS_OUTPUT {
    float4 vPos : SV_POSITION;
    float4 vColor : COLOR0;
    float clipDistance : SV_ClipDistance;
    float cullDistance : SV_CullDistance;
};

cbuffer Camera : register(b0) {
    float4x4 matProjView[2];
};

cbuffer Model : register(b1) {
    float4x4 matModel[2];
    float4 color;
};

VS_OUTPUT main(VS_INPUT input) {
    const float4 eyeClipEdge[2] = { { -1, 0, 0, 1 }, { 1, 0, 0, 1 } };
    const float eyeOffsetScale[2] = { -0.5, 0.5 };

    float4 pos = float4(input.vPos.xyz, 1.0f);
    pos = mul(matModel[input.instanceId], pos);
    pos = mul(matProjView[input.instanceId], pos);

    VS_OUTPUT output;
    output.vColor = color;
    output.clipDistance = mul(pos, eyeClipEdge[input.instanceId]);
    output.cullDistance = output.clipDistance;

    pos.x *= 0.5;
    pos.x += eyeOffsetScale[input.instanceId] * pos.w;
    output.vPos = pos;

    return output;
}
)";

    constexpr const auto pixelShaderSource = R"(
struct PS_INPUT {
    float4 pos : SV_POSITION;
    float4 color : COLOR0;
};

struct PS_OUTPUT {
    float4 color : SV_Target;
};

PS_OUTPUT main(PS_INPUT input) {
    PS_OUTPUT output;
    output.color = input.color;
    return output;
}
)";

    void CreateShaders()
    {
        // Create vertex shader

        UINT compileFlags = D3DCOMPILE_ENABLE_STRICTNESS | D3DCOMPILE_PACK_MATRIX_COLUMN_MAJOR;
        winrt::com_ptr<ID3DBlob> errorBlob;
        winrt::com_ptr<ID3DBlob> vertexShaderBinary;

        HRESULT result = D3DCompile(
            vertexShaderSource,
            strlen(vertexShaderSource),
            nullptr,
            nullptr,
            nullptr,
            "main",
            "vs_5_0",
            compileFlags,
            0,
            vertexShaderBinary.put(),
            errorBlob.put()
        );

        if (FAILED(result)) {
            _ERROR("Vertex shader failed to compile");
            if (errorBlob) {
                _ERROR(static_cast<LPCSTR>(errorBlob->GetBufferPointer()));
            }
        }

        result = g_renderGlobals->device->CreateVertexShader(vertexShaderBinary->GetBufferPointer(), vertexShaderBinary->GetBufferSize(), nullptr, &g_vertexShader);
        if (FAILED(result)) {
            _ERROR("Failed to create vertex shader");
        }

        // Create pixel shader

        winrt::com_ptr<ID3DBlob> pixelShaderBinary;

        result = D3DCompile(
            pixelShaderSource,
            strlen(pixelShaderSource),
            nullptr,
            nullptr,
            nullptr,
            "main",
            "ps_5_0",
            compileFlags,
            0,
            pixelShaderBinary.put(),
            errorBlob.put()
        );

        if (FAILED(result)) {
            if (errorBlob) {
                _ERROR(static_cast<LPCSTR>(errorBlob->GetBufferPointer()));
            }
        }

        result = g_renderGlobals->device->CreatePixelShader(pixelShaderBinary->GetBufferPointer(), pixelShaderBinary->GetBufferSize(), nullptr, &g_pixelShader);
        if (FAILED(result)) {
            _ERROR("Failed to create pixel shader");
        }

        // Create input layout
        std::vector<D3D11_INPUT_ELEMENT_DESC> layout = {
            { "POS", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 }
        };

        result = g_renderGlobals->device->CreateInputLayout(
            layout.data(), layout.size(),
            vertexShaderBinary->GetBufferPointer(), vertexShaderBinary->GetBufferSize(),
            &g_inputLayout
        );
        if (FAILED(result)) {
            _ERROR("Failed to create input layout");
        }
    }

    void SetVSConstantBuffer(UInt32 startSlot, ID3D11Buffer *buffer)
    {
        // We do this so that the game properly sets the constant buffer next time it checks if it needs to
        ID3D11Buffer *&currentBuffer = (*g_currentVSConstantBuffers)[startSlot];
        if (buffer != currentBuffer) {
            g_renderGlobals->deviceContext->VSSetConstantBuffers(startSlot, 1, &buffer);
            currentBuffer = buffer;
        }
    }

    void SetPSConstantBuffer(UInt32 startSlot, ID3D11Buffer *buffer)
    {
        ID3D11Buffer *&currentBuffer = (*g_currentPSConstantBuffers)[startSlot];
        if (buffer != currentBuffer) {
            g_renderGlobals->deviceContext->PSSetConstantBuffers(startSlot, 1, &buffer);
            currentBuffer = buffer;
        }
    }

    std::pair<std::vector<Vertex>, std::vector<WORD>> GetSphereVertices(float radius)
    {
        std::vector<Vertex> vertices;
        std::vector<WORD> indices;

        const int numSegments = 12;
        int numRings = numSegments;

        for (int ring = 0; ring <= numRings; ring++) {
            const float v = ring / (float)numRings;
            const float theta = v * M_PI;

            for (int segment = 0; segment <= numSegments; segment++) {
                const float u = segment / (float)numSegments;
                const float phi = u * M_PI * 2;

                const float x = cos(phi) * sin(theta);
                const float y = cos(theta);
                const float z = sin(phi) * sin(theta);

                Vertex vertex;
                vertex.pos = { x * radius, y * radius, z * radius };
                vertices.push_back(vertex);
            }
        }

        for (int ring = 0; ring < numRings; ring++) {
            for (int segment = 0; segment < numSegments; segment++) {
                const int index = ring * (numSegments + 1) + segment;

                indices.push_back(index);
                indices.push_back(index + numSegments + 1);
                indices.push_back(index + 1);

                indices.push_back(index + 1);
                indices.push_back(index + numSegments + 1);
                indices.push_back(index + numSegments + 2);
            }
        }

        return { vertices, indices };
    }


    void DrawThing(const ShapeBuffers &buffers, const NiTransform &transform, const NiColorA &color)
    {
        UINT stride = sizeof(Vertex);
        UINT offset = 0;
        g_renderGlobals->deviceContext->IASetVertexBuffers(0, 1, buffers.vertexBuffer.GetAddressOf(), &stride, &offset);
        g_renderGlobals->deviceContext->IASetIndexBuffer(buffers.indexBuffer.Get(), DXGI_FORMAT_R16_UINT, 0);

        { // Model data (object transform)

            // Each eye
            PerObjectVSData modelData;
            XMMATRIXFromNiTransform(&modelData.matModel[0], &transform, 0);
            XMMATRIXFromNiTransform(&modelData.matModel[1], &transform, 1);
            modelData.color = color;

            D3D11_MAPPED_SUBRESOURCE mappedResource;
            g_renderGlobals->deviceContext->Map(g_modelBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
            memcpy(mappedResource.pData, &modelData, sizeof(PerObjectVSData));
            g_renderGlobals->deviceContext->Unmap(g_modelBuffer, 0);

            SetVSConstantBuffer(1, g_modelBuffer);
        }

        g_renderGlobals->deviceContext->DrawIndexedInstanced(buffers.numIndices, 2, 0, 0, 0);
    }

    ShapeBuffers g_sphereBuffers;
    void DrawSphere(const NiTransform &transform, const NiColorA &color)
    {
        DrawThing(g_sphereBuffers, transform, color);
    }

    bool g_drawInitialized = false;
    void StartDraw()
    {
        if (!g_drawInitialized) {
            CreateShaders();
            CreateConstantBuffers();
            CreateRasterizerState(true);

            auto [vertices, indices] = GetSphereVertices(1.f);
            g_sphereBuffers = CreateVertexAndIndexBuffers(vertices, indices);

            g_drawInitialized = true;
        }

        g_renderGlobals->deviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

        g_renderGlobals->deviceContext->RSSetState(g_rasterizerState);

        g_renderGlobals->deviceContext->IASetInputLayout(g_inputLayout);

        g_renderGlobals->deviceContext->VSSetShader(g_vertexShader, nullptr, 0);
        g_renderGlobals->deviceContext->PSSetShader(g_pixelShader, nullptr, 0);

        { // Camera data (eye transforms)
            PerFrameVSData cameraData;
            cameraData.matProjView[0] = g_rendererShadowState->m_CameraData[0].m_ViewProjMat;
            cameraData.matProjView[1] = g_rendererShadowState->m_CameraData[1].m_ViewProjMat;

            D3D11_MAPPED_SUBRESOURCE mappedResource;
            g_renderGlobals->deviceContext->Map(g_cameraBuffer, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
            memcpy(mappedResource.pData, &cameraData, sizeof(PerFrameVSData));
            g_renderGlobals->deviceContext->Unmap(g_cameraBuffer, 0);

            SetVSConstantBuffer(0, g_cameraBuffer);
        }
    }
}

#endif // _DEBUG