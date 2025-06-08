#pragma once

#include <d3d11.h>
#include <d3d11shader.h>
#include <d3dcompiler.h>
#include <winrt/base.h>
#include <wrl/client.h>

#include "skse64_common/Relocation.h"
#include "skse64/NiGeometry.h"

namespace Draw {
    struct RenderGlobals
    {
        void *rendererData;					// RendererData *
        ID3D11Device *device; // 08
        HWND m_Window; // 10

        //
        // These are pools for efficient data uploads to the GPU. Each frame can use any buffer as long as there
        // is sufficient space. If there's no space left, delay execution until m_DynamicVertexBufferAvailQuery[] says a buffer
        // is no longer in use.
        //
        ID3D11Buffer *m_DynamicVertexBuffers[3];			// DYNAMIC (VERTEX | INDEX) CPU_ACCESS_WRITE
        UInt32			m_CurrentDynamicVertexBuffer;

        UInt32			m_CurrentDynamicVertexBufferOffset;	// Used in relation with m_DynamicVertexBufferAvailQuery[]
        ID3D11Buffer *m_SharedParticleIndexBuffer;		// DEFAULT INDEX CPU_ACCESS_NONE
        ID3D11Buffer *m_SharedParticleStaticBuffer;		// DEFAULT VERTEX CPU_ACCESS_NONE
        struct ID3D11InputLayout *m_ParticleShaderInputLayout;
        struct ID3D11InputLayout *m_UnknownInputLayout2;
        float oldClearColor[4]; // 58
        ID3D11DeviceContext *deviceContext; // 68
        UInt32 unk70;
        UInt32 unk74;
        struct ID3D11Query *queries[67]; // 78
        UInt32 unk290;
        UInt32 unk294;

        float m_DepthBiasFactors[3][4]; // 298
        UInt32 unk2C8;
        UInt32 unk2CC;

        struct ID3D11DepthStencilState *m_DepthStates[6][40];		// 2D0 - OMSetDepthStencilState
        struct ID3D11RasterizerState *m_RasterStates[2][3][13][2];	// A50 - RSSetState
        struct ID3D11BlendState *m_BlendStates[9][2][13][2];		// F30 - OMSetBlendState
        struct ID3D11SamplerState *m_SamplerStates[6][5];			// 1DD0 - Samplers[AddressMode][FilterMode] (Used for PS and CS)

        void *vertexShader1EC0; // 1EC0
        struct ID3D11InputLayout *inputLayoud1EC8; // 1EC8
        BSTriShape *hiddenAreaMesh; // 1ED0

        ID3D11Buffer *m_ConstantBuffers1[4];				// 1ED8 - Sizes: 3840 bytes
        ID3D11Buffer *m_AlphaTestRefCB;					// CONSTANT_GROUP_LEVEL_ALPHA_TEST_REF (Index 11) - 16 bytes
        ID3D11Buffer *m_ConstantBuffers2[20];			// Sizes: 0, 16, 32, 48, ... 304 bytes
        ID3D11Buffer *m_ConstantBuffers3[10];			// Sizes: 0, 16, 32, 48, ... 144 bytes
        ID3D11Buffer *m_ConstantBuffers4[42];			// Sizes: 0, 16, 32, 48, ... 432 bytes
        ID3D11Buffer *m_ConstantBuffers5[20];			// Sizes: 0, 16, 32, 48, ... 304 bytes
        ID3D11Buffer *m_ConstantBuffers6[20];			// Sizes: 0, 16, 32, 48, ... 304 bytes
        ID3D11Buffer *m_ConstantBuffers7[54];			// Sizes: 0, 16, 32, 48, ... 624 bytes
        ID3D11Buffer *m_TempConstantBuffer2;				// 576 bytes
        ID3D11Buffer *m_TempConstantBuffer3;				// CONSTANT_GROUP_LEVEL_PERFRAME (Index 12) - 720 bytes
        ID3D11Buffer *m_TempConstantBuffer4;				// 16 bytes

        struct IDXGIOutput *m_DXGIAdapterOutput; // 2448

        void *m_FrameDurationStringHandle;					// 2450 - "Frame Duration" but stored in their global string pool
    };

    typedef __m128 XMVECTOR;

    struct XMMATRIX
    {
        // 0-2 rotation
        // 3 position
        XMVECTOR r[4];
    };


    struct ViewData
    {
        XMVECTOR m_ViewUp; // 00
        XMVECTOR m_ViewRight; // 10
        XMVECTOR m_ViewForward; // 20
        XMMATRIX m_ViewMat; // 30
        XMMATRIX m_ProjMat; // 70
        XMMATRIX m_ViewProjMat; // B0
        XMMATRIX m_UnknownMat1; // F0 - all 0?
        XMMATRIX m_ViewProjMatrixUnjittered; // 130
        XMMATRIX m_PreviousViewProjMatrixUnjittered; // 170
        XMMATRIX m_ProjMatrixUnjittered; // 1B0
        XMMATRIX m_UnknownMat2; // 1F0 - all 0?
        float m_ViewPort[4];// 230 - NiRect<float> { left = 0, right = 1, top = 1, bottom = 0 }
        NiPoint2 m_ViewDepthRange; // 240
        char _pad0[0x8]; // 248
    }; // size == 250

    struct RendererShadowState
    {
        UInt32 m_StateUpdateFlags;						// Flags +0x0  0xFFFFFFFF; global state updates
        uint32_t m_PSResourceModifiedBits;					// Flags +0x4  0xFFFF
        uint32_t m_PSSamplerModifiedBits;					// Flags +0x8  0xFFFF
        uint32_t m_CSResourceModifiedBits;					// Flags +0xC  0xFFFF
        uint32_t m_CSSamplerModifiedBits;					// Flags +0x10 0xFFFF
        uint32_t m_CSUAVModifiedBits;						// Flags +0x14 0xFF
        uint32_t m_OMUAVModifiedBits;						// Flags +0x18 0xFF
        uint32_t m_SRVModifiedBits;						// Flags +0x1C 0xFF

        uint32_t m_RenderTargets[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT]; // 20
        uint32_t m_DepthStencil;							// 40 - Index
        uint32_t m_DepthStencilSlice;						// Index
        uint32_t m_CubeMapRenderTarget;						// 48 = Index
        uint32_t m_CubeMapRenderTargetView;					// Index

        UInt32 m_SetRenderTargetMode[D3D11_SIMULTANEOUS_RENDER_TARGET_COUNT]; // 50
        UInt32 m_SetDepthStencilMode; // 70
        UInt32 m_SetCubeMapRenderTargetMode;

        D3D11_VIEWPORT m_ViewPort; // 78

        UInt32 m_DepthStencilDepthMode;
        UInt32 m_DepthStencilDepthModePrevious; // 94 - also some kind of mode
        uint32_t m_DepthStencilStencilMode;
        uint32_t m_StencilRef;

        uint32_t m_RasterStateFillMode;
        uint32_t m_RasterStateCullMode;
        uint32_t m_RasterStateDepthBiasMode;
        uint32_t m_RasterStateScissorMode;

        uint32_t m_AlphaBlendMode;
        uint32_t m_AlphaBlendAlphaToCoverage;
        uint32_t m_AlphaBlendWriteMode;

        bool m_AlphaTestEnabled; // BC
        float m_AlphaTestRef; // C0

        uint32_t m_PSTextureAddressMode[16];
        uint32_t m_PSTextureFilterMode[16];
        ID3D11ShaderResourceView *m_PSTexture[16]; // 148

        uint32_t m_CSTextureAddressMode[16];
        uint32_t m_CSTextureFilterMode[16]; // 208

        ID3D11ShaderResourceView *m_CSTexture[16]; // 248
        uint32_t m_CSTextureMinLodMode[16]; // 2C8
        struct ID3D11UnorderedAccessView *m_CSUAV[8]; // 308

        UInt8 unk348[0x388 - 0x348]; // 348

        uint64_t m_VertexDesc; // 388
        void *m_CurrentVertexShader; // 390
        void *m_CurrentPixelShader; // 398
        D3D11_PRIMITIVE_TOPOLOGY m_Topology; // 3A0

        NiPoint3 m_PosAdjust[2]; // 3A4
        NiPoint3 m_PreviousPosAdjust[2]; // 3BC
        ViewData m_CameraData[2]; // 3E0 - size of each is 250

        uint32_t m_AlphaBlendModeExtra; // 880
        float unk884;
        float unk888;
        UInt32 unk88C;

        ID3D11Buffer *VSConstantBuffers[12]; // 890
        ID3D11Buffer *PSConstantBuffers[12]; // 8F0
    };


    extern RelocPtr<RendererShadowState> g_rendererShadowState;

    extern RelocPtr<ID3D11Buffer *[]> g_currentVSConstantBuffers;
    extern RelocPtr<ID3D11Buffer *[]> g_currentPSConstantBuffers;

    extern RelocPtr<RenderGlobals> g_renderGlobals;

    typedef void(*_XMMATRIXFromNiTransform)(XMMATRIX *out, const NiTransform *transform, int eyeIndex);
    extern RelocAddr<_XMMATRIXFromNiTransform> XMMATRIXFromNiTransform;


    void StartDraw();
    void DrawSphere(const NiTransform &transform, const NiColorA &color);
}
