#include "stdafx.h"
#include "RhinoVrHiddenAreaMeshDisplayConduit.h"

RhinoVrHiddenAreaMeshDisplayConduit::RhinoVrHiddenAreaMeshDisplayConduit()
  : CRhinoDisplayConduit(
    CSupportChannels::SC_PREDRAWOBJECTS)
  , m_draw_hidden_area_mesh(false)
  , m_hidden_area_mesh_left(nullptr)
  , m_hidden_area_mesh_right(nullptr)
  , m_hidden_area_mesh_left_xform(ON_Xform::IdentityTransformation)
  , m_hidden_area_mesh_right_xform(ON_Xform::IdentityTransformation)
  , m_hidden_area_mesh_left_cache_handle(nullptr)
  , m_hidden_area_mesh_right_cache_handle(nullptr)
{
}

bool RhinoVrHiddenAreaMeshDisplayConduit::ExecConduit(CRhinoDisplayPipeline& dp, UINT nActiveChannel, bool & bTerminateChannel)
{
  if (m_draw_hidden_area_mesh)
  {
    if (nActiveChannel == CSupportChannels::SC_PREDRAWOBJECTS)
    {
      if (m_pDisplayAttrs->m_bShadeSurface)
      {
        //auto vr_render_context = m_pDisplayAttrs->GetVrRenderContext();

        if (m_hidden_area_mesh_left)
        {
          dp.PushModelTransform(m_hidden_area_mesh_left_xform);
          dp.DrawMesh(*m_hidden_area_mesh_left, false, true, m_hidden_area_mesh_left_cache_handle);
          dp.PopModelTransform();
        }
        else if (m_hidden_area_mesh_right)
        {
          dp.PushModelTransform(m_hidden_area_mesh_right_xform);
          dp.DrawMesh(*m_hidden_area_mesh_right, false, true, m_hidden_area_mesh_right_cache_handle);
          dp.PopModelTransform();
        }
      }
    }
  }

  return true;
}

void RhinoVrHiddenAreaMeshDisplayConduit::Enable(unsigned int uiDocSerialNumber)
{
  CRhinoDisplayConduit::Enable(uiDocSerialNumber);
}

void RhinoVrHiddenAreaMeshDisplayConduit::SetHiddenAreaMesh(const ON_Mesh* hidden_area_mesh, vr::EVREye eye)
{
  if (eye == vr::Eye_Left)
  {
    m_hidden_area_mesh_left = hidden_area_mesh;
  }
  else
  {
    m_hidden_area_mesh_right = hidden_area_mesh;
  }

  if (m_hidden_area_mesh_left && m_hidden_area_mesh_right)
  {
    m_draw_hidden_area_mesh = true;
  }
}

void RhinoVrHiddenAreaMeshDisplayConduit::SetHiddenAreaMeshXform(const ON_Xform & hidden_area_mesh_xform, vr::EVREye eye)
{
  if (eye == vr::Eye_Left)
  {
    m_hidden_area_mesh_left_xform = hidden_area_mesh_xform;
  }
  else
  {
    m_hidden_area_mesh_right_xform = hidden_area_mesh_xform;
  }
}

void RhinoVrHiddenAreaMeshDisplayConduit::SetHiddenAreaMeshCacheHandle(CRhinoCacheHandle* cache_handle, vr::EVREye eye)
{
  if (eye == vr::Eye_Left)
  {
    m_hidden_area_mesh_left_cache_handle = cache_handle;
  }
  else
  {
    m_hidden_area_mesh_right_cache_handle = cache_handle;
  }
}

void RhinoVrHiddenAreaMeshDisplayConduit::Empty()
{
  m_draw_hidden_area_mesh = false;
  m_hidden_area_mesh_left = nullptr;
  m_hidden_area_mesh_right = nullptr;
}
