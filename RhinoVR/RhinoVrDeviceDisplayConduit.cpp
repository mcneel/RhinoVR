#include "StdAfx.h"
#include "RhinoVrDeviceDisplayConduit.h"

RhinoVrDeviceDisplayConduit::RhinoVrDeviceDisplayConduit()
  : CRhinoDisplayConduit(
    CSupportChannels::SC_CALCCLIPPINGPLANES |
    CSupportChannels::SC_CALCBOUNDINGBOX |
    CSupportChannels::SC_POSTDRAWOBJECTS)
  , m_draw_device_mesh(false)
  , m_device_mesh(nullptr)
  , m_device_material(nullptr)
  , m_device_mesh_xform(ON_Xform::IdentityTransformation)
  , m_device_cache_handle(nullptr)
  , m_mesh_plane_cache_handles()
  , m_bounding_box(ON_BoundingBox::UnsetBoundingBox)
{
}

bool RhinoVrDeviceDisplayConduit::ExecConduit(
  CRhinoDisplayPipeline&  dp,
  UINT                    nActiveChannel,
  bool&                   bTerminateChannel)
{
  if (m_draw_device_mesh)
  {
    if (nActiveChannel == CSupportChannels::SC_CALCBOUNDINGBOX)
    {
      ON_BoundingBox xformed_bb = m_bounding_box;
      xformed_bb.Transform(m_device_mesh_xform);

      m_pChannelAttrs->m_BoundingBox.Union(xformed_bb);
    }
    else if (nActiveChannel == CSupportChannels::SC_POSTDRAWOBJECTS)
    {
      dp.PushModelTransform(m_device_mesh_xform);

      for (int i = 0; i < m_start_pts.Count(); ++i)
      {
        dp.DrawLine(m_start_pts[i], m_end_pts[i], m_colors[i]);
      }

      dp.DrawShadedMeshes(&m_device_mesh, 1, m_device_material, &m_device_cache_handle);

      for (int i = 0; i < m_plane_meshes.Count(); ++i)
      {
        const ON_Mesh* mesh = &m_plane_meshes[i];
        dp.DrawShadedMeshes(&mesh, 1, m_plane_materials[i], &m_mesh_plane_cache_handles[i]);
      }

      dp.PopModelTransform();
    }
  }

  return true;
}

void RhinoVrDeviceDisplayConduit::Enable(unsigned int uiDocSerialNumber)
{
  CRhinoDisplayConduit::Enable(uiDocSerialNumber);
}

void RhinoVrDeviceDisplayConduit::SetDeviceMesh(const ON_Mesh* device_mesh)
{
  m_device_mesh = device_mesh;

  if (device_mesh)
  {
    m_draw_device_mesh = true;
    m_bounding_box.Union(device_mesh->BoundingBox());
  }
}

void RhinoVrDeviceDisplayConduit::SetDeviceMaterial(const CDisplayPipelineMaterial* device_material)
{
  m_device_material = device_material;
}

void RhinoVrDeviceDisplayConduit::SetDeviceMeshXform(const ON_Xform& device_xform)
{
  m_device_mesh_xform = device_xform;
}

void RhinoVrDeviceDisplayConduit::SetDeviceMeshCacheHandle(CRhinoCacheHandle* cache_handle)
{
  m_device_cache_handle = cache_handle;
}

void RhinoVrDeviceDisplayConduit::AddWindowMesh(const ON_Mesh& mesh, const CDisplayPipelineMaterial* material)
{
  m_bounding_box.Set(mesh.m_V[0], TRUE);
  m_bounding_box.Set(mesh.m_V[1], TRUE);
  m_bounding_box.Set(mesh.m_V[2], TRUE);
  m_bounding_box.Set(mesh.m_V[3], TRUE);

  m_plane_meshes.Append(mesh);
  m_plane_materials.Append(material);
  m_mesh_plane_cache_handles.Append(nullptr);
}

void RhinoVrDeviceDisplayConduit::AddLine(const ON_3dPoint& from, const ON_3dPoint& to, const ON_Color& color)
{
  m_start_pts.Append(from);
  m_end_pts.Append(to);
  m_colors.Append(color);

  m_bounding_box.Set(from, TRUE);
  m_bounding_box.Set(to, TRUE);
}

void RhinoVrDeviceDisplayConduit::Empty()
{
  m_start_pts.Empty();
  m_end_pts.Empty();
  m_colors.Empty();

  m_bounding_box = ON_BoundingBox::EmptyBoundingBox;

  m_draw_device_mesh = false;
  m_device_mesh = nullptr;
  m_device_material = nullptr;

  m_plane_meshes.Empty();
  m_plane_xforms.Empty();
  m_plane_materials.Empty();
}

void RhinoVrDeviceDisplayConduit::InvalidateWindowMeshCache()
{
  for (int i = 0; i < m_mesh_plane_cache_handles.Count(); ++i)
  {
    delete m_mesh_plane_cache_handles[i];
    m_mesh_plane_cache_handles[i] = nullptr;
  }
}

RhinoVrFrustumConduit::RhinoVrFrustumConduit()
  : CRhinoDisplayConduit(CSupportChannels::SC_CALCCLIPPINGPLANES)
  , m_left_frus_near(0.0)
  , m_left_frus_far(0.0)
  , m_left_frus_left(0.0)
  , m_left_frus_right(0.0)
  , m_left_frus_top(0.0)
  , m_left_frus_bottom(0.0)
  , m_right_frus_near(0.0)
  , m_right_frus_far(0.0)
  , m_right_frus_left(0.0)
  , m_right_frus_right(0.0)
  , m_right_frus_top(0.0)
  , m_right_frus_bottom(0.0)
{
}

bool RhinoVrFrustumConduit::ExecConduit(
  CRhinoDisplayPipeline&  dp,
  UINT                    nActiveChannel,
  bool&                   bTerminateChannel)
{
  if (nActiveChannel == CSupportChannels::SC_CALCCLIPPINGPLANES)
  {
    if (
      dp.m_pDisplayAttrs->GetStereoRenderContext() == CDisplayPipelineAttributes::StereoRenderContext::RenderingLeftEye &&
      m_left_frus_near != 0.0 &&
      m_left_frus_far != 0.0 &&
      m_left_frus_left != 0.0 &&
      m_left_frus_right != 0.0 &&
      m_left_frus_top != 0.0 &&
      m_left_frus_bottom != 0.0)
    {
      m_pChannelAttrs->m_dNear = m_left_frus_near;
      m_pChannelAttrs->m_dFar = m_left_frus_far;
      m_pChannelAttrs->m_dLeft = m_left_frus_left;
      m_pChannelAttrs->m_dRight = m_left_frus_right;
      m_pChannelAttrs->m_dTop = m_left_frus_top;
      m_pChannelAttrs->m_dBottom = m_left_frus_bottom;
    }

    if (
      dp.m_pDisplayAttrs->GetStereoRenderContext() == CDisplayPipelineAttributes::StereoRenderContext::RenderingRightEye &&
      m_right_frus_near != 0.0 &&
      m_right_frus_far != 0.0 &&
      m_right_frus_left != 0.0 &&
      m_right_frus_right != 0.0 &&
      m_right_frus_top != 0.0 &&
      m_right_frus_bottom != 0.0)
    {
      m_pChannelAttrs->m_dNear = m_right_frus_near;
      m_pChannelAttrs->m_dFar = m_right_frus_far;
      m_pChannelAttrs->m_dLeft = m_right_frus_left;
      m_pChannelAttrs->m_dRight = m_right_frus_right;
      m_pChannelAttrs->m_dTop = m_right_frus_top;
      m_pChannelAttrs->m_dBottom = m_right_frus_bottom;
    }
  }

  return true;
}

void RhinoVrFrustumConduit::Enable(unsigned int uiDocSerialNumber)
{
  CRhinoDisplayConduit::Enable(uiDocSerialNumber);
}

void RhinoVrFrustumConduit::SetFrustumLeft(
  double frus_near, double frus_far,
  double frus_left, double frus_right,
  double frus_top, double frus_bottom)
{
  m_left_frus_near = frus_near;
  m_left_frus_far = frus_far;
  m_left_frus_left = frus_left;
  m_left_frus_right = frus_right;
  m_left_frus_top = frus_top;
  m_left_frus_bottom = frus_bottom;
}

void RhinoVrFrustumConduit::SetFrustumRight(
  double frus_near, double frus_far,
  double frus_left, double frus_right,
  double frus_top, double frus_bottom)
{
  m_right_frus_near = frus_near;
  m_right_frus_far = frus_far;
  m_right_frus_left = frus_left;
  m_right_frus_right = frus_right;
  m_right_frus_top = frus_top;
  m_right_frus_bottom = frus_bottom;
}

