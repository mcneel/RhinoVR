#include "StdAfx.h"
#include "RhinoVrDeviceDisplayConduit.h"

RhinoVrDeviceDisplayConduit::RhinoVrDeviceDisplayConduit()
  : CRhinoDisplayConduit(
    CSupportChannels::SC_CALCCLIPPINGPLANES |
    CSupportChannels::SC_CALCBOUNDINGBOX |
    CSupportChannels::SC_POSTDRAWOBJECTS)
  , m_frus_near_suggestion(-1.0)
  , m_frus_far_suggestion(-1.0)
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
    if (nActiveChannel == CSupportChannels::SC_CALCCLIPPINGPLANES)
    {
      if (m_frus_near_suggestion >= 0.0 && m_frus_near_suggestion >= 0.0)
      {
        m_pChannelAttrs->m_dNear = m_frus_near_suggestion;
        m_pChannelAttrs->m_dFar = m_frus_far_suggestion;
      }
    }
    else if (nActiveChannel == CSupportChannels::SC_CALCBOUNDINGBOX)
    {
      ON_BoundingBox xformed_bb = m_bounding_box;
      xformed_bb.Transform(m_device_mesh_xform);

      m_pChannelAttrs->m_BoundingBox.Union(xformed_bb);
    }
    else if (nActiveChannel == CSupportChannels::SC_POSTDRAWOBJECTS)
    {
      if (m_pDisplayAttrs->m_bShadeSurface)
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

void RhinoVrDeviceDisplayConduit::SetFrustumNearFarSuggestion(double frus_near, double frus_far)
{
  if (frus_near > 0.0 && frus_far > frus_near)
  {
    m_frus_near_suggestion = frus_near;
    m_frus_far_suggestion = frus_far;
  }
  else
  {
    m_frus_near_suggestion = m_frus_far_suggestion = -1.0;
  }
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
