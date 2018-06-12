#pragma once

#include "../OpenVR/headers/openvr.h"

class RhinoVrHiddenAreaMeshDisplayConduit : public CRhinoDisplayConduit
{
public:
  RhinoVrHiddenAreaMeshDisplayConduit();

  bool ExecConduit(CRhinoDisplayPipeline& dp, UINT nActiveChannel, bool& bTerminateChannel) override;
  void Enable(unsigned int doc_sn);

  void SetHiddenAreaMesh(const ON_Mesh* device_mesh, vr::EVREye eye);
  void SetHiddenAreaMeshXform(const ON_Xform& device_xform, vr::EVREye eye);
  void SetHiddenAreaMeshCacheHandle(CRhinoCacheHandle* cache_handle, vr::EVREye eye);

  void Empty();

private:
  unsigned int m_uiDocSerialNumber;

  bool m_draw_hidden_area_mesh;

  const ON_Mesh* m_hidden_area_mesh_left;
  const ON_Mesh* m_hidden_area_mesh_right;
  ON_Xform m_hidden_area_mesh_left_xform;
  ON_Xform m_hidden_area_mesh_right_xform;
  CRhinoCacheHandle* m_hidden_area_mesh_left_cache_handle;
  CRhinoCacheHandle* m_hidden_area_mesh_right_cache_handle;
};

