#pragma once

class RhinoVrDeviceDisplayConduit : public CRhinoDisplayConduit
{
public:
  RhinoVrDeviceDisplayConduit();

  bool ExecConduit(CRhinoDisplayPipeline& dp, UINT nActiveChannel, bool& bTerminateChannel) override;
  void Enable(unsigned int uiDocSerialNumber);

  void SetDeviceMesh(const ON_Mesh* device_mesh);
  void SetDeviceMaterial(const CDisplayPipelineMaterial* device_material);
  void SetDeviceMeshXform(const ON_Xform& device_xform);
  void SetDeviceMeshCacheHandle(CRhinoCacheHandle* cache_handle);

  void AddWindowMesh(const ON_Mesh& mesh, const CDisplayPipelineMaterial* material);
  void AddLine(const ON_3dPoint& from, const ON_3dPoint& to, const ON_Color& color);
  void Empty();
  void InvalidateWindowMeshCache();

private:

  ON_SimpleArray<ON_3dPoint> m_start_pts;
  ON_SimpleArray<ON_3dPoint> m_end_pts;
  ON_SimpleArray<ON_Color> m_colors;

  ON_ClassArray<ON_Mesh> m_plane_meshes;
  ON_ClassArray<ON_Xform> m_plane_xforms;
  ON_SimpleArray<const CDisplayPipelineMaterial*> m_plane_materials;
  ON_SimpleArray<CRhinoCacheHandle*> m_mesh_plane_cache_handles;

  bool m_draw_device_mesh;

  const ON_Mesh* m_device_mesh;
  const CDisplayPipelineMaterial* m_device_material;
  ON_Xform m_device_mesh_xform;
  CRhinoCacheHandle* m_device_cache_handle;

  ON_BoundingBox m_bounding_box;
};

class RhinoVrFrustumConduit : public CRhinoDisplayConduit
{
public:
  RhinoVrFrustumConduit();

  bool ExecConduit(CRhinoDisplayPipeline& dp, UINT nActiveChannel, bool& bTerminateChannel) override;
  void Enable(unsigned int uiDocSerialNumber);

  void SetFrustumLeft(
    double frus_near, double frus_far,
    double frus_left, double frus_right,
    double frus_top, double frus_bottom);

  void SetFrustumRight(
    double frus_near, double frus_far,
    double frus_left, double frus_right,
    double frus_top, double frus_bottom);

private:
  double m_left_frus_near;
  double m_left_frus_far;
  double m_left_frus_left;
  double m_left_frus_right;
  double m_left_frus_top;
  double m_left_frus_bottom;

  double m_right_frus_near;
  double m_right_frus_far;
  double m_right_frus_left;
  double m_right_frus_right;
  double m_right_frus_top;
  double m_right_frus_bottom;
};
