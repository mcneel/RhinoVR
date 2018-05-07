#pragma once

#include "../OpenVR/headers/openvr.h"
#include <vector>

class RhinoVrDeviceDisplayConduit : public CRhinoDisplayConduit
{
public:
  RhinoVrDeviceDisplayConduit();

  bool ExecConduit(CRhinoDisplayPipeline& dp, UINT nActiveChannel, bool& bTerminateChannel) override;
  void Enable(unsigned int uiDocSerialNumber);

  void SetDeviceMesh(const ON_Mesh* device_mesh);
  void SetDeviceMeshXform(const ON_Xform& device_xform);
  void SetDeviceMeshCacheHandle(CRhinoCacheHandle* cache_handle);

  void SetFrustumNearFarSuggestion(double frus_near, double frus_far);
  void AddLine(const ON_3dPoint& from, const ON_3dPoint& to, const ON_Color& color);
  void Empty();

private:
  unsigned int m_uiDocSerialNumber;

  double m_frus_near_suggestion;
  double m_frus_far_suggestion;

  ON_SimpleArray<ON_3dPoint> m_start_pts;
  ON_SimpleArray<ON_3dPoint> m_end_pts;
  ON_SimpleArray<ON_Color> m_colors;

  bool m_draw_device_mesh;

  const ON_Mesh* m_device_mesh;
  ON_Xform m_device_mesh_xform;
  CRhinoCacheHandle* m_device_cache_handle;

  ON_BoundingBox m_bounding_box;
};

class RhinoVrHiddenAreaMeshDisplayConduit : public CRhinoDisplayConduit
{
public:
  RhinoVrHiddenAreaMeshDisplayConduit();

  bool ExecConduit(CRhinoDisplayPipeline& dp, UINT nActiveChannel, bool& bTerminateChannel) override;
  void Enable(unsigned int uiDocSerialNumber);

  void SetActiveEye(vr::EVREye active_eye);
  void SetHiddenAreaMesh(const ON_Mesh* device_mesh, vr::EVREye eye);
  void SetHiddenAreaMeshXform(const ON_Xform& device_xform, vr::EVREye eye);
  void SetHiddenAreaMeshCacheHandle(CRhinoCacheHandle* cache_handle, vr::EVREye eye);

  //void SetFrustumNearFarSuggestion(double frus_near, double frus_far);
  //void AddLine(const ON_3dPoint& from, const ON_3dPoint& to, const ON_Color& color);
  void Empty();

private:
  unsigned int m_uiDocSerialNumber;

  bool m_draw_hidden_area_mesh;
  vr::EVREye m_active_eye;

  const ON_Mesh* m_hidden_area_mesh_left;
  const ON_Mesh* m_hidden_area_mesh_right;
  ON_Xform m_hidden_area_mesh_left_xform;
  ON_Xform m_hidden_area_mesh_right_xform;
  CRhinoCacheHandle* m_hidden_area_mesh_left_cache_handle;
  CRhinoCacheHandle* m_hidden_area_mesh_right_cache_handle;
};

class RhinoVrDeviceModel
{
public:
  RhinoVrDeviceModel() = default;
  RhinoVrDeviceModel(const ON_String& sRenderModelName);
  bool Init(const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture, double unit_scale);
  const ON_String& GetName() const;

  ON_String m_device_name;
  ON_Mesh m_device_mesh;
  CRhinoCacheHandle m_cache_handle;
};

class RhinoVrDeviceController
{
public:
  bool m_touchpad_button_pressed = false;
  bool m_top_button_pressed = false;
  bool m_grip_button_pressed = false;
  bool m_trigger_pressed = false;

  bool m_finger_on_touchpad = false;
  ON_2dPoint m_touchpad_point = ON_2dPoint::Origin;
};

class RhinoVrRenderer
{
public:
  RhinoVrRenderer(unsigned int doc_sn, unsigned int view_sn, unsigned int viewport_sn);
  virtual ~RhinoVrRenderer();

  bool InitializeVrRenderer();
  void HandleInputAndRenderFrame();
  bool BeginFrameDraw();
  void EndFrameDraw();
  bool Draw();
  void HandleInput();
  bool CalculateWindowCoordsForClickSimulation(const ON_Xform& device_pose, ON_2iPoint& window_coords);
  void ProcessVREvent(const vr::VREvent_t & event);

protected:
  ON_Xform OpenVrMatrixToXform(const vr::HmdMatrix34_t& matPose);
  void UpdateHMDMatrixPose();

  void SetupRenderModels();
  void SetupRenderModelForDevice(vr::TrackedDeviceIndex_t unTrackedDeviceIndex);
  ON_Mesh LoadHiddenAreaMesh(vr::Hmd_Eye eye);
  RhinoVrDeviceModel* FindOrLoadRenderModel(const char* pchRenderModelName);
  void UpdateDeviceState(const ON_Xform& device_to_world);
  void UpdateState();

  bool GetWorldPickLineAndClipRegion(
    const ON_Xform& device_xform,
    ON_Line& world_line,
    ON_ClippingRegion& clip_region,
    ON_Viewport& line_vp,
    ON_2iPoint& line_pixel);

protected:
  unsigned int m_doc_sn;
  unsigned int m_view_sn;
  unsigned int m_vr_viewport_sn;

  CRhinoDoc* m_doc;
  CRhinoView* m_view;

  int m_window_width;
  int m_window_height;

  float m_near_clip;
  float m_far_clip;

  float m_left_frus_left, m_left_frus_right, m_left_frus_top, m_left_frus_bottom;
  float m_right_frus_left, m_right_frus_right, m_right_frus_top, m_right_frus_bottom;

  double m_unit_scale;

  ON_Xform m_cam_to_eye_xform_left;
  ON_Xform m_cam_to_eye_xform_right;

  ON_Xform m_hmd_xform;
  ON_Xform m_hmd_location_correction_xform;

  bool m_hmd_location_correction_acquired;

  ON_Xform m_cam_to_world;
  ON_Xform m_world_to_cam;

  ON_Xform m_clip_to_eye_xform_left;
  ON_Xform m_clip_to_eye_xform_right;

  ON_3dVector m_camera_translation;
  double m_camera_rotation;
  ON_3dVector m_previous_camera_direction;

  // The original viewport from the Rhino view.
  ON_Viewport m_vp_orig;

  // The original viewport from the Rhino view,
  // but modified to have a frustum in accordance
  // to the HMD's field of view.
  ON_Viewport m_vp_orig_hmd_frus;

  // A viewport which represents the world-space
  // location, orientation and field of view of the HMD.
  ON_Viewport m_vp_hmd;

  // A viewport which represents the world-space
  // location, orientation and field of view of the HMD's
  // left eye.
  ON_Viewport m_vp_left_eye;

  // A viewport which represents the world-space
  // location, orientation and field of view of the HMD's
  // right eye.
  ON_Viewport m_vp_right_eye;

  // A line which represents the object-space pointer line
  // shooting out from the controllers.
  ON_Line m_pointer_line;

  float m_trigger_value;
  ON_2dPoint m_trackpad_point;

  vr::TrackedDevicePose_t m_device_pose[vr::k_unMaxTrackedDeviceCount];
  RhinoVrDeviceModel* m_device_render_model[vr::k_unMaxTrackedDeviceCount];

  ON_Xform m_device_xform[vr::k_unMaxTrackedDeviceCount];
  bool m_show_device[vr::k_unMaxTrackedDeviceCount];
  ON_SimpleArray<RhinoVrDeviceModel*> m_device_render_models;

  RhinoVrDeviceController m_device_controller[vr::k_unMaxTrackedDeviceCount];
  RhinoVrDeviceDisplayConduit m_device_display_conduit[vr::k_unMaxTrackedDeviceCount];

  //ON_Mesh m_hidden_area_mesh_left;
  //ON_Mesh m_hidden_area_mesh_right;
  //CRhinoCacheHandle m_hidden_area_mesh_left_cache_handle;
  //CRhinoCacheHandle m_hidden_area_mesh_right_cache_handle;

  //RhinoVrHiddenAreaMeshDisplayConduit m_hidden_mesh_display_conduit;

private:
  vr::IVRSystem* m_hmd;
  vr::IVRRenderModels* m_render_models;
  vr::IVRCompositor* m_compositor;
};

