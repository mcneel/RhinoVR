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

  //double m_frus_near_suggestion;
  //double m_frus_far_suggestion;

  //ON_SimpleArray<ON_3dPoint> m_start_pts;
  //ON_SimpleArray<ON_3dPoint> m_end_pts;
  //ON_SimpleArray<ON_Color> m_colors;

  bool m_draw_hidden_area_mesh;
  vr::EVREye m_active_eye;

  const ON_Mesh* m_hidden_area_mesh_left;
  const ON_Mesh* m_hidden_area_mesh_right;
  ON_Xform m_hidden_area_mesh_left_xform;
  ON_Xform m_hidden_area_mesh_right_xform;
  CRhinoCacheHandle* m_hidden_area_mesh_left_cache_handle;
  CRhinoCacheHandle* m_hidden_area_mesh_right_cache_handle;

  //ON_BoundingBox m_bounding_box;
};

class RhinoVrDeviceModel
{
public:
  RhinoVrDeviceModel() = default;
  RhinoVrDeviceModel(const ON_String& sRenderModelName);
  bool Init(const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture);
  const ON_String& GetName() const;

  ON_String m_device_name;
  ON_Mesh m_device_mesh;
  CRhinoCacheHandle m_cache_handle;
};

class RhinoVrDeviceController
{
public:
  bool m_top_button_pressed = false;
  bool m_grip_button_pressed = false;
  bool m_trigger_pressed = false;
};

class RhinoVrRenderer
{
public:
  RhinoVrRenderer(unsigned int doc_sn, unsigned int view_sn, unsigned int viewport_sn);
  virtual ~RhinoVrRenderer();

  bool InitializeVrRenderer();
  void HandleInputAndRenderFrame();
  bool DrawStereoFrame();
  void HandleInput();
  bool CalculateWindowCoordsForClickSimulation(const ON_Xform& device_pose, ON_2iPoint& window_coords);
  void ProcessVREvent(const vr::VREvent_t & event);

  void SetTargetFrameRate(int target_frame_rate);

protected:
  bool InitializeVrSystem();
  ON_Xform ConvertOpenVRMatrixToXform(const vr::HmdMatrix34_t& matPose);
  ON_Xform GetHMDMatrixProjectionEye(vr::Hmd_Eye nEye);
  ON_Xform GetHMDMatrixPoseEye(vr::Hmd_Eye nEye);
  void UpdateHMDMatrixPose();

  void SetupRenderModels();
  void SetupRenderModelForTrackedDevice(vr::TrackedDeviceIndex_t unTrackedDeviceIndex);
  ON_Mesh LoadHiddenAreaMesh(vr::Hmd_Eye eye);
  RhinoVrDeviceModel* FindOrLoadRenderModel(const char* pchRenderModelName);
  void UpdateDeviceState(const ON_Xform& device_to_world);
  void MeasureFramesPerSecond();
  void UpdateState();

  bool GetWorldPickLineAndClipRegion(
    const ON_Xform& device_xform,
    ON_Line& world_line,
    ON_ClippingRegion& clip_region,
    ON_Viewport& line_vp,
    ON_2iPoint& line_pixel);

protected:
  unsigned int m_vr_doc_sn;
  unsigned int m_vr_view_sn;
  unsigned int m_vr_viewport_sn;

  int m_window_width;
  int m_window_height;

  int m_target_frame_rate;

  float m_fNearClip;
  float m_fFarClip;

  float m_left_frus_left, m_left_frus_right, m_left_frus_top, m_left_frus_bottom;
  float m_right_frus_left, m_right_frus_right, m_right_frus_top, m_right_frus_bottom;

  ON_Xform m_mat4ProjectionLeft;
  ON_Xform m_mat4ProjectionRight;
  ON_Xform m_mat4eyePosLeft;
  ON_Xform m_mat4eyePosRight;

  ON_Xform m_mat4HMDPose;
  ON_Xform m_mat4HMDPoseCorrection;

  ON_Xform m_cam_to_world;
  ON_Xform m_world_to_cam;

  ON_Xform m_clip_to_left_eye;
  ON_Xform m_clip_to_right_eye;

  ON_3dVector m_camera_translation;
  double m_camera_rotation;
  ON_3dVector m_previous_cam_dir;

  ON_Viewport m_vp_orig;
  ON_Viewport m_vp_orig_vr_frus;
  ON_Viewport m_vp_hmd;
  ON_Viewport m_vp_left_eye;
  ON_Viewport m_vp_right_eye;

  ON_Line m_pointer_line;

  float m_trigger_value;

  vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
  RhinoVrDeviceModel* m_rTrackedDeviceToRenderModel[vr::k_unMaxTrackedDeviceCount];

  ON_Xform m_rmat4DevicePose[vr::k_unMaxTrackedDeviceCount];
  bool m_rbShowTrackedDevice[vr::k_unMaxTrackedDeviceCount];
  uint64_t m_device_packet_num[vr::k_unMaxTrackedDeviceCount];
  std::vector<RhinoVrDeviceModel*> m_vecRenderModels;

  ON_Mesh m_hidden_area_mesh_left;
  ON_Mesh m_hidden_area_mesh_right;
  CRhinoCacheHandle m_hidden_area_mesh_left_cache_handle;
  CRhinoCacheHandle m_hidden_area_mesh_right_cache_handle;

  RhinoVrDeviceController m_device_controllers[vr::k_unMaxTrackedDeviceCount];

  int m_iValidPoseCount;

  std::string m_strPoseClasses; // what classes we saw poses for this frame
  char m_rDevClassChar[vr::k_unMaxTrackedDeviceCount];   // for each device, a character representing its class

  RhinoVrDeviceDisplayConduit m_device_display_conduit[vr::k_unMaxTrackedDeviceCount];
  RhinoVrHiddenAreaMeshDisplayConduit m_hidden_mesh_display_conduit;

private:
  CRhinoDib m_framebuffer_dib;

  vr::IVRSystem* m_pHMD;
  vr::IVRRenderModels* m_pRenderModels;
};

