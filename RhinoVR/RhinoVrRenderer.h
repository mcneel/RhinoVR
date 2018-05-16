#pragma once

#include "../OpenVR/headers/openvr.h"
#include "RhinoVrDeviceDisplayConduit.h"
#include "RhinoVrHiddenAreaMeshDisplayConduit.h"
#include <vector>

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

struct RhinoVrDeviceController
{
  bool m_touchpad_button_down = false;
  bool m_touchpad_button_pressed = false;
  bool m_touchpad_button_released = false;
  bool m_touchpad_button_touched = false;

  bool m_appmenu_button_down = false;
  bool m_appmenu_button_pressed = false;
  bool m_appmenu_button_released = false;

  bool m_grip_button_down = false;
  bool m_grip_button_pressed = false;
  bool m_grip_button_released = false;

  bool m_trigger_button_down = false;
  bool m_trigger_button_pressed = false;
  bool m_trigger_button_released = false;

  float m_trigger_button_value = 0.0f;
  ON_2dPoint m_touchpad_touch_point = ON_2dPoint::Origin;
};

struct RhinoVrDeviceData
{
  bool m_show = false;
  ON_Xform m_xform = ON_Xform::IdentityTransformation;
  RhinoVrDeviceModel* m_render_model = nullptr;
  RhinoVrDeviceDisplayConduit m_display_conduit;
  RhinoVrDeviceController m_controller;
};

class RhinoVrRenderer
{
public:
  RhinoVrRenderer(unsigned int doc_sn, unsigned int view_sn);
  virtual ~RhinoVrRenderer();

  bool Initialize();
  void ProcessInputAndRenderFrame();

protected:
  ON_Xform OpenVrMatrixToXform(const vr::HmdMatrix34_t& matPose);

  bool UpdateDeviceXforms();

  void SetupRenderModels();
  void SetupRenderModelForDevice(vr::TrackedDeviceIndex_t unTrackedDeviceIndex);
  ON_Mesh LoadHiddenAreaMesh(vr::Hmd_Eye eye);
  RhinoVrDeviceModel* FindOrLoadRenderModel(const char* pchRenderModelName);
  void UpdateDeviceDisplayConduits(const ON_Xform& device_to_world);

  bool AttachDocAndView();
  void DetachDocAndView();
  bool Draw();
  bool HandleInput();
  bool UpdateState();
  void GetRhinoVrControllerState(const vr::VRControllerState_t& state, RhinoVrDeviceController& controller);
  void ProcessVrEvent(const vr::VREvent_t & event);

  void RhinoVrGetPoint(const ON_Xform& picking_device_xform);
  void RhinoVrOnMouseMove(const ON_Xform& picking_device_xform);
  bool RhinoVrGetIntersectingObject(
    const ON_Xform& picking_device_xform, const CRhinoObject*& isect_object, ON_3dPoint& isect_point);

  bool GetWorldPickLineAndClipRegion(
    const ON_Xform& device_xform, ON_Line& world_line,
    ON_ClippingRegion& clip_region, ON_Viewport& line_vp, ON_2iPoint& line_pixel);

protected:
  unsigned int m_doc_sn;
  unsigned int m_view_sn;

  CRhinoDoc* m_doc;
  CRhinoView* m_view;

  std::unique_ptr<CRhinoViewport> m_vr_vp;
  std::unique_ptr<CRhinoDisplayPipeline> m_vr_dp;
  CRhinoDisplayPipeline_OGL* m_vr_dp_ogl;

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

  vr::TrackedDevicePose_t m_device_poses[vr::k_unMaxTrackedDeviceCount];

  ON_ClassArray<RhinoVrDeviceData> m_device_data;
  ON_SimpleArray<RhinoVrDeviceModel*> m_device_render_models;

  ON_Mesh m_hidden_area_mesh_left;
  ON_Mesh m_hidden_area_mesh_right;
  CRhinoCacheHandle m_hidden_area_mesh_left_cache_handle;
  CRhinoCacheHandle m_hidden_area_mesh_right_cache_handle;

  RhinoVrHiddenAreaMeshDisplayConduit m_hidden_mesh_display_conduit;

private:
  vr::IVRSystem* m_hmd;
  vr::IVRRenderModels* m_render_models;
  vr::IVRCompositor* m_compositor;
};

