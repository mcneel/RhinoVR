#pragma once

#include "../OpenVR/headers/openvr.h"
#include "RhinoVrDeviceDisplayConduit.h"
#include "RhinoVrHiddenAreaMeshDisplayConduit.h"
#include <vector>

enum class VrSystemType
{
    Unknown,
    Vive,
    Rift
};

// This class represents a VR device. It can be used
// to render the device in Rhino.
class RhinoVrDeviceModel
{
public:
  RhinoVrDeviceModel(const ON_String& device_name);
  bool Initialize(
    const vr::RenderModel_t& model,
    const vr::RenderModel_TextureMap_t& diffuse_texture,
    double unit_scale,
    const CRhinoDoc& doc);

  ON_String m_device_name;
  ON_Mesh m_device_mesh;
  CDisplayPipelineMaterial m_device_material;
  CRhinoCacheHandle m_cache_handle;
};

// This struct represents the current state of
// a VR controller.
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

  bool m_a_button_down = false;
  bool m_a_button_pressed = false;
  bool m_a_button_released = false;

  bool m_dpad_left_down = false;
  bool m_dpad_left_pressed = false;
  bool m_dpad_left_released = false;

  bool m_dpad_right_down = false;
  bool m_dpad_right_pressed = false;
  bool m_dpad_right_released = false;

  bool m_dpad_up_down = false;
  bool m_dpad_up_pressed = false;
  bool m_dpad_up_released = false;

  bool m_dpad_down_down = false;
  bool m_dpad_down_pressed = false;
  bool m_dpad_down_released = false;

  bool m_dpad_center_down = false;
  bool m_dpad_center_pressed = false;
  bool m_dpad_center_released = false;

  float m_trigger_button_value = 0.0f;
  ON_2dPoint m_touchpad_touch_point = ON_2dPoint::Origin;
};

struct RhinoVrAppWindow
{
  bool       m_enabled = false;
  ON_wString m_title;
  ON__UINT32 m_crc = 0;
  HWND       m_hwnd = nullptr;
  CRhinoDib  m_dib;
  LONG       m_width = 0;
  LONG       m_height = 0;
  ON_Mesh    m_mesh;
  double     m_mesh_width = 0.0;
  double     m_mesh_height = 0.0;
  double     m_opacity = 1.0;
  CDisplayPipelineMaterial m_material;
};

// This struct contains up-to-date information of a tracked
// VR device, such as location/orientation, geometry, and
// the state of any buttons/triggers/touchpads.
struct RhinoVrDeviceData
{
  bool m_show = false;
  ON_Xform m_xform = ON_Xform::IdentityTransformation;
  RhinoVrDeviceModel* m_render_model = nullptr;
  RhinoVrDeviceDisplayConduit m_display_conduit;
  RhinoVrDeviceController m_controller;
};

// RhinoVrRenderer is the main class of RhinoVR. It handles VR library
// initialization, input handling and frame drawing.
class RhinoVrRenderer
{
public:
  RhinoVrRenderer(unsigned int doc_sn, unsigned int view_sn);
  virtual ~RhinoVrRenderer();

  bool Initialize();
  void ProcessInputAndRenderFrame();

protected:
  // Converts an OpenVR matrix to an ON_Xform.
  ON_Xform OpenVrMatrixToXform(const vr::HmdMatrix34_t& matrix);

  // Loads all needed render models from the VR library.
  void SetupRenderModels();

  // Loads a render model for a specific device.
  void SetupRenderModelForDevice(vr::TrackedDeviceIndex_t device_index);

  // Loads the "hidden area mesh" geometry which is intended to block
  // out the parts of the screens which won't be visible to the eyes.
  // This is done to improve rendering  performance.
  ON_Mesh LoadHiddenAreaMesh(vr::Hmd_Eye eye);

  // Returns a render model by name. If it hasn't been loaded yet, it will
  // first be loaded from the VR library.
  RhinoVrDeviceModel* FindOrLoadRenderModel(const char* render_model_name);

  // Provide device display conduits with updated device information.
  void UpdateDeviceDisplayConduits(
    const ON_Xform& camera_to_world_xform,
    const ON_Xform& clip_to_left_eye_xform,
    const ON_Xform& clip_to_right_eye_xform);

  bool AttachDocAndView();
  void DetachDocAndView();
  
  // Updates the transforms of all tracked devices.
  // This function will wait for the HMD's vertical sync signal.
  bool UpdatePosesAndWaitForVSync();

  // Updates the device and controller states. Calculates view orientation.
  // Sets view frustum and near/far planes.
  bool UpdateState();

  // Performs actions based on controller inputs.
  bool HandleInput();

  // Draws the left and right eye views and sends the result to the HMD.
  bool Draw();

  // Fills in the high-level controller state from the data provided by the VR library.
  void GetRhinoVrControllerState(const vr::VRControllerState_t& state, RhinoVrDeviceController& controller);
  
  // Processes VR events.
  void ProcessVrEvent(const vr::VREvent_t & event);

  // Simulate Rhino's GetPoint in VR.
  void RhinoVrGetPoint(const ON_Xform& picking_device_xform);

  // Simulate Rhino's OnMouseMove in VR.
  void RhinoVrOnMouseMove(const ON_Xform& picking_device_xform);

  bool RhinoVrGetIntersectingAppWindow(const RhinoVrAppWindow& app_window, const ON_Xform& ray_xform, const ON_Xform& window_mesh_xform, ON_3dPoint& world_point, ON_2dPoint& screen_uvs);

  // Find object intersection with an eye-space ray transformed by 'picking_device_xform'.
  // The eye-space ray is (0.0, 0.0, -frustum_near) to (0.0, 0.0, -frustum_far).
  bool RhinoVrGetIntersectingObject(
    const ON_Xform& picking_device_xform, const CRhinoObject*& isect_object, ON_3dPoint& isect_point);

  bool GetWorldPickLineAndClipRegion(
    const ON_Xform& picking_device_xform, ON_Line& world_line,
    ON_ClippingRegion& clip_region, ON_Viewport& line_vp, ON_2iPoint& line_pixel);

protected: // Timing related functions

  // Measures time spent processing input and drawing a stereo frame.
  void FrameTimingStart();
  void FrameTimingStop();

  // Measures time spent outside RhinoVR.
  void RhinoTimingStart();
  void RhinoTimingStop();

  // Measures time spent waiting for vertical sync.
  void VsyncTimingStart();
  void VsyncTimingStop();

  // Measures the total frame time and frames per second.
  void FpsTiming();

  RhTimestamp TimingStart();
  void TimingStop(const RhTimestamp& start_time, const ON_wString& message);

  RhTimestamp m_frame_time_start;
  RhTimestamp m_rhino_time_start;
  RhTimestamp m_vsync_time_start;
  RhTimestamp m_fps_time_start;

  unsigned int m_frame_counter;

protected:
  // The serial number of the Rhino document
  // used when executing RhinoVR.
  unsigned int m_doc_sn;

  // The serial number of the Rhino view
  // which was active when executing RhinoVR.
  unsigned int m_view_sn;

  CRhinoDoc*  m_doc;  // The Rhino document acquired from 'm_doc_sn'.
  CRhinoView* m_view; // The Rhino view acquired from 'm_view_sn'.

  // Viewport representing the location, orientation and field of view of the HMD.
  std::unique_ptr<CRhinoViewport> m_vr_vp;

  // The display pipeline used for rendering in VR.
  std::unique_ptr<CRhinoDisplayPipeline> m_vr_dp;

  // The VR display pipeline down-cast to the OpenGL pipeline.
  CRhinoDisplayPipeline_OGL* m_vr_dp_ogl;

  UUID m_previous_display_mode;

  float m_near_clip; // VR frustum near.
  float m_far_clip;  // VR frustum far.

  // Left eye frustum.
  float m_left_frus_left, m_left_frus_right, m_left_frus_top, m_left_frus_bottom;

  // Right eye frustum.
  float m_right_frus_left, m_right_frus_right, m_right_frus_top, m_right_frus_bottom;

  // Scaling applied to all geometry fetched from the VR library.
  // The unit of VR library geometry is meters.
  // If the Rhino document units is millimeters, we need to scale all
  // VR library geometry by 1000.0, i.e. we need to express their
  // real-world size in millimeters.
  double m_unit_scale;

  ON_Xform m_hmd_xform;       // HMD transform.
  ON_Xform m_left_eye_xform;  // Left eye transform.
  ON_Xform m_right_eye_xform; // Right eye transform.

  // HMD location correction.
  // This transform is set once, and will always 
  // be applied to the HMD transform. It makes sure
  // that the initial location of the HMD will always
  // be where the Rhino viewport camera was originally.
  ON_Xform m_hmd_location_correction_xform;

  // Indicates whether we have set the HMD location correction xform.
  bool m_hmd_location_correction_acquired;

  // Transform from camera-space to world-space, including movement by controller.
  ON_Xform m_cam_to_world_xform;

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

  // A mesh representing the pointer line.
  ON_Mesh m_pointer_mesh;

  CDisplayPipelineMaterial m_pointer_mesh_material;

  CRhinoCacheHandle m_pointer_mesh_cache_handle;

  VrSystemType m_vr_system_type = VrSystemType::Unknown;

  // The transforms of all tracked devices.
  vr::TrackedDevicePose_t m_device_poses[vr::k_unMaxTrackedDeviceCount];

  vr::TrackedDeviceIndex_t m_device_index_left_hand;  // Device index for left hand controller.
  vr::TrackedDeviceIndex_t m_device_index_right_hand; // Device index for right hand controller.

  // The device data of all tracked devices.
  ON_ClassArray<RhinoVrDeviceData> m_device_data;

  // The render models of all tracked devices.
  ON_ClassArray<std::unique_ptr<RhinoVrDeviceModel>> m_device_render_models;

  RhinoVrFrustumConduit m_frustum_conduit;

  RhinoVrAppWindow m_gh_window;
  RhinoVrAppWindow m_rh_window;

  bool m_gh_window_left_btn_down;
  bool m_window_intersected_this_frame;

  ON_2iPoint m_last_window_click_pos;

  RhTimestamp m_last_window_update;

  const double MoveSpeedMax = 10.0;
  const double MoveAcceleration = 3.0;
  const double MoveDecelerationSoft = 6.0;
  const double MoveDecelerationHard = 18.0;

  double m_move_speed; // Movement speed in meters per second
  double m_turn_speed; // Turning speed in degrees per second
  double m_last_frame_time;
  RhTimestamp m_frame_timestamp;

  double m_move_speed_when_start_moving;
  RhTimestamp m_start_moving_timestamp;

  double m_move_speed_when_stop_moving;
  RhTimestamp m_stop_moving_timestamp;

  ON_Mesh m_hidden_mesh_left;  // The hidden area mesh for the left eye.
  ON_Mesh m_hidden_mesh_right; // The hidden area mesh for the right eye.

  CRhinoCacheHandle m_hidden_mesh_left_cache_handle;  // The left hidden area mesh cache handle.
  CRhinoCacheHandle m_hidden_mesh_right_cache_handle; // The right hidden area mesh cache handle.

  // The hidden area mesh display conduit.
  RhinoVrHiddenAreaMeshDisplayConduit m_hidden_mesh_display_conduit;

private:
  vr::IVRSystem*       m_hmd;           // System interface owned by the VR library.
  vr::IVRRenderModels* m_render_models; // Render models interface owned by the VR library.
  vr::IVRCompositor*   m_compositor;    // Compositor interface owned by the VR library.
};
