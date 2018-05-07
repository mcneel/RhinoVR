#include "StdAfx.h"
#include "RhinoVrRenderer.h"

#pragma comment(lib, "../OpenVR/lib/win64/openvr_api.lib")

void RhinoVrPostDigitizerEvent(ON_3dRay ray, LPARAM nFlags);
void RhinoVrPostGetObjectEvent(CRhinoGetObject* go, CRhinoView* view);
unsigned long long RhinoVrGetEyeTextureHandle(CRhinoDisplayPipeline* dp, int eye);

RhinoVrRenderer::RhinoVrRenderer(unsigned int doc_sn, unsigned int view_sn)
  : m_doc_sn(doc_sn)
  , m_view_sn(view_sn)
  , m_doc(nullptr)
  , m_view(nullptr)
  , m_vr_vp(nullptr)
  , m_vr_dp(nullptr)
  , m_hmd(nullptr)
  , m_render_models(nullptr)
  , m_compositor(nullptr)
  , m_near_clip(1.0f)
  , m_far_clip(100.0f)
  , m_cam_to_eye_xform_left(ON_Xform::IdentityTransformation)
  , m_cam_to_eye_xform_right(ON_Xform::IdentityTransformation)
  , m_hmd_xform(ON_Xform::IdentityTransformation)
  , m_hmd_location_correction_xform(ON_Xform::IdentityTransformation)
  , m_clip_to_eye_xform_left(ON_Xform::IdentityTransformation)
  , m_clip_to_eye_xform_right(ON_Xform::IdentityTransformation)
  , m_camera_translation(ON_3dVector::ZeroVector)
  , m_pointer_line(ON_Line::UnsetLine)
  , m_trigger_value(0.0f)
  , m_previous_camera_direction(ON_3dVector::UnsetVector)
  , m_camera_rotation(0.0)
  , m_hmd_location_correction_acquired(false)
  , m_unit_scale(1.0)
{
  memset(m_show_device, 0, sizeof(m_show_device));
  memset(m_device_pose, 0, sizeof(m_device_pose));
  memset(m_device_render_model, 0, sizeof(m_device_render_model));
  memset(m_device_controller, 0, sizeof(m_device_controller));
}

RhinoVrRenderer::~RhinoVrRenderer()
{
  m_hmd = nullptr;
  m_render_models = nullptr;
  m_compositor = nullptr;

  vr::VR_Shutdown();

  if (m_vr_vp)
  {
    // Prevent any copied conduit bindings from getting unbound by
    // viewport's destructor...Since "CopyFrom" also forces copying
    // the viewport's Id so that any bound conduits work on copied viewport,
    // we must also force the Id to something other than the VP that 
    // got copied, so the destructor does not then unbind any conduits 
    // to said VP...
    // RH-34780: http://mcneel.myjetbrains.com/youtrack/issue/RH-34780
    m_vr_vp->m_v.m_vp.ChangeViewportId(ON_nil_uuid);
  }
}

bool RhinoVrRenderer::InitializeVrRenderer()
{
  vr::EVRInitError ovr_error = vr::VRInitError_None;

  m_hmd = vr::VR_Init(&ovr_error, vr::VRApplication_Scene);

  if (m_hmd == nullptr || ovr_error != vr::VRInitError_None)
  {
    char buf[1024];
    sprintf_s(buf, sizeof(buf), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(ovr_error));
    RhinoApp().Print(buf);
    return false;
  }

  m_render_models = (vr::IVRRenderModels*) vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, &ovr_error);
  if (m_render_models == nullptr || ovr_error != vr::VRInitError_None)
  {
    char buf[1024];
    sprintf_s(buf, sizeof(buf), "Unable to get render model interface: %s", vr::VR_GetVRInitErrorAsEnglishDescription(ovr_error));
    RhinoApp().Print(buf);
    return false;
  }

  m_compositor = (vr::IVRCompositor*) vr::VR_GetGenericInterface(vr::IVRCompositor_Version, &ovr_error);
  if (m_compositor == nullptr || ovr_error != vr::VRInitError_None)
  {
    char buf[1024];
    sprintf_s(buf, sizeof(buf), "Unable to init VR compositor: %s", vr::VR_GetVRInitErrorAsEnglishDescription(ovr_error));
    RhinoApp().Print(buf);
    return false;
  }

  CRhinoDoc* rhino_doc = CRhinoDoc::FromRuntimeSerialNumber(m_doc_sn);
  if (rhino_doc == nullptr)
  {
    return false;
  }
  
  m_unit_scale = rhino_doc->ModelUnits().MetersPerUnit();
  m_pointer_line = ON_Line(m_unit_scale*ON_3dPoint(0, 0, -0.02f), m_unit_scale*ON_3dPoint(0, 0, -500.0f));

  SetupRenderModels();

  uint32_t rec_width, rec_height;
  m_hmd->GetRecommendedRenderTargetSize(&rec_width, &rec_height);

  CRhinoView* view = CRhinoView::FromRuntimeSerialNumber(m_view_sn);
  if (view == nullptr)
    return false;

  CRhinoDisplayPipeline* view_dp = view->DisplayPipeline();
  if (view_dp == nullptr)
    return false;

  m_vr_vp = std::make_unique<CRhinoViewport>();
  m_vr_vp->CopyFrom(view->Viewport(), true);
  m_vr_vp->SetScreenSize(rec_width, rec_height);

  view_dp->OpenPipeline();
  m_vr_dp = std::unique_ptr<CRhinoDisplayPipeline>(view_dp->ClonePipeline(*m_vr_vp));
  view_dp->ClosePipeline();

  if (m_vr_dp == nullptr)
    return false;

  ON_Viewport vp = m_vp_orig = m_vr_dp->VP();

  double l, r, b, t, n, f;
  vp.GetFrustum(&l, &r, &b, &t, &n, &f);

  m_near_clip = (float)(m_unit_scale * 0.01);
  m_far_clip = (float)(m_near_clip / vp.PerspectiveMinNearOverFar());

  m_hmd->GetProjectionRaw(vr::Eye_Left,
    &m_left_frus_left, &m_left_frus_right, &m_left_frus_top, &m_left_frus_bottom);

  m_left_frus_left *= m_near_clip;
  m_left_frus_right *= m_near_clip;
  m_left_frus_top *= m_near_clip;
  m_left_frus_bottom *= m_near_clip;

  m_hmd->GetProjectionRaw(vr::Eye_Right,
    &m_right_frus_left, &m_right_frus_right, &m_right_frus_top, &m_right_frus_bottom);

  m_right_frus_left *= m_near_clip;
  m_right_frus_right *= m_near_clip;
  m_right_frus_top *= m_near_clip;
  m_right_frus_bottom *= m_near_clip;

  vp.SetFrustumLeftRightSymmetry(false);
  vp.SetFrustumTopBottomSymmetry(false);
  vp.SetFrustum(
    m_left_frus_left, m_right_frus_right,
    m_left_frus_top, m_left_frus_bottom,
    m_near_clip, m_far_clip);

  m_vp_orig_hmd_frus = vp;

  int vp_width = vp.ScreenPortWidth();
  int vp_height = vp.ScreenPortHeight();

  double frus_aspect;
  vp.GetFrustumAspect(frus_aspect);

  int vp_new_width = vp_width / 2;
  int vp_new_height = (int)floor(vp_new_width / frus_aspect + 0.5);

  ON_wString script;
  script.Format(L"-_ViewportProperties _Size %d %d _Enter", vp_new_width, vp_new_height);

  RhinoApp().RunScriptEx(m_doc_sn, script, nullptr, 0);

  vp.SetScreenPort(0, vp_width, 0, vp_height);

  view->ActiveViewport().SetVP(vp, TRUE);
  view->Redraw();

  return true;
}

ON_String GetTrackedDeviceString(vr::IVRSystem& hmd, vr::TrackedDeviceIndex_t device_idx, vr::TrackedDeviceProperty device_property, vr::TrackedPropertyError* peError = nullptr)
{
  uint32_t required_buffer_len = hmd.GetStringTrackedDeviceProperty(device_idx, device_property, NULL, 0, peError);
  if (required_buffer_len == 0)
    return ON_String("");

  char* pchBuffer = new char[required_buffer_len];
  
  required_buffer_len = hmd.GetStringTrackedDeviceProperty(device_idx, device_property, pchBuffer, required_buffer_len, peError);
  ON_String device_string = pchBuffer;

  delete[] pchBuffer;

  return device_string;
}

RhinoVrDeviceModel* RhinoVrRenderer::FindOrLoadRenderModel(const char* render_model_name)
{
  if (m_render_models == nullptr)
  {
    return nullptr;
  }

  RhinoVrDeviceModel* render_model = nullptr;
  for (int i = 0; i < m_device_render_models.Count(); i++)
  {
    RhinoVrDeviceModel* dm = m_device_render_models[i];

    if (dm->GetName().EqualOrdinal(render_model_name, false))
    {
      render_model = dm;
      break;
    }
  }

  if (render_model == nullptr)
  {
    vr::RenderModel_t* model;
    vr::EVRRenderModelError error;

    while (1)
    {
      error = m_render_models->LoadRenderModel_Async(render_model_name, &model);
      if (error != vr::VRRenderModelError_Loading)
        break;

      Sleep(1);
    }

    if (model == nullptr || error != vr::VRRenderModelError_None)
    {
      RhinoApp().Print("Unable to load render model %s - %s\n", render_model_name, m_render_models->GetRenderModelErrorNameFromEnum(error));
      return nullptr;
    }

    vr::RenderModel_TextureMap_t* texture;
    while (1)
    {
      error = m_render_models->LoadTexture_Async(model->diffuseTextureId, &texture);
      if (error != vr::VRRenderModelError_Loading)
        break;

      Sleep(1);
    }

    if (texture == nullptr || error != vr::VRRenderModelError_None)
    {
      m_render_models->FreeRenderModel(model);

      RhinoApp().Print("Unable to load render texture id:%d for render model %s\n", model->diffuseTextureId, render_model_name);

      return nullptr;
    }

    render_model = new RhinoVrDeviceModel(render_model_name);
    if (!render_model->Init(*model, *texture, m_unit_scale))
    {
      delete render_model;
      render_model = nullptr;

      RhinoApp().Print("Unable to create Rhino Mesh model from render model %s\n", render_model_name);
    }
    else
    {
      m_device_render_models.Append(render_model);
    }

    m_render_models->FreeRenderModel(model);
    m_render_models->FreeTexture(texture);
  }

  return render_model;
}

void RhinoVrRenderer::SetupRenderModelForDevice(vr::TrackedDeviceIndex_t device_index)
{
  if (m_hmd == nullptr)
    return;

  if (device_index >= vr::k_unMaxTrackedDeviceCount)
    return;

  ON_String render_model_name = GetTrackedDeviceString(*m_hmd, device_index, vr::Prop_RenderModelName_String);

  RhinoVrDeviceModel* render_model = FindOrLoadRenderModel(render_model_name);
  if (!render_model)
  {
    ON_String tracking_system_name = GetTrackedDeviceString(*m_hmd, device_index, vr::Prop_TrackingSystemName_String);
    RhinoApp().Print("Unable to load render model for tracked device %d (%s.%s)",
      device_index, tracking_system_name.Array(), render_model_name.Array());
  }
  else
  {
    m_device_render_model[device_index] = render_model;

    if (!render_model_name.EqualOrdinal("lh_basestation_vive", false))
    {
      m_show_device[device_index] = true;
    }
  }
}

ON_Mesh RhinoVrRenderer::LoadHiddenAreaMesh(vr::Hmd_Eye eye)
{
  vr::HiddenAreaMesh_t hidden_mesh = m_hmd->GetHiddenAreaMesh(eye);

  const int vertex_count = hidden_mesh.unTriangleCount * 3;
  const int face_count = hidden_mesh.unTriangleCount;

  ON_Mesh mesh;
  mesh.m_V.SetCapacity(vertex_count);
  mesh.m_V.SetCount(vertex_count);

  for (int vi = 0; vi < vertex_count; ++vi)
  {
    double x = (double)hidden_mesh.pVertexData[vi].v[0];
    double y = (double)hidden_mesh.pVertexData[vi].v[1];

    mesh.m_V[vi] = ON_3fPoint((float)(2.0*x - 1.0), (float)(2.0*y - 1.0), 1.0f);
  }

  mesh.m_F.SetCapacity(face_count);
  mesh.m_F.SetCount(face_count);

  for (int fi = 0, vi = 0; fi < face_count; ++fi)
  {
    ON_MeshFace& face = mesh.m_F[fi];
    face.vi[0] = vi++;
    face.vi[1] = vi++;
    face.vi[2] = vi++;
    face.vi[3] = face.vi[2];
  }

  return mesh;
}

void RhinoVrRenderer::SetupRenderModels()
{
  if (!m_hmd)
    return;

  for (auto device_idx = vr::k_unTrackedDeviceIndex_Hmd + 1; device_idx < vr::k_unMaxTrackedDeviceCount; device_idx++)
  {
    if (!m_hmd->IsTrackedDeviceConnected(device_idx))
      continue;

    SetupRenderModelForDevice(device_idx);
  }

  //m_hidden_area_mesh_left = LoadHiddenAreaMesh(vr::Eye_Left);
  //m_hidden_area_mesh_right = LoadHiddenAreaMesh(vr::Eye_Right);
}

void RhinoVrRenderer::UpdateDeviceState(const ON_Xform& device_to_world)
{
  bool bIsInputAvailable = m_hmd->IsInputAvailable();

  for (uint32_t unTrackedDevice = 0; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++)
  {
    RhinoVrDeviceDisplayConduit& ddc = m_device_display_conduit[unTrackedDevice];
    ddc.Empty();

    RhinoVrDeviceModel* device_model = m_device_render_model[unTrackedDevice];

    if (!device_model || !m_show_device[unTrackedDevice])
      continue;

    const vr::TrackedDevicePose_t& pose = m_device_pose[unTrackedDevice];
    if (!pose.bPoseIsValid)
      continue;

    const bool is_controller = (m_hmd->GetTrackedDeviceClass(unTrackedDevice) == vr::TrackedDeviceClass_Controller);

    if (!bIsInputAvailable && is_controller)
      continue;

    if (is_controller)
    {
      ddc.AddLine(m_pointer_line.from, m_pointer_line.to, ON_Color::SaturatedGreen);
    }

    const ON_Mesh& device_mesh = device_model->m_device_mesh;
    const ON_Xform device_xform = device_to_world * m_device_xform[unTrackedDevice];
    CRhinoCacheHandle& device_cache_handle = device_model->m_cache_handle;
    
    ddc.SetDeviceMesh(&device_mesh);
    ddc.SetDeviceMeshXform(device_xform);
    ddc.SetDeviceMeshCacheHandle(&device_cache_handle);

    if (!ddc.IsEnabled())
    {
      ddc.SetFrustumNearFarSuggestion(m_near_clip, m_far_clip);
      ddc.Enable(m_doc_sn);
    }
  }

  //m_hidden_mesh_display_conduit.SetHiddenAreaMesh(&m_hidden_area_mesh_left, vr::Eye_Left);
  //m_hidden_mesh_display_conduit.SetHiddenAreaMesh(&m_hidden_area_mesh_right, vr::Eye_Right);

  //m_hidden_mesh_display_conduit.SetHiddenAreaMeshXform(m_clip_to_left_eye, vr::Eye_Left);
  //m_hidden_mesh_display_conduit.SetHiddenAreaMeshXform(m_clip_to_right_eye, vr::Eye_Right);

  //m_hidden_mesh_display_conduit.SetHiddenAreaMeshCacheHandle(&m_hidden_area_mesh_left_cache_handle, vr::Eye_Left);
  //m_hidden_mesh_display_conduit.SetHiddenAreaMeshCacheHandle(&m_hidden_area_mesh_right_cache_handle, vr::Eye_Right);

  //if (!m_hidden_mesh_display_conduit.IsEnabled())
  //{
  //  m_hidden_mesh_display_conduit.Enable(m_vr_doc_sn);
  //}
}

bool IsKeyPressed(int uKey) { return ((::GetAsyncKeyState(uKey) & 0x8000) == 0x8000); }

void RhinoVrRenderer::UpdateState()
{
  if (m_view == nullptr || m_doc == nullptr)
  {
    return;
  }

  UpdateHMDMatrixPose();

  if (m_trackpad_point != ON_2dPoint::UnsetPoint)
  {
    if (!m_doc->InCommand())
    {
      double x = m_trackpad_point.x;
      double x_sign = (x >= 0.0 ? 1.0 : -1.0);

      if (abs(x) >= 0.4)
      {
        double x_scaled = x_sign * (abs(x) - 0.4)*(1.0 / 0.6);
        m_camera_rotation = m_camera_rotation - 2.0*x_scaled*ON_DEGREES_TO_RADIANS;
      }

      double y = m_trackpad_point.y;
      double y_sign = (y >= 0.0 ? 1.0 : -1.0);

      if (abs(y) >= 0.4)
      {
        double y_scaled = y_sign * (abs(y) - 0.4)*(1.0 / 0.6);

        if (m_previous_camera_direction != ON_3dVector::UnsetVector)
        {
          m_camera_translation = m_camera_translation + 0.25*y_scaled*m_previous_camera_direction*m_unit_scale;
        }
      }
    }
  }

  //if (IsKeyPressed(VK_UP) && m_previous_cam_dir != ON_3dVector::UnsetVector)
  //{
  //  m_camera_translation = m_camera_translation + 0.5*m_previous_cam_dir*m_unit_scale;
  //}
  //if (IsKeyPressed(VK_DOWN) && m_previous_cam_dir != ON_3dVector::UnsetVector)
  //{
  //  m_camera_translation = m_camera_translation - 0.5*m_previous_cam_dir*m_unit_scale;
  //}
  //if (IsKeyPressed(VK_LEFT))
  //{
  //  m_camera_rotation = m_camera_rotation + 9.0*ON_DEGREES_TO_RADIANS;
  //}
  //if (IsKeyPressed(VK_RIGHT))
  //{
  //  m_camera_rotation = m_camera_rotation - 9.0*ON_DEGREES_TO_RADIANS;
  //}

  const ON_Viewport& rhino_vp = m_view->ActiveViewport().VP();

  ON_Viewport vp = m_vp_orig_hmd_frus;
  vp.SetProjection(rhino_vp.Projection());
  vp.SetCameraLocation(rhino_vp.CameraLocation());
  vp.SetCameraDirection(rhino_vp.CameraDirection());
  vp.SetCameraUp(rhino_vp.CameraUp());
  vp.SetTargetPoint(rhino_vp.TargetPoint());
  vp.SetFrustumNearFar(m_near_clip, m_far_clip);

  vp.DollyCamera(m_camera_translation);
  vp.DollyFrustum(m_camera_translation.z);
  vp.Rotate(m_camera_rotation, ON_3dVector::ZAxis, vp.CameraLocation());

  vp.GetXform(ON::coordinate_system::camera_cs, ON::coordinate_system::world_cs, m_cam_to_world);
  vp.GetXform(ON::coordinate_system::world_cs, ON::coordinate_system::camera_cs, m_world_to_cam);

  m_vp_hmd = vp;
  ON_Xform cam_to_hmd_xform = m_cam_to_world * m_hmd_xform * m_world_to_cam;

  m_vp_hmd.Transform(cam_to_hmd_xform);

  m_previous_camera_direction = m_vp_hmd.CameraDirection().UnitVector();

  // By default, the view is set to the HMD viewport.
  m_vr_vp->SetVP(m_vp_hmd, TRUE);

  ON_Xform cam_to_left_eye_xform = m_cam_to_world * m_hmd_xform * m_cam_to_eye_xform_left * m_world_to_cam;

  m_vp_left_eye = vp;

  m_vp_left_eye.Transform(cam_to_left_eye_xform);
  m_vp_left_eye.SetFrustum(
    m_left_frus_left, m_left_frus_right,
    m_left_frus_top, m_left_frus_bottom,
    m_near_clip, m_far_clip);

  m_vp_left_eye.GetXform(ON::coordinate_system::clip_cs, ON::coordinate_system::world_cs, m_clip_to_eye_xform_left);

  ON_Xform cam_to_right_eye_xform = m_cam_to_world * m_hmd_xform * m_cam_to_eye_xform_right * m_world_to_cam;

  m_vp_right_eye = vp;

  m_vp_right_eye.Transform(cam_to_right_eye_xform);
  m_vp_right_eye.SetFrustum(
    m_right_frus_left, m_right_frus_right,
    m_right_frus_top, m_right_frus_bottom,
    m_near_clip, m_far_clip);

  m_vp_right_eye.GetXform(ON::coordinate_system::clip_cs, ON::coordinate_system::world_cs, m_clip_to_eye_xform_right);

  UpdateDeviceState(m_cam_to_world);
}

void DrawStereoFrameBuffer(
  CRhinoDisplayPipeline& dp,
  CDisplayPipelineAttributes& dpa,
  ON_Viewport& vp_left_eye, ON_Viewport& vp_right_eye,
  unsigned long long& eye_handle_left, unsigned long long& eye_handle_right)
{
  dp.EnableDynamicDisplayDowngrade(false);

  bool frame_buffer_capture_enabled = CRhinoDisplayPipeline::FrameBufferCaptureEnabled();
  if (frame_buffer_capture_enabled)
  {
    CRhinoDisplayPipeline::EnableFrameBufferCapture(false);
  }

  //m_hidden_mesh_display_conduit.SetActiveEye(vr::Eye_Left);

  dp.ClosePipeline();
  dp.DrawFrameBuffer(dpa, vp_left_eye, true, true);
  dp.OpenPipeline();

  eye_handle_left = RhinoVrGetEyeTextureHandle(&dp, 0);

  //m_hidden_mesh_display_conduit.SetActiveEye(vr::Eye_Right);

  dp.ClosePipeline();
  dp.DrawFrameBuffer(dpa, vp_right_eye, true, true);
  dp.OpenPipeline();

  eye_handle_right = RhinoVrGetEyeTextureHandle(&dp, 1);

  CRhinoDisplayPipeline::EnableFrameBufferCapture(frame_buffer_capture_enabled);

  dp.ClosePipeline();
}

bool RhinoVrRenderer::Draw()
{
  if (m_compositor == nullptr || m_view == nullptr)
    return false;

  CDisplayPipelineAttributes* vr_dpa = m_view->DisplayAttributes();

  if (vr_dpa == nullptr)
    return false;

  unsigned long long eye_left_handle = 0;
  unsigned long long eye_right_handle = 0;

  DrawStereoFrameBuffer(*m_vr_dp, *vr_dpa, m_vp_left_eye, m_vp_right_eye, eye_left_handle, eye_right_handle);

  vr::EVRCompositorError ovr_error = vr::VRCompositorError_None;

  m_vr_dp->OpenPipeline();

  vr::Texture_t leftEyeTexture = { (void*)(uintptr_t)eye_left_handle, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
  ovr_error = m_compositor->Submit(vr::Eye_Left, &leftEyeTexture);
  if (ovr_error != vr::VRCompositorError_None && ovr_error != vr::VRCompositorError_DoNotHaveFocus)
  {
    ASSERT(false);
  }

  vr::Texture_t rightEyeTexture = { (void*)(uintptr_t)eye_right_handle, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
  ovr_error = m_compositor->Submit(vr::Eye_Right, &rightEyeTexture);
  if (ovr_error != vr::VRCompositorError_None && ovr_error != vr::VRCompositorError_DoNotHaveFocus)
  {
    ASSERT(false);
  }

  m_vr_dp->ClosePipeline();

  return true;
}

void RhinoVrRenderer::ProcessVREvent(const vr::VREvent_t & event)
{
  switch (event.eventType)
  {
  case vr::VREvent_TrackedDeviceActivated:
  {
    SetupRenderModelForDevice(event.trackedDeviceIndex);
    //dprintf("Device %u attached. Setting up render model.\n", event.trackedDeviceIndex);
  }
  break;
  case vr::VREvent_TrackedDeviceDeactivated:
  {
    //dprintf("Device %u detached.\n", event.trackedDeviceIndex);
  }
  break;
  case vr::VREvent_TrackedDeviceUpdated:
  {
    //dprintf("Device %u updated.\n", event.trackedDeviceIndex);
  }
  break;
  }
}

int sort_objref_by_seldist(const CRhinoObjRef* a, const CRhinoObjRef* b)
{
  double d = a->SelectionDistance() - b->SelectionDistance();
  if (d < 0.0)
    return -1; // a was closer to pick point
  if (d > 0.0)
    return 1;  // b was closer to pick point

  d = a->SelectionDepth() - b->SelectionDepth();
  if (d > 0.0)
    return -1; // a was nearer to the camera
  if (d < 0.0)
    return 1;  // b was nearer to the camera

  return 0;
}

bool RhinoVrRenderer::CalculateWindowCoordsForClickSimulation(
  const ON_Xform& device_pose,
  ON_2iPoint& window_coords)
{
  window_coords.x = window_coords.y = -1;

  if (m_doc == nullptr || m_view == nullptr)
    return false;

  ON_Viewport ray_vp = m_vp_hmd;

  ON_Xform world_to_screen;
  m_vp_hmd.GetXform(ON::coordinate_system::world_cs, ON::coordinate_system::screen_cs, world_to_screen);

  ray_vp.Transform(m_cam_to_world*device_pose*m_hmd_xform.Inverse()*m_world_to_cam);

  CRhinoPickContext pc;
  pc.m_view = m_view;
  pc.m_pick_mode = CRhinoPickContext::shaded_pick;
  pc.m_pick_style = CRhinoPickContext::point_pick;

  ON_2iPoint center_pixel = ON_2iPoint(ray_vp.ScreenPortWidth() / 2, ray_vp.ScreenPortHeight() / 2);

  m_vr_vp->SetVP(ray_vp, TRUE);
  m_vr_vp->SetClippingRegionTransformation(center_pixel.x, center_pixel.y, pc.m_pick_region);
  m_vr_vp->SetVP(m_vp_hmd, TRUE);

  ray_vp.GetFrustumLine(center_pixel.x, center_pixel.y, pc.m_pick_line);

  pc.UpdateClippingPlanes();

  //vr_doc->AddCurveObject(pc.m_pick_line);

  CRhinoObjRefArray rhino_objs;
  m_doc->PickObjects(pc, rhino_objs);

  rhino_objs.QuickSort(sort_objref_by_seldist);

  if (rhino_objs.Count() > 0)
  {
    CRhinoObjRef objref = rhino_objs[0];

    ON_3dPoint isect_pt;
    if (objref.SelectionPoint(isect_pt))
    {
      ON_3dPoint pt_screen = world_to_screen * isect_pt;
      window_coords = ON_2iPoint((int)pt_screen.x, (int)pt_screen.y);
    }
  }

  return true;
}

bool RhinoVrRenderer::GetWorldPickLineAndClipRegion(
  const ON_Xform& device_xform,
  ON_Line& world_line,
  ON_ClippingRegion& clip_region,
  ON_Viewport& line_vp,
  ON_2iPoint& line_pixel)
{
  line_vp = m_vp_hmd;

  line_vp.SetCameraLocation(ON_3dPoint::Origin);
  line_vp.SetCameraDirection(-ON_3dVector::ZAxis);
  line_vp.SetCameraUp(ON_3dVector::YAxis);
  line_vp.Transform(m_cam_to_world*device_xform);

  int l, r, b, t;
  line_vp.GetScreenPort(&l, &r, &b, &t);

  // Make screen port an uneven amount of pixels.
  // This way the center pixel will be truly at
  // center of the screen, which in turn will 
  // make the pick as accurately as possible.
  if (r % 2 == 0) r += 1;
  if (b % 2 == 0) b += 1;

  line_vp.SetScreenPort(l, r, b, t);

  double near_dist = line_vp.FrustumNear();
  line_vp.SetFrustumNearFar(near_dist, near_dist + m_pointer_line.Length());

  line_pixel = ON_2iPoint(line_vp.ScreenPortWidth() / 2 + 1, line_vp.ScreenPortHeight() / 2 + 1);

  m_vr_vp->SetVP(line_vp, TRUE);
  m_vr_vp->SetClippingRegionTransformation(line_pixel.x, line_pixel.y, clip_region);
  m_vr_vp->SetVP(m_vp_hmd, TRUE);

  line_vp.GetFrustumLine(line_pixel.x, line_pixel.y, world_line);

  return true;
}

void RhinoVrRenderer::HandleInput()
{
  if (m_doc == nullptr || m_view == nullptr)
    return;

  vr::VREvent_t event;
  while (m_hmd->PollNextEvent(&event, sizeof(event)))
  {
    ProcessVREvent(event);
  }

  for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
  {
    if (m_hmd->GetTrackedDeviceClass(unDevice) != vr::TrackedDeviceClass_Controller)
      continue;

    vr::VRControllerState_t state;
    if (m_hmd->GetControllerState(unDevice, &state, sizeof(state)))
    {
      RhinoVrDeviceController& controller = m_device_controller[unDevice];

      if (controller.m_touchpad_button_pressed)
      {
        auto touch_button_check = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
        if ((state.ulButtonPressed & touch_button_check) == 0)
        {
          controller.m_touchpad_button_pressed = false;
        }
      }
      else
      {
        auto touch_button_check = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
        if (state.ulButtonPressed & touch_button_check)
        {
          controller.m_touchpad_button_pressed = true;

          if (m_doc->InGetPoint())
          {
            CRhinoGetPoint* gp = m_doc->InGetPoint();

            ON_Line world_line;
            ON_ClippingRegion clip_region;
            ON_Viewport line_vp;
            ON_2iPoint line_pixel;

            if (GetWorldPickLineAndClipRegion(m_device_xform[unDevice], world_line, clip_region, line_vp, line_pixel))
            {
              LPARAM nFlags = 0;
              ON_3dPoint screen_pt = ON_3dPoint(line_pixel.x, line_pixel.y, 0.0);

              CRhinoViewport& rhino_vp = m_view->ActiveViewport();

              const ON_Viewport orig_vp = rhino_vp.VP();
              const int orig_width = orig_vp.ScreenPortWidth();
              const int orig_height = orig_vp.ScreenPortHeight();

              rhino_vp.SetVP(line_vp, TRUE);
              rhino_vp.SetScreenSize(line_vp.ScreenPortWidth(), line_vp.ScreenPortHeight());

              ON_3dPoint world_point;
              if (gp->GetView3dPoint(1, *m_view, nFlags, screen_pt, world_line, world_point))
              {
                ON_3dRay ray;
                ray.m_P = world_point;
                ray.m_V = ON_3dVector::ZeroVector;

                LPARAM nFlags = 0; // MK_LBUTTON;

                RhinoVrPostDigitizerEvent(ray, nFlags);
              }

              rhino_vp.SetVP(orig_vp, TRUE);
              rhino_vp.SetScreenSize(orig_width, orig_height);
            }
          }
          else
          {
            ON_Viewport line_vp;
            ON_2iPoint line_pixel;

            CRhinoPickContext pc;
            pc.m_view = m_view;
            pc.m_pick_mode = CRhinoPickContext::shaded_pick;
            pc.m_pick_style = CRhinoPickContext::point_pick;

            if (GetWorldPickLineAndClipRegion(m_device_xform[unDevice], pc.m_pick_line, pc.m_pick_region, line_vp, line_pixel))
            {
              pc.UpdateClippingPlanes();

              //rhino_doc->AddCurveObject(pc.m_pick_line);

              CRhinoObjRefArray rhino_objs;
              m_doc->PickObjects(pc, rhino_objs);

              rhino_objs.QuickSort(sort_objref_by_seldist);

              if (rhino_objs.Count() > 0)
              {
                CRhinoObjRef objref = rhino_objs[0];

                if (m_doc->InGetObject())
                {
                  CRhinoGetObject* go = m_doc->InGetObject();
                  CRhinoObjRefArray& obj_ref_array = const_cast<CRhinoObjRefArray&>(go->PickList());
                  obj_ref_array.Append(objref);

                  RhinoVrPostGetObjectEvent(go, m_view);

                }
                else if (m_doc->InGetPoint())
                {
                  ON_3dPoint isect_pt;
                  if (objref.SelectionPoint(isect_pt))
                  {
                    ON_3dRay ray;
                    ray.m_P = isect_pt;
                    ray.m_V = ON_3dVector::ZeroVector;

                    LPARAM nFlags = 0; // MK_LBUTTON;

                    RhinoVrPostDigitizerEvent(ray, nFlags);
                  }
                }
                else
                {
                  objref.Object()->Select();
                }
              }
            }
          }
        }
      }

      auto tackpad_button_touch = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);
      if (state.ulButtonTouched & tackpad_button_touch)
      {
        controller.m_touchpad_point.x = state.rAxis[0].x;
        controller.m_touchpad_point.y = state.rAxis[0].y;

        m_trackpad_point = controller.m_touchpad_point;
      }
      else
      {
        m_trackpad_point = controller.m_touchpad_point = ON_2dPoint::UnsetPoint;
      }

      if (controller.m_top_button_pressed)
      {
        auto top_button_check = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_ApplicationMenu);
        if ((state.ulButtonPressed & top_button_check) == 0)
        {
          controller.m_top_button_pressed = false;
        }
      }
      else
      {
        auto top_button_check = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_ApplicationMenu);
        if (state.ulButtonPressed & top_button_check)
        {
          controller.m_top_button_pressed = true;
          RhinoApp().ExecuteCommand(m_doc_sn, L"Move");
        }
      }

      if (controller.m_grip_button_pressed)
      {
        auto grip_button_check = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_Grip);
        if ((state.ulButtonPressed & grip_button_check) == 0)
        {
          controller.m_grip_button_pressed = false;
        }
      }
      else
      {
        auto grip_button_check = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_Grip);
        if (state.ulButtonPressed & grip_button_check)
        {
          controller.m_grip_button_pressed = true;
        }
      }

      if (controller.m_trigger_pressed)
      {
        if (state.rAxis[1].x < 1.0)
        {
          controller.m_trigger_pressed = false;
        }
      }
      else
      {
        if (state.rAxis[1].x == 1.0)
        {
          controller.m_trigger_pressed = true;

          RhinoApp().ExecuteCommand(m_doc_sn, L"_Enter");
        }
        else
        {
          if (m_doc->InGetPoint())
          {
            CRhinoGetPoint* gp = m_doc->InGetPoint();

            ON_Line world_line;
            ON_ClippingRegion clip_region;
            ON_Viewport line_vp;
            ON_2iPoint line_pixel;

            if (GetWorldPickLineAndClipRegion(m_device_xform[unDevice], world_line, clip_region, line_vp, line_pixel))
            {
              LPARAM nFlags = 0;
              ON_3dPoint screen_pt = ON_3dPoint(line_pixel.x, line_pixel.y, 0.0);

              CRhinoViewport& rhino_vp = m_view->ActiveViewport();

              const ON_Viewport orig_vp = rhino_vp.VP();
              const int orig_width = orig_vp.ScreenPortWidth();
              const int orig_height = orig_vp.ScreenPortHeight();

              rhino_vp.SetVP(line_vp, TRUE);
              rhino_vp.SetScreenSize(line_vp.ScreenPortWidth(), line_vp.ScreenPortHeight());

              ON_3dPoint world_point;
              if (gp->GetView3dPoint(1, *m_view, (UINT_PTR)nFlags, screen_pt, world_line, world_point))
              {
                gp->OnMouseMove(*m_vr_vp, (UINT)nFlags, world_point, (const ON_2iPoint*) nullptr);
              }

              rhino_vp.SetVP(orig_vp, TRUE);
              rhino_vp.SetScreenSize(orig_width, orig_height);
            }
          }
        }
      }

      //auto trigger_check = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Trigger);

      //if ((state.ulButtonPressed & trigger_check) > 0/* && m_device_packet_num[unDevice] != state.unPacketNum*/)
      //{
      //  //m_rbShowTrackedDevice[unDevice] = !m_rbShowTrackedDevice[unDevice];
      //  //m_device_packet_num[unDevice] = state.unPacketNum;
      //}
    }
  }

  //if (GetView3dPoint(1, // 1 = mouse pointing device
  //  view,
  //  nFlags,
  //  ON_3dPoint(point.x, point.y, 0.0),
  //  world_line, world_point))
  //{
}

void RhinoVrRenderer::HandleInputAndRenderFrame()
{
  if (BeginFrameDraw())
  {
    UpdateState();
    HandleInput();
    Draw();
    EndFrameDraw();
  }
}

bool RhinoVrRenderer::BeginFrameDraw()
{
  if (m_hmd == nullptr || m_render_models == nullptr || m_compositor == nullptr)
    return false;

  m_doc = CRhinoDoc::FromRuntimeSerialNumber(m_doc_sn);
  if (m_doc == nullptr)
    return false;

  m_view = CRhinoView::FromRuntimeSerialNumber(m_view_sn);
  if (m_view == nullptr)
    return false;

  return true;
}

void RhinoVrRenderer::EndFrameDraw()
{
  m_doc = nullptr;
  m_view = nullptr;
}

ON_Xform RhinoVrRenderer::OpenVrMatrixToXform(const vr::HmdMatrix34_t &mat)
{
  ON_Xform xform;

  xform.m_xform[0][0] = mat.m[0][0];
  xform.m_xform[0][1] = mat.m[0][1];
  xform.m_xform[0][2] = mat.m[0][2];
  xform.m_xform[0][3] = mat.m[0][3] * m_unit_scale;

  xform.m_xform[1][0] = mat.m[1][0];
  xform.m_xform[1][1] = mat.m[1][1];
  xform.m_xform[1][2] = mat.m[1][2];
  xform.m_xform[1][3] = mat.m[1][3] * m_unit_scale;

  xform.m_xform[2][0] = mat.m[2][0];
  xform.m_xform[2][1] = mat.m[2][1];
  xform.m_xform[2][2] = mat.m[2][2];
  xform.m_xform[2][3] = mat.m[2][3] * m_unit_scale;

  xform.m_xform[3][0] = 0.0;
  xform.m_xform[3][1] = 0.0;
  xform.m_xform[3][2] = 0.0;
  xform.m_xform[3][3] = 1.0;

  return xform;
}

void RhinoVrRenderer::UpdateHMDMatrixPose()
{
  if (m_hmd == nullptr || m_compositor == nullptr)
    return;

  const uint32_t hmd_device_index = vr::k_unTrackedDeviceIndex_Hmd;

  m_compositor->WaitGetPoses(m_device_pose, vr::k_unMaxTrackedDeviceCount, nullptr, 0);

  if (!m_hmd_location_correction_acquired && m_device_pose[hmd_device_index].bPoseIsValid)
  {
    m_hmd_location_correction_acquired = true;

    ON_Xform xform = OpenVrMatrixToXform(m_device_pose[hmd_device_index].mDeviceToAbsoluteTracking);
    m_hmd_location_correction_xform = ON_Xform::TranslationTransformation(ON_3dVector(-xform[0][3], -xform[1][3], -xform[2][3]));
  }

  for (int device_idx = 0; device_idx < vr::k_unMaxTrackedDeviceCount; ++device_idx)
  {
    if (m_device_pose[device_idx].bPoseIsValid)
    {
      const vr::HmdMatrix34_t& ovr_device_matrix = m_device_pose[device_idx].mDeviceToAbsoluteTracking;
      m_device_xform[device_idx] = m_hmd_location_correction_xform * OpenVrMatrixToXform(ovr_device_matrix);
    }
  }

  if (m_device_pose[hmd_device_index].bPoseIsValid)
  {
    m_hmd_xform = m_device_xform[hmd_device_index];

    m_cam_to_eye_xform_left = OpenVrMatrixToXform(m_hmd->GetEyeToHeadTransform(vr::Eye_Left));
    m_cam_to_eye_xform_right = OpenVrMatrixToXform(m_hmd->GetEyeToHeadTransform(vr::Eye_Right));
  }
}

RhinoVrDeviceDisplayConduit::RhinoVrDeviceDisplayConduit()
  : CRhinoDisplayConduit(
    CSupportChannels::SC_CALCCLIPPINGPLANES |
    CSupportChannels::SC_CALCBOUNDINGBOX |
    CSupportChannels::SC_POSTDRAWOBJECTS)
  , m_frus_near_suggestion(-1.0)
  , m_frus_far_suggestion(-1.0)
  , m_draw_device_mesh(false)
  , m_device_mesh(nullptr)
  , m_device_mesh_xform(ON_Xform::IdentityTransformation)
  , m_device_cache_handle(nullptr)
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

          //ON_Sphere sphere = ON_Sphere(m_end_pts[i], 0.01);
          //dp.DrawSphere(sphere);
        }

        dp.DrawMesh(*m_device_mesh, false, true, m_device_cache_handle);

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
}

RhinoVrHiddenAreaMeshDisplayConduit::RhinoVrHiddenAreaMeshDisplayConduit()
  : CRhinoDisplayConduit(
    //CSupportChannels::SC_CALCCLIPPINGPLANES |
    //CSupportChannels::SC_CALCBOUNDINGBOX |
    CSupportChannels::SC_PREDRAWOBJECTS)
  , m_draw_hidden_area_mesh(false)
  , m_hidden_area_mesh_left(nullptr)
  , m_hidden_area_mesh_right(nullptr)
  , m_active_eye(vr::Eye_Left)
  , m_hidden_area_mesh_left_xform(ON_Xform::IdentityTransformation)
  , m_hidden_area_mesh_right_xform(ON_Xform::IdentityTransformation)
  , m_hidden_area_mesh_left_cache_handle(nullptr)
  , m_hidden_area_mesh_right_cache_handle(nullptr)
  //, m_bounding_box(ON_BoundingBox::UnsetBoundingBox)
{
}

bool RhinoVrHiddenAreaMeshDisplayConduit::ExecConduit(CRhinoDisplayPipeline & dp, UINT nActiveChannel, bool & bTerminateChannel)
{
  if (m_draw_hidden_area_mesh)
  {
    if (nActiveChannel == CSupportChannels::SC_PREDRAWOBJECTS)
    {
      if (m_pDisplayAttrs->m_bShadeSurface)
      {
        if (m_active_eye == vr::Eye_Left && m_hidden_area_mesh_left)
        {
          dp.PushModelTransform(m_hidden_area_mesh_left_xform);
          dp.DrawMesh(*m_hidden_area_mesh_left, false, true, m_hidden_area_mesh_left_cache_handle);
          dp.PopModelTransform();
        }
        else if (m_active_eye == vr::Eye_Right && m_hidden_area_mesh_right)
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

void RhinoVrHiddenAreaMeshDisplayConduit::SetActiveEye(vr::EVREye active_eye)
{
  m_active_eye = active_eye;
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

RhinoVrDeviceModel::RhinoVrDeviceModel(const ON_String& sRenderModelName)
  : m_device_name(sRenderModelName)
{
}

bool RhinoVrDeviceModel::Init(const vr::RenderModel_t& model, const vr::RenderModel_TextureMap_t& diffuse_texture, double unit_scale)
{
  m_device_mesh.Destroy();
  m_device_mesh.m_V.SetCapacity(model.unVertexCount);
  m_device_mesh.m_N.SetCapacity(model.unVertexCount);
  m_device_mesh.m_T.SetCapacity(model.unVertexCount);

  m_device_mesh.m_V.SetCount(model.unVertexCount);
  m_device_mesh.m_N.SetCount(model.unVertexCount);
  m_device_mesh.m_T.SetCount(model.unVertexCount);

  for (unsigned int vi = 0; vi < model.unVertexCount; ++vi)
  {
    const vr::RenderModel_Vertex_t& rm_vertex = model.rVertexData[vi];

    m_device_mesh.m_V[vi] = unit_scale * ON_3fPoint((const float*)(&rm_vertex.vPosition));
    m_device_mesh.m_N[vi] = ON_3fPoint((const float*)(&rm_vertex.vNormal));
    m_device_mesh.m_T[vi] = ON_2fPoint((const float*)(&rm_vertex.rfTextureCoord));
  }

  m_device_mesh.m_F.SetCapacity(model.unTriangleCount);
  m_device_mesh.m_F.SetCount(model.unTriangleCount);

  const int index_count = model.unTriangleCount * 3;

  for (unsigned int fi = 0, idx = 0; fi < model.unTriangleCount; ++fi)
  {
    ON_MeshFace& triangle = m_device_mesh.m_F[fi];

    triangle.vi[0] = model.rIndexData[idx++];
    triangle.vi[1] = model.rIndexData[idx++];
    triangle.vi[2] = model.rIndexData[idx++];
    triangle.vi[3] = triangle.vi[2];
  }

  //// create and populate the texture
  //glGenTextures(1, &m_glTexture);
  //glBindTexture(GL_TEXTURE_2D, m_glTexture);

  //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, vrDiffuseTexture.unWidth, vrDiffuseTexture.unHeight,
  //  0, GL_RGBA, GL_UNSIGNED_BYTE, vrDiffuseTexture.rubTextureMapData);

  //// If this renders black ask McJohn what's wrong.
  //glGenerateMipmap(GL_TEXTURE_2D);

  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

  //GLfloat fLargest;
  //glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &fLargest);
  //glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, fLargest);

  //glBindTexture(GL_TEXTURE_2D, 0);

  //m_unVertexCount = vrModel.unTriangleCount * 3;

  return true;
}

const ON_String& RhinoVrDeviceModel::GetName() const
{
  return m_device_name;
}
