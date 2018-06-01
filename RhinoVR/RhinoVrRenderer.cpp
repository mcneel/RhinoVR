#include "stdafx.h"
#include "RhinoVrRenderer.h"
#include <gl/GL.h>

//#define RHINOVR_TIMING_ENABLED

#pragma comment(lib, "../OpenVR/lib/win64/openvr_api.lib")

RhinoVrRenderer::RhinoVrRenderer(unsigned int doc_sn, unsigned int view_sn)
  : m_doc_sn(doc_sn)
  , m_view_sn(view_sn)
  , m_doc(nullptr)
  , m_view(nullptr)
  , m_vr_vp(nullptr)
  , m_vr_dp(nullptr)
  , m_vr_dp_ogl(nullptr)
  , m_hmd(nullptr)
  , m_render_models(nullptr)
  , m_compositor(nullptr)
  , m_near_clip(1.0f)
  , m_far_clip(100.0f)
  , m_left_eye_xform(ON_Xform::IdentityTransformation)
  , m_right_eye_xform(ON_Xform::IdentityTransformation)
  , m_hmd_xform(ON_Xform::IdentityTransformation)
  , m_hmd_location_correction_xform(ON_Xform::IdentityTransformation)
  , m_cam_to_world_xform(ON_Xform::Unset)
  , m_pointer_line(ON_Line::UnsetLine)
  , m_hmd_location_correction_acquired(false)
  , m_unit_scale(1.0)
  , m_frame_time_start(0)
  , m_rhino_time_start(0)
  , m_vsync_time_start(0)
{
  memset(m_device_poses, 0, sizeof(m_device_poses));
  
  m_device_data.SetCapacity(vr::k_unMaxTrackedDeviceCount);
  m_device_data.SetCount(vr::k_unMaxTrackedDeviceCount);
}

RhinoVrRenderer::~RhinoVrRenderer()
{
  CWnd* main_window = CWnd::FromHandle(RhinoApp().MainWnd());
  if (main_window)
    main_window->KillTimer(2029);

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
    m_vr_vp->m_v.m_vp.ChangeViewportId(ON_nil_uuid);
  }

  RhinoApp().Print(L"RhinoVR is OFF.\n");
}

bool RhinoVrRenderer::Initialize()
{
  RhinoApp().Print(L"Initializing RhinoVR...\n");

  vr::EVRInitError ovr_error = vr::VRInitError_None;

  m_hmd = vr::VR_Init(&ovr_error, vr::VRApplication_Scene);

  if (m_hmd == nullptr || ovr_error != vr::VRInitError_None)
  {
    ON_String str;
    str.Format("Unable to initialize VR runtime: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(ovr_error));
    RhinoApp().Print(str);
    return false;
  }

  m_render_models = (vr::IVRRenderModels*) vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, &ovr_error);
  if (m_render_models == nullptr || ovr_error != vr::VRInitError_None)
  {
    ON_String str;
    str.Format("Unable to get render models interface: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(ovr_error));
    RhinoApp().Print(str);
    return false;
  }

  m_compositor = (vr::IVRCompositor*) vr::VR_GetGenericInterface(vr::IVRCompositor_Version, &ovr_error);
  if (m_compositor == nullptr || ovr_error != vr::VRInitError_None)
  {
    ON_String str;
    str.Format("Unable to initialize VR compositor : %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(ovr_error));
    RhinoApp().Print(str);
  }

  CRhinoDoc* rhino_doc = m_doc = CRhinoDoc::FromRuntimeSerialNumber(m_doc_sn);
  if (rhino_doc == nullptr)
    return false;
  
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

  CRhinoDisplayPipeline_OGL* view_dp_ogl = dynamic_cast<CRhinoDisplayPipeline_OGL*>(view_dp);
  if (view_dp_ogl == nullptr)
    return false;

  m_vr_vp = std::make_unique<CRhinoViewport>();
  m_vr_vp->CopyFrom(view->Viewport(), true);
  m_vr_vp->SetScreenSize(rec_width, rec_height);

  view_dp_ogl->OpenPipeline();
  m_vr_dp = std::unique_ptr<CRhinoDisplayPipeline>(view_dp_ogl->ClonePipeline(*m_vr_vp));
  view_dp_ogl->ClosePipeline();

  if (m_vr_dp == nullptr)
    return false;

  m_vr_dp_ogl = dynamic_cast<CRhinoDisplayPipeline_OGL*>(m_vr_dp.get());
  if (m_vr_dp_ogl == nullptr)
    return false;

  ON_Viewport vp = m_vp_orig = m_vr_dp->VP();

  // We need to be able to see objects that are 1 cm (0.01 m) in front of us.
  m_near_clip = (float)(m_unit_scale * 0.01);
  m_far_clip = (float)(m_near_clip / vp.PerspectiveMinNearOverFar());

  m_hmd->GetProjectionRaw(vr::Eye_Left,
    &m_left_frus_left, &m_left_frus_right, &m_left_frus_top, &m_left_frus_bottom);

  m_left_frus_left   *= m_near_clip;
  m_left_frus_right  *= m_near_clip;
  m_left_frus_top    *= m_near_clip;
  m_left_frus_bottom *= m_near_clip;

  m_hmd->GetProjectionRaw(vr::Eye_Right,
    &m_right_frus_left, &m_right_frus_right, &m_right_frus_top, &m_right_frus_bottom);

  m_right_frus_left   *= m_near_clip;
  m_right_frus_right  *= m_near_clip;
  m_right_frus_top    *= m_near_clip;
  m_right_frus_bottom *= m_near_clip;

  // We have asymmetric frustums
  vp.SetFrustumLeftRightSymmetry(false);
  vp.SetFrustumTopBottomSymmetry(false);
  vp.SetFrustum(
    m_left_frus_left, m_right_frus_right,
    m_left_frus_top, m_left_frus_bottom,
    m_near_clip, m_far_clip);

  m_vp_orig_hmd_frus = vp;

  {
    // We imitate the VR view in the Rhino viewport.
    int vp_width = vp.ScreenPortWidth();

    double frus_aspect;
    vp.GetFrustumAspect(frus_aspect);

    int vp_new_width = vp_width / 2;
    int vp_new_height = (int)floor(vp_new_width / frus_aspect + 0.5);

    ON_wString script;
    script.Format(L"-_ViewportProperties _Size %d %d _Enter", vp_new_width, vp_new_height);

    RhinoApp().RunScriptEx(m_doc_sn, script, nullptr, 0);

    view->ActiveViewport().SetVP(vp, TRUE);
    view->Redraw();
  }

  // ATTENTION: The following lines are a (hopefully temporary) hack.
  // Rhino uses MFC, and MFC has a main message loop which calls
  // GetMessage(). If there are no messages in the message queue,
  // GetMessage() will block until it finds a message. This is
  // unacceptable since we want Rhino/MFC to call into RhinoVR as
  // quickly and often as possible. To get around this, we start a
  // timer which will always put a WM_TIMER message in the message
  // queue if it is empty.
  CWnd* main_window = CWnd::FromHandle(RhinoApp().MainWnd());
  if(main_window)
    main_window->SetTimer(2029, 0, NULL);

  RhinoApp().Print(L"RhinoVR is ON. Please put on the VR headset.\n");

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
  if (m_render_models == nullptr || m_doc == nullptr)
    return nullptr;

  RhinoVrDeviceModel* render_model = nullptr;
  for (int i = 0; i < m_device_render_models.Count(); i++)
  {
    RhinoVrDeviceModel* dm = m_device_render_models[i].get();

    if (dm->m_device_name.EqualOrdinal(render_model_name, false))
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

    std::unique_ptr<RhinoVrDeviceModel>& render_model_unique = m_device_render_models.AppendNew();
    render_model_unique.reset(new RhinoVrDeviceModel(render_model_name));

    render_model = render_model_unique.get();
    if (!render_model->Initialize(*model, *texture, m_unit_scale, *m_doc))
    {
      m_device_render_models.Remove();
      RhinoApp().Print("Unable to create Rhino Mesh model from render model %s\n", render_model_name);
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
    RhinoVrDeviceData& device_data = m_device_data[device_index];
    device_data.m_render_model = render_model;
    device_data.m_show = true;

    // We don't want to show the Vive base stations or the Rift cameras.
    if (render_model_name.EqualOrdinal("lh_basestation_vive", false) ||
        render_model_name.EqualOrdinal("rift_camera", false))
    {
      device_data.m_show = false;
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

  for (uint32_t device_idx = vr::k_unTrackedDeviceIndex_Hmd + 1; device_idx < vr::k_unMaxTrackedDeviceCount; device_idx++)
  {
    if (!m_hmd->IsTrackedDeviceConnected(device_idx))
      continue;

    SetupRenderModelForDevice(device_idx);
  }

  m_hidden_mesh_left = LoadHiddenAreaMesh(vr::Eye_Left);
  m_hidden_mesh_right = LoadHiddenAreaMesh(vr::Eye_Right);
}

void RhinoVrRenderer::UpdateDeviceDisplayConduits(
  const ON_Xform& camera_to_world_xform,
  const ON_Xform& clip_to_left_eye_xform,
  const ON_Xform& clip_to_right_eye_xform)
{
  bool is_input_available = m_hmd->IsInputAvailable();

  for (uint32_t device_idx = 0; device_idx < vr::k_unMaxTrackedDeviceCount; device_idx++)
  {
    RhinoVrDeviceData& device_data = m_device_data[device_idx];

    RhinoVrDeviceDisplayConduit& ddc = device_data.m_display_conduit;
    ddc.Empty();

    RhinoVrDeviceModel* device_model = device_data.m_render_model;

    if (!device_model || !device_data.m_show)
      continue;

    const vr::TrackedDevicePose_t& pose = m_device_poses[device_idx];
    if (!pose.bPoseIsValid)
      continue;

    const bool is_controller = (m_hmd->GetTrackedDeviceClass(device_idx) == vr::TrackedDeviceClass_Controller);

    if (!is_input_available && is_controller)
      continue;

    if (is_controller)
    {
      ddc.AddLine(m_pointer_line.from, m_pointer_line.to, ON_Color::SaturatedGreen);
    }

    const ON_Mesh& device_mesh = device_model->m_device_mesh;
    const CDisplayPipelineMaterial& device_material = device_model->m_device_material;
    const ON_Xform device_xform = camera_to_world_xform * device_data.m_xform;
    CRhinoCacheHandle& device_cache_handle = device_model->m_cache_handle;
    
    ddc.SetDeviceMesh(&device_mesh);
    ddc.SetDeviceMaterial(&device_material);
    ddc.SetDeviceMeshXform(device_xform);
    ddc.SetDeviceMeshCacheHandle(&device_cache_handle);

    if (!ddc.IsEnabled())
    {
      ddc.SetFrustumNearFarSuggestion(m_near_clip, m_far_clip);
      ddc.Enable(m_doc_sn);
    }
  }

  m_hidden_mesh_display_conduit.SetHiddenAreaMesh(&m_hidden_mesh_left, vr::Eye_Left);
  m_hidden_mesh_display_conduit.SetHiddenAreaMesh(&m_hidden_mesh_right, vr::Eye_Right);

  m_hidden_mesh_display_conduit.SetHiddenAreaMeshXform(clip_to_left_eye_xform, vr::Eye_Left);
  m_hidden_mesh_display_conduit.SetHiddenAreaMeshXform(clip_to_right_eye_xform, vr::Eye_Right);

  m_hidden_mesh_display_conduit.SetHiddenAreaMeshCacheHandle(&m_hidden_mesh_left_cache_handle, vr::Eye_Left);
  m_hidden_mesh_display_conduit.SetHiddenAreaMeshCacheHandle(&m_hidden_mesh_right_cache_handle, vr::Eye_Right);

  if (!m_hidden_mesh_display_conduit.IsEnabled())
  {
    m_hidden_mesh_display_conduit.Enable(m_doc_sn);
  }
}

bool RhinoVrRenderer::UpdatePosesAndWaitForVSync()
{
  if (m_compositor == nullptr)
    return false;

  // WaitGetPoses waits for vertical sync
  m_compositor->WaitGetPoses(m_device_poses, vr::k_unMaxTrackedDeviceCount, nullptr, 0);

  return true;
}

bool RhinoVrRenderer::UpdateState()
{
  if (m_hmd == nullptr || m_view == nullptr || m_doc == nullptr)
  {
    return false;
  }

  ON_2dPoint touchpad_point = ON_2dPoint::Origin;

  for (vr::TrackedDeviceIndex_t device_idx = 0; device_idx < vr::k_unMaxTrackedDeviceCount; device_idx++)
  {
    if (m_hmd->GetTrackedDeviceClass(device_idx) != vr::TrackedDeviceClass_Controller)
      continue;

    vr::VRControllerState_t state;
    if (m_hmd->GetControllerState(device_idx, &state, sizeof(state)))
    {
      RhinoVrDeviceController& controller = m_device_data[device_idx].m_controller;
      GetRhinoVrControllerState(state, controller);

      ON_2dPoint tpp = controller.m_touchpad_touch_point;

      if (tpp != ON_2dPoint::Origin)
      {
        if (abs(tpp.x) > abs(touchpad_point.x))
          touchpad_point.x = tpp.x;

        if (abs(tpp.y) > abs(touchpad_point.y))
          touchpad_point.y = tpp.y;
      }
    }
  }

  double camera_rotation = 0.0;
  double camera_translation_distance = 0.0;

  if (touchpad_point != ON_2dPoint::Origin)
  {
    if (!m_doc->InCommand())
    {
      // If both X and Y magnitudes are under 0.6 then we don't move/rotate.
      // In other words, the touchpad needs to be touched close to the edge.
      const double threshold = 0.6;

      double x = touchpad_point.x;
      double x_sign = (x >= 0.0 ? 1.0 : -1.0);

      if (abs(x) >= threshold)
      {
        double x_scaled = x_sign * (abs(x) - threshold)*(1.0 / (1.0 - threshold));
        camera_rotation = -2.0*x_scaled*ON_DEGREES_TO_RADIANS;
      }

      double y = touchpad_point.y;
      double y_sign = (y >= 0.0 ? 1.0 : -1.0);

      if (abs(y) >= threshold)
      {
        double y_scaled = y_sign * (abs(y) - threshold)*(1.0 / (1.0 - threshold));

        camera_translation_distance = 0.25*y_scaled*m_unit_scale;
      }
    }
  }

  const uint32_t hmd_device_index = vr::k_unTrackedDeviceIndex_Hmd;

  vr::TrackedDevicePose_t& hmd_device_pose = m_device_poses[hmd_device_index];

  if (!m_hmd_location_correction_acquired && hmd_device_pose.bPoseIsValid)
  {
    m_hmd_location_correction_acquired = true;

    ON_Xform xform = OpenVrMatrixToXform(hmd_device_pose.mDeviceToAbsoluteTracking);
    m_hmd_location_correction_xform = ON_Xform::TranslationTransformation(ON_3dVector(-xform[0][3], -xform[1][3], -xform[2][3]));
  }

  for (int device_idx = 0; device_idx < vr::k_unMaxTrackedDeviceCount; ++device_idx)
  {
    vr::TrackedDevicePose_t& device_pose = m_device_poses[device_idx];

    if (device_pose.bPoseIsValid)
    {
      const vr::HmdMatrix34_t& ovr_device_matrix = device_pose.mDeviceToAbsoluteTracking;
      m_device_data[device_idx].m_xform = m_hmd_location_correction_xform * OpenVrMatrixToXform(ovr_device_matrix);
    }
  }

  if (hmd_device_pose.bPoseIsValid)
  {
    m_hmd_xform = m_device_data[hmd_device_index].m_xform;

    m_left_eye_xform  = OpenVrMatrixToXform(m_hmd->GetEyeToHeadTransform(vr::Eye_Left));
    m_right_eye_xform = OpenVrMatrixToXform(m_hmd->GetEyeToHeadTransform(vr::Eye_Right));
  }

  const ON_Viewport& rhino_vp = m_view->ActiveViewport().VP();

  ON_Viewport vp = m_vp_orig_hmd_frus;
  vp.SetProjection     (rhino_vp.Projection());
  vp.SetCameraLocation ( ON_3dPoint::Origin);
  vp.SetCameraDirection(-ON_3dVector::ZAxis);
  vp.SetCameraUp       ( ON_3dVector::YAxis);
  vp.SetTargetPoint    (-ON_3dVector::ZAxis*rhino_vp.TargetDistance(true));
  vp.SetFrustumNearFar(m_near_clip, m_far_clip);

  if (m_cam_to_world_xform == ON_Xform::Unset)
  {
    // If not set, initialize to Rhino camera xform.
    rhino_vp.GetXform(ON::coordinate_system::camera_cs, ON::coordinate_system::world_cs, m_cam_to_world_xform);
  }

  m_vp_hmd = vp;

  ON_Xform hmd_to_world_xform = m_cam_to_world_xform * m_hmd_xform;

  // Transform the HMD to it's world position last frame.
  m_vp_hmd.Transform(hmd_to_world_xform);

  // Apply rotation due to controller.
  ON_3dPoint hmd_loc = m_vp_hmd.CameraLocation();
  m_vp_hmd.Rotate(camera_rotation, ON_3dVector::ZAxis, hmd_loc);

  // Apply translation due to controller.
  ON_3dVector hmd_dir   = m_vp_hmd.CameraDirection();
  ON_3dVector hmd_dolly = camera_translation_distance * hmd_dir;
  m_vp_hmd.DollyCamera(hmd_dolly);
  m_vp_hmd.DollyFrustum(hmd_dolly.z);

  // Now extract the final xform which includes movements from the controller.
  ON_Xform hmd_to_world_final_xform;
  m_vp_hmd.GetXform(ON::coordinate_system::camera_cs, ON::coordinate_system::world_cs, hmd_to_world_final_xform);

  // We get the controller-only xform by "subtracting" away hmd_to_world.
  ON_Xform controller_xform = hmd_to_world_final_xform * hmd_to_world_xform.Inverse();

  // We update cam_to_world to incorporate the controller xform.
  m_cam_to_world_xform = controller_xform * m_cam_to_world_xform;

  m_vr_vp->SetVP(m_vp_hmd, TRUE);

  ON_Xform cam_to_left_eye_xform = hmd_to_world_final_xform * m_left_eye_xform;

  m_vp_left_eye = vp;

  m_vp_left_eye.Transform(cam_to_left_eye_xform);
  m_vp_left_eye.SetFrustum(
    m_left_frus_left, m_left_frus_right,
    m_left_frus_top, m_left_frus_bottom,
    m_near_clip, m_far_clip);

  ON_Xform clip_to_left_eye_xform;
  m_vp_left_eye.GetXform(ON::coordinate_system::clip_cs, ON::coordinate_system::world_cs, clip_to_left_eye_xform);

  ON_Xform cam_to_right_eye_xform = hmd_to_world_final_xform * m_right_eye_xform;

  m_vp_right_eye = vp;

  m_vp_right_eye.Transform(cam_to_right_eye_xform);
  m_vp_right_eye.SetFrustum(
    m_right_frus_left, m_right_frus_right,
    m_right_frus_top, m_right_frus_bottom,
    m_near_clip, m_far_clip);

  ON_Xform clip_to_right_eye_xform;
  m_vp_right_eye.GetXform(ON::coordinate_system::clip_cs, ON::coordinate_system::world_cs, clip_to_right_eye_xform);

  UpdateDeviceDisplayConduits(m_cam_to_world_xform, clip_to_left_eye_xform, clip_to_right_eye_xform);

  return true;
}

bool RhinoVrRenderer::Draw()
{
  if (m_compositor == nullptr || m_view == nullptr || m_vr_dp_ogl == nullptr)
    return false;

  CDisplayPipelineAttributes* vr_dpa = m_view->DisplayAttributes();

  if (vr_dpa == nullptr)
    return false;

  unsigned long long eye_left_handle = 0;
  unsigned long long eye_right_handle = 0;

  bool draw_success = m_vr_dp_ogl->DrawStereoFrameBuffer(*vr_dpa, m_vp_left_eye, m_vp_right_eye, eye_left_handle, eye_right_handle);
  
  if (!draw_success)
    return false;

  // The pipeline needs to be opened for compositor to work.
  m_vr_dp->OpenPipeline();

  vr::EVRCompositorError ovr_error = vr::VRCompositorError_None;

  vr::Texture_t leftEyeTexture = { (void*)(uintptr_t)eye_left_handle, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
  ovr_error = m_compositor->Submit(vr::Eye_Left, &leftEyeTexture);
  if (ovr_error != vr::VRCompositorError_None && ovr_error != vr::VRCompositorError_DoNotHaveFocus)
  {
    m_vr_dp->ClosePipeline();

    return false;
  }

  vr::Texture_t rightEyeTexture = { (void*)(uintptr_t)eye_right_handle, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
  ovr_error = m_compositor->Submit(vr::Eye_Right, &rightEyeTexture);
  if (ovr_error != vr::VRCompositorError_None && ovr_error != vr::VRCompositorError_DoNotHaveFocus)
  {
    m_vr_dp->ClosePipeline();

    return false;
  }

#ifdef RHINOVR_TIMING_ENABLED
  ::glFlush();
  ::glFinish();
#endif

  m_vr_dp->ClosePipeline();

  return true;
}

void RhinoVrRenderer::ProcessVrEvent(const vr::VREvent_t & event)
{
  switch (event.eventType)
  {
  case vr::VREvent_TrackedDeviceActivated:
  {
    SetupRenderModelForDevice(event.trackedDeviceIndex);
  }
  break;
  case vr::VREvent_TrackedDeviceDeactivated:
  {
    // React to device being deactivated
  }
  break;
  case vr::VREvent_TrackedDeviceUpdated:
  {
    // React to device being updated
  }
  break;
  }
}

bool RhinoVrRenderer::GetWorldPickLineAndClipRegion(
  const ON_Xform& picking_device_xform,
  ON_Line& world_line,
  ON_ClippingRegion& clip_region,
  ON_Viewport& line_vp,
  ON_2iPoint& line_pixel)
{
  line_vp = m_vp_hmd;

  line_vp.SetCameraLocation(ON_3dPoint::Origin);
  line_vp.SetCameraDirection(-ON_3dVector::ZAxis);
  line_vp.SetCameraUp(ON_3dVector::YAxis);
  line_vp.Transform(m_cam_to_world_xform*picking_device_xform);

  int l, r, b, t;
  line_vp.GetScreenPort(&l, &r, &b, &t);

  // Make screen port an uneven amount of pixels.
  // This way the center pixel will be truly at
  // center of the screen, which in turn will 
  // make the pick point as accurate as possible.
  if (r % 2 == 0) r += 1;
  if (b % 2 == 0) b += 1;

  line_vp.SetScreenPort(l, r, b, t);

  // We need to force the frustum to be symmetric, otherwise the 
  // center pixel will not be exactly where the pick line is.
  double lf, rf, bf, tf, nf, ff;
  line_vp.GetFrustum(&lf, &rf, &bf, &tf, &nf, &ff);

  double new_tbf = max(abs(bf), abs(tf));
  bf = -new_tbf;
  tf =  new_tbf;

  line_vp.SetFrustum(lf, rf, bf, tf, nf, ff);

  double frus_near = m_unit_scale * 0.01;
  line_vp.SetFrustumNearFar(frus_near, frus_near / line_vp.PerspectiveMinNearOverFar());

  line_pixel = ON_2iPoint(line_vp.ScreenPortWidth() / 2 + 1, line_vp.ScreenPortHeight() / 2 + 1);

  m_vr_vp->SetVP(line_vp, TRUE);
  m_vr_vp->SetClippingRegionTransformation(line_pixel.x, line_pixel.y, clip_region);
  m_vr_vp->SetVP(m_vp_hmd, TRUE);

  line_vp.GetFrustumLine(line_pixel.x, line_pixel.y, world_line);

  return true;
}

void RhinoVrRenderer::GetRhinoVrControllerState(
  const vr::VRControllerState_t& state,
  RhinoVrDeviceController& controller)
{
  {
    static uint64_t touchpad_btn_mask = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);

    bool was_down   = controller.m_touchpad_button_down;
    bool is_down    = (state.ulButtonPressed & touchpad_btn_mask);
    bool is_touched = (state.ulButtonTouched & touchpad_btn_mask);

    controller.m_touchpad_button_down     = is_down;
    controller.m_touchpad_button_pressed  = is_down && !was_down;
    controller.m_touchpad_button_released = !is_down && was_down;
    controller.m_touchpad_button_touched  = is_touched;
    controller.m_touchpad_touch_point     = ON_2dPoint(state.rAxis[0].x, state.rAxis[0].y);
  }

  {
    static uint64_t appmenu_btn_mask = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_ApplicationMenu);

    bool was_down = controller.m_appmenu_button_down;
    bool is_down  = (state.ulButtonPressed & appmenu_btn_mask);

    controller.m_appmenu_button_down     = is_down;
    controller.m_appmenu_button_pressed  = is_down && !was_down;
    controller.m_appmenu_button_released = !is_down && was_down;
  }

  {
    static uint64_t grip_btn_mask = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_Grip);

    bool was_down = controller.m_grip_button_down;
    bool is_down  = (state.ulButtonPressed & grip_btn_mask);

    controller.m_grip_button_down     = is_down;
    controller.m_grip_button_pressed  = is_down && !was_down;
    controller.m_grip_button_released = !is_down && was_down;
  }

  {
    float value   = state.rAxis[1].x;
    bool was_down = controller.m_grip_button_down;
    bool is_down  = (value == 1.0f);

    controller.m_trigger_button_down     = is_down;
    controller.m_trigger_button_pressed  = is_down && !was_down;
    controller.m_trigger_button_released = !is_down && was_down;
    controller.m_trigger_button_value    = value;
  }
}

void RhinoVrRenderer::RhinoVrGetPoint(const ON_Xform& picking_device_xform)
{
  CRhinoGetPoint* gp = m_doc->InGetPoint();
  if (gp == nullptr)
    return;

  ON_Line world_line;
  ON_ClippingRegion clip_region;
  ON_Viewport line_vp;
  ON_2iPoint line_pixel;

  if (GetWorldPickLineAndClipRegion(picking_device_xform, world_line, clip_region, line_vp, line_pixel))
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

      LPARAM nFlags = 0;

      m_view->PostDigitizerPointEvent(ray, nFlags);
    }

    rhino_vp.SetVP(orig_vp, TRUE);
    rhino_vp.SetScreenSize(orig_width, orig_height);
  }
}

void RhinoVrRenderer::RhinoVrOnMouseMove(const ON_Xform& picking_device_xform)
{
  CRhinoGetPoint* gp = m_doc->InGetPoint();
  if (gp == nullptr)
    return;

  ON_Line world_line;
  ON_ClippingRegion clip_region;
  ON_Viewport line_vp;
  ON_2iPoint line_pixel;

  if (GetWorldPickLineAndClipRegion(picking_device_xform, world_line, clip_region, line_vp, line_pixel))
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

int SortObjRefBySelDist(const CRhinoObjRef* a, const CRhinoObjRef* b)
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

bool RhinoVrRenderer::RhinoVrGetIntersectingObject(const ON_Xform& picking_device_xform, const CRhinoObject*& isect_object, ON_3dPoint& isect_point)
{
  ON_Viewport line_vp;
  ON_2iPoint line_pixel;

  CRhinoPickContext pc;
  pc.m_view = m_view;
  pc.m_pick_mode = CRhinoPickContext::shaded_pick;
  pc.m_pick_style = CRhinoPickContext::point_pick;

  if (GetWorldPickLineAndClipRegion(picking_device_xform, pc.m_pick_line, pc.m_pick_region, line_vp, line_pixel))
  {
    pc.UpdateClippingPlanes();

    CRhinoObjRefArray rhino_objs;
    m_doc->PickObjects(pc, rhino_objs);

    rhino_objs.QuickSort(SortObjRefBySelDist);

    if (rhino_objs.Count() > 0)
    {
      CRhinoObjRef objref = rhino_objs[0];
      ON_3dPoint isect_pt;

      if (objref.Object() && objref.SelectionPoint(isect_pt) && isect_pt.IsValid())
      {
        isect_object = objref.Object();
        isect_point = isect_pt;

        return true;
      }
    }
  }

  return false;
}

bool RhinoVrRenderer::HandleInput()
{
  if (m_hmd == nullptr || m_doc == nullptr || m_view == nullptr)
    return false;

  vr::VREvent_t event;
  while (m_hmd->PollNextEvent(&event, sizeof(event)))
  {
    ProcessVrEvent(event);
  }

  for (vr::TrackedDeviceIndex_t device_idx = 0; device_idx < vr::k_unMaxTrackedDeviceCount; device_idx++)
  {
    if (m_hmd->GetTrackedDeviceClass(device_idx) != vr::TrackedDeviceClass_Controller)
      continue;

    RhinoVrDeviceData& device_data = m_device_data[device_idx];
    RhinoVrDeviceController& controller = device_data.m_controller;

    if (controller.m_touchpad_button_pressed)
    {
      if (m_doc->InGetPoint())
      {
        RhinoVrGetPoint(device_data.m_xform);
      }
      else
      {
        const CRhinoObject* isect_object = nullptr;
        ON_3dPoint isect_point = ON_3dPoint::UnsetPoint;

        if (RhinoVrGetIntersectingObject(device_data.m_xform, isect_object, isect_point))
        {
          if (m_doc->InGetObject())
          {
            CRhinoGetObject* go = m_doc->InGetObject();

            CRhinoObjRefArray& obj_ref_array = const_cast<CRhinoObjRefArray&>(go->PickList());
            obj_ref_array.Append(CRhinoObjRef(isect_object));

            go->PostObjectSelectionChangedEvent(m_view);
          }
          else
          {
            isect_object->Select();
          }
        }
      }
    }
    else if (controller.m_appmenu_button_pressed)
    {
      RhinoApp().ExecuteCommand(m_doc_sn, L"_Move");

      // Need to re-attach since ExecuteCommand pumps messages
      // and can basically do anything, including e.g. deleting views.
      if (!AttachDocAndView())
        return false;
    }
    else if (controller.m_grip_button_pressed)
    {
      RhinoApp().ExecuteCommand(m_doc_sn, L"_Cancel");

      // Need to re-attach since ExecuteCommand pumps messages
      // and can basically do anything, including e.g. deleting views.
      if (!AttachDocAndView())
        return false;
    }
    else if (controller.m_trigger_button_pressed)
    {
      RhinoApp().ExecuteCommand(m_doc_sn, L"_Enter");

      // Need to re-attach since ExecuteCommand pumps messages
      // and can basically do anything, including e.g. deleting views.
      if (!AttachDocAndView())
        return false;
    }
    else
    {
      if (m_doc->InGetPoint())
      {
        RhinoVrOnMouseMove(device_data.m_xform);
      }
    }
  }

  return true;
}

void RhinoVrRenderer::ProcessInputAndRenderFrame()
{
  RhinoTimingStop();

  if (AttachDocAndView())
  {
    VsyncTimingStart();

    if (!UpdatePosesAndWaitForVSync())
      return;

    VsyncTimingStop();
    FrameTimingStart();

    if (!UpdateState())
      return;

    if (!HandleInput())
      return;

    if (!Draw())
      return;

    FrameTimingStop();

    DetachDocAndView();
  }

  RhinoTimingStart();
}

bool RhinoVrRenderer::AttachDocAndView()
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

void RhinoVrRenderer::DetachDocAndView()
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

RhinoVrDeviceModel::RhinoVrDeviceModel(const ON_String& sRenderModelName)
  : m_device_name(sRenderModelName)
{
}

bool RhinoVrDeviceModel::Initialize(
  const vr::RenderModel_t& model,
  const vr::RenderModel_TextureMap_t& diffuse_texture,
  double unit_scale,
  const CRhinoDoc& doc)
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

  CRhinoDib texture_dib(diffuse_texture.unWidth, diffuse_texture.unHeight, 32);

  texture_dib.ProcessPixels_SingleThreaded(
    [](CRhinoDib::Pixel& pixel, void* pvData)
    {
      const vr::RenderModel_TextureMap_t* texdata = (const vr::RenderModel_TextureMap_t*)pvData;
      size_t offset = (size_t)(4*(pixel.y*texdata->unWidth + pixel.x));
      const uint8_t* data_offset = texdata->rubTextureMapData + offset;
      
      unsigned char r = (unsigned char)data_offset[0];
      unsigned char g = (unsigned char)data_offset[1];
      unsigned char b = (unsigned char)data_offset[2];
      unsigned char a = (unsigned char)data_offset[3];

      pixel.Set(r, g, b, a);
    },
    (void*)&diffuse_texture );

  
  CRhRdkTexture* rdk_texture = ::RhRdkNewDibTexture(&texture_dib, &doc);

  CRhRdkBasicMaterial* rdk_material = new CRhRdkBasicMaterial();
  rdk_material->Initialize();

  ON_wString child_slot_name = rdk_material->TextureChildSlotName(CRhRdkMaterial::ChildSlotUsage::Diffuse);

  rdk_material->SetChild(rdk_texture, child_slot_name);
  rdk_material->SetChildSlotOn(child_slot_name, true);
  rdk_material->SetChildSlotAmount(child_slot_name, 100.0);

  m_device_material = rdk_material->SimulatedMaterial();

  rdk_material->Uninitialize();
  delete rdk_material;

  return true;
}

void RhinoVrRenderer::FrameTimingStart()
{
  m_frame_time_start = TimingStart();
}

void RhinoVrRenderer::FrameTimingStop()
{
  TimingStop(m_frame_time_start, L"Frame time");
}

void RhinoVrRenderer::RhinoTimingStart()
{
  m_rhino_time_start = TimingStart();
}

void RhinoVrRenderer::RhinoTimingStop()
{
  TimingStop(m_rhino_time_start, L"Rhino time");
}

void RhinoVrRenderer::VsyncTimingStart()
{
  m_vsync_time_start = TimingStart();
}

void RhinoVrRenderer::VsyncTimingStop()
{
  TimingStop(m_vsync_time_start, L"Vsync time");
}

RhTimestamp RhinoVrRenderer::TimingStart()
{
#ifdef RHINOVR_TIMING_ENABLED
  return RhinoGetTimestamp();
#else
  return (RhTimestamp)0;
#endif
}

#ifdef RHINOVR_TIMING_ENABLED
void RhinoVrRenderer::TimingStop(const RhTimestamp& start_time, const ON_wString& message)
{
  if (start_time == 0)
    return;

  double time_millis = 1000.0*RhinoGetTimeInSecondsSince(start_time);
  
  ON_wString str;
  str.Format(L"%s: %f ms\n", message.Array(), time_millis);

  OutputDebugString(str);
}
#else
void RhinoVrRenderer::TimingStop(const RhTimestamp& /*start_time*/, const ON_wString& /*message*/)
{
}
#endif
