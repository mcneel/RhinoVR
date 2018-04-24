#include "StdAfx.h"
#include "RhinoVrWindow.h"

#pragma comment(lib, "../OpenVR/lib/win64/openvr_api.lib")

void RhinoVrPostDigitizerEvent(ON_3dRay ray, LPARAM nFlags);
void RhinoVrPostGetObjectEvent(CRhinoGetObject* go, CRhinoView* view);
unsigned long long RhinoVrGetEyeTextureHandle(CRhinoDisplayPipeline* dp, int eye);

RhinoVrRenderer::RhinoVrRenderer(unsigned int doc_sn, unsigned int view_sn, unsigned int viewport_sn)
  : m_vr_doc_sn(doc_sn)
  , m_vr_view_sn(view_sn)
  , m_vr_viewport_sn(viewport_sn)
  , m_target_frame_rate(0)
  , m_pHMD(nullptr)
  , m_pRenderModels(nullptr)
  , m_fNearClip(1.0f)
  , m_fFarClip(100.0f)
  , m_mat4eyePosLeft(ON_Xform::IdentityTransformation)
  , m_mat4eyePosRight(ON_Xform::IdentityTransformation)
  , m_mat4ProjectionLeft(ON_Xform::IdentityTransformation)
  , m_mat4ProjectionRight(ON_Xform::IdentityTransformation)
  , m_mat4HMDPose(ON_Xform::IdentityTransformation)
  , m_mat4HMDPoseCorrection(ON_Xform::IdentityTransformation)
  , m_camera_translation(ON_3dVector::ZeroVector)
  , m_pointer_line(ON_3dPoint(0, 0, 0), ON_3dPoint(0, 0, -1))
  , m_trigger_value(0.0f)
  , m_previous_cam_dir(ON_3dVector::UnsetVector)
  , m_camera_rotation(0.0)
  , m_hmd_pose_correction_acquired(false)
  , m_unit_scale(1.0)
{
  memset(m_rDevClassChar, 0, sizeof(m_rDevClassChar));
  memset(m_rbShowTrackedDevice, 0, sizeof(m_rbShowTrackedDevice));
  memset(m_rTrackedDeviceToRenderModel, 0, sizeof(m_rTrackedDeviceToRenderModel));
  memset(m_device_packet_num, 0, sizeof(m_device_packet_num));
  memset(m_device_controllers, 0, sizeof(m_device_controllers));
}

RhinoVrRenderer::~RhinoVrRenderer()
{
  vr::VR_Shutdown();
}

bool RhinoVrRenderer::InitializeVrRenderer()
{
  bool rc = true;

  if (!InitializeVrSystem())
  {
    rc = false;
  }

  return rc;
}

ON_String GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL)
{
  uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
  if (unRequiredBufferLen == 0)
    return "";

  char *pchBuffer = new char[unRequiredBufferLen];
  unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
  ON_String sResult = pchBuffer;
  delete[] pchBuffer;
  return sResult;
}

RhinoVrDeviceModel* RhinoVrRenderer::FindOrLoadRenderModel(const char* pchRenderModelName)
{
  RhinoVrDeviceModel* pRenderModel = nullptr;
  for (auto i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++)
  {
    RhinoVrDeviceModel* dm = *i;

    if (dm->GetName().EqualOrdinal(pchRenderModelName, false))
    {
      pRenderModel = dm;
      break;
    }
  }

  // load the model if we didn't find one
  if (pRenderModel == nullptr)
  {
    vr::RenderModel_t *pModel;
    vr::EVRRenderModelError error;
    while (1)
    {
      error = vr::VRRenderModels()->LoadRenderModel_Async(pchRenderModelName, &pModel);
      if (error != vr::VRRenderModelError_Loading)
        break;

      Sleep(1);
    }

    if (error != vr::VRRenderModelError_None)
    {
      RhinoApp().Print("Unable to load render model %s - %s\n", pchRenderModelName, vr::VRRenderModels()->GetRenderModelErrorNameFromEnum(error));
      return nullptr; // move on to the next tracked device
    }

    vr::RenderModel_TextureMap_t *pTexture;
    while (1)
    {
      error = vr::VRRenderModels()->LoadTexture_Async(pModel->diffuseTextureId, &pTexture);
      if (error != vr::VRRenderModelError_Loading)
        break;

      Sleep(1);
    }

    if (error != vr::VRRenderModelError_None)
    {
      RhinoApp().Print("Unable to load render texture id:%d for render model %s\n", pModel->diffuseTextureId, pchRenderModelName);
      vr::VRRenderModels()->FreeRenderModel(pModel);
      return nullptr; // move on to the next tracked device
    }

    pRenderModel = new RhinoVrDeviceModel(pchRenderModelName);
    if (!pRenderModel->Init(*pModel, *pTexture, m_unit_scale))
    {
      RhinoApp().Print("Unable to create Rhino Mesh model from render model %s\n", pchRenderModelName);
      delete pRenderModel;
      pRenderModel = NULL;
    }
    else
    {
      m_vecRenderModels.push_back(pRenderModel);
    }
    vr::VRRenderModels()->FreeRenderModel(pModel);
    vr::VRRenderModels()->FreeTexture(pTexture);
  }
  return pRenderModel;
}

void RhinoVrRenderer::SetupRenderModelForTrackedDevice(vr::TrackedDeviceIndex_t unTrackedDeviceIndex)
{
  if (unTrackedDeviceIndex >= vr::k_unMaxTrackedDeviceCount)
    return;

  // try to find a model we've already set up
  ON_String sRenderModelName = GetTrackedDeviceString(m_pHMD, unTrackedDeviceIndex, vr::Prop_RenderModelName_String);

  RhinoVrDeviceModel* pRenderModel = FindOrLoadRenderModel(sRenderModelName);
  if (!pRenderModel)
  {
    ON_String sTrackingSystemName = GetTrackedDeviceString(m_pHMD, unTrackedDeviceIndex, vr::Prop_TrackingSystemName_String);
    RhinoApp().Print("Unable to load render model for tracked device %d (%s.%s)", unTrackedDeviceIndex, sTrackingSystemName.Array(), sRenderModelName.Array());
  }
  else
  {
    m_rTrackedDeviceToRenderModel[unTrackedDeviceIndex] = pRenderModel;

    if (!sRenderModelName.EqualOrdinal("lh_basestation_vive", false))
    {
      m_rbShowTrackedDevice[unTrackedDeviceIndex] = true;
    }
  }
}

ON_Mesh RhinoVrRenderer::LoadHiddenAreaMesh(vr::Hmd_Eye eye)
{
  vr::HiddenAreaMesh_t hidden_mesh = m_pHMD->GetHiddenAreaMesh(eye);

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
  if (!m_pHMD)
    return;

  for (uint32_t unTrackedDevice = vr::k_unTrackedDeviceIndex_Hmd + 1; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++)
  {
    if (!m_pHMD->IsTrackedDeviceConnected(unTrackedDevice))
      continue;

    SetupRenderModelForTrackedDevice(unTrackedDevice);
  }

  m_hidden_area_mesh_left = LoadHiddenAreaMesh(vr::Eye_Left);
  m_hidden_area_mesh_right = LoadHiddenAreaMesh(vr::Eye_Right);
}

void RhinoVrRenderer::UpdateDeviceState(const ON_Xform& device_to_world)
{
  bool bIsInputAvailable = m_pHMD->IsInputAvailable();

  for (uint32_t unTrackedDevice = 0; unTrackedDevice < vr::k_unMaxTrackedDeviceCount; unTrackedDevice++)
  {
    RhinoVrDeviceDisplayConduit& ddc = m_device_display_conduit[unTrackedDevice];
    ddc.Empty();

    RhinoVrDeviceModel* device_model = m_rTrackedDeviceToRenderModel[unTrackedDevice];

    if (!device_model || !m_rbShowTrackedDevice[unTrackedDevice])
      continue;

    const vr::TrackedDevicePose_t& pose = m_rTrackedDevicePose[unTrackedDevice];
    if (!pose.bPoseIsValid)
      continue;

    const bool is_controller = (m_pHMD->GetTrackedDeviceClass(unTrackedDevice) == vr::TrackedDeviceClass_Controller);

    if (!bIsInputAvailable && is_controller)
      continue;

    if (is_controller)
    {
      ddc.AddLine(m_pointer_line.from, m_pointer_line.to, ON_Color::SaturatedGreen);
    }

    const ON_Mesh& device_mesh = device_model->m_device_mesh;
    const ON_Xform device_xform = device_to_world * m_rmat4DevicePose[unTrackedDevice];
    CRhinoCacheHandle& device_cache_handle = device_model->m_cache_handle;
    
    ddc.SetDeviceMesh(&device_mesh);
    ddc.SetDeviceMeshXform(device_xform);
    ddc.SetDeviceMeshCacheHandle(&device_cache_handle);

    if (!ddc.IsEnabled())
    {
      ddc.SetFrustumNearFarSuggestion(m_fNearClip, m_fFarClip);
      ddc.Enable(m_vr_doc_sn);
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
  CRhinoView* rhino_view = CRhinoView::FromRuntimeSerialNumber(m_vr_view_sn);
  CRhinoViewport* vr_vp = CRhinoViewport::FromRuntimeSerialNumber(m_vr_viewport_sn);

  if (rhino_view == nullptr || vr_vp == nullptr)
  {
    return;
  }

  UpdateHMDMatrixPose();

  if (m_trackpad_point != ON_2dPoint::UnsetPoint)
  {
    CRhinoDoc* rhino_doc = CRhinoDoc::FromRuntimeSerialNumber(m_vr_doc_sn);
    if (rhino_doc)
    {
      if (!rhino_doc->InCommand())
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

          if (m_previous_cam_dir != ON_3dVector::UnsetVector)
          {
            m_camera_translation = m_camera_translation + 0.25*y_scaled*m_previous_cam_dir*m_unit_scale;
          }
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

  const ON_Viewport& rhino_vp = rhino_view->ActiveViewport().VP();

  ON_Viewport vp = m_vp_orig_vr_frus;
  vp.SetProjection(rhino_vp.Projection());
  vp.SetCameraLocation(rhino_vp.CameraLocation());
  vp.SetCameraDirection(rhino_vp.CameraDirection());
  vp.SetCameraUp(rhino_vp.CameraUp());
  vp.SetTargetPoint(rhino_vp.TargetPoint());
  vp.SetFrustumNearFar(m_fNearClip, m_fFarClip);

  vp.DollyCamera(m_camera_translation);
  vp.DollyFrustum(m_camera_translation.z);
  vp.Rotate(m_camera_rotation, ON_3dVector::ZAxis, vp.CameraLocation());

  vp.GetXform(ON::coordinate_system::camera_cs, ON::coordinate_system::world_cs, m_cam_to_world);
  vp.GetXform(ON::coordinate_system::world_cs, ON::coordinate_system::camera_cs, m_world_to_cam);

  m_vp_hmd = vp;
  ON_Xform cam_to_hmd_xform = m_cam_to_world * m_mat4HMDPose * m_world_to_cam;

  m_vp_hmd.Transform(cam_to_hmd_xform);

  m_previous_cam_dir = m_vp_hmd.CameraDirection().UnitVector();

  // By default, the view is set to the HMD viewport.
  vr_vp->SetVP(m_vp_hmd, TRUE);

  ON_Xform cam_to_left_eye_xform = m_cam_to_world * m_mat4HMDPose * m_mat4eyePosLeft * m_world_to_cam;

  m_vp_left_eye = vp;

  m_vp_left_eye.Transform(cam_to_left_eye_xform);
  m_vp_left_eye.SetFrustum(
    m_left_frus_left, m_left_frus_right,
    m_left_frus_top, m_left_frus_bottom,
    m_fNearClip, m_fFarClip);

  m_vp_left_eye.GetXform(ON::coordinate_system::clip_cs, ON::coordinate_system::world_cs, m_clip_to_left_eye);
  //m_clip_to_left_eye = m_clip_to_left_eye * ON_Xform::ScaleTransformation(ON_3dPoint::Origin, 1.0 - m_trigger_value);

  ON_Xform cam_to_right_eye_xform = m_cam_to_world * m_mat4HMDPose * m_mat4eyePosRight * m_world_to_cam;

  m_vp_right_eye = vp;

  m_vp_right_eye.Transform(cam_to_right_eye_xform);
  m_vp_right_eye.SetFrustum(
    m_right_frus_left, m_right_frus_right,
    m_right_frus_top, m_right_frus_bottom,
    m_fNearClip, m_fFarClip);

  m_vp_right_eye.GetXform(ON::coordinate_system::clip_cs, ON::coordinate_system::world_cs, m_clip_to_right_eye);
  //m_clip_to_right_eye = m_clip_to_right_eye * ON_Xform::ScaleTransformation(ON_3dPoint::Origin, 1.0 - m_trigger_value);

  UpdateDeviceState(m_cam_to_world);
}

bool RhinoVrRenderer::DrawStereoFrame()
{
  bool rc = false;

  CRhinoView* rhino_view = CRhinoView::FromRuntimeSerialNumber(m_vr_view_sn);
  CRhinoViewport* vr_vp = CRhinoViewport::FromRuntimeSerialNumber(m_vr_viewport_sn);

  if (rhino_view == nullptr || vr_vp == nullptr)
  {
    return false;
  }

  CRhinoDisplayPipeline* vr_dp = vr_vp->DisplayPipeline();
  CDisplayPipelineAttributes* vr_dpa = rhino_view->DisplayAttributes();

  if (vr_dp == nullptr || vr_dpa == nullptr)
  {
    return false;
  }

  vr_dp->EnableDynamicDisplayDowngrade(false);

  bool frame_buffer_capture_enabled = CRhinoDisplayPipeline::FrameBufferCaptureEnabled();
  if (frame_buffer_capture_enabled)
  {
    CRhinoDisplayPipeline::EnableFrameBufferCapture(false);
  }

  m_hidden_mesh_display_conduit.SetActiveEye(vr::Eye_Left);

  vr_vp->SetVP(m_vp_left_eye, TRUE);
  vr_dp->ClosePipeline();
  vr_dp->DrawFrameBuffer(*vr_dpa, m_vp_left_eye, true, true);
  vr_dp->OpenPipeline();

  unsigned long long eye_left_handle = RhinoVrGetEyeTextureHandle(vr_dp, 0);

  m_hidden_mesh_display_conduit.SetActiveEye(vr::Eye_Right);

  vr_vp->SetVP(m_vp_right_eye, TRUE);
  vr_dp->ClosePipeline();
  vr_dp->DrawFrameBuffer(*vr_dpa, m_vp_right_eye, true, true);
  vr_dp->OpenPipeline();

  unsigned long long eye_right_handle = RhinoVrGetEyeTextureHandle(vr_dp, 1);

  vr::EVRCompositorError err = vr::VRCompositorError_None;

  vr::Texture_t leftEyeTexture = { (void*)(uintptr_t)eye_left_handle, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
  err = vr::VRCompositor()->Submit(vr::Eye_Left, &leftEyeTexture);
  if (err != vr::VRCompositorError_None && err != vr::VRCompositorError_DoNotHaveFocus)
  {
    ASSERT(false);
  }

  vr::Texture_t rightEyeTexture = { (void*)(uintptr_t)eye_right_handle, vr::TextureType_OpenGL, vr::ColorSpace_Gamma };
  err = vr::VRCompositor()->Submit(vr::Eye_Right, &rightEyeTexture);
  if (err != vr::VRCompositorError_None && err != vr::VRCompositorError_DoNotHaveFocus)
  {
    ASSERT(false);
  }

  //::glFlush();

  vr_dp->ClosePipeline();

  CRhinoDisplayPipeline::EnableFrameBufferCapture(frame_buffer_capture_enabled);

  return rc;
}

void RhinoVrRenderer::MeasureFramesPerSecond()
{
  //++m_last_frame_count;
  //++m_frames_rendered;

  //auto current_timestamp = RhTiming::GetHighPrecisionTimestamp();
  //auto time_between = RhTiming::GetTimeInSecondsBetween(m_last_timestamp, current_timestamp);

  //if (time_between >= 1.0)
  //{
  //  double fps = m_last_frame_count / time_between;

  //  m_last_frame_count = 0;
  //  m_last_timestamp = current_timestamp;

  //  ON_wString fps_string;
  //  fps_string.Format(L"FPS: %f\n", fps);

  //  OutputDebugString(fps_string.Array());
  //  //RhinoApp().Print(fps_string);
  //}
}

void RhinoVrRenderer::ProcessVREvent(const vr::VREvent_t & event)
{
  switch (event.eventType)
  {
  case vr::VREvent_TrackedDeviceActivated:
  {
    SetupRenderModelForTrackedDevice(event.trackedDeviceIndex);
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

  CRhinoDoc* rhino_doc = CRhinoDoc::FromRuntimeSerialNumber(m_vr_doc_sn);
  CRhinoView* rhino_view = CRhinoView::FromRuntimeSerialNumber(m_vr_view_sn);
  CRhinoViewport* vr_vp = CRhinoViewport::FromRuntimeSerialNumber(m_vr_viewport_sn);

  if (rhino_doc == nullptr || rhino_view == nullptr || vr_vp == nullptr)
  {
    return false;
  }

  ON_Viewport ray_vp = m_vp_hmd;

  ON_Xform world_to_screen;
  m_vp_hmd.GetXform(ON::coordinate_system::world_cs, ON::coordinate_system::screen_cs, world_to_screen);

  ray_vp.Transform(m_cam_to_world*device_pose*m_mat4HMDPose.Inverse()*m_world_to_cam);

  CRhinoPickContext pc;
  pc.m_view = rhino_view;
  pc.m_pick_mode = CRhinoPickContext::shaded_pick;
  pc.m_pick_style = CRhinoPickContext::point_pick;

  ON_2iPoint center_pixel = ON_2iPoint(ray_vp.ScreenPortWidth() / 2, ray_vp.ScreenPortHeight() / 2);

  vr_vp->SetVP(ray_vp, TRUE);
  vr_vp->SetClippingRegionTransformation(center_pixel.x, center_pixel.y, pc.m_pick_region);
  vr_vp->SetVP(m_vp_hmd, TRUE);

  ray_vp.GetFrustumLine(center_pixel.x, center_pixel.y, pc.m_pick_line);

  pc.UpdateClippingPlanes();

  //vr_doc->AddCurveObject(pc.m_pick_line);

  CRhinoObjRefArray rhino_objs;
  rhino_doc->PickObjects(pc, rhino_objs);

  rhino_objs.QuickSort(sort_objref_by_seldist);

  if (rhino_objs.Count() > 0)
  {
    CRhinoObjRef objref = rhino_objs[0];
//    objref.Object()->Select();

    ON_3dPoint isect_pt;
    if (objref.SelectionPoint(isect_pt))
    {
      ON_3dPoint pt_screen = world_to_screen * isect_pt;
      window_coords = ON_2iPoint((int)pt_screen.x, (int)pt_screen.y);
    }
  }

  return true;
}

bool RhinoVrRenderer::GetWorldPickLineAndClipRegion(const ON_Xform& device_xform, ON_Line& world_line, ON_ClippingRegion& clip_region, ON_Viewport& line_vp, ON_2iPoint& line_pixel)
{
  CRhinoViewport* vr_vp = CRhinoViewport::FromRuntimeSerialNumber(m_vr_viewport_sn);

  if (vr_vp == nullptr)
  {
    return false;
  }

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
  double far_dist = line_vp.FrustumFar();
  line_vp.SetFrustumNearFar(near_dist, near_dist + m_pointer_line.Length());

  line_pixel = ON_2iPoint(line_vp.ScreenPortWidth() / 2 + 1, line_vp.ScreenPortHeight() / 2 + 1);

  vr_vp->SetVP(line_vp, TRUE);
  vr_vp->SetClippingRegionTransformation(line_pixel.x, line_pixel.y, clip_region);
  vr_vp->SetVP(m_vp_hmd, TRUE);

  line_vp.GetFrustumLine(line_pixel.x, line_pixel.y, world_line);

  return true;
}

void RhinoVrRenderer::HandleInput()
{
  // Process SteamVR events
  vr::VREvent_t event;
  while (m_pHMD->PollNextEvent(&event, sizeof(event)))
  {
    ProcessVREvent(event);
  }

  // Process SteamVR controller state
  for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
  {
    if (m_pHMD->GetTrackedDeviceClass(unDevice) != vr::TrackedDeviceClass_Controller)
      continue;

    vr::VRControllerState_t state;
    if (m_pHMD->GetControllerState(unDevice, &state, sizeof(state)))
    {
      RhinoVrDeviceController& controller = m_device_controllers[unDevice];

      //m_trigger_value = 0.0;

      //if (state.rAxis[1].x > 0.0)
      //{
      //  m_trigger_value = state.rAxis[1].x;
      //  break;
      //}

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

          CRhinoDoc* rhino_doc = CRhinoDoc::FromRuntimeSerialNumber(m_vr_doc_sn);
          CRhinoView* rhino_view = CRhinoView::FromRuntimeSerialNumber(m_vr_view_sn);

          if (rhino_doc && rhino_view)
          {
            if (rhino_doc->InGetPoint())
            {
              CRhinoGetPoint* gp = rhino_doc->InGetPoint();

              ON_Line world_line;
              ON_ClippingRegion clip_region;
              ON_Viewport line_vp;
              ON_2iPoint line_pixel;

              if (GetWorldPickLineAndClipRegion(m_rmat4DevicePose[unDevice], world_line, clip_region, line_vp, line_pixel))
              {
                LPARAM nFlags = 0;
                ON_3dPoint screen_pt = ON_3dPoint(line_pixel.x, line_pixel.y, 0.0);

                CRhinoViewport& rhino_vp = rhino_view->ActiveViewport();

                const ON_Viewport orig_vp = rhino_vp.VP();
                const int orig_width = orig_vp.ScreenPortWidth();
                const int orig_height = orig_vp.ScreenPortHeight();

                rhino_vp.SetVP(line_vp, TRUE);
                rhino_vp.SetScreenSize(line_vp.ScreenPortWidth(), line_vp.ScreenPortHeight());

                ON_3dPoint world_point;
                if (gp->GetView3dPoint(1, *rhino_view, nFlags, screen_pt, world_line, world_point))
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
              pc.m_view = rhino_view;
              pc.m_pick_mode = CRhinoPickContext::shaded_pick;
              pc.m_pick_style = CRhinoPickContext::point_pick;

              if (GetWorldPickLineAndClipRegion(m_rmat4DevicePose[unDevice], pc.m_pick_line, pc.m_pick_region, line_vp, line_pixel))
              {
                pc.UpdateClippingPlanes();

                //rhino_doc->AddCurveObject(pc.m_pick_line);

                CRhinoObjRefArray rhino_objs;
                rhino_doc->PickObjects(pc, rhino_objs);

                rhino_objs.QuickSort(sort_objref_by_seldist);

                if (rhino_objs.Count() > 0)
                {
                  CRhinoObjRef objref = rhino_objs[0];

                  if (rhino_doc->InGetObject())
                  {
                    CRhinoGetObject* go = rhino_doc->InGetObject();
                    CRhinoObjRefArray& obj_ref_array = const_cast<CRhinoObjRefArray&>(go->PickList());
                    obj_ref_array.Append(objref);

                    RhinoVrPostGetObjectEvent(go, rhino_view);

                  }
                  else if (rhino_doc->InGetPoint())
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
          RhinoApp().ExecuteCommand(m_vr_doc_sn, L"Move");
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

          RhinoApp().ExecuteCommand(m_vr_doc_sn, L"_Enter");
        }
        else
        {
          CRhinoDoc* rhino_doc = CRhinoDoc::FromRuntimeSerialNumber(m_vr_doc_sn);
          CRhinoView* rhino_view = CRhinoView::FromRuntimeSerialNumber(m_vr_view_sn);

          if (rhino_doc && rhino_view)
          {
            if (rhino_doc->InGetPoint())
            {
              CRhinoGetPoint* gp = rhino_doc->InGetPoint();

              ON_Line world_line;
              ON_ClippingRegion clip_region;
              ON_Viewport line_vp;
              ON_2iPoint line_pixel;

              if (GetWorldPickLineAndClipRegion(m_rmat4DevicePose[unDevice], world_line, clip_region, line_vp, line_pixel))
              {
                LPARAM nFlags = 0;
                ON_3dPoint screen_pt = ON_3dPoint(line_pixel.x, line_pixel.y, 0.0);

                CRhinoViewport& rhino_vp = rhino_view->ActiveViewport();

                const ON_Viewport orig_vp = rhino_vp.VP();
                const int orig_width = orig_vp.ScreenPortWidth();
                const int orig_height = orig_vp.ScreenPortHeight();

                rhino_vp.SetVP(line_vp, TRUE);
                rhino_vp.SetScreenSize(line_vp.ScreenPortWidth(), line_vp.ScreenPortHeight());

                ON_3dPoint world_point;
                if (gp->GetView3dPoint(1, *rhino_view, (UINT_PTR)nFlags, screen_pt, world_line, world_point))
                {
                  //rhino_doc->AddPointObject(world_point);

                  CRhinoViewport* vr_vp = CRhinoViewport::FromRuntimeSerialNumber(m_vr_viewport_sn);
                  if (vr_vp)
                  {
                    gp->OnMouseMove(*vr_vp, (UINT)nFlags, world_point, (const ON_2iPoint*) nullptr);
                  }
                }
                
                rhino_vp.SetVP(orig_vp, TRUE);
                rhino_vp.SetScreenSize(orig_width, orig_height);
              }
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

void RhinoVrRenderer::SetTargetFrameRate(int target_frame_rate)
{
  m_target_frame_rate = target_frame_rate;
}

void RhinoVrRenderer::HandleInputAndRenderFrame()
{
  UpdateState();
  HandleInput();
  DrawStereoFrame();
  MeasureFramesPerSecond();
}

//-----------------------------------------------------------------------------
// Purpose: Converts a SteamVR matrix to our local matrix class
//-----------------------------------------------------------------------------
ON_Xform RhinoVrRenderer::ConvertOpenVRMatrixToXform(const vr::HmdMatrix34_t &mat)
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

//-----------------------------------------------------------------------------
// Purpose: Gets a Matrix Projection Eye with respect to nEye.
//-----------------------------------------------------------------------------
ON_Xform RhinoVrRenderer::GetHMDMatrixProjectionEye(vr::Hmd_Eye nEye)
{
  if (!m_pHMD)
    return ON_Xform();

  vr::HmdMatrix44_t mat = m_pHMD->GetProjectionMatrix(nEye, m_fNearClip, m_fFarClip);

  ON_Xform xform;

  xform.m_xform[0][0] = mat.m[0][0];
  xform.m_xform[0][1] = mat.m[1][0];
  xform.m_xform[0][2] = mat.m[2][0];
  xform.m_xform[0][3] = mat.m[3][0];

  xform.m_xform[1][0] = mat.m[0][1];
  xform.m_xform[1][1] = mat.m[1][1];
  xform.m_xform[1][2] = mat.m[2][1];
  xform.m_xform[1][3] = mat.m[3][1];

  xform.m_xform[2][0] = mat.m[0][2];
  xform.m_xform[2][1] = mat.m[1][2];
  xform.m_xform[2][2] = mat.m[2][2];
  xform.m_xform[2][3] = mat.m[3][2];

  xform.m_xform[3][0] = mat.m[0][3];
  xform.m_xform[3][1] = mat.m[1][3];
  xform.m_xform[3][2] = mat.m[2][3];
  xform.m_xform[3][3] = mat.m[3][3];

  xform.Invert();

  return xform;
}

//-----------------------------------------------------------------------------
// Purpose: Gets an HMDMatrixPoseEye with respect to nEye.
//-----------------------------------------------------------------------------
ON_Xform RhinoVrRenderer::GetHMDMatrixPoseEye(vr::Hmd_Eye nEye)
{
  if (!m_pHMD)
    return ON_Xform();

  vr::HmdMatrix34_t mat = m_pHMD->GetEyeToHeadTransform(nEye);

  return ConvertOpenVRMatrixToXform(mat);
}

void RhinoVrRenderer::UpdateHMDMatrixPose()
{
  if (!m_pHMD)
    return;

  vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, nullptr, 0);

  if (!m_hmd_pose_correction_acquired && m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
  {
    m_hmd_pose_correction_acquired = true;

    ON_Xform xform = ConvertOpenVRMatrixToXform(m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking);
    m_mat4HMDPoseCorrection = ON_Xform::TranslationTransformation(ON_3dVector(-xform[0][3], -xform[1][3], -xform[2][3]));
  }

  m_iValidPoseCount = 0;
  m_strPoseClasses = "";
  for (int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice)
  {
    if (m_rTrackedDevicePose[nDevice].bPoseIsValid)
    {
      m_iValidPoseCount++;
      m_rmat4DevicePose[nDevice] = m_mat4HMDPoseCorrection * ConvertOpenVRMatrixToXform(m_rTrackedDevicePose[nDevice].mDeviceToAbsoluteTracking);
      if (m_rDevClassChar[nDevice] == 0)
      {
        switch (m_pHMD->GetTrackedDeviceClass(nDevice))
        {
        case vr::TrackedDeviceClass_Controller:        m_rDevClassChar[nDevice] = 'C'; break;
        case vr::TrackedDeviceClass_HMD:               m_rDevClassChar[nDevice] = 'H'; break;
        case vr::TrackedDeviceClass_Invalid:           m_rDevClassChar[nDevice] = 'I'; break;
        case vr::TrackedDeviceClass_GenericTracker:    m_rDevClassChar[nDevice] = 'G'; break;
        case vr::TrackedDeviceClass_TrackingReference: m_rDevClassChar[nDevice] = 'T'; break;
        default:                                       m_rDevClassChar[nDevice] = '?'; break;
        }
      }
      m_strPoseClasses += m_rDevClassChar[nDevice];
    }
  }

  if (m_rTrackedDevicePose[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
  {

    m_mat4HMDPose = m_rmat4DevicePose[vr::k_unTrackedDeviceIndex_Hmd];
    m_mat4eyePosLeft = GetHMDMatrixPoseEye(vr::Eye_Left);
    m_mat4eyePosRight = GetHMDMatrixPoseEye(vr::Eye_Right);
  }
}

bool RhinoVrRenderer::InitializeVrSystem()
{
  // Loading the SteamVR Runtime
  vr::EVRInitError eError = vr::VRInitError_None;
  m_pHMD = vr::VR_Init(&eError, vr::VRApplication_Scene);

  if (eError != vr::VRInitError_None)
  {
    m_pHMD = nullptr;
    char buf[1024];
    sprintf_s(buf, sizeof(buf), "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
    RhinoApp().Print(buf);
    //SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL);
    return false;
  }

  m_pRenderModels = (vr::IVRRenderModels *)vr::VR_GetGenericInterface(vr::IVRRenderModels_Version, &eError);
  if (!m_pRenderModels)
  {
    m_pHMD = nullptr;
    vr::VR_Shutdown();

    char buf[1024];
    sprintf_s(buf, sizeof(buf), "Unable to get render model interface: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
    RhinoApp().Print(buf);
    //SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "VR_Init Failed", buf, NULL);
    return false;
  }

  CRhinoDoc* rhino_doc = CRhinoDoc::FromRuntimeSerialNumber(m_vr_doc_sn);
  if (rhino_doc)
  {
    m_unit_scale = rhino_doc->ModelUnits().MetersPerUnit();
  }

  m_pointer_line = ON_Line(m_unit_scale*ON_3dPoint(0, 0, -0.02f), m_unit_scale*ON_3dPoint(0, 0, -500.0f));
  
  uint32_t rec_width, rec_height;
  m_pHMD->GetRecommendedRenderTargetSize(&rec_width, &rec_height);

  CRhinoViewport* vr_vp = CRhinoViewport::FromRuntimeSerialNumber(m_vr_viewport_sn);
  CRhinoView* rhino_view = CRhinoView::FromRuntimeSerialNumber(m_vr_view_sn);

  if (vr_vp == nullptr || rhino_view == nullptr)
  {
    m_pHMD = nullptr;
    vr::VR_Shutdown();
  }

  m_vp_orig = vr_vp->VP();

  ON_Viewport vp = m_vp_orig;

  double l, r, b, t, n, f;
  vp.GetFrustum(&l, &r, &b, &t, &n, &f);

  m_fNearClip = (float)(m_unit_scale * 0.01);
  m_fFarClip = (float)(m_fNearClip / vp.PerspectiveMinNearOverFar());

  m_pHMD->GetProjectionRaw(vr::Eye_Left,
    &m_left_frus_left, &m_left_frus_right, &m_left_frus_top, &m_left_frus_bottom);

  m_left_frus_left *= m_fNearClip;
  m_left_frus_right *= m_fNearClip;
  m_left_frus_top *= m_fNearClip;
  m_left_frus_bottom *= m_fNearClip;

  m_pHMD->GetProjectionRaw(vr::Eye_Right,
    &m_right_frus_left, &m_right_frus_right, &m_right_frus_top, &m_right_frus_bottom);

  m_right_frus_left *= m_fNearClip;
  m_right_frus_right *= m_fNearClip;
  m_right_frus_top *= m_fNearClip;
  m_right_frus_bottom *= m_fNearClip;

  vp.SetFrustumLeftRightSymmetry(false);
  vp.SetFrustumTopBottomSymmetry(false);
  vp.SetFrustum(
    m_left_frus_left, m_right_frus_right,
    m_left_frus_top, m_left_frus_bottom,
    m_fNearClip, m_fFarClip);

  m_vp_orig_vr_frus = vp;

  int vp_width = vp.ScreenPortWidth();
  int vp_height = vp.ScreenPortHeight();

  double frus_aspect;
  vp.GetFrustumAspect(frus_aspect);

  int vp_new_width = vp_width/2;
  int vp_new_height = (int)(vp_new_width / frus_aspect + 0.5);

  ON_wString script;
  script.Format(L"-_ViewportProperties _Size %d %d _Enter", vp_new_width, vp_new_height);
  
  RhinoApp().RunScriptEx(m_vr_doc_sn, script, nullptr, 0);

  vp.SetScreenPort(0, vp_width, 0, vp_height);
  rhino_view->ActiveViewport().SetVP(vp, TRUE);
  rhino_view->Redraw();

  //m_mat4eyePosLeft = GetHMDMatrixPoseEye(vr::Eye_Left);
  //m_mat4eyePosRight = GetHMDMatrixPoseEye(vr::Eye_Right);

  SetupRenderModels();

  if (!vr::VRCompositor())
  {
    printf("Compositor initialization failed. See log file for details\n");
    return false;
  }

  return true;
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

RhinoVrDeviceModel::RhinoVrDeviceModel(const ON_String & sRenderModelName)
  : m_device_name(sRenderModelName)
{
}

bool RhinoVrDeviceModel::Init(const vr::RenderModel_t & vrModel, const vr::RenderModel_TextureMap_t & vrDiffuseTexture, double unit_scale)
{
  m_device_mesh.Destroy();
  m_device_mesh.m_V.SetCapacity(vrModel.unVertexCount);
  m_device_mesh.m_N.SetCapacity(vrModel.unVertexCount);
  m_device_mesh.m_T.SetCapacity(vrModel.unVertexCount);

  m_device_mesh.m_V.SetCount(vrModel.unVertexCount);
  m_device_mesh.m_N.SetCount(vrModel.unVertexCount);
  m_device_mesh.m_T.SetCount(vrModel.unVertexCount);

  for (unsigned int vi = 0; vi < vrModel.unVertexCount; ++vi)
  {
    const vr::RenderModel_Vertex_t& rm_vertex = vrModel.rVertexData[vi];

    m_device_mesh.m_V[vi] = unit_scale * ON_3fPoint((const float*)(&rm_vertex.vPosition));
    m_device_mesh.m_N[vi] = ON_3fPoint((const float*)(&rm_vertex.vNormal));
    m_device_mesh.m_T[vi] = ON_2fPoint((const float*)(&rm_vertex.rfTextureCoord));
  }

  m_device_mesh.m_F.SetCapacity(vrModel.unTriangleCount);
  m_device_mesh.m_F.SetCount(vrModel.unTriangleCount);

  const int index_count = vrModel.unTriangleCount * 3;

  for (unsigned int fi = 0, idx = 0; fi < vrModel.unTriangleCount; ++fi)
  {
    ON_MeshFace& triangle = m_device_mesh.m_F[fi];

    triangle.vi[0] = vrModel.rIndexData[idx++];
    triangle.vi[1] = vrModel.rIndexData[idx++];
    triangle.vi[2] = vrModel.rIndexData[idx++];
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
