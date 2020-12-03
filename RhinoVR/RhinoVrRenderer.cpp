#include "stdafx.h"
#include "RhinoVrRenderer.h"
#include <gl/GL.h>

//#define RHINOVR_FRAME_TIMING
//#define RHINOVR_DETAILED_TIMING

#pragma comment(lib, "../OpenVR/lib/win64/openvr_api.lib")

RhinoVrRenderer::RhinoVrRenderer(unsigned int doc_sn, unsigned int view_sn)
  : m_doc_sn(doc_sn)
  , m_view_sn(view_sn)
  , m_doc(nullptr)
  , m_view(nullptr)
  , m_vr_vp(nullptr)
  , m_vr_dp(nullptr)
  , m_vr_dp_ogl(nullptr)
  , m_previous_display_mode(ON_nil_uuid)
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
  , m_fps_time_start(0)
  , m_frame_counter(0)
  , m_last_window_update(0)
  , m_move_speed(0.0)
  , m_turn_speed(90.0)
  , m_last_frame_time(0)
  , m_frame_timestamp(0)
  , m_move_speed_when_start_moving(0.0)
  , m_start_moving_timestamp(0)
  , m_move_speed_when_stop_moving(0.0)
  , m_stop_moving_timestamp(0)
  , m_gh_window_left_btn_down(false)
  , m_window_intersected_this_frame(false)
  , m_last_window_click_pos(ON_2iPoint::Unset)
{
  memset(m_device_poses, 0, sizeof(m_device_poses));
  
  m_device_data.SetCapacity(vr::k_unMaxTrackedDeviceCount);
  m_device_data.SetCount(vr::k_unMaxTrackedDeviceCount);
}

RhinoVrRenderer::~RhinoVrRenderer()
{
  RhinoDisableContinuousMainLoop();

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

HWND FindApplicationWindow(const wchar_t* app_title_string)
{
  const int window_text_length = 256;
  wchar_t window_text[window_text_length];

  auto hwnd_parent = FindWindowEx(nullptr, nullptr, nullptr, nullptr);

  while (hwnd_parent)
  {
    if (IsWindowVisible(hwnd_parent) && GetWindowText(hwnd_parent, window_text, window_text_length))
    {
      ON_wString str(window_text);
      if (str.Find(app_title_string) >= 0)
      {
        return hwnd_parent;
      }
    }

    hwnd_parent = FindWindowEx(nullptr, hwnd_parent, nullptr, nullptr);
  }

  return nullptr;
}

ON_Mesh CreateAppWindowMesh(double extent_x, double extent_y, double extent_z)
{
  ON_Mesh mesh;

  mesh.m_V.Append(ON_3fPoint(-0.5f, -0.5f, -0.05f));
  mesh.m_V.Append(ON_3fPoint(0.5f, -0.5f, -0.05f));
  mesh.m_V.Append(ON_3fPoint(0.5f, 0.5f, -0.05f));
  mesh.m_V.Append(ON_3fPoint(-0.5f, 0.5f, -0.05f));

  mesh.m_T.Append(ON_2fPoint(0.0f, 1.0f));
  mesh.m_T.Append(ON_2fPoint(1.0f, 1.0f));
  mesh.m_T.Append(ON_2fPoint(1.0f, 0.0f));
  mesh.m_T.Append(ON_2fPoint(0.0f, 0.0f));

  ON_MeshFace& face = mesh.m_F.AppendNew();
  face.vi[0] = 0;
  face.vi[1] = 1;
  face.vi[2] = 2;
  face.vi[3] = 3;

  ON_Xform scale = ON_Xform::DiagonalTransformation(extent_x, extent_y, extent_z);

  ON_Xform rotation;
  rotation.Rotation(90.0*ON_DEGREES_TO_RADIANS, ON_3dVector::XAxis, ON_3dPoint::Origin);

  mesh.Transform(rotation * scale);

  return mesh;
}

void InitializeAppWindow(RhinoVrAppWindow& app, const ON_wString& app_title)
{
  app.m_enabled = false;
  app.m_title = app_title;
  app.m_crc   = app_title.DataCRC(0);
  app.m_hwnd  = FindApplicationWindow(app_title);
  app.m_mesh = CreateAppWindowMesh(1.0, 1.0, 1.0);
}

ON_Mesh CreatePointerLineMesh(const ON_Line& pl, double unit_scale)
{
  const float r = (float)unit_scale*0.0015f;

  ON_Mesh mesh;
  mesh.m_V.Append(ON_3fPoint(-r, 0.0f, (float)pl.from.z));
  mesh.m_V.Append(ON_3fPoint(+r, 0.0f, (float)pl.from.z));
  mesh.m_V.Append(ON_3fPoint(+r, 0.0f, (float)pl.to.z));
  mesh.m_V.Append(ON_3fPoint(-r, 0.0f, (float)pl.to.z));

  ON_MeshFace& face = mesh.m_F.AppendNew();
  face.vi[0] = 0;
  face.vi[1] = 1;
  face.vi[2] = 2;
  face.vi[3] = 3;

  return mesh;
}

bool CreateVrViewport(CRhinoView& view, std::unique_ptr<CRhinoViewport>* vp)
{
  if (vp == nullptr)
    return false;

  *vp = std::make_unique<CRhinoViewport>();
  (*vp)->CopyFrom(view.Viewport(), true);

  return true;
}

bool CreateVrDisplayPipeline(
  CRhinoView& view, CRhinoViewport& vp,
  std::unique_ptr<CRhinoDisplayPipeline>* dp,
  CRhinoDisplayPipeline_OGL** dp_ogl)
{
  if (dp == nullptr || dp_ogl == nullptr)
    return false;

  CRhinoDisplayPipeline* view_dp = view.DisplayPipeline();
  if (view_dp == nullptr)
    return false;

  CRhinoDisplayPipeline_OGL* view_dp_ogl = dynamic_cast<CRhinoDisplayPipeline_OGL*>(view_dp);
  if (view_dp_ogl == nullptr)
    return false;

  view_dp_ogl->OpenPipeline();
  *dp = std::unique_ptr<CRhinoDisplayPipeline>(view_dp_ogl->ClonePipeline(vp));
  view_dp_ogl->ClosePipeline();

  if (dp->get() == nullptr)
    return false;

  *dp_ogl = dynamic_cast<CRhinoDisplayPipeline_OGL*>(dp->get());
  if (*dp_ogl == nullptr)
    return false;

  return true;
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
    str.Format("Unable to initialize VR compositor: %s\n", vr::VR_GetVRInitErrorAsEnglishDescription(ovr_error));
    RhinoApp().Print(str);
  }

  CRhinoDoc* rhino_doc = m_doc = CRhinoDoc::FromRuntimeSerialNumber(m_doc_sn);
  if (rhino_doc == nullptr)
    return false;
  
  m_unit_scale = rhino_doc->ModelUnits().MetersPerUnit(ON_DBL_QNAN);
  m_unit_scale = 1.0 / m_unit_scale;
  m_pointer_line = ON_Line(m_unit_scale*ON_3dPoint(0, 0, -0.02), m_unit_scale*ON_3dPoint(0, 0, -250.0));

  m_pointer_mesh = CreatePointerLineMesh(m_pointer_line, m_unit_scale);
  m_pointer_mesh_material.m_FrontMaterial.SetDiffuse(ON_Color::SaturatedGreen);
  m_pointer_mesh_material.m_FrontMaterial.SetDisableLighting(true);

  SetupRenderModels();

  uint32_t rec_width, rec_height;
  m_hmd->GetRecommendedRenderTargetSize(&rec_width, &rec_height);

  // For now, let's force the resolution to be the same as the native
  // screen resolution of both the Vive and the Oculus.
  //rec_width = 1080;
  //rec_height = 1200;

  CRhinoView* view = CRhinoView::FromRuntimeSerialNumber(m_view_sn);
  if (view == nullptr)
    return false;

  if (!CreateVrViewport(*view, &m_vr_vp))
  {
    return false;
  }

  m_vr_vp->SetScreenSize(rec_width, rec_height);

  if (!CreateVrDisplayPipeline(*view, *m_vr_vp, &m_vr_dp, &m_vr_dp_ogl))
  {
    return false;
  }

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

  m_frustum_conduit.SetFrustumLeft(
    m_near_clip, m_far_clip,
    m_left_frus_left, m_left_frus_right,
    m_left_frus_top, m_left_frus_bottom);

  m_frustum_conduit.SetFrustumRight(
    m_near_clip, m_far_clip,
    m_right_frus_left, m_right_frus_right,
    m_right_frus_top, m_right_frus_bottom);

  m_frustum_conduit.Enable(m_doc_sn);

  InitializeAppWindow(m_gh_window, L"Grasshopper");
  //InitializeAppWindow(m_rh_window, L"Rhinoceros 6");

  m_last_window_update = RhinoGetTimestamp();

  if (!RhinoEnableContinuousMainLoop())
  {
    RhinoApp().Print("RhinoVR warning: Failed to enable continuous Rhino main loop. RhinoVR performance may suffer.\n");
  }

  RhinoApp().Print(L"RhinoVR is ON. Please put on the VR headset.\n");

  return true;
}

ON_String GetTrackedDeviceString(vr::IVRSystem& hmd, vr::TrackedDeviceIndex_t device_idx, vr::TrackedDeviceProperty device_property, vr::TrackedPropertyError* error = nullptr)
{
  uint32_t required_buffer_len = hmd.GetStringTrackedDeviceProperty(device_idx, device_property, NULL, 0, error);
  if (required_buffer_len == 0)
    return ON_String("");

  char* buffer = new char[required_buffer_len];
  
  hmd.GetStringTrackedDeviceProperty(device_idx, device_property, buffer, required_buffer_len, error);
  ON_String device_string = buffer;

  delete[] buffer;

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

  vr::ETrackedDeviceClass device_class = m_hmd->GetTrackedDeviceClass(device_index);
  if (device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_HMD)
  {
      ON_String system_name = GetTrackedDeviceString(*m_hmd, device_index, vr::Prop_TrackingSystemName_String);
      if (system_name.EqualOrdinal("oculus", false))
      {
          m_vr_system_type = VrSystemType::Rift;
      }
      else
      {
          m_vr_system_type = VrSystemType::Vive;
      }
  }

  ON_String render_model_name = GetTrackedDeviceString(*m_hmd, device_index, vr::Prop_RenderModelName_String);

  // We don't want to show the headset, Vive base stations or the Rift cameras.
  if (render_model_name.EqualOrdinal("generic_hmd", false) ||
      render_model_name.EqualOrdinal("lh_basestation_vive", false) ||
      render_model_name.EqualOrdinal("rift_camera", false))
  {
      return;
  }

  RhinoVrDeviceModel* render_model = FindOrLoadRenderModel(render_model_name);
  if (render_model == nullptr)
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

  for (uint32_t device_idx = vr::k_unTrackedDeviceIndex_Hmd; device_idx < vr::k_unMaxTrackedDeviceCount; device_idx++)
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

  RhTimestamp now = RhinoGetTimestamp();
  double time_since_update = RhinoGetTimeInSecondsBetween(m_last_window_update, now);

  static int last_window_updated = 1;

  bool gh_window_needs_update = false;
  bool rh_window_needs_update = false;

  if (time_since_update >= 1.0 / 20.0)
  {
    m_last_window_update = now;

    if (m_rh_window.m_hwnd && m_rh_window.m_enabled && last_window_updated == 0)
    {
      rh_window_needs_update = true;
      last_window_updated = 1;
    }
    else if (m_gh_window.m_hwnd && m_gh_window.m_enabled && last_window_updated == 1)
    {
      gh_window_needs_update = true;
      last_window_updated = 0;
    }
    else if (m_rh_window.m_hwnd && m_rh_window.m_enabled)
    {
      rh_window_needs_update = true;
      last_window_updated = 1;
    }
    else if (m_gh_window.m_hwnd && m_gh_window.m_enabled)
    {
      gh_window_needs_update = true;
      last_window_updated = 0;
    }
  }

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
      bool draw_pointer = true;

      RhinoVrAppWindow* app_ptr = nullptr;
      bool current_window_needs_update = false;

      if (device_idx == m_device_index_left_hand)
      {
        app_ptr = &m_gh_window;
        current_window_needs_update = gh_window_needs_update;
      }
      else if (device_idx == m_device_index_right_hand)
      {
        app_ptr = &m_rh_window;
        current_window_needs_update = rh_window_needs_update;
      }

      if (app_ptr && app_ptr->m_enabled && app_ptr->m_hwnd)
      {
        RhinoVrAppWindow& app = *app_ptr;

        if (current_window_needs_update)
        {
          RECT window_dim;
          if (GetClientRect(app.m_hwnd, (LPRECT)&window_dim))
          {
            LONG width = window_dim.right - window_dim.left;
            LONG height = window_dim.bottom - window_dim.top;

            if (app.m_width != width || app.m_height != height)
            {
              double aspect = double(width) / height;
              app.m_mesh_width = 0.50*m_unit_scale*aspect;
              app.m_mesh_height = 0.50*m_unit_scale;
              app.m_mesh = CreateAppWindowMesh(app.m_mesh_width, app.m_mesh_height, m_unit_scale);

              ddc.InvalidateWindowMeshCache();
            }

            app.m_width = width;
            app.m_height = height;

            HDC app_hdc = GetDC(app.m_hwnd);

            if (app.m_dib.Width() != width || app.m_dib.Height() != height)
            {
              app.m_dib.ReuseDib(width, height, 32, true);
            }
            else
            {
              app.m_dib.DCSelectBitmap(true);
            }

            if (BitBlt(app.m_dib, 0, 0, width, height, app_hdc, 0, 0, SRCCOPY))
            {
              ON_FileReference file_ref = RhinoGetDibAsTextureFileReference(app.m_dib, app.m_crc);

              ON_Texture tex;
              tex.m_mode = ON_Texture::MODE::decal_texture;
              tex.m_type = ON_Texture::TYPE::bitmap_texture;
              tex.m_minfilter = tex.m_magfilter = ON_Texture::FILTER::nearest_filter;
              tex.m_image_file_reference = file_ref;

              ON_Material mat;
              mat.AddTexture(tex);
              mat.SetDisableLighting(true);

              app.m_material = mat;
            }
          }
        }

        double opacity = app.m_opacity;
        if (opacity >= 0.99)
          opacity = 1.0;

        if (opacity > 0.001)
        {
          draw_pointer = false;
        }

        app.m_material.m_FrontMaterial.m_transparency = (1.0 - opacity);
        ddc.AddWindowMesh(app.m_mesh, &app.m_material);
      }

      if (draw_pointer)
      {
        //ddc.AddLine(m_pointer_line.from, m_pointer_line.to, ON_Color::SaturatedGreen);

        ddc.SetPointerMesh(&m_pointer_mesh);
        ddc.SetPointerMeshMaterial(&m_pointer_mesh_material);
        ddc.SetPointerMeshCacheHandle(&m_pointer_mesh_cache_handle);
      }
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

  if (m_frame_timestamp == 0)
  {
    m_frame_timestamp = RhinoGetTimestamp();
  }
  else
  {
    RhTimestamp now = RhinoGetTimestamp();
    m_last_frame_time = RhinoGetTimeInSecondsBetween(m_frame_timestamp, now);
    m_frame_timestamp = now;
  }

  ON_2dVector camera_translation_vector = ON_2dVector::ZeroVector;
  double camera_translation = 0.0;
  double camera_horizontal_rotation = 0.0;
  double camera_translation_updown = 0.0;

  if (!m_doc->InCommand())
  {
    if (!m_gh_window_left_btn_down &&
        !m_window_intersected_this_frame)
    {
      if (m_device_index_left_hand >= 0 && m_device_index_left_hand < vr::k_unMaxTrackedDeviceCount)
      {
        RhinoVrDeviceController& controller = m_device_data[m_device_index_left_hand].m_controller;

        bool dpad_left  = controller.m_dpad_left_down;
        bool dpad_right = controller.m_dpad_right_down;
        bool dpad_up    = controller.m_dpad_up_down;
        bool dpad_down  = controller.m_dpad_down_down;

        ON_2dVector movement = ON_2dVector::ZeroVector;

        if (dpad_left)
          movement.x = -1.0;
        else if (dpad_right)
          movement.x = +1.0;

        double dir = dpad_up ? 1.0 : (dpad_down ? -1.0 : 0.0);

        bool dpad_up_pressed = controller.m_dpad_up_pressed;
        bool dpad_up_released = controller.m_dpad_up_released;

        bool dpad_down_pressed = controller.m_dpad_down_pressed;
        bool dpad_down_released = controller.m_dpad_down_released;

        if (dpad_up_pressed || dpad_down_pressed)
        {
          m_start_moving_timestamp = RhinoGetTimestamp();
          m_move_speed_when_start_moving = m_move_speed;
        }
        else if (dpad_up || dpad_down)
        {
          double time_since_start_moving = RhinoGetTimeInSecondsSince(m_start_moving_timestamp);

          if ((dir ==  1.0 && m_move_speed < 0.0) ||
              (dir == -1.0 && m_move_speed > 0.0))
          {
            m_start_moving_timestamp = RhinoGetTimestamp();
            m_move_speed_when_start_moving = 0.0;
          }

          m_move_speed = m_move_speed_when_start_moving + dir * time_since_start_moving * MoveAcceleration;
          
          if (m_move_speed > MoveSpeedMax)
            m_move_speed = MoveSpeedMax;
          else if (m_move_speed < -MoveSpeedMax)
            m_move_speed = -MoveSpeedMax;
        }
        else if ((dpad_up_released || dpad_down_released) && m_move_speed != 0.0)
        {
          m_stop_moving_timestamp = RhinoGetTimestamp();
          m_move_speed_when_stop_moving = m_move_speed;
        }
        else if(m_move_speed != 0.0)
        {
          dir = (m_move_speed < 0.0 ? 1.0 : -1.0);

          double time_since_stop_moving = RhinoGetTimeInSecondsSince(m_stop_moving_timestamp);

          m_move_speed = m_move_speed_when_stop_moving + dir * time_since_stop_moving * MoveDecelerationSoft;
          if (dir == 1.0 && m_move_speed > 0.0)
          {
            m_move_speed = 0.0;
          }
          else if (dir == -1.0 && m_move_speed < 0.0)
          {
            m_move_speed = 0.0;
          }
        }

        double move_distance = m_last_frame_time * m_move_speed;
        double turn_distance = m_last_frame_time * m_turn_speed * ON_DEGREES_TO_RADIANS;

        camera_translation = move_distance * m_unit_scale;
        camera_horizontal_rotation = -turn_distance * movement.x;
      }

      if (false)
      {
        // If both X and Y magnitudes are under 0.4 then we don't move/rotate.
        // In other words, the touchpad needs to be touched close to the edge.
        const double threshold = 0.4;

        if (m_device_index_left_hand >= 0 && m_device_index_left_hand < vr::k_unMaxTrackedDeviceCount)
        {
          RhinoVrDeviceController& controller = m_device_data[m_device_index_left_hand].m_controller;

          ON_2dVector analog_vec = controller.m_touchpad_touch_point;

          if (analog_vec.Length() >= threshold)
          {
            ON_2dVector offset_vec = -threshold * analog_vec.UnitVector();
            ON_2dVector translate_vec = analog_vec + offset_vec;

            double move_distance = m_last_frame_time * m_move_speed;
            camera_translation_vector = move_distance * m_unit_scale*translate_vec;
          }
        }

        if (m_device_index_right_hand >= 0 && m_device_index_right_hand < vr::k_unMaxTrackedDeviceCount)
        {
          RhinoVrDeviceController& controller = m_device_data[m_device_index_right_hand].m_controller;

          ON_2dVector analog_vec = controller.m_touchpad_touch_point;

          if (analog_vec.Length() >= threshold)
          {
            ON_2dVector offset_vec = -threshold * analog_vec.UnitVector();
            ON_2dVector rotation_angles = analog_vec + offset_vec;

            double turn_distance = m_last_frame_time * m_turn_speed * ON_DEGREES_TO_RADIANS;
            double move_distance = m_last_frame_time * m_move_speed;

            camera_horizontal_rotation = -turn_distance * rotation_angles.x;
            camera_translation_updown = move_distance * m_unit_scale*rotation_angles.y;
          }
        }
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

  // Transform the HMD to its world position last frame.
  m_vp_hmd.Transform(hmd_to_world_xform);

  {
    // Apply rotation due to controller.
    ON_3dPoint hmd_loc = m_vp_hmd.CameraLocation();
    m_vp_hmd.Rotate(camera_horizontal_rotation, ON_3dVector::ZAxis, hmd_loc);
  }

  if (m_device_index_left_hand >= 0 && m_device_index_left_hand < vr::k_unMaxTrackedDeviceCount)
  {
    const ON_Xform& left_hand_xform = m_device_data[m_device_index_left_hand].m_xform;

    ON_Plane frame = ON_Plane::World_xy;
    frame.Transform(m_cam_to_world_xform * left_hand_xform);

    // Apply translation due to controller.
    ON_3dVector contr_dir   = frame.zaxis;
    ON_3dVector contr_up    = frame.yaxis;
    ON_3dVector contr_right = frame.xaxis;

    ON_3dVector hmd_dolly = ON_3dVector::ZeroVector;
    hmd_dolly += -camera_translation * contr_dir;

    m_vp_hmd.DollyCamera(hmd_dolly);
    m_vp_hmd.DollyFrustum(hmd_dolly.z);
  }

  if(false)
  {
    // Apply translation due to controller.
    ON_3dVector hmd_dir = m_vp_hmd.CameraDirection();
    ON_3dVector hmd_up  = m_vp_hmd.CameraUp();
    ON_3dVector hmd_right = ON_CrossProduct(hmd_dir, hmd_up);

    ON_3dVector hmd_dolly = ON_3dVector::ZeroVector;
    hmd_dolly += camera_translation_vector.x * hmd_right;
    hmd_dolly += camera_translation_vector.y * hmd_dir;
    hmd_dolly += camera_translation_updown * ON_3dVector::ZAxis;

    m_vp_hmd.DollyCamera(hmd_dolly);
    m_vp_hmd.DollyFrustum(hmd_dolly.z);
  }

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

  const UUID& current_display_mode = m_view->DisplayPipeline()->DeferredDisplayMode();
  if (current_display_mode != m_previous_display_mode)
  {
    if (!CreateVrDisplayPipeline(*m_view, *m_vr_vp, &m_vr_dp, &m_vr_dp_ogl))
    {
      RhinoApp().Print("Unable to re-create VR display pipeline. Viewport display mode may look wrong.");
    }

    m_previous_display_mode = current_display_mode;
  }

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

#if defined(RHINOVR_FRAME_TIMING) || defined(RHINOVR_DETAILED_TIMING)
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
  static uint64_t touchpad_btn_mask = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_SteamVR_Touchpad);

  bool touchpad_is_down = (state.ulButtonPressed & touchpad_btn_mask);
  bool touchpad_is_touched = (state.ulButtonTouched & touchpad_btn_mask);

  {
    bool was_down   = controller.m_touchpad_button_down;
    bool is_down    = touchpad_is_down;
    bool is_touched = touchpad_is_touched;

    controller.m_touchpad_button_down     = is_down;
    controller.m_touchpad_button_pressed  = is_down && !was_down;
    controller.m_touchpad_button_released = !is_down && was_down;
    controller.m_touchpad_button_touched  = is_touched;
    controller.m_touchpad_touch_point     = ON_2dPoint(state.rAxis[0].x, state.rAxis[0].y);
  }

  const double dpad_threshold = 0.5;
  ON_2dVector touch_pt = ON_2dVector(state.rAxis[0].x, state.rAxis[0].y);

  // Make sure it's either clearly up/down or left/right.
  if (abs(touch_pt.x) >= abs(touch_pt.y))
  {
    touch_pt.y = 0.0;
  }
  else
  {
    touch_pt.x = 0.0;
  }

  {
    bool was_down = controller.m_dpad_left_down;
    bool is_down = touchpad_is_down && touch_pt.x <= -dpad_threshold;
    if (m_vr_system_type == VrSystemType::Rift)
      is_down = touch_pt.x <= -dpad_threshold;

    controller.m_dpad_left_down     = is_down;
    controller.m_dpad_left_pressed  = is_down && !was_down;
    controller.m_dpad_left_released = !is_down && was_down;
  }

  {
    bool was_down = controller.m_dpad_right_down;
    bool is_down = touchpad_is_down && touch_pt.x >= dpad_threshold;
    if (m_vr_system_type == VrSystemType::Rift)
      is_down = touch_pt.x >= dpad_threshold;

    controller.m_dpad_right_down = is_down;
    controller.m_dpad_right_pressed = is_down && !was_down;
    controller.m_dpad_right_released = !is_down && was_down;
  }

  {
    bool was_down = controller.m_dpad_up_down;
    bool is_down = touchpad_is_down && touch_pt.y >= dpad_threshold;
    if (m_vr_system_type == VrSystemType::Rift)
      is_down = touch_pt.y >= dpad_threshold;

    controller.m_dpad_up_down = is_down;
    controller.m_dpad_up_pressed = is_down && !was_down;
    controller.m_dpad_up_released = !is_down && was_down;
  }

  {
    bool was_down = controller.m_dpad_down_down;
    bool is_down = touchpad_is_down && touch_pt.y <= -dpad_threshold;
    if (m_vr_system_type == VrSystemType::Rift)
      is_down = touch_pt.y <= -dpad_threshold;

    controller.m_dpad_down_down = is_down;
    controller.m_dpad_down_pressed = is_down && !was_down;
    controller.m_dpad_down_released = !is_down && was_down;
  }

  const bool dpad_direction_down =
    controller.m_dpad_up_down ||
    controller.m_dpad_down_down ||
    controller.m_dpad_left_down ||
    controller.m_dpad_right_down;

  {
    bool was_down = controller.m_dpad_center_down;
    bool is_down = touchpad_is_down && !dpad_direction_down;
    if (m_vr_system_type == VrSystemType::Rift)
      is_down = touchpad_is_down && !dpad_direction_down;

    controller.m_dpad_center_down = is_down;
    controller.m_dpad_center_pressed = is_down && !was_down;
    controller.m_dpad_center_released = !is_down && was_down;
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
    bool was_down = controller.m_trigger_button_down;
    bool is_down  = (value == 1.0f);

    controller.m_trigger_button_down     = is_down;
    controller.m_trigger_button_pressed  = is_down && !was_down;
    controller.m_trigger_button_released = !is_down && was_down;
    controller.m_trigger_button_value    = value;
  }

  {
    static uint64_t a_btn_mask = vr::ButtonMaskFromId(vr::EVRButtonId::k_EButton_A);

    bool was_down = controller.m_a_button_down;
    bool is_down  = (state.ulButtonPressed & a_btn_mask);

    controller.m_a_button_down     = is_down;
    controller.m_a_button_pressed  = is_down && !was_down;
    controller.m_a_button_released = !is_down && was_down;
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

bool RhinoVrRenderer::RhinoVrGetIntersectingAppWindow(const RhinoVrAppWindow& app_window, const ON_Xform& ray_xform, const ON_Xform& window_mesh_xform, ON_3dPoint& world_point, ON_2dPoint& screen_uvs)
{
  ON_Line pointer_line = m_pointer_line;
  pointer_line.Transform(window_mesh_xform.Inverse() * ray_xform);

  const ON_Mesh& mesh = app_window.m_mesh;

  const ON_MeshTree* mesh_tree = mesh.MeshTree(true);
  if (mesh_tree)
  {
    ON_SimpleArray<ON_CMX_EVENT> isects;
    if (mesh_tree->IntersectLine(pointer_line, isects) > 0)
    {
      ON_CMX_EVENT& isect = isects[0];
      if (isect.m_type == ON_CMX_EVENT::cmx_point)
      {
        ON_MESH_POINT& pt = isect.m_M[0];
        if (pt.m_ci.m_type == ON_COMPONENT_INDEX::mesh_face)
        {
          ON_2dPoint uv = 
            mesh.m_T[0] * pt.m_t[0] +
            mesh.m_T[1] * pt.m_t[1] +
            mesh.m_T[2] * pt.m_t[2] +
            mesh.m_T[3] * pt.m_t[3];

          world_point = pt.m_P;
          screen_uvs = uv;

          return true;
        }
      }
    }
  }

  return false;
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

POINT ScreenUvToPt(ON_2dPoint screen_uv, LONG width, LONG height)
{
  POINT client_pt;
  client_pt.x = (LONG)floor(screen_uv.x*width);
  client_pt.y = (LONG)floor((1.0 - screen_uv.y)*height);

  return client_pt;
}

bool RhinoVrWindowMouseButtonEvent(HWND hwnd, POINT client_pt, DWORD button_event)
{
  bool rc = false;

  POINT screen_pt = client_pt;
  if (ClientToScreen(hwnd, &screen_pt))
  {
    if (SetCursorPos((int)screen_pt.x, (int)screen_pt.y))
    {
      INPUT input = {};
      input.type = INPUT_MOUSE;
      input.mi.dwFlags = button_event;
      if (SendInput(1, &input, sizeof(INPUT)) > 0)
      {
        rc = true;
      }
    }
  }

  return rc;
}

bool RhinoVrWindowMouseLeftBtnDown(HWND hwnd, POINT client_pt)
{
  return RhinoVrWindowMouseButtonEvent(hwnd, client_pt, MOUSEEVENTF_LEFTDOWN);
}

bool RhinoVrWindowMouseLeftBtnUp(HWND hwnd, POINT client_pt)
{
  return RhinoVrWindowMouseButtonEvent(hwnd, client_pt, MOUSEEVENTF_LEFTUP);
}

bool RhinoVrWindowMouseRightBtnDown(HWND hwnd, POINT client_pt)
{
  return RhinoVrWindowMouseButtonEvent(hwnd, client_pt, MOUSEEVENTF_RIGHTDOWN);
}

bool RhinoVrWindowMouseRightBtnUp(HWND hwnd, POINT client_pt)
{
  return RhinoVrWindowMouseButtonEvent(hwnd, client_pt, MOUSEEVENTF_RIGHTUP);
}

bool RhinoVrWindowMouseLeftClick(HWND hwnd, POINT client_pt)
{
  bool rc = false;

  if (RhinoVrWindowMouseLeftBtnDown(hwnd, client_pt) && RhinoVrWindowMouseLeftBtnUp(hwnd, client_pt))
  {
    rc = true;
  }

  return rc;
}

bool RhinoVrWindowMouseMove(HWND hwnd, POINT client_pt)
{
  static int screen_width = GetSystemMetrics(SM_CXSCREEN);
  static int screen_height = GetSystemMetrics(SM_CYSCREEN);

  bool rc = false;

  POINT screen_pt = client_pt;
  if (ClientToScreen(hwnd, &screen_pt))
  {
    INPUT input = {};
    input.type = INPUT_MOUSE;
    input.mi.dx = (LONG)((double(screen_pt.x) / screen_width) * 0xFFFF);
    input.mi.dy = (LONG)((double(screen_pt.y) / screen_height) * 0xFFFF);
    input.mi.dwFlags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE;
    if (SendInput(1, &input, sizeof(INPUT)) > 0)
    {
      rc = true;
    }
  }

  return rc;
}

bool RhinoVrWindowMouseScroll(HWND hwnd, POINT client_pt, double x, double y)
{
  bool rc = false;

  POINT screen_pt = client_pt;
  if (ClientToScreen(hwnd, &screen_pt))
  {
    if (SetCursorPos((int)screen_pt.x, (int)screen_pt.y))
    {
      if (x != 0.0)
      {
        INPUT input = {};
        input.type = INPUT_MOUSE;
        input.mi.dwFlags = MOUSEEVENTF_HWHEEL;
        input.mi.mouseData = (DWORD)(WHEEL_DELTA * x);
        if (SendInput(1, &input, sizeof(INPUT)) > 0)
        {
          rc = true;
        }
      }

      if (y != 0.0)
      {
        INPUT input = {};
        input.type = INPUT_MOUSE;
        input.mi.dwFlags = MOUSEEVENTF_WHEEL;
        input.mi.mouseData = (DWORD)(WHEEL_DELTA * y);
        if (SendInput(1, &input, sizeof(INPUT)) > 0)
        {
          rc = true;
        }
      }
    }
  }

  return rc;
}

bool RhinoVrRenderer::HandleInput()
{
  m_window_intersected_this_frame = false;

  if (m_hmd == nullptr || m_doc == nullptr || m_view == nullptr)
    return false;

  for (vr::TrackedDeviceIndex_t device_idx = 0; device_idx < vr::k_unMaxTrackedDeviceCount; device_idx++)
  {
    if (m_hmd->GetTrackedDeviceClass(device_idx) != vr::TrackedDeviceClass_Controller)
      continue;

    vr::VRControllerState_t state;
    if (m_hmd->GetControllerState(device_idx, &state, sizeof(state)))
    {
      RhinoVrDeviceController& controller = m_device_data[device_idx].m_controller;
      GetRhinoVrControllerState(state, controller);
    }
  }

  static bool scale_changed = false;

  bool key_pressed_ctrl = (GetKeyState(VK_CONTROL) & 0x8000);
  bool key_pressed_1 = (GetKeyState('1') & 0x8000);
  bool key_pressed_2 = (GetKeyState('2') & 0x8000);

  if (key_pressed_ctrl && !scale_changed)
  {
    float scale = 1.0f;
    if (key_pressed_1)
    {
      scale = 10.0f;
    }
    else if (key_pressed_2)
    {
      scale = 0.1f;
    }

    if (scale != 1.0f)
    {
      scale_changed = true;

      // Change scale here. Not yet implemented.
    }
  }

  if (!key_pressed_1 && !key_pressed_2)
  {
    scale_changed = false;
  }

  m_device_index_left_hand = m_hmd->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_LeftHand);
  m_device_index_right_hand = m_hmd->GetTrackedDeviceIndexForControllerRole(vr::TrackedControllerRole_RightHand);

  vr::VREvent_t event;
  while (m_hmd->PollNextEvent(&event, sizeof(event)))
  {
    ProcessVrEvent(event);
  }

  static bool tried_launching_gh = false;

  // If we lose the window, then reset variables.
  if (m_gh_window.m_hwnd &&
    (IsWindow(m_gh_window.m_hwnd) == FALSE  ||
      IsWindowVisible(m_gh_window.m_hwnd) == FALSE))
  {
    m_gh_window.m_hwnd = nullptr;
    tried_launching_gh = false;
  }

  // If we lose the window, then reset variables.
  if (m_rh_window.m_hwnd && IsWindow(m_rh_window.m_hwnd) == FALSE)
  {
    m_rh_window.m_hwnd = nullptr;
  }

  HWND isect_gh_window = nullptr;
  HWND isect_rh_window = nullptr;
  POINT isect_gh_window_pt = {};
  POINT isect_rh_window_pt = {};

  if (m_device_index_left_hand  < vr::k_unMaxTrackedDeviceCount &&
      m_device_index_right_hand < vr::k_unMaxTrackedDeviceCount)
  {
    const ON_Xform& left_hand_xform = m_device_data[m_device_index_left_hand].m_xform;
    const ON_Xform& right_hand_xform = m_device_data[m_device_index_right_hand].m_xform;

    ON_3dPoint isect_point = ON_3dPoint::UnsetPoint;
    ON_2dPoint window_uv = ON_2dPoint::UnsetPoint;

    if (m_rh_window.m_enabled)
    {
      if (RhinoVrGetIntersectingAppWindow(m_rh_window, left_hand_xform, right_hand_xform, isect_point, window_uv))
      {
        isect_rh_window = m_rh_window.m_hwnd;
        isect_rh_window_pt = ScreenUvToPt(window_uv, m_rh_window.m_width, m_rh_window.m_height);
      }
    }

    if (m_gh_window.m_enabled)
    {
      if (RhinoVrGetIntersectingAppWindow(m_gh_window, right_hand_xform, left_hand_xform, isect_point, window_uv))
      {
        isect_gh_window = m_gh_window.m_hwnd;
        isect_gh_window_pt = ScreenUvToPt(window_uv, m_gh_window.m_width, m_gh_window.m_height);
      }
    }
  }

  HWND isect_window = nullptr;
  POINT isect_window_pt = {};

  if (isect_gh_window)
  {
    isect_window = isect_gh_window;
    isect_window_pt = isect_gh_window_pt;
  }
  else if (isect_rh_window)
  {
    isect_window = isect_rh_window;
    isect_window_pt = isect_rh_window_pt;
  }

  if (isect_window)
  {
    m_window_intersected_this_frame = true;
  }

  for (vr::TrackedDeviceIndex_t device_idx = 0; device_idx < vr::k_unMaxTrackedDeviceCount; device_idx++)
  {
    if (m_hmd->GetTrackedDeviceClass(device_idx) != vr::TrackedDeviceClass_Controller)
      continue;

    RhinoVrDeviceData& device_data = m_device_data[device_idx];
    RhinoVrDeviceController& controller = device_data.m_controller;

    bool is_left_hand = (device_idx == m_device_index_left_hand);
    bool is_right_hand = (device_idx == m_device_index_right_hand);

    if (controller.m_dpad_center_pressed)
    {
      if (is_right_hand && m_doc->InGetPoint())
      {
        RhinoVrGetPoint(device_data.m_xform);
      }
      else if (isect_window)
      {
        RhinoVrWindowMouseLeftBtnDown(isect_window, isect_window_pt);
        if (isect_gh_window)
        {
          m_gh_window_left_btn_down = true;
          m_last_window_click_pos = ON_2iPoint(isect_window_pt.x, isect_window_pt.y);
        }
      }
      else if(is_right_hand)
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
    else if (controller.m_dpad_center_released)
    {
      if (isect_window)
      {
        RhinoVrWindowMouseLeftBtnUp(isect_window, isect_window_pt);
        if (isect_gh_window)
        {
          m_gh_window_left_btn_down = false;
        }
      }
    }
    else if (controller.m_dpad_center_down)
    {
      if (isect_window)
      {
        RhinoVrWindowMouseMove(isect_window, isect_window_pt);
      }
    }
    else if (is_left_hand && controller.m_appmenu_button_pressed)
    {
      RhinoApp().ExecuteCommand(m_doc_sn, L"_Undo");

      // Need to re-attach since ExecuteCommand pumps messages
      // and can basically do anything, including e.g. deleting views.
      if (!AttachDocAndView())
        return false;
    }
    else if (is_right_hand && controller.m_appmenu_button_pressed)
    {
      RhinoApp().ExecuteCommand(m_doc_sn, L"_Move");

      // Need to re-attach since ExecuteCommand pumps messages
      // and can basically do anything, including e.g. deleting views.
      if (!AttachDocAndView())
        return false;
    }
    else if (is_right_hand && controller.m_grip_button_pressed)
    {
      RhinoApp().ExecuteCommand(m_doc_sn, L"_Cancel");

      // Need to re-attach since ExecuteCommand pumps messages
      // and can basically do anything, including e.g. deleting views.
      if (!AttachDocAndView())
        return false;
    }
    else if (is_right_hand &&
      (controller.m_trigger_button_pressed || controller.m_a_button_pressed))
    {
      RhinoApp().ExecuteCommand(m_doc_sn, L"_Enter");

      // Need to re-attach since ExecuteCommand pumps messages
      // and can basically do anything, including e.g. deleting views.
      if (!AttachDocAndView())
        return false;
    }
    else if (is_left_hand && controller.m_trigger_button_released)
    {
      m_gh_window.m_enabled = false;
      m_gh_window_left_btn_down = false;
    }
    else
    {
      if (is_right_hand && m_doc->InGetPoint())
      {
        RhinoVrOnMouseMove(device_data.m_xform);
      }
      else if (isect_window)
      {
        RhinoVrWindowMouseMove(isect_window, isect_window_pt);
      }
    }

    if (is_left_hand)
    {
      if (controller.m_trigger_button_value > 0.001)
      {
        if (m_gh_window.m_hwnd == nullptr && controller.m_trigger_button_value > 0.9)
        {
          InitializeAppWindow(m_gh_window, L"Grasshopper");

          if (m_gh_window.m_hwnd == nullptr && !tried_launching_gh)
          {
            RhinoApp().ExecuteCommand(m_doc_sn, L"Grasshopper");
            tried_launching_gh = true;

            // Need to re-attach since ExecuteCommand pumps messages
            // and can basically do anything, including e.g. deleting views.
            if (!AttachDocAndView())
              return false;
          }
        }
        else if (m_gh_window.m_hwnd == nullptr)
        {
          m_gh_window.m_enabled = false;
          m_gh_window.m_opacity = 1.0;
        }

        m_gh_window.m_enabled = true;
        m_gh_window.m_opacity = controller.m_trigger_button_value;
      }
      else
      {
        m_gh_window.m_enabled = false;
        m_gh_window.m_opacity = 1.0;
      }

      if (controller.m_dpad_up_down || controller.m_dpad_down_down)
      {
        if (isect_gh_window)
        {
          if (!m_gh_window_left_btn_down)
          {
            const double zoom_threshold = 0.4;

            double vertical_offset = controller.m_touchpad_touch_point.y;

            if (abs(vertical_offset) > zoom_threshold)
            {
              double sign = vertical_offset >= 0.0 ? 1.0 : -1.0;
              double zoom_magnitude = sign * (abs(vertical_offset) - zoom_threshold) / (1.0 - zoom_threshold);

              RhinoVrWindowMouseScroll(isect_gh_window, isect_gh_window_pt, 0.0, zoom_magnitude);
            }
          }
        }
      }
    }
    else if (is_right_hand)
    {
      if (isect_gh_window)
      {
        if (!m_gh_window_left_btn_down)
        {
          if (controller.m_trigger_button_pressed)
          {
            RhinoVrWindowMouseRightBtnDown(isect_gh_window, isect_gh_window_pt);
          }
          else if (controller.m_trigger_button_released)
          {
            RhinoVrWindowMouseRightBtnUp(isect_gh_window, isect_gh_window_pt);
          }
        }
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

    if (!HandleInput())
      return;

    if (!UpdateState())
      return;

    if (!Draw())
      return;

    FrameTimingStop();
    FpsTiming();

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

ON_Xform RhinoVrRenderer::OpenVrMatrixToXform(const vr::HmdMatrix34_t& matrix)
{
  ON_Xform xform;

  xform.m_xform[0][0] = matrix.m[0][0];
  xform.m_xform[0][1] = matrix.m[0][1];
  xform.m_xform[0][2] = matrix.m[0][2];
  xform.m_xform[0][3] = matrix.m[0][3] * m_unit_scale;

  xform.m_xform[1][0] = matrix.m[1][0];
  xform.m_xform[1][1] = matrix.m[1][1];
  xform.m_xform[1][2] = matrix.m[1][2];
  xform.m_xform[1][3] = matrix.m[1][3] * m_unit_scale;

  xform.m_xform[2][0] = matrix.m[2][0];
  xform.m_xform[2][1] = matrix.m[2][1];
  xform.m_xform[2][2] = matrix.m[2][2];
  xform.m_xform[2][3] = matrix.m[2][3] * m_unit_scale;

  xform.m_xform[3][0] = 0.0;
  xform.m_xform[3][1] = 0.0;
  xform.m_xform[3][2] = 0.0;
  xform.m_xform[3][3] = 1.0;

  return xform;
}

RhinoVrDeviceModel::RhinoVrDeviceModel(const ON_String& device_name)
  : m_device_name(device_name)
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

void RhinoVrRenderer::FpsTiming()
{
#ifdef RHINOVR_FRAME_TIMING
  if (m_fps_time_start == 0)
  {
    m_fps_time_start = TimingStart();
  }
  else
  {
    m_frame_counter++;

    double time_seconds = RhinoGetTimeInSecondsSince(m_fps_time_start);
    if (time_seconds >= 1.0)
    {
      double frame_time_seconds = time_seconds / m_frame_counter;
      double frames_per_second = 1.0 / frame_time_seconds;

      ON_wString str;
      str.Format(L"Full frame time: %.1f ms. Frames per second: %.1f\n", 1000.0*frame_time_seconds, frames_per_second);

      OutputDebugString(str);

      time_seconds = 0.0;
      m_frame_counter = 0;

      m_fps_time_start = TimingStart();
    }
  }
#endif
}

RhTimestamp RhinoVrRenderer::TimingStart()
{
#if defined(RHINOVR_FRAME_TIMING) || defined(RHINOVR_DETAILED_TIMING)
  return RhinoGetTimestamp();
#else
  return (RhTimestamp)0;
#endif
}

#ifdef RHINOVR_DETAILED_TIMING
void RhinoVrRenderer::TimingStop(const RhTimestamp& start_time, const ON_wString& message)
{
  if (start_time == 0)
    return;

  double time_millis = 1000.0*RhinoGetTimeInSecondsSince(start_time);
  
  ON_wString str;
  str.Format(L"%s: %.1f ms\n", message.Array(), time_millis);

  OutputDebugString(str);
}
#else
void RhinoVrRenderer::TimingStop(const RhTimestamp& /*start_time*/, const ON_wString& /*message*/)
{
}
#endif
