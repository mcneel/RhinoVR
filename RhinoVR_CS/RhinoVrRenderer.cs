
using Rhino;
using Rhino.Display;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Render;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;
using System.Runtime.InteropServices;
using Valve.VR;
using System.Diagnostics;
using Rhino.UI.Runtime;
using System.Security.Cryptography;

// This class represents a VR device. It can be used
// to render the device in Rhino.
public class RhinoVrDeviceModel
{
    public RhinoVrDeviceModel(string device_name)
    {
        m_device_name = device_name;
    }

    public bool Initialize(RenderModel_t model, RenderModel_TextureMap_t diffuse_texture,
        float unit_scale, RhinoDoc doc)
    {
        m_device_mesh = new Mesh();

        int vertex_count = (int)model.unVertexCount;

        m_device_mesh.Vertices.Capacity = vertex_count;
        m_device_mesh.Vertices.Count    = vertex_count;

        m_device_mesh.Normals.Capacity = vertex_count;
        m_device_mesh.Normals.Count    = vertex_count;

        m_device_mesh.TextureCoordinates.Capacity = vertex_count;
        m_device_mesh.TextureCoordinates.Count    = vertex_count;

        var vertices = OpenVrMarshal.FromIntPtrToArray<RenderModel_Vertex_t>(model.rVertexData, model.unVertexCount);

        for (int vi = 0; vi < vertex_count; ++vi)
        {
            RenderModel_Vertex_t rm_vertex = vertices[vi];

            HmdVector3_t v = rm_vertex.vPosition;
            HmdVector3_t n = rm_vertex.vNormal;
            float uv0 = rm_vertex.rfTextureCoord0;
            float uv1 = rm_vertex.rfTextureCoord1;
            
            m_device_mesh.Vertices[vi] = new Point3f(v.v0, v.v1, v.v2) * unit_scale;
            m_device_mesh.Normals[vi] = new Vector3f(n.v0, n.v1, n.v2);
            m_device_mesh.TextureCoordinates[vi] = new Point2f(uv0, 1.0 - uv1);
        }

        var triangle_count = (int)model.unTriangleCount;

        m_device_mesh.Faces.Capacity = triangle_count;
        m_device_mesh.Faces.Count    = triangle_count;

        var vertex_indices = OpenVrMarshal.FromIntPtrToArray<UInt16>(model.rIndexData, model.unTriangleCount*3);

        for (int fi = 0, idx = 0; fi < triangle_count; ++fi)
        {
            var triangle = new MeshFace
            {
                A = vertex_indices[idx++],
                B = vertex_indices[idx++],
                C = vertex_indices[idx++]
            };
            triangle.D = triangle.C;

            m_device_mesh.Faces[fi] = triangle;
        }

        var pixels = OpenVrMarshal.FromIntPtrToArray<byte>(
            diffuse_texture.rubTextureMapData, (uint)diffuse_texture.unWidth * diffuse_texture.unHeight * 4);

        var bitmap = new Bitmap(diffuse_texture.unWidth, diffuse_texture.unHeight);
        for (int y = 0; y < bitmap.Height; ++y)
        {
            for (int x = 0; x < bitmap.Width; ++x)
            {
                int pixel_idx = 4 * (y * bitmap.Width + x);

                byte r = pixels[pixel_idx + 0];
                byte g = pixels[pixel_idx + 1];
                byte b = pixels[pixel_idx + 2];
                byte a = pixels[pixel_idx + 3];

                bitmap.SetPixel(x, y, Color.FromArgb(a, r, g, b));
            }
        }

        var render_texture = RenderTexture.NewBitmapTexture(bitmap, doc);
        var rdk_material = RenderMaterial.CreateBasicMaterial(new Material(), doc);

        var child_slot_name = rdk_material.TextureChildSlotName(RenderMaterial.StandardChildSlots.Diffuse);

        rdk_material.SetChild(render_texture, child_slot_name);
        rdk_material.SetChildSlotOn(child_slot_name, true, RenderContent.ChangeContexts.Program);
        rdk_material.SetChildSlotAmount(child_slot_name, 100.0, RenderContent.ChangeContexts.Program);

        m_device_material = new DisplayMaterial(rdk_material.SimulateMaterial(false));

        return true;
    }

    public string m_device_name;
    public Mesh m_device_mesh = null;
    public DisplayMaterial m_device_material = null;
}

// This struct represents the current state of
// a VR controller.
public class RhinoVrDeviceController
{
    public bool m_touchpad_button_down = false;
    public bool m_touchpad_button_pressed = false;
    public bool m_touchpad_button_released = false;
    public bool m_touchpad_button_touched = false;

    public bool m_appmenu_button_down = false;
    public bool m_appmenu_button_pressed = false;
    public bool m_appmenu_button_released = false;

    public bool m_grip_button_down = false;
    public bool m_grip_button_pressed = false;
    public bool m_grip_button_released = false;

    public bool m_trigger_button_down = false;
    public bool m_trigger_button_pressed = false;
    public bool m_trigger_button_released = false;

    public bool m_a_button_down = false;
    public bool m_a_button_pressed = false;
    public bool m_a_button_released = false;

    public bool m_dpad_left_down = false;
    public bool m_dpad_left_pressed = false;
    public bool m_dpad_left_released = false;

    public bool m_dpad_right_down = false;
    public bool m_dpad_right_pressed = false;
    public bool m_dpad_right_released = false;

    public bool m_dpad_up_down = false;
    public bool m_dpad_up_pressed = false;
    public bool m_dpad_up_released = false;

    public bool m_dpad_down_down = false;
    public bool m_dpad_down_pressed = false;
    public bool m_dpad_down_released = false;

    public bool m_dpad_center_down = false;
    public bool m_dpad_center_pressed = false;
    public bool m_dpad_center_released = false;

    public float m_trigger_button_value = 0.0f;
    public Point2d m_touchpad_touch_point = Point2d.Origin;
};

public class RhinoVrAppWindow
{
    public bool m_enabled = false;
    public string m_title = null;
    public string m_md5_str = null;
    public IntPtr m_hwnd = IntPtr.Zero;
    public Bitmap m_dib = null;
    public uint m_width = 0;
    public uint m_height = 0;
    public Mesh m_mesh = null;
    public double m_mesh_width = 0.0;
    public double m_mesh_height = 0.0;
    public double m_opacity = 1.0;
    public DisplayMaterial m_material = null;
};

// This struct contains up-to-date information of a tracked
// VR device, such as location/orientation, geometry, and
// the state of any buttons/triggers/touchpads.
public class RhinoVrDeviceData
{
    public bool m_show = false;
    public Transform m_xform = Transform.Identity;
    public RhinoVrDeviceModel m_render_model = null;
    public RhinoVrDeviceDisplayConduit m_display_conduit = new RhinoVrDeviceDisplayConduit();
    public RhinoVrDeviceController m_controller = new RhinoVrDeviceController();
};

// RhinoVrRenderer is the main class of RhinoVR. It handles VR library
// initialization, input handling and frame drawing.
public class RhinoVrRenderer : IDisposable
{
    public RhinoVrRenderer(uint doc_sn, uint view_sn)
    {
        m_doc_sn = doc_sn;
        m_view_sn = view_sn;

        for (int i = 0; i < m_device_poses.Length; ++i)
        {
            m_device_poses[i] = new TrackedDevicePose_t();
        }

        for (int i = 0; i < OpenVR.k_unMaxTrackedDeviceCount; ++i)
        {
            m_device_data.Add(new RhinoVrDeviceData());
        }
    }

    private bool disposed = false; // To detect redundant calls

    protected virtual void Dispose(bool disposing)
    {
        if (!disposed)
        {
            if (disposing)
            {
                RhinoApp.DisableContinuousMainLoop();

                OpenVR.Shutdown();

                RhinoApp.WriteLine("RhinoVR is OFF.");
            }

            disposed = true;
        }
    }

    // This code added to correctly implement the disposable pattern.
    public void Dispose()
    {
        Dispose(true);
    }

    public bool Initialize()
    {
        RhinoApp.WriteLine("Initializing RhinoVR...");

        var init_error = EVRInitError.None;

        OpenVR.Init(ref init_error);

        if (init_error != EVRInitError.None)
        {
            RhinoApp.WriteLine("Unable to initialize VR runtime: {0}", OpenVR.GetStringForHmdError(init_error));
            return false;
        }

        OpenVR.GetGenericInterface(OpenVR.IVRRenderModels_Version, ref init_error);

        if (init_error != EVRInitError.None)
        {
            RhinoApp.WriteLine("Unable to get render models interface: {0}", OpenVR.GetStringForHmdError(init_error));
            return false;
        }

        OpenVR.GetGenericInterface(OpenVR.IVRCompositor_Version, ref init_error);

        if (init_error != EVRInitError.None)
        {
            RhinoApp.WriteLine("Unable to initialize VR compositor: {0}", OpenVR.GetStringForHmdError(init_error));
            return false;
        }

        m_doc = RhinoDoc.FromRuntimeSerialNumber(m_doc_sn);
        if (m_doc == null)
            return false;
        
        m_unit_scale = RhinoMath.UnitScale(m_doc.ModelUnitSystem, UnitSystem.Meters);

        var start = new Point3d(0.0, 0.0, -0.02);
        var end = new Point3d(0.0, 0.0, -500.0);
        m_pointer_line = new Line(m_unit_scale * start, m_unit_scale * end);

        SetupRenderModels();

        uint rec_width = 0, rec_height = 0;
        OpenVR.System.GetRecommendedRenderTargetSize(ref rec_width, ref rec_height);

        // For now, let's force the resolution to be the same as the native
        // screen resolution of both the Vive and the Oculus.
        rec_width = 1080;
        rec_height = 1200;

        RhinoView view = RhinoView.FromRuntimeSerialNumber(m_view_sn);
        if (view == null)
            return false;

        m_vr_vp = new RhinoViewport(view.ActiveViewport)
        {
            Size = new Size((int)rec_width, (int)rec_height)
        };

        DisplayPipeline view_dp = view.DisplayPipeline;
        if (view_dp == null)
            return false;

        view_dp.Open();
        m_vr_dp = view_dp.Clone(m_vr_vp);
        view_dp.Close();

        if (m_vr_dp == null)
            return false;

        m_vp_orig = new ViewportInfo(m_vr_dp.Viewport);
        var vp = new ViewportInfo(m_vp_orig);

        m_near_clip = (float)(m_unit_scale * 0.01);
        m_far_clip = (float)(m_near_clip / vp.PerspectiveMinNearOverFar);

        OpenVR.System.GetProjectionRaw(EVREye.Eye_Left,
            ref m_left_frus_left, ref m_left_frus_right,
            ref m_left_frus_top, ref m_left_frus_bottom);

        m_left_frus_left *= m_near_clip;
        m_left_frus_right *= m_near_clip;
        m_left_frus_top *= m_near_clip;
        m_left_frus_bottom *= m_near_clip;

        OpenVR.System.GetProjectionRaw(EVREye.Eye_Right,
            ref m_right_frus_left, ref m_right_frus_right,
            ref m_right_frus_top, ref m_right_frus_bottom);

        m_right_frus_left *= m_near_clip;
        m_right_frus_right *= m_near_clip;
        m_right_frus_top *= m_near_clip;
        m_right_frus_bottom *= m_near_clip;

        // We have asymmetric frustums
        vp.UnlockFrustumSymmetry();
        vp.SetFrustum(
            m_left_frus_left, m_right_frus_right,
            m_left_frus_top, m_left_frus_bottom,
            m_near_clip, m_far_clip);

        m_vp_orig_hmd_frus = new ViewportInfo(vp);

        {
            // We imitate the VR view in the Rhino viewport.
            int vp_new_width = vp.ScreenPort.Width / 2;
            int vp_new_height = (int)Math.Floor(vp_new_width / vp.FrustumAspect + 0.5);

            string script = string.Format("-_ViewportProperties _Size {0} {1} _Enter", vp_new_width, vp_new_height);
            RhinoApp.RunScript(script, false);

            view.ActiveViewport.SetViewProjection(vp, true);
            view.Redraw();
        }

        if(!RhinoApp.EnableContinuousMainLoop())
        {
            RhinoApp.WriteLine("RhinoVR warning: Failed to enable continuous Rhino main loop. RhinoVR performance may suffer.");
        }

        RhinoApp.WriteLine("RhinoVR is ON. Please put on the VR headset.");

        return true;
    }

    public void ProcessInputAndRenderFrame()
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
            FpsTiming();

            DetachDocAndView();
        }

        RhinoTimingStart();
    }

    // Converts an OpenVR matrix to a Transform.
    protected Transform OpenVrMatrixToRhinoTransform(HmdMatrix34_t matrix)
    {
        Transform transform = new Transform();

        transform[0, 0] = matrix.m0;
        transform[0, 1] = matrix.m1;
        transform[0, 2] = matrix.m2;
        transform[0, 3] = matrix.m3 * m_unit_scale;

        transform[1, 0] = matrix.m4;
        transform[1, 1] = matrix.m5;
        transform[1, 2] = matrix.m6;
        transform[1, 3] = matrix.m7 * m_unit_scale;

        transform[2, 0] = matrix.m8;
        transform[2, 1] = matrix.m9;
        transform[2, 2] = matrix.m10;
        transform[2, 3] = matrix.m11 * m_unit_scale;

        transform[3, 0] = 0.0;
        transform[3, 1] = 0.0;
        transform[3, 2] = 0.0;
        transform[3, 3] = 1.0;

        return transform;
    }

    // Loads all needed render models from the VR library.
    protected void SetupRenderModels()
    {
        for (uint device_idx = OpenVR.k_unTrackedDeviceIndex_Hmd; device_idx < OpenVR.k_unMaxTrackedDeviceCount; ++device_idx)
        {
            if (!OpenVR.System.IsTrackedDeviceConnected(device_idx))
                continue;

            SetupRenderModelForDevice(device_idx);
        }

        m_hidden_mesh_left = LoadHiddenAreaMesh(EVREye.Eye_Left);
        m_hidden_mesh_right = LoadHiddenAreaMesh(EVREye.Eye_Right);
    }

    private string GetTrackedDeviceString(uint device_idx, ETrackedDeviceProperty device_property, out ETrackedPropertyError error)
    {
        error = ETrackedPropertyError.TrackedProp_Success;

        uint required_buffer_len = OpenVR.System.GetStringTrackedDeviceProperty(device_idx, device_property, null, 0, ref error);
        if (required_buffer_len == 0)
            return string.Empty;

        var string_builder = new StringBuilder();

        OpenVR.System.GetStringTrackedDeviceProperty(device_idx, device_property, string_builder, required_buffer_len, ref error);

        return string_builder.ToString();
    }

    // Loads a render model for a specific device.
    protected void SetupRenderModelForDevice(uint device_index)
    {
        if (device_index >= OpenVR.k_unMaxTrackedDeviceCount)
            return;

        string system_name = GetTrackedDeviceString(device_index, ETrackedDeviceProperty.Prop_TrackingSystemName_String, out ETrackedPropertyError error);

        ETrackedDeviceClass device_class = OpenVR.System.GetTrackedDeviceClass(device_index);
        if (device_class == ETrackedDeviceClass.HMD)
        {
            if (system_name.Equals("oculus", StringComparison.Ordinal))
            {
                m_vr_system_type = VrSystemType.Rift;
            }
            else
            {
                m_vr_system_type = VrSystemType.Vive;
            }
        }

        string render_model_name = GetTrackedDeviceString(device_index, ETrackedDeviceProperty.Prop_RenderModelName_String, out error);

        // We don't want to show the headset, Vive base stations or the Rift cameras.
        if (render_model_name.Equals("generic_hmd", StringComparison.Ordinal) ||
            render_model_name.Equals("lh_basestation_vive", StringComparison.Ordinal) ||
            render_model_name.Equals("rift_camera", StringComparison.Ordinal))
        {
            return;
        }

        RhinoVrDeviceModel render_model = FindOrLoadRenderModel(render_model_name);
        if (render_model == null)
        {
            RhinoApp.WriteLine("Unable to load render model for tracked device {0} ({1}.{2})",
              device_index, system_name, render_model_name);
        }
        else
        {
            RhinoVrDeviceData device_data = m_device_data[(int)device_index];
            device_data.m_render_model = render_model;
            device_data.m_show = true;
        }
    }

    // Loads the "hidden area mesh" geometry which is intended to block
    // out the parts of the screens which won't be visible to the eyes.
    // This is done to improve rendering  performance.
    protected Mesh LoadHiddenAreaMesh(EVREye eye)
    {
        HiddenAreaMesh_t hidden_mesh = OpenVR.System.GetHiddenAreaMesh(eye, EHiddenAreaMeshType.k_eHiddenAreaMesh_Standard);

        var vertex_data = OpenVrMarshal.FromIntPtrToArray<HmdVector2_t>(hidden_mesh.pVertexData, hidden_mesh.unTriangleCount * 3);

        var mesh = new Mesh();
        mesh.Vertices.Capacity = vertex_data.Length;
        mesh.Vertices.Count = vertex_data.Length;

        for (int vi = 0; vi < vertex_data.Length; ++vi)
        {
            double x = vertex_data[vi].v0;
            double y = vertex_data[vi].v1;

            mesh.Vertices[vi] = new Point3f((float)(2.0 * x - 1.0), (float)(2.0 * y - 1.0), 1.0f);
        }

        mesh.Faces.Capacity = (int)hidden_mesh.unTriangleCount;
        mesh.Faces.Count = (int)hidden_mesh.unTriangleCount;

        for (int fi = 0, vi = 0; fi < mesh.Faces.Count; ++fi)
        {
            MeshFace face = mesh.Faces[fi];
            face.A = vi++;
            face.B = vi++;
            face.C = vi++;
            face.D = face.C;
        }

        return mesh;
    }

    // Returns a render model by name. If it hasn't been loaded yet, it will
    // first be loaded from the VR library.
    protected RhinoVrDeviceModel FindOrLoadRenderModel(string render_model_name)
    {
        if (m_doc == null)
            return null;

        RhinoVrDeviceModel render_model = null;
        foreach (RhinoVrDeviceModel dm in m_device_render_models)
        {
            if (dm.m_device_name.Equals(render_model_name, StringComparison.Ordinal))
            {
                render_model = dm;
                break;
            }
        }

        if (render_model == null)
        {
            EVRRenderModelError error;

            var model_ptr = new IntPtr();

            while (true)
            {
                error = OpenVR.RenderModels.LoadRenderModel_Async(render_model_name, ref model_ptr);
                if (error != EVRRenderModelError.Loading)
                {
                    break;
                }

                System.Threading.Thread.Sleep(1);
            }

            if (error != EVRRenderModelError.None)
            {
                RhinoApp.WriteLine("Unable to load render model {0} - {1}", render_model_name, OpenVR.RenderModels.GetRenderModelErrorNameFromEnum(error));
                return null;
            }

            var model = (RenderModel_t)Marshal.PtrToStructure(model_ptr, typeof(RenderModel_t));

            var texture_ptr = new IntPtr();

            while (true)
            {
                error = OpenVR.RenderModels.LoadTexture_Async(model.diffuseTextureId, ref texture_ptr);
                if (error != EVRRenderModelError.Loading)
                {
                    break;
                }

                System.Threading.Thread.Sleep(1);
            }

            if (error != EVRRenderModelError.None)
            {
                OpenVR.RenderModels.FreeRenderModel(model_ptr);

                RhinoApp.WriteLine("Unable to load render texture id:{0} for render model {1}", model.diffuseTextureId, render_model_name);

                return null;
            }

            var texture = (RenderModel_TextureMap_t)Marshal.PtrToStructure(texture_ptr, typeof(RenderModel_TextureMap_t));

            render_model = new RhinoVrDeviceModel(render_model_name);

            if (render_model.Initialize(model, texture, (float)m_unit_scale, m_doc))
            {
                m_device_render_models.Add(render_model);
            }
            else
            {
                RhinoApp.WriteLine("Unable to create Rhino Mesh model from render model {0}", render_model_name);
            }

            OpenVR.RenderModels.FreeRenderModel(model_ptr);
            OpenVR.RenderModels.FreeTexture(texture_ptr);
        }

        return render_model;
    }

    // Provide device display conduits with updated device information.
    protected void UpdateDeviceDisplayConduits(
        Transform camera_to_world_xform,
        Transform clip_to_left_eye_xform,
        Transform clip_to_right_eye_xform)
    {
        bool is_input_available = OpenVR.System.IsInputAvailable();
        
        double time_since_update = m_last_window_update_stopwatch.Elapsed.TotalSeconds;
        
        bool gh_window_needs_update = false;
        bool rh_window_needs_update = false;

        if (time_since_update >= 1.0 / 20.0)
        {
            m_last_window_update_stopwatch.Restart();

            if (m_rh_window.m_hwnd != IntPtr.Zero && m_rh_window.m_enabled && m_last_window_updated == 0)
            {
                rh_window_needs_update = true;
                m_last_window_updated = 1;
            }
            else if (m_gh_window.m_hwnd != IntPtr.Zero && m_gh_window.m_enabled && m_last_window_updated == 1)
            {
                gh_window_needs_update = true;
                m_last_window_updated = 0;
            }
            else if (m_rh_window.m_hwnd != IntPtr.Zero && m_rh_window.m_enabled)
            {
                rh_window_needs_update = true;
                m_last_window_updated = 1;
            }
            else if (m_gh_window.m_hwnd != IntPtr.Zero && m_gh_window.m_enabled)
            {
                gh_window_needs_update = true;
                m_last_window_updated = 0;
            }
        }

        for (uint device_idx = 0; device_idx < OpenVR.k_unMaxTrackedDeviceCount; device_idx++)
        {
            RhinoVrDeviceData device_data = m_device_data[(int)device_idx];

            RhinoVrDeviceDisplayConduit ddc = device_data.m_display_conduit;
            ddc.Empty();

            RhinoVrDeviceModel device_model = device_data.m_render_model;

            if (device_model == null || !device_data.m_show)
                continue;

            TrackedDevicePose_t pose = m_device_poses[device_idx];
            if (!pose.bPoseIsValid)
                continue;

            bool is_controller = (OpenVR.System.GetTrackedDeviceClass(device_idx) == ETrackedDeviceClass.Controller);

            if (!is_input_available && is_controller)
                continue;

            if (is_controller)
            {
                bool draw_pointer = true;

                RhinoVrAppWindow app = null;
                bool current_window_needs_update = false;

                if (device_idx == m_device_index_left_hand)
                {
                    app = m_gh_window;
                    current_window_needs_update = gh_window_needs_update;
                }
                else if (device_idx == m_device_index_right_hand)
                {
                    app = m_rh_window;
                    current_window_needs_update = rh_window_needs_update;
                }

                if (app != null && app.m_enabled && app.m_hwnd != IntPtr.Zero)
                {
                    if (current_window_needs_update)
                    {
                        // TODO: Implement later once we can use 'GetClientRect', 'GetDC',
                        // 'BitBlt' and more in C#/RhinoCommon/Eto...

                        /*
                        RECT window_dim;
                        if (GetClientRect(app.m_hwnd, (LPRECT) & window_dim))
                        {
                            LONG width = window_dim.right - window_dim.left;
                            LONG height = window_dim.bottom - window_dim.top;

                            if (app.m_width != width || app.m_height != height)
                            {
                                double aspect = double(width) / height;
                                app.m_mesh_width = 0.50 * m_unit_scale * aspect;
                                app.m_mesh_height = 0.50 * m_unit_scale;
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
                        */
                    }

                    double opacity = app.m_opacity;
                    if (opacity >= 0.99)
                        opacity = 1.0;

                    if (opacity > 0.001)
                    {
                        draw_pointer = false;
                    }

                    app.m_material.Transparency = (1.0 - opacity);
                    ddc.AddWindowMesh(app.m_mesh, app.m_material);
                }

                if (draw_pointer)
                {
                    ddc.SetPointerMesh(m_pointer_mesh);
                    ddc.SetPointerMeshMaterial(m_pointer_mesh_material);
                }
            }

            Mesh device_mesh = device_model.m_device_mesh;
            DisplayMaterial device_material = device_model.m_device_material;
            Transform device_xform = camera_to_world_xform * device_data.m_xform;

            ddc.SetDeviceMesh(device_mesh);
            ddc.SetDeviceMaterial(device_material);
            ddc.SetDeviceMeshXform(device_xform);

            if (!ddc.Enabled)
            {
                ddc.Enabled = true;
            }
        }

        if(m_hidden_mesh_display_conduit != null)
        {
            m_hidden_mesh_display_conduit.SetHiddenAreaMesh(m_hidden_mesh_left, EVREye.Eye_Left);
            m_hidden_mesh_display_conduit.SetHiddenAreaMesh(m_hidden_mesh_right, EVREye.Eye_Right);

            m_hidden_mesh_display_conduit.SetHiddenAreaMeshXform(clip_to_left_eye_xform, EVREye.Eye_Left);
            m_hidden_mesh_display_conduit.SetHiddenAreaMeshXform(clip_to_right_eye_xform, EVREye.Eye_Right);
            
            if (!m_hidden_mesh_display_conduit.Enabled)
            {
                m_hidden_mesh_display_conduit.Enabled = true;
            }
        }
    }

    protected bool AttachDocAndView()
    {
        m_doc = RhinoDoc.FromRuntimeSerialNumber(m_doc_sn);
        if (m_doc == null)
            return false;

        m_view = RhinoView.FromRuntimeSerialNumber(m_view_sn);
        if (m_view == null)
            return false;

        return true;
    }

    protected void DetachDocAndView()
    {
        m_doc = null;
        m_view = null;
    }

    // Updates the transforms of all tracked devices.
    // This function will wait for the HMD's vertical sync signal.
    protected bool UpdatePosesAndWaitForVSync()
    {
        OpenVR.Compositor.WaitGetPoses(m_device_poses, m_game_poses);

        return true;
    }

    // Updates the device and controller states. Calculates view orientation.
    // Sets view frustum and near/far planes.
    protected bool UpdateState()
    {
        if (!m_movement_stopwatch.IsRunning)
        {
            m_movement_stopwatch.Start();
        }
        else
        {
            m_last_frame_time = m_movement_stopwatch.ElapsedMilliseconds / 1000.0;
            m_movement_stopwatch.Restart();
        }

        Vector2d camera_translation_vector = Vector2d.Zero;
        double camera_translation = 0.0;
        double camera_horizontal_rotation = 0.0;

        if (!m_doc.IsCommandRunning)
        {
            if (!m_gh_window_left_btn_down &&
                !m_window_intersected_this_frame)
            {
                if (m_device_index_left_hand >= 0 && m_device_index_left_hand < OpenVR.k_unMaxTrackedDeviceCount)
                {
                    RhinoVrDeviceController controller = m_device_data[(int)m_device_index_left_hand].m_controller;

                    bool dpad_left = controller.m_dpad_left_down;
                    bool dpad_right = controller.m_dpad_right_down;
                    bool dpad_up = controller.m_dpad_up_down;
                    bool dpad_down = controller.m_dpad_down_down;

                    Vector2d movement = Vector2d.Zero;

                    if (dpad_left)
                        movement.X = -1.0;
                    else if (dpad_right)
                        movement.X = +1.0;

                    double dir = dpad_up ? 1.0 : (dpad_down ? -1.0 : 0.0);

                    bool dpad_up_pressed = controller.m_dpad_up_pressed;
                    bool dpad_up_released = controller.m_dpad_up_released;

                    bool dpad_down_pressed = controller.m_dpad_down_pressed;
                    bool dpad_down_released = controller.m_dpad_down_released;

                    if (dpad_up_pressed || dpad_down_pressed)
                    {
                        m_start_moving_stopwatch.Restart();
                        m_move_speed_when_start_moving = m_move_speed;
                    }
                    else if (dpad_up || dpad_down)
                    {
                        double time_since_start_moving = m_start_moving_stopwatch.Elapsed.TotalSeconds;

                        if ((dir == 1.0 && m_move_speed < 0.0) ||
                            (dir == -1.0 && m_move_speed > 0.0))
                        {
                            m_start_moving_stopwatch.Restart();
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
                        m_stop_moving_stopwatch.Restart();
                        m_move_speed_when_stop_moving = m_move_speed;
                    }
                    else if (m_move_speed != 0.0)
                    {
                        dir = (m_move_speed < 0.0 ? 1.0 : -1.0);

                        double time_since_stop_moving = m_stop_moving_stopwatch.Elapsed.TotalSeconds;

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
                    double turn_distance = m_last_frame_time * RhinoMath.ToRadians(m_turn_speed);

                    camera_translation = move_distance * m_unit_scale;
                    camera_horizontal_rotation = -turn_distance * movement.X;
                }
            }
        }

        uint hmd_device_index = OpenVR.k_unTrackedDeviceIndex_Hmd;

        TrackedDevicePose_t hmd_device_pose = m_device_poses[hmd_device_index];

        if (!m_hmd_location_correction_acquired && hmd_device_pose.bPoseIsValid)
        {
            m_hmd_location_correction_acquired = true;

            Transform xform = OpenVrMatrixToXform(hmd_device_pose.mDeviceToAbsoluteTracking);
            m_hmd_location_correction_xform = Transform.Translation(new Vector3d(-xform.M03, -xform.M13, -xform.M23));
        }

        for (int device_idx = 0; device_idx < OpenVR.k_unMaxTrackedDeviceCount; ++device_idx)
        {
            TrackedDevicePose_t device_pose = m_device_poses[device_idx];

            if (device_pose.bPoseIsValid)
            {
                HmdMatrix34_t ovr_device_matrix = device_pose.mDeviceToAbsoluteTracking;
                m_device_data[device_idx].m_xform = m_hmd_location_correction_xform * OpenVrMatrixToXform(ovr_device_matrix);
            }
        }

        if (hmd_device_pose.bPoseIsValid)
        {
            m_hmd_xform = m_device_data[(int)hmd_device_index].m_xform;

            m_left_eye_xform = OpenVrMatrixToXform(OpenVR.System.GetEyeToHeadTransform(EVREye.Eye_Left));
            m_right_eye_xform = OpenVrMatrixToXform(OpenVR.System.GetEyeToHeadTransform(EVREye.Eye_Right));
        }

        ViewportInfo rhino_vp = new ViewportInfo(m_view.ActiveViewport);

        var vp = new ViewportInfo(m_vp_orig_hmd_frus)
        {
            IsParallelProjection = rhino_vp.IsParallelProjection,
            IsPerspectiveProjection = rhino_vp.IsPerspectiveProjection
        };
        vp.SetCameraLocation(Point3d.Origin);
        vp.SetCameraDirection(-Vector3d.ZAxis);
        vp.SetCameraUp(Vector3d.YAxis);
        vp.TargetPoint = new Point3d(-Vector3d.ZAxis * rhino_vp.TargetDistance(true));
        vp.SetFrustumNearFar(m_near_clip, m_far_clip);

        if (m_cam_to_world_xform == Transform.Unset)
        {
            // If not set, initialize to Rhino camera xform.
            m_cam_to_world_xform = rhino_vp.GetXform(CoordinateSystem.Camera, CoordinateSystem.World);
        }

        m_vp_hmd = new ViewportInfo(vp);

        Transform hmd_to_world_xform = m_cam_to_world_xform * m_hmd_xform;

        // Transform the HMD to its world position last frame.
        m_vp_hmd.TransformCamera(hmd_to_world_xform);

        {
            // Apply rotation due to controller.
            Point3d hmd_loc = m_vp_hmd.CameraLocation;
            m_vp_hmd.RotateCamera(camera_horizontal_rotation, Vector3d.ZAxis, hmd_loc);
        }

        if (m_device_index_left_hand >= 0 && m_device_index_left_hand < OpenVR.k_unMaxTrackedDeviceCount)
        {
            Transform left_hand_xform = m_device_data[(int)m_device_index_left_hand].m_xform;

            Plane frame = Plane.WorldXY;
            frame.Transform(m_cam_to_world_xform * left_hand_xform);

            // Apply translation due to controller.
            Vector3d contr_dir = frame.ZAxis;
            Vector3d contr_up = frame.YAxis;
            Vector3d contr_right = frame.XAxis;

            Vector3d hmd_dolly = Vector3d.Zero;
            hmd_dolly += -camera_translation * contr_dir;

            m_vp_hmd.DollyCamera(hmd_dolly);
            m_vp_hmd.DollyFrustum(hmd_dolly.Z);
        }

        // Now extract the final xform which includes movements from the controller.
        Transform hmd_to_world_final_xform = m_vp_hmd.GetXform(CoordinateSystem.Camera, CoordinateSystem.World);

        // We get the controller-only xform by "subtracting" away hmd_to_world.
        if(!hmd_to_world_xform.TryGetInverse(out Transform hmd_to_world_xform_inverse))
        {
            hmd_to_world_xform_inverse = Transform.Identity;
        }

        Transform controller_xform = hmd_to_world_final_xform * hmd_to_world_xform_inverse;

        // We update cam_to_world to incorporate the controller xform.
        m_cam_to_world_xform = controller_xform * m_cam_to_world_xform;

        m_vr_vp.SetViewProjection(m_vp_hmd, true);

        Transform cam_to_left_eye_xform = hmd_to_world_final_xform * m_left_eye_xform;

        m_vp_left_eye = new ViewportInfo(vp);

        m_vp_left_eye.TransformCamera(cam_to_left_eye_xform);
        m_vp_left_eye.SetFrustum(
          m_left_frus_left, m_left_frus_right,
          m_left_frus_top, m_left_frus_bottom,
          m_near_clip, m_far_clip);
        
        Transform clip_to_left_eye_xform = m_vp_left_eye.GetXform(CoordinateSystem.Clip, CoordinateSystem.World);

        Transform cam_to_right_eye_xform = hmd_to_world_final_xform * m_right_eye_xform;

        m_vp_right_eye = new ViewportInfo(vp);

        m_vp_right_eye.TransformCamera(cam_to_right_eye_xform);
        m_vp_right_eye.SetFrustum(
          m_right_frus_left, m_right_frus_right,
          m_right_frus_top, m_right_frus_bottom,
          m_near_clip, m_far_clip);

        Transform clip_to_right_eye_xform = m_vp_right_eye.GetXform(CoordinateSystem.Clip, CoordinateSystem.World);

        UpdateDeviceDisplayConduits(m_cam_to_world_xform, clip_to_left_eye_xform, clip_to_right_eye_xform);

        return true;
    }

    // Performs actions based on controller inputs.
    protected bool HandleInput()
    {
        m_window_intersected_this_frame = false;

        for (uint device_idx = 0; device_idx < OpenVR.k_unMaxTrackedDeviceCount; device_idx++)
        {
            if (OpenVR.System.GetTrackedDeviceClass(device_idx) != ETrackedDeviceClass.Controller)
                continue;

            var state = new VRControllerState_t();
            if (OpenVR.System.GetControllerState(device_idx, ref state, (uint)Marshal.SizeOf(typeof(VRControllerState_t))))
            {
                RhinoVrDeviceController controller = m_device_data[(int)device_idx].m_controller;
                GetRhinoVrControllerState(state, controller);
            }
        }

        bool scale_changed = false;

        bool key_pressed_ctrl = Eto.Forms.Keyboard.Modifiers.HasFlag(Eto.Forms.Keys.Control);
        bool key_pressed_1 = Eto.Forms.Keyboard.Modifiers.HasFlag(Eto.Forms.Keys.D1);
        bool key_pressed_2 = Eto.Forms.Keyboard.Modifiers.HasFlag(Eto.Forms.Keys.D2);

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

        m_device_index_left_hand = OpenVR.System.GetTrackedDeviceIndexForControllerRole(ETrackedControllerRole.LeftHand);
        m_device_index_right_hand = OpenVR.System.GetTrackedDeviceIndexForControllerRole(ETrackedControllerRole.RightHand);

        var vr_event = new VREvent_t();
        while (OpenVR.System.PollNextEvent(ref vr_event, (uint)Marshal.SizeOf(typeof(VREvent_t))))
        {
            ProcessVrEvent(vr_event);
        }
        
        // If we lose the window, then reset variables.
        if (m_gh_window.m_hwnd != IntPtr.Zero)
        {
            Eto.Forms.Window eto_window = PlatformServiceProvider.Service.GetEtoWindow(m_gh_window.m_hwnd);

            if(eto_window == null || !eto_window.Visible)
            {
                m_gh_window.m_hwnd = IntPtr.Zero;
                m_tried_launching_gh = false;
            }
        }

        // If we lose the window, then reset variables.
        if (m_rh_window.m_hwnd != IntPtr.Zero)
        {
            Eto.Forms.Window eto_window = PlatformServiceProvider.Service.GetEtoWindow(m_rh_window.m_hwnd);

            if (eto_window == null)
            {
                m_rh_window.m_hwnd = IntPtr.Zero;
            }
        }

        IntPtr isect_gh_window = IntPtr.Zero;
        IntPtr isect_rh_window = IntPtr.Zero;
        PixelCoord isect_gh_window_pt = new PixelCoord();
        PixelCoord isect_rh_window_pt = new PixelCoord();

        if (m_device_index_left_hand < OpenVR.k_unMaxTrackedDeviceCount &&
            m_device_index_right_hand < OpenVR.k_unMaxTrackedDeviceCount)
        {
            Transform left_hand_xform = m_device_data[(int)m_device_index_left_hand].m_xform;
            Transform right_hand_xform = m_device_data[(int)m_device_index_right_hand].m_xform;

            // TODO: Implement once we can simulate mouse events in Eto.

            //if (m_rh_window.m_enabled)
            //{
            //    if (RhinoVrGetIntersectingAppWindow(m_rh_window, left_hand_xform, right_hand_xform, out Point3d isect_point, out Point2d window_uv))
            //    {
            //        isect_rh_window = m_rh_window.m_hwnd;
            //        isect_rh_window_pt = ScreenUvToPt(window_uv, m_rh_window.m_width, m_rh_window.m_height);
            //    }
            //}

            //if (m_gh_window.m_enabled)
            //{
            //    if (RhinoVrGetIntersectingAppWindow(m_gh_window, right_hand_xform, left_hand_xform, out Point3d isect_point, out Point2d window_uv))
            //    {
            //        isect_gh_window = m_gh_window.m_hwnd;
            //        isect_gh_window_pt = ScreenUvToPt(window_uv, m_gh_window.m_width, m_gh_window.m_height);
            //    }
            //}
        }

        IntPtr isect_window = IntPtr.Zero;
        PixelCoord isect_window_pt = new PixelCoord();

        if (isect_gh_window != IntPtr.Zero)
        {
            isect_window = isect_gh_window;
            isect_window_pt = isect_gh_window_pt;
        }
        else if (isect_rh_window != IntPtr.Zero)
        {
            isect_window = isect_rh_window;
            isect_window_pt = isect_rh_window_pt;
        }

        if (isect_window != IntPtr.Zero)
        {
            m_window_intersected_this_frame = true;
        }

        for (uint device_idx = 0; device_idx < OpenVR.k_unMaxTrackedDeviceCount; device_idx++)
        {
            if (OpenVR.System.GetTrackedDeviceClass(device_idx) != ETrackedDeviceClass.Controller)
                continue;

            RhinoVrDeviceData device_data = m_device_data[(int)device_idx];
            RhinoVrDeviceController controller = device_data.m_controller;

            bool is_left_hand = (device_idx == m_device_index_left_hand);
            bool is_right_hand = (device_idx == m_device_index_right_hand);

            if (controller.m_dpad_center_pressed)
            {
                // TODO: Implement once we can simulate mouse events in Eto.

                //if (is_right_hand && Rhino.Input.RhinoGet.InGetPoint(m_doc))
                //{
                //    RhinoVrGetPoint(device_data.m_xform);
                //}
                //else if (isect_window != IntPtr.Zero)
                //{
                //    RhinoVrWindowMouseLeftBtnDown(isect_window, isect_window_pt);
                //    if (isect_gh_window != IntPtr.Zero)
                //    {
                //        m_gh_window_left_btn_down = true;
                //        m_last_window_click_pos = isect_window_pt;
                //    }
                //}
                //else if (is_right_hand)
                //{
                //    if (RhinoVrGetIntersectingObject(device_data.m_xform, out RhinoObject isect_object, out Point3d isect_point))
                //    {
                //        if (Rhino.Input.RhinoGet.InGetObject(m_doc))
                //        {
                //            Rhino.Input.Custom.GetObject go = Rhino.Input.Custom.GetObject.ActiveGetObject(m_doc);
                //            go.AppendToPickList(new ObjRef(isect_object));

                //            // TODO: Find out if this is needed.
                //            //go->PostObjectSelectionChangedEvent(m_view);
                //        }
                //        else
                //        {
                //            isect_object.Select(true);
                //        }
                //    }
                //}
            }
            else if (controller.m_dpad_center_released)
            {
                if (isect_window != IntPtr.Zero)
                {
                    // TODO: Implement once we can simulate mouse events in Eto.

                    //RhinoVrWindowMouseLeftBtnUp(isect_window, isect_window_pt);
                    //if (isect_gh_window != IntPtr.Zero)
                    //{
                    //    m_gh_window_left_btn_down = false;
                    //}
                }
            }
            else if (controller.m_dpad_center_down)
            {
                if (isect_window != IntPtr.Zero)
                {
                    // TODO: Implement once we can simulate mouse events in Eto.

                    //RhinoVrWindowMouseMove(isect_window, isect_window_pt);
                }
            }
            else if (is_left_hand && controller.m_appmenu_button_pressed)
            {
                RhinoApp.ExecuteCommand(m_doc, "_Undo");

                // Need to re-attach since ExecuteCommand pumps messages
                // and can basically do anything, including e.g. deleting views.
                if (!AttachDocAndView())
                    return false;
            }
            else if (is_right_hand && controller.m_appmenu_button_pressed)
            {
                RhinoApp.ExecuteCommand(m_doc, "_Move");

                // Need to re-attach since ExecuteCommand pumps messages
                // and can basically do anything, including e.g. deleting views.
                if (!AttachDocAndView())
                    return false;
            }
            else if (is_right_hand && controller.m_grip_button_pressed)
            {
                RhinoApp.ExecuteCommand(m_doc, "_Cancel");

                // Need to re-attach since ExecuteCommand pumps messages
                // and can basically do anything, including e.g. deleting views.
                if (!AttachDocAndView())
                    return false;
            }
            else if (is_right_hand &&
              (controller.m_trigger_button_pressed || controller.m_a_button_pressed))
            {
                RhinoApp.ExecuteCommand(m_doc, "_Enter");

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
                if (is_right_hand && Rhino.Input.RhinoGet.InGetPoint(m_doc))
                {
                    // TODO: Implement once we can simulate mouse events in Eto.

                    //RhinoVrOnMouseMove(device_data.m_xform);
                }
                else if (isect_window != IntPtr.Zero)
                {
                    // TODO: Implement once we can simulate mouse events in Eto.

                    //RhinoVrWindowMouseMove(isect_window, isect_window_pt);
                }
            }

            // TODO: Implement once we can simulate mouse events in Eto.

            //if (is_left_hand)
            //{
            //    if (controller.m_trigger_button_value > 0.001)
            //    {
            //        if (m_gh_window.m_hwnd == IntPtr.Zero && controller.m_trigger_button_value > 0.9)
            //        {
            //            InitializeAppWindow(m_gh_window, "Grasshopper");

            //            if (m_gh_window.m_hwnd == IntPtr.Zero && !m_tried_launching_gh)
            //            {
            //                RhinoApp.ExecuteCommand(m_doc, "Grasshopper");
            //                m_tried_launching_gh = true;

            //                // Need to re-attach since ExecuteCommand pumps messages
            //                // and can basically do anything, including e.g. deleting views.
            //                if (!AttachDocAndView())
            //                    return false;
            //            }
            //        }
            //        else if (m_gh_window.m_hwnd == IntPtr.Zero)
            //        {
            //            m_gh_window.m_enabled = false;
            //            m_gh_window.m_opacity = 1.0;
            //        }

            //        m_gh_window.m_enabled = true;
            //        m_gh_window.m_opacity = controller.m_trigger_button_value;
            //    }
            //    else
            //    {
            //        m_gh_window.m_enabled = false;
            //        m_gh_window.m_opacity = 1.0;
            //    }

            //    if (controller.m_dpad_up_down || controller.m_dpad_down_down)
            //    {
            //        if (isect_gh_window != IntPtr.Zero)
            //        {
            //            if (!m_gh_window_left_btn_down)
            //            {
            //                const double zoom_threshold = 0.4;

            //                double vertical_offset = controller.m_touchpad_touch_point.Y;

            //                if (Math.Abs(vertical_offset) > zoom_threshold)
            //                {
            //                    double sign = vertical_offset >= 0.0 ? 1.0 : -1.0;
            //                    double zoom_magnitude = sign * (Math.Abs(vertical_offset) - zoom_threshold) / (1.0 - zoom_threshold);

            //                    RhinoVrWindowMouseScroll(isect_gh_window, isect_gh_window_pt, 0.0, zoom_magnitude);
            //                }
            //            }
            //        }
            //    }
            //}
            //else if (is_right_hand)
            //{
            //    if (isect_gh_window != IntPtr.Zero)
            //    {
            //        if (!m_gh_window_left_btn_down)
            //        {
            //            if (controller.m_trigger_button_pressed)
            //            {
            //                RhinoVrWindowMouseRightBtnDown(isect_gh_window, isect_gh_window_pt);
            //            }
            //            else if (controller.m_trigger_button_released)
            //            {
            //                RhinoVrWindowMouseRightBtnUp(isect_gh_window, isect_gh_window_pt);
            //            }
            //        }
            //    }
            //}
        }

        return true;
    }

    public static string ToHexString(byte[] bytes)
    {
        StringBuilder result = new StringBuilder(bytes.Length * 2);

        for (int i = 0; i < bytes.Length; i++)
            result.Append(bytes[i].ToString("x2"));

        return result.ToString();
    }

    protected static IntPtr FindApplicationWindow(string app_title_string)
    {
        //int window_text_length = 256;
        //string window_text[window_text_length];

        //auto hwnd_parent = FindWindowEx(nullptr, nullptr, nullptr, nullptr);

        //while (hwnd_parent)
        //{
        //    if (IsWindowVisible(hwnd_parent) && GetWindowText(hwnd_parent, window_text, window_text_length))
        //    {
        //        ON_wString str(window_text);
        //        if (str.Find(app_title_string) >= 0)
        //        {
        //            return hwnd_parent;
        //        }
        //    }

        //    hwnd_parent = FindWindowEx(nullptr, hwnd_parent, nullptr, nullptr);
        //}

        return IntPtr.Zero;
    }

    protected Mesh CreateAppWindowMesh(double extent_x, double extent_y, double extent_z)
    {
        Mesh mesh = new Mesh();

        mesh.Vertices.Add(new Point3f(-0.5f, -0.5f, -0.05f));
        mesh.Vertices.Add(new Point3f(0.5f, -0.5f, -0.05f));
        mesh.Vertices.Add(new Point3f(0.5f, 0.5f, -0.05f));
        mesh.Vertices.Add(new Point3f(-0.5f, 0.5f, -0.05f));

        mesh.TextureCoordinates.Add(new Point2f(0.0f, 1.0f));
        mesh.TextureCoordinates.Add(new Point2f(1.0f, 1.0f));
        mesh.TextureCoordinates.Add(new Point2f(1.0f, 0.0f));
        mesh.TextureCoordinates.Add(new Point2f(0.0f, 0.0f));

        var face = new MeshFace
        {
            A = 0,
            B = 1,
            C = 2,
            D = 3
        };

        mesh.Faces.AddFace(face);

        Transform scale = Transform.Scale(new Plane(Point3d.Origin, new Vector3d(1, 0, 0), new Vector3d(0, 1, 0)), extent_x, extent_y, extent_z);
        Transform rotation = Transform.Rotation(RhinoMath.ToRadians(90.0), Vector3d.XAxis, Point3d.Origin);

        mesh.Transform(rotation * scale);

        return mesh;
    }

    protected void InitializeAppWindow(RhinoVrAppWindow app, string app_title)
    {
        app.m_enabled = false;
        app.m_title = app_title;

        HashAlgorithm algorithm = MD5.Create();
        string crc_md5_string = ToHexString(algorithm.ComputeHash(Encoding.UTF8.GetBytes(app_title)));

        app.m_md5_str = crc_md5_string;
        app.m_hwnd = FindApplicationWindow(app_title);
        app.m_mesh = CreateAppWindowMesh(1.0, 1.0, 1.0);
    }

public bool RhinoVrGetIntersectingAppWindow(RhinoVrAppWindow app_window, Transform ray_xform, Transform window_mesh_xform, out Point3d world_point, out Point2d screen_uvs)
    {
        world_point = new Point3d();
        screen_uvs = new Point2d();

        if (!window_mesh_xform.TryGetInverse(out Transform window_mesh_xform_inverse))
        {
            return false;
        }

        Line pointer_line = m_pointer_line;
        pointer_line.Transform(window_mesh_xform_inverse * ray_xform);

        Mesh mesh = app_window.m_mesh;

        Point3d[] isects = Rhino.Geometry.Intersect.Intersection.MeshLine(mesh, pointer_line, out int[] face_indices);

        if (isects.Length > 0)
        {
            world_point = isects[0];

            MeshPoint mp = mesh.ClosestMeshPoint(world_point, double.MaxValue);

            var tc0 = mesh.TextureCoordinates[0] * (float)mp.T[0];
            var tc1 = mesh.TextureCoordinates[1] * (float)mp.T[1];
            var tc2 = mesh.TextureCoordinates[2] * (float)mp.T[2];
            var tc3 = mesh.TextureCoordinates[3] * (float)mp.T[3];

            var tc_x = tc0.X + tc1.X + tc2.X + tc3.X;
            var tc_y = tc0.Y + tc1.Y + tc2.Y + tc3.Y;
            
            screen_uvs = new Point2d(tc_x, tc_y);
        
            return true;
        }

        return false;
    }

    PixelCoord ScreenUvToPt(Point2d screen_uv, uint width, uint height)
    {
        var client_pt = new PixelCoord
        {
            X = (uint)Math.Floor(screen_uv.X * width),
            Y = (uint)Math.Floor((1.0 - screen_uv.Y) * height)
        };

        return client_pt;
    }

    bool RhinoVrWindowMouseButtonEvent(IntPtr hwnd, PixelCoord client_pt, int button_event)
    {
        bool rc = false;

        Eto.Forms.Window eto_window = PlatformServiceProvider.Service.GetEtoWindow(m_gh_window.m_hwnd);
        if(eto_window != null)
        {
            var screen_pt = eto_window.PointToScreen(new Eto.Drawing.PointF(client_pt.X, client_pt.Y));
            if (screen_pt != Eto.Drawing.PointF.Empty)
            {
                // TODO: Implement once we can simulate mouse events in Eto.

                //if (SetCursorPos((int)screen_pt.x, (int)screen_pt.y))
                //{
                //    INPUT input = { };
                //    input.type = INPUT_MOUSE;
                //    input.mi.dwFlags = button_event;
                //    if (SendInput(1, &input, sizeof(INPUT)) > 0)
                //    {
                //        rc = true;
                //    }
                //}
            }
        }



        return rc;
    }

    bool RhinoVrWindowMouseLeftBtnDown(IntPtr hwnd, PixelCoord client_pt)
    {
        return RhinoVrWindowMouseButtonEvent(hwnd, client_pt, 0/*MOUSEEVENTF_LEFTDOWN*/);
    }

    bool RhinoVrWindowMouseLeftBtnUp(IntPtr hwnd, PixelCoord client_pt)
    {
        return RhinoVrWindowMouseButtonEvent(hwnd, client_pt, 0/*MOUSEEVENTF_LEFTUP*/);
    }

    bool RhinoVrWindowMouseRightBtnDown(IntPtr hwnd, PixelCoord client_pt)
    {
        return RhinoVrWindowMouseButtonEvent(hwnd, client_pt, 0/*MOUSEEVENTF_RIGHTDOWN*/);
    }

    bool RhinoVrWindowMouseRightBtnUp(IntPtr hwnd, PixelCoord client_pt)
    {
        return RhinoVrWindowMouseButtonEvent(hwnd, client_pt, 0/*MOUSEEVENTF_RIGHTUP*/);
    }

    bool RhinoVrWindowMouseLeftClick(IntPtr hwnd, PixelCoord client_pt)
    {
        bool rc = false;

        if (RhinoVrWindowMouseLeftBtnDown(hwnd, client_pt) && RhinoVrWindowMouseLeftBtnUp(hwnd, client_pt))
        {
            rc = true;
        }

        return rc;
    }

    bool RhinoVrWindowMouseMove(IntPtr hwnd, PixelCoord client_pt)
    {
        int screen_width = (int)Eto.Forms.Screen.DisplayBounds.Width;
        int screen_height = (int)Eto.Forms.Screen.DisplayBounds.Height;

        bool rc = false;

        Eto.Forms.Window eto_window = PlatformServiceProvider.Service.GetEtoWindow(m_gh_window.m_hwnd);
        if (eto_window != null)
        {
            var screen_pt = eto_window.PointToScreen(new Eto.Drawing.PointF(client_pt.X, client_pt.Y));

            // TODO: Implement once we can simulate mouse events in Eto.

            //INPUT input = { };
            //input.type = INPUT_MOUSE;
            //input.mi.dx = (LONG)((double(screen_pt.x) / screen_width) * 0xFFFF);
            //input.mi.dy = (LONG)((double(screen_pt.y) / screen_height) * 0xFFFF);
            //input.mi.dwFlags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE;
            //if (SendInput(1, &input, sizeof(INPUT)) > 0)
            //{
            //    rc = true;
            //}
        }

        return rc;
    }

    bool RhinoVrWindowMouseScroll(IntPtr hwnd, PixelCoord client_pt, double x, double y)
    {
        bool rc = false;

        Eto.Forms.Window eto_window = PlatformServiceProvider.Service.GetEtoWindow(m_gh_window.m_hwnd);
        if (eto_window != null)
        {
            var screen_pt = eto_window.PointToScreen(new Eto.Drawing.PointF(client_pt.X, client_pt.Y));

            // TODO: Implement once we can simulate mouse events in Eto.

            //if (SetCursorPos((int)screen_pt.x, (int)screen_pt.y))
            //{
            //    if (x != 0.0)
            //    {
            //        INPUT input = { };
            //        input.type = INPUT_MOUSE;
            //        input.mi.dwFlags = MOUSEEVENTF_HWHEEL;
            //        input.mi.mouseData = (DWORD)(WHEEL_DELTA * x);
            //        if (SendInput(1, &input, sizeof(INPUT)) > 0)
            //        {
            //            rc = true;
            //        }
            //    }

            //    if (y != 0.0)
            //    {
            //        INPUT input = { };
            //        input.type = INPUT_MOUSE;
            //        input.mi.dwFlags = MOUSEEVENTF_WHEEL;
            //        input.mi.mouseData = (DWORD)(WHEEL_DELTA * y);
            //        if (SendInput(1, &input, sizeof(INPUT)) > 0)
            //        {
            //            rc = true;
            //        }
            //    }
            //}
        }

        return rc;
    }

    // Draws the left and right eye views and sends the result to the HMD.
    protected bool Draw()
    {
        // TODO: Wrap DeferredDisplayMode()

        //var current_display_mode = m_view.DisplayPipeline()->DeferredDisplayMode();
        //if (current_display_mode != m_previous_display_mode)
        //{
        //    if (!CreateVrDisplayPipeline(*m_view, *m_vr_vp, &m_vr_dp, &m_vr_dp_ogl))
        //    {
        //        RhinoApp().Print("Unable to re-create VR display pipeline. Viewport display mode may look wrong.");
        //    }

        //    m_previous_display_mode = current_display_mode;
        //}

        bool draw_success = m_vr_dp.DrawStereoFrameBuffer(m_vp_left_eye, m_vp_right_eye, out uint eye_left_handle, out uint eye_right_handle);
        
        if (!draw_success)
            return false;

        // The pipeline needs to be opened for compositor to work.
        m_vr_dp.Open();

        var ovr_error = EVRCompositorError.None;

        var leftEyeTexture = new Texture_t()
        {
            handle = new IntPtr(eye_left_handle),
            eType = ETextureType.OpenGL,
            eColorSpace = EColorSpace.Gamma
        };

        var leftEyeTextureBounds = new VRTextureBounds_t()
        {
            uMin = 0.0f,
            uMax = 1.0f,
            vMin = 0.0f,
            vMax = 1.0f,
        };

        ovr_error = OpenVR.Compositor.Submit(EVREye.Eye_Left, ref leftEyeTexture, ref leftEyeTextureBounds, EVRSubmitFlags.Submit_Default);
        if (ovr_error != EVRCompositorError.None && ovr_error != EVRCompositorError.DoNotHaveFocus)
        {
            m_vr_dp.Close();

            return false;
        }

        var rightEyeTexture = new Texture_t()
        {
            handle = new IntPtr(eye_right_handle),
            eType = ETextureType.OpenGL,
            eColorSpace = EColorSpace.Gamma
        };

        var rightEyeTextureBounds = new VRTextureBounds_t()
        {
            uMin = 0.0f,
            uMax = 1.0f,
            vMin = 0.0f,
            vMax = 1.0f,
        };

        ovr_error = OpenVR.Compositor.Submit(EVREye.Eye_Right, ref rightEyeTexture, ref rightEyeTextureBounds, EVRSubmitFlags.Submit_Default);
        if (ovr_error != EVRCompositorError.None && ovr_error != EVRCompositorError.DoNotHaveFocus)
        {
            m_vr_dp.Close();

            return false;
        }

#if RHINOVR_FRAME_TIMING || RHINOVR_DETAILED_TIMING
      //::glFlush();
      //::glFinish();
#endif

        m_vr_dp.Close();

        return true;
    }

    Transform OpenVrMatrixToXform(HmdMatrix34_t matrix)
    {
        var xform = new Transform
        {
            M00 = matrix.m0,
            M01 = matrix.m1,
            M02 = matrix.m2,
            M03 = matrix.m3 * m_unit_scale,

            M10 = matrix.m4,
            M11 = matrix.m5,
            M12 = matrix.m6,
            M13 = matrix.m7 * m_unit_scale,

            M20 = matrix.m8,
            M21 = matrix.m9,
            M22 = matrix.m10,
            M23 = matrix.m11 * m_unit_scale,

            M30 = 0.0,
            M31 = 0.0,
            M32 = 0.0,
            M33 = 1.0
        };

        return xform;
    }

    // Fills in the high-level controller state from the data provided by the VR library.
    protected void GetRhinoVrControllerState(VRControllerState_t state, RhinoVrDeviceController controller)
    {
        ulong touchpad_btn_mask = (1ul << (int)EVRButtonId.k_EButton_SteamVR_Touchpad);

        bool touchpad_is_down = ((state.ulButtonPressed & touchpad_btn_mask) > 0);
        bool touchpad_is_touched = ((state.ulButtonTouched & touchpad_btn_mask) > 0);

        {
            bool was_down = controller.m_touchpad_button_down;
            bool is_down = touchpad_is_down;
            bool is_touched = touchpad_is_touched;

            controller.m_touchpad_button_down = is_down;
            controller.m_touchpad_button_pressed = is_down && !was_down;
            controller.m_touchpad_button_released = !is_down && was_down;
            controller.m_touchpad_button_touched = is_touched;
            controller.m_touchpad_touch_point = new Point2d(state.rAxis0.x, state.rAxis0.y);
        }

        const double dpad_threshold = 0.5;
        Vector2d touch_vec = new Vector2d(state.rAxis0.x, state.rAxis0.y);

        // Make sure it's either clearly up/down or left/right.
        if (Math.Abs(touch_vec.X) >= Math.Abs(touch_vec.Y))
        {
            touch_vec.Y = 0.0;
        }
        else
        {
            touch_vec.X = 0.0;
        }

        {
            bool was_down = controller.m_dpad_left_down;
            bool is_down = touchpad_is_down && touch_vec.X <= -dpad_threshold;
            if (m_vr_system_type == VrSystemType.Rift)
                is_down = touch_vec.X <= -dpad_threshold;

            controller.m_dpad_left_down = is_down;
            controller.m_dpad_left_pressed = is_down && !was_down;
            controller.m_dpad_left_released = !is_down && was_down;
        }

        {
            bool was_down = controller.m_dpad_right_down;
            bool is_down = touchpad_is_down && touch_vec.X >= dpad_threshold;
            if (m_vr_system_type == VrSystemType.Rift)
                is_down = touch_vec.X >= dpad_threshold;

            controller.m_dpad_right_down = is_down;
            controller.m_dpad_right_pressed = is_down && !was_down;
            controller.m_dpad_right_released = !is_down && was_down;
        }

        {
            bool was_down = controller.m_dpad_up_down;
            bool is_down = touchpad_is_down && touch_vec.Y >= dpad_threshold;
            if (m_vr_system_type == VrSystemType.Rift)
                is_down = touch_vec.Y >= dpad_threshold;

            controller.m_dpad_up_down = is_down;
            controller.m_dpad_up_pressed = is_down && !was_down;
            controller.m_dpad_up_released = !is_down && was_down;
        }

        {
            bool was_down = controller.m_dpad_down_down;
            bool is_down = touchpad_is_down && touch_vec.Y <= -dpad_threshold;
            if (m_vr_system_type == VrSystemType.Rift)
                is_down = touch_vec.Y <= -dpad_threshold;

            controller.m_dpad_down_down = is_down;
            controller.m_dpad_down_pressed = is_down && !was_down;
            controller.m_dpad_down_released = !is_down && was_down;
        }

        bool dpad_direction_down =
            controller.m_dpad_up_down || controller.m_dpad_down_down ||
            controller.m_dpad_left_down || controller.m_dpad_right_down;

        {
            bool was_down = controller.m_dpad_center_down;
            bool is_down = touchpad_is_down && !dpad_direction_down;
            if (m_vr_system_type == VrSystemType.Rift)
                is_down = touchpad_is_down && !dpad_direction_down;

            controller.m_dpad_center_down = is_down;
            controller.m_dpad_center_pressed = is_down && !was_down;
            controller.m_dpad_center_released = !is_down && was_down;
        }

        {
            ulong appmenu_btn_mask = (1ul << (int)EVRButtonId.k_EButton_ApplicationMenu);

            bool was_down = controller.m_appmenu_button_down;
            bool is_down = ((state.ulButtonPressed & appmenu_btn_mask) > 0);

            controller.m_appmenu_button_down = is_down;
            controller.m_appmenu_button_pressed = is_down && !was_down;
            controller.m_appmenu_button_released = !is_down && was_down;
        }

        {
            ulong grip_btn_mask = (1ul << (int)EVRButtonId.k_EButton_Grip);

            bool was_down = controller.m_grip_button_down;
            bool is_down = ((state.ulButtonPressed & grip_btn_mask) > 0);

            controller.m_grip_button_down = is_down;
            controller.m_grip_button_pressed = is_down && !was_down;
            controller.m_grip_button_released = !is_down && was_down;
        }

        {
            float value = state.rAxis1.x;
            bool was_down = controller.m_trigger_button_down;
            bool is_down = (value == 1.0f);

            controller.m_trigger_button_down = is_down;
            controller.m_trigger_button_pressed = is_down && !was_down;
            controller.m_trigger_button_released = !is_down && was_down;
            controller.m_trigger_button_value = value;
        }

        {
            ulong a_btn_mask = (1ul << (int)EVRButtonId.k_EButton_A);

            bool was_down = controller.m_a_button_down;
            bool is_down = ((state.ulButtonPressed & a_btn_mask) > 0);

            controller.m_a_button_down = is_down;
            controller.m_a_button_pressed = is_down && !was_down;
            controller.m_a_button_released = !is_down && was_down;
        }
    }

    // Processes VR events.
    protected void ProcessVrEvent(VREvent_t vr_event)
    {
        var event_type = (EVREventType)vr_event.eventType;

        switch (event_type)
        {
            case EVREventType.VREvent_TrackedDeviceActivated:
                {
                    SetupRenderModelForDevice(vr_event.trackedDeviceIndex);
                }
                break;
            case EVREventType.VREvent_TrackedDeviceDeactivated:
                {
                    // React to device being deactivated
                }
                break;
            case EVREventType.VREvent_TrackedDeviceUpdated:
                {
                    // React to device being updated
                }
                break;
        }
    }

    // Simulate Rhino's GetPoint in VR.
    protected void RhinoVrGetPoint(Transform picking_device_xform)
    {
        //CRhinoGetPoint* gp = Rhino.Input.Custom.GetPoint.
        //if (gp == nullptr)
        //    return;

        //ON_Line world_line;
        //ON_ClippingRegion clip_region;
        //ON_Viewport line_vp;
        //ON_2iPoint line_pixel;

        //if (GetWorldPickLineAndClipRegion(picking_device_xform, world_line, clip_region, line_vp, line_pixel))
        //{
        //    LPARAM nFlags = 0;
        //    ON_3dPoint screen_pt = ON_3dPoint(line_pixel.x, line_pixel.y, 0.0);

        //    CRhinoViewport & rhino_vp = m_view->ActiveViewport();

        //    const ON_Viewport orig_vp = rhino_vp.VP();
        //    const int orig_width = orig_vp.ScreenPortWidth();
        //    const int orig_height = orig_vp.ScreenPortHeight();

        //    rhino_vp.SetVP(line_vp, TRUE);
        //    rhino_vp.SetScreenSize(line_vp.ScreenPortWidth(), line_vp.ScreenPortHeight());

        //    ON_3dPoint world_point;
        //    if (gp->GetView3dPoint(1, *m_view, nFlags, screen_pt, world_line, world_point))
        //    {
        //        ON_3dRay ray;
        //        ray.m_P = world_point;
        //        ray.m_V = ON_3dVector::ZeroVector;

        //        LPARAM nFlags = 0;

        //        m_view->PostDigitizerPointEvent(ray, nFlags);
        //    }

        //    rhino_vp.SetVP(orig_vp, TRUE);
        //    rhino_vp.SetScreenSize(orig_width, orig_height);
        //}
    }

    // Simulate Rhino's OnMouseMove in VR.
    protected void RhinoVrOnMouseMove(Transform picking_device_xform)
    {
        //CRhinoGetPoint* gp = m_doc->InGetPoint();
        //if (gp == nullptr)
        //    return;

        //ON_Line world_line;
        //ON_ClippingRegion clip_region;
        //ON_Viewport line_vp;
        //ON_2iPoint line_pixel;

        //if (GetWorldPickLineAndClipRegion(picking_device_xform, world_line, clip_region, line_vp, line_pixel))
        //{
        //    LPARAM nFlags = 0;
        //    ON_3dPoint screen_pt = ON_3dPoint(line_pixel.x, line_pixel.y, 0.0);

        //    CRhinoViewport & rhino_vp = m_view->ActiveViewport();

        //    const ON_Viewport orig_vp = rhino_vp.VP();
        //    const int orig_width = orig_vp.ScreenPortWidth();
        //    const int orig_height = orig_vp.ScreenPortHeight();

        //    rhino_vp.SetVP(line_vp, TRUE);
        //    rhino_vp.SetScreenSize(line_vp.ScreenPortWidth(), line_vp.ScreenPortHeight());

        //    ON_3dPoint world_point;
        //    if (gp->GetView3dPoint(1, *m_view, (UINT_PTR)nFlags, screen_pt, world_line, world_point))
        //    {
        //        gp->OnMouseMove(*m_vr_vp, (UINT)nFlags, world_point, (const ON_2iPoint*) nullptr);
        //    }

        //    rhino_vp.SetVP(orig_vp, TRUE);
        //    rhino_vp.SetScreenSize(orig_width, orig_height);
        //}
    }

    // Find object intersection with an eye-space ray transformed by 'picking_device_xform'.
    // The eye-space ray is (0.0, 0.0, -frustum_near) to (0.0, 0.0, -frustum_far).
    protected bool RhinoVrGetIntersectingObject(
        Transform picking_device_xform, out RhinoObject isect_object, out Point3d isect_point)
    {
        isect_object = null;
        isect_point = Point3d.Unset;

        var pc = new Rhino.Input.Custom.PickContext
        {
            View = m_view,
            PickMode = Rhino.Input.Custom.PickMode.Shaded,
            PickStyle = Rhino.Input.Custom.PickStyle.PointPick
        };
        
        if (GetWorldPickLineAndClipRegion(picking_device_xform, out Line pick_line, out Transform pick_xform, out ViewportInfo line_vp, out int line_pixel_x, out int line_pixel_y))
        {
            pc.PickLine = pick_line;
            pc.SetPickTransform(pick_xform);
            pc.UpdateClippingPlanes();

            Plane np = line_vp.FrustumNearPlane;

            ObjRef[] rhino_objs = m_doc.Objects.PickObjects(pc);
            
            Array.Sort(rhino_objs,
                delegate (ObjRef obj0, ObjRef obj1)
            {
                Point3d obj0_pt = obj0.SelectionPoint();
                Point3d obj1_pt = obj1.SelectionPoint();

                double pick_dist0 = (pick_line.ClosestPoint(obj0_pt, false) - obj0_pt).SquareLength;
                double pick_dist1 = (pick_line.ClosestPoint(obj1_pt, false) - obj1_pt).SquareLength;

                double pick_dist_diff = pick_dist0 - pick_dist1;
                if (pick_dist_diff < 0.0)
                    return -1;
                else if (pick_dist_diff > 0.0)
                    return +1;

                double pick_depth0 = (np.ClosestPoint(obj0_pt) - obj0_pt).SquareLength;
                double pick_depth1 = (np.ClosestPoint(obj1_pt) - obj1_pt).SquareLength;

                double pick_depth_diff = pick_depth0 - pick_depth1;
                if (pick_depth_diff < 0.0)
                    return -1;
                else if (pick_depth_diff > 0.0)
                    return +1;

                return 0;
            });
            
            if (rhino_objs.Length > 0)
            {
                ObjRef objref = rhino_objs[0];
                Point3d isect_pt = objref.SelectionPoint();

                if (objref.Object() != null && isect_pt != Point3d.Unset && isect_pt.IsValid)
                {
                    isect_object = objref.Object();
                    isect_point = isect_pt;

                    return true;
                }
            }
        }

        return false;
    }

    protected bool GetWorldPickLineAndClipRegion(
        Transform picking_device_xform,
        out Line world_line,
        out Transform pick_xform,
        out ViewportInfo line_vp,
        out int line_pixel_x,
        out int line_pixel_y)
    {
        line_vp = m_vp_hmd;

        line_vp.SetCameraLocation(Point3d.Origin);
        line_vp.SetCameraDirection(-Vector3d.ZAxis);
        line_vp.SetCameraUp(Vector3d.YAxis);
        line_vp.TransformCamera(m_cam_to_world_xform * picking_device_xform);
        
        Rectangle sp = line_vp.GetScreenPort();

        // Make screen port an uneven amount of pixels.
        // This way the center pixel will be truly at
        // center of the screen, which in turn will 
        // make the pick point as accurate as possible.
        if (sp.Right % 2 == 0) sp.Width += 1;
        if (sp.Bottom % 2 == 0) sp.Height += 1;

        line_vp.SetScreenPort(sp);

        // We need to force the frustum to be symmetric, otherwise the 
        // center pixel will not be exactly where the pick line is.
        line_vp.GetFrustum(out double lf, out double rf, out double bf, out double tf, out double nf, out double ff);

        double new_tbf = Math.Max(Math.Abs(bf), Math.Abs(tf));
        bf = -new_tbf;
        tf = new_tbf;

        line_vp.SetFrustum(lf, rf, bf, tf, nf, ff);

        double frus_near = m_unit_scale * 0.01;
        line_vp.SetFrustumNearFar(frus_near, frus_near / line_vp.PerspectiveMinNearOverFar);

        PixelCoord line_pixel = new PixelCoord((uint)line_vp.ScreenPort.Width / 2 + 1, (uint)line_vp.ScreenPort.Height / 2 + 1);
        line_pixel_x = (int)line_pixel.X;
        line_pixel_y = (int)line_pixel.Y;

        m_vr_vp.SetViewProjection(line_vp, true);
        pick_xform = m_vr_vp.GetPickTransform(line_pixel_x, line_pixel_y);
        m_vr_vp.SetViewProjection(m_vp_hmd, true);

        world_line = line_vp.GetFrustumLine(line_pixel_x, line_pixel_y);

        return true;
    }

    // Measures time spent processing input and drawing a stereo frame.
    protected void FrameTimingStart()
    {
        TimingStart(m_frame_time_stopwatch);
    }

    protected void FrameTimingStop()
    {
        TimingStop(m_frame_time_stopwatch, "Frame time");
    }

    // Measures time spent outside RhinoVR.
    protected void RhinoTimingStart()
    {
        TimingStart(m_rhino_time_stopwatch);
    }

    protected void RhinoTimingStop()
    {
        TimingStop(m_rhino_time_stopwatch, "Rhino time");
    }

    // Measures time spent waiting for vertical sync.
    protected void VsyncTimingStart()
    {
        TimingStart(m_vsync_time_stopwatch);
    }

    protected void VsyncTimingStop()
    {
        TimingStop(m_vsync_time_stopwatch, "Vsync time");
    }

    // Measures the total frame time and frames per second.
    protected void FpsTiming()
    {
#if RHINOVR_FRAME_TIMING
        if (!m_fps_time_stopwatch.IsRunning)
        {
            TimingStart(m_fps_time_stopwatch);
        }
        else
        {
            m_frame_counter++;

            long time_millis = m_fps_time_stopwatch.ElapsedMilliseconds;
            if (time_millis >= 1000)
            {
                double frame_time_seconds = (time_millis/1000.0) / m_frame_counter;
                double frames_per_second = 1.0 / frame_time_seconds;

                Debug.Print("Full frame time: {0:.##} ms. Frames per second: {0:.##}\n", 1000.0 * frame_time_seconds, frames_per_second);

                m_frame_counter = 0;
                m_fps_time_stopwatch.Restart();
            }
        }
#endif
    }

    protected void TimingStart(Stopwatch stopwatch)
    {
#if RHINOVR_FRAME_TIMING || RHINOVR_DETAILED_TIMING
        if(stopwatch != null)
        {
            stopwatch.Restart();
        }
#endif
    }

    void TimingStop(Stopwatch stopwatch, string message)
    {
#if RHINOVR_FRAME_TIMING || RHINOVR_DETAILED_TIMING
        if(stopwatch != null && stopwatch.IsRunning)
        {
            stopwatch.Stop();
            Debug.Print("{0}: {1:.##} ms\n", message, stopwatch.ElapsedMilliseconds);
        }
#endif
    }

    protected Stopwatch m_frame_time_stopwatch = new Stopwatch();
    protected Stopwatch m_rhino_time_stopwatch = new Stopwatch();
    protected Stopwatch m_vsync_time_stopwatch = new Stopwatch();
    protected Stopwatch m_fps_time_stopwatch = new Stopwatch();
    protected uint m_frame_counter = 0;

    // The serial number of the Rhino document
    // used when executing RhinoVR.
    protected uint m_doc_sn = 0;

    // The serial number of the Rhino view
    // which was active when executing RhinoVR.
    protected uint m_view_sn = 0;

    protected RhinoDoc m_doc  = null; // The Rhino document acquired from 'm_doc_sn'.
    protected RhinoView m_view = null; // The Rhino view acquired from 'm_view_sn'.

    // Viewport representing the location, orientation and field of view of the HMD.
    protected RhinoViewport m_vr_vp = null;

    // The display pipeline used for rendering in VR.
    protected DisplayPipeline m_vr_dp = null;

    // The VR display pipeline down-cast to the OpenGL pipeline.
    //protected DisplayPipeline_OGL m_vr_dp_ogl = null;

    protected float m_near_clip = 1.0f;   // VR frustum near.
    protected float m_far_clip  = 100.0f; // VR frustum far.

    // Left eye frustum.
    protected float m_left_frus_left   = 0.0f;
    protected float m_left_frus_right  = 0.0f;
    protected float m_left_frus_top    = 0.0f;
    protected float m_left_frus_bottom = 0.0f;

    // Right eye frustum.
    protected float m_right_frus_left   = 0.0f;
    protected float m_right_frus_right  = 0.0f;
    protected float m_right_frus_top    = 0.0f;
    protected float m_right_frus_bottom = 0.0f;

    // Scaling applied to all geometry fetched from the VR library.
    // The unit of VR library geometry is meters.
    // If the Rhino document units is millimeters, we need to scale all
    // VR library geometry by 1000.0, i.e. we need to express their
    // real-world size in millimeters.
    protected double m_unit_scale = 1.0;

    protected double MoveSpeedMax = 10.0;
    protected double MoveAcceleration = 3.0;
    protected double MoveDecelerationSoft = 6.0;
    protected double MoveDecelerationHard = 18.0;

    protected double m_move_speed; // Movement speed in meters per second
    protected double m_turn_speed = 90.0; // Turning speed in degrees per second
    protected double m_last_frame_time;
    protected Stopwatch m_movement_stopwatch = new Stopwatch();

    protected double m_move_speed_when_start_moving;
    protected Stopwatch m_start_moving_stopwatch = new Stopwatch();

    protected double m_move_speed_when_stop_moving;
    protected Stopwatch m_stop_moving_stopwatch = new Stopwatch();

    protected Transform m_hmd_xform       = Transform.Identity; // HMD transform.
    protected Transform m_left_eye_xform  = Transform.Identity; // Left eye transform.
    protected Transform m_right_eye_xform = Transform.Identity; // Right eye transform.

    // HMD location correction.
    // This transform is set once, and will always 
    // be applied to the HMD transform. It makes sure
    // that the initial location of the HMD will always
    // be where the Rhino viewport camera was originally.
    protected Transform m_hmd_location_correction_xform = Transform.Identity;

    // Indicates whether we have set the HMD location correction xform.
    protected bool m_hmd_location_correction_acquired = false;

    // Transform from camera-space to world-space, including movement by controller.
    protected Transform m_cam_to_world_xform = Transform.Unset;

    // The original viewport from the Rhino view.
    protected ViewportInfo m_vp_orig = null;

    // The original viewport from the Rhino view,
    // but modified to have a frustum in accordance
    // to the HMD's field of view.
    protected ViewportInfo m_vp_orig_hmd_frus = null;

    // A viewport which represents the world-space
    // location, orientation and field of view of the HMD.
    protected ViewportInfo m_vp_hmd = null;

    // A viewport which represents the world-space
    // location, orientation and field of view of the HMD's
    // left eye.
    protected ViewportInfo m_vp_left_eye = null;

    // A viewport which represents the world-space
    // location, orientation and field of view of the HMD's
    // right eye.
    protected ViewportInfo m_vp_right_eye = null;

    // A line which represents the object-space pointer line
    // shooting out from the controllers.
    protected Line m_pointer_line = Line.Unset;

    // A mesh representing the pointer line.
    protected Mesh m_pointer_mesh = new Mesh();

    protected DisplayMaterial m_pointer_mesh_material = new DisplayMaterial();

    //CacheHandle m_pointer_mesh_cache_handle;

    protected VrSystemType m_vr_system_type = VrSystemType.Unknown;

    // The transforms of all tracked devices.
    protected TrackedDevicePose_t[] m_device_poses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];
    protected TrackedDevicePose_t[] m_game_poses = new TrackedDevicePose_t[0];

    protected uint m_device_index_left_hand  = 0; // Device index for left hand controller.
    protected uint m_device_index_right_hand = 0; // Device index for right hand controller.

    // The device data of all tracked devices.
    protected List<RhinoVrDeviceData> m_device_data = new List<RhinoVrDeviceData>((int)OpenVR.k_unMaxTrackedDeviceCount);

    // The render models of all tracked devices.
    protected List<RhinoVrDeviceModel> m_device_render_models = new List<RhinoVrDeviceModel>();

    protected RhinoVrAppWindow m_gh_window = new RhinoVrAppWindow();
    protected RhinoVrAppWindow m_rh_window = new RhinoVrAppWindow();

    protected bool m_tried_launching_gh = false;
    protected bool m_gh_window_left_btn_down = false;
    protected bool m_window_intersected_this_frame = false;

    protected Stopwatch m_last_window_update_stopwatch = new Stopwatch();
    protected PixelCoord m_last_window_click_pos = new PixelCoord(0, 0);
    protected int m_last_window_updated = 1;

    protected Mesh m_hidden_mesh_left  = null; // The hidden area mesh for the left eye.
    protected Mesh m_hidden_mesh_right = null; // The hidden area mesh for the right eye.

    // The hidden area mesh display conduit.
    protected RhinoVrHiddenAreaMeshDisplayConduit m_hidden_mesh_display_conduit;

    protected enum VrSystemType
    {
        Unknown,
        Vive,
        Rift
    };

    protected struct PixelCoord
    {
        public PixelCoord(uint x, uint y)
        {
            X = x;
            Y = y;
        }

        public uint X;
        public uint Y;
    }
}
