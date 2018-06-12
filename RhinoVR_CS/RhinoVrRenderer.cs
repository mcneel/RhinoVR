
using Rhino;
using Rhino.Display;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Render;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Text;
using Valve.VR;

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
            
            m_device_mesh.Vertices[vi] = unit_scale * new Point3f(v.v0, v.v1, v.v2);
            m_device_mesh.Normals[vi]  = unit_scale * new Vector3f(n.v0, n.v1, n.v2);
            m_device_mesh.TextureCoordinates[vi] = new Point2f(uv0, uv1) * unit_scale;
        }

        var triangle_count = (int)model.unTriangleCount;

        m_device_mesh.Faces.Capacity = triangle_count;
        m_device_mesh.Faces.Count    = triangle_count;

        var vertex_indices = OpenVrMarshal.FromIntPtrToArray<UInt16>(model.rIndexData, model.unTriangleCount*3);

        for (int fi = 0, idx = 0; fi < triangle_count; ++fi)
        {
            MeshFace triangle = m_device_mesh.Faces[fi];
            triangle.A = vertex_indices[idx++];
            triangle.B = vertex_indices[idx++];
            triangle.C = vertex_indices[idx++];
            triangle.D = triangle.C;
        }
        
        // TODO: I don't know how to do this yet
        SimulatedTexture st = new SimulatedTexture();
        st.Filename = "test";
        var render_texture = RenderTexture.NewBitmapTexture(st, doc);
        // END TODO
        
        var rdk_material = RenderMaterial.CreateBasicMaterial(new Material(), doc);
        rdk_material.Initialize();

        var child_slot_name = rdk_material.TextureChildSlotName(RenderMaterial.StandardChildSlots.Diffuse);

        rdk_material.SetChild(render_texture, child_slot_name);
        rdk_material.SetChildSlotOn(child_slot_name, true, RenderContent.ChangeContexts.Program);
        rdk_material.SetChildSlotAmount(child_slot_name, 100.0, RenderContent.ChangeContexts.Program);

        m_device_material = new DisplayMaterial(rdk_material.SimulateMaterial(false));

        rdk_material.Uninitialize();

        return true;
    }

    public string m_device_name;
    public Mesh m_device_mesh = null;
    public DisplayMaterial m_device_material = null;
    //public CRhinoCacheHandle m_cache_handle = null;
}

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

    public float m_trigger_button_value = 0.0f;
    public Point2d m_touchpad_touch_point = Point2d.Origin;
}

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
public class RhinoVrRenderer
{
    public RhinoVrRenderer(uint doc_sn, uint view_sn)
    {
        m_doc_sn = doc_sn;
        m_view_sn = view_sn;

        for (int i = 0; i < OpenVR.k_unMaxTrackedDeviceCount; ++i)
        {
            m_device_data.Add(new RhinoVrDeviceData());
        }
    }

    ~RhinoVrRenderer()
    {
        // TODO: How do I handle WM_TIMER messages in C#?
        // We need to kill our custom timer here.

        OpenVR.Shutdown();

        // TODO: How do I change ViewportId to nil_uuid?

        RhinoApp.WriteLine("RhinoVR is OFF.");
    }

    public bool Initialize()
    {
        RhinoApp.WriteLine("Initializing RhinoVR...");

        var init_error = EVRInitError.None;

        OpenVR.Init(ref init_error);

        if ( init_error != EVRInitError.None)
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

        // TODO: ON_UnitSystem has not been wrapped.
        m_unit_scale = 1.0;

        var start = new Point3d(0.0, 0.0, -0.02);
        var end   = new Point3d(0.0, 0.0, -500.0);
        m_pointer_line = new Line(m_unit_scale*start, m_unit_scale*end);

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

        DisplayPipeline view_dp = view.DisplayPipeline;
        if (view_dp == null)
            return false;

        //DisplayPipeline_OGL view_dp_ogl = (DisplayPipeline_OGL)view_dp;
        //if (view_dp_ogl == null)
        //    return false;

        m_vr_vp = new RhinoViewport(view.ActiveViewport);
        m_vr_vp.Size = new Size((int)rec_width, (int)rec_height);

        // TOOD: Open/ClosePipeline not wrapped.
        // TODO: ClonePipeline not wrapped.
        //view_dp_ogl.OpenPipeline();
        //m_vr_dp = view_dp.ClonePipeline(m_vr_vp);
        //view_dp_ogl.ClosePipeline();

        if (m_vr_dp == null)
            return false;

        //m_vr_dp_ogl = dynamic_cast<CRhinoDisplayPipeline_OGL*>(m_vr_dp.get());
        //if (m_vr_dp_ogl == nullptr)
        //    return false;

        m_vp_orig = new ViewportInfo(m_vr_dp.Viewport);
        var vp    = new ViewportInfo(m_vp_orig);

        m_near_clip = (float)(m_unit_scale * 0.01);
        m_far_clip = (float)(m_near_clip / 0.0005); // TODO: Need to use PerspectiveMinNearOverFar

        OpenVR.System.GetProjectionRaw(EVREye.Eye_Left,
            ref m_left_frus_left, ref m_left_frus_right,
            ref m_left_frus_top, ref m_left_frus_bottom);

        m_left_frus_left   *= m_near_clip;
        m_left_frus_right  *= m_near_clip;
        m_left_frus_top    *= m_near_clip;
        m_left_frus_bottom *= m_near_clip;

        OpenVR.System.GetProjectionRaw(EVREye.Eye_Right,
            ref m_right_frus_left, ref m_right_frus_right,
            ref m_right_frus_top, ref m_right_frus_bottom);

        m_right_frus_left   *= m_near_clip;
        m_right_frus_right  *= m_near_clip;
        m_right_frus_top    *= m_near_clip;
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

            String script = String.Format("-_ViewportProperties _Size {0} {1} _Enter", vp_new_width, vp_new_height);
            RhinoApp.RunScript(script, false);

            view.ActiveViewport.SetViewProjection(vp, true);
            view.Redraw();
        }

        // TODO: How do I handle WM_TIMER messages in C#?
        // We need to start our custom timer here.

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

    // Converts an OpenVR matrix to an ON_Xform.
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
        for(uint device_idx = OpenVR.k_unTrackedDeviceIndex_Hmd + 1; device_idx < OpenVR.k_unMaxTrackedDeviceCount; ++device_idx )
        {
            if (!OpenVR.System.IsTrackedDeviceConnected(device_idx))
                continue;

            SetupRenderModelForDevice(device_idx);
        }

        m_hidden_mesh_left  = LoadHiddenAreaMesh(EVREye.Eye_Left);
        m_hidden_mesh_right = LoadHiddenAreaMesh(EVREye.Eye_Right);
    }

    private String GetTrackedDeviceString(uint device_idx, ETrackedDeviceProperty device_property, out ETrackedPropertyError error)
    {
        error = ETrackedPropertyError.TrackedProp_Success;

        uint required_buffer_len = OpenVR.System.GetStringTrackedDeviceProperty(device_idx, device_property, null, 0, ref error);
        if (required_buffer_len == 0)
            return String.Empty;

        var string_builder = new StringBuilder();

        OpenVR.System.GetStringTrackedDeviceProperty(device_idx, device_property, string_builder, required_buffer_len, ref error);

        return string_builder.ToString();
    }

    // Loads a render model for a specific device.
    protected void SetupRenderModelForDevice(uint device_index)
    {
        if (device_index >= OpenVR.k_unMaxTrackedDeviceCount)
            return;
        
        String render_model_name = GetTrackedDeviceString(device_index, ETrackedDeviceProperty.Prop_RenderModelName_String, out ETrackedPropertyError error);

        RhinoVrDeviceModel render_model = FindOrLoadRenderModel(render_model_name);
        if (render_model == null)
        {

        }
    }

    // Loads the "hidden area mesh" geometry which is intended to block
    // out the parts of the screens which won't be visible to the eyes.
    // This is done to improve rendering  performance.
    protected Mesh LoadHiddenAreaMesh(EVREye eye)
    {
        return null;
    }

    // Returns a render model by name. If it hasn't been loaded yet, it will
    // first be loaded from the VR library.
    protected RhinoVrDeviceModel FindOrLoadRenderModel(string render_model_name)
    {
        return null;
    }

    // Provide device display conduits with updated device information.
    protected void UpdateDeviceDisplayConduits(
        Transform camera_to_world_xform,
        Transform clip_to_left_eye_xform,
        Transform clip_to_right_eye_xform)
    {

    }

    protected bool AttachDocAndView()
    {
        return true;
    }

    protected void DetachDocAndView()
    {

    }

    // Updates the transforms of all tracked devices.
    // This function will wait for the HMD's vertical sync signal.
    protected bool UpdatePosesAndWaitForVSync()
    {
        return true;
    }

    // Updates the device and controller states. Calculates view orientation.
    // Sets view frustum and near/far planes.
    protected bool UpdateState()
    {
        return true;
    }

    // Performs actions based on controller inputs.
    protected bool HandleInput()
    {
        return true;
    }

    // Draws the left and right eye views and sends the result to the HMD.
    protected bool Draw()
    {
        return true;
    }

    // Fills in the high-level controller state from the data provided by the VR library.
    protected void GetRhinoVrControllerState(VRControllerState_t state, RhinoVrDeviceController controller)
    {
        
    }
  
    // Processes VR events.
    protected void ProcessVrEvent(VREvent_t vr_event)
    {

    }

    // Simulate Rhino's GetPoint in VR.
    protected void RhinoVrGetPoint(Transform picking_device_xform)
    {

    }

    // Simulate Rhino's OnMouseMove in VR.
    protected void RhinoVrOnMouseMove(Transform picking_device_xform)
    {

    }

    // Find object intersection with an eye-space ray transformed by 'picking_device_xform'.
    // The eye-space ray is (0.0, 0.0, -frustum_near) to (0.0, 0.0, -frustum_far).
    protected bool RhinoVrGetIntersectingObject(
        Transform picking_device_xform, out RhinoObject isect_object, out Point3d isect_point)
    {
        isect_object = null;
        isect_point = Point3d.Unset;
        return true;
    }

    protected bool GetWorldPickLineAndClipRegion(
        Transform picking_device_xform,
        out Line world_line,
        out int/*ClippingRegion*/ clip_region,
        out ViewportInfo line_vp,
        out int line_pixel_x,
        out int line_pixel_y)
    {
        world_line = Line.Unset;
        clip_region = 0;
        line_vp = null;
        line_pixel_x = 0;
        line_pixel_y = 0;
        return false;
    }

    // Measures time spent processing input and drawing a stereo frame.
    protected void FrameTimingStart()
    {

    }

    protected void FrameTimingStop()
    {

    }

    // Measures time spent outside RhinoVR.
    protected void RhinoTimingStart()
    {

    }

    protected void RhinoTimingStop()
    {

    }

    // Measures time spent waiting for vertical sync.
    protected void VsyncTimingStart()
    {

    }

    protected void VsyncTimingStop()
    {

    }

    // Measures the total frame time and frames per second.
    protected void FpsTiming()
    {

    }

    protected long TimingStart()
    {
        return 0;
    }

    void TimingStop(long start_time, string message)
    {

    }

    protected long m_frame_time_start = 0;
    protected long m_rhino_time_start = 0;
    protected long m_vsync_time_start = 0;
    protected long m_fps_time_start = 0;
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
    protected Transform m_cam_to_world_xform = Transform.Identity;

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

    // The transforms of all tracked devices.
    protected TrackedDevicePose_t[] m_device_poses = new TrackedDevicePose_t[OpenVR.k_unMaxTrackedDeviceCount];

    protected uint m_device_index_left_hand  = 0; // Device index for left hand controller.
    protected uint m_device_index_right_hand = 0; // Device index for right hand controller.

    // The device data of all tracked devices.
    protected List<RhinoVrDeviceData> m_device_data = new List<RhinoVrDeviceData>((int)OpenVR.k_unMaxTrackedDeviceCount);

    // The render models of all tracked devices.
    protected List<RhinoVrDeviceModel> m_device_render_models = new List<RhinoVrDeviceModel>();

    protected Mesh m_hidden_mesh_left  = null; // The hidden area mesh for the left eye.
    protected Mesh m_hidden_mesh_right = null; // The hidden area mesh for the right eye.

    //protected CRhinoCacheHandle m_hidden_mesh_left_cache_handle;  // The left hidden area mesh cache handle.
    //protected CRhinoCacheHandle m_hidden_mesh_right_cache_handle; // The right hidden area mesh cache handle.

    // The hidden area mesh display conduit.
    protected RhinoVrHiddenAreaMeshDisplayConduit m_hidden_mesh_display_conduit;
}
