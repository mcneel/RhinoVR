
using Rhino;
using Rhino.Display;
using Rhino.DocObjects;
using Rhino.Geometry;
using System.Collections.Generic;
using Valve.VR;

// RhinoVrRenderer is the main class of RhinoVR. It handles VR library
// initialization, input handling and frame drawing.
public class RhinoVrRenderer
{
    public RhinoVrRenderer(uint doc_sn, uint view_sn)
    {

    }

    ~RhinoVrRenderer()
    {

    }

    public bool Initialize()
    {
        return true;
    }

    public void ProcessInputAndRenderFrame()
    {

    }

    // Converts an OpenVR matrix to an ON_Xform.
    protected Transform OpenVrMatrixToRhinoTransform(HmdMatrix34_t matrix)
    {

    }

    // Loads all needed render models from the VR library.
    protected void SetupRenderModels()
    {

    }

    // Loads a render model for a specific device.
    protected void SetupRenderModelForDevice(uint tracked_device_index)
    {

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
        return true;
    }

    protected bool GetWorldPickLineAndClipRegion(
        Transform picking_device_xform,
        out Line world_line,
        out ClippingRegion clip_region,
        out ViewportInfo line_vp,
        out int line_pixel_x,
        out int line_pixel_y)
    {

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
    protected DisplayPipeline_OGL m_vr_dp_ogl = null;

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
    protected List<RhinoVrDeviceData> m_device_data = new List<RhinoVrDeviceData>();

    // The render models of all tracked devices.
    protected List<RhinoVrDeviceModel> m_device_render_models = new List<RhinoVrDeviceModel>();

    protected Mesh m_hidden_mesh_left  = null; // The hidden area mesh for the left eye.
    protected Mesh m_hidden_mesh_right = null; // The hidden area mesh for the right eye.

    protected CRhinoCacheHandle m_hidden_mesh_left_cache_handle;  // The left hidden area mesh cache handle.
    protected CRhinoCacheHandle m_hidden_mesh_right_cache_handle; // The right hidden area mesh cache handle.

    // The hidden area mesh display conduit.
    protected RhinoVrHiddenAreaMeshDisplayConduit m_hidden_mesh_display_conduit;

    private IVRSystem       m_hmd;           // System interface owned by the VR library.
    private IVRRenderModels m_render_models; // Render models interface owned by the VR library.
    private IVRCompositor   m_compositor;    // Compositor interface owned by the VR library.
}
