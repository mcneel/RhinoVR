
using Rhino.Display;
using Rhino.Geometry;
using System.Collections.Generic;

public class RhinoVrDeviceDisplayConduit : DisplayConduit
{
    public RhinoVrDeviceDisplayConduit()
    {
    }

    protected override void ObjectCulling(CullObjectEventArgs e)
    {
        base.ObjectCulling(e);
    }

    protected override void CalculateBoundingBox(CalculateBoundingBoxEventArgs e)
    {
        base.CalculateBoundingBox(e);
    }

    protected override void PostDrawObjects(DrawEventArgs e)
    {
        base.PostDrawObjects(e);
    }

    public void Enable(uint doc_sn)
    {

    }

    public void SetDeviceMesh(Mesh device_mesh)
    {

    }

    public void SetDeviceMaterial(DisplayMaterial device_material)
    {

    }

    public void SetDeviceMeshXform(Transform device_transform)
    {

    }

    //public void SetDeviceMeshCacheHandle(CRhinoCacheHandle cache_handle)
    //{

    //}

    public void SetFrustumNearFarSuggestion(double frus_near, double frus_far)
    {

    }

    public void AddLine(Point3d from, Point3d to, Color4f color)
    {

    }

    public void Empty()
    {

    }

    private double m_frus_near_suggestion = -1.0;
    private double m_frus_far_suggestion  = -1.0;

    private List<Point3d> m_start_pts = new List<Point3d>();
    private List<Point3d> m_end_pts   = new List<Point3d>();
    private List<Color4f> m_colors    = new List<Color4f>();

    private bool m_draw_device_mesh = false;

    private Mesh m_device_mesh = null;
    private DisplayMaterial m_device_material = null;
    private Transform m_device_mesh_transform = Transform.Identity;
    //private CRhinoCacheHandle m_device_cache_handle;

    private BoundingBox m_bounding_box = BoundingBox.Empty;
};
