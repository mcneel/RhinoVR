
using Rhino.Display;
using Rhino.Geometry;
using Valve.VR;

public class RhinoVrHiddenAreaMeshDisplayConduit : DisplayConduit
{
    public RhinoVrHiddenAreaMeshDisplayConduit()
    {

    }

    protected override void PreDrawObjects(DrawEventArgs e)
    {
        base.PreDrawObjects(e);
    }

    public void Enable(uint doc_sn)
    {

    }

    public void SetHiddenAreaMesh(Mesh device_mesh, EVREye eye)
    {

    }
    public void SetHiddenAreaMeshXform(Transform device_transform, EVREye eye)
    {

    }

    //public void SetHiddenAreaMeshCacheHandle(CRhinoCacheHandle cache_handle, EVREye eye);

    public void Empty()
    {

    }

    private bool m_draw_hidden_area_mesh = false;

    private Mesh m_hidden_area_mesh_left = null;
    private Mesh m_hidden_area_mesh_right = null;
    private Transform m_hidden_area_mesh_left_xform = Transform.Identity;
    private Transform m_hidden_area_mesh_right_xform = Transform.Identity;

    //private CRhinoCacheHandle m_hidden_area_mesh_left_cache_handle;
    //private CRhinoCacheHandle m_hidden_area_mesh_right_cache_handle;
}
