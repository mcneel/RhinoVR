
using Rhino.Display;
using Rhino.Geometry;
using System.Collections.Generic;

public class RhinoVrDeviceDisplayConduit : DisplayConduit
{
    protected override void CalculateBoundingBox(CalculateBoundingBoxEventArgs e)
    {
        BoundingBox xformed_bb = m_bounding_box;
        xformed_bb.Transform(m_device_mesh_xform);

        e.BoundingBox.Union(xformed_bb);
    }

    protected override void PostDrawObjects(DrawEventArgs e)
    {
        e.Display.PushModelTransform(m_device_mesh_xform);

        if (m_pointer_mesh != null)
        {
            e.Display.DrawMeshShaded(m_pointer_mesh, m_pointer_mesh_material);
        }

        e.Display.DrawMeshShaded(m_device_mesh, m_device_material);

        for (int i = 0; i < m_plane_meshes.Count; ++i)
        {
            Mesh mesh = m_plane_meshes[i];
            e.Display.DrawMeshShaded(mesh, m_plane_materials[i]);
        }

        e.Display.PopModelTransform();
    }

    public void SetPointerMesh(Mesh pointer_mesh)
    {
        m_pointer_mesh = pointer_mesh;
    }

    public void SetPointerMeshMaterial(DisplayMaterial pointer_mesh_material)
    {
        m_pointer_mesh_material = pointer_mesh_material;
    }

    public void SetDeviceMesh(Mesh device_mesh)
    {
        m_device_mesh = device_mesh;
    }

    public void SetDeviceMaterial(DisplayMaterial device_material)
    {
        m_device_material = device_material;
    }

    public void SetDeviceMeshXform(Transform device_xform)
    {
        m_device_mesh_xform = device_xform;
    }

    public void AddWindowMesh(Mesh mesh, DisplayMaterial material)
    {
        m_bounding_box.Union(mesh.Vertices[0]);
        m_bounding_box.Union(mesh.Vertices[1]);
        m_bounding_box.Union(mesh.Vertices[2]);
        m_bounding_box.Union(mesh.Vertices[3]);

        m_plane_meshes.Add(mesh);
        m_plane_materials.Add(material);
    }

    public void AddLine(Point3d from, Point3d to, Color4f color)
    {
        m_start_pts.Add(from);
        m_end_pts.Add(to);
        m_colors.Add(color);

        m_bounding_box.Union(from);
        m_bounding_box.Union(to);
    }

    public void Empty()
    {
        m_start_pts.Clear();
        m_end_pts.Clear();
        m_colors.Clear();

        m_bounding_box = BoundingBox.Empty;

        m_draw_device_mesh = false;
        m_device_mesh = null;
        m_device_material = null;
        
        m_plane_meshes.Clear();
        m_plane_xforms.Clear();
        m_plane_materials.Clear();
    }

    private double m_frus_near_suggestion = -1.0;
    private double m_frus_far_suggestion  = -1.0;

    private List<Point3d> m_start_pts = new List<Point3d>();
    private List<Point3d> m_end_pts   = new List<Point3d>();
    private List<Color4f> m_colors    = new List<Color4f>();

    private Mesh m_pointer_mesh = null;
    private DisplayMaterial m_pointer_mesh_material = null;

    private bool m_draw_device_mesh = false;

    private Mesh m_device_mesh = null;
    private DisplayMaterial m_device_material = null;
    private Transform m_device_mesh_xform = Transform.Identity;

    List<Mesh> m_plane_meshes = new List<Mesh>();
    List<Transform> m_plane_xforms = new List<Transform>();
    List<DisplayMaterial> m_plane_materials = new List<DisplayMaterial>();

    private BoundingBox m_bounding_box = BoundingBox.Empty;
};
