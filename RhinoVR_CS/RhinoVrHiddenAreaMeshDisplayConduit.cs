
using Rhino.Display;
using Rhino.Geometry;
using Valve.VR;

public class RhinoVrHiddenAreaMeshDisplayConduit : DisplayConduit
{
    protected override void PreDrawObjects(DrawEventArgs e)
    {
        if (m_draw_hidden_area_mesh && e.Display.DisplayPipelineAttributes.ShadingEnabled)
        {
            var stereo_render_context = e.Display.DisplayPipelineAttributes.StereoRenderContext;

            if (m_hidden_area_mesh_left != null &&
              (stereo_render_context == DisplayPipelineAttributes.StereoRenderContextEnum.RenderingLeftEye ||
               stereo_render_context == DisplayPipelineAttributes.StereoRenderContextEnum.RenderingBothEyes))
            {
                e.Display.PushModelTransform(m_hidden_area_mesh_left_xform);
                e.Display.DrawMeshShaded(m_hidden_area_mesh_left, null);
                e.Display.PopModelTransform();
            }

            if (m_hidden_area_mesh_right != null &&
              (stereo_render_context == DisplayPipelineAttributes.StereoRenderContextEnum.RenderingRightEye ||
               stereo_render_context == DisplayPipelineAttributes.StereoRenderContextEnum.RenderingBothEyes))
            {
                e.Display.PushModelTransform(m_hidden_area_mesh_right_xform);
                e.Display.DrawMeshShaded(m_hidden_area_mesh_right, null);
                e.Display.PopModelTransform();
            }
        }
    }

    public void SetHiddenAreaMesh(Mesh hidden_area_mesh, EVREye eye)
    {
        if (eye == EVREye.Eye_Left)
        {
            m_hidden_area_mesh_left = hidden_area_mesh;
        }
        else
        {
            m_hidden_area_mesh_right = hidden_area_mesh;
        }

        if (m_hidden_area_mesh_left != null && m_hidden_area_mesh_right != null)
        {
            m_draw_hidden_area_mesh = true;
        }
    }
    public void SetHiddenAreaMeshXform(Transform hidden_area_mesh_xform, EVREye eye)
    {
        if (eye == EVREye.Eye_Left)
        {
            m_hidden_area_mesh_left_xform = hidden_area_mesh_xform;
        }
        else
        {
            m_hidden_area_mesh_right_xform = hidden_area_mesh_xform;
        }
    }

    public void Empty()
    {
        m_draw_hidden_area_mesh = false;
        m_hidden_area_mesh_left = null;
        m_hidden_area_mesh_right = null;
    }

    private bool m_draw_hidden_area_mesh = false;

    private Mesh m_hidden_area_mesh_left = null;
    private Mesh m_hidden_area_mesh_right = null;
    private Transform m_hidden_area_mesh_left_xform = Transform.Identity;
    private Transform m_hidden_area_mesh_right_xform = Transform.Identity;
}
