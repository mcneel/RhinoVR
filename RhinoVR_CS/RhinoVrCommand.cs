using System;
using Rhino;
using Rhino.Commands;

namespace RhinoVR_CS
{
    public static class RhinoVrStruct
    {
        public static bool m_running = false;
        public static RhinoVrRenderer m_renderer = null;
    }

    public class RhinoVrCommand : Command
    {
        public RhinoVrCommand()
        {
            // Rhino only creates one instance of each command class defined in a
            // plug-in, so it is safe to store a refence in a static property.
            Instance = this;
        }

        ///<summary>The only instance of this command.</summary>
        public static RhinoVrCommand Instance
        {
            get; private set;
        }

        ///<returns>The command name as it appears on the Rhino command line.</returns>
        public override string EnglishName
        {
            get { return "RhinoVRCommon"; }
        }

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            if(RhinoVrStruct.m_running)
            {
                RhinoVrTearDown();

                return Result.Success;
            }

            var rhino_view = doc.Views.ActiveView;
            if (rhino_view == null)
                return Result.Failure;

            var doc_sn = doc.RuntimeSerialNumber;
            var view_sn = rhino_view.RuntimeSerialNumber;

            RhinoVrStruct.m_renderer = new RhinoVrRenderer(doc_sn, view_sn);

            if(!RhinoVrStruct.m_renderer.Initialize())
            {
                RhinoVrStruct.m_renderer = null;

                return Result.Failure;
            }
            
            RhinoVrStruct.m_running = true;

            RhinoDoc.CloseDocument += OnCloseDocument;
            RhinoApp.MainLoop      += OnMainLoop;

            return Result.Success;
        }

        protected void OnMainLoop(object sender, EventArgs e)
        {
            if (RhinoVrStruct.m_running && (RhinoVrStruct.m_renderer != null))
            {
                RhinoVrStruct.m_renderer.ProcessInputAndRenderFrame();
            }
        }

        protected void OnCloseDocument(object sender, EventArgs e)
        {
            if (RhinoVrStruct.m_running)
            {
                RhinoVrTearDown();
            }
        }

        protected void RhinoVrTearDown()
        {
            RhinoVrStruct.m_running = false;

            RhinoApp.MainLoop      -= OnMainLoop;
            RhinoDoc.CloseDocument -= OnCloseDocument;
            
            RhinoVrStruct.m_renderer = null;
        }
    }
}
