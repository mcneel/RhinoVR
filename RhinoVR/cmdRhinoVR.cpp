// cmdRhinoVR.cpp : command file
//

#include "StdAfx.h"
#include "RhinoVrWindow.h"

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
//
// BEGIN RhinoVR command
//

#pragma region RhinoVR command

class CCommandRhinoVR : public CRhinoCommand
{
public:
  CCommandRhinoVR() = default;
  ~CCommandRhinoVR() = default;

  UUID CommandUUID() override
  {
    // {83BECDCB-0B7D-4C07-8002-829318E2E394}
    static const GUID RhinoVRCommand_UUID =
    { 0x83BECDCB, 0xB7D, 0x4C07, { 0x80, 0x2, 0x82, 0x93, 0x18, 0xE2, 0xE3, 0x94 } };
    return RhinoVRCommand_UUID;
  }
  const wchar_t* EnglishCommandName() override { return L"RhinoVR"; }

  CRhinoCommand::result RunCommand(const CRhinoCommandContext& context) override;
};

// The one and only CCommandRhinoVR object
// Do NOT create any other instance of a CCommandRhinoVR class.
static class CCommandRhinoVR theRhinoVRCommand;

CRhinoViewport* vr_vp = nullptr;
CRhinoDisplayPipeline* vr_dp = nullptr;

struct RhinoVrStruct
{
  bool m_running = false;
  RhinoVrRenderer* m_renderer = nullptr;
} g_rhino_vr;

void RhinoVrUpdate()
{
  if (g_rhino_vr.m_running && g_rhino_vr.m_renderer)
  {
    g_rhino_vr.m_renderer->HandleInputAndRenderFrame();
  }

}

//const int vr_view_width = 1512;
//const int vr_view_height = 1680;

//const int vr_view_width = 1512/2;
//const int vr_view_height = 1680/2;

const int vr_view_width = 1080;
const int vr_view_height = 1200;

CRhinoCommand::result CCommandRhinoVR::RunCommand(const CRhinoCommandContext& context)
{
  if (g_rhino_vr.m_running)
  {
    // Let the timer do the teardown.
    g_rhino_vr.m_running = false;

    delete g_rhino_vr.m_renderer;
    g_rhino_vr.m_renderer = nullptr;

    if (vr_vp)
    {
      // Prevent any copied conduit bindings from getting unbound by
      // viewport's destructor...Since "CopyFrom" also forces copying
      // the viewport's Id so that any bound conduits work on copied viewport,
      // we must also force the Id to something other than the VP that 
      // got copied, so the destructor does not then unbind any conduits 
      // to said VP...
      // RH-34780: http://mcneel.myjetbrains.com/youtrack/issue/RH-34780
      vr_vp->m_v.m_vp.ChangeViewportId(ON_nil_uuid);

      delete vr_vp;
      vr_vp = nullptr;
    }

    if (vr_dp)
    {
      delete vr_dp;
      vr_dp = nullptr;
    }

    return CRhinoCommand::success;
  }


  CRhinoView* rhino_view = RhinoApp().ActiveView();
  if (rhino_view == nullptr)
  {
    return CRhinoCommand::failure;
  }

  CRhinoDisplayPipeline* rhino_dp = rhino_view->DisplayPipeline();
  if (rhino_dp == nullptr)
  {
    return CRhinoCommand::failure;
  }

  CRhinoViewport& rhino_vp = rhino_view->ActiveViewport();

  vr_vp = new CRhinoViewport();
  vr_vp->CopyFrom(rhino_vp, true);
  vr_vp->SetScreenSize(vr_view_width, vr_view_height);

  //vr_dp = rhino_dp;
  //vr_vp->AttachPipeline(vr_dp);

  rhino_dp->OpenPipeline();
  vr_dp = rhino_dp->ClonePipeline(*vr_vp);
  rhino_dp->ClosePipeline();

  if (vr_dp == nullptr)
  {
    return CRhinoCommand::failure;
  }

  unsigned int vr_doc_sn = context.m_rhino_doc_sn;
  unsigned int vr_view_sn = rhino_view->RuntimeSerialNumber();
  unsigned int vr_viewport_sn = vr_vp->RuntimeSerialNumber();

  g_rhino_vr.m_renderer = new RhinoVrRenderer(vr_doc_sn, vr_view_sn, vr_viewport_sn);
  if (!g_rhino_vr.m_renderer->InitializeVrRenderer())
  {

  }

  g_rhino_vr.m_running = true;

  return CRhinoCommand::success;
}

#pragma endregion

//
// END RhinoVR command
//
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
