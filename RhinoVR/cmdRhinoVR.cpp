// cmdRhinoVR.cpp : command file
//

#include "StdAfx.h"
#include "RhinoVrRenderer.h"
#include "RhinoVRPlugIn.h"

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

class RhinoVrMainLoopEventHook;

struct RhinoVrStruct
{
  bool m_running = false;
  RhinoVrRenderer* m_renderer = nullptr;
  RhinoVrMainLoopEventHook* m_loop_hook = nullptr;
} g_rhino_vr;


class RhinoVrMainLoopEventHook : public CRhinoOnMainLoopEvent
{
public:
  RhinoVrMainLoopEventHook(ON_UUID plugin_id) : CRhinoOnMainLoopEvent(plugin_id) {}

  void Notify(const class CRhinoOnMainLoopEvent::CParameters& params) override
  {
    if (g_rhino_vr.m_running && g_rhino_vr.m_renderer)
    {
      g_rhino_vr.m_renderer->ProcessInputAndRenderFrame();
    }
  }
};

CRhinoCommand::result CCommandRhinoVR::RunCommand(const CRhinoCommandContext& context)
{
  if (g_rhino_vr.m_running)
  {
    g_rhino_vr.m_running = false;

    delete g_rhino_vr.m_loop_hook;
    g_rhino_vr.m_loop_hook = nullptr;

    delete g_rhino_vr.m_renderer;
    g_rhino_vr.m_renderer = nullptr;

    return CRhinoCommand::success;
  }

  CRhinoView* rhino_view = RhinoApp().ActiveView();
  if (rhino_view == nullptr)
    return CRhinoCommand::failure;

  unsigned int vr_doc_sn = context.m_rhino_doc_sn;
  unsigned int vr_view_sn = rhino_view->RuntimeSerialNumber();

  g_rhino_vr.m_renderer = new RhinoVrRenderer(vr_doc_sn, vr_view_sn);

  if (!g_rhino_vr.m_renderer->Initialize())
  {
    delete g_rhino_vr.m_renderer;
    g_rhino_vr.m_renderer = nullptr;

    return CRhinoCommand::failure;
  }

  g_rhino_vr.m_loop_hook = new RhinoVrMainLoopEventHook(RhinoVRPlugIn().PlugInID());
  g_rhino_vr.m_loop_hook->Register();

  g_rhino_vr.m_running = true;

  return CRhinoCommand::success;
}

#pragma endregion

//
// END RhinoVR command
//
////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
