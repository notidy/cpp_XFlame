#include "XFlame.h"

extern "C"
{

    XFlame* XFlame_Create()
    {
        return new XFlame();
    }

    void XFlame_Destroy(XFlame* p)
    {
        delete p;
    }

    bool XFlame_Init(
            XFlame* p,
            const void* generic_model,
            size_t generic_model_size,
            const void* flame_track,
            size_t flame_track_size,
            const void* data_info,
            size_t data_info_size,
            const void* drive_mesh,
            size_t drive_mesh_size)
    {
        return p->Init(
            generic_model,
            generic_model_size,
            flame_track,
            flame_track_size,
            data_info,
            data_info_size,
            drive_mesh,
            drive_mesh_size
        );
    }

    void XFlame_ComputeFrame(XFlame* p, int frame, const float* opt_motion_data = nullptr, int cur_expr_jaw_frame = -1)
    {
        p->ComputeFrame(frame, opt_motion_data, cur_expr_jaw_frame);
    }

    const float* XFlame_GetDriveMeshVtsPtr(XFlame* p)
    {
        return p->GetDriveMeshVtsPtr();
    }

    int XFlame_GetDriveMeshVtsCount(XFlame* p)
    {
        return p->GetDriveMeshVtsCount();
    }

    const int* XFlame_GetDriveMeshFacePtr(XFlame* p)
    {
        return p->GetDriveMeshFacePtr();
    }

    int XFlame_GetDriveMeshFaceCount(XFlame* p)
    {
        return p->GetDriveMeshFaceCount();
    }

	int XFlame_GetNFrames(XFlame* p)
	{
		return p->GetNFrames();
	}

    const float* XFlame_GetDriveMeshRestNormalPtr(XFlame* p)
	{
		return p->GetDriveMeshRestNormalPtr();
	}

    const float* XFlame_GetDriveMeshNormalPtr(XFlame* p)
	{
		return p->GetDriveMeshNormalPtr();
	}

	const float* XFlame_GetCompParmPtr(XFlame* p)
	{
		return p->GetCompParmPtr();
	}

    const float* XFlame_GetCamParmPtr(XFlame* p)
    {
        return p->GeCamParmPtr();
    }

	const unsigned* XFlame_GetRenderSizePtr(XFlame* p)
	{
		return p->GetRenderSizePtr();
	}
}
