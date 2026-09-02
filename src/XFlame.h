#pragma once
#include <Eigen/Dense>
#include "XDataUtil.h"


struct RigidTransformResult
{
    Eigen::Matrix<float, 5, 3> posed_joints;

    // 每个joint一块4×4
    std::array<Eigen::Matrix4f, 5> rel_transforms;
};


Eigen::Matrix3f Rodrigues(const Eigen::Vector3f& r);
std::vector<Eigen::Matrix3f> BatchRodrigues(const Eigen::MatrixXf& rot_vecs);
RigidTransformResult BatchRigidTransform(
    const std::array<Eigen::Matrix3f, 5>& rot_mats,
    const Eigen::Matrix<float, 5, 3>& joints,
    const Eigen::Ref<const Eigen::Matrix<float, 1, 5>>& parents
);

inline Eigen::Matrix4f TransformMat(
    const Eigen::Matrix3f& R,
    const Eigen::Vector3f& t
)
{
    Eigen::Matrix4f T;

    T <<
        R(0, 0), R(0, 1), R(0, 2), t(0),
        R(1, 0), R(1, 1), R(1, 2), t(1),
        R(2, 0), R(2, 1), R(2, 2), t(2),
        0, 0, 0, 1;

    return T;
}

class XFlame
{
public:
    bool Init(
        const void* ptr_generic_model_data, size_t bsize_generic_model_data,
        const void* ptr_flame_track_data, size_t bsize_flame_track_data,
        const void* ptr_data_info_data, size_t bsize_data_info_data,
        const void* ptr_drive_mesh_data, size_t bsize_drive_mesh_data
    );
    void ComputeFrame(const int& frame, const float* opt_motion_data=nullptr, const int& cur_expr_jaw_frame=-1);

    const float* GetDriveMeshVtsPtr() const { return drive_mesh_data.vts.data(); }
    int GetDriveMeshVtsCount() const { return drive_mesh_data.vts.rows(); }
    const int* GetDriveMeshFacePtr() const { return drive_mesh_data.faces.data(); }
    int GetDriveMeshFaceCount() const { return drive_mesh_data.faces.rows(); }
	int GetNFrames() const { return nframes; }

    //获取法线 -----
    const float* GetDriveMeshRestNormalPtr() const { return drive_mesh_data.rest_normals.data(); }
    const float* GetDriveMeshNormalPtr() const { return drive_mesh_data.normals.data(); }

    const float* GetCompParmPtr() { return gs_comp_parm_data; }
    const float* GeCamParmPtr() { return cam_parm; }
    const unsigned* GetRenderSizePtr() { return gs_render_size_data; }
    
    const char* GetLastError() const { return last_error.c_str(); }

private:
    void ComputeFrameRest();

    std::string last_error;

    // drive_mesh_data ------------------------------------------------------
    XMeshData drive_mesh_data;

    // generic_model_data ------------------------------------------------------
    Eigen::Matrix<float, -1, 3, Eigen::RowMajor> v_template_data;
    Eigen::Matrix<float, -1, 400, Eigen::RowMajor> shapedirs_data;
    Eigen::Matrix<float, 36, -1, Eigen::ColMajor> posedirs_data;
    Eigen::Matrix<float, -1, 5, Eigen::RowMajor> weights_data;
    Eigen::Matrix<float, 2, 5, Eigen::RowMajor> kintree_table_data;
    Eigen::Matrix<float, 5, -1, Eigen::RowMajor> J_regressor_data;

    // flame_track_data ------------------------------------------------------
    Eigen::Matrix<float, 1, 300, Eigen::RowMajor> shape_data;
    Eigen::Matrix<float, Eigen::Dynamic, 100, Eigen::RowMajor> expr_data;
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> neck_pose_data;
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> jaw_pose_data;
    Eigen::Matrix<float, Eigen::Dynamic, 6, Eigen::RowMajor> eye_pose_data;
    //Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> global_orient_data;
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> transl_data;
    std::vector<Eigen::Matrix3f> g_R;

    // data_info_data ------------------------------------------------------
    Eigen::VectorXi drive_mesh_tar_pt_ind_data;
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> mesh_pos_offset_data;
    Eigen::Matrix<float, Eigen::Dynamic, 100, Eigen::RowMajor> expr_offset_data;
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> jaw_pose_offset_data;
    Eigen::Matrix<float, Eigen::Dynamic, 6, Eigen::RowMajor> eye_pose_offset_data;
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> neck_pose_offset_data;
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> trans_offset_data;
    float gs_comp_parm_data[9], cam_parm[4];
    unsigned gs_render_size_data[2];

    int nframes = -1;
    int drive_mesh_npts = -1;
    int flame_npts = -1;
    int drive_mesh_main_pt_count = -1;

};

