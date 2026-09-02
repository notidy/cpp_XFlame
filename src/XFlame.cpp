#include <cstring>
#include "XFlame.h"


Eigen::Matrix3f Rodrigues(const Eigen::Vector3f& r)
{
    float angle = r.norm();
    if (angle < 1e-8f)
        return Eigen::Matrix3f::Identity();
    return Eigen::AngleAxisf(angle, r.normalized()).toRotationMatrix();
}

std::vector<Eigen::Matrix3f> BatchRodrigues(const Eigen::MatrixXf& rot_vecs)
{
    int N = rot_vecs.rows();
    std::vector<Eigen::Matrix3f> out(N);
    for (int i = 0; i < N; ++i)
        out[i] = Rodrigues(rot_vecs.row(i).transpose());
    return out;
}

RigidTransformResult BatchRigidTransform(
    const std::array<Eigen::Matrix3f, 5>& rot_mats,
    const Eigen::Matrix<float, 5, 3>& joints,
    const Eigen::Ref<const Eigen::Matrix<float, 1, 5>>& parents
)
{
    RigidTransformResult result{};
    std::array<Eigen::Matrix4f, 5> transforms;

    //-------------------------------------
    // root
    //-------------------------------------

    transforms[0] = TransformMat(rot_mats[0], joints.row(0));

    //-------------------------------------
    // chain
    //-------------------------------------

    for (int i = 1; i < 5; i++)
    {
        Eigen::Vector3f rel = joints.row(i) - joints.row((int)parents(i));
        transforms[i] = transforms[(int)parents(i)] * TransformMat(rot_mats[i], rel);
    }

    //-------------------------------------
    // output
    //-------------------------------------
    for (int i = 0; i < 5; i++)
    {
        result.posed_joints.row(i) = transforms[i].block<3, 1>(0, 3).transpose();
        Eigen::Vector4f j;
        j << joints.row(i).transpose(), 0;

        Eigen::Matrix4f pad = Eigen::Matrix4f::Zero();
        pad.col(3) = transforms[i] * j;
        result.rel_transforms[i] = transforms[i] - pad;
    }

    return result;
}

void ComputeVertexNormals(
    const Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& vts,
    const Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>& faces,
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>& normals
)
{
    const int vertexCount = vts.rows();
    const int faceCount = faces.rows();

    normals.resize(vertexCount, 3);
    normals.setZero();

    for (int i = 0; i < faceCount; ++i)
    {
        const int i0 = faces(i, 0);
        const int i1 = faces(i, 1);
        const int i2 = faces(i, 2);

        const Eigen::Vector3f v0 = vts.row(i0);
        const Eigen::Vector3f v1 = vts.row(i1);
        const Eigen::Vector3f v2 = vts.row(i2);

        Eigen::Vector3f e1 = v1 - v0;
        Eigen::Vector3f e2 = v2 - v0;

        Eigen::Vector3f faceNormal = e1.cross(e2);

        normals.row(i0) += faceNormal;
        normals.row(i1) += faceNormal;
        normals.row(i2) += faceNormal;
    }

    for (int i = 0; i < vertexCount; ++i)
    {
        Eigen::Vector3f n = normals.row(i);
        float len = n.norm();

        if (len > 1e-8f)
        {
            normals.row(i) = n / len;
        }
        else
        {
            normals.row(i).setZero();
        }
    }
}

bool XFlame::Init
(
    const void* ptr_generic_model_data, size_t bsize_generic_model_data,
    const void* ptr_flame_track_data, size_t bsize_flame_track_data,
    const void* ptr_data_info_data, size_t bsize_data_info_data,
    const void* ptr_drive_mesh_data, size_t bsize_drive_mesh_data
)
{
    if(!XDataUtil::LoadObj(ptr_drive_mesh_data, bsize_drive_mesh_data, drive_mesh_data))
	{
		last_error = "Failed to load drive mesh from data. Please check mesh file.";
		return false;
	}
    drive_mesh_npts = drive_mesh_data.orig_vts.rows();

    // generic_model_data ------------------------------------------------------
    auto generic_model_data = XDataUtil::LoadBin32(ptr_generic_model_data, bsize_generic_model_data);
    flame_npts = generic_model_data["v_template"].size() / 12;

    v_template_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(generic_model_data, "v_template", flame_npts, 3);
    shapedirs_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 400>(generic_model_data, "shapedirs", flame_npts * 3, 400);
    posedirs_data = XDataUtil::GetBinMatrix<float, 36, Eigen::Dynamic, Eigen::ColMajor>(generic_model_data, "posedirs", 36, flame_npts * 3);
    weights_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 5>(generic_model_data, "weights", flame_npts, 5);
    kintree_table_data = XDataUtil::GetBinMatrix<float, 2, 5>(generic_model_data, "kintree_table", 2, 5);
    J_regressor_data = XDataUtil::GetBinMatrix<float, 5, Eigen::Dynamic>(generic_model_data, "J_regressor", 5, flame_npts);

    // flame_track_data ------------------------------------------------------
    auto flame_track_data = XDataUtil::LoadBin32(ptr_flame_track_data, bsize_flame_track_data);
    nframes = flame_track_data["expr"].size() / 400;

    shape_data = XDataUtil::GetBinMatrix<float, 1, 300>(flame_track_data, "shape", 1, 300);
    expr_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 100>(flame_track_data, "expr", nframes, 100);
    jaw_pose_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(flame_track_data, "jaw_pose", nframes, 3);
    eye_pose_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 6>(flame_track_data, "eye_pose", nframes, 6);
    neck_pose_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(flame_track_data, "neck_pose", nframes, 3);
    auto global_orient_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(flame_track_data, "global_orient", nframes, 3);
    transl_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(flame_track_data, "transl", nframes, 3);
    std::memcpy(cam_parm, flame_track_data.at("cam_para").data(), sizeof(cam_parm));

    g_R = std::vector<Eigen::Matrix3f>(nframes);
    for (int i = 0; i < nframes; ++i)
        g_R[i] = Rodrigues(global_orient_data.row(i).transpose());

    // data_info_data ------------------------------------------------------
    auto data_info_data = XDataUtil::LoadBin32(ptr_data_info_data, bsize_data_info_data);

    drive_mesh_tar_pt_ind_data = XDataUtil::GetBinMatrix<int, Eigen::Dynamic, 1, Eigen::ColMajor>(data_info_data, "drive_mesh_tar_pt_ind", drive_mesh_npts, 1);

    //计算驱动网格中有映射点的点数量（其余牙齿部分使用关节矩阵来变换）
    drive_mesh_main_pt_count = 0;
    while (drive_mesh_main_pt_count < drive_mesh_npts && drive_mesh_tar_pt_ind_data(drive_mesh_main_pt_count) >= 0)
        ++drive_mesh_main_pt_count;

    //auto gs_pos_offset_raw_data = XDataUtil::GetBinData(data_info_data, "gs_pos_offset");
    //auto gs_pos_offset_data = gs_pos_offset_raw_data.reshaped<Eigen::RowMajor>();
    //auto gs_pos_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(data_info_data, "gs_pos_offset", drive_mesh_npts, 3);

    mesh_pos_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(data_info_data, "mesh_pos_offset", drive_mesh_main_pt_count, 3);
    expr_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 100>(data_info_data, "expr_offset", nframes, 100);
    jaw_pose_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(data_info_data, "jaw_pose_offset", nframes, 3);
    eye_pose_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 6>(data_info_data, "eye_pose_offset", nframes, 6);
    neck_pose_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(data_info_data, "neck_pose_offset", nframes, 3);
    trans_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(data_info_data, "trans_offset", nframes, 3);
    std::memcpy(gs_comp_parm_data, data_info_data.at("gs_comp_parm").data(), sizeof(gs_comp_parm_data));
    std::memcpy(gs_render_size_data, data_info_data.at("gs_render_size").data(), sizeof(gs_render_size_data));

    //添加偏移量
    expr_data += expr_offset_data;
    jaw_pose_data += jaw_pose_offset_data;
    eye_pose_data += eye_pose_offset_data;
    neck_pose_data += neck_pose_offset_data;
    transl_data += trans_offset_data;

    // 计算静态网格顶点法线
    ComputeFrameRest();
    ComputeVertexNormals(
        drive_mesh_data.vts,
        drive_mesh_data.faces,
        drive_mesh_data.rest_normals
    );
	drive_mesh_data.normals = drive_mesh_data.rest_normals;

    return true;
}

void XFlame::ComputeFrameRest()
{
    //lbs --------------------------------
    //calc shape ------------
    Eigen::VectorXf betas(400);
    betas.setZero();
    betas.head<300>() = shape_data.row(0).transpose();
    //betas << shape_data.row(0).transpose();
    Eigen::MatrixXf v_shaped = v_template_data + (shapedirs_data * betas).reshaped<Eigen::RowMajor>(flame_npts, 3);
    Eigen::Matrix<float, 5, 3> J = J_regressor_data * v_shaped;

    //calc pose offset ----------
    std::array<Eigen::Matrix3f, 5> rot_mats;

    rot_mats[0] = Eigen::Matrix3f::Identity();
    rot_mats[1] = Eigen::Matrix3f::Identity();
    rot_mats[2] = Rodrigues(Eigen::Vector3f(0.15f,0.f,0.f));
    rot_mats[3] = Eigen::Matrix3f::Identity();
    rot_mats[4] = Eigen::Matrix3f::Identity();

    Eigen::VectorXf pose_feature(36);
    for (int j = 1; j < 5; ++j)
    {
        auto diff = (rot_mats[j] - Eigen::Matrix3f::Identity());
        Eigen::Map<Eigen::Matrix<float, 1, 9, Eigen::RowMajor>>(pose_feature.data() + (j - 1) * 9) = diff.reshaped<Eigen::RowMajor>(1, 9);
    }
    Eigen::MatrixXf pose_offsets = (pose_feature.transpose() * posedirs_data).reshaped<Eigen::RowMajor>(flame_npts, 3);
    Eigen::MatrixXf v_posed = pose_offsets + v_shaped;

    auto Jtrans_A = BatchRigidTransform(rot_mats, J, kintree_table_data.row(0));

    std::vector<Eigen::Matrix4f> T(flame_npts);
    for (int v = 0; v < flame_npts; ++v)
    {
        Eigen::Matrix4f M = Eigen::Matrix4f::Zero();
        for (int j = 0; j < 5; ++j)
            M.noalias() += weights_data(v, j) * Jtrans_A.rel_transforms[j];
        T[v] = M;
    }

    //flame网格点位置赋值给插值网格主体点------------
    auto* dm_tar = drive_mesh_tar_pt_ind_data.data();
    for (int dm_i = 0; dm_i < drive_mesh_main_pt_count; ++dm_i)
    {
        auto xform = T[dm_tar[dm_i]];
        float x = v_shaped(dm_tar[dm_i], 0) + mesh_pos_offset_data(dm_i, 0);
        float y = v_shaped(dm_tar[dm_i], 1) + mesh_pos_offset_data(dm_i, 1);
        float z = v_shaped(dm_tar[dm_i], 2) + mesh_pos_offset_data(dm_i, 2);

        drive_mesh_data.vts(dm_i, 0) = xform(0, 0) * x + xform(0, 1) * y + xform(0, 2) * z + xform(0, 3);
        drive_mesh_data.vts(dm_i, 1) = xform(1, 0) * x + xform(1, 1) * y + xform(1, 2) * z + xform(1, 3);
        drive_mesh_data.vts(dm_i, 2) = xform(2, 0) * x + xform(2, 1) * y + xform(2, 2) * z + xform(2, 3);
    }

    //插值网格牙齿点驱动------------
    for (int dm_i = drive_mesh_main_pt_count; dm_i < drive_mesh_npts; ++dm_i) {
        auto xform = Jtrans_A.rel_transforms[-dm_tar[dm_i]];
        float x = drive_mesh_data.orig_vts(dm_i, 0);
        float y = drive_mesh_data.orig_vts(dm_i, 1);
        float z = drive_mesh_data.orig_vts(dm_i, 2);

        drive_mesh_data.vts(dm_i, 0) = xform(0, 0) * x + xform(0, 1) * y + xform(0, 2) * z + xform(0, 3);
        drive_mesh_data.vts(dm_i, 1) = xform(1, 0) * x + xform(1, 1) * y + xform(1, 2) * z + xform(1, 3);
        drive_mesh_data.vts(dm_i, 2) = xform(2, 0) * x + xform(2, 1) * y + xform(2, 2) * z + xform(2, 3);
    }

    return;
}

void XFlame::ComputeFrame(const int& frame, const float* opt_motion_data, const int& cur_expr_jaw_frame)
{
	if (frame < 0 || frame >= nframes)
		return;
    int cur_motion_frame = cur_expr_jaw_frame >= nframes ? -1 : cur_expr_jaw_frame;

    //lbs --------------------------------
    //calc shape ------------
    Eigen::VectorXf betas(400);
    if (opt_motion_data)
    {
        betas.head<300>() = shape_data.row(0).transpose();
        betas.tail<100>() = Eigen::Map<const Eigen::VectorXf>(opt_motion_data, 100);
    }
    else
        betas << shape_data.row(0).transpose(), expr_data.row(cur_motion_frame>=0? cur_motion_frame:frame).transpose();

    Eigen::MatrixXf v_shaped = v_template_data + (shapedirs_data * betas).reshaped<Eigen::RowMajor>(flame_npts, 3);
    Eigen::Matrix<float, 5, 3> J = J_regressor_data * v_shaped;

    //calc pose offset ----------
    std::array<Eigen::Matrix3f, 5> rot_mats;
    rot_mats[0] = Eigen::Matrix3f::Identity();
    rot_mats[1] = Rodrigues(neck_pose_data.row(frame).transpose());
    // jaw_pose
    if (opt_motion_data)
        rot_mats[2] = Rodrigues(Eigen::Map<const Eigen::Vector3f>(opt_motion_data + 100));
    else
        rot_mats[2] = Rodrigues(jaw_pose_data.row(cur_motion_frame>=0? cur_motion_frame:frame).transpose());
    rot_mats[3] = Rodrigues(eye_pose_data.row(frame).head<3>().transpose());
    rot_mats[4] = Rodrigues(eye_pose_data.row(frame).tail<3>().transpose());

    Eigen::VectorXf pose_feature(36);
    for (int j = 1; j < 5; ++j)
    {
        auto diff = (rot_mats[j] - Eigen::Matrix3f::Identity());
        Eigen::Map<Eigen::Matrix<float, 1, 9, Eigen::RowMajor>>(pose_feature.data() + (j - 1) * 9) = diff.reshaped<Eigen::RowMajor>(1, 9);
    }
    Eigen::MatrixXf pose_offsets = (pose_feature.transpose() * posedirs_data).reshaped<Eigen::RowMajor>(flame_npts, 3);
    Eigen::MatrixXf v_posed = pose_offsets + v_shaped;

    auto Jtrans_A = BatchRigidTransform(rot_mats, J, kintree_table_data.row(0));

    std::vector<Eigen::Matrix4f> T(flame_npts);
    for (int v = 0; v < flame_npts; ++v)
    {
        Eigen::Matrix4f M = Eigen::Matrix4f::Zero();
        for (int j = 0; j < 5; ++j)
            M.noalias() += weights_data(v, j) * Jtrans_A.rel_transforms[j];
        T[v] = M;
    }

    //flame网格点位置赋值给插值网格主体点------------
    auto* dm_tar = drive_mesh_tar_pt_ind_data.data();
    for (int dm_i = 0; dm_i < drive_mesh_main_pt_count; ++dm_i)
    {
        auto xform = T[dm_tar[dm_i]];
        float x = v_shaped(dm_tar[dm_i], 0) + mesh_pos_offset_data(dm_i, 0);
        float y = v_shaped(dm_tar[dm_i], 1) + mesh_pos_offset_data(dm_i, 1);
        float z = v_shaped(dm_tar[dm_i], 2) + mesh_pos_offset_data(dm_i, 2);

        drive_mesh_data.vts(dm_i, 0) = xform(0, 0) * x + xform(0, 1) * y + xform(0, 2) * z + xform(0, 3);
        drive_mesh_data.vts(dm_i, 1) = xform(1, 0) * x + xform(1, 1) * y + xform(1, 2) * z + xform(1, 3);
        drive_mesh_data.vts(dm_i, 2) = xform(2, 0) * x + xform(2, 1) * y + xform(2, 2) * z + xform(2, 3);
    }

    //插值网格牙齿点驱动------------
    for (int dm_i = drive_mesh_main_pt_count; dm_i < drive_mesh_npts; ++dm_i) {
        auto xform = Jtrans_A.rel_transforms[-dm_tar[dm_i]];
        float x = drive_mesh_data.orig_vts(dm_i, 0);
        float y = drive_mesh_data.orig_vts(dm_i, 1);
        float z = drive_mesh_data.orig_vts(dm_i, 2);

        drive_mesh_data.vts(dm_i, 0) = xform(0, 0) * x + xform(0, 1) * y + xform(0, 2) * z + xform(0, 3);
        drive_mesh_data.vts(dm_i, 1) = xform(1, 0) * x + xform(1, 1) * y + xform(1, 2) * z + xform(1, 3);
        drive_mesh_data.vts(dm_i, 2) = xform(2, 0) * x + xform(2, 1) * y + xform(2, 2) * z + xform(2, 3);
    }

    //global r t----------
    const auto& R = g_R[frame];
    const auto g_t = transl_data.row(frame);
    drive_mesh_data.vts = (drive_mesh_data.vts * R.transpose());
    drive_mesh_data.vts.rowwise() += g_t;

    //计算动态法线 ---------
    ComputeVertexNormals(
        drive_mesh_data.vts,
        drive_mesh_data.faces,
        drive_mesh_data.normals
    );

    return;
}
