#include "XTest.h"
#include "../src/XFlame.h"
#include <chrono>
#include <iostream>


void XTest::test()
{
    std::string data_dir = "D:/Some/Project/Web_3dgs/dataset/0525_xiaoshiguan_teeth_audio/flame_render_engine_data";
    std::string drive_mesh_f_path = data_dir + "/drive_mesh.obj";
    std::string data_info_f_path = data_dir + "/data_info.bin";
    std::string generic_model_f_path = data_dir + "/generic_model.bin";
    std::string flame_track_f_path = data_dir + "/flame_track.bin";

    XMeshData drive_mesh_data;
    XDataUtil::LoadObj(drive_mesh_f_path, drive_mesh_data);
    int drive_mesh_npts = drive_mesh_data.vts.rows();

    // generic_model_data ------------------------------------------------------
    auto generic_model_data = XDataUtil::LoadBin32(generic_model_f_path);
    int flame_npts = generic_model_data["v_template"].size() / 12;

    auto v_template_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(generic_model_data, "v_template", flame_npts, 3);
    auto shapedirs_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 400>(generic_model_data, "shapedirs", flame_npts * 3, 400);
    auto posedirs_data = XDataUtil::GetBinMatrix<float, 36, Eigen::Dynamic, Eigen::ColMajor>(generic_model_data, "posedirs", 36, flame_npts * 3);
    auto weights_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 5>(generic_model_data, "weights", flame_npts, 5);
    auto kintree_table_data = XDataUtil::GetBinMatrix<float, 2, 5>(generic_model_data, "kintree_table", 2, 5);
    auto J_regressor_data = XDataUtil::GetBinMatrix<float, 5, Eigen::Dynamic>(generic_model_data, "J_regressor", 5, flame_npts);

    // flame_track_data ------------------------------------------------------
    auto flame_track_data = XDataUtil::LoadBin32(flame_track_f_path);
    int nframes = flame_track_data["expr"].size() / 400;

    auto shape_data = XDataUtil::GetBinMatrix<float, 1, 300>(flame_track_data, "shape", 1, 300);
    Eigen::Matrix<float, Eigen::Dynamic, 100, Eigen::RowMajor> expr_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 100>(flame_track_data, "expr", nframes, 100);
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> jaw_pose_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(flame_track_data, "jaw_pose", nframes, 3);
    Eigen::Matrix<float, Eigen::Dynamic, 6, Eigen::RowMajor> eye_pose_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 6>(flame_track_data, "eye_pose", nframes, 6);
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> neck_pose_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(flame_track_data, "neck_pose", nframes, 3);
    auto global_orient_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(flame_track_data, "global_orient", nframes, 3);
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> transl_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(flame_track_data, "transl", nframes, 3);

    std::vector<Eigen::Matrix3f> g_R(nframes);
    for (int i = 0; i < nframes; ++i)
        g_R[i] = Rodrigues(global_orient_data.row(i).transpose());

    // data_info_data ------------------------------------------------------
    auto data_info_data = XDataUtil::LoadBin32(data_info_f_path);

    auto drive_mesh_tar_pt_ind_data = XDataUtil::GetBinMatrix<int, Eigen::Dynamic, 1, Eigen::ColMajor>(data_info_data, "drive_mesh_tar_pt_ind", drive_mesh_npts, 1);

    //计算驱动网格中有映射点的点数量（其余牙齿部分使用关节矩阵来变换）
    int drive_mesh_main_pt_count = 0;
    while (drive_mesh_main_pt_count < drive_mesh_npts && drive_mesh_tar_pt_ind_data(drive_mesh_main_pt_count) >= 0)
        ++drive_mesh_main_pt_count;

    //auto gs_pos_offset_raw_data = XDataUtil::GetBinData(data_info_data, "gs_pos_offset");
    //auto gs_pos_offset_data = gs_pos_offset_raw_data.reshaped<Eigen::RowMajor>();
    //auto gs_pos_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(data_info_data, "gs_pos_offset", drive_mesh_npts, 3);

    auto mesh_pos_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(data_info_data, "mesh_pos_offset", drive_mesh_main_pt_count, 3);
    auto expr_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 100>(data_info_data, "expr_offset", nframes, 100);
    auto jaw_pose_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(data_info_data, "jaw_pose_offset", nframes, 3);
    auto eye_pose_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 6>(data_info_data, "eye_pose_offset", nframes, 6);
    auto neck_pose_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(data_info_data, "neck_pose_offset", nframes, 3);
    auto trans_offset_data = XDataUtil::GetBinMatrix<float, Eigen::Dynamic, 3>(data_info_data, "trans_offset", nframes, 3);

    //添加偏移量
    expr_data += expr_offset_data;
    jaw_pose_data += jaw_pose_offset_data;
    eye_pose_data += eye_pose_offset_data;
    neck_pose_data += neck_pose_offset_data;
    transl_data += trans_offset_data;

    //驱动插值网格形变 ---------------------------------------
    int cur_frame = 0;

    //记录起始时间-------------------
    auto start = std::chrono::high_resolution_clock::now();

    for (cur_frame = 0; cur_frame < nframes; ++cur_frame) {
        //lbs --------------------------------
        //calc shape ------------
        Eigen::VectorXf betas(400);
        betas << shape_data.row(0).transpose(), expr_data.row(cur_frame).transpose();
        Eigen::MatrixXf v_shaped = v_template_data + (shapedirs_data * betas).reshaped<Eigen::RowMajor>(flame_npts, 3);
        Eigen::Matrix<float, 5, 3> J = J_regressor_data * v_shaped;

        //calc pose offset ----------
        std::array<Eigen::Matrix3f, 5> rot_mats;
        rot_mats[0] = Eigen::Matrix3f::Identity();
        rot_mats[1] = Rodrigues(neck_pose_data.row(cur_frame).transpose());
        rot_mats[2] = Rodrigues(jaw_pose_data.row(cur_frame).transpose());
        rot_mats[3] = Rodrigues(eye_pose_data.row(cur_frame).head<3>().transpose());
        rot_mats[4] = Rodrigues(eye_pose_data.row(cur_frame).tail<3>().transpose());

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
            float x = drive_mesh_data.vts(dm_i, 0);
            float y = drive_mesh_data.vts(dm_i, 1);
            float z = drive_mesh_data.vts(dm_i, 2);

            drive_mesh_data.vts(dm_i, 0) = xform(0, 0) * x + xform(0, 1) * y + xform(0, 2) * z + xform(0, 3);
            drive_mesh_data.vts(dm_i, 1) = xform(1, 0) * x + xform(1, 1) * y + xform(1, 2) * z + xform(1, 3);
            drive_mesh_data.vts(dm_i, 2) = xform(2, 0) * x + xform(2, 1) * y + xform(2, 2) * z + xform(2, 3);
        }

        //global r t----------
        const auto& R = g_R[cur_frame];
        const auto g_t = transl_data.row(cur_frame);
        drive_mesh_data.vts.noalias() = (drive_mesh_data.vts * R.transpose());
        drive_mesh_data.vts.rowwise() += g_t;

        if (cur_frame % 10 == 0 || cur_frame == nframes - 1)
            std::cout << cur_frame << " over...\n";
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << elapsed.count() << " ms\n";
    std::cout << float(nframes) / elapsed.count() * 1000.0 << " fps\n";

    //std::cout <<v_template_data(0,0) <<shapedirs_data(0,0);
    return;
}

void XTest::test_interface()
{
    //初始化 -------------------------------------
    std::string data_dir = "D:/Some/Project/Web_3dgs/dataset/0525_xiaoshiguan_teeth_audio/flame_render_engine_data";
    std::string generic_model_f_path = data_dir + "/generic_model.bin";
    std::string flame_track_f_path = data_dir + "/flame_track.bin";
    std::string data_info_f_path = data_dir + "/data_info.bin";
    std::string drive_mesh_f_path = data_dir + "/drive_mesh.obj";

    auto generic_model = XDataUtil::ReadBinaryFile(generic_model_f_path);
    auto flame_track = XDataUtil::ReadBinaryFile(flame_track_f_path);
    auto data_info = XDataUtil::ReadBinaryFile(data_info_f_path);
    auto drive_mesh = XDataUtil::ReadBinaryFile(drive_mesh_f_path);

    XFlame xflame;
    if (!xflame.Init(
        generic_model.data(), generic_model.size(),
        flame_track.data(), flame_track.size(),
        data_info.data(), data_info.size(),
        drive_mesh.data(), drive_mesh.size()
    ))
    {
        return;
    }

    //驱动插值网格形变 ---------------------------------------
    int cur_frame = 0;
    auto start = std::chrono::high_resolution_clock::now();
    auto last_time = std::chrono::steady_clock::now();
    auto nframes = xflame.GetNFrames();
    for (cur_frame = 0; cur_frame < nframes; ++cur_frame) {
        xflame.ComputeFrame(cur_frame);

        auto cur_time = std::chrono::steady_clock::now();
        if (std::chrono::duration<float>(cur_time - last_time).count() > 0.05f || cur_frame == nframes - 1)
        {
            printf("\r%.2f%% (%d/%d)", 100.f * (cur_frame + 1) / nframes, cur_frame + 1, nframes);
            last_time = cur_time;
        }
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "\n" << elapsed.count() << " ms\n";
    std::cout << float(nframes) / elapsed.count() * 1000.0 << " fps\n";

}
