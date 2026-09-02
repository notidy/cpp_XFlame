#pragma once

#include <Eigen/Dense>
#include <unordered_map>
#include <string>
#include <vector>


using XBinDataMap = std::unordered_map<std::string, std::vector<char>>;
struct XMeshData
{
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> orig_vts;  //原始网格点数据
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> vts;  //驱动后的网格点数据
    Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor> faces;

    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> rest_normals;  //原始网格点法线数据
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> normals;  //驱动后的网格点法线数据
};

class XDataUtil
{
private:


public:
    static inline void SkipSpace(const char*& p)
    {
        while (*p == ' ' || *p == '\t')
            ++p;
    }
    static inline void SkipToNextSpace(const char*& p)
    {
        while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n')
            ++p;
    }
    static inline bool ReadFloat(const char*& p, float& value)
    {
        SkipSpace(p);
        char* end;
        value = strtof(p, &end);
        if (end == p)
            return false;
        p = end;
        return true;
    }
    static inline bool ReadIndex(const char*& p, int& index)
    {
        SkipSpace(p);
        char* end;
        long v = strtol(p, &end, 10);
        if (end == p)
            return false;
        index = (int)v - 1;
        p = end;
        while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n')
            ++p;
        return true;
    }
    
    static bool LoadObj(const void* data, size_t size, XMeshData& out_mesh);
    static bool LoadObj(const std::string& filename, XMeshData& out_mesh);

    static std::vector<char> ReadBinaryFile(const std::string& path);

    static XBinDataMap LoadBin32(const std::string& path);
    static XBinDataMap LoadBin32(const void* data, size_t dataSize);

    template<typename T=float>
    static Eigen::Matrix<T, Eigen::Dynamic, 1> GetBinData(
        const XBinDataMap& data,
        const std::string& name
    )
    {
        auto it = data.find(name);
        if (it == data.end())
            return {};

        const auto& buf = it->second;
        if (buf.size() % sizeof(T) != 0)
            return {}; // 或 throw

        size_t count = buf.size() / sizeof(T);
        Eigen::Matrix<T, Eigen::Dynamic, 1> result(count);
        memcpy(
            result.data(),
            buf.data(),
            buf.size()
        );
        return result;
    }

    template<typename T = float, int _Rows = Eigen::Dynamic, int _Cols = Eigen::Dynamic, int _Options = Eigen::RowMajor>
    static Eigen::Matrix<T, _Rows, _Cols, _Options> GetBinMatrix(const XBinDataMap& data, const std::string& name, int rows, int cols)
    {
        auto it = data.find(name);
        if (it == data.end())
            return {};

        const auto& buf = it->second;
        size_t count = buf.size() / sizeof(T);

        if (count != rows * cols)
            return {};

        return Eigen::Map<const Eigen::Matrix<T, _Rows, _Cols, _Options>>(reinterpret_cast<const T*>(buf.data()), rows, cols);
    }

};

