#include "XDataUtil.h"
#include <fstream>


bool XDataUtil::LoadObj(const void* data, size_t size, XMeshData& out_mesh)
{
    const char* begin = (const char*)data;
    const char* end = begin + size;

    //----------------------------------------
    // pass1
    //----------------------------------------

    size_t vertexCount = 0;
    size_t faceCount = 0;

    const char* p = begin;

    while (p < end)
    {
        // 跳过行首空格和 Tab
        while (p < end && (*p == ' ' || *p == '\t'))
            ++p;

        if (p[0] == 'v' && p[1] == ' ')
            ++vertexCount;
        else if (p[0] == 'f' && p[1] == ' ')
            ++faceCount;

        while (p < end && *p != '\n')
            ++p;

        if (p < end)
            ++p;
    }

    //----------------------------------------

    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor>
        vertices(vertexCount, 3);

    Eigen::Matrix<int, Eigen::Dynamic, 3, Eigen::RowMajor>
        faces(faceCount, 3);

    //----------------------------------------
    // pass2
    //----------------------------------------

    p = begin;

    size_t v = 0;
    size_t f = 0;

    while (p < end)
    {
        // 跳过行首空格和 Tab
        while (p < end && (*p == ' ' || *p == '\t'))
            ++p;

        if (p[0] == 'v' && p[1] == ' ')
        {
            p += 2;

            float x, y, z;

            if (!ReadFloat(p, x)) return false;
            if (!ReadFloat(p, y)) return false;
            if (!ReadFloat(p, z)) return false;

            vertices(v, 0) = x;
            vertices(v, 1) = y;
            vertices(v, 2) = z;

            ++v;
        }
        else if (p[0] == 'f' && p[1] == ' ')
        {
            p += 2;

            int a, b, c;

            if (!ReadIndex(p, a)) return false;
            if (!ReadIndex(p, b)) return false;
            if (!ReadIndex(p, c)) return false;

            faces(f, 0) = a;
            faces(f, 1) = b;
            faces(f, 2) = c;

            ++f;
        }

        while (p < end && *p != '\n')
            ++p;

        if (p < end)
            ++p;
    }

    out_mesh.orig_vts = vertices;
    out_mesh.vts = std::move(vertices);
    out_mesh.faces = std::move(faces);

    return true;
}

bool XDataUtil::LoadObj(const std::string& filename, XMeshData& out_mesh)
{
    std::ifstream file(filename, std::ios::binary);

    if (!file)
        return false;

    file.seekg(0, std::ios::end);
    size_t size = (size_t)file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    file.read(buffer.data(), size);
    return LoadObj(buffer.data(), size, out_mesh);
}

std::vector<char> XDataUtil::ReadBinaryFile(const std::string& path)
{
    std::ifstream file(path, std::ios::binary | std::ios::ate);

    if (!file)
        return {};

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<char> buffer(size);
    if (!file.read(buffer.data(), size))
        return {};

    return buffer;
}

XBinDataMap XDataUtil::LoadBin32(const std::string& path)
{
    std::ifstream file(
        path,
        std::ios::binary | std::ios::ate
    );

    if (!file)
        return {};

    size_t size = file.tellg();

    file.seekg(
        0,
        std::ios::beg
    );

    std::vector<char> buffer(
        size
    );

    file.read(
        buffer.data(),
        size
    );

    return LoadBin32(
        buffer.data(),
        size
    );
}

XBinDataMap XDataUtil::LoadBin32(const void* data, size_t dataSize)
{
    XBinDataMap datas;

    const char* ptr =
        (const char*)data;

    const char* end =
        ptr + dataSize;

#define READ(dst, size)                     \
    if (ptr + size > end) return {};        \
    memcpy(dst, ptr, size);                 \
    ptr += size;

    //--------------------------------

    char magic[7];

    READ(
        magic,
        7
    );

    if (
        memcmp(
            magic,
            "NPZB32\0",
            7
        ) != 0
        )
        return {};

    uint32_t version;
    uint32_t count;

    READ(
        &version,
        4
    );

    READ(
        &count,
        4
    );

    struct Meta
    {
        std::string name;
        uint64_t size;
    };

    std::vector<Meta> metas;

    metas.reserve(
        count
    );

    //--------------------------------

    for (
        uint32_t i = 0;
        i < count;
        ++i
        )
    {
        uint32_t nameLen;

        READ(
            &nameLen,
            4
        );

        if (
            ptr + nameLen > end
            )
            return {};

        std::string name(
            ptr,
            nameLen
        );

        ptr += nameLen;

        uint64_t size;

        READ(
            &size,
            8
        );

        metas.push_back(
            {
                std::move(name),
                size
            }
        );
    }

    //--------------------------------

    for (
        auto& m :
        metas
        )
    {
        if (
            ptr + m.size > end
            )
            return {};

        std::vector<char> cur(
            m.size
        );

        memcpy(
            cur.data(),
            ptr,
            m.size
        );

        ptr += m.size;

        datas.emplace(
            m.name,
            std::move(cur)
        );
    }

#undef READ

    return datas;
}
