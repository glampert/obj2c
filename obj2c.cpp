
// ================================================================================================
// -*- C++ -*-
// File: obj2c.cpp
// Author: Guilherme R. Lampert
// Created on: 04/08/16
//
// Brief: Very basic command line tool that converts Wavefront Object (.obj) 3D mesh
//        files into C/C++ arrays of data that can be directly embedded in source code.
//
// Released under the MIT license. See the accompanying LICENSE file
// or visit <https://opensource.org/licenses/MIT>
// ================================================================================================

// c++ -std=c++11 -Wall -Wextra -Wshadow -pedantic obj2c.cpp -o obj2c

#include <cassert>
#include <cctype>
#include <cerrno>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>
#include <unordered_map>

#include "spatial_sort.hpp"

// ========================================================

//
// Command line options/flags:
//
static struct {
    bool verbose        = false; // -v, --verbose:        Be verbose? If set timings are also printed at the end.
    bool staticArrays   = false; // -s, --static_arrays:  Write arrays/sizes with the 'static' qualifier?
    bool writeCounts    = false; // -c, --write_counts:   Write array lengths as constants?
    bool smoothNormals  = false; // -n, --smooth_normals: Gen smooth per-vertex normals? Default is flat per-face. Requires vb_friendly!
    bool vbFriendly     = false; // -f, --vb_friendly:    Make output Vertex Buffer friendly? That is, single index per-vertex.
    bool noUVs          = false; // --no_uvs:             If set, don't output UVs (texture coordinates).
    bool genIncludeFile = false; // --inc_file:           Generate an include file with name equal to gIncFileName.
    bool stdintData     = false; //                       Use data types from cstdint/stdint.h if vb_friendly is set.
    int  indexDataSize  = 32;    //                       Index data size in bits. 16 or 32. Only relevant if vb_friendly == true.
} gOptions;

//
// Input/output file names:
//
static std::string gSourceFileName; // Name of source .obj file.
static std::string gTargetFileName; // Name of .c/.cpp file to write.
static std::string gIncFileName;    // Optional output include file for when 'genIncludeFile' option is set.
static std::string gCmdlineStr;     // Whole command line into a string for printing in the output file.

//
// Clock to measure the taken to do our work:
//
using Clock    = std::chrono::high_resolution_clock;
using TimeUnit = std::chrono::milliseconds;
static const char * gTimeUnitSuffix = "milliseconds";

//
// OBJ file structures:
//
struct ObjFace
{
    // NOTE: Only triangles are supported right now!
    std::uint32_t vertexIndexes[3];
    std::uint32_t normalIndexes[3];
    std::uint32_t texCoordIndexes[3];
};
struct ObjModel
{
    std::vector<Vec3>          vertexes;
    std::vector<Vec3>          normals;
    std::vector<Vec2>          texCoords;
    std::vector<ObjFace>       faces;
    std::vector<std::uint32_t> indexBuffer; // Only filled if 'vb_friendly' is set.
};

// The object being imported from file.
static ObjModel gObjModel;

// ========================================================

#if defined(__GNUC__) || defined(__clang__)
static bool errorF(const char * fmt, ...)        __attribute__((format(printf, 1, 2)));
static void verbosePrintF(const char * fmt, ...) __attribute__((format(printf, 1, 2)));
#endif // GNU || Clang

static bool errorF(const char * fmt, ...)
{
    std::printf("ERROR: ");

    va_list vaList;
    va_start(vaList, fmt);
    std::vprintf(fmt, vaList);
    va_end(vaList);

    std::printf("\n");
    return false;
}

static void verbosePrintF(const char * fmt, ...)
{
    if (!gOptions.verbose)
    {
        return;
    }

    va_list vaList;
    va_start(vaList, fmt);
    std::vprintf(fmt, vaList);
    va_end(vaList);

    std::printf("\n");
}

static std::size_t maxOfN(const std::size_t first, ...)
{
    va_list vaList;
    std::size_t num = first;
    std::size_t largestNum = 0;

    va_start(vaList, first);
    while (num != std::size_t(~0))
    {
        if (num > largestNum)
        {
            largestNum = num;
        }
        num = va_arg(vaList, std::size_t);
    }
    va_end(vaList);

    return largestNum;
}

static std::uint32_t numberPad(const std::size_t num)
{
    if (num <= 9)         return 1;
    if (num <= 99)        return 2;
    if (num <= 999)       return 3;
    if (num <= 9999)      return 4;
    if (num <= 99999)     return 5;
    if (num <= 999999)    return 6;
    if (num <= 9999999)   return 7;
    if (num <= 99999999)  return 8;
    if (num <= 999999999) return 9;
    return 10;
}

static void printStats(const char * const progName, const TimeUnit timeTaken)
{
    // Print some stats about the program execution
    // just before exiting successfully.

    std::printf("%s ran in %llu %s.\n",
                progName, static_cast<unsigned long long>(timeTaken.count()), gTimeUnitSuffix);

    const std::size_t largestNum = maxOfN(
        gObjModel.vertexes.size(),
        gObjModel.normals.size(),
        gObjModel.texCoords.size(),
        gObjModel.faces.size(),
        std::size_t(~0));

    const std::uint32_t pad = numberPad(largestNum);

    std::printf("Outputted:\n");
    std::printf("%0*zu vertex positions.\n", pad, gObjModel.vertexes.size());
    std::printf("%0*zu vertex normals.\n",   pad, gObjModel.normals.size());
    std::printf("%0*zu texture vertexes.\n", pad, gObjModel.texCoords.size());

    if (gOptions.vbFriendly)
    {
        std::printf("%0*zu indexes\n", pad, gObjModel.indexBuffer.size());
    }
    else
    {
        std::printf("%0*zu faces\n", pad, gObjModel.faces.size());
    }
}

static void printHelpText(const char * const progName)
{
    std::printf(
        "Convert Wavefront OBJ mesh file to C/C++ data arrays.\n\n"
        "Usage:\n"
        "  $ %s <source-file> <target-file> [options]\n"
        "Options:\n"
        "  -h, --help            Shows this help text.\n"
        "  -v, --verbose         Be verbose; output a lot of info and timings.\n"
        "  -s, --static_arrays   If present, add the 'static' qualifier to array declarations.\n"
        "  -c, --write_counts    Write lengths of data arrays as constants.\n"
        "      --inc_file[=name] If flag present, generate an include file externing the array variables.\n"
        "                        Incompatible with 'static_arrays'. If no filename provided, uses the target file name.\n"
        "  -n, --smooth_normals  If set, gen smooth per-vertex normals. Default are shared per-face 'flat' normals.\n"
        "  -f, --vb_friendly     Make the output 'Vertex Buffer friendly'. That is, single index per-vertex.\n"
        "      --ib_type=<type>  Index buffer data type for when using 'vb_friendly'.\n"
        "                        Possible values are 16, 16std, 32 and 32std.\n"
        "                        The 'std' suffix causes the use of the standard C data types found in <stdint.h>\n"
        "      --no_uvs          Don't output mesh UVs, even if they are present in the OBJ file.\n"
        "\n"
        "Created by Guilherme R. Lampert, %s.\n\n",
        progName, __DATE__);
}

static bool parseCommandLine(const int argc, const char * argv[])
{
    // Help run?
    if (argc == 2 && (std::strcmp(argv[1], "-h") == 0 || std::strcmp(argv[1], "--help") == 0))
    {
        printHelpText(argv[0]);
        return false;
    }

    if (argc < 3) // prog_name + source_file + target_file at least
    {
        errorF("Not enough arguments!\n");
        printHelpText(argv[0]);
        return false;
    }

    gSourceFileName = argv[1];
    gTargetFileName = argv[2];

    // Saved for later...
    gCmdlineStr += argv[0]; gCmdlineStr += " ";
    gCmdlineStr += argv[1]; gCmdlineStr += " ";
    gCmdlineStr += argv[2]; gCmdlineStr += " ";

    std::string arg;
    char incFileStr[512]   = {'\0'};
    char ibTypeFlagStr[64] = {'\0'};

    for (int i = 3; i < argc; ++i)
    {
        // Put the command line aside for later:
        gCmdlineStr += argv[i];
        gCmdlineStr += " ";

        // Check for the accepted flags:
        arg = argv[i];
        if (arg == "-v" || arg == "--verbose")
        {
            gOptions.verbose = true;
        }
        else if (arg == "-s" || arg == "--static_arrays")
        {
            gOptions.staticArrays = true;
        }
        else if (arg == "-c" || arg == "--write_counts")
        {
            gOptions.writeCounts = true;
        }
        else if (arg == "-n" || arg == "--smooth_normals")
        {
            gOptions.smoothNormals = true;
        }
        else if (arg == "-f" || arg == "--vb_friendly")
        {
            gOptions.vbFriendly = true;
        }
        else if (arg == "--no_uvs")
        {
            gOptions.noUVs = true;
        }
        else if (arg.compare(0, std::strlen("--inc_file"), "--inc_file") == 0)
        {
            gOptions.genIncludeFile = true;

            if (std::sscanf(arg.c_str(), "--inc_file=%s", incFileStr) == 1)
            {
                gIncFileName = incFileStr;
            }
            else // Use name of target file replacing extension with '.h'
            {
                const auto lastDot = gTargetFileName.find_last_of('.');
                if (lastDot != std::string::npos)
                {
                    gIncFileName = gTargetFileName.substr(0, lastDot);
                }
                else
                {
                    gIncFileName = gTargetFileName;
                }
                gIncFileName += ".h";
            }
        }
        else if (arg.compare(0, std::strlen("--ib_type"), "--ib_type") == 0)
        {
            if (std::sscanf(arg.c_str(), "--ib_type=%s", ibTypeFlagStr) != 1)
            {
                return errorF("Missing value after 'ib_type=...'! Use 16, 16std, 32 or 32std.");
            }

            if (std::strcmp(ibTypeFlagStr, "16") == 0)
            {
                gOptions.indexDataSize = 16;
                gOptions.stdintData    = false;
            }
            else if (std::strcmp(ibTypeFlagStr, "16std") == 0)
            {
                gOptions.indexDataSize = 16;
                gOptions.stdintData    = true;
            }
            else if (std::strcmp(ibTypeFlagStr, "32") == 0)
            {
                gOptions.indexDataSize = 32;
                gOptions.stdintData    = false;
            }
            else if (std::strcmp(ibTypeFlagStr, "32std") == 0)
            {
                gOptions.indexDataSize = 32;
                gOptions.stdintData    = true;
            }
            else
            {
                return errorF("'ib_type' flag value must be equal to 16, 16std, 32 or 32std.");
            }
        }
    }

    if (gOptions.smoothNormals && !gOptions.vbFriendly)
    {
        return errorF("'smooth_normals' flag requires 'vb_friendly'!");
    }
    else if (gOptions.genIncludeFile && gOptions.staticArrays)
    {
        return errorF("'inc_file' option is incompatible with 'static_arrays'!");
    }

    if (gOptions.verbose)
    {
        verbosePrintF("- Source file:         '%s'", gSourceFileName.c_str());
        verbosePrintF("- Target file:         '%s'", gTargetFileName.c_str());
        verbosePrintF("- Include file:        '%s'", gIncFileName.c_str());
        verbosePrintF("- Static arrays?       %s",   (gOptions.staticArrays   ? "yes" : "no"));
        verbosePrintF("- Output array counts? %s",   (gOptions.writeCounts    ? "yes" : "no"));
        verbosePrintF("- VB friendly?         %s",   (gOptions.vbFriendly     ? "yes" : "no"));
        verbosePrintF("- Smooth normals?      %s",   (gOptions.smoothNormals  ? "yes" : "no"));
        verbosePrintF("- Omit UVs?            %s",   (gOptions.noUVs          ? "yes" : "no"));
        verbosePrintF("- Gen include file?    %s",   (gOptions.genIncludeFile ? "yes" : "no"));
        verbosePrintF("- stdint data types?   %s",   (gOptions.stdintData     ? "yes" : "no"));
        verbosePrintF("- Index data size:     %i",   gOptions.indexDataSize);
    }
    return true;
}

static bool openFile(FILE ** outHandle, const char * const filename, const char * const modeStr)
{
    FILE * file;
    errno = 0;

    // fopen_s avoids a deprecation warning for std::fopen on MSVC.
    #ifdef _MSC_VER
    if (fopen_s(&file, filename, modeStr) != 0)
    {
        file = nullptr;
    }
    #else  // !_MSC_VER
    file = std::fopen(filename, modeStr);
    #endif // _MSC_VER

    if (file == nullptr)
    {
        (*outHandle) = nullptr;
        return errorF("Unable to open file \"%s\" with mode '%s' => %s",
                      filename, modeStr, std::strerror(errno));
    }
    else
    {
        (*outHandle) = file;
        return true;
    }
}

static bool allocateObjData(const std::size_t nVertexes,  const std::size_t nNormals,
                            const std::size_t nTexCoords, const std::size_t nFaces)
{
    try
    {
        if (nVertexes != 0)
        {
            gObjModel.vertexes.resize(nVertexes);
            std::memset(gObjModel.vertexes.data(), 0, gObjModel.vertexes.size() * sizeof(Vec3));
            verbosePrintF("- Allocated %zu vertexes.", nVertexes);
        }
        if (nNormals != 0)
        {
            gObjModel.normals.resize(nNormals);
            std::memset(gObjModel.normals.data(), 0, gObjModel.normals.size() * sizeof(Vec3));
            verbosePrintF("- Allocated %zu vertex normals.", nNormals);
        }
        if (nTexCoords != 0)
        {
            gObjModel.texCoords.resize(nTexCoords);
            std::memset(gObjModel.texCoords.data(), 0, gObjModel.texCoords.size() * sizeof(Vec2));
            verbosePrintF("- Allocated %zu texture vertexes.", nTexCoords);
        }
        if (nFaces != 0)
        {
            gObjModel.faces.resize(nFaces);
            std::memset(gObjModel.faces.data(), 0, gObjModel.faces.size() * sizeof(ObjFace));
            verbosePrintF("- Allocated %zu faces.", nFaces);
        }
        return true;
    }
    catch (...)
    {
        return false;
    }
}

static bool readObjTriFace(const char * const buffer, const std::size_t faceIndex)
{
    // FIXME: This could be more robust.
    // Currently if whitespace doesn't match the expected this function might fail.

    assert(faceIndex < gObjModel.faces.size());
    ObjFace & face = gObjModel.faces[faceIndex];
    std::uint32_t v1, t1, n1, v2, t2, n2, v3, t3, n3;

    if (std::sscanf(buffer, "f %u/%u/%u %u/%u/%u %u/%u/%u", &v1, &t1, &n1, &v2, &t2, &n2, &v3, &t3, &n3) == 9)
    {
        // Vertex + Texture + Normal:
        face.vertexIndexes[0]   = --v1;
        face.vertexIndexes[1]   = --v2;
        face.vertexIndexes[2]   = --v3;
        face.normalIndexes[0]   = --n1;
        face.normalIndexes[1]   = --n2;
        face.normalIndexes[2]   = --n3;
        face.texCoordIndexes[0] = --t1;
        face.texCoordIndexes[1] = --t2;
        face.texCoordIndexes[2] = --t3;
        return true;
    }
    else if (std::sscanf(buffer, "f %u/%u %u/%u %u/%u", &v1, &t1, &v2, &t2, &v3, &t3) == 6)
    {
        // Vertex + Texture:
        face.vertexIndexes[0]   = --v1;
        face.vertexIndexes[1]   = --v2;
        face.vertexIndexes[2]   = --v3;
        face.texCoordIndexes[0] = --t1;
        face.texCoordIndexes[1] = --t2;
        face.texCoordIndexes[2] = --t3;
        return true;
    }
    else if (std::sscanf(buffer, "f %u//%u %u//%u %u//%u", &v1, &n1, &v2, &n2, &v3, &n3) == 6)
    {
        // Vertex + Normal:
        face.vertexIndexes[0] = --v1;
        face.vertexIndexes[1] = --v2;
        face.vertexIndexes[2] = --v3;
        face.normalIndexes[0] = --n1;
        face.normalIndexes[1] = --n2;
        face.normalIndexes[2] = --n3;
        return true;
    }
    else if (std::sscanf(buffer, "f %u %u %u", &v1, &v2, &v3) == 3)
    {
        // Vertex Only:
        face.vertexIndexes[0] = --v1;
        face.vertexIndexes[1] = --v2;
        face.vertexIndexes[2] = --v3;
        return true;
    }

    return false;
}

static bool stringToFloat(const char * nPtr, char ** endPtr, float * result)
{
    char * ep = nullptr;
    *result = static_cast<float>(std::strtod(nPtr, &ep));
    *endPtr = ep;
    return (ep != nullptr && ep != nPtr);
}

static bool readVecFloat(const char * const buffer, const int count, float * outFloats)
{
    const char * p = buffer;
    char * endPtr  = nullptr;
    int numbersGot = 0;

    while (p && numbersGot < count)
    {
        if (std::isdigit(*p) || *p == '-' || *p == '+' || *p == '.')
        {
            if (stringToFloat(p, &endPtr, &outFloats[numbersGot]))
            {
                ++numbersGot;
                p = endPtr;
                continue;
            }
        }
        ++p;
    }
    return numbersGot == count;
}

static bool readObjFile()
{
    verbosePrintF("- Preparing to read input OBJ file...");

    FILE * fp;
    if (!openFile(&fp, gSourceFileName.c_str(), "rt"))
    {
        return false;
    }

    std::size_t facesCount     = 0;
    std::size_t vertexCount    = 0;
    std::size_t normalsCount   = 0;
    std::size_t texCoordsCount = 0;
    char line[2048] = {'\0'};

    // First, count all the stuff, so we can allocate exact memory:
    while (!std::feof(fp))
    {
        std::fgets(line, sizeof(line), fp);
        switch (line[0])
        {
        case 'v':
            {
                switch (line[1])
                {
                case ' ':
                case '\t':
                    ++vertexCount;
                    break;
                case 'n':
                    ++normalsCount;
                    break;
                case 't':
                    ++texCoordsCount;
                    break;
                default:
                    break;
                } // switch (line[1])
                break;
            }
        case 'f':
            ++facesCount;
            break;
        default:
            break;
        } // switch (line[0])
    } // while

    // Attempt memory allocation:
    if (!allocateObjData(vertexCount, normalsCount, texCoordsCount, facesCount))
    {
        std::fclose(fp);
        return errorF("Memory allocation failed! OBJ file is too big!");
    }

    // Rewind file for the second pass:
    std::rewind(fp);

    float xyz[3];
    float uv[2];
    std::size_t vIndex = 0;
    std::size_t nIndex = 0;
    std::size_t tIndex = 0;
    std::size_t fIndex = 0;

    // Now read in the stuff:
    while (!std::feof(fp))
    {
        std::fgets(line, sizeof(line), fp);
        switch (line[0])
        {
        case 'v': // Mesh vertex
            {
                // Vertex position "v ":
                if ((line[1] == ' ' || line[1] == '\t') && readVecFloat(line, 3, xyz))
                {
                    gObjModel.vertexes[vIndex].x = xyz[0];
                    gObjModel.vertexes[vIndex].y = xyz[1];
                    gObjModel.vertexes[vIndex].z = xyz[2];
                    ++vIndex;
                }
                // Normal vector "vn":
                else if (line[1] == 'n' && readVecFloat(line, 3, xyz))
                {
                    gObjModel.normals[nIndex].x = xyz[0];
                    gObjModel.normals[nIndex].y = xyz[1];
                    gObjModel.normals[nIndex].z = xyz[2];
                    ++nIndex;
                }
                // Texture coordinate "vt":
                else if (line[1] == 't' && readVecFloat(line, 2, uv))
                {
                    gObjModel.texCoords[tIndex].x = uv[0];
                    gObjModel.texCoords[tIndex].y = uv[1];
                    ++tIndex;
                }
                break;
            }
        case 'f': // Obj face def
            {
                if (readObjTriFace(line, fIndex))
                {
                    ++fIndex;
                }
                else
                {
                    std::printf("WARNING: Unhandled polygon type found. Faces must be triangulated first!\n");
                }
                break;
            }
        default:
            break;
        } // switch (line[0])
    } // while

    std::fclose(fp);

    // Only if we got something wrong, but to be sure...
    assert(fIndex == facesCount);
    assert(vIndex == vertexCount);
    assert(nIndex == normalsCount);
    assert(tIndex == texCoordsCount);

    verbosePrintF("- Done reading input file!");
    return true;
}

//
// Following is used to create a vertex buffer for the vb_friendly option.
//
struct PackedVertex
{
    Vec3 position;
    Vec3 normal;
    Vec2 uv;

    bool operator == (const PackedVertex & other) const
    {
        const bool positionsEq = (position.x == other.position.x &&
                                  position.y == other.position.y &&
                                  position.z == other.position.z);

        const bool normalsEq = (normal.x == other.normal.x &&
                                normal.y == other.normal.y &&
                                normal.z == other.normal.z);

        const bool uvsEq = (uv.x == other.uv.x &&
                            uv.y == other.uv.y);

        return positionsEq && normalsEq && uvsEq;
    }
};

struct PackedVertexHasher
{
    std::size_t operator()(const PackedVertex & packed) const
    {
        constexpr std::size_t count = sizeof(PackedVertex);
        const auto bytes = reinterpret_cast<const std::uint8_t *>(&packed);

        // Simple and fast One-at-a-Time (OAT) hash algorithm:
        //  http://en.wikipedia.org/wiki/Jenkins_hash_function
        //
        std::uint32_t h = 0;
        for (std::size_t i = 0; i < count; ++i)
        {
            h += bytes[i];
            h += (h << 10);
            h ^= (h >> 6);
        }
        h += (h << 3);
        h ^= (h >> 11);
        h += (h << 15);
        return h;
    }
};

using VertToIndexMap = std::unordered_map<PackedVertex, std::uint32_t, PackedVertexHasher>;

static bool findSimilarVertexIndex(const PackedVertex & packed,
                                   const VertToIndexMap & vertexMap,
                                   std::uint32_t * result)
{
    const auto it = vertexMap.find(packed);
    if (it == std::end(vertexMap))
    {
        return false;
    }

    (*result) = it->second;
    return true;
}

static void createIB(const std::vector<PackedVertex> & inVertexes,
                     std::vector<std::uint32_t> & outIndexes,
                     std::vector<Vec3> & outPositions,
                     std::vector<Vec3> & outNormals,
                     std::vector<Vec2> & outUVs)
{
    VertToIndexMap vertexMap;
    const std::size_t inVertexCount = inVertexes.size();

    // Worst case we will have a 1:1 mapping with the vertex count.
    outIndexes.reserve(inVertexCount);
    outPositions.reserve(inVertexCount);
    outNormals.reserve(inVertexCount);
    outUVs.reserve(inVertexCount);

    // For each input vertex:
    for (std::size_t i = 0; i < inVertexCount; ++i)
    {
        const PackedVertex & packed = inVertexes[i];

        // Try to find a similar vertex already in the output:
        std::uint32_t index;
        if (findSimilarVertexIndex(packed, vertexMap, &index))
        {
            // An equivalent vertex is already in the VB, use it instead:
            outIndexes.push_back(index);
        }
        else
        {
            // If not, it needs to be added in the output data:
            outPositions.push_back(inVertexes[i].position);
            outNormals.push_back(inVertexes[i].normal);
            outUVs.push_back(inVertexes[i].uv);

            const auto newIndex = static_cast<std::uint32_t>(outPositions.size() - 1);
            outIndexes.push_back(newIndex);

            // Add it to the map of unique vertexes:
            vertexMap[packed] = newIndex;
        }
    }
}

static bool makeVBFriendly()
{
    verbosePrintF("- Making output data Vertex Buffer friendly (creating an Index Buffer)...");

    try
    {
        bool noNormals   = false;
        bool noTexCoords = false;

        // To make our lives easier in case some data is
        // missing allocate some placeholder arrays:
        if (gObjModel.normals.empty())
        {
            gObjModel.normals.resize(gObjModel.vertexes.size(), { 0.0f, 0.0f, 0.0f });
            noNormals = true;
        }
        if (gObjModel.texCoords.empty())
        {
            gObjModel.texCoords.resize(gObjModel.vertexes.size(), { 0.0f, 0.0f });
            noTexCoords = true;
        }

        std::vector<PackedVertex> packedVerts;
        packedVerts.reserve(gObjModel.faces.size() * 3);

        const std::size_t facesCount = gObjModel.faces.size();
        for (std::size_t fi = 0; fi < facesCount; ++fi)
        {
            const ObjFace & face = gObjModel.faces[fi];
            for (std::size_t vi = 0; vi < 3; ++vi)
            {
                const Vec3 & v  = gObjModel.vertexes[face.vertexIndexes[vi]];
                const Vec3 & vn = gObjModel.normals[face.normalIndexes[vi]];
                const Vec2 & vt = gObjModel.texCoords[face.texCoordIndexes[vi]];

                PackedVertex packed;
                packed.position = v;
                packed.normal   = vn;
                packed.uv       = vt;
                packedVerts.push_back(packed);
            }
        }

        // Discard current data, no longer needed.
        gObjModel.vertexes.clear();
        gObjModel.normals.clear();
        gObjModel.texCoords.clear();
        gObjModel.indexBuffer.clear();

        // Create an index buffer, replacing the old data:
        createIB(packedVerts, gObjModel.indexBuffer, gObjModel.vertexes, gObjModel.normals, gObjModel.texCoords);

        if (noNormals)
        {
            gObjModel.normals.clear();
        }
        if (noTexCoords)
        {
            gObjModel.texCoords.clear();
        }

        return true;
    }
    catch (...)
    {
        return errorF("Unable to create index buffer! Possibly out of memory!");
    }
}

static void computeMeshBounds(const Vec3 * const vertexPositions, const std::size_t vertCount, Vec3 * outMins, Vec3 * outMaxs)
{
    auto minPerElem = [](const Vec3 & vec0, const Vec3 & vec1) -> Vec3
    {
        return { std::min(vec0.x, vec1.x),
                 std::min(vec0.y, vec1.y),
                 std::min(vec0.z, vec1.z) };
    };
    auto maxPerElem = [](const Vec3 & vec0, const Vec3 & vec1) -> Vec3
    {
        return { std::max(vec0.x, vec1.x),
                 std::max(vec0.y, vec1.y),
                 std::max(vec0.z, vec1.z) };
    };

    Vec3 mins = {  INFINITY,  INFINITY,  INFINITY };
    Vec3 maxs = { -INFINITY, -INFINITY, -INFINITY };

    for (std::size_t v = 0; v < vertCount; ++v)
    {
        mins = minPerElem(vertexPositions[v], mins);
        maxs = maxPerElem(vertexPositions[v], maxs);
    }

    (*outMins) = mins;
    (*outMaxs) = maxs;
}

static float computePositionEpsilon(const Vec3 * const vertexPositions, const std::size_t vertCount)
{
    constexpr float epsilon = 1e-4f;

    // Calculate the position bounds so we have a reliable
    // epsilon to check position differences against.
    Vec3 minVec, maxVec;
    computeMeshBounds(vertexPositions, vertCount, &minVec, &maxVec);

    const float dx = maxVec.x - minVec.x;
    const float dy = maxVec.y - minVec.y;
    const float dz = maxVec.z - minVec.z;
    return std::sqrt((dx * dx) + (dy * dy) + (dz * dz)) * epsilon;
}

static void computeFaceNormals(PackedVertex * inOutVertexes, const std::uint32_t * const indexes, const std::size_t indexCount)
{
    if ((indexCount % 3) != 0)
    {
        errorF("Expected triangles! 'smooth_normals' option might not work...");
    }

    // For every triangle...
    for (std::size_t i = 0; i < indexCount; i += 3)
    {
        const std::uint32_t idx0 = indexes[i + 0];
        const std::uint32_t idx1 = indexes[i + 1];
        const std::uint32_t idx2 = indexes[i + 2];

        const Vec3 v0 = inOutVertexes[idx0].position;
        const Vec3 v1 = inOutVertexes[idx1].position;
        const Vec3 v2 = inOutVertexes[idx2].position;

        Vec3 a;
        a.x = v1.x - v0.x;
        a.y = v1.y - v0.y;
        a.z = v1.z - v0.z;

        Vec3 b;
        b.x = v2.x - v0.x;
        b.y = v2.y - v0.y;
        b.z = v2.z - v0.z;

        // Cross product and normalize:
        Vec3 normal;
        normal.x = (a.y * b.z) - (a.z * b.y);
        normal.y = (a.z * b.x) - (a.x * b.z);
        normal.z = (a.x * b.y) - (a.y * b.x);

        const float len = 1.0f / std::sqrt((normal.x * normal.x) + (normal.y * normal.y) + (normal.z * normal.z));
        normal.x *= len;
        normal.y *= len;
        normal.z *= len;

        inOutVertexes[idx0].normal = normal;
        inOutVertexes[idx1].normal = normal;
        inOutVertexes[idx2].normal = normal;
    }
}

static void computeSmoothNormals(PackedVertex * inOutVertexes, const std::size_t vertCount,
                                 const std::uint32_t * const indexes, const std::size_t indexCount,
                                 const float posEpsilon, const int maxAngleDegs)
{
    //
    // This implementation is largely based on code found in the ASSIMP mesh importer library.
    // http://www.assimp.org/
    //

    std::vector<std::uint32_t> vertsFound;
    std::vector<Vec3>          newNormals;
    SpatialSort                vertexFinder;

    // Compute per-face normals but store them per-vertex:
    computeFaceNormals(inOutVertexes, indexes, indexCount);

    // Set up a SpatialSort to quickly find all vertexes close to a given position.
    vertexFinder.fillStructured(inOutVertexes, vertCount, true);
    newNormals.resize(vertCount, { 0.0f, 0.0f, 0.0f });

    if (maxAngleDegs >= 175.0f)
    {
        // There is no angle limit. Thus all vertexes with positions close
        // to each other will receive the same vertex normal. This allows us
        // to optimize the whole algorithm a little bit.
        std::vector<bool> had(vertCount, false);
        for (std::size_t i = 0; i < vertCount; ++i)
        {
            if (had[i])
            {
                continue;
            }

            // Get all vertexes that share this one:
            vertexFinder.findPositions(inOutVertexes[i].position, posEpsilon, &vertsFound);
            const std::size_t numVertexesFound = vertsFound.size();

            Vec3 normal;
            for (std::size_t a = 0; a < numVertexesFound; ++a)
            {
                const Vec3 v = inOutVertexes[vertsFound[a]].normal;
                if (!std::isnan(v.x) && !std::isnan(v.y) && !std::isnan(v.z))
                {
                    normal.x += v.x;
                    normal.y += v.y;
                    normal.z += v.z;
                }
            }

            const float len = 1.0f / std::sqrt((normal.x * normal.x) + (normal.y * normal.y) + (normal.z * normal.z));
            normal.x *= len;
            normal.y *= len;
            normal.z *= len;

            // Write the smoothed normal back to all affected normals:
            for (std::size_t a = 0; a < numVertexesFound; ++a)
            {
                const auto idx  = vertsFound[a];
                newNormals[idx] = normal;
                had[idx] = true;
            }
        }
    }
    else
    {
        // Slower code path if a smooth angle is set. There are many ways to achieve
        // the effect, this one is the most straightforward one.
        const float fLimit = std::cos(maxAngleDegs * (M_PI / 180.0));

        for (std::size_t i = 0; i < vertCount; ++i)
        {
            // Get all vertexes that share this one:
            vertexFinder.findPositions(inOutVertexes[i].position, posEpsilon, &vertsFound);

            Vec3 normal;
            for (std::size_t a = 0; a < vertsFound.size(); ++a)
            {
                const Vec3 v  = inOutVertexes[vertsFound[a]].normal;
                const Vec3 vi = inOutVertexes[i].normal;
                const float d = (v.x * vi.x) + (v.y * vi.y) + (v.z * vi.z);

                if (std::isnan(d) || d < fLimit)
                {
                    continue;
                }

                normal.x += v.x;
                normal.y += v.y;
                normal.z += v.z;
            }

            const float len = 1.0f / std::sqrt((normal.x * normal.x) + (normal.y * normal.y) + (normal.z * normal.z));
            normal.x *= len;
            normal.y *= len;
            normal.z *= len;
            newNormals[i] = normal;
        }
    }

    // Copy new normals to input:
    for (std::size_t i = 0; i < vertCount; ++i)
    {
        inOutVertexes[i].normal = newNormals[i];
    }
}

static void genSmoothNormals()
{
    if (gObjModel.normals.size() < gObjModel.vertexes.size())
    {
        gObjModel.normals.resize(gObjModel.vertexes.size(), { 0.0f, 0.0f, 0.0f });
    }

    const float posEpsilon = computePositionEpsilon(gObjModel.vertexes.data(), gObjModel.vertexes.size());
    verbosePrintF("- Mesh pos epsilon for normal smoothing: %f", posEpsilon);

    PackedVertex pv{};
    std::vector<PackedVertex> packedVerts;
    packedVerts.reserve(gObjModel.vertexes.size());

    for (std::size_t i = 0; i < gObjModel.vertexes.size(); ++i)
    {
        pv.position = gObjModel.vertexes[i];
        pv.normal   = gObjModel.normals[i];
        // UVs are not required for normal computation.
        packedVerts.push_back(pv);
    }

    computeSmoothNormals(
        /* inOutVertexes  = */ packedVerts.data(),
        /* vertCount      = */ packedVerts.size(),
        /* indexes        = */ gObjModel.indexBuffer.data(),
        /* indexCount     = */ gObjModel.indexBuffer.size(),
        /* posEpsilon     = */ posEpsilon,
        /* maxAngleDegs   = */ 175.0f); // TODO should the angle be configurable?

    // Replace current flat normals with the smoothed ones:
    assert(gObjModel.normals.size() == packedVerts.size());
    for (std::size_t i = 0; i < gObjModel.normals.size(); ++i)
    {
        gObjModel.normals[i] = packedVerts[i].normal;
    }

    verbosePrintF("- Smooth vertex normals computed for the model...");
}

static bool isIntegerNumStr(const char * str)
{
    for (; *str != '\0'; ++str)
    {
        if (*str == '.')
        {
            return false; // If the string has a dot, it is a decimal number.
        }
    }
    return true;
}

static const char * strFloat(const float value)
{
    static char str[128];
    std::snprintf(str, sizeof(str), "%f", value);

    // Trim trailing zeros:
    for (char * ptr = str; *ptr != '\0'; ++ptr)
    {
        if (*ptr == '.')
        {
            while (*++ptr) // Find the end of the string.
            {
            }
            while (*--ptr == '0') // Remove trailing zeros.
            {
                *ptr = '\0';
            }
            if (*ptr == '.') // If the dot was left alone at the end, remove it too.
            {
                *ptr = '\0';
            }
            break;
        }
    }
    return str;
}

static const char * strVecFloat(const float * const vec, const int elementCount)
{
    assert(elementCount <= 4);

    static char str[512];
    const char * fval[4] = { nullptr, nullptr, nullptr, nullptr };

    std::memset(str, 0, sizeof(str));
    for (int i = 0; i < elementCount; ++i)
    {
        if (vec[i] == 0.0f)
        {
            std::strcat(str, "0.0f");
        }
        else
        {
            fval[i] = strFloat(vec[i]);
            std::strcat(str, fval[i]);

            if (isIntegerNumStr(fval[i]))
            {
                std::strcat(str, ".0f");
            }
            else
            {
                std::strcat(str, "f");
            }
        }

        if (i != (elementCount - 1))
        {
            std::strcat(str, ", ");
        }
    }

    return str;
}

static void trimString(std::string * s)
{
    // LTrim
    const auto firstNonBlank = s->find_first_not_of(" \t\r\n\v\f");
    s->erase(0, firstNonBlank);

    // RTrim
    const auto lastNonBlank = s->find_last_not_of(" \t\r\n\v\f");
    s->erase(lastNonBlank != std::string::npos ? lastNonBlank + 1 : 0);
}

static const char * getIBTypeStr()
{
    bool isCFile;
    const auto lastDot = gTargetFileName.find_last_of('.');

    if (lastDot != std::string::npos)
    {
        const std::string ext = gTargetFileName.substr(lastDot);
        isCFile = (ext == ".c" || ext == ".C");
    }
    else
    {
        isCFile = false;
    }

    if (gOptions.indexDataSize == 16)
    {
        if (isCFile) { return gOptions.stdintData ? "uint16_t"      : "unsigned short"; }
        else         { return gOptions.stdintData ? "std::uint16_t" : "unsigned short"; }
    }
    else // indexDataSize == 32
    {
        if (isCFile) { return gOptions.stdintData ? "uint32_t"      : "unsigned int"; }
        else         { return gOptions.stdintData ? "std::uint32_t" : "unsigned int"; }
    }
}

static void assembleName(std::string * outName, const std::string & prefix, const char * const suffix)
{
    // Example:
    // in:  prefix="CubeLowPoly" / suffix="Verts"
    // out: "cubeLowPolyVerts"

    (*outName) = prefix;
    (*outName) += suffix;
    (*outName)[0] = std::tolower((*outName)[0]);
    trimString(outName);
}

static std::string baseName(std::string filename)
{
    const auto lastSlash = filename.find_last_of("\\/");
    if (lastSlash != std::string::npos)
    {
        filename = filename.substr(lastSlash + 1);
    }

    const auto lastDot = filename.find_last_of('.');
    if (lastDot != std::string::npos)
    {
        filename = filename.substr(0, lastDot);
    }

    return filename;
}

static bool writeIncludeFile(const std::string & externDecls)
{
    FILE * fp;
    if (!openFile(&fp, gIncFileName.c_str(), "wt"))
    {
        return false;
    }

    std::string headerGuardName = gIncFileName;

    // Replace dots with underscores:
    std::string::size_type dot;
    while ((dot = headerGuardName.find_last_of('.')) != std::string::npos)
    {
        headerGuardName[dot] = '_';
    }

    // Make it all uppercase for a C-style macro name:
    for (char & ch : headerGuardName)
    {
        ch = std::toupper(ch);
    }

    std::fprintf(fp, "\n");
    std::fprintf(fp, "#ifndef %s\n", headerGuardName.c_str());
    std::fprintf(fp, "#define %s\n", headerGuardName.c_str());
    std::fprintf(fp, "\n// File automatically generated by obj2c.\n\n");

    if (gOptions.vbFriendly && gOptions.stdintData)
    {
        std::fprintf(fp, "#ifdef __cplusplus\n"
                         "    #include <cstdint>\n"
                         "#else // !__cplusplus\n"
                         "    #include <stdint.h>\n"
                         "#endif // __cplusplus\n\n");
    }

    std::fprintf(fp, "%s", externDecls.c_str());
    std::fprintf(fp, "#endif // %s\n", headerGuardName.c_str());

    std::fclose(fp);
    verbosePrintF("- Optional include file successfully written!");
    return true;
}

static void writeFileHeader(FILE * fp)
{
    std::fprintf(fp, "\n");
    std::fprintf(fp, "//\n");
    std::fprintf(fp, "// File automatically generated by obj2c from command line:\n");
    std::fprintf(fp, "//  %s\n", gCmdlineStr.c_str());
    std::fprintf(fp, "//\n");
    std::fprintf(fp, "\n");

    if (gOptions.vbFriendly && gOptions.stdintData)
    {
        std::fprintf(fp, "#ifdef __cplusplus\n"
                         "    #include <cstdint>\n"
                         "#else // !__cplusplus\n"
                         "    #include <stdint.h>\n"
                         "#endif // __cplusplus\n\n");
    }
}

static bool writeOutputFiles()
{
    verbosePrintF("- Preparing to write output file(s)...");

    FILE * fp;
    std::string externDecls;
    std::string arrayName;

    if (!openFile(&fp, gTargetFileName.c_str(), "wt"))
    {
        return false;
    }

    writeFileHeader(fp);

    // Get just the base name of the output file without extension or path.
    // This will be used to name the data arrays.
    const std::string filename = baseName(gTargetFileName);

    // Vertex positions: -----------------------------------------------------------
    if (!gObjModel.vertexes.empty())
    {
        assembleName(&arrayName, filename, "Verts");
        verbosePrintF("- Outputting vertex array '%s'...", arrayName.c_str());

        if (gOptions.staticArrays)
        {
            std::fprintf(fp, "static const float %s[][3] = {\n", arrayName.c_str());
        }
        else
        {
            std::fprintf(fp, "const float %s[][3] = {\n", arrayName.c_str());

            externDecls += "extern const float " + arrayName + "[][3];\n";
            if (gOptions.writeCounts)
            {
                externDecls += "extern const int " + arrayName + "Count;\n\n";
            }
        }

        const std::size_t vertCount = gObjModel.vertexes.size();
        for (std::size_t i = 0; i < vertCount; ++i)
        {
            std::fprintf(fp, "  { %s }", strVecFloat(reinterpret_cast<const float *>(&gObjModel.vertexes[i]), 3));
            if (i != (vertCount - 1))
            {
                std::fprintf(fp, ",\n");
            }
        }
        std::fprintf(fp, "\n};\n");

        if (gOptions.writeCounts)
        {
            if (gOptions.staticArrays)
            {
                std::fprintf(fp, "static const int %s = %zu;\n", (arrayName + "Count").c_str(), vertCount);
            }
            else
            {
                std::fprintf(fp, "const int %s = %zu;\n", (arrayName + "Count").c_str(), vertCount);
            }
        }
        std::fprintf(fp, "\n");
    }

    // Vertex normals: -------------------------------------------------------------
    if (!gObjModel.normals.empty())
    {
        assembleName(&arrayName, filename, "Normals");
        verbosePrintF("- Outputting normals array '%s'...", arrayName.c_str());

        if (gOptions.staticArrays)
        {
            std::fprintf(fp, "static const float %s[][3] = {\n", arrayName.c_str());
        }
        else
        {
            std::fprintf(fp, "const float %s[][3] = {\n", arrayName.c_str());

            externDecls += "extern const float " + arrayName + "[][3];\n";
            if (gOptions.writeCounts)
            {
                externDecls += "extern const int " + arrayName + "Count;\n\n";
            }
        }

        const std::size_t normalsCount = gObjModel.normals.size();
        for (std::size_t i = 0; i < normalsCount; ++i)
        {
            std::fprintf(fp, "  { %s }", strVecFloat(reinterpret_cast<const float *>(&gObjModel.normals[i]), 3));
            if (i != (normalsCount - 1))
            {
                std::fprintf(fp, ",\n");
            }
        }
        std::fprintf(fp, "\n};\n");

        if (gOptions.writeCounts)
        {
            if (gOptions.staticArrays)
            {
                std::fprintf(fp, "static const int %s = %zu;\n", (arrayName + "Count").c_str(), normalsCount);
            }
            else
            {
                std::fprintf(fp, "const int %s = %zu;\n", (arrayName + "Count").c_str(), normalsCount);
            }
        }
        std::fprintf(fp, "\n");
    }

    // Texture coordinates: --------------------------------------------------------
    if (!gObjModel.texCoords.empty() && !gOptions.noUVs)
    {
        assembleName(&arrayName, filename, "TexCoords");
        verbosePrintF("- Outputting tex coords array '%s'...", arrayName.c_str());

        if (gOptions.staticArrays)
        {
            std::fprintf(fp, "static const float %s[][2] = {\n", arrayName.c_str());
        }
        else
        {
            std::fprintf(fp, "const float %s[][2] = {\n", arrayName.c_str());

            externDecls += "extern const float " + arrayName + "[][2];\n";
            if (gOptions.writeCounts)
            {
                externDecls += "extern const int " + arrayName + "Count;\n\n";
            }
        }

        const std::size_t texCoordsCount = gObjModel.texCoords.size();
        for (std::size_t i = 0; i < texCoordsCount; ++i)
        {
            std::fprintf(fp, "  { %s }", strVecFloat(reinterpret_cast<const float *>(&gObjModel.texCoords[i]), 2));
            if (i != (texCoordsCount - 1))
            {
                std::fprintf(fp, ",\n");
            }
        }
        std::fprintf(fp, "\n};\n");

        if (gOptions.writeCounts)
        {
            if (gOptions.staticArrays)
            {
                std::fprintf(fp, "static const int %s = %zu;\n", (arrayName + "Count").c_str(), texCoordsCount);
            }
            else
            {
                std::fprintf(fp, "const int %s = %zu;\n", (arrayName + "Count").c_str(), texCoordsCount);
            }
        }
        std::fprintf(fp, "\n");
    }

    // Object faces: ---------------------------------------------------------------
    if (!gObjModel.faces.empty() && !gOptions.vbFriendly)
    {
        assembleName(&arrayName, filename, "FaceIndexes");
        verbosePrintF("- Outputting OBJ face indexes array '%s'...", arrayName.c_str());

        if (gOptions.staticArrays)
        {
            std::fprintf(fp, "static const unsigned int %s[][3][3] = {\n", arrayName.c_str());
        }
        else
        {
            std::fprintf(fp, "const unsigned int %s[][3][3] = {\n", arrayName.c_str());

            externDecls += "extern const unsigned int " + arrayName + "[][3][3];\n";
            if (gOptions.writeCounts)
            {
                externDecls += "extern const int " + arrayName + "Count;\n\n";
            }
        }

        const std::size_t facesCount = gObjModel.faces.size();
        for (std::size_t i = 0; i < facesCount; ++i)
        {
            const ObjFace & face = gObjModel.faces[i];
            std::fprintf(fp, "  { { %u, %u, %u }, { %u, %u, %u }, { %u, %u, %u } }",
                         face.vertexIndexes[0],   face.vertexIndexes[1],   face.vertexIndexes[2],
                         face.normalIndexes[0],   face.normalIndexes[1],   face.normalIndexes[2],
                         face.texCoordIndexes[0], face.texCoordIndexes[1], face.texCoordIndexes[2]);

            if (i != (facesCount - 1))
            {
                std::fprintf(fp, ",\n");
            }
        }
        std::fprintf(fp, "\n};\n");

        if (gOptions.writeCounts)
        {
            if (gOptions.staticArrays)
            {
                std::fprintf(fp, "static const int %s = %zu;\n", (arrayName + "Count").c_str(), facesCount);
            }
            else
            {
                std::fprintf(fp, "const int %s = %zu;\n", (arrayName + "Count").c_str(), facesCount);
            }
        }
        std::fprintf(fp, "\n");
    }

    // Index buffer: ---------------------------------------------------------------
    if (!gObjModel.indexBuffer.empty())
    {
        assert(gOptions.vbFriendly == true);

        assembleName(&arrayName, filename, "Indexes");
        verbosePrintF("- Outputting index array/buffer '%s'...", arrayName.c_str());

        if (gOptions.staticArrays)
        {
            std::fprintf(fp, "static const %s %s[] = {\n", getIBTypeStr(), arrayName.c_str());
        }
        else
        {
            std::fprintf(fp, "const %s %s[] = {\n", getIBTypeStr(), arrayName.c_str());

            externDecls += "extern const " + std::string(getIBTypeStr()) + " " + arrayName + "[];\n";
            if (gOptions.writeCounts)
            {
                externDecls += "extern const int " + arrayName + "Count;\n\n";
            }
        }

        const std::size_t indexesCount = gObjModel.indexBuffer.size();
        for (std::size_t i = 0; i < indexesCount; ++i)
        {
            std::fprintf(fp, "  %u", gObjModel.indexBuffer[i]);
            if (i != (indexesCount - 1))
            {
                std::fprintf(fp, ",\n");
            }
        }
        std::fprintf(fp, "\n};\n");

        if (gOptions.writeCounts)
        {
            if (gOptions.staticArrays)
            {
                std::fprintf(fp, "static const int %s = %zu;\n", (arrayName + "Count").c_str(), indexesCount);
            }
            else
            {
                std::fprintf(fp, "const int %s = %zu;\n", (arrayName + "Count").c_str(), indexesCount);
            }
        }
        std::fprintf(fp, "\n");
    }

    std::fclose(fp);
    verbosePrintF("- Done writing output file!");

    // Write the optional include file: --------------------------------------------
    if (gOptions.genIncludeFile)
    {
        if (!writeIncludeFile(externDecls))
        {
            return false;
        }
    }

    return true;
}

// ========================================================

int main(const int argc, const char * argv[])
{
    try
    {
        const auto startTime = Clock::now();

        if (!parseCommandLine(argc, argv))
        {
            return EXIT_FAILURE;
        }

        if (!readObjFile())
        {
            return EXIT_FAILURE;
        }

        if (gOptions.vbFriendly && !makeVBFriendly())
        {
            return EXIT_FAILURE;
        }

        if (gOptions.smoothNormals && gOptions.vbFriendly)
        {
            genSmoothNormals();
        }

        if (!writeOutputFiles())
        {
            return EXIT_FAILURE;
        }

        const auto endTime = Clock::now();
        const TimeUnit timeTaken = std::chrono::duration_cast<TimeUnit>(endTime - startTime);

        if (gOptions.verbose)
        {
            printStats(argv[0], timeTaken);
        }

        return EXIT_SUCCESS;
    }
    catch (...)
    {
        errorF("Ooops! Looks like we hit a bump on the road... Try running it again ;)");
        return EXIT_FAILURE;
    }
}
