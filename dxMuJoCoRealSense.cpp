#include "dxMuJoCoRealSense.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace
{
constexpr double kDegToRad = 3.141592653589793 / 180.0;
}

dxMuJoCoRealSense::dxMuJoCoRealSense(mjModel* model, mjData* data)
    : mModel(model), mData(data)
{
    mjv_defaultScene(&mScn);
    mjv_defaultCamera(&mCam);
}

dxMuJoCoRealSense::~dxMuJoCoRealSense()
{
    if (mSceneReady)
    {
        mjv_freeScene(&mScn);
        mSceneReady = false;
    }
}

void dxMuJoCoRealSense::setModel(mjModel* model, mjData* data)
{
    mModel = model;
    mData = data;
    mCameraId = -1;
    mBaseBodyId = -1;
    if (mSceneReady)
    {
        mjv_freeScene(&mScn);
        mSceneReady = false;
    }
}

void dxMuJoCoRealSense::setCameraName(const std::string& name)
{
    mCameraName = name;
    mCameraId = -1;
}

void dxMuJoCoRealSense::setBaseBodyName(const std::string& name)
{
    mBaseBodyName = name;
    mBaseBodyId = -1;
}

void dxMuJoCoRealSense::setResolution(int width, int height)
{
    if (width > 0)
    {
        mWidth = width;
    }
    if (height > 0)
    {
        mHeight = height;
    }
}

void dxMuJoCoRealSense::setMaxGeom(int maxGeom)
{
    if (maxGeom > 0)
    {
        mMaxGeom = maxGeom;
    }
}

void dxMuJoCoRealSense::setOverrideFovy(double fovyDeg)
{
    if (fovyDeg > 0.0)
    {
        mOverrideFovy = fovyDeg;
        mUseOverrideFovy = true;
    }
}

void dxMuJoCoRealSense::clearOverrideFovy()
{
    mUseOverrideFovy = false;
}

bool dxMuJoCoRealSense::captureRgbDepth(mjvOption* opt, mjrContext* ctx, Frame& frame)
{
    mLastError.clear();
    if (!mModel || !mData || !opt || !ctx)
    {
        mLastError = "captureRgbDepth: missing model/data/opt/context.";
        return false;
    }
    if (!ensureScene())
    {
        mLastError = "captureRgbDepth: failed to create scene.";
        return false;
    }
    if (!resolveCameraId())
    {
        mLastError = "captureRgbDepth: camera not found.";
        return false;
    }

    updateFixedCamera();

    mjrRect viewport = { 0, 0, mWidth, mHeight };
    mjv_updateScene(mModel, mData, opt, nullptr, &mCam, mjCAT_ALL, &mScn);
    mjr_render(viewport, &mScn, ctx);

    frame.width = mWidth;
    frame.height = mHeight;
    frame.rgb.assign(static_cast<size_t>(mWidth * mHeight * 3), 0);
    frame.depth.assign(static_cast<size_t>(mWidth * mHeight), 0.0f);

    mjr_readPixels(frame.rgb.data(), frame.depth.data(), viewport, ctx);
    flipVerticalRGB(frame.rgb, mWidth, mHeight);
    flipVerticalDepth(frame.depth, mWidth, mHeight);

    const float extent = static_cast<float>(mModel->stat.extent);
    const float znear = static_cast<float>(mModel->vis.map.znear) * extent;
    const float zfar = static_cast<float>(mModel->vis.map.zfar) * extent;
    depthBufferToMeters(frame.depth, znear, zfar);

    return true;
}

bool dxMuJoCoRealSense::computePointCloudInBase(const std::vector<float>& depthMeters,
                                                std::vector<std::array<float, 3>>& pointsBase)
{
    pointsBase.clear();
    mLastError.clear();
    if (!mModel || !mData)
    {
        mLastError = "computePointCloudInBase: missing model/data.";
        return false;
    }
    if (depthMeters.size() != static_cast<size_t>(mWidth * mHeight))
    {
        mLastError = "computePointCloudInBase: depth size mismatch.";
        return false;
    }
    if (!resolveCameraId() || !resolveBaseBodyId())
    {
        mLastError = "computePointCloudInBase: camera/base not found.";
        return false;
    }

    Intrinsics intr;
    if (!computeIntrinsics(intr))
    {
        mLastError = "computePointCloudInBase: failed to compute intrinsics.";
        return false;
    }

    const double* camPos = mData->cam_xpos + 3 * mCameraId;
    const double* camMat = mData->cam_xmat + 9 * mCameraId;

    const double* basePos = mData->xpos + 3 * mBaseBodyId;
    const double* baseMat = mData->xmat + 9 * mBaseBodyId;

    pointsBase.reserve(depthMeters.size());

    for (int v = 0; v < mHeight; ++v)
    {
        for (int u = 0; u < mWidth; ++u)
        {
            const float z = depthMeters[static_cast<size_t>(v * mWidth + u)];
            if (!std::isfinite(z) || z <= 0.0f)
            {
                continue;
            }

            const float x = static_cast<float>((u - intr.cx) * z / intr.fx);
            const float y = static_cast<float>((v - intr.cy) * z / intr.fy);

            const float pCam[3] = { x, y, z };
            float pWorld[3];
            rotateColumnMajor(camMat, pCam, pWorld);
            pWorld[0] += static_cast<float>(camPos[0]);
            pWorld[1] += static_cast<float>(camPos[1]);
            pWorld[2] += static_cast<float>(camPos[2]);

            float delta[3] =
            {
                static_cast<float>(pWorld[0] - basePos[0]),
                static_cast<float>(pWorld[1] - basePos[1]),
                static_cast<float>(pWorld[2] - basePos[2])
            };

            float pBase[3];
            rotateColumnMajorT(baseMat, delta, pBase);

            pointsBase.push_back({ pBase[0], pBase[1], pBase[2] });
        }
    }

    return !pointsBase.empty();
}

bool dxMuJoCoRealSense::computePointCloudInCamera(const std::vector<float>& depthMeters,
                                                  std::vector<std::array<float, 3>>& pointsCam)
{
    pointsCam.clear();
    mLastError.clear();
    if (depthMeters.size() != static_cast<size_t>(mWidth * mHeight))
    {
        mLastError = "computePointCloudInCamera: depth size mismatch.";
        return false;
    }

    Intrinsics intr;
    if (!computeIntrinsics(intr))
    {
        mLastError = "computePointCloudInCamera: failed to compute intrinsics.";
        return false;
    }

    pointsCam.reserve(depthMeters.size());
    for (int v = 0; v < mHeight; ++v)
    {
        for (int u = 0; u < mWidth; ++u)
        {
            const float z = depthMeters[static_cast<size_t>(v * mWidth + u)];
            if (!std::isfinite(z) || z <= 0.0f)
            {
                continue;
            }

            const float x = static_cast<float>((u - intr.cx) * z / intr.fx);
            const float y = static_cast<float>((v - intr.cy) * z / intr.fy);
            pointsCam.push_back({ x, y, z });
        }
    }

    return !pointsCam.empty();
}

bool dxMuJoCoRealSense::getCameraPoseWorld(double pos[3], double mat[9]) const
{
    if (!mModel || !mData || !pos || !mat)
    {
        return false;
    }
    if (mCameraId < 0 && mCameraName.empty())
    {
        return false;
    }

    const int camId = (mCameraId >= 0) ? mCameraId : mj_name2id(mModel, mjOBJ_CAMERA, mCameraName.c_str());
    if (camId < 0 || camId >= mModel->ncam)
    {
        return false;
    }

    const double* camPos = mData->cam_xpos + 3 * camId;
    const double* camMat = mData->cam_xmat + 9 * camId;

    pos[0] = camPos[0];
    pos[1] = camPos[1];
    pos[2] = camPos[2];
    std::memcpy(mat, camMat, sizeof(double) * 9);
    return true;
}

bool dxMuJoCoRealSense::getBasePoseWorld(double pos[3], double mat[9]) const
{
    if (!mModel || !mData || !pos || !mat)
    {
        return false;
    }
    if (mBaseBodyId < 0 && mBaseBodyName.empty())
    {
        return false;
    }

    const int bodyId = (mBaseBodyId >= 0) ? mBaseBodyId : mj_name2id(mModel, mjOBJ_BODY, mBaseBodyName.c_str());
    if (bodyId < 0 || bodyId >= mModel->nbody)
    {
        return false;
    }

    const double* basePos = mData->xpos + 3 * bodyId;
    const double* baseMat = mData->xmat + 9 * bodyId;

    pos[0] = basePos[0];
    pos[1] = basePos[1];
    pos[2] = basePos[2];
    std::memcpy(mat, baseMat, sizeof(double) * 9);
    return true;
}

bool dxMuJoCoRealSense::getIntrinsics(Intrinsics& out) const
{
    return computeIntrinsics(out);
}

const std::string& dxMuJoCoRealSense::lastError() const
{
    return mLastError;
}

bool dxMuJoCoRealSense::ensureScene()
{
    if (mSceneReady)
    {
        return true;
    }
    if (!mModel)
    {
        mLastError = "ensureScene: missing model.";
        return false;
    }

    mjv_defaultScene(&mScn);
    mjv_makeScene(mModel, &mScn, mMaxGeom);
    mSceneReady = true;
    return true;
}

bool dxMuJoCoRealSense::resolveCameraId()
{
    if (mCameraId >= 0)
    {
        return mCameraId < mModel->ncam;
    }
    if (!mModel)
    {
        mLastError = "resolveCameraId: missing model.";
        return false;
    }
    if (mCameraName.empty())
    {
        if (mModel->ncam <= 0)
        {
            mLastError = "resolveCameraId: model has no cameras.";
            return false;
        }
        mCameraId = 0;
        return true;
    }
    const int id = mj_name2id(mModel, mjOBJ_CAMERA, mCameraName.c_str());
    if (id < 0 || id >= mModel->ncam)
    {
        mLastError = "resolveCameraId: camera name not found.";
        return false;
    }
    mCameraId = id;
    return true;
}

bool dxMuJoCoRealSense::resolveBaseBodyId()
{
    if (mBaseBodyId >= 0)
    {
        return mBaseBodyId < mModel->nbody;
    }
    if (mBaseBodyName.empty() || !mModel)
    {
        mLastError = "resolveBaseBodyId: base body name not set.";
        return false;
    }
    const int id = mj_name2id(mModel, mjOBJ_BODY, mBaseBodyName.c_str());
    if (id < 0 || id >= mModel->nbody)
    {
        mLastError = "resolveBaseBodyId: base body not found.";
        return false;
    }
    mBaseBodyId = id;
    return true;
}

bool dxMuJoCoRealSense::computeIntrinsics(Intrinsics& out) const
{
    if (mWidth <= 0 || mHeight <= 0)
    {
        return false;
    }

    double fovy = 0.0;
    if (mUseOverrideFovy)
    {
        fovy = mOverrideFovy;
    }
    else if (mModel && mCameraId >= 0)
    {
        fovy = mModel->cam_fovy[mCameraId];
    }

    if (fovy <= 0.0 && mModel)
    {
        fovy = mModel->vis.global.fovy;
    }
    if (fovy <= 0.0)
    {
        fovy = 45.0;
    }

    const double fy = 0.5 * static_cast<double>(mHeight) / std::tan(0.5 * fovy * kDegToRad);
    const double fx = fy;

    out.width = mWidth;
    out.height = mHeight;
    out.fx = fx;
    out.fy = fy;
    out.cx = 0.5 * static_cast<double>(mWidth - 1);
    out.cy = 0.5 * static_cast<double>(mHeight - 1);
    out.fovyDeg = fovy;

    return true;
}

void dxMuJoCoRealSense::updateFixedCamera()
{
    mjv_defaultCamera(&mCam);
    mCam.type = mjCAMERA_FIXED;
    mCam.fixedcamid = mCameraId;
}

void dxMuJoCoRealSense::flipVerticalRGB(std::vector<unsigned char>& rgb, int width, int height)
{
    if (width <= 0 || height <= 0)
    {
        return;
    }
    const size_t rowStride = static_cast<size_t>(width * 3);
    std::vector<unsigned char> tmp(rowStride);
    for (int y = 0; y < height / 2; ++y)
    {
        unsigned char* rowTop = rgb.data() + static_cast<size_t>(y) * rowStride;
        unsigned char* rowBot = rgb.data() + static_cast<size_t>(height - 1 - y) * rowStride;
        std::memcpy(tmp.data(), rowTop, rowStride);
        std::memcpy(rowTop, rowBot, rowStride);
        std::memcpy(rowBot, tmp.data(), rowStride);
    }
}

void dxMuJoCoRealSense::flipVerticalDepth(std::vector<float>& depth, int width, int height)
{
    if (width <= 0 || height <= 0)
    {
        return;
    }
    const size_t rowStride = static_cast<size_t>(width);
    std::vector<float> tmp(rowStride);
    for (int y = 0; y < height / 2; ++y)
    {
        float* rowTop = depth.data() + static_cast<size_t>(y) * rowStride;
        float* rowBot = depth.data() + static_cast<size_t>(height - 1 - y) * rowStride;
        std::memcpy(tmp.data(), rowTop, rowStride * sizeof(float));
        std::memcpy(rowTop, rowBot, rowStride * sizeof(float));
        std::memcpy(rowBot, tmp.data(), rowStride * sizeof(float));
    }
}

void dxMuJoCoRealSense::depthBufferToMeters(std::vector<float>& depth, float znear, float zfar)
{
    if (znear <= 0.0f || zfar <= znear)
    {
        return;
    }
    const float scale = 1.0f - znear / zfar;
    for (float& z : depth)
    {
        const float denom = 1.0f - z * scale;
        if (denom > std::numeric_limits<float>::min())
        {
            z = znear / denom;
        }
        else
        {
            z = 0.0f;
        }
    }
}

void dxMuJoCoRealSense::rotateColumnMajor(const double* mat, const float in[3], float out[3])
{
    out[0] = static_cast<float>(mat[0] * in[0] + mat[3] * in[1] + mat[6] * in[2]);
    out[1] = static_cast<float>(mat[1] * in[0] + mat[4] * in[1] + mat[7] * in[2]);
    out[2] = static_cast<float>(mat[2] * in[0] + mat[5] * in[1] + mat[8] * in[2]);
}

void dxMuJoCoRealSense::rotateColumnMajorT(const double* mat, const float in[3], float out[3])
{
    out[0] = static_cast<float>(mat[0] * in[0] + mat[1] * in[1] + mat[2] * in[2]);
    out[1] = static_cast<float>(mat[3] * in[0] + mat[4] * in[1] + mat[5] * in[2]);
    out[2] = static_cast<float>(mat[6] * in[0] + mat[7] * in[1] + mat[8] * in[2]);
}
