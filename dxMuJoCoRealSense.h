#pragma once

#include <array>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

class dxMuJoCoRealSense
{
public:
    struct Intrinsics
    {
        int width = 0;
        int height = 0;
        double fx = 0.0;
        double fy = 0.0;
        double cx = 0.0;
        double cy = 0.0;
        double fovyDeg = 0.0;
    };

    struct Frame
    {
        int width = 0;
        int height = 0;
        std::vector<unsigned char> rgb;
        std::vector<float> depth;
    };

    explicit dxMuJoCoRealSense(mjModel* model = nullptr, mjData* data = nullptr);
    ~dxMuJoCoRealSense();

    void setModel(mjModel* model, mjData* data);
    void setCameraName(const std::string& name);
    void setBaseBodyName(const std::string& name);
    void setResolution(int width, int height);
    void setMaxGeom(int maxGeom);
    void setOverrideFovy(double fovyDeg);
    void clearOverrideFovy();

    bool captureRgbDepth(mjvOption* opt, mjrContext* ctx, Frame& frame);

    bool computePointCloudInBase(const std::vector<float>& depthMeters,
                                 std::vector<std::array<float, 3>>& pointsBase);
    bool computePointCloudInCamera(const std::vector<float>& depthMeters,
                                   std::vector<std::array<float, 3>>& pointsCam);

    bool getCameraPoseWorld(double pos[3], double mat[9]) const;
    bool getBasePoseWorld(double pos[3], double mat[9]) const;
    bool getIntrinsics(Intrinsics& out) const;
    const std::string& lastError() const;

private:
    bool ensureScene();
    bool resolveCameraId();
    bool resolveBaseBodyId();
    bool computeIntrinsics(Intrinsics& out) const;
    void updateFixedCamera();

    static void flipVerticalRGB(std::vector<unsigned char>& rgb, int width, int height);
    static void flipVerticalDepth(std::vector<float>& depth, int width, int height);
    static void depthBufferToMeters(std::vector<float>& depth, float znear, float zfar);
    static void rotateColumnMajor(const double* mat, const float in[3], float out[3]);
    static void rotateColumnMajorT(const double* mat, const float in[3], float out[3]);

    mjModel* mModel = nullptr;
    mjData* mData = nullptr;

    std::string mCameraName;
    int mCameraId = -1;

    std::string mBaseBodyName;
    int mBaseBodyId = -1;

    int mWidth = 640;
    int mHeight = 480;
    int mMaxGeom = 4000;

    bool mSceneReady = false;
    bool mUseOverrideFovy = false;
    double mOverrideFovy = 0.0;

    mjvScene mScn;
    mjvCamera mCam;
    std::string mLastError;
};
