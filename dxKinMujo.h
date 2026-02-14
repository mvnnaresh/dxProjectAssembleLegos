#pragma once

#include <array>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include <visp3/core/vpHomogeneousMatrix.h>

class dxKinMujo
{
public:
    using PoseResult = std::tuple<vpHomogeneousMatrix, Eigen::Matrix4f, std::vector<double>>;

    explicit dxKinMujo(mjModel* model = nullptr, mjData* data = nullptr);
    ~dxKinMujo();

    dxKinMujo(const dxKinMujo&) = delete;
    dxKinMujo& operator=(const dxKinMujo&) = delete;

    bool setModel(mjModel* model, mjData* data = nullptr);
    void setReferenceData(mjData* data);

    bool setEndEffectorSite(const std::string& siteName);
    const std::string& endEffectorSite() const
    {
        return mEESiteName;
    }

    int getDoF() const;

    bool getFK(const std::vector<double>& qpos, PoseResult& out);
    bool getFKCurrent(PoseResult& out) const;

    bool getIK(const std::vector<double>& seed,
               const std::vector<double>& targetPosQuat,
               std::vector<double>& solution,
               int maxIters = 100,
               double posTol = 1e-4,
               double oriTol = 1e-3,
               double damping = 1e-3);

private:
    bool resolveEndEffectorSite();
    bool ensureScratchData();
    void buildIndices();
    bool applyQpos(const std::vector<double>& qpos, mjData* data) const;
    std::vector<double> extractDofQpos(const mjData* data) const;
    bool computePoseFromData(const mjData* data, PoseResult& out) const;

    mjModel* mModel = nullptr;
    mjData* mDataRef = nullptr;
    mjData* mDataScratch = nullptr;

    std::string mEESiteName;
    int mEESiteId = -1;

    std::vector<int> mJointIds;
    std::vector<int> mQposIndices;
    std::vector<int> mDofIndices;
};
