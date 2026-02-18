#pragma once

#include <array>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <mujoco/mujoco.h>
#include <visp3/core/vpHomogeneousMatrix.h>

class dxKinMuJoCo
{
public:
    using PoseResult = std::tuple<vpHomogeneousMatrix, Eigen::Matrix4f, std::vector<double>>;

    explicit dxKinMuJoCo(mjModel* model = nullptr, mjData* data = nullptr);

    ~dxKinMuJoCo();

    dxKinMuJoCo(const dxKinMuJoCo&) = delete;

    dxKinMuJoCo& operator=(const dxKinMuJoCo&) = delete;

    bool init();

    int getDoF() const;
    int getNq() const;

    std::vector<double> getDofQpos(const std::vector<double>& qpos) const;

    bool getFK(const std::vector<double>& qpos, PoseResult& out);

    bool getFKCurrent(PoseResult& out) const;

    bool getIK(const std::vector<double>& seed,
               const std::vector<double>& targetPosQuat,
               std::vector<double>& solution,
               int maxIters = 100,
               double posTol = 1e-4,
               double oriTol = 1e-3,
               double damping = 1e-3);
    bool isCollisionFree(const std::vector<double>& qpos, double minDist = 0.0);
    bool buildFullQpos(const std::vector<double>& qpos, std::vector<double>& full) const;
    void printContacts(const std::vector<double>& qpos, int maxContacts = 10);

protected:
    bool setModel(mjModel* model, mjData* data = nullptr);
    void setReferenceData(mjData* data);

private:
    bool resolveEndEffectorBody();
    bool ensureScratchData();
    void buildIndices();
    void buildRobotBodies();
    bool applyQpos(const std::vector<double>& qpos, mjData* data) const;
    std::vector<double> extractDofQpos(const mjData* data) const;
    bool computePoseFromData(const mjData* data, PoseResult& out) const;
    void buildPoseVectorFromData(const mjData* data, std::vector<double>& out) const;

    mjModel* mModel = nullptr;
    mjData* mDataRef = nullptr;
    mjData* mDataScratch = nullptr;

    int mEEBodyId = -1;
    int mEESiteId = -1;

    std::vector<int> mJointIds;
    std::vector<int> mQposIndices;
    std::vector<int> mDofIndices;
    std::vector<char> mRobotBodies;
};
