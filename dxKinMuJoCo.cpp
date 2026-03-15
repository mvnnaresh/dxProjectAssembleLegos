#include "dxKinMuJoCo.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace
{
constexpr double kEps = 1e-9;

double clamp_double(double v, double lo, double hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

void normalize_quat(double q[4])
{
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n < kEps)
    {
        return;
    }
    q[0] /= n;
    q[1] /= n;
    q[2] /= n;
    q[3] /= n;
}

void quat_conjugate(const double q[4], double out[4])
{
    out[0] = q[0];
    out[1] = -q[1];
    out[2] = -q[2];
    out[3] = -q[3];
}

void quat_multiply(const double a[4], const double b[4], double out[4])
{
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}
}

dxKinMuJoCo::dxKinMuJoCo(mjModel* model, mjData* data)
    : mModel(model), mDataRef(data)
{
    init();
}

dxKinMuJoCo::~dxKinMuJoCo()
{
    if (mDataScratch)
    {
        mj_deleteData(mDataScratch);
        mDataScratch = nullptr;
    }
}

bool dxKinMuJoCo::setModel(mjModel* model, mjData* data)
{
    if (mDataScratch)
    {
        mj_deleteData(mDataScratch);
        mDataScratch = nullptr;
    }

    mModel = model;
    mDataRef = data;
    mEEBodyId = -1;
    mEESiteId = -1;
    mUseMidpoint = false;
    mEEMidBodyA = -1;
    mEEMidBodyB = -1;
    mEEMidOrientBody = -1;
    mJointIds.clear();
    mQposIndices.clear();
    mDofIndices.clear();
    mRobotBodies.clear();

    if (!mModel)
    {
        return false;
    }

    buildIndices();
    buildRobotBodies();
    resolveEndEffectorBody();
    return ensureScratchData() && (mEEBodyId >= 0);
}

bool dxKinMuJoCo::init()
{
    if (!mModel)
    {
        return false;
    }
    if (!setModel(mModel, mDataRef))
    {
        return false;
    }
    if (mDataRef)
    {
        setReferenceData(mDataRef);
    }
    return true;
}

void dxKinMuJoCo::setReferenceData(mjData* data)
{
    mDataRef = data;
}

int dxKinMuJoCo::getDoF() const
{
    return static_cast<int>(mQposIndices.size());
}

int dxKinMuJoCo::getNq() const
{
    if (!mModel)
    {
        return 0;
    }
    return mModel->nq;
}

std::vector<double> dxKinMuJoCo::getDofQpos(const std::vector<double>& qpos) const
{
    std::vector<double> out;
    if (!mModel)
    {
        return out;
    }
    if (qpos.size() == mQposIndices.size())
    {
        return qpos;
    }
    if (qpos.size() != static_cast<size_t>(mModel->nq))
    {
        return out;
    }
    out.reserve(mQposIndices.size());
    for (int idx : mQposIndices)
    {
        out.push_back(qpos[idx]);
    }
    return out;
}

bool dxKinMuJoCo::getFK(const std::vector<double>& qpos, PoseResult& out)
{
    if (!mModel)
    {
        return false;
    }
    if (!ensureScratchData())
    {
        return false;
    }
    if (mEEBodyId < 0 && mEESiteId < 0)
    {
        return false;
    }

    mj_resetData(mModel, mDataScratch);
    if (!applyQpos(qpos, mDataScratch))
    {
        return false;
    }

    mj_forward(mModel, mDataScratch);
    if (!computePoseFromData(mDataScratch, out))
    {
        return false;
    }

    return true;
}

bool dxKinMuJoCo::getFKCurrent(PoseResult& out) const
{
    if (!mModel)
    {
        return false;
    }
    if (mEEBodyId < 0 && mEESiteId < 0)
    {
        return false;
    }

    mjData* data = mDataRef ? mDataRef : mDataScratch;
    if (!data)
    {
        return false;
    }

    mj_forward(mModel, data);
    if (!computePoseFromData(data, out))
    {
        return false;
    }

    return true;
}

bool dxKinMuJoCo::getIK(const std::vector<double>& seed,
                        const std::vector<double>& targetPosQuat,
                        std::vector<double>& solution,
                        int maxIters,
                        double posTol,
                        double oriTol,
                        double damping)
{
    if (!mModel)
    {
        return false;
    }
    if (!ensureScratchData())
    {
        return false;
    }
    if (mEEBodyId < 0)
    {
        return false;
    }
    if (targetPosQuat.size() != 7)
    {
        return false;
    }

    const int dofCount = static_cast<int>(mQposIndices.size());
    if (seed.size() != static_cast<size_t>(mModel->nq) &&
            seed.size() != static_cast<size_t>(dofCount))
    {
        return false;
    }

    mj_resetData(mModel, mDataScratch);
    if (!applyQpos(seed, mDataScratch))
    {
        return false;
    }

    solution = seed;

    const double targetPos[3] = { targetPosQuat[0], targetPosQuat[1], targetPosQuat[2] };
    double targetQuat[4] = { targetPosQuat[3], targetPosQuat[4], targetPosQuat[5], targetPosQuat[6] };
    normalize_quat(targetQuat);

    std::vector<mjtNum> jacp(3 * mModel->nv, 0.0);
    std::vector<mjtNum> jacr(3 * mModel->nv, 0.0);
    std::vector<mjtNum> jacpA;
    std::vector<mjtNum> jacpB;
    std::vector<mjtNum> jacrOrient;
    if (mUseMidpoint)
    {
        jacpA.assign(3 * mModel->nv, 0.0);
        jacpB.assign(3 * mModel->nv, 0.0);
        jacrOrient.assign(3 * mModel->nv, 0.0);
    }

    for (int iter = 0; iter < maxIters; ++iter)
    {
        mj_forward(mModel, mDataScratch);

        double pos[3] = { 0.0, 0.0, 0.0 };
        double mat[9] = { 0.0 };
        if (!getEEPose(mDataScratch, pos, mat))
        {
            return false;
        }

        double currQuat[4] = { 1.0, 0.0, 0.0, 0.0 };
        mju_mat2Quat(currQuat, mat);
        normalize_quat(currQuat);

        double posErr[3] =
        {
            targetPos[0] - pos[0],
            targetPos[1] - pos[1],
            targetPos[2] - pos[2]
        };

        double conjCurr[4];
        quat_conjugate(currQuat, conjCurr);
        double qErr[4];
        quat_multiply(targetQuat, conjCurr, qErr);
        normalize_quat(qErr);
        if (qErr[0] < 0.0)
        {
            qErr[0] = -qErr[0];
            qErr[1] = -qErr[1];
            qErr[2] = -qErr[2];
            qErr[3] = -qErr[3];
        }

        double angErr[3];
        const double sinHalf = std::sqrt(std::max(0.0, 1.0 - qErr[0] * qErr[0]));
        if (sinHalf < 1e-8)
        {
            angErr[0] = 2.0 * qErr[1];
            angErr[1] = 2.0 * qErr[2];
            angErr[2] = 2.0 * qErr[3];
        }
        else
        {
            const double angle = 2.0 * std::acos(clamp_double(qErr[0], -1.0, 1.0));
            angErr[0] = (qErr[1] / sinHalf) * angle;
            angErr[1] = (qErr[2] / sinHalf) * angle;
            angErr[2] = (qErr[3] / sinHalf) * angle;
        }

        const double posNorm = std::sqrt(posErr[0] * posErr[0] +
                                         posErr[1] * posErr[1] +
                                         posErr[2] * posErr[2]);
        const double angNorm = std::sqrt(angErr[0] * angErr[0] +
                                         angErr[1] * angErr[1] +
                                         angErr[2] * angErr[2]);

        if (posNorm < posTol && angNorm < oriTol)
        {
            if (seed.size() == static_cast<size_t>(mModel->nq))
            {
                solution.assign(mDataScratch->qpos, mDataScratch->qpos + mModel->nq);
            }
            else
            {
                solution = extractDofQpos(mDataScratch);
            }
            return true;
        }

        if (mUseMidpoint && mEEMidBodyA >= 0 && mEEMidBodyB >= 0)
        {
            mj_jacBody(mModel, mDataScratch, jacpA.data(), jacr.data(), mEEMidBodyA);
            mj_jacBody(mModel, mDataScratch, jacpB.data(), jacr.data(), mEEMidBodyB);
            for (int i = 0; i < 3 * mModel->nv; ++i)
            {
                jacp[i] = static_cast<mjtNum>(0.5) * (jacpA[i] + jacpB[i]);
            }
            const int orientBody = (mEEMidOrientBody >= 0) ? mEEMidOrientBody : mEEBodyId;
            mj_jacBody(mModel, mDataScratch, jacpA.data(), jacrOrient.data(), orientBody);
            for (int i = 0; i < 3 * mModel->nv; ++i)
            {
                jacr[i] = jacrOrient[i];
            }
        }
        else if (mEESiteId >= 0)
        {
            mj_jacSite(mModel, mDataScratch, jacp.data(), jacr.data(), mEESiteId);
        }
        else
        {
            mj_jacBody(mModel, mDataScratch, jacp.data(), jacr.data(), mEEBodyId);
        }

        Eigen::MatrixXd J(6, dofCount);
        for (int i = 0; i < dofCount; ++i)
        {
            const int didx = mDofIndices[i];
            J(0, i) = jacp[0 * mModel->nv + didx];
            J(1, i) = jacp[1 * mModel->nv + didx];
            J(2, i) = jacp[2 * mModel->nv + didx];
            J(3, i) = jacr[0 * mModel->nv + didx];
            J(4, i) = jacr[1 * mModel->nv + didx];
            J(5, i) = jacr[2 * mModel->nv + didx];
        }

        Eigen::Matrix<double, 6, 1> err;
        err << posErr[0], posErr[1], posErr[2], angErr[0], angErr[1], angErr[2];

        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        JJt += (damping * damping) * Eigen::Matrix<double, 6, 6>::Identity();

        Eigen::Matrix<double, 6, 1> y = JJt.ldlt().solve(err);
        Eigen::VectorXd dq = J.transpose() * y;

        for (int i = 0; i < dofCount; ++i)
        {
            const int qadr = mQposIndices[i];
            mDataScratch->qpos[qadr] += dq[i];

            const int jid = mJointIds[i];
            if (mModel->jnt_limited[jid])
            {
                const double lo = mModel->jnt_range[2 * jid];
                const double hi = mModel->jnt_range[2 * jid + 1];
                mDataScratch->qpos[qadr] = clamp_double(mDataScratch->qpos[qadr], lo, hi);
            }
        }
    }

    if (seed.size() == static_cast<size_t>(mModel->nq))
    {
        solution.assign(mDataScratch->qpos, mDataScratch->qpos + mModel->nq);
    }
    else
    {
        solution = extractDofQpos(mDataScratch);
    }
    return false;
}

bool dxKinMuJoCo::isCollisionFree(const std::vector<double>& qpos, double minDist)
{
    if (!mModel)
    {
        return false;
    }
    if (!ensureScratchData())
    {
        return false;
    }

    std::vector<double> fullQpos;
    if (!buildFullQpos(qpos, fullQpos))
    {
        return false;
    }

    mj_resetData(mModel, mDataScratch);
    if (!applyQpos(fullQpos, mDataScratch))
    {
        return false;
    }

    mj_forward(mModel, mDataScratch);
    if (mDataScratch->ncon <= 0)
    {
        return true;
    }

    const int ncon = mDataScratch->ncon;
    for (int i = 0; i < ncon; ++i)
    {
        const mjContact& contact = mDataScratch->contact[i];
        const int g1 = contact.geom1;
        const int g2 = contact.geom2;
        const int b1 = (g1 >= 0 && g1 < mModel->ngeom) ? mModel->geom_bodyid[g1] : -1;
        const int b2 = (g2 >= 0 && g2 < mModel->ngeom) ? mModel->geom_bodyid[g2] : -1;
        const bool robotContact =
            (b1 >= 0 && b1 < static_cast<int>(mRobotBodies.size()) && mRobotBodies[static_cast<size_t>(b1)]) ||
            (b2 >= 0 && b2 < static_cast<int>(mRobotBodies.size()) && mRobotBodies[static_cast<size_t>(b2)]);
        if (!robotContact)
        {
            continue;
        }
        if (contact.dist <= minDist)
        {
            return false;
        }
    }

    return true;
}

bool dxKinMuJoCo::buildFullQpos(const std::vector<double>& qpos, std::vector<double>& full) const
{
    if (!mModel)
    {
        return false;
    }
    if (qpos.size() == static_cast<size_t>(mModel->nq))
    {
        full = qpos;
        return true;
    }
    if (qpos.size() != mQposIndices.size())
    {
        return false;
    }
    if (!mDataRef)
    {
        return false;
    }

    full.assign(mDataRef->qpos, mDataRef->qpos + mModel->nq);
    for (size_t i = 0; i < mQposIndices.size(); ++i)
    {
        full[mQposIndices[i]] = qpos[i];
    }
    return true;
}

void dxKinMuJoCo::printContacts(const std::vector<double>& qpos, int maxContacts)
{
    if (!mModel)
    {
        return;
    }
    if (!ensureScratchData())
    {
        return;
    }
    if (maxContacts <= 0)
    {
        return;
    }

    std::vector<double> fullQpos;
    if (!buildFullQpos(qpos, fullQpos))
    {
        return;
    }

    mj_resetData(mModel, mDataScratch);
    if (!applyQpos(fullQpos, mDataScratch))
    {
        return;
    }
    mj_forward(mModel, mDataScratch);

    const int ncon = mDataScratch->ncon;
    if (ncon <= 0)
    {
        return;
    }

    const int count = std::min(ncon, maxContacts);
    std::fprintf(stderr, "[dxKinMuJoCo] Contacts: %d\n", ncon);
    for (int i = 0; i < count; ++i)
    {
        const mjContact& contact = mDataScratch->contact[i];
        const int g1 = contact.geom1;
        const int g2 = contact.geom2;
        const int b1 = (g1 >= 0 && g1 < mModel->ngeom) ? mModel->geom_bodyid[g1] : -1;
        const int b2 = (g2 >= 0 && g2 < mModel->ngeom) ? mModel->geom_bodyid[g2] : -1;
        const char* g1Name = mj_id2name(mModel, mjOBJ_GEOM, g1);
        const char* g2Name = mj_id2name(mModel, mjOBJ_GEOM, g2);
        const char* b1Name = mj_id2name(mModel, mjOBJ_BODY, b1);
        const char* b2Name = mj_id2name(mModel, mjOBJ_BODY, b2);

        std::fprintf(stderr,
                     "  - geom1: %s (body %s), geom2: %s (body %s), dist=%.6f\n",
                     g1Name ? g1Name : "(unnamed)",
                     b1Name ? b1Name : "(unnamed)",
                     g2Name ? g2Name : "(unnamed)",
                     b2Name ? b2Name : "(unnamed)",
                     contact.dist);
    }
    if (ncon > count)
    {
        std::fprintf(stderr, "  ... (%d more)\n", ncon - count);
    }
}

bool dxKinMuJoCo::resolveEndEffectorBody()
{
    if (!mModel)
    {
        return false;
    }

    mUseMidpoint = false;
    mEEMidBodyA = -1;
    mEEMidBodyB = -1;
    mEEMidOrientBody = -1;

    int bestBody = -1;
    int bestDepth = -1;

    for (int jid : mJointIds)
    {
        const int bodyId = mModel->jnt_bodyid[jid];
        int depth = 0;
        int cur = bodyId;
        while (cur > 0)
        {
            cur = mModel->body_parentid[cur];
            ++depth;
        }

        if (depth > bestDepth || (depth == bestDepth && bodyId > bestBody))
        {
            bestDepth = depth;
            bestBody = bodyId;
        }
    }

    if (bestBody < 0 && mModel->nbody > 1)
    {
        bestBody = mModel->nbody - 1;
    }

    std::vector<char> isChainBody(static_cast<size_t>(mModel->nbody), 0);
    std::vector<char> hasChild(static_cast<size_t>(mModel->nbody), 0);
    for (int jid : mJointIds)
    {
        const int bodyId = mModel->jnt_bodyid[jid];
        if (bodyId >= 0 && bodyId < mModel->nbody)
        {
            isChainBody[static_cast<size_t>(bodyId)] = 1;
        }
    }
    for (int bodyId = 0; bodyId < mModel->nbody; ++bodyId)
    {
        if (!isChainBody[static_cast<size_t>(bodyId)])
        {
            continue;
        }
        const int parentId = mModel->body_parentid[bodyId];
        if (parentId >= 0 && parentId < mModel->nbody && isChainBody[static_cast<size_t>(parentId)])
        {
            hasChild[static_cast<size_t>(parentId)] = 1;
        }
    }

    std::vector<int> leafBodies;
    for (int bodyId = 0; bodyId < mModel->nbody; ++bodyId)
    {
        if (isChainBody[static_cast<size_t>(bodyId)] && !hasChild[static_cast<size_t>(bodyId)])
        {
            leafBodies.push_back(bodyId);
        }
    }

    if (leafBodies.size() >= 2)
    {
        auto bodyDepth = [this](int bodyId)
        {
            int depth = 0;
            int cur = bodyId;
            while (cur > 0)
            {
                cur = mModel->body_parentid[cur];
                ++depth;
            }
            return depth;
        };

        std::sort(leafBodies.begin(), leafBodies.end(),
                  [&bodyDepth](int a, int b)
                  {
                      const int da = bodyDepth(a);
                      const int db = bodyDepth(b);
                      if (da != db)
                      {
                          return da > db;
                      }
                      return a > b;
                  });

        const int leafA = leafBodies[0];
        const int leafB = leafBodies[1];

        std::vector<char> ancestor(static_cast<size_t>(mModel->nbody), 0);
        int cur = leafA;
        while (cur >= 0 && cur < mModel->nbody)
        {
            ancestor[static_cast<size_t>(cur)] = 1;
            const int parentId = mModel->body_parentid[cur];
            if (parentId == cur)
            {
                break;
            }
            cur = parentId;
        }

        int orientBody = -1;
        cur = leafB;
        while (cur >= 0 && cur < mModel->nbody)
        {
            if (ancestor[static_cast<size_t>(cur)])
            {
                orientBody = cur;
                break;
            }
            const int parentId = mModel->body_parentid[cur];
            if (parentId == cur)
            {
                break;
            }
            cur = parentId;
        }

        if (orientBody < 0)
        {
            orientBody = bestBody;
        }

        mUseMidpoint = true;
        mEEMidBodyA = leafA;
        mEEMidBodyB = leafB;
        mEEMidOrientBody = orientBody;
    }

    mEEBodyId = bestBody;
    return (mEEBodyId >= 0) || (mEESiteId >= 0);
}

bool dxKinMuJoCo::ensureScratchData()
{
    if (!mModel)
    {
        return false;
    }
    if (!mDataScratch)
    {
        mDataScratch = mj_makeData(mModel);
    }
    return mDataScratch != nullptr;
}

void dxKinMuJoCo::buildIndices()
{
    if (!mModel)
    {
        return;
    }
    for (int jid = 0; jid < mModel->njnt; ++jid)
    {
        const int type = mModel->jnt_type[jid];
        if (type != mjJNT_HINGE && type != mjJNT_SLIDE)
        {
            continue;
        }

        const int qposAdr = mModel->jnt_qposadr[jid];
        const int dofAdr = mModel->jnt_dofadr[jid];
        if (qposAdr < 0 || qposAdr >= mModel->nq)
        {
            continue;
        }
        if (dofAdr < 0 || dofAdr >= mModel->nv)
        {
            continue;
        }

        mJointIds.push_back(jid);
        mQposIndices.push_back(qposAdr);
        mDofIndices.push_back(dofAdr);
    }
}

void dxKinMuJoCo::buildRobotBodies()
{
    if (!mModel)
    {
        return;
    }
    mRobotBodies.assign(static_cast<size_t>(mModel->nbody), 0);
    for (int jid : mJointIds)
    {
        int bodyId = mModel->jnt_bodyid[jid];
        while (bodyId >= 0 && bodyId < mModel->nbody)
        {
            mRobotBodies[static_cast<size_t>(bodyId)] = 1;
            const int parentId = mModel->body_parentid[bodyId];
            if (parentId == bodyId)
            {
                break;
            }
            bodyId = parentId;
        }
    }
}

bool dxKinMuJoCo::applyQpos(const std::vector<double>& qpos, mjData* data) const
{
    if (!mModel || !data)
    {
        return false;
    }
    if (qpos.size() == static_cast<size_t>(mModel->nq))
    {
        for (int i = 0; i < mModel->nq; ++i)
        {
            data->qpos[i] = qpos[i];
        }
        return true;
    }

    if (qpos.size() == mQposIndices.size())
    {
        for (size_t i = 0; i < mQposIndices.size(); ++i)
        {
            data->qpos[mQposIndices[i]] = qpos[i];
        }
        return true;
    }

    return false;
}

std::vector<double> dxKinMuJoCo::extractDofQpos(const mjData* data) const
{
    std::vector<double> out;
    if (!data)
    {
        return out;
    }
    out.reserve(mQposIndices.size());
    for (int idx : mQposIndices)
    {
        out.push_back(data->qpos[idx]);
    }
    return out;
}

bool dxKinMuJoCo::computePoseFromData(const mjData* data, PoseResult& out) const
{
    if (!data || (mEEBodyId < 0 && mEESiteId < 0))
    {
        return false;
    }
    double pos[3] = { 0.0, 0.0, 0.0 };
    double mat[9] = { 0.0 };
    if (!getEEPose(data, pos, mat))
    {
        return false;
    }

    vpHomogeneousMatrix pose;
    pose[0][0] = mat[0];
    pose[0][1] = mat[1];
    pose[0][2] = mat[2];
    pose[0][3] = pos[0];
    pose[1][0] = mat[3];
    pose[1][1] = mat[4];
    pose[1][2] = mat[5];
    pose[1][3] = pos[1];
    pose[2][0] = mat[6];
    pose[2][1] = mat[7];
    pose[2][2] = mat[8];
    pose[2][3] = pos[2];
    pose[3][0] = 0.0;
    pose[3][1] = 0.0;
    pose[3][2] = 0.0;
    pose[3][3] = 1.0;

    Eigen::Matrix4f eigenPose = Eigen::Matrix4f::Identity();
    eigenPose(0, 0) = static_cast<float>(mat[0]);
    eigenPose(0, 1) = static_cast<float>(mat[1]);
    eigenPose(0, 2) = static_cast<float>(mat[2]);
    eigenPose(1, 0) = static_cast<float>(mat[3]);
    eigenPose(1, 1) = static_cast<float>(mat[4]);
    eigenPose(1, 2) = static_cast<float>(mat[5]);
    eigenPose(2, 0) = static_cast<float>(mat[6]);
    eigenPose(2, 1) = static_cast<float>(mat[7]);
    eigenPose(2, 2) = static_cast<float>(mat[8]);
    eigenPose(0, 3) = static_cast<float>(pos[0]);
    eigenPose(1, 3) = static_cast<float>(pos[1]);
    eigenPose(2, 3) = static_cast<float>(pos[2]);

    std::get<0>(out) = pose;
    std::get<1>(out) = eigenPose;
    buildPoseVectorFromData(data, std::get<2>(out));
    return true;
}

void dxKinMuJoCo::buildPoseVectorFromData(const mjData* data, std::vector<double>& out) const
{
    out.clear();
    if (!data || (mEEBodyId < 0 && mEESiteId < 0))
    {
        return;
    }

    double pos[3] = { 0.0, 0.0, 0.0 };
    double mat[9] = { 0.0 };
    if (!getEEPose(data, pos, mat))
    {
        return;
    }
    double quat[4] = { 1.0, 0.0, 0.0, 0.0 };
    mju_mat2Quat(quat, mat);

    out.reserve(7);
    out.push_back(pos[0]);
    out.push_back(pos[1]);
    out.push_back(pos[2]);
    out.push_back(quat[0]);
    out.push_back(quat[1]);
    out.push_back(quat[2]);
    out.push_back(quat[3]);
}

bool dxKinMuJoCo::getEEPose(const mjData* data, double pos[3], double mat[9]) const
{
    if (!data)
    {
        return false;
    }

    if (mUseMidpoint && mEEMidBodyA >= 0 && mEEMidBodyB >= 0)
    {
        const double* posA = data->xpos + 3 * mEEMidBodyA;
        const double* posB = data->xpos + 3 * mEEMidBodyB;
        pos[0] = 0.5 * (posA[0] + posB[0]);
        pos[1] = 0.5 * (posA[1] + posB[1]);
        pos[2] = 0.5 * (posA[2] + posB[2]);

        const int orientBody = (mEEMidOrientBody >= 0) ? mEEMidOrientBody : mEEBodyId;
        if (orientBody < 0)
        {
            return false;
        }
        const double* matPtr = data->xmat + 9 * orientBody;
        for (int i = 0; i < 9; ++i)
        {
            mat[i] = matPtr[i];
        }
        return true;
    }

    if (mEESiteId >= 0)
    {
        const double* posPtr = data->site_xpos + 3 * mEESiteId;
        const double* matPtr = data->site_xmat + 9 * mEESiteId;
        pos[0] = posPtr[0];
        pos[1] = posPtr[1];
        pos[2] = posPtr[2];
        for (int i = 0; i < 9; ++i)
        {
            mat[i] = matPtr[i];
        }
        return true;
    }

    if (mEEBodyId >= 0)
    {
        const double* posPtr = data->xpos + 3 * mEEBodyId;
        const double* matPtr = data->xmat + 9 * mEEBodyId;
        pos[0] = posPtr[0];
        pos[1] = posPtr[1];
        pos[2] = posPtr[2];
        for (int i = 0; i < 9; ++i)
        {
            mat[i] = matPtr[i];
        }
        return true;
    }

    return false;
}
