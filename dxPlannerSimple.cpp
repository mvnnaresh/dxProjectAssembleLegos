#include "dxPlannerSimple.h"

#include <algorithm>
#include <cmath>
#include <array>
#include <cstdio>

namespace
{
constexpr double kEps = 1e-9;

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

void slerp_quat(const double a[4], const double b[4], double t, double out[4])
{
    double qa[4] = { a[0], a[1], a[2], a[3] };
    double qb[4] = { b[0], b[1], b[2], b[3] };
    normalize_quat(qa);
    normalize_quat(qb);

    double dot = qa[0] * qb[0] + qa[1] * qb[1] + qa[2] * qb[2] + qa[3] * qb[3];
    if (dot < 0.0)
    {
        dot = -dot;
        qb[0] = -qb[0];
        qb[1] = -qb[1];
        qb[2] = -qb[2];
        qb[3] = -qb[3];
    }

    if (dot > 0.9995)
    {
        out[0] = qa[0] + t * (qb[0] - qa[0]);
        out[1] = qa[1] + t * (qb[1] - qa[1]);
        out[2] = qa[2] + t * (qb[2] - qa[2]);
        out[3] = qa[3] + t * (qb[3] - qa[3]);
        normalize_quat(out);
        return;
    }

    const double theta = std::acos(std::max(-1.0, std::min(1.0, dot)));
    const double sinTheta = std::sin(theta);
    const double w1 = std::sin((1.0 - t) * theta) / sinTheta;
    const double w2 = std::sin(t * theta) / sinTheta;

    out[0] = w1 * qa[0] + w2 * qb[0];
    out[1] = w1 * qa[1] + w2 * qb[1];
    out[2] = w1 * qa[2] + w2 * qb[2];
    out[3] = w1 * qa[3] + w2 * qb[3];
    normalize_quat(out);
}
}

dxPlannerSimple::dxPlannerSimple(dxKinMuJoCo* kin): mKin(kin)
{
}

bool dxPlannerSimple::init(dxKinMuJoCo* kin)
{
    if (kin)
    {
        mKin = kin;
    }
    if (!mKin)
    {
        return false;
    }
    return true;
}

void dxPlannerSimple::setKinematics(dxKinMuJoCo* kin)
{
    mKin = kin;
    if (!mKin)
    {
    }
}

void dxPlannerSimple::setStart(const std::vector<double>& start)
{
    mStart = start;
    mHasStart = !mStart.empty();
}

void dxPlannerSimple::setGoal(const std::vector<double>& goal)
{
    mGoal = goal;
    mHasGoal = !mGoal.empty();
}

void dxPlannerSimple::setParams(const Params& params)
{
    mParams = params;
}

bool dxPlannerSimple::solve()
{
    mPath.clear();
    if (!mHasStart || !mHasGoal)
    {
        return false;
    }
    if (mStart.size() != mGoal.size())
    {
        return false;
    }

    int steps = mParams.steps;
    if (steps < 2)
    {
        steps = 2;
    }

    mPath = planJoints(mStart, mGoal, steps);
    return !mPath.empty();
}

const std::vector<std::vector<double>>& dxPlannerSimple::getPath() const
{
    return mPath;
}

std::vector<std::vector<double>> dxPlannerSimple::buildTrajectory() const
{
    return mPath;
}

std::vector<std::vector<double>> dxPlannerSimple::buildTrajectory(const std::vector<std::vector<double>>& path) const
{
    return path;
}

std::vector<std::array<double, 3>> dxPlannerSimple::getPathAs3DPoints() const
{
    return build3DPoints(mPath);
}

std::vector<std::array<double, 3>> dxPlannerSimple::getTrajAs3DPoints(
    const std::vector<std::vector<double>>& trajectory) const
{
    return build3DPoints(trajectory);
}

std::vector<std::vector<double>> dxPlannerSimple::planJoints(const std::vector<double>& start,
                              const std::vector<double>& goal,
                              int steps) const
{
    std::vector<std::vector<double>> trajectory;
    if (start.size() != goal.size())
    {
        return trajectory;
    }
    if (mParams.checkCollisions && !mKin)
    {
        return trajectory;
    }

    if (steps < 2)
    {
        steps = 2;
    }
    trajectory.reserve(static_cast<size_t>(steps));

    for (int i = 0; i < steps; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(steps - 1);
        std::vector<double> point(start.size(), 0.0);
        for (size_t j = 0; j < start.size(); ++j)
        {
            point[j] = start[j] + (goal[j] - start[j]) * t;
        }
        if (mParams.checkCollisions &&
            !mKin->isCollisionFree(point, mParams.collisionDist))
        {
            std::fprintf(stderr,
                         "[dxPlannerSimple] Joint plan blocked by collision at step %d/%d (minDist=%.6f).\n",
                         i, steps - 1, mParams.collisionDist);
            if (mParams.debugPaths && mKin)
            {
                mKin->printContacts(point, 5);
            }
            trajectory.clear();
            return trajectory;
        }
        trajectory.push_back(std::move(point));
    }

    return trajectory;
}

std::vector<std::array<double, 3>> dxPlannerSimple::build3DPoints(
    const std::vector<std::vector<double>>& qpath) const
{
    std::vector<std::array<double, 3>> points;
    if (!mKin)
    {
        return points;
    }
    points.reserve(qpath.size());
    for (const auto& qpos : qpath)
    {
        dxKinMuJoCo::PoseResult fkPlanned;
        if (!mKin->getFK(qpos, fkPlanned))
        {
            continue;
        }
        const Eigen::Matrix4f& pose = std::get<1>(fkPlanned);
        points.push_back({{
            static_cast<double>(pose(0, 3)),
            static_cast<double>(pose(1, 3)),
            static_cast<double>(pose(2, 3))
        }});
    }
    return points;
}

bool dxPlannerSimple::planCartesian(const std::vector<double>& startPose,
                                    const std::vector<double>& goalPose,
                                    const std::vector<double>& seed,
                                    std::vector<std::vector<double>>& trajectory,
                                    int steps)
{
    trajectory.clear();
    if (!mKin)
    {
        return false;
    }
    if (startPose.size() != 7 || goalPose.size() != 7)
    {
        return false;
    }
    if (seed.empty())
    {
        return false;
    }
    if (mParams.checkCollisions && !mKin)
    {
        return false;
    }

    if (steps < 2)
    {
        steps = 2;
    }
    trajectory.reserve(static_cast<size_t>(steps));

    double qa[4] = { startPose[3], startPose[4], startPose[5], startPose[6] };
    double qb[4] = { goalPose[3], goalPose[4], goalPose[5], goalPose[6] };
    normalize_quat(qa);
    normalize_quat(qb);

    std::vector<double> currentSeed = seed;

    for (int i = 0; i < steps; ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(steps - 1);
        std::vector<double> pose(7, 0.0);
        pose[0] = startPose[0] + (goalPose[0] - startPose[0]) * t;
        pose[1] = startPose[1] + (goalPose[1] - startPose[1]) * t;
        pose[2] = startPose[2] + (goalPose[2] - startPose[2]) * t;

        double qinterp[4];
        slerp_quat(qa, qb, t, qinterp);
        pose[3] = qinterp[0];
        pose[4] = qinterp[1];
        pose[5] = qinterp[2];
        pose[6] = qinterp[3];

        std::vector<double> solution;
        const bool ok = mKin->getIK(currentSeed, pose, solution);
        if (!ok)
        {
            trajectory.clear();
            return false;
        }
        if (mParams.checkCollisions &&
            !mKin->isCollisionFree(solution, mParams.collisionDist))
        {
            std::fprintf(stderr,
                         "[dxPlannerSimple] Cartesian plan blocked by collision at step %d/%d (minDist=%.6f).\n",
                         i, steps - 1, mParams.collisionDist);
            if (mParams.debugPaths && mKin)
            {
                mKin->printContacts(solution, 5);
            }
            trajectory.clear();
            return false;
        }

        trajectory.push_back(solution);
        currentSeed = solution;
    }

    return true;
}
