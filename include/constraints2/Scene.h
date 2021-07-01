#pragma once

#include <vector>
#include <memory>

#include <assert.h>

namespace GCS { class System; }
namespace gs { class Shape2D; class Point2D; class Line2D; }

namespace ct2
{

class SceneData;

class Scene
{
public:
    enum class PointPos
    {
        None,
        Start,
        End,
        Mid
    };

public:
    Scene();
    ~Scene();

    int Solve();

    size_t AddPoint(const std::shared_ptr<gs::Point2D>& pt);
    size_t AddLine(const std::shared_ptr<gs::Line2D>& line);

    // basic constraints
    size_t AddDistanceConstraint(size_t geo1, PointPos pos1,
        size_t geo2, PointPos pos2, double* value);
    size_t AddDistanceConstraint(size_t line, double* value);

    // derived constraints
    size_t AddVerticalConstraint(size_t geo1, PointPos pos1,
        size_t geo2, PointPos pos2);
    size_t AddVerticalConstraint(size_t line);
    size_t AddHorizontalConstraint(size_t geo1, PointPos pos1,
        size_t geo2, PointPos pos2);
    size_t AddHorizontalConstraint(size_t line);

    void ResetSolver();

    void Clear();
    void ClearConstraints();

private:
    struct Geometry
    {
        size_t GetPointID(PointPos pos) const
        {
            switch (pos)
            {
            case PointPos::Start:
                return start_pt_idx;
            case PointPos::End:
                return end_pt_idx;
            case PointPos::Mid:
                return mid_pt_idx;
            default:
                assert(0);
                return 0;
            }
        }

        std::shared_ptr<gs::Shape2D> shape = nullptr;
        size_t index = 0;

        size_t start_pt_idx = 0;
        size_t mid_pt_idx = 0;
        size_t end_pt_idx = 0;
    };

    void BeforeSolve();
    void AfterSolve();

private:
    // solving parameters
    std::vector<double*> m_parameters;
    std::shared_ptr<SceneData> m_data = nullptr;

    std::shared_ptr<GCS::System> m_gcs_sys = nullptr;
    int m_constraints_counter = 0;

    std::vector<Geometry> m_geoms;

}; // Scene

}