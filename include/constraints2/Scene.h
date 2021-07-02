#pragma once

#include "constraints2/typedef.h"

#include <vector>
#include <memory>

#include <assert.h>

namespace GCS { class System; }
namespace gs { class Shape2D; class Point2D; class Line2D; }

namespace ct2
{

class SceneData;
class Constraint;

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

    GeoID AddGeometry(const std::shared_ptr<gs::Shape2D>& shape);
    ConsID AddConstraint(const std::shared_ptr<Constraint>& cons);

    void ResetSolver();

    void Clear();
    void ClearConstraints();

private:
    GeoID AddPoint(const std::shared_ptr<gs::Point2D>& pt);
    GeoID AddLine(const std::shared_ptr<gs::Line2D>& line);

    // basic constraints
    ConsID AddDistanceConstraint(GeoID geo1, PointPos pos1,
        GeoID geo2, PointPos pos2, double* value);
    ConsID AddDistanceConstraint(GeoID line, double* value);

    // derived constraints
    ConsID AddVerticalConstraint(GeoID geo1, PointPos pos1,
        GeoID geo2, PointPos pos2);
    ConsID AddVerticalConstraint(GeoID line);
    ConsID AddHorizontalConstraint(GeoID geo1, PointPos pos1,
        GeoID geo2, PointPos pos2);
    ConsID AddHorizontalConstraint(GeoID line);

private:
    struct Geometry
    {
        GeoID GetPointID(PointPos pos) const
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
                return -1;
            }
        }

        std::shared_ptr<gs::Shape2D> shape = nullptr;
        size_t index = 0;

        GeoID start_pt_idx = -1;
        GeoID mid_pt_idx = -1;
        GeoID end_pt_idx = -1;
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