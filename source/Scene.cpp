#include "constraints2/Scene.h"

#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>

namespace ct2
{

Scene::Scene()
    : defaultSolver(GCS::DogLeg)
    , defaultSolverRedundant(GCS::DogLeg)
{
    ResetSolver();
}

Scene::~Scene()
{
    for (auto& p : m_parameters) {
        delete p;
    }
}

int Scene::Solve()
{
    int ret = m_gcs_sys.solve();

    if (ret == GCS::Success) {
        m_gcs_sys.applySolution();
        /*valid_solution = */UpdateGeometry();
        //if (!valid_solution) {
        //    GCSsys.undoSolution();
        //    updateGeometry();
        //    Base::Console().Warning("Invalid solution from %s solver.\n", solvername.c_str());
        //}
        //else {
        //    updateNonDrivingConstraints();
        //}
    }

    return ret;
}

size_t Scene::AddPoint(const std::shared_ptr<gs::Point2D>& pt)
{
    auto& pos = pt->GetPos();
    auto x = new double(pos.x);
    auto y = new double(pos.y);

    GCS::Point dst;
    dst.x = x;
    dst.y = y;

    m_parameters.push_back(x);
    m_parameters.push_back(y);

    auto point_id = m_points.size();
    m_points.push_back(dst);

    Geometry geo;
    geo.shape = pt;
    geo.index = point_id;
    geo.start_pt_idx = point_id;
    geo.mid_pt_idx = point_id;
    geo.end_pt_idx = point_id;

    auto geo_id = m_geoms.size();
    m_geoms.push_back(geo);
    return geo_id;
}

size_t Scene::AddLine(const std::shared_ptr<gs::Line2D>& line)
{
    auto& s = line->GetStart();
    auto sx = new double(s.x);
    auto sy = new double(s.y);

    auto& e = line->GetEnd();
    auto ex = new double(e.x);
    auto ey = new double(e.y);

    GCS::Point p1, p2;

    m_parameters.push_back(new double(s.x));
    m_parameters.push_back(new double(s.y));
    p1.x = m_parameters[m_parameters.size() - 2];
    p1.y = m_parameters[m_parameters.size() - 1];

    m_parameters.push_back(new double(e.x));
    m_parameters.push_back(new double(e.y));
    p2.x = m_parameters[m_parameters.size() - 2];
    p2.y = m_parameters[m_parameters.size() - 1];

    Geometry geo;
    geo.shape = line;
    geo.start_pt_idx = m_points.size();
    geo.end_pt_idx   = m_points.size() + 1;
    m_points.push_back(p1);
    m_points.push_back(p2);

    GCS::Line l;
    l.p1 = p1;
    l.p2 = p2;
    geo.index = m_lines.size();
    m_lines.push_back(l);

    auto geo_id = m_geoms.size();
    m_geoms.push_back(geo);
    return geo_id;
}

size_t Scene::AddDistanceConstraint(size_t geo1, PointPos pos1, size_t geo2, PointPos pos2, double* value)
{
    assert(geo1 < m_geoms.size() && geo2 < m_geoms.size());
    auto p1 = m_geoms[geo1].GetPointID(pos1);
    auto p2 = m_geoms[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    int tag = ++m_constraints_counter;
//    m_gcs_sys.addConstraintP2PDistance(m_points[p1], m_points[p2], value, tag);
    m_gcs_sys.addConstraintP2PDistance(m_points[0], m_points[1], value, tag);

    ResetSolver();

    return tag;
}

size_t Scene::AddDistanceConstraint(size_t line, double* value)
{
    assert(line < m_geoms.size());
    assert(m_geoms[line].shape->get_type() == rttr::type::get<gs::Line2D>());
    return AddDistanceConstraint(line, PointPos::Start, line, PointPos::End, value);
}

size_t Scene::AddVerticalConstraint(size_t geo1, PointPos pos1, size_t geo2, PointPos pos2, double* value)
{
    assert(geo1 < m_geoms.size() && geo2 < m_geoms.size());
    auto p1 = m_geoms[geo1].GetPointID(pos1);
    auto p2 = m_geoms[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    int tag = ++m_constraints_counter;
    m_gcs_sys.addConstraintVertical(m_points[p1], m_points[p2], tag);

    ResetSolver();

    return tag;
}

size_t Scene::AddHorizontalConstraint(size_t geo1, PointPos pos1, size_t geo2, PointPos pos2, double* value)
{
    assert(geo1 < m_geoms.size() && geo2 < m_geoms.size());
    auto p1 = m_geoms[geo1].GetPointID(pos1);
    auto p2 = m_geoms[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    int tag = ++m_constraints_counter;
    m_gcs_sys.addConstraintHorizontal(m_points[p1], m_points[p2], tag);

    ResetSolver();

    return tag;
}

void Scene::ResetSolver()
{
    //m_gcs_sys.initSolution(GCS::Algorithm(GCS::DogLeg));
    /////////////////////


    //m_gcs_sys.clearByTag(-1);
    m_gcs_sys.declareUnknowns(m_parameters);
    //m_gcs_sys.declareDrivenParams(DrivenParameters);
    m_gcs_sys.initSolution(defaultSolverRedundant);
    //m_gcs_sys.getConflicting(Conflicting);
    //m_gcs_sys.getRedundant(Redundant);
    //m_gcs_sys.getDependentParams(pconstraintplistOut);

    //calculateDependentParametersElements();

    //m_gcs_sys.dofsNumber();
}

void Scene::UpdateGeometry()
{
    for (auto& geo : m_geoms)
    {
        auto type = geo.shape->get_type();
        if (type == rttr::type::get<gs::Point2D>())
        {
            auto dst = std::static_pointer_cast<gs::Point2D>(geo.shape);
            auto& src = m_points[geo.start_pt_idx];
            dst->SetPos(sm::vec2(
                static_cast<float>(*src.x), static_cast<float>(*src.y))
            );
        }
    }
}

void Scene::Clear()
{
    for (auto& p : m_parameters) {
        delete p;
    }
    m_parameters.clear();

    m_points.clear();
    m_lines.clear();

    m_constraints_counter = 0;

    m_geoms.clear();
}

}