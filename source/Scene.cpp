#include "constraints2/Scene.h"

#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>

#include <PlaneGCS/GCS.h>

namespace ct2
{

struct SceneData
{
    GCS::Algorithm defaultSolver = GCS::DogLeg;
    GCS::Algorithm defaultSolverRedundant = GCS::DogLeg;

    std::vector<GCS::Point> points;
    std::vector<GCS::Line>  lines;
};

Scene::Scene()
{
    m_data = std::make_shared<SceneData>();

    m_gcs_sys = std::make_shared<GCS::System>();

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
    BeforeSolve();
    int ret = m_gcs_sys->solve();
    if (ret == GCS::Success) {
        m_gcs_sys->applySolution();
        AfterSolve();
    }

    return ret;
}

GeoID Scene::AddPoint(const std::shared_ptr<gs::Point2D>& pt)
{
    auto& pos = pt->GetPos();
    auto x = new double(pos.x);
    auto y = new double(pos.y);

    GCS::Point dst;
    dst.x = x;
    dst.y = y;

    m_parameters.push_back(x);
    m_parameters.push_back(y);

    auto point_id = m_data->points.size();
    m_data->points.push_back(dst);

    Geometry geo;
    geo.shape = pt;
    geo.index = point_id;
    geo.start_pt_idx = point_id;
    geo.mid_pt_idx = point_id;
    geo.end_pt_idx = point_id;

    GeoID geo_id = m_geoms.size();
    m_geoms.push_back(geo);
    return geo_id;
}

GeoID Scene::AddLine(const std::shared_ptr<gs::Line2D>& line)
{
    auto& s = line->GetStart();
    auto& e = line->GetEnd();
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
    geo.start_pt_idx = m_data->points.size();
    geo.end_pt_idx   = m_data->points.size() + 1;
    m_data->points.push_back(p1);
    m_data->points.push_back(p2);

    GCS::Line l;
    l.p1 = p1;
    l.p2 = p2;
    geo.index = m_data->lines.size();
    m_data->lines.push_back(l);

    GeoID geo_id = m_geoms.size();
    m_geoms.push_back(geo);
    return geo_id;
}

ConsID Scene::AddDistanceConstraint(GeoID geo1, PointPos pos1, GeoID geo2, PointPos pos2, double* value)
{
    assert(geo1 < m_geoms.size() && geo2 < m_geoms.size());
    auto p1 = m_geoms[geo1].GetPointID(pos1);
    auto p2 = m_geoms[geo2].GetPointID(pos2);
    assert(p1 < m_data->points.size() && p2 < m_data->points.size());

    int tag = ++m_constraints_counter;
    m_gcs_sys->addConstraintP2PDistance(m_data->points[p1], m_data->points[p2], value, tag);

    ResetSolver();

    return tag;
}

ConsID Scene::AddDistanceConstraint(GeoID line, double* value)
{
    assert(line < m_geoms.size());
    assert(m_geoms[line].shape->GetType() == gs::ShapeType2D::Line);
    return AddDistanceConstraint(line, PointPos::Start, line, PointPos::End, value);
}

ConsID Scene::AddVerticalConstraint(GeoID geo1, PointPos pos1, GeoID geo2, PointPos pos2)
{
    assert(geo1 < m_geoms.size() && geo2 < m_geoms.size());
    auto p1 = m_geoms[geo1].GetPointID(pos1);
    auto p2 = m_geoms[geo2].GetPointID(pos2);
    assert(p1 < m_data->points.size() && p2 < m_data->points.size());

    int tag = ++m_constraints_counter;
    m_gcs_sys->addConstraintVertical(m_data->points[p1], m_data->points[p2], tag);

    ResetSolver();

    return tag;
}

ConsID Scene::AddVerticalConstraint(GeoID line)
{
    assert(line < m_geoms.size());
    assert(m_geoms[line].shape->GetType() == gs::ShapeType2D::Line);
    return AddVerticalConstraint(line, PointPos::Start, line, PointPos::End);
}

ConsID Scene::AddHorizontalConstraint(GeoID geo1, PointPos pos1, GeoID geo2, PointPos pos2)
{
    assert(geo1 < m_geoms.size() && geo2 < m_geoms.size());
    auto p1 = m_geoms[geo1].GetPointID(pos1);
    auto p2 = m_geoms[geo2].GetPointID(pos2);
    assert(p1 < m_data->points.size() && p2 < m_data->points.size());

    int tag = ++m_constraints_counter;
    m_gcs_sys->addConstraintHorizontal(m_data->points[p1], m_data->points[p2], tag);

    ResetSolver();

    return tag;
}

ConsID Scene::AddHorizontalConstraint(GeoID line)
{
    assert(line < m_geoms.size());
    assert(m_geoms[line].shape->GetType() == gs::ShapeType2D::Line);
    return AddHorizontalConstraint(line, PointPos::Start, line, PointPos::End);
}

void Scene::ResetSolver()
{
    //m_gcs_sys->initSolution(GCS::Algorithm(GCS::DogLeg));
    /////////////////////


    //m_gcs_sys->clearByTag(-1);
    m_gcs_sys->declareUnknowns(m_parameters);
    //m_gcs_sys->declareDrivenParams(DrivenParameters);
    m_gcs_sys->initSolution(m_data->defaultSolverRedundant);
    //m_gcs_sys->getConflicting(Conflicting);
    //m_gcs_sys->getRedundant(Redundant);
    //m_gcs_sys->getDependentParams(pconstraintplistOut);

    //calculateDependentParametersElements();

    //m_gcs_sys->dofsNumber();
}

void Scene::BeforeSolve()
{
    for (auto& geo : m_geoms)
    {
        auto type = geo.shape->GetType();
        if (type == gs::ShapeType2D::Point)
        {
            auto src = std::static_pointer_cast<gs::Point2D>(geo.shape);
            auto& dst = m_data->points[geo.index];
            *dst.x = static_cast<double>(src->GetPos().x);
            *dst.y = static_cast<double>(src->GetPos().y);
        }
        else if (type == gs::ShapeType2D::Line)
        {
            auto src = std::static_pointer_cast<gs::Line2D>(geo.shape);
            auto& dst = m_data->lines[geo.index];
            *dst.p1.x = static_cast<double>(src->GetStart().x);
            *dst.p1.y = static_cast<double>(src->GetStart().y);
            *dst.p2.x = static_cast<double>(src->GetEnd().x);
            *dst.p2.y = static_cast<double>(src->GetEnd().y);
        }
    }
}

void Scene::AfterSolve()
{
    for (auto& geo : m_geoms)
    {
        auto type = geo.shape->GetType();
        if (type == gs::ShapeType2D::Point)
        {
            auto dst = std::static_pointer_cast<gs::Point2D>(geo.shape);
            auto& src = m_data->points[geo.index];
            dst->SetPos(sm::vec2(
                static_cast<float>(*src.x), static_cast<float>(*src.y))
            );
        }
        else if (type == gs::ShapeType2D::Line)
        {
            auto dst = std::static_pointer_cast<gs::Line2D>(geo.shape);
            auto& src = m_data->lines[geo.index];
            dst->SetStart(sm::vec2(static_cast<float>(*src.p1.x), static_cast<float>(*src.p1.y)));
            dst->SetEnd(sm::vec2(static_cast<float>(*src.p2.x), static_cast<float>(*src.p2.y)));
        }
    }
}

void Scene::Clear()
{
    for (auto& p : m_parameters) {
        delete p;
    }
    m_parameters.clear();

    m_data->points.clear();
    m_data->lines.clear();

    m_constraints_counter = 0;

    m_geoms.clear();
}

void Scene::ClearConstraints()
{
    for (int i = 0; i < m_constraints_counter; ++i) {
        m_gcs_sys->clearByTag(i + 1);
    }
    m_constraints_counter = 0;
}

}