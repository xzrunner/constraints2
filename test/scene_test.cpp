#include <constraints2/Scene.h>

#include <SM_Calc.h>
#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>

#include <catch/catch.hpp>

TEST_CASE("add point")
{
    ct2::Scene scene;

    auto id = scene.AddPoint(std::make_shared<gs::Point2D>(sm::vec2(0, 0)));
    REQUIRE(id == 0);

    id = scene.AddPoint(std::make_shared<gs::Point2D>(sm::vec2(1, 0)));
    REQUIRE(id == 1);
}

TEST_CASE("add line")
{
    ct2::Scene scene;

    auto id = scene.AddLine(std::make_shared<gs::Line2D>(sm::vec2(0, 0), sm::vec2(0, 1)));
    REQUIRE(id == 0);

    id = scene.AddLine(std::make_shared<gs::Line2D>(sm::vec2(1, 1), sm::vec2(2, 2)));
    REQUIRE(id == 1);
}

TEST_CASE("distance constraint - two points")
{
    ct2::Scene scene;

    auto p0 = std::make_shared<gs::Point2D>(sm::vec2(0, 0));
    auto p0_id = scene.AddPoint(p0);
    REQUIRE(p0_id == 0);

    auto p1 = std::make_shared<gs::Point2D>(sm::vec2(5, 0));
    auto p1_id = scene.AddPoint(p1);
    REQUIRE(p1_id == 1);

    double dist = 1.0f;
    auto cons = scene.AddDistanceConstraint(p0_id, ct2::Scene::PointPos::Start,
        p1_id, ct2::Scene::PointPos::Start, &dist);

    scene.Solve();

    REQUIRE(sm::dis_pos_to_pos(p0->GetPos(), p1->GetPos()) == Approx(dist));

    dist = 2;
    scene.Solve();
    REQUIRE(sm::dis_pos_to_pos(p0->GetPos(), p1->GetPos()) == Approx(dist));
}

TEST_CASE("distance constraint - line")
{
    ct2::Scene scene;

    auto line = std::make_shared<gs::Line2D>(sm::vec2(0, 0), sm::vec2(0, 5));
    auto line_id = scene.AddLine(line);
    REQUIRE(line_id == 0);

    double dist = 1.0f;
    auto cons = scene.AddDistanceConstraint(line_id, &dist);

    scene.Solve();
    REQUIRE(sm::dis_pos_to_pos(line->GetStart(), line->GetEnd()) == Approx(dist));

    dist = 2;
    scene.Solve();
    REQUIRE(sm::dis_pos_to_pos(line->GetStart(), line->GetEnd()) == Approx(dist));

    line->SetStart(sm::vec2(1, 0));
    line->SetEnd(sm::vec2(1, 5));
    scene.Solve();
}

TEST_CASE("horizontal constraint - line")
{
    ct2::Scene scene;

    auto line = std::make_shared<gs::Line2D>(sm::vec2(0, 0), sm::vec2(5, 5));
    auto line_id = scene.AddLine(line);
    REQUIRE(line_id == 0);

    auto cons = scene.AddHorizontalConstraint(line_id);

    scene.Solve();

    REQUIRE(line->GetEnd() == sm::vec2(5, 0));
}

TEST_CASE("vertical constraint - line")
{
    ct2::Scene scene;

    auto line = std::make_shared<gs::Line2D>(sm::vec2(0, 0), sm::vec2(5, 5));
    auto line_id = scene.AddLine(line);
    REQUIRE(line_id == 0);

    auto cons = scene.AddVerticalConstraint(line_id);

    scene.Solve();

    REQUIRE(line->GetEnd() == sm::vec2(0, 5));
}