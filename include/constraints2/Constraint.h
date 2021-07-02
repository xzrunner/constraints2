#pragma once

#include "constraints2/typedef.h"

namespace ct2
{

enum class ConstraintType
{
	None,
	Distance,
};

class Constraint
{
public:
	Constraint(ConstraintType type, GeoID geo0, GeoID geo1, double value)
		: m_type(type)
		, m_geo0(geo0)
		, m_geo1(geo1)
		, m_value(value)
	{
	}

	auto GetType() const { return m_type; }

	auto GetGeo0() const { return m_geo0; }
	auto GetGeo1() const { return m_geo1; }

	double* GetValuePtr() { return &m_value; }
	void SetValue(double value) { m_value = value; }

private:
	ConstraintType m_type = ConstraintType::None;

	GeoID m_geo0 = -1, m_geo1 = -1;

	double m_value = 0.0;

}; // Constraint

}