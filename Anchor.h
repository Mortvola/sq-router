/*
 * Anchor.h
 *
 *  Created on: Oct 29, 2019
 *      Author: richard
 */

#pragma once
#include "Point.h"

class Anchor
{
public:

	Anchor () = default;

	Point m_point;
	std::string m_type;

	struct LineEndpoint
	{
		int m_lineId {-1};
		double m_fraction {};
	}

  // 0: previous line
  // 1: next line
  m_lineEndpoint[2];
};


