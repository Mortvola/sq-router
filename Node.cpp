#include "Node.h"
#include "DBConnection.h"
#include "Elevation.h"
#include <algorithm>

void Node::addEdge(const std::shared_ptr<Edge> &newEdge)
{
	std::unique_lock<std::mutex> lock(m_edgesAccess);

	auto i = std::find_if(m_edges.begin(), m_edges.end(), [newEdge](const std::shared_ptr<Edge> &edge)
			{
				return edge->m_lineId == newEdge->m_lineId
          && edge->m_edgeEnd[Edge::EndType::Start].m_nodeId == newEdge->m_edgeEnd[Edge::EndType::Start].m_nodeId
          && edge->m_edgeEnd[Edge::EndType::End].m_nodeId == newEdge->m_edgeEnd[Edge::EndType::End].m_nodeId;
			});

	if (i == m_edges.end())
	{
		m_edges.push_back(newEdge);
	}
}

void updateNavNodeElevations()
{
	try
	{
    auto dbConnection = std::make_shared<DBConnection>();

		PreparedStatement updateNavNodeElevations(
        dbConnection,
				"update nav_nodes "
				"set way = ST_SetSRID(ST_GeomFromGeoJSON($1), 3857) "
				"where id = $2 ");

		std::string sql =
			"SELECT id, ST_AsGeoJSON(way) AS way1, ST_AsGeoJSON(ST_TRANSFORM(way, 4326)) as way2 "
			"FROM nav_nodes ";

		std::cerr << "Querying rows to update..." << std::endl;
		auto r = dbConnection->exec(sql);

		Json::Reader reader;
		Json::FastWriter fastWriter;

		std::cerr << "Updating rows..." << std::endl;
		auto transaction = dbConnection->newTransaction ();

		int rowsUpdated {0};

		for (auto const &row: r)
		{
			Json::Value root1;
			reader.parse(row["way1"].as<std::string>(), root1);

			Json::Value root2;
			reader.parse(row["way2"].as<std::string>(), root2);

			root1["coordinates"][2] = Elevation::getInstance()->getElevation(LatLng(root2["coordinates"][1].asDouble(), root2["coordinates"][0].asDouble()));

			auto updatedJSON = fastWriter.write(root1);
			transaction.exec (updateNavNodeElevations, updatedJSON, row["id"].as<int>());
			rowsUpdated++;

			if (rowsUpdated % 10000 == 0)
			{
				std::cerr << "Updated " << rowsUpdated << std::endl;
			}
		}

		transaction.commit ();
	}
	catch (const pqxx::pqxx_exception &e)
	{
		std::cerr << e.base().what() << std::endl;
		const pqxx::sql_error *s = dynamic_cast<const pqxx::sql_error*>(&e.base());
		if (s)
		{
			std::cerr << "Query was: " << s->query() << std::endl;
		}
	}
	catch (...)
	{
		std::cerr << "updateRouteElevations error" << std::endl;
	}
}
