/****************************************************************************/
/*Routing using CCH*/
/****************************************************************************/
#pragma once
#include <config.h>

#include <string>
#include <functional>
#include <vector>
#include <set>
#include <limits>
#include <algorithm>
#include <iterator>
#include <map>
#include <deque>
#include <string>
#include <utils/common/SysUtils.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StdDefs.h>
#include <utils/router/SUMOAbstractRouter.h>
#include "../../microsim/MSNet.h"

#include "../HL/akiba.hpp"
#include  "../HL/ghl.hpp"
#include "../HL/labeling.hpp"
#include "../HL/ordering.hpp"
#include "../HL/graph.hpp"
#include "../HL/hhl.hpp"
#include "../HL/ghl.hpp"
#include "../HL/labeling_check.hpp"
#include "../RoutingKitdll/vector_io.h"
#include "../HL/pruned_highway_labeling.h"
#include "../RoutingKitdll/permutation.h"

using namespace RoutingKit;


class MSEdge;
class MSBaseVehicle;
class MSRoutingEngine;

using namespace hl;

template<class E, class V>
class HLRouter : public SUMOAbstractRouter<E, V> {

public:

	HLRouter(const std::vector<E*>& edges, bool unbuildIsWarning, typename SUMOAbstractRouter<E, V>::Operation operation,
		const SUMOVehicleClass svc,
		SUMOTime weightPeriod,
		const bool havePermissions, const bool haveRestrictions) :
		SUMOAbstractRouter<E, V>("HLRouter", unbuildIsWarning, operation, nullptr, havePermissions, haveRestrictions),
		myEdges(edges),
		myWeightPeriod(weightPeriod),
		myValidUntil(0),
		mySVC(SVC_PASSENGER) {
		auto msTime = MSNet::getInstance()->getCurrentTimeStep();


		for (const E* edge : myEdges) {
			if (edge->getFromJunction() != nullptr && edge->getToJunction() != nullptr) {
				if (degree.find(std::stoul(edge->getFromJunction()->getID())) == degree.end())
				{
					degree[std::stoul(edge->getFromJunction()->getID())] = 1;

				}
				else
				{
					degree[std::stoul(edge->getFromJunction()->getID())] = degree[std::stoul(edge->getFromJunction()->getID())] + 1;
				}

				if (degree.find(std::stoul(edge->getToJunction()->getID())) == degree.end())
				{
					degree[std::stoul(edge->getToJunction()->getID())] = 1;

				}
				else
				{
					degree[std::stoul(edge->getToJunction()->getID())] = degree[std::stoul(edge->getToJunction()->getID())] + 1;
				}

			}
		}
		g.set_n(degree.size());
		for (const E* edge : myEdges) {
			if (edge->getFromJunction() != nullptr && edge->getToJunction() != nullptr) {
				auto effort = this->getEffort(edge, nullptr, STEPS2TIME(msTime));
				nodeEdgeMap.insert(std::make_pair(std::make_pair(std::stoul(edge->getFromJunction()->getID()), std::stoul(edge->getToJunction()->getID())), edge));
				g.add_arc(std::stoi(edge->getFromJunction()->getID()), std::stoi(edge->getToJunction()->getID()), effort, false);
				nodesWeightMap.insert(std::make_pair(std::make_pair(std::stoul(edge->getFromJunction()->getID()), std::stoul(edge->getToJunction()->getID())), effort));
				EdgeWeightMap.insert(std::make_pair(std::stoul(edge->getID()), effort));
			}
		}

		g.finalize();
		/*This part calculates the order of nodes based on the degree of each node*/
		std::vector< std::pair<size_t, Vertex> > d(g.get_n());
		std::vector<Vertex> order(g.get_n());
		preprocessing_starttime = SysUtils::getCurrentMillis();
	/*	for (Vertex v = 0; v < g.get_n(); ++v) d[v] = std::make_pair(g.get_degree(v), v);
		std::sort(d.begin(), d.end());
		for (size_t i = 0; i < g.get_n(); ++i) order[i] = d[d.size() - i - 1].second;
		preprocessing_duration += (SysUtils::getCurrentMillis() - preprocessing_starttime);
		myOrder = order;*/

		Labeling labels(g.get_n());
		//PrunedHighwayLabeling phl();

		/*This part read the order of nodes from a file*/
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Experiments/HL/hl/MtlG/MtlG.flowcutter20.order.txt");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG_TT_FC20_order");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG_TT_FC20_order_test");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG_Dist_FC20_order");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/eichstaett/eichstaett_TT_Metis_order");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/eichstaett/eichstaett_Dist_Metis_order");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/eichstaett/eichstaett-LG-Dist-Order");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/eichstaett/eichstaett-PG-Dist-Order");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/eichstaett/eichstaett-LG-TT-Order");
		myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/eichstaett/eichstaett-PG-TT-Order");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG-LG-Dist-Order");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG-PG-Dist-Order");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG-LG-TT-Order");
		//myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG-PG-TT-Order");

		preprocessing_starttime = SysUtils::getCurrentMillis();
		/*This part read the labels arealdy calculated and saved in a file*/
		//labels.read("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG-labels_TT_FC20_order_new.txt", g.get_n());
		//labels.read("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG-labels_Dist_FC20_order_new.txt", g.get_n());
		//labels.read("R:/Education/Concordia/Thesis/Experiments/HL/hl/MtlG/MtlG-labels_dist_flowcutter20_order.txt", g.get_n());
		//labels.read("R:/Education/Concordia/Thesis/Experiments/HL/hl/MtlG/MtlG-labels_dist_degree_order.txt", g.get_n());
		//labels.read("R:/Education/Concordia/Thesis/Experiments/HL/hl/MtlG/MtlG-labels_TT_degree_order.txt", g.get_n());
		//labels.read("R:/Education/Concordia/Thesis/Experiments/HL/hl/MtlG/MtlG-labels_dist.txt", g.get_n());
		//labels.read("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/TestNet2-labels_Dist_withRankId_new.txt", g.get_n());
		rank = invert_permutation(myOrder);
		/*This part build the index of the graph g with respect to the order in myOrder variable and label construction algorithm proposed by Akiba et al.*/
		Akiba ak(g);
		ak.run(myOrder, rank, labels);

		/*This part build the index of the graph g using the label greedy order and label construction algorithm proposed by Delling et al.*/
		//HHL hl(g, 8);
		//hl.run(0, myOrder, labels);

		/*This part build the index of the graph g using the path greedy order and label construction algorithm proposed by Delling et al.*/
		preprocessing_duration += (SysUtils::getCurrentMillis() - preprocessing_starttime);
		PROGRESS_BEGIN_MESSAGE("preprocessing took " + std::to_string(preprocessing_duration) + " ms.");
		/*This part sort the labels of all the nodes already calculated in the previous step*/
		//labels.sort();

		/*This part is like the sanity check of the lables*/
		//LabelingCheck lc(g, 3);
		//auto check = lc.run(labels);

		/*This part write labels into a file*/
		//labels.write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG-labels_TT_FC20_order_new.txt");
		//labels.write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/RoutingSim3_labels.txt");
		//labels.my_write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/RoutingSim3_mywrite_labels.txt");
		//labels.my_write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG-labels_TT_LG_order.txt");
		//labels.my_write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/TestNet2-labels_Dist.txt");
		//labels.my_write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/TestNet2-labels_Dist_withRankId.txt");
		//labels.my_write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/eichstaett/eichstaett-labels_degree_Dist_order.txt");
		//labels.write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/MtlG/MtlG-labels_Dist_FC20_order_new.txt");
		//labels.write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/TestNet2-labels_Dist_withRankId_new.txt");
		//labels.my_write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/TestNet2-labels_Dist_withRankId_new_mywrite.txt");


		myHLBuilder = labels;

		/*This part calculated the average size of the labels*/
		auto avg_label_size = labels.get_avg();
		PROGRESS_BEGIN_MESSAGE("The average label size is: " + std::to_string(avg_label_size) + " .");
	}

	HLRouter(const std::vector<E*>& edges, bool unbuildIsWarning, typename SUMOAbstractRouter<E, V>::Operation operation,
		const SUMOVehicleClass svc,
		const bool havePermissions, const bool haveRestrictions) :
		SUMOAbstractRouter<E, V>("HLRouterClone", unbuildIsWarning, operation, nullptr, havePermissions, haveRestrictions),
		myEdges(edges),
		myHLBuilder(myHLBuilder),
		myWeightPeriod(SUMOTime_MAX),
		myValidUntil(SUMOTime_MAX),
		mySVC(svc) {

	}


	virtual ~HLRouter() {
	}

	virtual SUMOAbstractRouter<E, V>* clone() {
		if (myWeightPeriod == SUMOTime_MAX) {
			return new HLRouter<E, V>(myEdges, this->myErrorMsgHandler == MsgHandler::getWarningInstance(), this->myOperation,
				mySVC, this->myHavePermissions, this->myHaveRestrictions);
		}
		return new HLRouter<E, V>(myEdges, this->myErrorMsgHandler == MsgHandler::getWarningInstance(), this->myOperation,
			mySVC, myWeightPeriod, this->myHavePermissions, this->myHaveRestrictions);
	}
	virtual bool compute(const E* from, const E* to, const V* const vehicle,
		SUMOTime msTime, std::vector<const E*>& into, bool silent = false) {
		assert(from != nullptr && to != nullptr);
		//if (msTime == 511000)
		//{
		//	std::cout << "check this";
		//}
		if (msTime >= myValidUntil) {
		if (msTime == 100000 )
				{
					std::cout << "check this";
				}
				//assert(myHLBuilder != nullptr); // only time independent clones do not have a builder
			while (msTime >= myValidUntil) {
				myValidUntil += myWeightPeriod;
			}
			this->CalcUpdateStart();
			//g.reset();      //added 2020-12-26
			//g.set_n(degree.size());//added 2020-12-26
			std::vector<const E*> dynamic_edges;
			for (const E* edge : myEdges) {
				if (edge->getFromJunction() != nullptr && edge->getToJunction() != nullptr) {
					//auto effort = this->getEffort(edge, nullptr, STEPS2TIME(msTime));
					//g.add_arc(std::stoul(edge->getFromJunction()->getID()), std::stoul(edge->getToJunction()->getID()), effort, false);
					dynamic_edges.push_back(edge);
				}
			}
			//g.finalize();
			Akiba ak(g);
			
			std::sort(dynamic_edges.begin(), dynamic_edges.end(), [&](auto a, auto b)->bool {return rank[std::stoul(a->getFromJunction()->getID())] < rank[std::stoul(b->getFromJunction()->getID())] && rank[std::stoul(a->getToJunction()->getID())] < rank[std::stoul(b->getToJunction()->getID())]; });
			
			for (const E * edge : dynamic_edges)
			{
				//if (edge->getFromJunction() != nullptr && edge->getToJunction() != nullptr) {
					auto effort = this->getEffort(edge, nullptr, STEPS2TIME(msTime));
					auto edge_id = std::stoul(edge->getID());
					auto s = std::stoul(edge->getFromJunction()->getID());
					auto t = std::stoul(edge->getToJunction()->getID());
					//std::cout << typeid(edge_id).name() <<" ";
					//std::cout << edge_id << " , ";
					/*if (EdgeWeightMap.find(edge_id) == EdgeWeightMap.end()) {
						std::cout << "there is an error" << " , ";
					}*/
					/*	if (!(std::stoi(edge->getID()) < 598 || std::stoi(edge->getID()) > 0))
						{
							std::cout << "check this";
						}*/
					if (effort < EdgeWeightMap[edge_id])
						//if (edge->getID() == "8") //{
						//	if (1 < EdgeWeightMap[std::make_pair(std::stoul(edge->getFromJunction()->getID()), std::stoul(edge->getToJunction()->getID()))])
					{

						//		EdgeWeightMap[std::make_pair(std::stoul(edge->getFromJunction()->getID()), std::stoul(edge->getToJunction()->getID()))] = 1;
						if (edge_id==581 || edge_id==582)
						{
							std::cout << "check this";
						}
						
						ak.Inc_PLL(std::stoul(edge->getFromJunction()->getID()), std::stoul(edge->getToJunction()->getID()), effort, nodesWeightMap, myHLBuilder, rank, myOrder);
						if (myHLBuilder.query_path(7, 142, 1).size() ==6)
						{
							std::cout << "check this";
							myHLBuilder.query_path(7, 142, 1);
						}
						EdgeWeightMap[edge_id] = effort;
						nodesWeightMap[std::pair<Vertex, Vertex>(s, t)] = effort;
						
						//std::cout << "Inc_PLL: "<<edge_id << " , ";
						//		ak.Inc_PLL(std::stoul(edge->getFromJunction()->getID()), std::stoul(edge->getToJunction()->getID()), 1, myHLBuilder, rank);
						//		myHLBuilder.my_write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/TestNet2-labels_Dist_IncPLL.txt");
						//myHLBuilder.sort();
						//myHLBuilder.my_write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/TestNet2-labels_Dist_IncPLL_new.txt");
					}
					else if (effort > EdgeWeightMap[edge_id]) {
						//if (edge->getID() == "6") {

							//if (5 > EdgeWeightMap[std::stoi(edge->getID())]) {
								//EdgeWeightMap[std::make_pair(std::stoul(edge->getFromJunction()->getID()), std::stoul(edge->getToJunction()->getID()))] = 5;
						/*if (edge_id == 92) {
							std::cout << "check this";
						}
						*/
						/*	if (edge->getFromJunction()->getID()=="17" && edge->getToJunction()->getID()=="165")
							{
								std::cout << "debug this";
							}*/
						if (edge_id == 265)
						{
							std::cout << "check this";
							myHLBuilder.query_path(131, 78, 1);
						}
						
						ak.Dec_PLL(std::stoul(edge->getFromJunction()->getID()), std::stoul(edge->getToJunction()->getID()), EdgeWeightMap[edge_id], effort, nodesWeightMap, myHLBuilder, rank, myOrder);
						//ak.Dec_PLL(std::stoul(edge->getFromJunction()->getID()), std::stoul(edge->getToJunction()->getID()), effort, myHLBuilder, rank, myOrder);
						if (myHLBuilder.query_path(7, 142, 1).size() == 6)
						{
							std::cout << "check this";
							myHLBuilder.query_path(7, 142, 1);
						}
						nodesWeightMap[std::pair<Vertex, Vertex>(s, t)] = effort;
						EdgeWeightMap[edge_id] = effort;
						//nodesWeightMap[std::pair<Vertex, Vertex>(s, t)] = effort;
						//std::cout <<"Dec_PLL: "<< edge_id << " , ";
						//ak.Dec_PLL(std::stoul(edge->getFromJunction()->getID()), std::stoul(edge->getToJunction()->getID()), 5, myHLBuilder, rank);
						//myHLBuilder.my_write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/TestNet2-labels_Dist_DecPLL_new.txt");
					//}
					//}
					//}
					}
				}
			//}
			this->CalcUpdateTime();
			//myHLBuilder.sort();
			//myHLBuilder.my_write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/TestNet2-labels_Dist_DecPLL.txt");
		}
		myVehicle = vehicle;
		unsigned source = 0;
		unsigned target = 0;
		if (from->isTazConnector() && to->isTazConnector()) {
			source = std::stoi(from->getID().substr(0, from->getID().find("-")));
			target = std::stoi(to->getID().substr(0, to->getID().find("-")));
		}
		else if (from->isNormal() && to->isNormal() && (to != from)) {
			source = std::stoi(from->getToJunction()->getID());
			target = std::stoi(to->getFromJunction()->getID());
		}
		else if (to == from) {
			source = std::stoi(from->getFromJunction()->getID());
			target = std::stoi(to->getToJunction()->getID());
		}

		this->startQuery();
		//auto distance = myHLBuilder.query(source, target);
		//this->endQuery(0);
		//this->startQuery();
	/*	if(to->getID() == "229" && from->getID()=="228")
		{
			std::cout << "check this";
		}*/
		source = rank[source];
		target = rank[target];
		if (source == 131 && target == 78) {
			std::cout << "check this";
		}
		myHLBuilder.reset_counter(source, target);
		auto path = myHLBuilder.query_path(source, target);

		this->endQuery(0);
		for (int i = 0; i < path.size() - 1; i++) {
			//into.push_back(nodeEdgeMap[std::make_pair(path[i], path[i + 1])]);
			if ((nodeEdgeMap[std::make_pair(myOrder[path[i]], myOrder[path[i + 1]])]) != nullptr) {
			into.push_back(nodeEdgeMap[std::make_pair(myOrder[path[i]], myOrder[path[i + 1]])]);
			}
			else {
				std::cout << "path is not correct";

			}
		}
		if (from->isNormal() && to->isNormal() && (to != from)) {
			into.push_back(to);
			auto it = into.begin();
			into.insert(it, from);
		}
		//if (vehicle->getID() == "2389")
		//{
		//	std::cout << "check this";
		//}
		auto result = true;
		return result;
	}




private:


	/// @brief all edges with numerical ids
	const std::vector<E*>& myEdges;


	/// @brief the validity duration of one weight interval
	const SUMOTime myWeightPeriod;

	/// @brief the validity duration of the current metric (exclusive)
	SUMOTime myValidUntil;

	/// @brief the permissions for which the hierarchy was constructed
	const SUMOVehicleClass mySVC = SVC_PASSENGER;

	const V* myVehicle;
	std::map<Vertex, int> degree;
	Graph g;
	Labeling myHLBuilder;
	std::vector< Vertex> myOrder;
	std::vector< Vertex> rank;
	std::map<std::pair<int, int>, const E*> nodeEdgeMap;
	std::vector<unsigned> mytail;
	std::vector<unsigned> myhead;
	std::vector<float> mylat;
	std::vector<float>mylon;
	unsigned node_count;
	std::map<unsigned long, double> EdgeWeightMap;
	std::map<std::pair<Vertex, Vertex>, double> nodesWeightMap;
	long long int preprocessing_duration = 0;
	long long int preprocessing_starttime = 0;


};

