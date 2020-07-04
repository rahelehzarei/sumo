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
#include <ostream>
#include <utils/common/SysUtils.h>
#include <utils/common/MsgHandler.h>
#include <utils/common/StdDefs.h>
#include <utils/router/SUMOAbstractRouter.h>
#include "../../microsim/MSNet.h"
#include "CHBuilder.h"
#include "../RoutingKitdll/nested_dissection.h"
#include "../RoutingKitdll/contraction_hierarchy.h"
#include "../RoutingKitdll/customizable_contraction_hierarchy.h"
#include "../RoutingKitdll/inverse_vector.h"
#include "../RoutingKitdll/graph_util.h"
#include "../RoutingKitdll/timer.h"
#include "../RoutingKitdll/sort.h"
#include "../RoutingKitdll/vector_io.h"


using namespace RoutingKit;

class MSEdge;
class MSBaseVehicle;
class MSRoutingEngine;


template<class E, class V>
class CCHRouter : public SUMOAbstractRouter<E, V> {

public:

	CCHRouter(const std::vector<E*>& edges, bool unbuildIsWarning, typename SUMOAbstractRouter<E, V>::Operation operation,
		const SUMOVehicleClass svc,
		SUMOTime weightPeriod,
		const bool havePermissions, const bool haveRestrictions) :
		SUMOAbstractRouter<E, V>("CCHRouter", unbuildIsWarning, operation, nullptr, havePermissions, haveRestrictions),
		myEdges(edges),
		myMetric(nullptr),
		myWeightPeriod(weightPeriod),
		myValidUntil(0),
		mySVC(SVC_PASSENGER) {
		unsigned i = 0;
		std::set<float> lat;
		std::set<float> lon;
		std::set <unsigned> tail;
		std::set <unsigned> head;
		//std::copy_if(myEdges.begin(), myEdges.end(),  std::back_inserter(hasFromToNode) , [](const E* e) {return (e->getFromJunction() != nullptr && e->getToJunction() != nullptr); });
		//std::copy_if(myEdges.begin(), myEdges.end(), std::back_inserter(hasFromToNode), [](const E* e) {return ((e->isNormal() || e->isInternal()) && (e->getEdgeType() == "highway.tertiary"||e->getEdgeType()=="highway.path" ||e->getEdgeType() == "highway.secondary" || e->getEdgeType() == "highway.secondary_link" || e->getEdgeType() == "highway.primary" || e->getEdgeType() == "highway.primary_link" ||e->getEdgeType() =="highway.motorway" || e->getEdgeType() == "highway.motorway_link" || e->getEdgeType() == "highway.residential")); });  //&& (e->getPermissions()==50593791 || e->getPermissions() == 50593791 || e->getPermissions() == 50495455 || e->getPermissions() == NULL)
		//std::copy_if(myEdges.begin(), myEdges.end(), std::back_inserter(hasFromToNode), [](const E* e) {return (e->isNormal() || e->isInternal()); });
		//std::copy_if(myEdges.begin(), myEdges.end(), std::back_inserter(hasFromToNode), [](const E* e) {return (e->getFromJunction() != nullptr && e->getToJunction() != nullptr); });
		//std::copy(myEdges.begin(), myEdges.end(), std::back_inserter(hasFromToNode));
		for (const E* edge : myEdges) {
			neighbors.clear();

			if (edge->getFromJunction() != nullptr) {
				Junc_to_LatLon.insert(std::make_pair(edge->getFromJunction()->getID(), std::make_pair((float)edge->getFromJunction()->getPosition().x(), (float)edge->getFromJunction()->getPosition().y())));
				for (const E* n : edge->getPredecessors()) {
					if (n->getToJunction() !=nullptr){
						neighbors.insert(n->getFromJunction()->getID());
					}
				}
				neighbors.insert(edge->getToJunction()->getID());
				Junc_for_metis.insert(std::make_pair(edge->getFromJunction()->getID(), neighbors));
				//mylat.push_back((float)edge->getFromJunction()->getPosition().x());
				//mylon.push_back((float)edge->getFromJunction()->getPosition().y());
				mytail.push_back(std::stoul(edge->getFromJunction()->getID()));	
			}
			neighbors.clear();
			if (edge->getToJunction() != nullptr){
				Junc_to_LatLon.insert(std::make_pair(edge->getToJunction()->getID(), std::make_pair((float)edge->getToJunction()->getPosition().x(), (float)edge->getToJunction()->getPosition().y())));
				for (const E* n: edge->getSuccessors(mySVC)){
					if (n->getToJunction() != nullptr) {
						neighbors.insert(n->getToJunction()->getID());
					}
				}
				neighbors.insert(edge->getFromJunction()->getID());
				Junc_for_metis.insert(std::make_pair(edge->getToJunction()->getID(), neighbors));
				//mylat.push_back((float)edge->getToJunction()->getPosition().x());
				//mylon.push_back((float)edge->getToJunction()->getPosition().y());
				myhead.push_back(std::stoul(edge->getToJunction()->getID()));
			}
		}

		for (const E* edge : myEdges) {
			if (edge->getFromJunction() != nullptr && edge->getToJunction() != nullptr) {
				nodeEdgeMap.insert(std::make_pair(std::make_pair(std::stoi(edge->getFromJunction()->getID()), std::stoi(edge->getToJunction()->getID())), edge));
			}
		}
		for (const auto j : Junc_to_LatLon) {
				mylat.push_back(j.second.first);
				mylon.push_back(j.second.second);
			}
		
		//make_metris_format("R:/Education/Concordia/Thesis/Experiments/InertialFlowCutter/MtlG/metis.txt", Junc_for_metis);

		std::copy(lat.begin(), lat.end(), std::back_inserter(mylat));
		std::copy(lon.begin(), lon.end(), std::back_inserter(mylon));
		std::set<unsigned> nodecount(mytail.begin(), mytail.end());
		std::copy(myhead.begin(), myhead.end(), std::inserter(nodecount, nodecount.end()));

		node_count = nodecount.size();

		setcurrent_weight(MSNet::getInstance()->getCurrentTimeStep(), nullptr);

		//auto input_arc_id = compute_sort_permutation_using_less(mytail);
		//mytail = apply_permutation(input_arc_id, mytail);
		//myhead = apply_permutation(input_arc_id, myhead);
		//std::vector<unsigned> first_out = invert_vector(mytail, node_count);
		//save_vector("R:/Education/Concordia/Thesis/Experiments/InertialFlowCutter/MtlG/first_out.txt", first_out);
		//save_vector("R:/Education/Concordia/Thesis/Experiments/InertialFlowCutter/MtlG/tail.txt", mytail);
		//save_vector("R:/Education/Concordia/Thesis/Experiments/InertialFlowCutter/MtlG/myhead.txt", myhead);
		//save_vector("R:/Education/Concordia/Thesis/Experiments/InertialFlowCutter/MtlG/travel_time.txt", current_weight);
		//save_vector("R:/Education/Concordia/Thesis/Experiments/InertialFlowCutter/MtlG/latitude.txt", mylat);
		//save_vector("R:/Education/Concordia/Thesis/Experiments/InertialFlowCutter/MtlG/longitude.txt", mylon);

		/*order nodes using nested dissection and make contraction hierarchy using this order, tails and heads*/
		//myOrder = compute_nested_node_dissection_order_using_inertial_flow(node_count, mytail, myhead, mylat, mylon);
		//auto gf = make_graph_fragment(node_count, mytail, myhead);
		
		//myOrder = compute_nested_node_dissection_order(gf, ;
		myOrder = load_vector<unsigned>("R:/Education/Concordia/Thesis/Experiments/InertialFlowCutter/MtlG/MtlG.metis.order");
		auto StartTime = SysUtils::getCurrentMillis();
		myHierarchyBuilder = new CustomizableContractionHierarchy(myOrder, mytail, myhead);
		auto duration = (SysUtils::getCurrentMillis() - StartTime); 

		myMetric = new CustomizableContractionHierarchyMetric(*myHierarchyBuilder, current_weight);


		/*use ch query and witness search*/
		//ch = myMetric->build_contraction_hierarchy_using_perfect_witness_search();
		//query = ContractionHierarchyQuery(ch);

		/*use cch query*/
		myMetric->customize();
		query = CustomizableContractionHierarchyQuery(*myMetric);

		/*RoutingKit CH*/
	/*	auto StartTime = SysUtils::getCurrentMillis();
		ch = ContractionHierarchy::build(node_count, mytail, myhead, current_weight);
		auto duration = (SysUtils::getCurrentMillis() - StartTime);
		ch.save_file("Saved_CH.txt");*/


		//std::ofstream first_out_file("../RoutingKitdll/first_out.txt");
		//for (auto i : first_out) {
		//	first_out_file << i << ",";
		//}
		//std::cout << "first_out: " << first_out.size() << first_out ;
		//std::cout << "Head: " << myhead.size() << m yhead ;
		//std::cout << "current_weight: " << current_weight.size() << current_weight;
		//std::cout << "latitude: " << mylat.size() << mylat;
		//std::cout << "longitude: " << mylon.size() << mylon;

	}



	/** @brief Cloning constructor, should be used only for time independent instances which build a hierarchy only once
	 */
	CCHRouter(const std::vector<E*>& edges, bool unbuildIsWarning, typename SUMOAbstractRouter<E, V>::Operation operation,
		const SUMOVehicleClass svc,
		const bool havePermissions, const bool haveRestrictions) :
		SUMOAbstractRouter<E, V>("CCHRouterClone", unbuildIsWarning, operation, nullptr, havePermissions, haveRestrictions),
		myEdges(edges),
		//myHierarchyBuilder(myHierarchyBuilder),
		myMetric(nullptr),
		myWeightPeriod(SUMOTime_MAX),
		myValidUntil(SUMOTime_MAX),
		mySVC(svc) {
	}

	virtual ~CCHRouter() {
		if (myHierarchyBuilder != nullptr) {
			delete myMetric;
			delete myHierarchyBuilder;
		}
	}

	virtual SUMOAbstractRouter<E, V>* clone() {
		if (myWeightPeriod == SUMOTime_MAX) {
			return new CCHRouter<E, V>(myEdges, this->myErrorMsgHandler == MsgHandler::getWarningInstance(), this->myOperation,
				mySVC, this->myHavePermissions, this->myHaveRestrictions);
		}
		return new CCHRouter<E, V>(myEdges, this->myErrorMsgHandler == MsgHandler::getWarningInstance(), this->myOperation,
			mySVC, myWeightPeriod, this->myHavePermissions, this->myHaveRestrictions);
	}


	virtual bool compute(const E* from, const E* to, const V* const vehicle,
		SUMOTime msTime, std::vector<const E*>& into, bool silent = false) {
		assert(from != nullptr && to != nullptr);
		if (msTime >= myValidUntil) {
			assert(myHierarchyBuilder != nullptr); // only time independent clones do not have a builder
			while (msTime >= myValidUntil) {
				myValidUntil += myWeightPeriod;
			}
			setcurrent_weight((myValidUntil - myWeightPeriod), vehicle);
			/*use witness search and CH*/
			//myMetric->reset(current_weight);
			//ch = myMetric->build_contraction_hierarchy_using_perfect_witness_search();
			//query.reset(ch);

			/*use CCH query*/
			myMetric->reset(*myHierarchyBuilder, current_weight);
			myMetric->customize();
			

			/*RoutingKit CH*/
			//ch = ContractionHierarchy::build(node_count, mytail, myhead, current_weight);
			//query.reset(ch);
		}

		myVehicle = vehicle;
		/*gets source and target from Juncval, where ther is a map between SUMO IDs and CCH node's IDs, to run edge to edge shortest path query*/
		unsigned source = 0;
		unsigned target = 0;
		if (from->isTazConnector() && to->isTazConnector()) {
			source = std::stoi(from->getID().substr(0, from->getID().find("-")));
			target = std::stoi(to->getID().substr(0, to->getID().find("-")));
		}
		else if (from->isNormal() && to->isNormal()) {
			source = std::stoi(from->getToJunction()->getID());
			target = std::stoi(to->getFromJunction()->getID());
		}

		query.reset().add_source(source).add_target(target);
		this->startQuery();
		query.run();  //stall-on-demand query
		//query.run_basic();
		
		//start building bath from nodes in the path
		auto path = query.get_node_path();
		for (int i = 0; i < path.size() - 1; i++) {
			into.push_back(nodeEdgeMap[std::make_pair(path[i], path[i + 1])]);
		}
		if (from->isNormal() && to->isNormal()) {
			into.push_back(to);
			auto it = into.begin();
			into.insert(it, from);
		}
		/*calculate number of visited node in ch and witness seach setup */
		//this->endQuery(query.get_visited_node_number());

		/*calculate number of visited node in cch and elimination tree seach setup*/
		auto visited_nodes = std::count(query.in_forward_search_space.begin(), query.in_forward_search_space.end(), true); //count number of visited node
		this->endQuery(visited_nodes);
		auto result = true;
		return result;

	}

	void CCHRouter::setcurrent_weight(SUMOTime msTime, const V* const vehicle)
	{
		current_weight.clear();
		for (const E * edge : myEdges)
		{
			if (edge->getFromJunction() != nullptr && edge->getToJunction() != nullptr) {
				current_weight.push_back(this->getEffort(edge, vehicle, STEPS2TIME(msTime)));
				//current_weight.push_back(edge->getLength());
			}

		}
	}

	void CCHRouter::make_metris_format(const std::string&file_name, const std::map<std::string, std::set<std::string>>& junction_to_metise)
	{
		int edge_count = 0;
		assert(junction_to_metise.size() != 0);
		for (const auto &en : junction_to_metise) {
		  	 edge_count += en.second.size();
		}
		edge_count = edge_count / 2;

		std::ofstream out(file_name);
		if (!out)
			throw std::runtime_error("Can not open \"" + file_name + "\" for writing.");
		
		out << std::to_string(junction_to_metise.size()) << " " << std::to_string(edge_count)<< "\n" ;
		for (const auto &m : junction_to_metise)
		{
			//out << m.first << ": ";
			for (const auto &n : m.second)
			{
				auto v = std::to_string(std::stoi(n) + 1);
				out << v << " " ;
			}
			out << "\n";
		}
	}


private:

	/// @brief all edges with numerical ids
	const std::vector<E*>& myEdges;

	// all edges with fromJunction and ToJunction value
	std::vector<E*> hasFromToNode;

	/// @brief the validity duration of one weight interval
	const SUMOTime myWeightPeriod;

	/// @brief the validity duration of the current metric (exclusive)
	SUMOTime myValidUntil;

	/// @brief the permissions for which the hierarchy was constructed
	const SUMOVehicleClass mySVC = SVC_PASSENGER;

	const V* myVehicle;

	std::map<double, const E*> test_weight;
	CustomizableContractionHierarchy* myHierarchyBuilder;
	CustomizableContractionHierarchyQuery query;
	//ContractionHierarchy ch;
	//ContractionHierarchyQuery query;
	CustomizableContractionHierarchyMetric* myMetric;
	std::vector<double> current_weight;
	std::map<std::string, unsigned> Juncval;
	std::map<std::pair<int, int>, const E*> nodeEdgeMap;
	std::vector<unsigned>myOrder;
	std::vector<unsigned> mytail;
	std::vector<unsigned> myhead;
	std::vector<float> mylat;
	std::vector<float>mylon;
	unsigned node_count;
	std::set<std::string> neighbors;
	std::map<std::string, std::set<std::string>> Junc_for_metis;
	std::map<std::string, std::pair<float, float>> Junc_to_LatLon;
};



template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
	os << "[";
	for (int i = 0; i < v.size(); ++i) {
		os << v[i];
		if (i != v.size() - 1)
			os << ", ";
	}
	os << "]\n";
	return os;
}
