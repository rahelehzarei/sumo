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
#include "CHBuilder.h"
#include "../RoutingKitdll/nested_dissection.h"
#include "../RoutingKitdll/customizable_contraction_hierarchy.h"
#include "../RoutingKitdll/graph_util.h"

using namespace RoutingKit;

class MSEdge;
class MSBaseVehicle;
class MSRoutingEngine;
//SUMOAbstractRouter<MSEdge, SUMOVehicle>::Operation MSRoutingEngine::myEffortFunc = &MSRoutingEngine::getEffort;

template<class E, class V>
class CCHRouter : public SUMOAbstractRouter<E, V> {

public:

	/** @brief Constructor
	 * @param[in] validatePermissions Whether a multi-permission hierarchy shall be built
	 *            If set to false, the net is pruned in synchronize() and the
	 *            hierarchy is tailored to the svc
	 */
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
		/*define map between sumo node ID and CCH node ID (which is between 0 to number of nodes -1),
		tail and head of all the edges (adjacency array), latitute and longitute of each node*/
		unsigned i = 0;
		std::vector<float> lat;
		std::vector<float> lon;
		std::map<std::string, SVCPermissions> perm;
		for (const E* edge : myEdges) {
			if (edge->isNormal() || edge->isInternal()) {
				perm.insert(std::make_pair(edge->getEdgeType(), edge->getPermissions()));
			}
		}
		//std::copy_if(myEdges.begin(), myEdges.end(),  std::back_inserter(hasFromToNode) , [](const E* e) {return (e->getFromJunction() != nullptr && e->getToJunction() != nullptr); });
		//std::copy_if(myEdges.begin(), myEdges.end(), std::back_inserter(hasFromToNode), [](const E* e) {return ((e->isNormal() || e->isInternal()) && (e->getEdgeType() == "highway.tertiary"||e->getEdgeType()=="highway.path" ||e->getEdgeType() == "highway.secondary" || e->getEdgeType() == "highway.secondary_link" || e->getEdgeType() == "highway.primary" || e->getEdgeType() == "highway.primary_link" ||e->getEdgeType() =="highway.motorway" || e->getEdgeType() == "highway.motorway_link" || e->getEdgeType() == "highway.residential")); });  //&& (e->getPermissions()==50593791 || e->getPermissions() == 50593791 || e->getPermissions() == 50495455 || e->getPermissions() == NULL)
		std::copy_if(myEdges.begin(), myEdges.end(), std::back_inserter(hasFromToNode), [](const E* e) {return (e->isNormal() || e->isInternal()); });
		//std::copy_if(myEdges.begin(), myEdges.end(), std::back_inserter(hasFromToNode), [](const E* e) {return (e->getFromJunction() != nullptr && e->getToJunction() != nullptr); });

		/*creates a map between SUMO junction IDs and node IDs in CCH in edge-to-edge routing (from 0 to number of nodes -1)*/
		for (const E* edge : hasFromToNode) {
			if (Juncval.find(edge->getFromJunction()->getID()) == Juncval.end()) {
				Juncval.insert(std::make_pair(edge->getFromJunction()->getID(), i++));
			} if (Juncval.find(edge->getToJunction()->getID()) == Juncval.end()) {
				Juncval.insert(std::make_pair(edge->getToJunction()->getID(), i++));}
			lat.push_back((float)edge->getFromJunction()->getPosition().x());
			lon.push_back((float)edge->getFromJunction()->getPosition().y());
			lat.push_back((float)edge->getToJunction()->getPosition().x());
			lon.push_back((float)edge->getToJunction()->getPosition().y());
		}

		for (const E* edge : hasFromToNode) {
			if (edge->isNormal()) {
				nodeEdgeMap.insert(std::make_pair(std::make_pair(Juncval[edge->getFromJunction()->getID()], Juncval[edge->getToJunction()->getID()]), edge));
			}
		}
		for (const E* edge : hasFromToNode) {
			mytail.push_back(Juncval[edge->getFromJunction()->getID()]);
			myhead.push_back(Juncval[edge->getToJunction()->getID()]);
		}

		std::copy(lat.begin(), lat.end(), std::back_inserter(mylat));
		std::copy(lon.begin(), lon.end(), std::back_inserter(mylon));

		node_count = Juncval.size();

		/*order nodes using nested dissection and make contraction hierarchy using this order, tails and heads*/
		myOrder = compute_nested_node_dissection_order_using_inertial_flow(node_count, mytail, myhead, mylat, mylon);
		myHierarchyBuilder = new CustomizableContractionHierarchy(myOrder, mytail, myhead);

	}

	/** @brief Cloning constructor, should be used only for time independent instances which build a hierarchy only once
	 */
	CCHRouter(const std::vector<E*>& edges, bool unbuildIsWarning, typename SUMOAbstractRouter<E, V>::Operation operation,
		const SUMOVehicleClass svc,
		const bool havePermissions, const bool haveRestrictions) :
		SUMOAbstractRouter<E, V>("CCHRouterClone", unbuildIsWarning, operation, nullptr, havePermissions, haveRestrictions),
		myEdges(edges),
		myHierarchyBuilder(myHierarchyBuilder),
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

	/** @brief Builds the route between the given edges using the minimum traveltime in the contracted graph
	 **/
	virtual bool compute(const E* from, const E* to, const V* const vehicle,
		SUMOTime msTime, std::vector<const E*>& into, bool silent = false) {
		assert(from != nullptr && to != nullptr);
		if (msTime >= myValidUntil) {
			assert(myHierarchyBuilder != nullptr); // only time independent clones do not have a builder
			while (msTime >= myValidUntil) {
				myValidUntil += myWeightPeriod;
			}
			if (myMetric == nullptr) {
				myVehicle = vehicle;
				setcurrent_weight(msTime);
				myMetric = new CustomizableContractionHierarchyMetric(*myHierarchyBuilder, current_weight);
				myMetric->customize();
			}
			else {
				std::vector<unsigned> prev_weight = current_weight;
				//std::map<int,std::pair<unsigned,unsigned>> diff_weight;
				setcurrent_weight(msTime);
				/*if (prev_weight == current_weight) 
				{ std::cout<< "matched"; }*/
			/*	for (int i=0; i< prev_weight.size(); i++){
					if (prev_weight[i] != current_weight[i])
						diff_weight.insert(std::make_pair(i , std::make_pair(prev_weight[i] , current_weight[i])));
				}*/ 
				myMetric->reset(*myHierarchyBuilder, current_weight);
				myMetric->customize();
			}
		}
			// ready for routing
			this->startQuery();
			CustomizableContractionHierarchyQuery query = CustomizableContractionHierarchyQuery(*myMetric);
			/*gets source and target from Juncval, where ther is a map between SUMO IDs and CCH node's IDs, to run edge to edge shortest path query*/
			unsigned source = 0;
			unsigned target = 0;
			if (from->isTazConnector() && to->isTazConnector()) {
				 source = Juncval[from->getID().substr(0, from->getID().find("-"))] ;
				 target = Juncval[to->getID().substr(0, to->getID().find("-"))];
			}
			else if (from->isNormal() && to->isNormal()) {
				source = Juncval[from->getToJunction()->getID()];
				target = Juncval[to->getFromJunction()->getID()];
			}
			/*gets source and target to run node to node shortest path query*/

			//unsigned source = Juncval[from->getID().substr(0, from->getID().find("-"))];
			//unsigned target = Juncval[to->getID().substr(0, to->getID().find("-"))];
			query.reset().add_source(source).add_target(target).run().get_distance();
			auto visited_nodes = query.number_of_visited_nodes();
			this->endQuery(visited_nodes);
			//query ended

			//start building bath from nodes in the path
			auto path = query.get_node_path();
			for (int i = 0; i < path.size() - 1; i++){
				into.push_back(nodeEdgeMap[std::make_pair(path[i], path[i + 1])]);
			}
			if (from->isNormal() && to->isNormal()) {
				into.push_back(to);
				auto it = into.begin();
				into.insert(it, from);
			}

	//for real city map
	//check if all the sequence of edges in the path are in the list of successors of previous their edge
	//if not, fix the route
	//auto tmpinto(into);
	//const E* edgeToInsert;
	//const SUMOVehicleClass vClass = vehicle == nullptr ? SVC_IGNORING : vehicle->getVClass();
	//for (int i = 0; i < tmpinto.size()-1; i++) {
	//	std::vector<const E*> followerInfo;
	//	followerInfo.clear();
	//	for (const std::pair<const E*, const E*>& follower : tmpinto[i]->getViaSuccessors(vClass)) {
	//		followerInfo.push_back(follower.first);
	//	}
	//	//if next edge in into list is not in the successor list of current edge, pick another edge and insert it in into vector
	//	auto minEffort = INFINITY;
	//	if (std::find(followerInfo.begin(), followerInfo.end(), tmpinto[i+1]) == followerInfo.end()) {
	//		for (const E* f : followerInfo) {
	//			if (f->prohibits(vehicle) || f->isTazConnector()) {
	//				continue;
	//			}
	//			auto oldEffort = getEffort(tmpinto[i], nullptr, STEPS2TIME(msTime));
	//			auto myCurrentEffort = oldEffort + getEffort(f, nullptr, STEPS2TIME(msTime));
	//			if (myCurrentEffort < minEffort) {
	//				minEffort = myCurrentEffort;
	//				edgeToInsert = f;
	//			}
	//			else { continue; }

	//		}
	//		tmpinto.insert(std::find (tmpinto.begin(), tmpinto.end(), tmpinto[i+1]), edgeToInsert);
	//	}

	//}
	//into =tmpinto;
	auto result = true;
	return result;
		
	}

	void CCHRouter::setcurrent_weight(SUMOTime msTime)
	{
		current_weight.clear();
		for (const E * edge : hasFromToNode)
		{
			//test_weight.insert(std::make_pair(getEffort(edge, nullptr, STEPS2TIME(msTime)), edge));
			current_weight.push_back(getEffort(edge, nullptr, STEPS2TIME(msTime)));
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
	CustomizableContractionHierarchyMetric* myMetric;
	std::vector<unsigned> current_weight;
	std::map<std::string, unsigned> Juncval;
	std::map<std::pair<unsigned, unsigned>, const E*> nodeEdgeMap;
	std::vector<unsigned>myOrder;
	std::vector<unsigned> mytail;
	std::vector<unsigned> myhead;
	std::vector<float> mylat;
	std::vector<float>mylon;
	unsigned node_count;
};
