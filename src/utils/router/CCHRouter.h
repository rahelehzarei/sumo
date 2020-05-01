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
//#define CHRouter_DEBUG_QUERY
//#define CHRouter_DEBUG_QUERY_PERF
class MSEdge;

// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class CHRouter
 * @brief Computes the shortest path through a customizable contraction hierarchy
 *
 * The template parameters are:
 * @param E The edge class to use (MSEdge/ROEdge)
 * @param V The vehicle class to use (MSVehicle/ROVehicle)
 *
 * The router is edge-based. It must know the number of edges for internal reasons
 *  and whether a missing connection between two given edges (unbuild route) shall
 *  be reported as an error or as a warning.
 *
 */
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
		//myHierarchyBuilder(new CHBuilder<E, V>(edges, unbuildIsWarning, svc, havePermissions)),
		myMetric(nullptr),
		myWeightPeriod(weightPeriod),
		myValidUntil(0),
		mySVC(svc) {

		unsigned i = 0;
		std::set<float> lat;
		std::set<float> lon;
		for (const MSEdge* edge : myEdges)
		{
			//if (Juncval.find(std::stoi(edge->getToJunction()->getID())) == Juncval.end())
			if (Juncval.find(edge->getFromJunction()->getID()) == Juncval.end())
			{
				
				//Juncval.insert(std::make_pair(std::stoi(edge->getFromJunction()->getID()), i));
				//Juncval.insert(std::make_pair(edge->getFromJunction()->getID(), i));
			//add from and to nodes of edge to Juncval and assign them a unique id
				Juncval.insert(std::make_pair(edge->getFromJunction()->getID(), i++));
				//i++;
			}
			if (Juncval.find(edge->getToJunction()->getID()) == Juncval.end())
			{
				Juncval.insert(std::make_pair(edge->getToJunction()->getID(), i++));
				//i++;
			}

				lat.insert((float)edge->getFromJunction()->getPosition().x());
				lon.insert((float)edge->getFromJunction()->getPosition().y());
				lat.insert((float)edge->getToJunction()->getPosition().x());
				lon.insert((float)edge->getToJunction()->getPosition().y());

			}
		//}
		for (const E* edge : myEdges)
		{
			if (edge->isNormal())
			{
				//nodeEdgeMap.insert(std::make_pair(std::make_pair(Juncval[std::stoi(edge->getFromJunction()->getID())], Juncval[std::stoi(edge->getToJunction()->getID())]), edge));
				nodeEdgeMap.insert(std::make_pair(std::make_pair(Juncval[edge->getFromJunction()->getID()], Juncval[edge->getToJunction()->getID()]), edge));
			}
		}

		for (const E* edge : myEdges)
		{
			//mytail.push_back(Juncval[std::stoi(edge->getFromJunction()->getID())]);
			//myhead.push_back(Juncval[std::stoi(edge->getToJunction()->getID())]);
			mytail.push_back(Juncval[edge->getFromJunction()->getID()]);
			myhead.push_back(Juncval[edge->getToJunction()->getID()]);
			//}
		}

		std::copy(lat.begin(), lat.end(), std::back_inserter(mylat));
		std::copy(lon.begin(), lon.end(), std::back_inserter(mylon));

		node_count = Juncval.size();
		//node_count = mytail.size();
		//if (myHierarchyBuilder)
		myOrder = compute_nested_node_dissection_order_using_inertial_flow(node_count, mytail, myhead, mylat, mylon);
		myHierarchyBuilder = new CustomizableContractionHierarchy(myOrder, mytail, myhead);

	}

	/** @brief Cloning constructor, should be used only for time independent instances which build a hierarchy only once
	 */
	CCHRouter(const std::vector<E*>& edges, bool unbuildIsWarning, typename SUMOAbstractRouter<E, V>::Operation operation,
		const SUMOVehicleClass svc,
		//const typename CHBuilder<E, V>::Hierarchy* hierarchy,
		const bool havePermissions, const bool haveRestrictions) :
		SUMOAbstractRouter<E, V>("CCHRouterClone", unbuildIsWarning, operation, nullptr, havePermissions, haveRestrictions),
		myEdges(edges),
		myHierarchyBuilder(myHierarchyBuilder),
		myMetric(nullptr),
		myWeightPeriod(SUMOTime_MAX),
		myValidUntil(SUMOTime_MAX),
		mySVC(svc) {
	}

	/// Destructor
	virtual ~CCHRouter() {
		if (myHierarchyBuilder != nullptr) {
			delete myMetric;
			delete myHierarchyBuilder;
		}
	}


	virtual SUMOAbstractRouter<E, V>* clone() {
		if (myWeightPeriod == SUMOTime_MAX) {
			// we only need one hierarchy
			return new CCHRouter<E, V>(myEdges, this->myErrorMsgHandler == MsgHandler::getWarningInstance(), this->myOperation,
				mySVC, this->myHavePermissions, this->myHaveRestrictions);
		}
		return new CCHRouter<E, V>(myEdges, this->myErrorMsgHandler == MsgHandler::getWarningInstance(), this->myOperation,
			mySVC, myWeightPeriod, this->myHavePermissions, this->myHaveRestrictions);
	}

	/** @brief Builds the route between the given edges using the minimum traveltime in the contracted graph
	 * @note: since the contracted graph is static (weights averaged over time)
	 * the computed routes only approximated shortest paths in the real graph
	 * */
	virtual bool compute(const E* from, const E* to, const V* const vehicle,
		SUMOTime msTime, std::vector<const E*>& into, bool silent = false) {
		assert(from != nullptr && to != nullptr);
		// assert(myHierarchyBuilder.mySPTree->validatePermissions() || vehicle->getVClass() == mySVC || mySVC == SVC_IGNORING);
		// do we need to rebuild the hierarchy?
		if (msTime >= myValidUntil) {
			assert(myHierarchyBuilder != nullptr); // only time independent clones do not have a builder
			while (msTime >= myValidUntil) {
				myValidUntil += myWeightPeriod;
			}

			if (myMetric == nullptr)
			{
				myVehicle = vehicle;
				setcurrent_weight();
				myMetric = new CustomizableContractionHierarchyMetric(*myHierarchyBuilder, current_weight);
				myMetric->customize();
			}
			else
			{
				setcurrent_weight();
				myMetric->reset(*myHierarchyBuilder, current_weight);
			}


			// ready for routing
			this->startQuery();
			CustomizableContractionHierarchyQuery query = CustomizableContractionHierarchyQuery(*myMetric);
			//unsigned source = Juncval[std::stoi(from->getFromJunction()->getID())];
			unsigned source = Juncval[from->getToJunction()->getID()];
			//unsigned target = Juncval[std::stoi(to->getToJunction()->getID())];
			unsigned target = Juncval[to->getFromJunction()->getID()];
			query.reset().add_source(source).add_target(target).run().get_distance();
			auto visited_nodes = query.number_of_visited_nodes();
			auto arcpath = query.get_arc_path();
			auto path = query.get_node_path();

			//normalEdges = myEdges[isnormal() == true];
			for (int i = 0; i < path.size() - 1; i++)
			{
				into.push_back(nodeEdgeMap[std::make_pair(path[i], path[i + 1])]);
			}

			into.push_back(to);
			auto it = into.begin();
			into.insert(it, from);
			auto result = true;

			this->endQuery(visited_nodes);
			return result;
		}
	}


	/// normal routing methods

	/// Builds the path from marked edges
	//void buildPathFromMeeting(Meeting meeting, std::vector<const E*>& into) const {
	//	std::deque<const E*> tmp;
	//	const auto* backtrack = meeting.first;
	//	while (backtrack != 0) {
	//		tmp.push_front((E*)backtrack->edge);  // !!!
	//		backtrack = backtrack->prev;
	//	}
	//	backtrack = meeting.second->prev; // don't use central edge twice
	//	while (backtrack != 0) {
	//		tmp.push_back((E*)backtrack->edge);  // !!!
	//		backtrack = backtrack->prev;
	//	}
	//	// expand shortcuts
	//	const E* prev = 0;
	//	while (!tmp.empty()) {
	//		const E* cur = tmp.front();
	//		tmp.pop_front();
	//		if (prev == 0) {
	//			into.push_back(cur);
	//			prev = cur;
	//		}
	//		else {
	//			const E* via = getVia(prev, cur);
	//			if (via == 0) {
	//				into.push_back(cur);
	//				prev = cur;
	//			}
	//			else {
	//				tmp.push_front(cur);
	//				tmp.push_front(via);
	//			}
	//		}
	//	}
	//}

//	// retrieve the via edge for a shortcut
//	const E* getVia(const E* forwardFrom, const E* forwardTo) const {
//		typename CHBuilder<E, V>::ConstEdgePair forward(forwardFrom, forwardTo);
//		typename CHBuilder<E, V>::ShortcutVia::const_iterator it = myHierarchy->shortcuts.find(forward);
//		if (it != myHierarchy->shortcuts.end()) {
//			return it->second;
//		}
//		else {
//			return 0;
//		}
//	}

	//setcurrent_weight(const MSEdge* edge, const SUMOVehicle* veh, double time )
	//{
	void CCHRouter::setcurrent_weight()
		//const double time_seconds = STEPS2TIME(msTime); // timelines store seconds!

	{
		current_weight.clear();
		for (const MSEdge * edge : myEdges)
		{
			current_weight.push_back(edge->getCurrentTravelTime());
		}
	}

private:
	/// @brief all edges with numerical ids
	const std::vector<E*>& myEdges;

	/// @brief the unidirectional search queues
	//Unidirectional myForwardSearch;
	//Unidirectional myBackwardSearch;

	/*CHBuilder<E, V>* myHierarchyBuilder;
	const typename CHBuilder<E, V>::Hierarchy* myHierarchy;*/

	/// @brief the validity duration of one weight interval
	const SUMOTime myWeightPeriod;

	/// @brief the validity duration of the current hierarchy (exclusive)
	SUMOTime myValidUntil;

	/// @brief the permissions for which the hierarchy was constructed
	const SUMOVehicleClass mySVC;

	const V* myVehicle;


	CustomizableContractionHierarchy* myHierarchyBuilder;
	CustomizableContractionHierarchyMetric* myMetric;
	std::vector<unsigned> current_weight;
	//std::map<unsigned, unsigned> Juncval;
	std::map<std::string, unsigned> Juncval;
	//std::map<std::pair<unsigned, unsigned>, const E*> nodeEdgeMap;
	std::map<std::pair<unsigned, unsigned>, const E*> nodeEdgeMap;
	std::vector<unsigned>myOrder;
	std::vector<unsigned> mytail;
	std::vector<unsigned> myhead;
	std::vector<float> mylat;
	std::vector<float>mylon;
	unsigned node_count;
};
