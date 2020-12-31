// Akiba et al. presented a 'pruned labeling' algorithm to build Hierarchical Hub Labels from a vertex order.
// This file contains Akiba et. al. algorithm implementation
//
// Copyright (c) 2014, 2015 savrus
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "graph.hpp"
#include "dijkstra.hpp"
#include "labeling.hpp"
#include <vector>
#include <cassert>
#include <iterator>
#include <set>
#include <algorithm>
#include <iterator>

namespace hl {

class Akiba : BasicDijkstra {

    void iteration(size_t i, bool forward, std::vector<Vertex> &order, std::vector<Vertex> &rank, Labeling &labeling) {
        clear();
        Vertex v = order[i];
        distance[v] = 0;
        update(v, 0);
        while (!queue.empty()) {
            Vertex u = queue.pop();
            Distance d = distance[u] ;
			Vertex p = parent[u];
            //labeling.add(u, !forward, i, d , p);
		/*	if (rank[v] == 11){
				std::cout << "check this";
			}*/
			labeling.add(rank[u], !forward, i, d, p != none ? rank[p] : p);
            for (Graph::arc_iterator a = g->begin(u, forward), end = g->end(u, forward); a < end; ++a) {
                Distance dd = d + a->length ;
                assert(dd > d && dd < infty);
			/*	if (rank[v] == 79 && rank[a->head] == 11) {
					std::cout << "check this";
				}*/
                if (dd < distance[a->head] && trunc(dd * 10000)/10000 < trunc(labeling.query(rank[v], rank[a->head], forward) * 10000)/10000 )
					update(a->head, dd ,u);
            }
        }
    }

	/*this function add a hub to the label of target in the direction dir. We need this function to update the labels in Incremental hub labeling in the case that the target
	doesn't already contain the hub in its label. We start the Dijkstra from node a(if the changed weight is for the edge a->b) considering the distance from hub of a to the node b or its neighbor(whatever the target is)
	*/
	void iteration_PLL(unsigned int hub, unsigned int origin, unsigned int par ,double dist, int target, bool dir, std::map<std::pair<unsigned, unsigned>, double>  & nodesWeightMap, std::vector<Vertex> &order, std::vector<Vertex> &rank, Labeling &labeling) {
		KHeap<Vertex, Distance> queue_PLL(g->get_n());
		auto n = g->get_n();
		std::vector<bool> is_dirty_PLL(n,false);          
		std::vector<Vertex> dirty_PLL(n,false);
		std::vector<double> distance_PLL(n, infty);
		std::vector<Vertex> parent_PLL(n, none);
		Vertex v = hub;

		//update(order[origin], dist, order[origin]);
		{
			
			//distance_PLL[order[origin]] = dist;	
			distance_PLL[order[v]] = 0;
			//parent_PLL[order[origin]] = order[par];
			parent_PLL[order[v]] = order[v];
			//queue_PLL.update(order[origin], dist);
			queue_PLL.update(order[v], 0);
			/*if (!is_dirty_PLL[order[origin]]) {
				dirty_PLL.push_back(order[origin]); is_dirty_PLL[order[origin]] = true;
			}*/
			if (!is_dirty_PLL[order[v]]) {
				dirty_PLL.push_back(order[v]); is_dirty_PLL[order[v]] = true;
			}
		}
		while (!queue_PLL.empty()) {
			Vertex u = queue_PLL.pop();
			Distance d = distance_PLL[u] ;
			Vertex p = parent_PLL[u];
			//labeling.add(u, !forward, i, d , p);
			if (rank[u] == target){
				if (target == 38 && v==35) {
					std::cout << "check this";
				}
				labeling.add(target, !dir, v, d, p != none ? rank[p] : p); break;
			}
			for (Graph::arc_iterator a = g->begin(u, dir), end = g->end(u, dir); a < end; ++a) {
				Distance dd = d + (dir ? nodesWeightMap[std::pair<Vertex, Vertex>(u, a->head)]: nodesWeightMap[std::pair<Vertex, Vertex>( a->head,u)]);//a->length ;
				assert(dd > d && dd < infty);
				/*	if (rank[v] == 79 && rank[a->head] == 11) {
						std::cout << "check this";
					}*/
				if (dd < distance_PLL[a->head] ){// && trunc(dd * 10000) / 10000 <= trunc(labeling.query(v, rank[a->head], dir) * 10000) / 10000) {
					distance_PLL[a->head] = dd;
					parent_PLL[a->head] = u;
					queue_PLL.update(a->head, dd);
					if (!is_dirty_PLL[a->head]) { dirty_PLL.push_back(a->head); is_dirty_PLL[a->head] = true; }
				}
			}
		}
	}
	
public:
    Akiba(Graph &g) : BasicDijkstra(g) {}

    // Build HHL from a vertex order
    void run(std::vector<Vertex> &order, std::vector<Vertex> &rank, Labeling &labeling) {
        assert(order.size() == g->get_n());
        labeling.clear();
        for (size_t i = 0; i < order.size(); ++i) {
            iteration(i, false, order, rank, labeling);
            iteration(i, true, order, rank, labeling);
        }
    }

	void run_PLL(unsigned int v, unsigned int origin, unsigned int par, double dist, int target, bool dir, std::map<std::pair<unsigned, unsigned>, double> nodesWeightMap, std::vector<Vertex> &order, std::vector<Vertex> &rank, Labeling &labeling) {
		assert(order.size() == g->get_n());
			iteration_PLL(v, origin, par, dist,target, dir, nodesWeightMap, order, rank, labeling);
	}

	Distance run_dij(Vertex v, Vertex y, std::map<std::pair<unsigned, unsigned>, double> nodesWeightMap, bool forward = true) {
		KHeap<Vertex, Distance> queue_dij(g->get_n());
		std::vector<bool> visited(g->get_n(), false);
		auto n = g->get_n();
		std::vector<double> distance_dij(n, infty);
		queue_dij.clear();
		queue_dij.update(v, 0);
		distance_dij[v] = 0;
		while (!queue_dij.empty()) {
			Vertex u = queue_dij.pop();
			if (u != y && !visited[u]) {
				Distance d = distance_dij[u];
				for (Graph::arc_iterator a = g->begin(u, forward), end = g->end(u, forward); a < end; ++a) {
					auto len = forward ? nodesWeightMap[std::pair<Vertex, Vertex>(u, a->head)] : nodesWeightMap[std::pair<Vertex, Vertex>(a->head,u)];
					Distance dd = d + len;
					assert(dd > d && dd < infty);
					//if (dd < distance[a->head]) queue_dij.update(a->head, dd);
					queue_dij.update(a->head, dd);
					if (dd < distance_dij[a->head]) distance_dij[a->head] = dd;
				}
				visited[u] = true;
			}
			else if (u == y)
				return distance_dij[u];
		}
	}

	void Dec_PLL(Vertex x, Vertex y, double old_w, double new_w, std::map<std::pair<unsigned, unsigned>, double> & nodesWeightMap, Labeling &labeling, std::vector<Vertex> &rank, std::vector<Vertex> &order)
	{
		/*if ((x == 100 && y == 8) )
		{
			std::cout << "this is an error case";
		}*/
		auto Aff_x = affected_vertices_of_x(x, y, old_w, nodesWeightMap,labeling, rank, order);
		auto Aff_y = affected_vertices_of_y(x, y, old_w, nodesWeightMap,labeling, rank, order);
		remove_outdated_hubs(Aff_x, Aff_y, rank, labeling);
		DecPLL_update_label(Aff_x, Aff_y, x, y, new_w, nodesWeightMap, order, rank, labeling);
	}


	//find affected nodes of x
	std::set<Vertex> affected_vertices_of_x(Vertex x, Vertex y, double old_w, std::map<std::pair<unsigned, unsigned>, double> & nodesWeightMap,Labeling& labeling, std::vector<Vertex> & rank, std::vector<Vertex> & order) {
		std::set<Vertex> affected_vertices_of_x;
		std::vector<bool> visited(g->get_n(), false);
		clear();
		update(x, 0);
		while (!queue.empty())
		{
			Vertex v = queue.pop();
			affected_vertices_of_x.insert(v);
			for (Graph::arc_iterator a = g->begin(v, 0), end = g->end(v, 0); a < end; a++) {
				if (!visited[a->head]) {
					// if di(u, y) is not equal to di−1(u, y) then
				/*	if (labeling.query(rank[a->head], rank[y],1)!= run_dij(a->head, y)) {
						visited[a->head] = 1; 
						queue.update(a->head, 0);
					}
					else
					{*/
						//find the intersection of x and a->head
						//auto a_hubs = labeling.get_label_hubs(y)[0];
						//auto b_hubs = labeling.get_label_hubs(a->head)[1];
						//std::vector<Vertex> ab_intersect;
						//std::set_intersection(a_hubs.begin(), a_hubs.end(), b_hubs.begin(), b_hubs.end(), std::back_inserter(ab_intersect));
			
						//Distance d = trunc(labeling.query(rank[a->head], rank[y])* 10000) / 10000;
					     Distance d = run_dij(a->head, y, nodesWeightMap, 1);
						//for (auto h : ab_intersect) {
						 labeling.query(rank[a->head], rank[y]);
							auto smallest_hub = labeling.get_smallest_hub();
							//auto ux = trunc(labeling.query(rank[a->head], rank[x], 1) * 10000) / 10000;
							auto ux = trunc(run_dij(a->head, x, nodesWeightMap, 1) * 10000) / 10000;
							old_w = trunc(old_w * 10000) / 10000;
							//h ∈ A(x) ∨ ((h = u ∨ h = y) ∧ di−1(u,y) = di−1(u, x) + w)
							if (std::find(affected_vertices_of_x.begin(), affected_vertices_of_x.end(), order[smallest_hub]) != affected_vertices_of_x.end()
								|| ((smallest_hub == rank[a->head] || smallest_hub == rank[y]) &&
								(abs(d - (ux + old_w)) < 0.0001)))
							{
								visited[a->head] = 1;
								queue.update(a->head, 0);
							}
						//}

						/*if (std::find(affected_vertices_of_x.begin(), affected_vertices_of_x.end(), order[h]) != affected_vertices_of_x.end()
							|| ((h == rank[a->head] || h == rank[y]) && (round(d * 10000) == round(labeling.query(rank[a->head], rank[x], 1) * 10000) + round(old_w * 10000))))
						{
							visited[a->head] = 1;
							queue.update(a->head, 0);
						}*/

					//}

				}
				else continue;
			}
		}
		return affected_vertices_of_x;
		
	}

	//find affected nodes of y
	std::set<Vertex> affected_vertices_of_y(Vertex x, Vertex y, double old_w, std::map<std::pair<unsigned, unsigned>, double> & nodesWeightMap, Labeling& labeling, std::vector<Vertex> & rank, std::vector<Vertex> & order) {
		std::set<Vertex> affected_vertices_of_y;
		std::vector<bool> visited(g->get_n(), false);
		clear();
		visited[y] = 1;
		update(y, 0);
		while (!queue.empty())
		{
			Vertex v = queue.pop();
			affected_vertices_of_y.insert(v);
			for (Graph::arc_iterator a = g->begin(v, 1), end = g->end(v, 1); a < end; a++)
			{
				if (!visited[a->head]){
					//if di(x,u) is not equal to di−1(x,u) then
					//if (labeling.query(rank[x], rank[a->head], 1) != run_dij(x, a->head)) {
					////if () != labeling.query(x, a->head, 1)) {
					//	visited[a->head] = 1;
						//queue.update(a->head, 0);
					//}
					//else
					//{
						//find the intersection of x and a->head
						/*auto a_hubs = labeling.get_label_hubs(x)[0];
						auto b_hubs = labeling.get_label_hubs(a->head)[1];
						//std::set<Vertex> ab_intersect_set;
						std::vector<Vertex> ab_intersect;
						std::set_intersection(a_hubs.begin(), a_hubs.end(), b_hubs.begin(), b_hubs.end(), std::back_inserter(ab_intersect));*/
		
						//Distance d = trunc(labeling.query(rank[x], rank[a->head]) * 10000)/10000;
					Distance d = trunc(run_dij(x, a->head,nodesWeightMap) * 10000) / 10000;
						//for (auto h : ab_intersect) {
					labeling.query(rank[x], rank[a->head]);
					auto smallest_hub = labeling.get_smallest_hub();
							//auto yu = trunc(labeling.query(rank[y], rank[a->head], 1) * 10000) / 10000;
							auto yu = trunc(run_dij(y, a->head, nodesWeightMap, 1) * 10000) / 10000;
							old_w = trunc(old_w * 10000) / 10000;
							if (std::find(affected_vertices_of_y.begin(), affected_vertices_of_y.end(), order[smallest_hub]) != affected_vertices_of_y.end()
								|| ((smallest_hub == rank[a->head] || smallest_hub == rank[x]) && (
									abs(d - (yu + old_w)) < 0.0001))) {
								visited[a->head] = 1;
								queue.update(a->head, 0);
							}
						//}
						/*if (std::find(affected_vertices_of_y.begin(), affected_vertices_of_y.end(), order[h]) != affected_vertices_of_y.end()
								|| ((h == rank[a->head] || h == rank[x]) && round(d * 10000) == round(labeling.query(rank[y], rank[a->head], 1) * 10000) + round(old_w * 10000))) {
								visited[a->head] = 1;
								queue.update(a->head, 0);
							}*/
					//}
				}
				else continue;
			}
		}
		return affected_vertices_of_y;
	}


	//remove outdated labels in Dec_PLL
	void remove_outdated_hubs(std::set<Vertex>& Aff_x, std::set<Vertex>& Aff_y,std::vector<Vertex> &rank ,Labeling& labeling) {
		for (auto v : Aff_x) {
			for (auto u : Aff_y) {
				//removes(u, δuv) from Lout(v) if u ∈ Lout(v)
				if (u == v) break;
				if (rank[v] > rank[u]) {
					auto label_v = labeling.get_label_hubs(rank[v])[1];
					for (size_t i = 0; i < label_v.size(); i++) {
						if (label_v[i] == rank[u]) {
							labeling.remove_hub(rank[v], 1, i);
							////I never expected this
							//auto label_u = labeling.get_label_hubs(rank[u])[0];
							//for (size_t j = 0; j < label_u.size(); j++) {
							//	if (label_u[j] == rank[v]) {
							//		std::cout << "I never expected this";
							//		labeling.remove_hub(rank[u], 0, j);
							//	}
							//}
							//
							break;
						}
						else if (label_v[i] > rank[u]) 
							break;
					}
				}
				else if (rank[v] < rank[u]) {
					//removes(v, δuv) from Lin(u) if u ∈ Lout(v)
					auto label_u = labeling.get_label_hubs(rank[u])[0];
					for (size_t j = 0; j < label_u.size(); j++) {
						if (label_u[j] == rank[v]) {
							labeling.remove_hub(rank[u], 0, j);
							break;
						}
						else if (label_u[j] > rank[v]) 
							break;
					}
				}
				//	}
				//}
			}
		}
		//labeling.my_write("R:/Education/Concordia/Thesis/Simulation/RoutingSimulation/TestNet2-Removed_hubs.txt");
	}

	//update labels in Dec_PLL
	void DecPLL_update_label(std::set<Vertex>& Aff_x, std::set<Vertex>& Aff_y, Vertex x, Vertex y, double new_w, std::map<std::pair<unsigned, unsigned>, double> &nodesWeightMap, std::vector<Vertex> & order, std::vector<Vertex> rank, Labeling& labeling) {
		std::set<Vertex> FA_temp;
		std::vector<Vertex> FA;
		std::vector<bool> visited(g->get_n(), false);
		//std::copy(Aff_x.begin(), Aff_x.end(), std::back_inserter(FA));
		//std::copy(Aff_y.begin(), Aff_y.end(), std::back_inserter(FA));
		std::copy(Aff_y.begin(), Aff_y.end(), std::inserter(FA_temp, FA_temp.end()));
		std::copy(Aff_x.begin(), Aff_x.end(), std::inserter(FA_temp, FA_temp.end()));
		std::copy(FA_temp.begin(), FA_temp.end(), std::back_inserter(FA));

		std::sort(FA.begin(), FA.end(), [&](auto a, auto b)->bool {return rank[a] < rank[b]; });

		std::vector<Vertex> V_a = rank;
		//clear();
		for (Vertex h : FA) {

			clear();
			visited[h] = true;
			distance[h] = 0;
			update(h, distance[h], h);
			while (!queue.empty())
			{
				Vertex v = queue.pop();
				Distance d = trunc(distance[v] * 10000) / 10000;
				Vertex p = parent[v];
				//if ((rank[h] == 36))//|| v == 88) )//|| (h == 88 && v == 134))
				//{
				//	std::cout << "check this out";
				//}

				if (rank[v] < rank[h]) continue;
				else
				{
					if (std::count(Aff_x.begin(), Aff_x.end(), h)) {
						if (std::count(Aff_y.begin(), Aff_y.end(), v)) {
							//if (rank[v] > rank[h]) {
							auto hv = trunc(labeling.query(rank[h], rank[v], 1) * 10000) / 10000;
							if (d < hv) {
								std::vector< std::vector<Vertex> > label = labeling.get_label_hubs(rank[v]);
								for (size_t i = 0; i < label[0].size();) {
									if (label[0][i] == rank[h])
									{
										labeling.set_label_distance(rank[v], distance[v], i, 0);
										//labeling.set_label_parent(rank[v], p != none ? rank[p] : p, i, 0);
									/*	if (rank[p] == 134) {
											std::cout << "there is a problem";
										}*/
										break;
										//}
									}
									else if (i == label[0].size() || (label[0][i] > rank[h])) {
										//labeling.add(rank[v], 0, rank[h], distance[v], p != none ? rank[p] : p); 
										auto dist = trunc(labeling.query(rank[h], rank[x], 1) * 10000) / 10000;
										run_PLL(rank[h], rank[y], rank[x], dist + new_w, rank[v], true, nodesWeightMap, order, rank, labeling);
										/*	if (rank[p] == 134) {
												std::cout << "there is a problem";
											}*/
										labeling.sort_label(rank[v], 0); break;
									}
									else i++;

								}

							}
						}

						for (Graph::arc_iterator a = g->begin(v, 1), end = g->end(v, 1); a < end; a++)
						{
							//if (!visited[a->head]) {
							auto len = (v == x && a->head == y) ? new_w : nodesWeightMap[std::pair<Vertex, Vertex>(v, a->head)];
							Distance dd = distance[v] + len;// a->length;
							if (trunc(dd * 10000) / 10000 < trunc(len * 10000) / 10000) {
								//	visited[a->head] = true;
								distance[a->head] = dd;
								update(a->head, dd, v);
							}
						}
					}
					else if (std::count(Aff_y.begin(), Aff_y.end(), h)) {
						if (std::count(Aff_x.begin(), Aff_x.end(), v)) {
							auto hv = trunc(labeling.query(rank[v], rank[h], 1) * 10000) / 10000;
								if (d < hv) {
									std::vector< std::vector<Vertex> > label = labeling.get_label_hubs(rank[v]);
										for (size_t i = 0; i < label[0].size();) {
											if (label[0][i] == rank[h])
											{
												labeling.set_label_distance(rank[v], distance[v], i, 0);
													//labeling.set_label_parent(rank[v], p != none ? rank[p] : p, i, 0);
												/*	if (rank[p] == 134) {
														std::cout << "there is a problem";
													}*/
													break;
												//}
											}
											else if (i == label[0].size() || (label[0][i] > rank[h])) {
												//labeling.add(rank[v], 0, rank[h], distance[v], p != none ? rank[p] : p); 
												auto dist = trunc(labeling.query(rank[h], rank[x], 1) * 10000) / 10000;
												run_PLL(rank[h], rank[y], rank[x], dist + new_w, rank[v], true, nodesWeightMap, order, rank, labeling);
												/*	if (rank[p] == 134) {
														std::cout << "there is a problem";
													}*/
												labeling.sort_label(rank[v], 0); break;
											}
											else i++;

										}

								}
						}
						for (Graph::arc_iterator a = g->begin(v, 0), end = g->end(v, 0); a < end; a++)
						{
							//if (!visited[a->head]) {
							auto len = (v == x && a->head == y) ? new_w : nodesWeightMap[std::pair<Vertex, Vertex>(a->head, v)];
							Distance dd = distance[v] + len;// a->length;
							if (trunc(dd * 10000) / 10000 < trunc(len * 10000) / 10000) {
								//	visited[a->head] = true;
								distance[a->head] = dd;
								update(a->head, dd, v);
							}
						}

					}


				}

				}
			}
		}
		//clear();
		//for (Vertex h : FA) {
		//	//for (Vertex u : V_a)
		//	//{
		//	//	visited[u] = false;
		//	//	distance[u] = infty;
		//	//}
		//	clear();
		//	visited[h] = true;
		//	distance[h] = 0;
		//	update(h, distance[h], h);
		//	while (!queue.empty())
		//	{
		//		Vertex v = queue.pop();
		//		Distance d = trunc(distance[v] * 10000) / 10000;
		//		Vertex p = parent[v];
		//		if ((h ==140 )&& (v == 116))//|| (h == 88 && v == 134))
		//		{
		//			std::cout << "check this out";
		//		}

		//		if (rank[v] < rank[h]) continue;
		//		else
		//		{
		//			if (std::count(Aff_y.begin(), Aff_y.end(), h)) {
		//				if (std::count(Aff_x.begin(), Aff_x.end(), v)) {
		//					//if (rank[h] < rank[v]) {
		//					auto vh = trunc(labeling.query(rank[v], rank[h], 1) * 10000) / 10000;
		//					if (d < vh) {
		//						std::vector< std::vector<Vertex> > label = labeling.get_label_hubs(rank[v]);
		//						for (size_t i = 0; i < label[1].size();) {
		//							if (label[1][i] == rank[h])
		//							{
		//								labeling.set_label_distance(rank[v], distance[v], i, 1);
		//								//labeling.set_label_parent(rank[v], p != none ? rank[p] : p, i, 1);
		//								/*if ( rank[p] == 134) {
		//									std::cout << "there is a problem";
		//								}*/
		//								break;
		//							}
		//							else if (i == label[1].size() || (label[1][i] > rank[h])) {
		//								//labeling.add(rank[v], 1, rank[h], distance[v], p != none ? rank[p] : p);
		//								auto dist = trunc(labeling.query(rank[y], rank[h], 1) * 10000) / 10000;
		//								run_PLL(rank[h], rank[x], rank[y], dist + new_w, rank[v], false, nodesWeightMap, order, rank, labeling);
		//						/*		if ( rank[p] == 134) {
		//									std::cout << "there is a problem";
		//								}*/
		//								labeling.sort_label(rank[v], 1); break;
		//							}
		//							else i++;
		//						}
		//					}
		//				}
		//				for (Graph::arc_iterator a = g->begin(v, 0), end = g->end(v, 0); a < end; a++)
		//				{
		//					//if (!visited[a->head]) {
		//					auto len = (v == y && a->head == x) ? new_w : nodesWeightMap[std::pair<Vertex, Vertex>(a->head, v)];
		//					Distance dd = distance[v] + (len);// a->length ;
		//					if (trunc(dd *10000) / 10000 < trunc(len * 10000)/10000) {
		//						distance[a->head] = dd;
		//						update(a->head, dd, v);
		//					}
		//				}
		//			}
		//		}
		//	}
		//}

	
	
	void Inc_PLL(Vertex aa, Vertex b, double w, std::map<std::pair<unsigned,unsigned>, double> & nodesWeightMap, Labeling &labeling, std::vector<Vertex> &rank, std::vector<Vertex> &order)
	{
		//if (a == 195 && b == 190)
		//{
		//	std::cout << "this is an error case";
		//}
		auto a_hubs = labeling.get_label_hubs(rank[aa])[0];
		auto b_hubs = labeling.get_label_hubs(rank[b])[1];
		/*std::sort(a_hubs.begin(), a_hubs.end(), [&](auto a, auto b)->bool {return rank[a] < rank[b]; });
		std::sort(b_hubs.begin(), b_hubs.end(), [&](auto a, auto b)->bool {return rank[a] < rank[b]; });*/
		clear();
		//for (auto v : ab_intersect)
		for (auto v : a_hubs)
		{
			//Distance d = labeling.query(v, rank[aa]);
			Distance d = run_dij(v, rank[aa],nodesWeightMap,1);
			auto mp = labeling.get_mp();
			auto predecessor = labeling.get_label_parent(rank[aa])[0][std::get<2>(mp)];
			//update(b, d + w, v);
			update(b, d + w , aa);
			//update(b, d + w, predecessor!= none? order[predecessor]: none);

			while (!queue.empty())
			{
				Vertex u = queue.pop();
				Distance m = distance[u] ;
				Vertex p = parent[u];
				//auto vu = trunc(labeling.query(v, rank[u], 1) * 10000) / 10000;
				auto vu = trunc(run_dij(v, rank[u],nodesWeightMap, 1) * 10000) / 10000;
				auto temp_m = trunc(m * 10000) / 10000;
				if (trunc(vu <= temp_m))
					continue;
				//labeling.add(u, 0, v, m, p);
				if (v < rank[u]) {
					//check if the hub already exist in backward label of u
					std::vector< std::vector<Vertex> > label = labeling.get_label_hubs(rank[u]);
					for (size_t i = 0; i < label[0].size();) {
						if (label[0][i] == v)
						{
							labeling.set_label_distance(rank[u], m, i, 0);
							/*	if (rank[p] == 134 && v == 115) {
									std::cout << "there is a problem";
								}*/

							//labeling.set_label_parent(rank[u], p != none ? rank[p] : p, i, 0);
							
							break;
							//}
						}
						else if (i == label[0].size() || (label[0][i] > v)) {
					//		labeling.add(rank[u], 0, v, m, p != none ? rank[p] : p); labeling.sort_label(rank[u], 0); break;}
						
							this->run_PLL(v, rank[aa], rank[b], d, rank[u], true, nodesWeightMap, order, rank, labeling); labeling.sort_label(rank[u], 0); break;
					}
						else i++;
					}
				}
				else if (v > rank[u]){
					//labeling.add(v, 1, u, m, p);
					std::vector< std::vector<Vertex> > label = labeling.get_label_hubs(v);
					for (size_t i = 0; i < label[1].size();) {
						if (label[1][i] == rank[u])
						{
							
							labeling.set_label_distance(v, m, i, 1);
							/*if (rank[p] == 134 && v == 115) {
								std::cout << "there is a problem";
							}*/
							//labeling.set_label_parent(v, p!=none ? rank[p]:p, i, 1);// we don't need this
							break;
							//}
						}
						else if (i == label[1].size()|| (label[1][i] > rank[u])) {
							/*if (rank[p] == 134 && v==115) {
								std::cout << "there is a problem";
							}*/
							//labeling.add(v, 1, rank[u], m, p != none ? rank[p] : p)

							//labeling.add(v, 1, rank[u], m, predecessor); labeling.sort_label(v, 1); break;
							this->run_PLL(rank[u], rank[aa], rank[b], distance[u]-d, v, false, nodesWeightMap, order, rank, labeling); labeling.sort_label(v, 1);  break;
						}
						else i++;
					}
				}
					
				for (Graph::arc_iterator a = g->begin(u, 1), end = g->end(u, 1); a < end; a++)
				{
					Distance dd = m + ((u == aa && a->head == b) ? w : nodesWeightMap[std::pair<Vertex, Vertex>(u, a->head)]);//a->length * 10000)/10000;
					assert(dd > m && dd < infty);
					//if (dd < distance[a->head] && dd < labeling.query(v, a->head, 1)) update(a->head, dd, u);
					update(a->head, dd, u);
				}
			}
		}
		clear();
		for (auto v : b_hubs)
		{
			//Distance d = labeling.query(rank[b], v);
			Distance d = run_dij(rank[b], v,nodesWeightMap,1);
			auto mp = labeling.get_mp();
			auto predecessor = labeling.get_label_parent(v)[0][std::get<2>(mp)];
			update(aa, d + w , b);
			while (!queue.empty())
			{
				Vertex u = queue.pop();
				Distance m = distance[u] ;
				Vertex p = parent[u];
				//auto uv = trunc(labeling.query(rank[u], v, 1) * 10000) / 10000;
				auto uv = trunc(run_dij(rank[u], v, nodesWeightMap, 1) * 10000) / 10000;
				auto temp_m = trunc(m * 10000) / 10000;
				if (uv <= temp_m )
					continue;
				if (v < rank[u]) {
					std::vector< std::vector<Vertex> > label = labeling.get_label_hubs(rank[u]);
					for (size_t i = 0; i < label[1].size();) {
						if (label[1][i] == v)
						{
							labeling.set_label_distance(rank[u], m, i, 1);
							//labeling.set_label_parent(rank[u], p!=none ? rank[p]:p, i, 1);
							break;
						}
						else if (i == label[1].size() || (label[1][i] > v)) { //labeling.add(rank[u], 1, v, m, p != none ? rank[p] : p); 
							run_PLL(v, rank[aa], rank[b], d+w, rank[u], false, nodesWeightMap, order, rank, labeling);
							labeling.sort_label(rank[u], 1); break;
						}
						else i++;
					}
				}
				else if (v > rank[u]) {
					std::vector< std::vector<Vertex> > label = labeling.get_label_hubs(v);
					for (size_t i = 0; i < label[0].size();) {
						if (label[0][i] == rank[u])
						{
							labeling.set_label_distance(v, m, i, 0);
							//labeling.set_label_parent(v, p!=none ? rank[p]: p, i, 0); we don't need this
							break;
						}
						else if (i == label[0].size() || (label[0][i] > rank[u])) {
						
							//labeling.add(v, 0, rank[u], m, p != none ? rank[p] : p)
							run_PLL(rank[u], rank[b], rank[aa], distance[u]-d-w, v, true, nodesWeightMap, order, rank, labeling);
							labeling.sort_label(v, 0); break; }
						else i++;
					}
				}
				
			
				for (Graph::arc_iterator a = g->begin(u, 0), end = g->end(u, 0); a < end; a++)
				{
					Distance dd = m + ((v == b && a->head == aa) ? w : nodesWeightMap[std::pair<Vertex, Vertex>(a->head,u)]);  //trunc(a->length * 10000) / 10000;
					assert(dd > m && dd < infty);
					//if (dd < distance[a->head] && dd < labeling.query(a->head, v)) update(a->head, dd, u);
					update(a->head, dd, u);
				}
			}
		}
	}

};

}
