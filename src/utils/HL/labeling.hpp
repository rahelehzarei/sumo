// Hub Labels are lists of hubs and distances to them attached to every vertex in a graph.
// This file contains the class to store labels.
// Available methods include making a query, write labels to file, read labels from file.
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
#include <vector>
#include <limits>
#include <cassert>
#include <algorithm>
#include <utility>
#include <fstream>
#include <istream>
#include<map>
#include <set>
#include <iostream>

namespace hl {

// Class to store labels
class Labeling {
    std::vector< std::vector< std::vector<Vertex> > > label_v;     // Lists of forward/reverse hubs
    std::vector< std::vector< std::vector<Distance> > > label_d;   // Lists of distances to hubs
	std::vector< std::vector< std::vector<Vertex> > > label_p;
    Vertex n;
	PATH path_f;
	std::set<Vertex> check_path;
	std::tuple<Vertex , int, int> meeting_point;
	std::tuple<Vertex, int, int> meeting_point_tmp;
	Vertex smallest_hub;
	
	
	int counter = 0; int s; int t;

public:
    Labeling(size_t n = 0) :
        label_v(n, std::vector< std::vector<Vertex> >(2)),
        label_d(n, std::vector< std::vector<Distance> >(2)),
		label_p(n, std::vector< std::vector<Vertex> >(2)),
        n(n) {}
	int return_counter() { return counter; }
	void reset_counter(unsigned source, unsigned target) { t = target; s = source; counter = 0; }

    // Find u-v distance
    Distance query(Vertex u, Vertex v, bool f = true) {
		std::get<0>(meeting_point) = none; std::get<1>(meeting_point) = none; std::get<2>(meeting_point) = none;
        Distance r = infty;
        for (size_t i=0, j=0; i < label_v[u][f].size() && j < label_v[v][!f].size();) {
            if (label_v[u][f][i] == label_v[v][!f][j]) {
                assert(label_d[u][f][i] < infty - label_d[v][!f][j]);
				auto curr_dist = r;
                r = std::min(curr_dist, label_d[u][f][i] + label_d[v][!f][j]);
				if (r < curr_dist) {
					std::get<0>(meeting_point) = label_v[u][f][i]; std::get<1>(meeting_point) = i; std::get<2>(meeting_point) = j;
					smallest_hub = label_v[u][f][i];
				}
				i++; j++;
            } else if (label_v[u][f][i] < label_v[v][!f][j]) ++i;
            else ++j;
        }
		
        return r;
    }

	Distance query_temp(Vertex u, Vertex v, bool f = true) {
		Distance r = infty;
		for (size_t i = 0, j = 0; i < label_v[u][f].size() && j < label_v[v][!f].size();) {
			if (label_v[u][f][i] == label_v[v][!f][j]) {
				assert(label_d[u][f][i] < infty - label_d[v][!f][j]);
				auto curr_dist = r;
				r = std::min(curr_dist, label_d[u][f][i] + label_d[v][!f][j]);
				if (r < curr_dist) {
					std::get<0>(meeting_point_tmp) = label_v[u][f][i]; std::get<1>(meeting_point_tmp) = i; std::get<2>(meeting_point_tmp) = j;
					smallest_hub = label_v[u][f][i];
				}
				i++; j++;
			}
			else if (label_v[u][f][i] < label_v[v][!f][j]) ++i;
			else ++j;
		}

		return r;
	}

	//Distance query(Vertex u, Vertex v, bool f = true) {
	//	Distance r = infty;
	//	for (size_t i = 0, j = 0; i < label_v[u][f].size() && j < label_v[v][!f].size();) {
	//		if (label_v[u][f][i] == label_v[v][!f][j]) {
	//			assert(label_d[u][f][i] < infty - label_d[v][!f][j]);
	//			r = std::min(r, label_d[u][f][i++] + label_d[v][!f][j++]);
	//		}
	//		else if (label_v[u][f][i] < label_v[v][!f][j]) ++i;
	//		else ++j;
	//	}
	//	return r;
	//}

	PATH path(Vertex u, Vertex v, bool f=true) { // , bool f = true) {
		
		counter += 1;
		if (return_counter() > 1000) {
			std::cout << "error";
		}
		//base case 0
		if (u == none || v == none)
		{
			std::vector<Vertex> bc;
			if (check_path.count(u) == 0 && u != none) {
				check_path.insert(u); bc.push_back(u);
			}
			if (check_path.count(v) == 0 && v != none) {
				check_path.insert(v); bc.push_back(v);
			}
			return bc;
		}

		query(u, v);
		size_t i = std::get<1>(meeting_point);
		size_t j = std::get<2>(meeting_point);
		Vertex MP = std::get<0>(meeting_point);
		
		//base case0
		//if (MP == none) { return {}; }
		
		//base cases1
		if (u == v) {
			std::vector<Vertex> ch ;
			if (check_path.count(u) == 0) {
				check_path.insert(u);  ch.push_back(u);  return ch;
			}
			else
				return ch;
				std::cout << "there might be an error in line 140";
		}

		////base cases1.1
		//if (u == none || v == none)
		//{
		//	std::cout << " there is an error";
		//	if (u != none && v == none)
		//	{
		//		if (check_path.count(u) == 0) {
		//			check_path.insert(u); std::vector<Vertex> ch(u);  return ch;
		//		}
		//	}
		//	else if (v != none && u==none)
		//	{
		//		if (check_path.count(v) == 0) {
		//			check_path.insert(v); std::vector<Vertex> ch(v);  return ch;
		//		}
		//	}
		//	else
		//	{
		//		std::cout << " both u and v are none";
		//		return {};

		//	}
		//		

		//}

		//base cases1.1.1
		if (label_p[u][f][i] == none && label_p[v][!f][j] == none )
		{

			std::vector<Vertex> ch;
			if (check_path.count(u) == 0) {
				check_path.insert(u); ch.push_back(u); 
			}
			if (check_path.count(v) == 0) {
				check_path.insert(v); ch.push_back(v);
			}
			return ch;
		}



		//new basecase
		//if ()

		
		//base cases2
		if (f)
		{
			/*std::cout << label_p[u][f][i];
			std::cout << label_p[v][!f][j];*/
			if (label_p[u][f][i] == u) {
				std::vector<Vertex> fh;
				if (check_path.count(u) == 0) {
					check_path.insert(u); fh.push_back(u);
				}
				if (check_path.count(v) == 0) {
					check_path.insert(v); fh.push_back(v);
				}

				return fh;
			}
				if (label_p[u][f][i] == v) {
					//std::cout << "1: I never expected this"; 
					std::vector<Vertex> fh;
					if (check_path.count(u) == 0) {
						check_path.insert(u); fh.push_back(u);
					}
					if (check_path.count(v) == 0) {
						check_path.insert(v); fh.push_back(v);
					}
					return fh;

					//fix me later. I think this is the point that we are adding node to incorrect place

				}
		}

	//general case
		std::vector<Vertex>  v1a = {};
		std::vector<Vertex>  v1b = {};
		std::vector<Vertex> v2a = {};
		std::vector<Vertex> v2b = {};
		if (label_p[u][f][i] != none) {
			//auto d1 = distance(u, label_p[u][f][i]);
			//auto d2 = distance(label_p[u][f][i], MP);
			/*query_temp(u, label_p[u][f][i]);
			auto m1 =std::get<0>(meeting_point_tmp);*/
			//std::vector<Vertex>  v1a = {};
			//std::vector<Vertex>  v1b = {};
			//std::vector<Vertex> v1;
			//if (m1 == u || m1== label_p[u][f][i]) {
				if (check_path.count(u) == 0 && u != none) {
					check_path.insert(u); 
					v1a = { u };
				}
				
				v1b = path(label_p[u][f][i], MP);
			}
			//else
			//{
			//	v1a = path(u, label_p[u][f][i], order);
			//	//add MP to vect
			//	//v1b = {MP} //no actually, the second recursive call will add MP
			//	if (check_path.count(MP) == 0 && MP != none) {
			//		check_path.insert(MP);
			//		v1b = { MP };
			//	}

			//}

			//std::vector<Vertex> v2;
			//if (check_path.count(MP) == 0) {
			//	check_path.insert(MP); 
			//	v2 = { MP };
			//}
		//}
		else
		{
			//v1a = path(u, label_p[u][f][i]); --> check it it's correct to remove this statement
			//I added this now
			if (check_path.count(u) == 0 && u != none) {
				check_path.insert(u);
				//v1b = { u };
				v1a = { u };
			}
			//end
			if (check_path.count(MP) == 0 && MP != none) {
				check_path.insert(MP);
				v1b = { MP };
			}

		}
		//auto v3=path(label_p[v][!f][j], v);
		if (label_p[v][!f][j] != none) {
		/*	query_temp(label_p[v][!f][j], v);
			auto m1 = std::get<0>(meeting_point_tmp);*/
			//std::vector<Vertex>  v1a = {};
			//std::vector<Vertex>  v1b = {};
			//std::vector<Vertex> v1;
			/*if (m1 == v || m1 == label_p[v][!f][j]) {*/
				//v1a = { u };
				v2a = path(MP, label_p[v][!f][j]);
				if (check_path.count(v) == 0 && v != none) {
					check_path.insert(v); 
					v2b = { v };
				}
				
			}
		//	else
		//	{
		//		if (check_path.count(MP) == 0 && MP != none) {
		//			check_path.insert(MP); 
		//			v2a = { MP };
		//		}
		//		//add MP to vect
		//		//v1b = {MP} //no actually, the second recursive call will add MP
		//		v2b = path(label_p[v][!f][j], v, order);

		//	}

		//}
		else
		{
			if (check_path.count(MP) == 0 && MP != none) {
				check_path.insert(MP); 
				v2a = { MP };
			}
			//v2b = path(label_p[v][!f][j], v);
			//v2b = path(label_p[v][!f][j], v);
			//I added this now
			if (check_path.count(v) == 0 && v != none) {
				check_path.insert(v);
				v2b = { v };
			}
			//end
		}
		//v1.insert(v1.end(), v2.begin(), v2.end());
		//v1a, v1b, v2a, v2b 
		v1a.insert(v1a.end(), v1b.begin(), v1b.end());
		v1a.insert(v1a.end(), v2a.begin(), v2a.end());
		v1a.insert(v1a.end(), v2b.begin(), v2b.end());
		return v1a;
	}	
	
	PATH query_new(Vertex u, Vertex v, std::vector<Vertex>& order, bool f = true){
		//base cases
		//1
		/*if (u == none || v == none)
		{
			std::vector<Vertex> bc;
			if (check_path.count(u) == 0 && u != none) {
				check_path.insert(u); bc.push_back(u);
			}
			if (check_path.count(v) == 0 && v != none) {
				check_path.insert(v); bc.push_back(v);
			}
			return bc;
		}*/
		//2
		if (u == v) {
			std::vector<Vertex> ch;
			if (check_path.count(u) == 0) {
				check_path.insert(u);  ch.push_back(u);  return ch;
			}
			else
				return ch;
		}
		query(u, v);
		size_t i = std::get<1>(meeting_point);
		size_t j = std::get<2>(meeting_point);
		Vertex MP = std::get<0>(meeting_point);
		//3
		//if (label_p[u][f][i] == v) {
		//	std::vector<Vertex> fh;
		//	if (check_path.count(u) == 0) {
		//		check_path.insert(u); fh.push_back(u);
		//	}
		//	if (check_path.count(v) == 0) {
		//		check_path.insert(v); fh.push_back(v);
		//	}
		//	return fh;
		//}
		////4
		//if (label_p[v][!f][i] == u) {
		//	std::vector<Vertex> fh;
		//	if (check_path.count(u) == 0) {
		//		check_path.insert(u); fh.push_back(u);
		//	}
		//	if (check_path.count(v) == 0) {
		//		check_path.insert(v); fh.push_back(v);
		//	}

		//	return fh;
		//}

		//general case
		std::vector<Vertex>  v1a = {};
		std::vector<Vertex>  v1b = {};
		std::vector<Vertex> v2a = {};
		std::vector<Vertex> v2b = {};

		if (label_p[u][f][i] != none) {
			v1a = { u };
			v1b = query_new(label_p[u][f][i], MP, order);
		}
		else
		{
			if (check_path.count(u) == 0 && u != none) {
				check_path.insert(u);
				v1a = { u };
			}
			if (check_path.count(MP) == 0 && MP != none) {
				check_path.insert(MP);
				v1b = { MP };
			}

		}
		if (label_p[v][!f][j] != none) {
				v2a = query_new(MP, label_p[v][!f][j], order);
				if (check_path.count(v) == 0 && v != none) {
					check_path.insert(v);
					v2b = { v };
				}

			}
		else
		{
			if (check_path.count(MP) == 0 && MP != none) {
					check_path.insert(MP);
					v2a = { MP };
			}
			v2b = {v};

			}
		v1a.insert(v1a.end(), v1b.begin(), v1b.end());
		v1a.insert(v1a.end(), v2a.begin(), v2a.end());
		v1a.insert(v1a.end(), v2b.begin(), v2b.end());
		return v1a;
	}
	
	PATH query_path(Vertex u, Vertex v, bool f = true) {
		//this->sort();
		//path_f.insert(path_f.begin(), u);
		//path_f.insert(path_f.end(), v);
		//check_path.insert(u);
		//check_path.insert(v);
		//auto final_path = query_new(u, v, order);
		auto final_path = path(u, v);
		//std::copy(path_f.begin(), path_f.end(), std::inserter(path, path.end()));
		path_f.clear();
		check_path.clear();
		return final_path;
	}

    // Add hub (v,d) to forward or reverse label of u
    void add(Vertex u, bool forward, Vertex v, Distance d, Vertex p=none) {
        label_v[u][forward].push_back(v);
        label_d[u][forward].push_back(d);
		label_p[u][forward].push_back(p);
		
    }

    // Get labels
    std::vector< std::vector<Vertex> > &get_label_hubs(Vertex u) { return label_v[u]; }
    std::vector< std::vector<Distance> > &get_label_distances(Vertex u) { return label_d[u]; }
	std::vector< std::vector<Vertex> > &get_label_parent(Vertex u) { return label_p[u]; }
	std::vector< std::vector<Vertex> > &get_parent_hubs(Vertex u) { return label_p[u]; } //added by me
	void set_label_distance(Vertex u, Distance d, int index, bool f = true) { label_d[u][f][index] = d; } //added by me. It sets the distance from a hub in label of vertex u
	void set_label_parent(Vertex u, Vertex v, int index, bool f = true) { label_p[u][f][index] = v;
	/*if (label_p[46][0].size() ==12 && label_p[46][0][8] == 107)
		std::cout << "check";*/
	} //added by me. It sets the predecessor of a hub in label of vertex u
	void remove_hub(Vertex v, bool f, int i) { label_d[v][f].erase(label_d[v][f].begin() + i); label_v[v][f].erase(label_v[v][f].begin() + i); label_p[v][f].erase(label_p[v][f].begin() + i);
	}
	std::tuple<Vertex, int, int> get_mp() { return meeting_point; } //added by me
	Vertex get_smallest_hub() { return smallest_hub; }
    // Get maximum label size
    size_t get_max() const {
        size_t max = 0;
        for (Vertex v = 0; v < n; ++v)
            for (int side = 0; side < 2; ++side)
                max = std::max(max, label_v[v][side].size());
        return max;
    }

    // Get average label size
    double get_avg() const {
        long long total = 0;
        for (Vertex v = 0; v < n; ++v)
            total += label_v[v][0].size() + label_v[v][1].size();
        return static_cast<double>(total)/n/2;
    }

	//original write function
	bool write(char *filename) {
		std::ofstream file;
		file.open(filename);
		file << n << std::endl;
		for (Vertex v = 0; v < n; ++v) {
			for (int side = 0; side < 2; ++side) {
				file << label_v[v][side].size();
				for (size_t i = 0; i < label_v[v][side].size(); ++i) {
					file << " " << label_v[v][side][i];
					file << " " << label_d[v][side][i];
					file << " " << label_p[v][side][i]; //I added this
				}
				file << std::endl;
			}
		}
		file.close();
		return file.good();
	}

    // Write labels to file
	//my version of write function
    bool my_write(char *filename) {
        std::ofstream file;
        file.open(filename);
        file << n << std::endl;
        for (Vertex v = 0; v < n; ++v) {
			file << "Vertex: " << v << std::endl;
            for (int side = 0; side < 2; ++side) {
				file << "label " << side << " size: " << label_v[v][side].size() << ": " ;
                for (size_t i = 0; i < label_v[v][side].size(); ++i) {
                    file << " " << label_v[v][side][i];
                    file << " " << label_d[v][side][i];
					file << " " << label_p[v][side][i];
					file << " - ";
                }
                file << std::endl;
            }
        }
        file.close();
        return file.good();
    }

    //// Read labels from file
    //bool read(char *filename, Vertex check_n = 0) {
    //    std::ifstream file;
    //    file.open(filename);
    //    file >> n;
    //    if (check_n && n != check_n) return false;
    //    label_v.resize(n, std::vector< std::vector<Vertex> >(2));
    //    label_d.resize(n, std::vector< std::vector<Distance> >(2));
    //    for (Vertex v = 0; v < n; ++v) {
    //        for (int side = 0; side < 2; ++side) {
    //            size_t s;
    //            file >> s;
    //            label_v[v][side].resize(s);
    //            label_d[v][side].resize(s);
    //            for (size_t i = 0; i < s; ++i) {
    //                file >> label_v[v][side][i];
    //                file >> label_d[v][side][i];
    //            }
    //        }
    //    }
    //    file >> std::ws;
    //    file.close();
    //    return file.eof() && !file.fail();
    //}


	//my version of read function
		// Read labels from file
	bool read(char *filename, Vertex check_n = 0) {
		std::ifstream file;
		file.open(filename);
		file >> n;
		if (check_n && n != check_n) return false;
		label_v.resize(n, std::vector< std::vector<Vertex> >(2));
		label_d.resize(n, std::vector< std::vector<Distance> >(2));
		label_p.resize(n, std::vector< std::vector<Vertex> >(2));
		for (Vertex v = 0; v < n; ++v) {
			for (int side = 0; side < 2; ++side) {
				size_t s;
				file >> s;
				label_v[v][side].resize(s);
				label_d[v][side].resize(s);
				label_p[v][side].resize(s);
				for (size_t i = 0; i < s; ++i) {
					file >> label_v[v][side][i];
					file >> label_d[v][side][i];
					file >> label_p[v][side][i];
				}
			}
		}
		file >> std::ws;
		file.close();
		return file.eof() && !file.fail();
	}
    // Clear labels
    void clear() {
        for (Vertex v = 0; v < n; ++v) {
            for (int side = 0; side < 2; ++side) {
                label_v[v][side].clear();
                label_d[v][side].clear();
            }
        }
    }

    // Sort labels before making queries
    void sort() {
        for (Vertex v = 0; v < n; ++v) {
            for (int side = 0; side < 2; ++side) {
                std::vector< std::tuple<Vertex,Distance,Vertex> > label(label_v[v][side].size());
                for (size_t i = 0; i < label_v[v][side].size(); ++i)
                    label[i] = std::make_tuple(label_v[v][side][i], label_d[v][side][i], label_p[v][side][i]);
				std::sort(label.begin(), label.end(), [](auto a, auto b) -> bool{return std::get<0>(a) < std::get<0>(b); });
                for (size_t i = 0; i < label_v[v][side].size(); ++i) {
                    label_v[v][side][i] = std::get<0>(label[i]);
                    label_d[v][side][i] = std::get<1>(label[i]);
					label_p[v][side][i] = std::get<2>(label[i]);
                }
            }
        }
    }

	void sort_label(Vertex v, int side) {
		std::vector< std::tuple<Vertex, Distance, Vertex> > label(label_v[v][side].size());
		for (size_t i = 0; i < label_v[v][side].size(); ++i)
			label[i] = std::make_tuple(label_v[v][side][i], label_d[v][side][i], label_p[v][side][i]);
		std::sort(label.begin(), label.end(), [](auto a, auto b) -> bool {return std::get<0>(a) < std::get<0>(b); });
				for (size_t i = 0; i < label_v[v][side].size(); ++i) {
					label_v[v][side][i] = std::get<0>(label[i]);
					label_d[v][side][i] = std::get<1>(label[i]);
					label_p[v][side][i] = std::get<2>(label[i]);
				}
			}
		
		


    #if 0
    // Print labels
    void print() const {
        for (Vertex v = 0; v < n; ++v) {
            for (int side = 0; side < 2; ++side) {
                std::cout << "L(" << v << "," << side << ") =";
                for (size_t i = 0; i < label_v[v][side].size(); ++i) std::cout << " (" << label_v[v][side][i] << "," << label_d[v][side][i] << ")";
                std::cout << std::endl;
            }
        }
    }
    #endif

};

}
