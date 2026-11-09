/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
//@For graph
#include <perception_3d/dynamic_graph.h>
#include <algorithm>

namespace perception_3d
{

void DynamicGraph::setValue(unsigned int key, double distance){
  //@ Keep the minimum clearance. Missing keys must NOT default-construct to 0 via
  //@ operator[], otherwise the first mark becomes min(0, distance)=0 and unmarked
  //@ nodes later read as lethal in RViz / planning.
  auto it = graph_.find(key);
  if (it == graph_.end()) {
    graph_.emplace(key, distance);
  } else {
    it->second = std::min(it->second, distance);
  }
}

void DynamicGraph::clearValue(unsigned int key, double distance){
  //@ overwrite the value
  graph_[key] = distance;
}

double DynamicGraph::getValue(const unsigned int index){
  auto it = graph_.find(index);
  if (it == graph_.end()) {
    //@ Unmarked == free: return max clearance instead of 0 (lethal).
    return max_obstacle_distance_;
  }
  return it->second;
}

void DynamicGraph::clear(){
  graph_.clear();
}

void DynamicGraph::initial(std::size_t n, double max_obstacle_distance){
  max_obstacle_distance_ = max_obstacle_distance;
  graph_.clear();
  graph_.reserve(n + 1);
  for(unsigned int i=0;i<=n;i++){
    graph_[i] = max_obstacle_distance_;
  }
}

}
