//
// Created by 汪子琦 on 04.09.22.
//

#ifndef PARTGRAPH_H
#define PARTGRAPH_H

#include <vector>
#include <set>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include "rigid_block/Analyzer.h"
#include "rigid_block/Assembly.h"
namespace search
{

class PartGraphNode
{
public:
    unsigned partID;
    bool grounded;
    bool visited;
};

class PartGraphEdge
{
public:
    std::vector<unsigned> partIDs;
};

class PartGraph
{
protected:

    std::vector<PartGraphNode> nodes_;

    std::vector<PartGraphEdge> edges_;

    std::shared_ptr<rigid_block::Assembly> assembly_;

    std::shared_ptr<rigid_block::Analyzer> analyzer_;

public:

    PartGraph(std::shared_ptr<rigid_block::Assembly> assembly);

public:

    int nNode() const {return nodes_.size();}

    void clearNodesVisit();

    void setNodesVisit(const std::vector<int> &nodeIDs);

public:

    void computeNodesNeighbour(const std::vector<int>& input_node_inds, std::vector<int> &output_node_inds, int nRing);

    void computeNodesNeighbour(const std::vector<int>& input_node_inds, const std::vector<int>& subset_node_inds, std::vector<int> &output_node_inds);

    std::vector<unsigned > computeNodeNeighbour(int node_ind){return edges_[node_ind].partIDs;}

    std::vector<bool> computePartsGround();

private:

    void computeNodesNeighbour(const std::vector<int>& input_node_inds, std::vector<int> &output_node_inds);

public:

    void writeDotGraph(std::string filename);

    std::string DotGraphString();

public:

    virtual double evaluateStability(const std::vector<int> &subPartIDs, const std::vector<int> &fixedPartIDs);

    virtual bool evaluateAssemblability(const std::vector<int> &subPartIDs){
        return true;
    }

    virtual double volume(int partID){
        return analyzer_->mass(partID);
    }

    virtual double height(int partID){
        return analyzer_->centroid(partID).y();
    }
};
}

#endif  //PARTGRAPH_H
