//
// Created by 汪子琦 on 04.09.22.
//

#include "search/PartGraph.h"
namespace search 
{
PartGraph::PartGraph(std::shared_ptr<rigid_block::Assembly> assembly)
{
    assembly_ = assembly;

    nodes_.resize(assembly->blocks_.size());
    for(int id = 0; id < nodes_.size(); id++){
        nodes_[id].visited = false;
        nodes_[id].grounded = assembly->blocks_[id]->ground_;
    }

    analyzer_ = assembly->createAnalyzer(true);

    std::vector<std::set<unsigned >> adjacency;
    adjacency.resize(analyzer_->n_part());
    for(int contact_id = 0; contact_id < analyzer_->n_contact(); contact_id++) {
        auto contact = analyzer_->contact(contact_id);
        adjacency[contact.partIDA].insert(contact.partIDB);
        adjacency[contact.partIDB].insert(contact.partIDA);
    }

    for(int id = 0; id < adjacency.size(); id++){
        PartGraphEdge edge;
        edge.partIDs = std::vector<unsigned>(adjacency[id].begin(), adjacency[id].end());
        edges_.push_back(edge);
    }
}

void PartGraph::clearNodesVisit()
{
    for (int id = 0; id < nodes_.size(); id++)
    {
        nodes_[id].visited = false;
    }
}

void PartGraph::setNodesVisit(const std::vector<int>& nodeIDs)
{
    for (int id = 0; id < nodeIDs.size(); id++)
    {
        int nID = nodeIDs[id];
        nodes_[nID].visited = true;
    }
}

void PartGraph::computeNodesNeighbour(const std::vector<int>& input_node_inds,
                                      std::vector<int>& output_node_inds,
                                      int nRing)
{
    clearNodesVisit();
    setNodesVisit(input_node_inds);

    std::vector<int> curr_node_inds = input_node_inds;
    for (int iR = 0; iR < nRing; iR++)
    {
        std::vector<int> adjacent_node_inds;
        computeNodesNeighbour(curr_node_inds, adjacent_node_inds);
        setNodesVisit(adjacent_node_inds);
        output_node_inds.insert(output_node_inds.end(), adjacent_node_inds.begin(), adjacent_node_inds.end());
        curr_node_inds = adjacent_node_inds;
    }

    for (int id = 0; id < nodes_.size(); id++)
    {
        if (!nodes_[id].visited && nodes_[id].grounded) {
            output_node_inds.push_back(id);
        }
    }
}

void PartGraph::computeNodesNeighbour(const std::vector<int>& input_node_inds, std::vector<int>& output_node_inds)
{
    std::set<int> neighbour_set;

    for (int id = 0; id < input_node_inds.size(); id++)
    {
        int node_ind = input_node_inds[id];
        for (int jd = 0; jd < edges_[node_ind].partIDs.size(); jd++)
        {
            int adjacent_node_ind = edges_[node_ind].partIDs[jd];
            if (nodes_[adjacent_node_ind].visited == false)
            {
                neighbour_set.insert(adjacent_node_ind);
            }
        }
    }

    for (auto it = neighbour_set.begin(); it != neighbour_set.end(); it++)
    {
        output_node_inds.push_back(*it);
    }

    return;
}

void PartGraph::computeNodesNeighbour(const std::vector<int>& input_node_inds,
                                      const std::vector<int>& subset_node_inds,
                                      std::vector<int>& output_node_inds)
{
    std::vector<int> adjacent_node_inds;
    computeNodesNeighbour(input_node_inds, adjacent_node_inds, 1);

    output_node_inds.clear();
    for (int id = 0; id < adjacent_node_inds.size(); id++)
    {
        auto find_it = std::find(subset_node_inds.begin(), subset_node_inds.end(), adjacent_node_inds[id]);
        if (find_it != subset_node_inds.end())
        {
            output_node_inds.push_back(adjacent_node_inds[id]);
        }
    }
    return;
}

void PartGraph::writeDotGraph(std::string filename)
{
    std::ofstream fout(filename);
    std::string buffer = DotGraphString();
    fout << buffer;
    fout.close();
}

std::string PartGraph::DotGraphString() {
    std::stringstream buffer;

    buffer << "graph {\n";
    for (int id = 0; id < nodes_.size(); id++) {
        buffer << id << "[label = \" " << id << "\"]\n";
    }

    for (int id = 0; id < edges_.size(); id++) {
        for (int jd = 0; jd < edges_[id].partIDs.size(); jd++) {
            if (id < edges_[id].partIDs[jd])
                buffer << id << "--" << edges_[id].partIDs[jd] << std::endl;
        }
    }
    buffer << "}\n";
    return buffer.str();
}

double PartGraph::evaluateStability(const std::vector<int> &subPartIDs, const std::vector<int> &fixedPartIDs) {
    std::vector<rigid_block::Analyzer::PartStatus> status;
    status.resize(nodes_.size(), rigid_block::Analyzer::Uninstalled);
    for(int partID : subPartIDs) {
        status[partID] = rigid_block::Analyzer::Installed;
    }
    for(int partID : fixedPartIDs) {
        status[partID] = rigid_block::Analyzer::Fixed;
    }
    rigid_block::AnalysisResult result;
    return analyzer_->solve(status, result);
}

std::vector<bool> PartGraph::computePartsGround() {
    std::vector<bool> parts;
    for(int part_id = 0; part_id < analyzer_->n_part(); part_id++) {
        parts.push_back(assembly_->blocks_[part_id]->ground_);
    }
    return parts;
}

}