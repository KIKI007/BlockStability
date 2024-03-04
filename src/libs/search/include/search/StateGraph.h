//
// Created by 汪子琦 on 10.03.22.
//

#ifndef ROBO_CRAFT_STATEGRAPH_H
#define ROBO_CRAFT_STATEGRAPH_H

#include <cmath>
#include <memory>
#include <tuple>
#include <vector>
#include <map>
#include "PartGraph.h"
#include "AssemblySequence.h"

namespace search
{

class StateGraphNode
{
public:
    int chunkOffset;
    int nodeID;

    double currentCost;
    int parent;
};

class StateGraphEdge{
public:
    std::string label;
    std::shared_ptr<StateGraphNode> prev_node, next_node;
};

struct StateCompare {
    bool operator()(const std::vector<unsigned> &nodeA, const std::vector<unsigned> &nodeB) const {
        for (int id = 0; id < nodeA.size(); id++) {
            if (nodeA[id] < nodeB[id]) {
                return true;
            }
            if (nodeA[id] > nodeB[id]) {
                return false;
            }
        }
        return false;
    }
};

/*
 * part_mode: Every part has its own mode (uninstalled/installed/fixed ..)
 * assembly_state: Combine all parts' mode as an integer list.
*/

class StateGraph : public std::enable_shared_from_this<StateGraph>
{
public:
    typedef std::shared_ptr<StateGraphNode> PtrN;

    typedef std::vector<unsigned int> State;

    std::vector<int> startPartIDs_;

    std::vector<int> endPartIDs_;

    std::vector<int> boundaryPartIDs_;

public:

    int nPart_;  //number of parts

    int nMode_;  //number of state for each part

    int nChunk_;  //number of chunk required for describing the states of the assembly

    const int chunkBit_ = 32;  //the number of bit each chuck can have

public:
    std::vector<unsigned> assemblyStates_;

    std::shared_ptr<PartGraph> partGraph_;

    std::vector<unsigned long> edges_;

    std::vector<StateGraphEdge> edges_items_;

    std::vector<PtrN> nodes_;

    std::vector<unsigned> nodeEdgeOffsetStart_;

    std::vector<unsigned> nodeEdgeOffsetEnd_;

    std::map<std::vector<unsigned>, unsigned, StateCompare> map_astate_nodeID_;

public:
    StateGraph() {
        nMode_ = nPart_ = 0;
        nChunk_ = 0;
        boundaryPartIDs_ = {};
        setStartnEnd({}, {});
    }

    StateGraph(int nPart, int nMode) : nPart_(nPart), nMode_(nMode) {
        nChunk_ = numChunkRequired();
        boundaryPartIDs_ = {};
        setStartnEnd({}, {});
    }

    StateGraph(std::shared_ptr<PartGraph> graph, int nMode) : partGraph_(graph), nMode_(nMode) {
        nPart_ = graph->nNode();
        nChunk_ = numChunkRequired();
        boundaryPartIDs_ = {};
        setStartnEnd({}, {});
    }

    ~StateGraph(){

    }

public:
    virtual void clear() {
        assemblyStates_.clear();
        nodeEdgeOffsetEnd_.clear();
        nodeEdgeOffsetStart_.clear();
        nodes_.clear();
        edges_.clear();
        edges_items_.clear();
        map_astate_nodeID_.clear();
    }

    void setStartnEnd(const std::vector<int> &start, const std::vector<int> &end);

    int numDigitPerMode();

    int numPartPerChunk();

    int numChunkRequired();

    unsigned modeMask(int N);

    std::tuple<unsigned, unsigned> computeChunkOffset(unsigned partID);

    int getChunk(const StateGraphNode &node, int offset);

    unsigned computePartState(unsigned partID, const StateGraphNode &node);

    unsigned computePartState(unsigned partID, const std::vector<unsigned> &assemblyState);

    void setPartMode(unsigned partID, int partState, std::vector<unsigned> &assemblyState);

    std::vector<unsigned> setPartMode(unsigned partID, int partState, const StateGraphNode &node);

    std::vector<unsigned> getAssemblyState(const StateGraphNode &node);

    virtual std::tuple<int, bool> addNewNode(std::vector<unsigned> &newAssemblyState);

    std::vector<int> getInstalledParts(PtrN node);

    std::vector<int> getInstalledParts(const std::vector<unsigned> &assemblyState);

    std::vector<int> getNewInstalledParts(const State &pre, const State &curr);

    std::vector<int> getFixedParts(const State &state);

    std::vector<int> getFixedParts(PtrN node);

    virtual std::string node_label(PtrN node);

    void writeDotGraph(std::string filename);

    std::string DotGraphString();

    bool virtual visibility(int nodeID) {
        return true;
    }

    void to_json(nlohmann::json &result);

    void from_json(const nlohmann::json &input);

public:
    virtual void createRootNode();

    virtual bool checkEndNode(PtrN node);

    virtual std::vector<unsigned> finishedAssemblyState() {
        std::vector<unsigned> fullAssemblyState;
        fullAssemblyState.resize(nChunk_, 0);
        for (int id = 0; id < nPart_; id++) {
            setPartMode(id, 1, fullAssemblyState);
        }
        return fullAssemblyState;
    }

    virtual void expandNodes(std::vector<PtrN> &input_nodes,
                             std::vector<StateGraphEdge> &edgeItems){

    }

    void enumerateAssemblyStates();

    std::vector<int> computeStablestSubAssemblyWithKParts(int K);

    virtual void enumerateNextAssemblyStates(int preStepOffset, int currStepOffset);

    //the nodes must follow the assembling order
    virtual void getSolution(std::vector<PtrN> nodes, AssemblySequence &sequence) {}

    std::string deform_to_string(double deform){
        double deformation = deform * 1E3;
        char deformation_str[256];
        sprintf(deformation_str, "%.1f mm", deformation);
        std::string deformation_string(deformation_str);
        return deformation_string;
    }

    void print(StateGraph::State state) {
        std::vector<int> installed_beams = getInstalledParts(state);
        std::vector<int> fixed_beams = getFixedParts(state);

        for(int id = 0; id < installed_beams.size(); id++){
            int part_id = installed_beams[id];
            std::cout << part_id;
            if(std::find(fixed_beams.begin(), fixed_beams.end(), part_id) != fixed_beams.end()){
                std::cout << "*";
            }
            std::cout << " ";
        }
        std::cout << std::endl;
    }

};
}
#endif  //ROBO_CRAFT_STATEGRAPH_H
