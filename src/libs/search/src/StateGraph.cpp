//
// Created by 汪子琦 on 14.03.22.
//

#include "search/StateGraph.h"
#include <tbb/parallel_for.h>

namespace search {

/*******************************************************************************************************************************
 *                                                      StateGraph
 ********************************************************************************************************************************/

int StateGraph::numDigitPerMode() {
    return std::ceil(std::log2(nMode_));
}

int StateGraph::numPartPerChunk() {
    int num_digit_per_mode = numDigitPerMode();
    int num_part_per_chunk = chunkBit_ / num_digit_per_mode;
    return num_part_per_chunk;
}

int StateGraph::numChunkRequired() {
    int num_part_per_chunk = numPartPerChunk();
    int num_chunk = std::ceil((double)nPart_ / num_part_per_chunk);
    return num_chunk;
}

unsigned StateGraph::modeMask(int N) {
    unsigned mask = 0;
    for (int id = 0; id < N; id++) {
        mask += 1 << (id);
    }
    return mask;
}

std::tuple<unsigned, unsigned> StateGraph::computeChunkOffset(unsigned partID) {
    int num_digit_per_mode = numDigitPerMode();
    int num_part_per_chunk = chunkBit_ / num_digit_per_mode;
    int offset0 = partID / num_part_per_chunk;
    unsigned offset1 = (partID % num_part_per_chunk) * num_digit_per_mode;
    return std::make_tuple(offset0, offset1);
}

int StateGraph::getChunk(const StateGraphNode &node, int offset) {
    return assemblyStates_[node.chunkOffset + offset];
}

unsigned StateGraph::computePartState(unsigned partID, const StateGraphNode &node) {
    auto [offset0, offset1] = computeChunkOffset(partID);
    return (getChunk(node, offset0) & (modeMask(numDigitPerMode()) << offset1)) >> offset1;
}

unsigned StateGraph::computePartState(unsigned int partID, const std::vector<unsigned> &assemblyState) {
    auto [offset0, offset1] = computeChunkOffset(partID);
    return (assemblyState[offset0] & (modeMask(numDigitPerMode()) << offset1)) >> offset1;
}

std::vector<unsigned> StateGraph::setPartMode(unsigned partID, int partState, const StateGraphNode &node) {
    auto [offset0, offset1] = computeChunkOffset(partID);
    std::vector<unsigned> assemblyState = getAssemblyState(node);
    unsigned prevPartState = assemblyState[offset0] & (modeMask(numDigitPerMode()) << offset1);
    assemblyState[offset0] -= prevPartState;
    assemblyState[offset0] += partState << offset1;
    return assemblyState;
}

void StateGraph::setPartMode(unsigned int partID, int partState, std::vector<unsigned int> &assemblyState) {
    auto [offset0, offset1] = computeChunkOffset(partID);
    unsigned prevPartState = assemblyState[offset0] & (modeMask(numDigitPerMode()) << offset1);
    assemblyState[offset0] -= prevPartState;
    assemblyState[offset0] += partState << offset1;
}

std::vector<unsigned> StateGraph::getAssemblyState(const StateGraphNode &node) {
    std::vector<unsigned> assemblyState;
    for (int id = 0; id < nChunk_; id++) {
        assemblyState.push_back(getChunk(node, id));
    }
    return assemblyState;
}

std::vector<int> StateGraph::getInstalledParts(PtrN node) {
    std::vector<int> parts;
    for (int id = 0; id < nPart_; id++) {
        unsigned state = computePartState(id, *node);
        if (state > 0) {
            parts.push_back(id);
        }
    }
    return parts;
}

std::vector<int> StateGraph::getInstalledParts(const std::vector<unsigned int> &assemblyState) {
    std::vector<int> parts;
    for (int id = 0; id < nPart_; id++) {
        unsigned part_state = computePartState(id, assemblyState);
        if (part_state > 0) {
            parts.push_back(id);
        }
    }
    return parts;
}

std::vector<int> StateGraph::getFixedParts(const State &state) {
    std::vector<int> fixed_part_ids;
    for(int part_id = 0; part_id < nPart_; part_id++){
        unsigned part_state = computePartState(part_id, state);
        if (part_state >= 2){
            fixed_part_ids.push_back(part_id);
        }
    }
    return fixed_part_ids;
}
std::vector<int> StateGraph::getFixedParts(StateGraph::PtrN node) {
    std::vector<int> fixed_part_ids;
    for(int part_id = 0; part_id < nPart_; part_id++){
        unsigned part_state = computePartState(part_id, *node);
        if (part_state >= 2){
            fixed_part_ids.push_back(part_id);
        }
    }
    return fixed_part_ids;
}

std::string StateGraph::node_label(PtrN node) {
    std::vector<int> parts = getInstalledParts(node);
    std::string node_label = "\"";
    for (int id = 0; id < parts.size(); id++) {
        node_label += std::to_string(parts[id]);
        if (id + 1 != parts.size()) {
            node_label += " ,";
        }
    }
    node_label += "\"";
    return node_label;
}

std::string StateGraph::DotGraphString() {
    std::stringstream buffer;
    buffer << "digraph {\nrankdir=LR;\n";
    for (int id = 0; id < nodes_.size(); id++) {
        if (visibility(id)) {
            buffer << id << " [label = " << node_label(nodes_[id]) << "]\n";
        }
    }

    for (int id = 0; id < nodes_.size(); id++)
    {
        for (int jd = nodeEdgeOffsetStart_[id]; jd < nodeEdgeOffsetEnd_[id]; jd++)
        {
            int adj_node = edges_[jd];
            if (visibility(id) && visibility(adj_node))
            {
                buffer << id << "-> " << adj_node << std::endl;
            }
        }
    }

    buffer << "}\n";
    return buffer.str();
}

void StateGraph::writeDotGraph(std::string filename) {
    std::ofstream fout(filename);
    std::string buffer = DotGraphString();
    fout << buffer;
    fout.close();
}

std::tuple<int, bool> StateGraph::addNewNode(std::vector<unsigned int> &newAssemblyState)
{
    auto find_it = map_astate_nodeID_.find(newAssemblyState);
    int newNodeID = -1;
    bool createNewNode = false;
    if (find_it == map_astate_nodeID_.end()) {
        assemblyStates_.insert(assemblyStates_.end(), newAssemblyState.begin(), newAssemblyState.end());
        PtrN newNode = std::make_shared<StateGraphNode>();

        int index = assemblyStates_.size() - nChunk_;
        newNode->chunkOffset = index;

        newNodeID = nodes_.size();
        newNode->nodeID = newNodeID;
        newNode->currentCost = std::numeric_limits<double>::max();
        newNode->parent = -1;

        nodes_.push_back(newNode);
        map_astate_nodeID_[newAssemblyState] = newNodeID;
        createNewNode = true;
    } else {
        newNodeID = find_it->second;
    }
    return std::make_tuple(newNodeID, createNewNode);
}


void StateGraph::enumerateAssemblyStates() {
    createRootNode();

    int preStepOffset = 0;
    int currStepOffset = nodes_.size();
    while(currStepOffset != preStepOffset)
    {
        enumerateNextAssemblyStates(preStepOffset, currStepOffset);
        preStepOffset = currStepOffset;
        currStepOffset = nodes_.size();
        if(checkEndNode(nodes_.back())){
            return ;
        }
    }
}

void StateGraph::enumerateNextAssemblyStates(int preStepOffset, int currStepOffset)
{
    std::map<std::vector<unsigned>, int, StateCompare> map_astate_nodeID;

    //collect nodes in the current layer
    std::vector<PtrN> inputNodes;
    for (int iS = preStepOffset; iS < currStepOffset; iS++) {
        inputNodes.push_back(nodes_[iS]);
    }

    int prevNumNode = nodes_.size();

    //expand into another layer
    std::vector<StateGraphEdge> edgeItems;
    expandNodes(inputNodes, edgeItems);
    int currNumNode = nodes_.size();

    //sort edges
    std::sort(edgeItems.begin(), edgeItems.end(), [](StateGraphEdge &edgeA, StateGraphEdge &edgeB){
        return edgeA.prev_node->nodeID < edgeB.prev_node->nodeID;
    });

    for (int id = prevNumNode; id < currNumNode; id++)
    {
        nodeEdgeOffsetStart_.push_back(0);
        nodeEdgeOffsetEnd_.push_back(0);
    }

    for (int id = 0; id < edgeItems.size(); id++)
    {
        PtrN next_node = edgeItems[id].next_node;
        PtrN prev_node = edgeItems[id].prev_node;

        if(id == 0 || edgeItems[id].prev_node->nodeID != edgeItems[id - 1].prev_node->nodeID)
        {
            nodeEdgeOffsetStart_[prev_node->nodeID] = edges_.size();
            nodeEdgeOffsetEnd_[prev_node->nodeID] = edges_.size() + 1;
        }
        else{
            nodeEdgeOffsetEnd_[prev_node->nodeID]++;
        }

        edges_.push_back(next_node->nodeID);
        edges_items_.push_back(edgeItems[id]);
    }
}

void StateGraph::createRootNode() {
    std::vector<unsigned> assemblyState;
    assemblyState.resize(nChunk_, 0);
    addNewNode(assemblyState);
    nodeEdgeOffsetStart_.push_back(0);
    nodeEdgeOffsetEnd_.push_back(0);
}

bool StateGraph::checkEndNode(StateGraph::PtrN node)
{
    return getInstalledParts(node).size() == nPart_;
}

std::vector<int> StateGraph::getNewInstalledParts(const State &prevNode, const State &currNode)
{
    std::vector<int> prevNodePartIDs = getInstalledParts(prevNode);
    std::vector<int> currNodePartIDs = getInstalledParts(currNode);
    std::vector<int> newInstalledParts;
    std::sort(currNodePartIDs.begin(), currNodePartIDs.end());
    std::sort(prevNodePartIDs.begin(), prevNodePartIDs.end());
    std::set_difference(currNodePartIDs.begin(),
                        currNodePartIDs.end(),
                        prevNodePartIDs.begin(),
                        prevNodePartIDs.end(),
                        std::back_inserter(newInstalledParts));
    return newInstalledParts;
}

void StateGraph::setStartnEnd(const std::vector<int> &start, const std::vector<int> &end) {
    if(start.empty() && end.empty())
    {
        startPartIDs_ = {};
        endPartIDs_ = {};
        for(int id = 0; id < nPart_; id++){
            endPartIDs_.push_back(id);
        }
    }
    else{
        startPartIDs_ = start;
        endPartIDs_ = end;
    }
}

void StateGraph::to_json(nlohmann::json &result)
{
    result["search"]["start_part_ids"] = startPartIDs_;
    result["search"]["end_part_ids"] = endPartIDs_;
    result["search"]["boundary_part_ids"] = boundaryPartIDs_;
}

void StateGraph::from_json(const nlohmann::json &input) {
    if(input.contains("search"))
    {
        if(input["search"].contains("start_part_ids")){
            startPartIDs_ = input["search"]["start_part_ids"].get<std::vector<int>>();
        }
        if(input["search"].contains("end_part_ids")){
            endPartIDs_ = input["search"]["end_part_ids"].get<std::vector<int>>();
        }
        if(input["search"].contains("boundary_part_ids")){
            boundaryPartIDs_ = input["search"]["boundary_part_ids"].get<std::vector<int>>();
        }
    }
    else{
        boundaryPartIDs_ = {};
        setStartnEnd({}, {});
    }
}
}
