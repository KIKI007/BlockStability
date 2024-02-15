//
// Created by 汪子琦 on 06.12.2023.
//

#include <search/SubsetTree.h>
#include <iostream>

namespace search {

std::shared_ptr<SubsetTree::SubsetTreeNode> SubsetTree::build(std::vector<std::set<int>> &lists, int partID)
{
    //only one set

    if(lists.size() == 1){
        std::shared_ptr<SubsetTreeNode> node = std::make_shared<SubsetTreeNode>();
        node->partID = -1;
        node->subset = lists.front();
        return node;
    }

    //termination
    if(partID > maximumPartID_ + 1){
        return nullptr;
    }

    int maxList = 0;
    for(auto &list : lists)
    {
        maxList = std::max(maxList, *std::max_element(list.begin(), list.end()));
    }

    // find the smallest partID that is larger (equal) then the current partID
    // the nextID should not be larger than the maximum of the lists
    bool foundPart = false;
    int nextPartID = partID;
    for (; nextPartID <= maxList; nextPartID++) {
        for (std::set<int> &list : lists) {
            if (list.find(nextPartID) != list.end()) {
                foundPart = true;
                break;
            }
        }

        if (foundPart) {
            break;
        }
    }

    //if such partID does not exist, return null
    if(nextPartID > maxList){
        return nullptr;
    }

    std::vector<std::set<int>> HasList;
    std::vector<std::set<int>> NoList;

    for (std::set<int> &list : lists) {
        if (list.find(nextPartID) != list.end()) {
            HasList.push_back(list);
        } else {
            NoList.push_back(list);
        }
    }

    std::shared_ptr<SubsetTreeNode> node = std::make_shared<SubsetTreeNode>();
    node->partID = nextPartID;
    node->subset = {};

    if (!NoList.empty())
        node->childrens[0] = build(NoList, nextPartID + 1);

    if (!HasList.empty())
        node->childrens[1] = build(HasList, nextPartID + 1);

    for (std::set<int> &list : lists) {
        if(*std::max_element(list.begin(), list.end()) == partID){
            node->subset = list;
        }
    }

    return node;
}

void SubsetTree::init(std::vector<std::set<int>> &lists) {
    {
        maximumPartID_ = 0;
        for(int id = 0; id < lists.size(); id++)
        {
            maximumPartID_ = std::max(maximumPartID_, *std::max_element(lists[id].begin(), lists[id].end()));
        }

        root_ = build(lists, 0);
    }
}

bool SubsetTree::checkSupersetExist(const std::set<int> &partIDs)
{
    int minPartinTree = root_->partID;
    if(minPartinTree > *std::min_element(partIDs.begin(), partIDs.end())){
        return false;
    }
    return checkSupersetExist(partIDs, root_);
}

bool SubsetTree::checkSupersetExist(const std::set<int> &partIDs, std::shared_ptr<SubsetTreeNode> node)
{
    if(node == nullptr){
        return false;
    }

    int partID = node->partID;
    if(!node->subset.empty()) {
        //partIDs must be in the node's subset
        if (std::includes(node->subset.begin(), node->subset.end(), partIDs.begin(), partIDs.end())) {
            return true;
        }
    }

    if(partIDs.find(partID) != partIDs.end())
    {
        //has the part
        //must choose the right side

        if(node->childrens[1] != nullptr)
        {
            if(checkSupersetExist(partIDs, node->childrens[1])){
                return true;
            }
        }
    }
    else
    {
        //does not have the part
        //can choose both side
        for(int id = 0; id < 2; id++)
        {
            if(node->childrens[id] != nullptr)
            {
                if(checkSupersetExist(partIDs, node->childrens[id])){
                    return true;
                }
            }
        }
    }

    return false;
}

bool SubsetTree::checkSubsetExist(const std::set<int> &partIDs)
{
    return checkSubsetExist(partIDs, root_);
}

bool SubsetTree::checkSubsetExist(const std::set<int> &partIDs, std::shared_ptr<SubsetTreeNode> node)
{
    if(node == nullptr)
    {
        return false;
    }

    int partID = node->partID;

    if(!node->subset.empty()){
        //node must be in the partIDs
        if(std::includes(partIDs.begin(), partIDs.end(), node->subset.begin(), node->subset.end())){
           return true;
        }
    }
    if(partIDs.find(partID) == partIDs.end())
    {
        //does not have the part
        //must choose the left side
        if(node->childrens[0] != nullptr)
        {
           if(checkSubsetExist(partIDs, node->childrens[0])){
                return true;
           }
        }
    }
    else
    {
        //has the part
        //can choose both side
        for(int id = 0; id < 2; id++)
        {
           if(node->childrens[id] != nullptr)
           {
                if(checkSubsetExist(partIDs, node->childrens[id])){
                    return true;
                }
           }
        }
    }

    return false;
}

void SubsetTree::from_json(nlohmann::json input)
{
    data_.clear();

    for(int id = 0; id < input.size(); id++)
    {
        std::set<int> list;
        for(int jd = 0; jd < input[id].size(); jd++){
           list.insert(input[id][jd].get<int>());
        }
        data_.push_back(list);
    }
    init(data_);
}

nlohmann::json SubsetTree::to_json() {
    nlohmann::json result;
    for(int id = 0; id < data_.size(); id++){
        nlohmann::json node;
        for(auto it = data_[id].begin(); it != data_[id].end(); ++it){
           node.push_back(*it);
        }
        result.push_back(node);
    }
    return result;
}

}  // namespace search