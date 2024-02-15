//
// Created by 汪子琦 on 06.12.2023.
//

#ifndef CMAKELISTS_TXT_SUBSETTREE_H
#define CMAKELISTS_TXT_SUBSETTREE_H

#include <vector>
#include <memory>
#include <set>
#include <nlohmann/json.hpp>

namespace search
{

class SubsetTree{
public:

    struct SubsetTreeNode{
        int partID;
        std::shared_ptr<SubsetTreeNode> childrens[2];
        std::set<int> subset;
    };

private:

    std::shared_ptr<SubsetTreeNode> root_;

    int maximumPartID_;

private:

    std::vector<std::set<int>> data_;

public:

    SubsetTree(){

    }

    SubsetTree(std::vector<std::set<int>> &lists){
        data_ = lists;
        init(lists);
    }

    void init(std::vector<std::set<int>> &lists);

    std::shared_ptr<SubsetTree::SubsetTreeNode> build(std::vector<std::set<int>> &lists, int partID);

    bool checkSupersetExist(const std::set<int> &partIDs);

    bool checkSupersetExist(const std::set<int> &partIDs, std::shared_ptr<SubsetTreeNode> node);

    bool checkSubsetExist(const std::set<int> &partIDs);

    bool checkSubsetExist(const std::set<int> &partIDs, std::shared_ptr<SubsetTreeNode> node);

    void from_json(nlohmann::json input);

    nlohmann::json to_json();

};
}


#endif  //CMAKELISTS_TXT_SUBSETTREE_H
