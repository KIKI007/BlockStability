//
// Created by 汪子琦 on 05.09.22.
//

#include "rigid_block/util/readOBJ.h"
#include <fstream>
#include <iostream>

namespace rigid_block {
    void readOBJ::split(std::vector<std::string> &words, const std::string &line, const char &separator) {
        int prev_it = 0;
        int it = 0;
        for (it = 0; it < line.size(); it++) {
            if (line[it] == separator) {
                std::string substr = line.substr(prev_it, it - prev_it);
                words.push_back(substr);
                prev_it = it + 1;
            }
        }
        if (prev_it != it) {
            std::string substr = line.substr(prev_it, it - prev_it);
            words.push_back(substr);
        }
    }

    void readOBJ::loadFromFile(std::string filename) {
        Vs_.clear();
        Fs_.clear();

        std::ifstream fin(filename);
        std::vector<Eigen::Vector3d> vertices;
        std::vector<Eigen::Vector3i> faces;
        std::vector<Eigen::Vector3d> normals;

        if (fin.is_open()) {
            std::string line;
            while (std::getline(fin, line)) {
                std::vector<std::string> words;
                split(words, line, ' ');
                if (words.empty())
                    continue;

                if (words[0] == "v") {
                    addVertex(line, vertices);
                } else if (words[0] == "vn") {
                    addNormal(line, normals);
                } else if (words[0] == "f") {
                    addFace(line, faces);
                } else if (words[0] == "o") {
                    addObject(vertices, faces, normals);
                    vertices.clear();
                    faces.clear();
                    normals.clear();
                }
            }
            addObject(vertices, faces, normals);
            fin.close();
        }
    }

    void readOBJ::addVertex(const std::string &line, std::vector<Eigen::Vector3d> &vV) {
        std::vector<std::string> words;
        split(words, line, ' ');

        Eigen::Vector3d pt;
        for (int id = 1; id < words.size() && id <= 3; id++) {
            pt(id - 1) = std::atof(words[id].c_str());
        }
        vV.push_back(pt);
    }

    void readOBJ::addNormal(const std::string &line, std::vector<Eigen::Vector3d> &vN) {
        std::vector<std::string> words;
        split(words, line, ' ');
        Eigen::Vector3d normal;
        for (int id = 1; id < words.size() && id <= 3; id++) {
            normal(id - 1) = std::atof(words[id].c_str());
        }
        vN.push_back(normal);
    }

    void readOBJ::addFace(const std::string &line, std::vector<Eigen::Vector3i> &vF) {
        std::vector<std::string> fi;
        split(fi, line, ' ');

        std::vector<int> fIndices;
        for (int id = 1; id < fi.size(); id++) {
            std::vector<std::string> fij;
            split(fij, fi[id], ' ');
            int fIndex = std::atoi(fij[0].c_str()) - 1 - vCount;
            fIndices.push_back(fIndex);
        }

        for (int id = 2; id < fIndices.size(); id++) {
            int f0 = fIndices[0];
            int f1 = fIndices[id - 1];
            int f2 = fIndices[id];
            vF.push_back(Eigen::Vector3i(f0, f1, f2));
        }
        return;
    }

    void readOBJ::addObject(const std::vector<Eigen::Vector3d> &vV, const std::vector<Eigen::Vector3i> &vF,
                            const std::vector<Eigen::Vector3d> &vN) {
        if (!vV.empty() && !vF.empty() && !vN.empty()) {
            Eigen::MatrixXd V, N;
            Eigen::MatrixXi F;
            list_to_matrix(vV, V);
            list_to_matrix(vF, F);
            list_to_matrix(vN, N);
            Vs_.push_back(V);
            Fs_.push_back(F);
            Ns_.push_back(N);
            vCount += V.rows();
        }
    }

    void readOBJ::list_to_matrix(const std::vector<Eigen::Vector3d> &vV, Eigen::MatrixXd &V) {
        V = Eigen::MatrixXd(vV.size(), 3);
        for (int id = 0; id < vV.size(); id++) {
            for (int jd = 0; jd < 3; jd++) {
                V(id, jd) = vV[id][jd];
            }
        }
    }

    void readOBJ::list_to_matrix(const std::vector<Eigen::Vector3i> &vF, Eigen::MatrixXi &F) {
        F = Eigen::MatrixXi(vF.size(), 3);
        for (int id = 0; id < vF.size(); id++) {
            for (int jd = 0; jd < 3; jd++) {
                F(id, jd) = vF[id][jd];
            }
        }
    }
}
