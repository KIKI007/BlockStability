//
// Created by 汪子琦 on 05.09.22.
//

#ifndef ROBO_CRAFT_READOBJ_H
#define ROBO_CRAFT_READOBJ_H
#include <Eigen/Dense>
#include <vector>

namespace util
{
    inline std::vector<std::string> split(const std::string &line, const char &separator)
    {
        std::vector<std::string> words;
        int prev_it = 0;
        int it = 0;
        for (it = 0; it < line.size(); it++) {
            if (line[it] == separator) {
                std::string substr = line.substr(prev_it, it - prev_it);
                words.push_back(substr);
                while(line[it + 1] == separator) {
                    it++;
                }
                prev_it = it + 1;
            }
        }
        if (prev_it != it) {
            std::string substr = line.substr(prev_it, it - prev_it);
            words.push_back(substr);
        }
        return words;
    }

    inline Eigen::Vector3d toVec(const std::string &pos_text)
    {
        auto pos_data = split(pos_text, ' ');
        Eigen::Vector3d pos; pos.setZero();
        for(int id = 0; id < 3; id++) pos(id) = std::atof(pos_data[id].c_str());
        return pos;
    }

    class readOBJ {
    public:
        std::vector<Eigen::MatrixXd> Vs_;

        std::vector<Eigen::MatrixXd> Ns_;

        std::vector<Eigen::MatrixXi> Fs_;

        int vCount = 0;

    public:
        void loadFromFile(std::string filename);

        void addObject(const std::vector<Eigen::Vector3d> &vV, const std::vector<Eigen::Vector3i> &vF,
                       const std::vector<Eigen::Vector3d> &vN);

        void addVertex(const std::string &line, std::vector<Eigen::Vector3d> &vV);

        void addFace(const std::string &line, std::vector<Eigen::Vector3i> &vF);

        void addNormal(const std::string &line, std::vector<Eigen::Vector3d> &vN);

        void list_to_matrix(const std::vector<Eigen::Vector3d> &vV, Eigen::MatrixXd &V);

        void list_to_matrix(const std::vector<Eigen::Vector3i> &vf, Eigen::MatrixXi &F);
    };
}


#endif  //ROBO_CRAFT_READOBJ_H
