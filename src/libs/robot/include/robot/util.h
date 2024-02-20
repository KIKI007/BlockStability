//
// Created by Ziqi Wang on 16.02.2024.
//

#ifndef UTIL_H
#define UTIL_H

namespace robot
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
}
#endif //UTIL_H
