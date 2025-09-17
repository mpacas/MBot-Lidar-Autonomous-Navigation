#ifndef AUTONOMOUS_NAVIGATION_UTILS_SERVER_UTILS_H
#define AUTONOMOUS_NAVIGATION_UTILS_SERVER_UTILS_H

#include <map>
#include <vector>
#include <memory>
#include <string>
#include <future>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <regex>

#include "graph_utils.h"

#include <simple-ws-server/server_ws.hpp>

using WsServer = SimpleWeb::SocketServer<SimpleWeb::WS>;

namespace server_utils
{

static inline std::string strip(const std::string& s)
{
    std::string r = s;
    r.erase(std::remove(r.begin(), r.end(), ' '), r.end());
    return r;
}

static inline std::string stripQuotes(const std::string& s)
{
    std::string r = s;
    r.erase(std::remove(r.begin(), r.end(), '\"'), r.end());
    r.erase(std::remove(r.begin(), r.end(), '\''), r.end());
    return r;
}

// split string
static inline std::vector<std::string> split(const std::string &s, char delim) {
    std::stringstream ss(s);
    std::string item;
    std::vector<std::string> tokens;
    while (std::getline(ss, item, delim)) {
        tokens.push_back(item);
    }
    return tokens;
}

// fetch string
static inline std::string fetch(const std::string& s, const std::string& keyword) {
    std::string exp_str = ".*" + keyword + "\":\"(.*?)\"";
    std::string exp_dict = ".*" + keyword + "\":\\{(.*?)\\}";

    std::smatch sm;
    std::regex e_str(exp_str);
    std::regex e_dict(exp_dict);

    if (std::regex_search(s, sm, e_str))
    {
        auto match = sm[sm.size() - 1];
        return match;
    }
    else if (std::regex_search(s, sm, e_dict))
    {
        auto match = sm[sm.size() - 1];
        return match;
    }

    return "";
}

template <class T>
static inline std::string keyValToJSON(const std::string& key, const T& val)
{
    std::ostringstream oss;
    oss << "\"" << key << "\":" << val;
    return oss.str();
}

static inline std::string keyStringToJSON(const std::string& key, const std::string& val)
{
    std::ostringstream oss;
    oss << "\"" << key << "\":" << val;
    return "\"" + key + "\":\"" + val + "\"";
}

template <class T>
static inline std::string vectorToJSON(const std::vector<T>& vals)
{
    std::ostringstream oss;

    if (vals.size() < 1)
    {
        oss << "[]";
        return oss.str();
    }

    oss << "[";
    for (size_t i = 0; i < vals.size() - 1; ++i)
    {
        oss << std::to_string(vals[i]) << ",";
    }
    oss << std::to_string(vals[vals.size() - 1]) << "]";

    return oss.str();
}

template <class T>
static inline std::string keyValsToJSON(const std::string& key, const std::vector<T>& vals)
{
    std::ostringstream oss;
    oss << "\"" << key << "\": ";
    oss << vectorToJSON(vals);

    return oss.str();
}

static inline std::string keyStringValsToJSON(const std::string& key, const std::vector<std::string>& vals)
{
    std::ostringstream oss;
    oss << "\"" << key << "\": [";
    for (size_t i = 0; i < vals.size() - 1; ++i)
    {
        oss << vals[i] << ",";
    }
    oss << vals[vals.size() - 1] << "]";

    return oss.str();
}

static inline std::string pathToMsg(const std::vector<Cell>& path)
{
    std::ostringstream oss;
    oss << "{";
    // Type info.
    oss << keyStringToJSON("type", "robot_path") << ",";

    std::vector<std::string> cells;
    for (auto& c : path)
    {
        std::vector<int> c_str({c.i, c.j});
        cells.push_back(vectorToJSON(c_str));
    }

    oss << "\"data\": {";
    oss << keyStringValsToJSON("path", cells);
    oss << "}";  // Close data
    oss << "}";  // Close msg.

    return oss.str();
}

static inline std::string cellToMsg(const int i, const int j)
{
    std::ostringstream oss;
    oss << "{";
    // Type info.
    oss << keyStringToJSON("type", "visited_cell") << ",";

    // Cell data.
    std::vector<int> c_vals({i, j});
    oss << "\"data\": {";
    oss << keyValsToJSON("cell", c_vals);
    oss << "}";  // Close data

    oss << "}";  // Close msg.

    return oss.str();
}

static inline std::string fieldToMsg(const std::vector<float>& field)
{
    std::ostringstream oss;
    oss << "{";
    // Type info.
    oss << keyStringToJSON("type", "field") << ",";
    oss << "\"data\": {";
    oss << keyValsToJSON("field", field);
    oss << "}";  // Close data
    oss << "}";  // Close msg.

    return oss.str();
}

static inline void sendVisitedCell(const std::shared_ptr<WsServer::Connection> connection, const int i, const int j)
{
    if (connection)
        connection->send(cellToMsg(i, j));
}

class PlanData
{
public:
    PlanData(const std::map<std::string, std::string>& data)
    {
        algo_ = data.at("algo");
        map_name_ = data.at("map_name");
        start_ = getArrayData(data.at("start"));
        goal_ = getArrayData(data.at("goal"));
    };

    Cell goal() const { return goal_; }
    Cell start() const { return start_; }
    std::string algo() const { return algo_; }
    std::string mapName() const { return map_name_; }

private:
    Cell getArrayData(const std::string& data)
    {
        Cell c;
        if (data.find("[") == std::string::npos)
        {
            std::cout << "Incoming message is not valid: " << data << std::endl;
            return c;
        }

        // Remove first and last brackets.
        std::string raw = data;
        raw.erase(0, raw.find("[") + 1);
        raw.erase(raw.find("]"), raw.find("]") + 1);

        // Get two numbers.
        auto vals = split(raw, ' ');
        c.i = std::stoi(vals[0]);
        c.j = std::stoi(vals[1]);

        return c;
    }

    std::string algo_, map_name_;
    Cell start_, goal_;
};

class InMessageHelper
{
public:
    InMessageHelper(const std::string& in_msg)
    {
        parseInput(in_msg);
    }

    std::string type() const { return type_; }

    std::map<std::string, std::string> getData() const
    {
        return data_;
    }

    bool hasKey(const std::string k) const
    {
        return (data_.find(k) != data_.end());
    }

    std::string getVal(const std::string& k) const
    {
        std::string val = data_.at(k);
        return val;
    }

private:
    void parseInput(const std::string& in_msg)
    {
        std::cout << "Parsing incoming message..." << std::endl;
        std::string raw = in_msg;

        if (raw.find("{") == std::string::npos)
        {
            std::cout << "Incoming message is not valid: " << raw << std::endl;
            return;
        }

        // Remove first and last brackets.
        raw.erase(0, 1);
        raw.erase(raw.length() - 1, raw.length());

        // Get the type.
        type_ = fetch(raw, "type");
        parseData(fetch(raw, "data"));
    }

    void parseData(const std::string& in_data)
    {
        // std::string raw = in_data;
        auto vals = split(in_data, ',');
        for (auto& str : vals)
        {
            auto key_val = split(str, ':');
            if (key_val.size() != 2)
            {
                std::cout << "Error! " << str << std::endl;
                continue;
            }

            std::string key = stripQuotes(key_val[0]);
            std::string val = stripQuotes(key_val[1]);

            data_.insert({key, val});
        }

        std::cout << "Data:" << std::endl;
        for (auto const& x : data_)
        {
            std::cout << "\t" << x.first << ": " << x.second << std::endl;
        }
        std::cout << std::endl;
    }

    std::map<std::string, std::string> data_;
    std::string type_;
};

}  // namespace server_utils

#endif  // AUTONOMOUS_NAVIGATION_UTILS_SERVER_UTILS_H
