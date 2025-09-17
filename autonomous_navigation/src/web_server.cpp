#include <iostream>
#include <string>
#include <memory>
#include <future>
#include <deque>
#include <mutex>

#include <simple-ws-server/server_ws.hpp>

#include <autonomous_navigation/utils/server_utils.h>
#include <autonomous_navigation/utils/math_helpers.h>
#include <autonomous_navigation/utils/graph_utils.h>
#include <autonomous_navigation/graph_search/graph_search.h>
#include <autonomous_navigation/potential_field/distance_transform.h>
#include <autonomous_navigation/potential_field/potential_field.h>
#include <autonomous_navigation/potential_field/local_search.h>

#define DT 0.01
#define LOG_HEADER "[SERVER] "

using WsServer = SimpleWeb::SocketServer<SimpleWeb::WS>;

class PathPlanServer
{
public:
    PathPlanServer(WsServer& server) :
        connected_(false)
    {
        // Init web socket.
        auto &omni_socket = server.endpoint["^/mb/?$"];

        omni_socket.on_message = [&](std::shared_ptr<WsServer::Connection> connection,
                                     std::shared_ptr<WsServer::InMessage> in_message) {
            auto string_msg = in_message->string();
            std::cout << LOG_HEADER << "Message received: \"" << string_msg << "\"";
            std::cout << " from " << connection.get() << std::endl;
            onMessage(connection, string_msg);
        };

        // Setup some basic functions.
        omni_socket.on_open = [&](std::shared_ptr<WsServer::Connection> connection) {
            std::cout << LOG_HEADER << "Opened connection " << connection.get() << std::endl;
            setConnection(connection);
        };

        // See RFC 6455 7.4.1. for status codes
        omni_socket.on_close = [&](std::shared_ptr<WsServer::Connection> connection,
                                   int status, const std::string & /*reason*/) {
            std::cout << LOG_HEADER << "Closed connection " << connection.get();
            std::cout << " with status code " << status << std::endl;
            resetConnection();
        };

        // Can modify handshake response headers here if needed
        omni_socket.on_handshake = [](std::shared_ptr<WsServer::Connection> /*connection*/,
                                      SimpleWeb::CaseInsensitiveMultimap & /*response_header*/) {
            return SimpleWeb::StatusCode::information_switching_protocols;  // Upgrade to websocket
        };

        // See http://www.boost.org/doc/libs/1_55_0/doc/html/boost_asio/reference.html for error code meanings
        omni_socket.on_error = [](std::shared_ptr<WsServer::Connection> connection, const SimpleWeb::error_code &ec) {
            std::cout << LOG_HEADER << "Error in connection " << connection.get() << ". "
                 << "Error: " << ec << ", error message: " << ec.message() << std::endl;
        };
    };

    void setConnection(std::shared_ptr<WsServer::Connection> connection)
    {
      connection_ = connection;
      connected_ = true;
    }

    void resetConnection()
    {
      connection_.reset();
      connected_ = false;
    }

    void onMessage(std::shared_ptr<WsServer::Connection> connection, const std::string& msg)
    {
        server_utils::InMessageHelper in_msg(msg);
        std::lock_guard<std::mutex> lock(msg_queue_mtx_);
        msg_queue_.push_back(in_msg);
    }

    void loop()
    {
        std::unique_lock<std::mutex> lock(msg_queue_mtx_);
        if (msg_queue_.size() > 0)
        {
            auto current_msg = msg_queue_.front();
            msg_queue_.pop_front();
            lock.unlock();

            if (current_msg.type() == "plan")
            {
                findPath(server_utils::PlanData(current_msg.getData()));
            }
            else if (current_msg.type() == "map_file")
            {
                loadGraph("../data/" + current_msg.getVal("file_name"));
            }
            else
            {
                std::cout << LOG_HEADER << "Unrecognized type " << current_msg.type() << std::endl;
            }
        }
    }

private:
    void loadGraph(const std::string& file_path)
    {
        std::cout << LOG_HEADER << "Loading map from: " << file_path << std::endl;
        loadFromFile(file_path, graph_);
        graph_.collision_radius = 0.137 + graph_.meters_per_cell;

        distanceTransform(graph_);

        std::cout << LOG_HEADER << "Sending obstacle distances." << std::endl;
        connection_->send(server_utils::fieldToMsg(graph_.obstacle_distances));
    }

    void findPath(const server_utils::PlanData& data)
    {
        // If the graph hasn't been loaded, load it.
        if (!isLoaded(graph_))
        {
            loadGraph("../data/" + data.mapName());
        }

        std::cout << LOG_HEADER << "Planning with algorithm: " << data.algo() << std::endl;

        std::vector<Cell> path;  // Final path will be stored here.
        // This lambda function lets the graph search functions show visited cells.
        auto showVisited = [&](int i, int j) { server_utils::sendVisitedCell(connection_, i, j); };

        if (data.algo() == "dfs")
        {
            path = depthFirstSearch(graph_, data.start(), data.goal(), showVisited);
        }
        else if (data.algo() == "bfs")
        {
            path = breadthFirstSearch(graph_, data.start(), data.goal(), showVisited);
        }
        else if (data.algo() == "astar")
        {
            path = aStarSearch(graph_, data.start(), data.goal(), showVisited);
        }
        else if (data.algo() == "pfield")
        {
            auto field = createPotentialField(graph_, data.goal());
            connection_->send(server_utils::fieldToMsg(field));

            path = localSearchFull(data.start(), graph_, field);
        }
        else
        {
            std::cout << LOG_HEADER << "Algo " << data.algo() << " not supported." << std::endl;
        }

        if (path.size() > 1)
        {
            std::cout << LOG_HEADER << "Success! Found path with " << path.size() << " cells." << std::endl;

            if (connection_)
            {
                connection_->send(server_utils::pathToMsg(path));
            }
        }
        else
        {
            std::cout << LOG_HEADER << "No path found :(" << std::endl;
        }
    }

    std::shared_ptr<WsServer::Connection> connection_;
    bool connected_;

    std::mutex msg_queue_mtx_;
    std::deque<server_utils::InMessageHelper> msg_queue_;

    GridGraph graph_;
};


int main(int argc, char const *argv[])
{
    // WebSocket (WS)-server at port 8080 using 1 thread
    WsServer server;
    server.config.port = 8080;
    std::shared_ptr<PathPlanServer> helper = std::make_shared<PathPlanServer>(server);

    // Start server and receive assigned port when server is listening for requests
    std::promise<unsigned short> server_port;
    std::thread server_thread([&server, &server_port]() {
        // Start server
        server.start([&server_port](unsigned short port) {
            server_port.set_value(port);
        });
    });

    std::cout << LOG_HEADER << "Server listening on port " << server_port.get_future().get() << std::endl << std::endl;

    while (true)
    {
        helper->loop();
        sleepFor(DT);
    }

    server_thread.join();
}
