/*
 * Copyright (c) 2009-2012, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_server_minimal");
  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle private_nh = ros::NodeHandle("~");
  std::string map_filename;
  std::string frame_id = "map";

  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h"))
  {
    ROS_ERROR("\nusage:\n\noctomap_server_minimal_node map.bt\n\n"
        "  map.bt: octomap 3D map file to read, minimal only handles binary maps\n");
    exit(-1);
  }

  if (argc == 2)
  {
    map_filename = std::string(argv[1]);
  }

  {
    std::string map_filename_param;
    if (private_nh.getParam("map_file", map_filename_param))
    {
      if (!map_filename.empty())
      {
        ROS_WARN_STREAM(
            "map_file is specified by the ROS parameter '" << map_filename_param <<
            "' but is overriden by command line argument '" << map_filename << "'");
      }
      else
      {
        map_filename = map_filename_param;
      }
    }
  }

  private_nh.param("frame_id", frame_id, frame_id);

  if (map_filename.empty())
  {
    ROS_ERROR("Minimal node must be given a file to serve!");
    exit(1);
  }

  ros::Publisher binary_map_publisher =
      nh.advertise<octomap_msgs::Octomap>("octomap_binary", 1, true);

  // The rest of the variables are temporary, but them in their own scope
  {
    std::string suffix = map_filename.substr(map_filename.length()-3, 3);
    if (suffix != ".bt")
    {
      ROS_ERROR("Minimal node must be given a .bt file!");
      exit(1);
    }
    std::ifstream infile(map_filename);
    if (!infile)
    {
      ROS_ERROR_STREAM("Unable to open file " << map_filename);
      exit(1);
    }

    size_t node_count = 0;
    octomap_msgs::Octomap msg;
    msg.binary = true;
    msg.header.frame_id = frame_id;
    msg.depth = 16;
    msg.resolution = 0.0;
    msg.id = "";

    std::string line;
    while (std::getline(infile, line))
    {
      std::istringstream ss(line);
      std::string first_word;
      ss >> first_word;
      if (first_word == "id")
      {
        ss >> msg.id;
        ROS_INFO_STREAM("id " << msg.id);
      }
      else if (first_word == "res")
      {
        ss >> msg.resolution;
        ROS_INFO_STREAM("resolution " << msg.resolution);
      }
      else if (first_word == "size")
      {
        ss >> node_count;
        ROS_INFO_STREAM("node count: " << node_count);
      }
      else if (first_word == "depth")
      {
        ss >> msg.depth;
        ROS_INFO_STREAM("tree depth: " << msg.depth);
      }
      else if (first_word == "data")
      {
        std::stringstream data_ss;
        infile >> data_ss.rdbuf();
        std::string msg_data = data_ss.str();
        msg.data.assign(msg_data.begin(), msg_data.end());
      }
    }

    if (msg.resolution == 0.0)
    {
      ROS_ERROR_STREAM("Unable to parse " << map_filename
                       << " could not read resolution");
      exit(1);
    }
    if (msg.id.empty())
    {
      ROS_ERROR_STREAM("Unable to parse " << map_filename
                       << " could not read id");
      exit(1);
    }
    if (msg.data.empty() && node_count > 0)
    {
      ROS_ERROR_STREAM("Unable to parse " << map_filename
                       << " data size mismatch. Header says there are "
                       << node_count << " nodes but actually read "
                       << msg.data.size() << " bytes.");
      exit(1);
    }
    // The message will stay latched in the publisher, so there is no need to
    // save a copy of it here.
    binary_map_publisher.publish(msg);
  }

  ROS_INFO_STREAM("Publishing map from " << map_filename
                  << " with frame_id: " << frame_id);
  ros::spin();

  return 0;
}
