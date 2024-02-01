/**
 *  \file       test_rosout_print.cpp
 *  \author     Jim Won <jwon@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2017, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 */
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Log.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <map>
#include <string>

#include "rosout_print/process_rosout.h"

using rosout_print_process_rosout::ColorCode;

const char kSourceTopicName[] = "/rosout_agg";

std::string createBagForTesting()
{
    rosbag::Bag bag;
    std::string bag_file_name("/tmp/test_rosout_print.bag");
    bag.open(bag_file_name, rosbag::bagmode::Write);
    rosgraph_msgs::Log msg;
    msg.level = rosgraph_msgs::Log::INFO;
    msg.name = "Name info";
    msg.msg = "Msg info";
    msg.file = "file_info";
    msg.function = "function_info";
    msg.line = 1;
    bag.write(kSourceTopicName, ros::TIME_MIN, msg);
    msg.level = rosgraph_msgs::Log::ERROR;
    msg.name = "Name error";
    msg.msg = "Msg error";
    msg.file = "file_error";
    msg.function = "function_error";
    bag.write(kSourceTopicName, ros::TIME_MAX, msg);
    bag.close();
    return bag_file_name;
}

class RosoutPrintTest : public ::testing::Test
{
public:
    void TearDown()
    {
        // Kill gpg-agent if running
        if (!system("pgrep gpg-agent > /dev/null"))
        {
            EXPECT_GE(system("kill -9 $(pidof gpg-agent)"), 0);
        }
    }
};

TEST_F(RosoutPrintTest, HandlesBagFileInput)
{
    std::map<ColorCode, std::string> color_table;
    // Create a bag file for testing.
    std::string bag_file_name = createBagForTesting();
    std::string output_file_name = "/tmp/test_rosout_print.log";
    // Test with level of 1 (INFO).
    int ret_val = rosout_print_process_rosout::processRosoutAggMsgsInBag(
        1, bag_file_name, output_file_name, kSourceTopicName,
        "",            // source_node_regex
        true,          // utc_time
        false,         // epoch_time
        false,         // multiple_lines
        false,         // function_detail
        color_table);  // Color table
    EXPECT_EQ(0, ret_val);
    std::string line;
    std::ifstream output_file(output_file_name.c_str());
    if (output_file.is_open())
    {
        getline(output_file, line);
        EXPECT_EQ(line, "[1970-01-01 00:00:00.000 GMT] [INFO ] [Name info]: Msg info");
        getline(output_file, line);
        EXPECT_EQ(line, "[2106-02-07 06:28:15.999 GMT] [ERROR] [Name error]: Msg error");
    }
    boost::filesystem::remove(bag_file_name);
    boost::filesystem::remove(output_file_name);
}

void msgPublisher(ros::NodeHandle& nh)
{
    ros::Publisher pub =
        ros::NodeHandle(nh).advertise<rosgraph_msgs::Log>(kSourceTopicName, 1, true);
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    EXPECT_TRUE(pub);
    rosgraph_msgs::Log msg;
    msg.level = rosgraph_msgs::Log::INFO;
    msg.name = "Name info";
    msg.msg = "Msg info";
    msg.file = "file_info";
    msg.function = "function_info";
    msg.line = 1;
    pub.publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    msg.level = rosgraph_msgs::Log::ERROR;
    msg.name = "Name error";
    msg.msg = "Msg error";
    msg.file = "file_error";
    msg.function = "function_error";
    pub.publish(msg);
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    ros::shutdown();
}

inline bool ends_with(const std::string& value, const std::string& ending)
{
    if (ending.size() > value.size())
        return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

TEST_F(RosoutPrintTest, HandlesLiveMsgInput)
{
    std::map<ColorCode, std::string> color_table;
    std::string output_file_name = "/tmp/test_rosout_print2.log";
    ros::NodeHandle nh;
    boost::thread msg_publisher_thread = boost::thread(msgPublisher, nh);
    // Test with level of sqrt(4) (WARN).
    int ret_val = rosout_print_process_rosout::processLiveRosoutAggMsgs(
        0, 0, 4, output_file_name, kSourceTopicName,
        "",     // source_node_regex
        true,   // utc_time
        false,  // epoch_time
        false,  // multiple_lines
        false,  // function_detail
        color_table);
    msg_publisher_thread.join();
    EXPECT_EQ(0, ret_val);
    std::string line;
    std::ifstream output_file(output_file_name.c_str());
    if (output_file.is_open())
    {
        getline(output_file, line);
        EXPECT_TRUE(ends_with(line, " GMT] [ERROR] [Name error]: Msg error"))
            << "Actual line text is " << line;
    }
    boost::filesystem::remove(output_file_name);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_rosout_print");
    return RUN_ALL_TESTS();
}
