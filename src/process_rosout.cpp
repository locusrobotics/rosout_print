/**
 *  \file       process_rosout.cpp
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
#include "rosout_print/process_rosout.h"

#include <ros/message_traits.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Log.h>
#include <stdint.h>
#include <topic_tools/shape_shifter.h>

#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rosout_print/thread_safe_queue.h"

namespace rosout_print_process_rosout
{
std::shared_ptr<std::regex> getMsgNameRegex(const std::string &source_node_regex)
{
    std::shared_ptr<std::regex> regex;
    if (!source_node_regex.empty())
    {
        regex.reset(new std::regex(source_node_regex));
    }
    return regex;
}

std::streambuf *getOutputStreamBuf(const std::string &output_file_name,
                                   std::ofstream &ofs)
{
    std::streambuf *buf;
    if (output_file_name.empty())
    {
        buf = std::cout.rdbuf();
    }
    else
    {
        ofs.open(output_file_name.c_str());
        buf = ofs.rdbuf();
    }
    return buf;
}

std::string byteToLevelStr(uint8_t level)
{
    switch (level)
    {
    case rosgraph_msgs::Log::DEBUG:
        return "DEBUG";
    case rosgraph_msgs::Log::INFO:
        return "INFO ";
    case rosgraph_msgs::Log::WARN:
        return "WARN ";
    case rosgraph_msgs::Log::ERROR:
        return "ERROR";
    case rosgraph_msgs::Log::FATAL:
        return "FATAL";
    default:
        std::stringstream ss;
        ss << "Unexpected level value: " << int(level);
        throw std::runtime_error(ss.str());
    }
    return "";
}

std::string rosTimeToStr(const ros::Time ts, struct tm *(*time_t_to_tm)(const time_t *))
{
    time_t time = ts.sec;
    struct tm ts_in_tm = *time_t_to_tm(&time);
    char buf[32], buf_tz[4];
    strftime(buf, 32, "%Y-%m-%d %H:%M:%S", &ts_in_tm);
    strftime(buf_tz, 4, "%Z", &ts_in_tm);
    std::ostringstream os;
    os << buf << "." << std::setfill('0') << std::setw(3) << ts.nsec / 1000000 << " "
       << buf_tz;
    return os.str();
}

std::string removeLineBreak(std::string str)
{
    std::regex_replace(str, std::regex("\n"), "\\n");
    return str;
}

struct ThreadArg
{
    ThreadArg(const uint8_t min_level,
              const std::shared_ptr<std::regex> msg_name_regex,
              const bool utc_time,
              const bool epoch_time,
              const bool use_curr_time_as_msg_time,
              const bool multiple_lines,
              const bool function_detail,
              std::map<ColorCode, std::string> &color_table,
              std::ostream &os)
      : min_level_(min_level)
      , msg_name_regex_(msg_name_regex)
      , utc_time_(utc_time)
      , epoch_time_(epoch_time)
      , use_curr_time_as_msg_time_(use_curr_time_as_msg_time)
      , multiple_lines_(multiple_lines)
      , function_detail_(function_detail)
      , color_table_(color_table)
      , os_(os)
    {
    }

public:
    const uint8_t min_level_;
    const std::shared_ptr<std::regex> msg_name_regex_;
    const bool utc_time_;
    const bool epoch_time_;
    const bool use_curr_time_as_msg_time_;
    const bool multiple_lines_;
    const bool function_detail_;
    std::map<ColorCode, std::string> &color_table_;
    std::ostream &os_;
};

bool isMsgToBeOutput(struct ThreadArg &arg, const rosgraph_msgs::Log::ConstPtr &msg_ptr)
{
    if (!msg_ptr)
        return false;
    if (msg_ptr->level < arg.min_level_)
        return false;
    if (arg.msg_name_regex_ && !std::regex_match(msg_ptr->name, *arg.msg_name_regex_))
        return false;
    return true;
}

bool g_running;
using TimeStampedMsg = std::pair<ros::Time, rosgraph_msgs::Log::ConstPtr>;
ThreadSafeQueue<TimeStampedMsg> live_msg_queue;

void enqueMsg(struct ThreadArg &arg, const rosgraph_msgs::Log::ConstPtr &msg_ptr)
{
    if (!isMsgToBeOutput(arg, msg_ptr))
        return;

    live_msg_queue.push(std::make_pair(ros::Time::now(), msg_ptr));
}

std::ostream &insertTimeStampToOutputStream(struct ThreadArg &arg,
                                            const ros::Time &msg_time,
                                            const rosgraph_msgs::Log::ConstPtr &msg_ptr)
{
    const ros::Time ts = arg.use_curr_time_as_msg_time_ ? ros::Time::now() : msg_time;
    arg.os_ << "[";
    if (arg.epoch_time_)
    {
        arg.os_ << arg.color_table_[static_cast<ColorCode>(msg_ptr->level)] << ts;
    }
    else if (arg.utc_time_)
    {
        arg.os_ << arg.color_table_[static_cast<ColorCode>(msg_ptr->level)]
                << rosTimeToStr(ts, gmtime);
    }
    else
    {
        arg.os_ << arg.color_table_[static_cast<ColorCode>(msg_ptr->level)]
                << rosTimeToStr(ts, localtime);
    }
    arg.os_ << arg.color_table_[ColorCode::RESET] << "] ";
    return arg.os_;
}

std::ostream &insertLevelToOutputStream(struct ThreadArg &arg,
                                        const rosgraph_msgs::Log::ConstPtr &msg_ptr)
{
    arg.os_ << "[" << arg.color_table_[static_cast<ColorCode>(msg_ptr->level)]
            << byteToLevelStr(msg_ptr->level) << arg.color_table_[ColorCode::RESET]
            << "] ";
    return arg.os_;
}

std::ostream &insertNameAndMsgToOutputStream(struct ThreadArg &arg,
                                             const rosgraph_msgs::Log::ConstPtr &msg_ptr)
{
    const std::string &node_name =
        (arg.multiple_lines_) ? msg_ptr->name : removeLineBreak(msg_ptr->name);
    const std::string &log_line =
        (arg.multiple_lines_) ? msg_ptr->msg : removeLineBreak(msg_ptr->msg);
    arg.os_ << "[" << arg.color_table_[static_cast<ColorCode>(msg_ptr->level)]
            << node_name << arg.color_table_[ColorCode::RESET]
            << "]: " << arg.color_table_[static_cast<ColorCode>(msg_ptr->level)]
            << log_line << arg.color_table_[ColorCode::RESET];
    return arg.os_;
}

std::ostream &
insertFunctionDetailToOutputStream(struct ThreadArg &arg,
                                   const rosgraph_msgs::Log::ConstPtr &msg_ptr)
{
    arg.os_ << " [" << arg.color_table_[ColorCode::BLUE] << msg_ptr->file
            << arg.color_table_[ColorCode::RESET] << ":"
            << arg.color_table_[ColorCode::BLUE] << msg_ptr->line
            << arg.color_table_[ColorCode::RESET] << "("
            << arg.color_table_[ColorCode::CYAN] << msg_ptr->function
            << arg.color_table_[ColorCode::RESET] << ")]";
    return arg.os_;
}

std::ostream &insertColorResetCodeToOutputStream(struct ThreadArg &arg)
{
    arg.os_ << arg.color_table_[ColorCode::RESET] << std::endl;
    return arg.os_;
}

void insertMsgToOutputStream(struct ThreadArg &arg,
                             const ros::Time &msg_time,
                             const rosgraph_msgs::Log::ConstPtr &msg_ptr)
{
    if (!msg_ptr)
        return;
    insertTimeStampToOutputStream(arg, msg_time, msg_ptr);
    insertLevelToOutputStream(arg, msg_ptr);
    insertNameAndMsgToOutputStream(arg, msg_ptr);
    if (arg.function_detail_)
        insertFunctionDetailToOutputStream(arg, msg_ptr);
    insertColorResetCodeToOutputStream(arg);
}

void dequeMsg(struct ThreadArg &arg)
{
    while (g_running)
    {
        TimeStampedMsg msg;
        live_msg_queue.waitAndPop(msg);
        insertMsgToOutputStream(arg, msg.first, msg.second);
    }
}

void callback(const ros::TimerEvent &event,
              const std::string source_node_regex,
              int print_dur)
{
    (void)event;
    std::vector<std::string> active_nodes;
    bool nodes_exist = false;
    ros::master::getNodes(active_nodes);
    for (auto s : active_nodes)
    {
        if (std::regex_match(s, std::regex(source_node_regex)))
        {
            nodes_exist = true;
            break;
        }
    }
    if (nodes_exist == false)
    {
        std::cout << "No specified nodes matching regex \"" << source_node_regex
                  << "\" found in the past " << print_dur << " seconds." << std::endl;
    }
}

int processLiveRosoutAggMsgs(int argc,
                             char **argv,
                             const uint8_t min_level,
                             const std::string &output_file_name,
                             const std::string &source_topic_name,
                             const std::string &source_node_regex,
                             const bool utc_time,
                             const bool epoch_time,
                             const bool multiple_lines,
                             const bool function_detail,
                             std::map<ColorCode, std::string> &color_table)
{
    // Setup regex for node names.
    auto msg_name_regex = getMsgNameRegex(source_node_regex);
    // Prepare output stream.
    std::ofstream ofs;
    std::ostream os(getOutputStreamBuf(output_file_name, ofs));
    // Interval for printing out message in case nodes are not found
    int print_dur = 10;
    // Subscribe to rosout messages to enqueue them.
    struct ThreadArg thread_arg(min_level, msg_name_regex, utc_time, epoch_time, true,
                                multiple_lines, function_detail, color_table, os);
    ros::init(argc, argv, "rosout_print", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Timer timer;
    if (source_node_regex != "")
    {
        timer = nh.createTimer(
            ros::Duration(print_dur),
            std::bind(callback, std::placeholders::_1, source_node_regex, print_dur));
    }
    ros::Subscriber sub = nh.subscribe<rosgraph_msgs::Log>(
        source_topic_name, 128, std::bind(enqueMsg, thread_arg, std::placeholders::_1));

    // Start thread to insert queued messags to output stream.
    g_running = true;
    std::thread deque_msg_thread(dequeMsg, std::ref(thread_arg));

    // Signal the thread to terminate.
    ros::spin();
    g_running = false;
    live_msg_queue.push(std::make_pair(ros::Time::now(), nullptr));
    deque_msg_thread.join();

    return 0;
}

int processRosoutAggMsgsInBag(const uint8_t min_level,
                              const std::string &bag_file_name,
                              const std::string &output_file_name,
                              const std::string &source_topic_name,
                              const std::string &source_node_regex,
                              const bool utc_time,
                              const bool epoch_time,
                              const bool multiple_lines,
                              const bool function_detail,
                              std::map<ColorCode, std::string> &color_table)
{
    // Setup regex for node names.
    auto msg_name_regex = getMsgNameRegex(source_node_regex);
    // Prepare output stream.
    std::ofstream ofs;
    std::ostream os(getOutputStreamBuf(output_file_name, ofs));

    // Open the bag.
    rosbag::Bag bag;
    try
    {
        bag.open(bag_file_name, rosbag::bagmode::Read);
    }
    catch (const rosbag::BagException &e)
    {
        std::cout << "Caught a BagException while opening " << bag_file_name << ": "
                  << e.what() << std::endl;
        return 0;
    }
    catch (...)
    {
        std::cout << "Caught an exception while opening " << bag_file_name << "."
                  << std::endl;
        return 0;
    }

    // Insert messages to output stream.
    rosbag::View view(bag);
    struct ThreadArg thread_arg(min_level, msg_name_regex, utc_time, epoch_time, false,
                                multiple_lines, function_detail, color_table, os);
    for (rosbag::MessageInstance const m : view)
    {
        if (m.getTopic() != source_topic_name)
            continue;
        try
        {
            const rosgraph_msgs::Log::ConstPtr msg_ptr =
                m.instantiate<rosgraph_msgs::Log>();
            if (!isMsgToBeOutput(thread_arg, msg_ptr))
                continue;
            insertMsgToOutputStream(thread_arg, m.getTime(), msg_ptr);
        }
        catch (const ros::serialization::StreamOverrunException &ex)
        {
            ROS_ERROR_STREAM("A " << source_topic_name
                                  << " message is not of rosgraph_msgs::Log type");
            return 1;
        }
    }
    bag.close();
    return 0;
}

}  // namespace rosout_print_process_rosout
