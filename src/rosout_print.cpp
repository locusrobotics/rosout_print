/**
 *  \file       rosout_print.cpp
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
#include <rosgraph_msgs/Log.h>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <iostream>
#include <map>
#include <string>

#include "rosout_print/process_rosout.h"

#define COLOR_RESET "\033[0m"
#define BOLD_TEXT "\033[1m"
#define BLACK_TEXT "\033[30;1m"
#define RED_TEXT "\033[31;1m"
#define GREEN_TEXT "\033[32;1m"
#define YELLOW_TEXT "\033[33;1m"
#define BLUE_TEXT "\033[34;1m"
#define MAGENTA_TEXT "\033[35;1m"
#define CYAN_TEXT "\033[36;1m"
#define WHITE_TEXT "\033[37;1m"

using rosout_print_process_rosout::ColorCode;

int main(int argc, char *argv[])
{
    std::string bag_file_name;
    int level = 1;
    std::string output_file_name;
    std::string source_topic_name;
    std::string source_node_regex;
    bool utc_time = false;
    bool epoch_time = false;
    bool multiple_lines = false;
    bool function_detail = false;
    bool without_color = false;

    namespace po = boost::program_options;
    po::options_description desc("Convert rosout_agg messages to text forms");
    desc.add_options()("help,h", "Produce help message.")(
        "bag-file,b", po::value<std::string>(&bag_file_name),
        "Bag file path name (if not specified, live ros "
        "messages are converted)")("level,l", po::value<int>(&level)->default_value(1),
                                   "Minimum level to be output (DEBUG=0, INFO=1, WARN=2, "
                                   "ERROR=3, FATAL=4)")(
        "output-file,o", po::value<std::string>(&output_file_name), "Output file name")(
        "topic,t",
        po::value<std::string>(&source_topic_name)->default_value("/rosout_agg"),
        "Topic where rosgraph_msgs::Log messages are published")(
        "node-regex,n", po::value<std::string>(&source_node_regex)->default_value(""),
        "Regex matching the names of nodes whose rosgraph_msgs::Log messages are to be "
        "processed")("utc-time,u", po::bool_switch(&utc_time), "Output time in UTC")(
        "epoch-time,e", po::bool_switch(&epoch_time), "Output time in Unix epoch")(
        "multiple-lines,m", po::bool_switch(&multiple_lines),
        "Line breaks in messages")("function-detail,f", po::bool_switch(&function_detail),
                                   "Output function name, file, and line number")(
        "no-color", po::bool_switch(&without_color), "do not colorize the output");
    po::variables_map vm;
    try
    {
        store(po::command_line_parser(argc, argv).options(desc).run(), vm);
        notify(vm);
    }
    catch (std::exception &e)
    {
        std::cout << std::endl << e.what() << std::endl;
        std::cout << desc << std::endl;
        return 0;
    }
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 0;
    }
    uint8_t level_in_power_of_two = (1 << level);
    if (level_in_power_of_two < rosgraph_msgs::Log::DEBUG ||
        level_in_power_of_two > rosgraph_msgs::Log::FATAL)
    {
        std::cout << "Invalid level value: " << level << std::endl;
        return 0;
    }

    std::map<ColorCode, std::string> color_table;
    if (!without_color)
    {
        color_table[static_cast<ColorCode>(rosgraph_msgs::Log::DEBUG)] =
            BOLD_TEXT GREEN_TEXT;
        color_table[static_cast<ColorCode>(rosgraph_msgs::Log::INFO)] = WHITE_TEXT;
        color_table[static_cast<ColorCode>(rosgraph_msgs::Log::WARN)] = YELLOW_TEXT;
        color_table[static_cast<ColorCode>(rosgraph_msgs::Log::ERROR)] = RED_TEXT;
        color_table[static_cast<ColorCode>(rosgraph_msgs::Log::FATAL)] =
            BOLD_TEXT RED_TEXT;
        color_table[ColorCode::RESET] = COLOR_RESET;
        color_table[ColorCode::BOLD] = BOLD_TEXT;
        color_table[ColorCode::BLACK] = BLACK_TEXT;
        color_table[ColorCode::RED] = RED_TEXT;
        color_table[ColorCode::GREEN] = GREEN_TEXT;
        color_table[ColorCode::YELLOW] = YELLOW_TEXT;
        color_table[ColorCode::BLUE] = BLUE_TEXT;
        color_table[ColorCode::MAGENTA] = MAGENTA_TEXT;
        color_table[ColorCode::CYAN] = CYAN_TEXT;
        color_table[ColorCode::WHITE] = WHITE_TEXT;
    }

    if (bag_file_name.empty())
    {
        return rosout_print_process_rosout::processLiveRosoutAggMsgs(
            argc, argv, level_in_power_of_two, output_file_name, source_topic_name,
            source_node_regex, utc_time, epoch_time, multiple_lines, function_detail,
            color_table);
    }
    else
    {
        return rosout_print_process_rosout::processRosoutAggMsgsInBag(
            level_in_power_of_two, bag_file_name, output_file_name, source_topic_name,
            source_node_regex, utc_time, epoch_time, multiple_lines, function_detail,
            color_table);
    }
}
