/**
 *  \file       process_rosout.h
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
#ifndef ROSOUT_PRINT_PROCESS_ROSOUT_H
#define ROSOUT_PRINT_PROCESS_ROSOUT_H

#include <cstdint>
#include <map>
#include <string>

namespace rosout_print_process_rosout
{
enum class ColorCode
{
    RESET = 10,
    BOLD,
    BLACK,
    RED,
    GREEN,
    YELLOW,
    BLUE,
    MAGENTA,
    CYAN,
    WHITE,
};

int processLiveRosoutAggMsgs(int,
                             char **,
                             const uint8_t,
                             const std::string &,
                             const std::string &,
                             const std::string &,
                             const bool,
                             const bool,
                             const bool,
                             const bool,
                             std::map<ColorCode, std::string> &);
int processRosoutAggMsgsInBag(const uint8_t,
                              const std::string &,
                              const std::string &,
                              const std::string &,
                              const std::string &,
                              const bool,
                              const bool,
                              const bool,
                              const bool,
                              std::map<ColorCode, std::string> &);

}  // namespace rosout_print_process_rosout

#endif  // ROSOUT_PRINT_PROCESS_ROSOUT_H
