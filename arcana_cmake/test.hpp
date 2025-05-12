#ifndef ARCANA_ROSD_HPP
#define ARCANA_ROSD_HPP
// ============================================================================
//                       ROS2 installation utils methods                       
// Auto-generated file that contains the ROS distro mappings
// 
// Copyright (c) Meltwin 2024 - 2025
// Author: Geoffrey Côte
// Part of the ros_arcana package
// 
// The MIT License (MIT)  https://mit-license.org/
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the “Software”), to de
// al in the Software without restriction, including without limitation the rig
// hts to use, copy, modify, merge, publish, distribute, sublicense, and/or sel
// l copies of the Software, and to permit persons to whom the Software is furn
// ished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in a
// ll copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR I
// MPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITN
// ESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTH
// ORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY
// , WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
// OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTW
// ARE.
// ============================================================================


#define ROSD_UNKNOWN 0
#define ROSD_ARDENT 2000
#define ROSD_BOUNCY 2001
#define ROSD_CRYSTAL 2002
#define ROSD_DASHING 2003
#define ROSD_ELOQUENT 2004
#define ROSD_FOXY 2005
#define ROSD_GALACTIC 2006
#define ROSD_HUMBLE 2007
#define ROSD_IRON 2008
#define ROSD_JAZZY 2009


/**
 * Return a stringified version of the ROS distro code.
*/
constexpr const char* ROSD2STR(const int v) {
  switch (v) {
    case ROSD_ARDENT:
      return "ARDENT";
    case ROSD_BOUNCY:
      return "BOUNCY";
    case ROSD_CRYSTAL:
      return "CRYSTAL";
    case ROSD_DASHING:
      return "DASHING";
    case ROSD_ELOQUENT:
      return "ELOQUENT";
    case ROSD_FOXY:
      return "FOXY";
    case ROSD_GALACTIC:
      return "GALACTIC";
    case ROSD_HUMBLE:
      return "HUMBLE";
    case ROSD_IRON:
      return "IRON";
    case ROSD_JAZZY:
      return "JAZZY";
    default:
      return "UNKNOWN";
  }
}
#endif
