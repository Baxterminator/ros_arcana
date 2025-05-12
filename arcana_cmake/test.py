# =============================================================================
#                        ROS2 installation utils methods                       
# Auto-generated file that contains the ROS distro mappings
# 
# Copyright (c) Meltwin 2024 - 2025
# Author: Geoffrey Côte
# Part of the ros_arcana package
# 
# The MIT License (MIT)  https://mit-license.org/
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the “Software”), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell co
# pies of the Software, and to permit persons to whom the Software is furnished
# to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in al
# l copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IM
# PLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNES
# S FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
# OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WH
# ETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
# =============================================================================

from enum import IntEnum

class ROSD:
	class Version(IntEnum):
		UNKNOWN = 0
		ARDENT = 2000
		BOUNCY = 2001
		CRYSTAL = 2002
		DASHING = 2003
		ELOQUENT = 2004
		FOXY = 2005
		GALACTIC = 2006
		HUMBLE = 2007
		IRON = 2008
		JAZZY = 2009
	class Name:
		UNKNOWN = "UNKNOWN"
		ARDENT = "ardent"
		BOUNCY = "bouncy"
		CRYSTAL = "crystal"
		DASHING = "dashing"
		ELOQUENT = "eloquent"
		FOXY = "foxy"
		GALACTIC = "galactic"
		HUMBLE = "humble"
		IRON = "iron"
		JAZZY = "jazzy"

	@staticmethod
	def from_string(n: str) -> "ROSD.Version":
		"""
		Return the equivalent version of the ROS distro
		"""
		match n:
			case ROSD.Name.ARDENT:
				return ROSD.Version.ARDENT
			case ROSD.Name.BOUNCY:
				return ROSD.Version.BOUNCY
			case ROSD.Name.CRYSTAL:
				return ROSD.Version.CRYSTAL
			case ROSD.Name.DASHING:
				return ROSD.Version.DASHING
			case ROSD.Name.ELOQUENT:
				return ROSD.Version.ELOQUENT
			case ROSD.Name.FOXY:
				return ROSD.Version.FOXY
			case ROSD.Name.GALACTIC:
				return ROSD.Version.GALACTIC
			case ROSD.Name.HUMBLE:
				return ROSD.Version.HUMBLE
			case ROSD.Name.IRON:
				return ROSD.Version.IRON
			case ROSD.Name.JAZZY:
				return ROSD.Version.JAZZY
			case _:
				return ROSD.Version.UNKNOWN

	@staticmethod
	def to_string(v: "ROSD.Version") -> str:
		"""
		Return a stringified version of the ROS distro code.
		"""
		match v:
			case ROSD.Version.ARDENT:
				return ROSD.Name.ARDENT
			case ROSD.Version.BOUNCY:
				return ROSD.Name.BOUNCY
			case ROSD.Version.CRYSTAL:
				return ROSD.Name.CRYSTAL
			case ROSD.Version.DASHING:
				return ROSD.Name.DASHING
			case ROSD.Version.ELOQUENT:
				return ROSD.Name.ELOQUENT
			case ROSD.Version.FOXY:
				return ROSD.Name.FOXY
			case ROSD.Version.GALACTIC:
				return ROSD.Name.GALACTIC
			case ROSD.Version.HUMBLE:
				return ROSD.Name.HUMBLE
			case ROSD.Version.IRON:
				return ROSD.Name.IRON
			case ROSD.Version.JAZZY:
				return ROSD.Name.JAZZY
			case _:
				return ROSD.Name.UNKNOWN

