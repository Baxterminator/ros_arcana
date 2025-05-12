#!/bin/env python3
# =============================================================================
#                           ROS2 Mappings Exports
#
# (c) Meltwin 2025
# Author: Geoffrey Côte
#
# Part of the arcana_cmake package
# This python file aims to create mappings in several languages for the ROS
# distros, e.g. to change behavior depending on versions.
# =============================================================================

from abc import abstractmethod, ABC
from argparse import ArgumentParser
from dataclasses import dataclass
from io import TextIOWrapper
import os
import sys
from typing import Dict, List, Literal
from yaml import safe_load

IOFile = TextIOWrapper  # type: ignore


# =============================================================================
# Generators definitions
# =============================================================================
class MappingGenerator(ABC):
    """
    Interface to generate mappings for one language.
    """

    def __init__(self, comment_prefix: str):
        self._comment_pre = comment_prefix
        self.n_cols = 80 - len(self._comment_pre) - 2
        self._f: IOFile

    def configure_file(self, f: IOFile) -> None:
        self._f = f

    # -----------------------------------------------------
    # Header / Footer
    # -----------------------------------------------------

    def out_header(self, ns: str) -> None:
        pass

    def _parse_licence(self, licence_path: str) -> List[str]:
        licence_lines = []
        with open(licence_path, "r") as licence:
            line = "f"
            while line != "":
                line = licence.readline().strip()
                if line == "//":
                    licence_lines.append("")
                else:
                    licence_lines.extend(
                        [
                            line[i : i + self.n_cols].strip()
                            for i in range(0, len(line), self.n_cols)
                        ]
                    )

        return licence_lines

    def out_copyright(self, licence_file: str) -> None:
        # Load the licence file
        licence = []
        if len(licence_file) != 0 and os.path.exists(licence_file):
            licence = self._parse_licence(licence_file)

        sep_line = "".center(self.n_cols, "=")
        header = f"""
{'ROS2 installation utils methods'.center(self.n_cols)}
Auto-generated file that contains the ROS distro mappings
""".splitlines()
        header = [sep_line, *header[1:], "", *licence, sep_line]

        # Export them
        self._f.writelines([f"{self._comment_pre} {line}\n" for line in header])

    def out_includes(self) -> None:
        pass

    def out_footer(self, ns: str) -> None:
        pass

    # -----------------------------------------------------
    # File contents
    # -----------------------------------------------------

    @abstractmethod
    def out_mappings(self, mappings: Dict[str, str]) -> None:
        pass

    @abstractmethod
    def out_version_to_name(self, mappings: Dict[str, str]) -> None:
        pass

    @abstractmethod
    def out_name_to_version(self, mappings: Dict[str, str]) -> None:
        pass

    # -----------------------------------------------------
    # Utils
    # -----------------------------------------------------
    @staticmethod
    def version_as_int(v: str) -> int:
        parts = v.split(".")
        out = 0
        out += int(parts[0]) * 1e3
        if len(parts) > 1:
            out += int(parts[1])
        return int(out)


# =============================================================================


class CMakeGenerator(MappingGenerator):
    """
    Generator specialization for CMake exports.
    """

    def __init__(self):
        super().__init__("#")

    # -----------------------------------------------------
    # File contents
    # -----------------------------------------------------

    def out_mappings(self, mappings: Dict[str, str]) -> None:
        self._f.write('set(ROSD_UNKNOWN "0.0")\n')
        for key, val in mappings.items():
            self._f.write(f'set(ROSD_{key.upper()} "{val}")\n')

    def out_version_to_name(self, mappings: Dict[str, str]) -> None:
        self._f.write("# Return a stringified version of the ROS distro code.\n")
        self._f.write("function (rosd_to_str v out)\n")
        first = True
        for key, val in mappings.items():
            if first:
                self._f.write(f"  if")
                first = False
            else:
                self._f.write(f"  elseif")
            self._f.write(f' (${{v}} VERSION_EQUAL "{val}")\n')
            self._f.write(f'    set(${{out}} "{key}" PARENT_SCOPE)\n')
            self._f.write(f"    return()\n")
        self._f.write("  else()\n")
        self._f.write('    set(${out} "unknown" PARENT_SCOPE)\n')
        self._f.write(
            '    message(WARNING "${v} does not correspond to any version code of the registered ROS distro")\n'
        )
        self._f.write("  endif()\n")
        self._f.write("endfunction()\n")

    def out_name_to_version(self, mappings: Dict[str, str]) -> None:
        self._f.write("# Return the equivalent version of the ROS distro.\n")
        self._f.write("function (str_to_rosd v out)\n")
        first = True
        for key, val in mappings.items():
            if first:
                self._f.write(f"  if")
                first = False
            else:
                self._f.write(f"  elseif")
            self._f.write(f' (${{v}} STREQUAL "{key}")\n')
            self._f.write(f"    set(${{out}} ${{ROSD_{key.upper()}}} PARENT_SCOPE)\n")
            self._f.write(f"    return()\n")
        self._f.write("  else()\n")
        self._f.write("    set(${out} ${ROSD_UNKNOWN} PARENT_SCOPE)\n")
        self._f.write(
            '    message(WARNING "${v} does not correspond to any of the registered ROS distro")\n'
        )
        self._f.write("  endif()\n")
        self._f.write("endfunction()\n")


class CPPGenerator(MappingGenerator):
    """
    Generator specialization for C++ exports.
    """

    def __init__(self):
        super().__init__("//")

    # -----------------------------------------------------
    # Header / Footer
    # -----------------------------------------------------

    def out_header(self, ns: str) -> None:
        self._f.writelines(
            [f"#ifndef {ns.upper()}_ROSD_HPP\n", f"#define {ns.upper()}_ROSD_HPP\n"]
        )

    def out_footer(self, ns: str) -> None:
        self._f.write("#endif\n")

    # -----------------------------------------------------
    # File contents
    # -----------------------------------------------------
    def out_mappings(self, mappings: Dict[str, str]) -> None:
        self._f.write("#define ROSD_UNKNOWN 0\n")
        for key, val in mappings.items():
            self._f.write(
                f"#define ROSD_{key.upper()} {MappingGenerator.version_as_int(val)}\n"
            )

    def out_name_to_version(self, mappings: Dict[str, str]) -> None:
        """
        This function is not implemented as it targets compile-time computations.
        """

    def out_version_to_name(self, mappings: Dict[str, str]) -> None:
        self._f.write(
            """/**
 * Return a stringified version of the ROS distro code.
*/
"""
        )
        self._f.write("constexpr const char* ROSD2STR(const int v) {\n")
        self._f.write("  switch (v) {\n")
        for key, _ in mappings.items():
            self._f.write(f"    case ROSD_{key.upper()}:\n")
            self._f.write(f'      return "{key.upper()}";\n')
        self._f.write(f"    default:\n")
        self._f.write(f'      return "UNKNOWN";\n')
        self._f.write("  }\n}")


class PythonGenerator(MappingGenerator):
    """
    Generator specialization for Python exports.
    """

    def __init__(self):
        super().__init__("#")

    def out_includes(self) -> None:
        self._f.write("from enum import IntEnum\n")

    # -----------------------------------------------------
    # File contents
    # -----------------------------------------------------

    def out_mappings(self, mappings: Dict[str, str]) -> None:
        self._f.write("class ROSD:\n")

        # Write versions
        self._f.write("\tclass Version(IntEnum):\n")
        self._f.write("\t\tUNKNOWN = 0\n")
        for key, val in mappings.items():
            self._f.write(
                f"\t\t{key.upper()} = {MappingGenerator.version_as_int(val)}\n"
            )

        # Write names
        self._f.write("\tclass Name:\n")
        self._f.write('\t\tUNKNOWN = "UNKNOWN"\n')
        for key, _ in mappings.items():
            self._f.write(f'\t\t{key.upper()} = "{key}"\n')

    def out_version_to_name(self, mappings: Dict[str, str]) -> None:
        self._f.write("\t@staticmethod\n")
        self._f.write('\tdef to_string(v: "ROSD.Version") -> str:\n')
        self._f.write(
            """\t\t\"\"\"
\t\tReturn a stringified version of the ROS distro code.
\t\t\"\"\"
"""
        )
        self._f.write("\t\tmatch v:\n")
        for key, _ in mappings.items():
            self._f.write(f"\t\t\tcase ROSD.Version.{key.upper()}:\n")
            self._f.write(f"\t\t\t\treturn ROSD.Name.{key.upper()}\n")
        self._f.write(f"\t\t\tcase _:\n")
        self._f.write(f"\t\t\t\treturn ROSD.Name.UNKNOWN\n")

    def out_name_to_version(self, mappings: Dict[str, str]) -> None:
        self._f.write("\t@staticmethod\n")
        self._f.write('\tdef from_string(n: str) -> "ROSD.Version":\n')
        self._f.write(
            """\t\t\"\"\"
\t\tReturn the equivalent version of the ROS distro
\t\t\"\"\"
"""
        )
        self._f.write("\t\tmatch n:\n")
        for key, _ in mappings.items():
            self._f.write(f"\t\t\tcase ROSD.Name.{key.upper()}:\n")
            self._f.write(f"\t\t\t\treturn ROSD.Version.{key.upper()}\n")
        self._f.write(f"\t\t\tcase _:\n")
        self._f.write(f"\t\t\t\treturn ROSD.Version.UNKNOWN\n")


# =============================================================================
# Main application
# =============================================================================
class ROSD_Export:
    """
    This programs export the ROS distro mappings.
    """

    @dataclass
    class Files:
        mappings: str = "./rosdistros.yaml"
        licence: str = ""

    def __init__(self):
        # Parse arguments
        self._parser = ArgumentParser("mk_rosd.py")
        self.files = ROSD_Export.Files()
        self.distro: Dict[str, str]
        self._parse_args()
        _args = self._parser.parse_args()

        # Assign parameters
        self.lang: Literal["cmake", "cpp", "py"] = _args.lang
        self.out: str = _args.out
        self.ns: str = _args.namespace
        self.files.mappings = _args.mapping
        self.files.licence = _args.licence

    def _parse_args(self):
        """
        Declare and parse the program arguments
        """
        self._parser.add_argument(
            "lang",
            choices=["cmake", "cpp", "py"],
            help="The language in which to export the ROS distro mappings",
        )
        self._parser.add_argument(
            "out",
            help="The path to the out file",
        )
        self._parser.add_argument(
            "-m",
            dest="mapping",
            help="The path to the YAML mapping file",
            default="./rosdistros.yaml",
        )
        self._parser.add_argument(
            "-l",
            dest="licence",
            help="The path to the licence file",
            default="",
        )
        self._parser.add_argument(
            "--ns",
            dest="namespace",
            help="The namespace for the constants",
            default="arcana",
        )

    def import_mappings(self) -> bool:
        """
        Import the local YAML mappings file
        """

        def print_error(e: str):
            print(e, file=sys.stderr)

        abs_file = os.path.abspath(self.files.mappings)
        if not os.path.exists(abs_file):
            print_error(f"Mapping file {abs_file} was not found !")
            return False
        with open(abs_file, "r") as mappings:
            yaml_data: dict = safe_load(mappings)

            # Check for a "distro" key:
            if "distro" not in yaml_data.keys():
                print_error('The parsed YAML does not contains a root "distro" object')
                return False
            self.distro = yaml_data["distro"]

            # Check every distro value
            for key, val in self.distro.items():
                if type(val) is not str:
                    print_error(f"Distro {key} does not have a string value!")
                    return False
        return True

    def export_mappings(self, gen: MappingGenerator) -> None:
        """
        Run the generator to make the mappings.

        Args:
            gen (MappingGenerator): the generator to use
        """
        # Test if build dir exist
        build_dir = os.path.dirname(self.out)
        if not os.path.exists(build_dir):
            os.mkdir(build_dir)

        with open(self.out, "w+") as mapping_files:
            gen.configure_file(mapping_files)

            # Export mapping
            gen.out_header(self.ns)
            gen.out_copyright(self.files.licence)
            mapping_files.write("\n")
            gen.out_includes()
            mapping_files.write("\n")
            gen.out_mappings(self.distro)
            mapping_files.write("\n")
            gen.out_name_to_version(self.distro)
            mapping_files.write("\n")
            gen.out_version_to_name(self.distro)
            mapping_files.write("\n")
            gen.out_footer(self.ns)


if __name__ == "__main__":
    exporter = ROSD_Export()
    print(f"Exporting file to {exporter.out}")
    # Try to load ros distro mappings
    if not exporter.import_mappings():
        exit(-1)

    # Export it in the right language
    match exporter.lang:
        case "cmake":
            exporter.export_mappings(CMakeGenerator())
        case "cpp":
            exporter.export_mappings(CPPGenerator())
        case "py":
            exporter.export_mappings(PythonGenerator())
