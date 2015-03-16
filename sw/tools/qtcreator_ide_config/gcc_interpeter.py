#!/usr/bin/python

import sys
import os
import re

# Set home directory and unittest directory
TOOL_DIR = os.path.dirname(os.path.realpath(__file__))
PAPARAZZI_HOME = os.path.join(TOOL_DIR, os.pardir, os.pardir, os.pardir)
PAPARAZZI_HOME = os.path.realpath(PAPARAZZI_HOME)
TEST_HOME = os.path.join(PAPARAZZI_HOME, "tests", "unittest")

# Only look at gcc command in which compilation of an object takes place.
# This has the -MMD flag.
if not (sys.argv[2] == "-MMD"):
    quit()

# Aircraft name substituted in arguments by configure_workspace.py
AIRCRAFT = sys.argv[1]

# Check if also a test environment is configured for this aircraft
if not os.path.isfile(os.path.join(TEST_HOME, AIRCRAFT + "Tester.creator")):
    TEST_CONF = False
else:
    TEST_CONF = True

# Update config+includes only once (this script is called many times, but
# config and includes are always the same)
if not os.path.isfile(os.path.join(PAPARAZZI_HOME, AIRCRAFT + ".config")):
    print "Generating config and includes file"
    config = open(os.path.join(PAPARAZZI_HOME, AIRCRAFT + ".config"), "w")
    includes = open(os.path.join(PAPARAZZI_HOME, AIRCRAFT + ".includes"), "w")
    if TEST_CONF:
        test_config_name = os.path.join(TEST_HOME, AIRCRAFT + "Tester.config")
        test_config = open(test_config_name, "w")
        test_inc_name = os.path.join(TEST_HOME, AIRCRAFT + "Tester.includes")
        test_includes = open(test_inc_name, "w")

    for argument in sys.argv:
        # Look for flag -Ddefine=value
        matchObj = re.match(r'-D(.+)=(.*)', argument)
        if(matchObj):
            newDefine = "#define " + matchObj.group(1) + " "
            newDefine += matchObj.group(2) + "\n"
            newDefine = newDefine.replace('\\"', '"')
            newDefine = newDefine.replace('\'', '')
            config.write(newDefine)
            if TEST_CONF:
                test_config.write(newDefine)
            continue

        # Look for flag -Ddefine (without value)
        matchObj = re.match(r'-D(.+)', argument)
        if(matchObj):
            newDefine = '#define ' + matchObj.group(1) + ' 1\n'
            config.write(newDefine)
            if TEST_CONF:
                test_config.write(newDefine)
            continue

        # Look for flag include path
        matchObj = re.match(r'-I(.+)', argument)
        if(matchObj):
            newInclude = os.path.normpath(os.path.join(PAPARAZZI_HOME,
                                                       "sw",
                                                       "airborne",
                                                       matchObj.group(1)))
            includes.write(newInclude + "\n")
            if TEST_CONF:
                test_includes.write(newInclude + "\n")
                newInclude = os.path.normpath(
                    os.path.join(TEST_HOME, "sw", "airborne",
                                 matchObj.group(1)))
                test_includes.write(newInclude + "\n")
    # end for loop

    # close the project files
    config.close()
    includes.close()
    if TEST_CONF:
        # extra includes for unittest
        EXT_TOOLS = os.path.join(PAPARAZZI_HOME, "sw", "ext")
        test_includes.write(os.path.join(EXT_TOOLS, "cmock", "src") + "\n")
        test_includes.write(os.path.join(EXT_TOOLS, "unity", "src") + "\n")
        test_includes.write(os.path.join(EXT_TOOLS, "unity", "extras",
                                         "fixture", "src") + "\n")
        # close test project files
        test_config.close()
        test_includes.close()

# The files-file must be update every every call, to resolve all dependencies
# First, extract all files currently in there
files_file = open(os.path.join(PAPARAZZI_HOME, AIRCRAFT + ".files"), "r")
all_files = files_file.read().split("\n")
all_files.remove("")
discovered_files = []

# Generate includeflags from projects includes file
includes = open(os.path.join(PAPARAZZI_HOME, AIRCRAFT + ".includes"))
includescontent = includes.read().split("\n")
includescontent.remove("")
includeflags = ""
for incdir in includescontent:
    includeflags += "-I" + incdir + " "
includes.close()

# Extract c file from arguments
for argument in sys.argv:
    if(argument.endswith('.c')):
        c_file = os.path.join(PAPARAZZI_HOME, "sw", "airborne", argument)
        discovered_files.append(c_file)

        # For this c-file, resolve all dependencies
        cmd = "gcc -MM " + c_file + " " + includeflags + " -include"
        cmd += os.path.join(PAPARAZZI_HOME, AIRCRAFT + ".config")
        headers = os.popen(cmd).read().split('\\')
        # Start at 2 because 0=.c and 1=.config
        for i in range(2, len(headers)-1):
            header = headers[i].strip()
            discovered_files.append(header)

# Append discovered files to all files
for newfile in discovered_files:
    if(all_files.count(newfile) == 0):
        all_files.append(newfile)

# Rewrite the files-file
files_file.close()
files_file = open(os.path.join(PAPARAZZI_HOME, AIRCRAFT + ".files"), "w")
for xfile in all_files:
    files_file.write(xfile + "\n")

files_file.close()
