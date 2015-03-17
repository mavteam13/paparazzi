#!/usr/bin/python

import sys
import os
import time

# Set home directory and unittest directory
TOOL_DIR = os.path.dirname(os.path.realpath(__file__))
PAPARAZZI_HOME = os.path.join(TOOL_DIR, os.pardir, os.pardir, os.pardir)
PAPARAZZI_HOME = os.path.realpath(PAPARAZZI_HOME)
TEST_HOME = os.path.join(PAPARAZZI_HOME, "tests", "unittest")

# Aircraft name is argument and should be equal to project name
AIRCRAFT = sys.argv[1]

# Check if the IDE files exist with that name
if not os.path.isfile(os.path.join(PAPARAZZI_HOME, AIRCRAFT + ".creator")):
    raise Exception("Aircraft name does not match project name.")

# Check if also a test environment is configured for this aircraft
if not os.path.isfile(os.path.join(TEST_HOME, AIRCRAFT + "Tester.creator")):
    print "No test environment configured for this aircraft. " + \
        "To do so, create a project called \"" + AIRCRAFT + \
        "Tester\" in " + TEST_HOME + "."
    TEST_CONF = False
else:
    TEST_CONF = True

# First, do make clean on the aircraft
cmd = "make -C " + PAPARAZZI_HOME + " -f Makefile.ac AIRCRAFT=" + \
    AIRCRAFT + " clean_ac"
print "========================"
print "Cleaning"
reply = os.popen(cmd).read()
#print reply
print "========================"

# Remove project files
# If the gcc_interpeter.py runs for the first time, it regenerates these files.
config_file = os.path.join(PAPARAZZI_HOME, AIRCRAFT + ".config")
includes_file = os.path.join(PAPARAZZI_HOME, AIRCRAFT + ".includes")
files_file = os.path.join(PAPARAZZI_HOME, AIRCRAFT + ".files")
if os.path.isfile(config_file):
    os.remove(config_file)
if os.path.isfile(includes_file):
    os.remove(includes_file)
if TEST_CONF:
    test_config_file = os.path.join(TEST_HOME, AIRCRAFT + "Tester.config")
    test_includes_file = os.path.join(TEST_HOME, AIRCRAFT + "Tester.includes")
    if os.path.isfile(test_config_file):
        os.remove(test_config_file)
    if os.path.isfile(test_includes_file):
        os.remove(test_includes_file)

# create empty files file
if os.path.isfile(files_file):
    os.remove(files_file)
new_files_file = open(files_file, "w")
new_files_file.close()

# Do a full build on the aircraft, but replace compiler with python script
print "Analyzing make output.."
gcc_interpeter = os.path.join(TOOL_DIR, "gcc_interpeter.py")
cmd = "make -f Makefile.ac "
make_flags = [
    "PAPARAZZI_HOME=" + PAPARAZZI_HOME,
    "PAPARAZZI_SRC=" + PAPARAZZI_HOME,
    "AIRCRAFT=" + AIRCRAFT,
    "CC='python " + gcc_interpeter + " " + AIRCRAFT + "'",
    "CP='python " + os.path.join(TOOL_DIR, "skip_command.py") + "'",
    "LD='python " + os.path.join(TOOL_DIR, "skip_command.py") + "'",
    "DMP='python " + os.path.join(TOOL_DIR, "skip_command.py") + "'",
    "NM='python " + os.path.join(TOOL_DIR, "skip_command.py") + "'",
    "NPROCS=1",
    "ap.compile"
    ]
cmd = cmd + " ".join(make_flags)
output = os.popen(cmd).read()
print output
print "Done analyzing make output."


## The next section generates the yml file used for unittesting.
if TEST_CONF:
    print "========================"
    print "Configuring unittest framework.."
    # Get all the defines from config file.

    test_config_name = os.path.join(TEST_HOME, AIRCRAFT + "Tester.config")
    test_config_file = open(test_config_name)
    macro_text = test_config_file.read()
    test_config_file.close()
    macro_flags = macro_text.split("\n")
    for i, s in enumerate(macro_flags):
        # defines in <AIRCRAFT>.yml
        macro_flags[i] = s.replace("#define ", "") + "'"
        macro_flags[i] = "    - -D" + macro_flags[i].replace(" ", "='")

    macro_flags.pop()
    yml_macro_flags = "\n".join(macro_flags)

    # Get include directories from testing includes file
    test_includes_name = os.path.join(TEST_HOME, AIRCRAFT + "Tester.includes")
    test_includes_file = open(test_includes_name)
    test_includes_content = test_includes_file.read().split("\n")
    test_includes_file.close()
    for i, s in enumerate(test_includes_content):
        test_includes_content[i] = "      - '" + s + "/'"

    test_includes_content.pop()
    yml_include_items = "\n".join(test_includes_content)

    # Configure the yml script for unit testing
    original_yml = open(os.path.join(TEST_HOME, "yml_template.txt"))
    original_content = original_yml.read()
    original_yml.close()

    new_content = original_content.replace("MACRO_ITEMS", yml_macro_flags)
    new_content = new_content.replace("INCLUDE_ITEMS", yml_include_items)
    new_yml = open(os.path.join(TEST_HOME, AIRCRAFT + ".yml"), "w")
    new_yml.write(new_content)
    new_yml.close()
    print "Done completing unittest framework."
    print "========================"
