# This file marks a workspace root for the Bazel build system.
# See `https://bazel.build/`.

workspace(name = "drake")

load("//tools/workspace:default.bzl", "add_default_workspace")

add_default_workspace()

load("@build_bazel_apple_support//crosstool:setup.bzl", "apple_cc_configure")

apple_cc_configure()

# Add some special heuristic logic for using CLion with Drake.
load("//tools/clion:repository.bzl", "drake_clion_environment")

drake_clion_environment()

load("@bazel_skylib//lib:versions.bzl", "versions")

# This needs to be in WORKSPACE or a repository rule for native.bazel_version
# to actually be defined. The minimum_bazel_version value should match the
# version passed to the find_package(Bazel) call in the root CMakeLists.txt.
versions.check(minimum_bazel_version = "6.0")

# The cargo_universe programs are only used by Drake's new_release tooling, not
# by any compilation rules. As such, we can put it directly into the WORKSPACE
# instead of into our `//tools/workspace:default.bzl` repositories.
load("@rules_rust//crate_universe:repositories.bzl", "crate_universe_dependencies")  # noqa

crate_universe_dependencies(bootstrap = True)

# custom libraries (hedaniel7)
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "gflags",
    urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
    strip_prefix = "gflags-2.2.2",
    build_file_content = """
cc_library(
    name = "gflags",
    srcs = glob(["src/*.cc"]),
    hdrs = glob(["src/*.h"]),
    includes = ["src"],
    visibility = ["//visibility:public"],
)
""",
)