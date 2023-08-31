load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "mpcinterface",
    srcs = ["src/mpcinterface.cpp"],
    hdrs = ["include/mpcinterface.h"],
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@eigen//:eigen",
    ],
)

cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        "@eigen//:eigen",
    ],
    copts = [
        "-Iexternal/eigen"],
)

