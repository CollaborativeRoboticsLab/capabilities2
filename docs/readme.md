# Docs

The documentation site is generated with Doxygen from the existing Markdown in this `docs/` tree together with comments from the source code. The Doxygen configuration lives at the repository root in [`Doxygen`](../Doxygen).

```bash
# from the repository root
doxygen Doxygen
```

The generated HTML site is written to `docs/html/` and the entry page is `docs/html/index.html`.

For GitHub Pages, this repository can use GitHub Actions as both the build source and the deployment source. The workflow in `.github/workflows/doxygen-pages.yml` rebuilds the site whenever files change and deploys the generated `docs/html/` artifact to Pages.

## Basic Tutorial

A basic tutorial can be found in [here](./basic_tutorial.md). This tutorial will guide you through the process of creating a simple capability and running it.

## Capabilities Server

The [capabilities server](../capabilities2_server/readme.md) is the main node in the capabilities2 package. It is responsible for managing capabilities, providers, and semantic interfaces. It is also responsible for starting and stopping capabilities.

## Capabilities Runner

A capabilities runner is a plugin that allows capabilities to be executed in a more arbitrary way. Base runner classes can be used to create custom runners for a given application. See the [capabilities2_runner](../capabilities2_runner/readme.md) package for more information.

## Capabilities Messages

This package contains messages for the capabilities2 package. See the [capabilities2_msgs](../capabilities2_msgs/readme.md) package for more information.

## Launch Runner Status

The `LaunchRunner` interface is still kept in the `capabilities2_runner` package, but launch-file execution support is intentionally deferred because of incompatibilities with the current ROS2 launch system. Documentation for launch-based capability execution will be restored once the replacement solution is ready.
