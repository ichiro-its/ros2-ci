# ROS 2 Continuous Integration

[![latest version](https://img.shields.io/github/v/release/ichiro-its/ros2-ci.svg)](https://github.com/ichiro-its/ros2-ci/releases/)
[![license](https://img.shields.io/github/license/ichiro-its/ros2-ci.svg)](./LICENSE)
[![workflows test status](https://github.com/ichiro-its/ros2-ci/actions/workflows/workflows-test.yml/badge.svg)](https://github.com/ichiro-its/ros2-ci/actions)

This action could be used as [a continuous integration (CI)](https://en.wikipedia.org/wiki/Continuous_integration) to build and test a [ROS 2](https://docs.ros.org/en/foxy/) project.

## How It Works?

This action works by building and testing a ROS 2 project inside a [Docker](https://www.docker.com/)'s container using [the official ROS 2 Docker image](https://hub.docker.com/_/ros).
To handle which ROS 2 environment to be used, this action uses [a Docker in Docker (DinD)](https://hub.docker.com/_/docker).
Hence the CI process happens inside the container's container, not just the first layer of the Docker's container for GitHub Actions.
Although, The CI process is guaranteed to be fast (approximately 3-5 minutes, depends on each package) as almost every component of the ROS 2 has already been installed inside the Docker's image.

To be able to build and test the project inside the repository, This action requires [the Checkout action](https://github.com/marketplace/actions/checkout).
Before the build process, the project files will be put under the `/ws/repo` directory inside the container.
Then, the build and test process will be done using [colcon](https://colcon.readthedocs.io/en/released/index.html) inside the `/ws` directory.
After the test process happens, the build results (`build`, `install`, and `log`) will be copied inside the `/ws/repo/.ws`.

The drawback of using this action is it still needs to install external dependencies that haven't been included in the ROS 2 default installation.
But that problem could be solved by adding a `apt-packages` or a `pip-packages` input inside the action configuration (see [this](#Install-Several-Dependencies)) or by cloning external packages inside the `/ws` directory before the build process happens (see [this](#Include-External-Project)).

## Usage

For more information, see [the action.yml](./action.yml) and [the GitHub Actions guide](https://docs.github.com/en/actions/learn-github-actions/introduction-to-github-actions).

### Default Usage

```yaml
name: Build and Test
on:
  pull_request:
    branches: [ main ]
  push:
    branches: [ main ]
jobs:
  build-and-test:
    runs-on: ubuntu-latest
    steps:
      - name: Checking out
        uses: actions/checkout@v2.3.4
      - name: Building and testing
        uses: ichiro-its/ros2-ci@v0.4.2
```
> This will be defaulted to use [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html).

### Using Different ROS 2 Distributin

```yaml
- name: Building and testing
  uses: ichiro-its/ros2-ci@v0.4.2
  with:
    ros2-distro: rolling
```

### Install Several Dependencies

```yaml
- name: Building and testing
  uses: ichiro-its/ros2-ci@v0.4.2
  with:
    apt-packages: libssh-dev libopencv-dev
```

### Run Command During the Process

```yaml
- name: Building and testing
  uses: ichiro-its/ros2-ci@v0.4.2
  with:
    post-test: ls .ws
```

### Include External Repositories

```yaml
- name: Building and testing
  uses: ichiro-its/ros2-ci@v0.4.2
  with:
    ros2-distro: foxy
    external-repos: https://github.com/ros2/example_interfaces#foxy ros2/examples#foxy
```

## License

This project is licensed under the [MIT License](./LICENSE).
