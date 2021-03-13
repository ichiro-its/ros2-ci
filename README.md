# ROS 2 Continuous Integration

<a href="https://github.com/ichiro-its/ros2-ci/actions"><img alt="GitHub Actions status" src="https://github.com/ichiro-its/ros2-ci/actions/workflows/test-ros2-ci.yml/badge.svg"></a>

This action could be used as [a continuous integration (CI)](https://en.wikipedia.org/wiki/Continuous_integration) to build and test a [ROS 2](https://docs.ros.org/en/foxy/) project.

## How It Works?

This action works by building and testing a ROS 2 project inside a [Docker](https://www.docker.com/)'s container using [the official ROS 2 docker image](https://hub.docker.com/_/ros).
To handle which ROS 2 environment to be used, this action uses [a Docker in Docker (DinD)](https://hub.docker.com/_/docker).
Hence the CI process happens inside the container's container, not just the first layer of the Docker's container for GitHub Actions.
The CI process is guaranteed to be fast (approximately 3-5 minutes, depends on each package) as almost every component of the ROS 2 has already been installed inside the Docker's image.

To be able to build and test the project inside the repository, This action requires [the Checkout action](https://github.com/marketplace/actions/checkout).
Before the build process, the project files will be put under the `/ws/repo` directory inside the container.
Then, the build and test process will be done using [colcon](https://colcon.readthedocs.io/en/released/index.html) inside the `/ws` directory and the results (`build`, `install`, and `log`) will be copied inside the `/ws/repo/.ws`.

The drawback of using this action is it still needs to install external dependencies that haven't been included in the ROS 2 default installation.
But that problem could be solved by adding a `apt-packages` or a `pip-packages` input inside the action configuration or by cloning external packages inside the `/ws` directory before the build process happens.

## Usage

Create a new workflow configuration on `.github/workflows` with the following content:
```yaml
name: ROS 2 CI
on:
  pull_request:
    branches: [ master ]
  push:
    branches: [ master ]
jobs:
  build-and-test:
    runs-on: ubuntu-latest
    name: Build and test the project
    steps:
      - name: Checkout
        uses: actions/checkout@v2.3.4
      - name: Build and test
        uses: ichiro-its/ros2-ci@v0.4.0
        with:
          ros2-distro: foxy
          apt-packages: libssl-dev libopencv-dev
          post-build: ros2 pkg list
```
> For more information, see [action.yml](./action.yml) and [this guide](https://docs.github.com/en/actions/learn-github-actions/introduction-to-github-actions).

## License

This project is licensed under the [MIT License](./LICENSE).
