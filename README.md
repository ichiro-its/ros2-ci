# ROS 2 Continuous Integration

Build and test your [ROS 2](https://docs.ros.org/en/foxy/) project using [GitHub Actions](https://github.com/features/actions).

## Usage

Create a new workflow configuration on `.github/workflows` with the following content:
```yaml
name: ROS 2 CI
on:
  pull_request:
    branches:
      - master
  push:
    branches:
      - master
jobs:
  build-and-test:
    runs-on: ubuntu-latest
    name: Build and test the project
    steps:
      - name: Checkout
        uses: actions/checkout@v2.3.4
      - name: Build and test
        uses: threeal/ros2-ci@v0.2.2
        with:
          ros2-distro: foxy
```
> For more information, see [action.yml](./action.yml) and [this guide](https://docs.github.com/en/actions/learn-github-actions/introduction-to-github-actions).

## License

This project is licensed under the [MIT License](./LICENSE).
