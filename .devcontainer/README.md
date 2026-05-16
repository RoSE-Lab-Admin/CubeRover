# Devcontainer Overview

This project uses VS Code Dev Containers to make setup easy for new developers. The instructions for getting setup to use this codebase are provided in the root-level `README.md`.

This README provides information for the development team on the current Docker image configuration and how to update the image.

# Docker Image Update

## Overview

* The `Dockerfile` in this folder provides instructions for Docker to build the image that the Dev Container uses.
* The Docker image being used by the Dev Container is specified at the top of the `devcontainer.json` file.
  - "image": Specifies the prebuilt Docker image being used.
  - "build": Specifies how the Dev Container can build a Docker image (if uncommented).

## Building a New Docker Image

**IMPORTANT:** Any images uploaded to DockerHub and used in the "image" field should be built using the Docker CLI tool.
- If you allow the Dev Container to build a local Docker image using the "build" field in the `devcontainer.json`, that image should NOT be uploaded to DockerHub and used for the "image" field.
    - When a Dev Container builds the Docker image from a `Dockerfile`, the `devcontainer.json` at the time that the image was built is baked into the image.
    - If you use an image built by a Dev Container, you will NOT be able to update the fields in the `devcontainer.json` and have the changes reflected in the container. The container will ALWAYS use the baked in `devcontainer.json` metadata.

### Steps to update the Docker Image (for use with the "image" field in `devcontainer.json`):

### Prerequisites:
* A [Docker Hub account](https://hub.docker.com/).
* A repository created on Docker Hub (though pushing will automatically create a public repository if it doesn't already exist).
* Docker installed and running on your local machine

### Steps to update the Docker Image:

1. **Authenticate to Docker Hub:** You must be logged in before building, because multi-platform builds push the image to the registry immediately.
  ```bash
  docker login
  ```

2. **Setup a Multi-Platform Builder (First time only):** By default, Docker only builds for your machine's architecture. Run this to enable cross-platform building (supporting both Intel/AMD and Apple Silicon/ARM processors):
  ```bash
  docker buildx create --use
  ```

3. **Build and Push the Image:** Navigate to the folder containing the `Dockerfile` and run the build command. (Note: Because this is a multi-platform build, the `--push` flag is required. Docker will build and upload the image simultaneously).
  ```bash
  # Make sure to replace 'yourusername' with your actual Docker Hub username
  docker buildx build --platform linux/amd64,linux/arm64 -t yourusername/cuberover-dev-jazzy:latest --push .
  ```

4. **Verify it is "Clean" (Optional):** Because you used the Docker CLI, the image is safe. However, if you want to verify no VS Code metadata was accidentally baked in, pull the freshly pushed image and inspect it. The following command should return NO output:
  ```bash
  docker pull yourusername/cuberover-dev-jazzy:latest
  docker inspect yourusername/cuberover-dev-jazzy:latest | grep "devcontainer.metadata"
  ```

5. **Update the `devcontainer.json`:** Double-check that the "image" being pointed to in the `devcontainer.json` file matches the new image that you pushed to Docker Hub.

6. **Team Should Rebuild Locally:** After the image is updated on Docker Hub, team members should run the "Dev Containers: Rebuild Without Cache" command in VS Code to use the latest image.
