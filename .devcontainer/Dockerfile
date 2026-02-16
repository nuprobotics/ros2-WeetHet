FROM ros:kilted

RUN apt-get update && apt-get install -y \
    curl \
    build-essential \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    fish \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=30005
ARG USER_GID=$USER_UID

RUN apt-get update \
    && apt-get install -y sudo \
    && rm -rf /var/lib/apt/lists/* \
    && if ! getent group $USER_GID; then groupadd --gid $USER_GID $USERNAME; fi \
    && if ! getent passwd $USER_UID; then useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash; fi \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME

RUN echo "source /opt/ros/kilted/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "exec fish" >> /home/${USERNAME}/.bashrc

RUN mkdir -p /home/${USERNAME}/.config/fish

RUN fish -c "curl -sL https://raw.githubusercontent.com/jorgebucaran/fisher/main/functions/fisher.fish | source && fisher install jorgebucaran/fisher"
RUN fish -c "fisher install edc/bass"
RUN fish -c "fisher install kpbaks/ros2.fish" || true

RUN curl -LsSf https://astral.sh/uv/install.sh | sh

WORKDIR /workspace
