#!/bin/bash
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Manuel Muth

function setup_env_dir() {
  echo "$( cd "$( dirname "${BASH_SOURCE[0]}" )" > /dev/null && pwd )"
}

function setup_env_workspace_dir() {
  echo "$( dirname "$( setup_env_dir )" )"
}

function setup_global_gitignore() {
    current=$(git config --global --get core.excludesFile || echo "")
    desired="$(setup_env_dir)/.global_gitignore"

    if [ "$current" != "$desired" ]; then
        git config --global core.excludesFile "$desired"
    fi
}

setup_bashrc_bootstrap() {
    local guard_file="$HOME/.env_setup_completed"
    local env_dir
    env_dir="$(setup_env_dir)"

    # write as $HOME/..., not /Users/you/...
    local relative_env_dir="${env_dir/#$HOME/\$HOME}"

    local source_line="if [ -f \"$relative_env_dir/setup_env.bash\" ]; then source \"$relative_env_dir/setup_env.bash\"; fi"

    # Only add to .bashrc if we haven't already
    if [ ! -f "$guard_file" ]; then
        echo "" >> "$HOME/.bashrc"
        echo "# Workspace environment bootstrap" >> "$HOME/.bashrc"
        echo "$source_line" >> "$HOME/.bashrc"

        # Create guard file
        touch "$guard_file"
    fi
}

function setup_env() {
    # Enable bash completion for colcon
    eval "$(register-python-argcomplete colcon)"
    
    setup_bashrc_bootstrap

    setup_global_gitignore

    local _bash_aliases_script="$(setup_env_dir)/bash_aliases.bash"
    if [ -f "${_bash_aliases_script}" ]; then
        source "${_bash_aliases_script}"
    fi

    local _bash_commands_script="$(setup_env_dir)/bash_commands.bash"
    if [ -f "${_bash_commands_script}" ]; then
        source "${_bash_commands_script}" 
    fi

    local _terminal_coloring_script="$(setup_env_dir)/terminal_coloring.bash"
    if [ -f "${_terminal_coloring_script}" ]; then
        source "${_terminal_coloring_script}" 
    fi

    local _ros_setup_script="$(setup_env_workspace_dir)/install/setup.bash"
    if [ -f "${_ros_setup_script}" ]; then
        source "${_ros_setup_script}" 
    fi
}

setup_env