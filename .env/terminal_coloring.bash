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

# Check this out: https://www.shellhacks.com/bash-colors/
export TERMINAL_COLOR_NC='\e[0m' # No Color
export TERMINAL_COLOR_BLACK='\e[0;30m'
export TERMINAL_COLOR_GRAY='\e[1;30m'
export TERMINAL_COLOR_RED='\e[0;31m'
export TERMINAL_COLOR_LIGHT_RED='\e[1;31m'
export TERMINAL_COLOR_GREEN='\e[0;32m'
export TERMINAL_COLOR_LIGHT_GREEN='\e[1;32m'
export TERMINAL_COLOR_BROWN='\e[0;33m'
export TERMINAL_COLOR_YELLOW='\e[1;33m'
export TERMINAL_COLOR_BLUE='\e[0;34m'
export TERMINAL_COLOR_LIGHT_BLUE='\e[1;34m'
export TERMINAL_COLOR_PURPLE='\e[0;35m'
export TERMINAL_COLOR_LIGHT_PURPLE='\e[1;35m'
export TERMINAL_COLOR_CYAN='\e[0;36m'
export TERMINAL_COLOR_LIGHT_CYAN='\e[1;36m'
export TERMINAL_COLOR_LIGHT_GRAY='\e[0;37m'
export TERMINAL_COLOR_WHITE='\e[1;37m'

export TERMINAL_BG_COLOR_BLACK='\e[40m'
export TERMINAL_BG_COLOR_GRAY='\e[1;40m'
export TERMINAL_BG_COLOR_RED='\e[41m'
export TERMINAL_BG_COLOR_LIGHT_RED='\e[1;41m'
export TERMINAL_BG_COLOR_GREEN='\e[42m'
export TERMINAL_BG_COLOR_LIGHT_GREEN='\e[1;42m'
export TERMINAL_BG_COLOR_BROWN='\e[43m'
export TERMINAL_BG_COLOR_YELLOW='\e[1;43m'
export TERMINAL_BG_COLOR_BLUE='\e[44m'
export TERMINAL_BG_COLOR_LIGHT_BLUE='\e[1;44m'
export TERMINAL_BG_COLOR_PURPLE='\e[45m'
export TERMINAL_BG_COLOR_LIGHT_PURPLE='\e[1;45m'
export TERMINAL_BG_COLOR_CYAN='\e[46m'
export TERMINAL_BG_COLOR_LIGHT_CYAN='\e[1;46m'
export TERMINAL_BG_COLOR_LIGHT_GRAY='\e[47m'
export TERMINAL_BG_COLOR_WHITE='\e[1;47m'

function get_gitbranch {
  git branch --show-current 2> /dev/null
}

function get_remote_status_symbol {
  # Return empty if not in repo or no branch
  local branch
  branch="$(git branch --show-current 2>/dev/null)"
  if [[ -z "$branch" ]]; then
    echo ""
    return
  fi

  # Capture porcelain v2 status lines
  local status
  status="$(git status --porcelain=2 --branch 2>/dev/null)"

  # Extract ahead/behind from "branch.ab"
  local ahead behind

  ahead="$(echo "$status" | grep "^# branch.ab" | sed -E 's/.*\+([0-9]+).*/\1/')"
  behind="$(echo "$status" | grep "^# branch.ab" | sed -E 's/.*-([0-9]+).*/\1/')"

  # If both values empty → no upstream or divergence not reported
  if [[ -z "$ahead" && -z "$behind" ]]; then
    echo ""
    return
  fi

  if [[ "$ahead" -gt 0 && "$behind" -gt 0 ]]; then
    echo "!"   # diverged
  elif [[ "$ahead" -gt 0 ]]; then
    echo "+"   # ahead only
  elif [[ "$behind" -gt 0 ]]; then
    echo "-"   # behind only
  else
    echo ""    # clean/no divergence
  fi
}

function get_status_color {
  local previous_color
  previous_color=$1
  local porcelain
  porcelain="$(git status --porcelain 2>/dev/null)"

  if [[ -z "$(git rev-parse --is-inside-work-tree 2>/dev/null)" ]]; then
    echo "${TERMINAL_COLOR_GREEN}"
    return
  fi

  # Untracked (??) or modified (M, A, D on right column)
  if echo "$porcelain" | grep -qE "^\?\?|^.M|^..M|^.D|^..D"; then
    echo "${TERMINAL_COLOR_RED}"
    return
  fi

  # Staged only (left column: A, M, D; right column clean or nothing)
  if echo "$porcelain" | grep -qE "^(A.|M.|D.)"; then
    echo "${TERMINAL_COLOR_BROWN}" # "orange" → mapped to BROWN
    return
  fi
  echo "${previous_color}"
}

function get_stash_color {
  local previous_color
  previous_color=$1
  if [[ -n "$(git rev-parse --is-inside-work-tree 2>/dev/null)" ]] \
     && git stash list 2>/dev/null | grep -q .; then
    echo "${TERMINAL_COLOR_YELLOW}"
  fi
  echo "${previous_color}"
}

function get_git_bracket {
  local branch
  branch="$(get_gitbranch)"

  if [[ -n "$branch" ]]; then
    echo "${TERMINAL_COLOR_GREEN}<"
  else
    echo ""
  fi
}

function get_git_branch_and_status_symbol {
  local branch
  branch="$(get_gitbranch)"

  if [[ -n "$branch" ]]; then
    local status_symbol
    status_symbol="$(get_remote_status_symbol)"

    if [[ -n "$status_symbol" ]]; then
      status_symbol=" ${status_symbol}"
    fi
    echo "${branch}${status_symbol}"
  fi
}

function full_qualified_git_branch {
  local branch
  branch="$(get_git_branch_and_status_symbol)"
  local git_bracket="$(get_git_bracket)"
  local default_color="${TERMINAL_COLOR_GREEN}"

  if [[ -n "$branch" ]]; then
    local stash_color
    stash_color="$(get_stash_color ${default_color})"
    local status_color
    status_color="$(get_status_color ${stash_color})"

    echo "${git_bracket}${status_color}${branch}"
  else
    echo ""
  fi
}

function detect_environment {
    # Detect remote SSH
    if [[ -n "$SSH_CONNECTION" ]] || [[ -n "$SSH_TTY" ]]; then
        echo "remote"
        return
    fi

    # Detect Docker container
    if [[ -f "/.dockerenv" ]] || grep -qi docker /proc/1/cgroup 2>/dev/null; then
        echo "docker"
        return
    fi

    # Default: local machine
    echo "local"
}

function env_color {
    local env
    env="$(detect_environment)"

    case "$env" in
        local)
            echo -e "${TERMINAL_COLOR_GREEN}"
            ;;
        remote)
            echo -e "${TERMINAL_COLOR_YELLOW}"
            ;;
        docker)
            # Assuming "local-like"
            echo -e "${TERMINAL_COLOR_YELLOW}"
            ;;
    esac
}

function env_tag {
    local env
    env="$(detect_environment)"

    case "$env" in
        local)
            echo ""
            ;;
        docker)
            echo "cntr"
            ;;
        remote)
            echo "ssh"
            ;;
    esac
}

function full_qualified_env_tag {
    local tag
    tag="$(env_tag)"
    local color
    color="$(env_color)"

    if [[ -n "$tag" ]]; then
        echo "[${color}${tag}]"
    fi
}

function __update_prompt {
  local git_component
  git_component="$(full_qualified_git_branch)"
  local env_color
  env_color="$(env_color)"
  local env_tag
  env_tag="$(full_qualified_env_tag)"

  PS1="\[${TERMINAL_COLOR_LIGHT_GREEN}\]\u\
\[${TERMINAL_COLOR_LIGHT_GRAY}\]@\
\[${env_color}\]\h\
\[${env_tag}\]\
\[${TERMINAL_COLOR_LIGHT_GRAY}\]:\
\[${git_component}\]\
\[${TERMINAL_COLOR_GREEN}\]>\
\[${TERMINAL_COLOR_LIGHT_PURPLE}\]\W\
\[${TERMINAL_COLOR_LIGHT_PURPLE}\]\$\
\[${TERMINAL_COLOR_NC}\] "
}
PROMPT_COMMAND=__update_prompt