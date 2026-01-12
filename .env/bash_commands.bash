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

# create a new directory and cd into it after creation
mkdircd() {
  mkdir -p "$1" && cd "$1"
}

kill_background_task() {
  # Get current jobs list
  local job_list
  job_list=$(jobs -p)

  # Exit if no jobs
  if [[ -z "$job_list" ]]; then
    echo "No background jobs to kill."
    return
  fi

  # Helper to kill a job by number
  _kill_job_by_number() {
    local job_num="$1"
    if jobs %"$job_num" &>/dev/null; then
      kill %"$job_num"
      echo "Killed job %$job_num"
    else
      echo "Invalid job number: $job_num\n"
      kill_background_task  # Recursive retry
    fi
  }

  # If argument is passed
  if [[ -n "$1" ]]; then
    _kill_job_by_number "$1"
  else
    # Show jobs
    jobs
    echo -n "Enter job number to kill (or press Enter to cancel): "
    read jobnum
    if [[ -z "$jobnum" ]]; then
      echo "Cancelled."
      return
    fi
    _kill_job_by_number "$jobnum"
  fi
}

cdws() {
  cd $(setup_env_workspace_dir)
}

cdwss() {
  cd $(setup_env_workspace_dir)/src
}

cdwsb() {
  cd $(setup_env_workspace_dir)/build
}

cdwsi() {
  cd $(setup_env_workspace_dir)/install
}