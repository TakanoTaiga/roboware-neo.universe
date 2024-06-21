# Copyright 2024 Hakoroboken
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

int32 START=0
int32 END=1
int32 SETPOSE=2
int32 ADDPOSE=3
int32 FIND=4


# Header for timestamp and frame id.
std_msgs/Header header

int32 task
int32 id

# Only used if the type specified has some use for them (eg. SETPOSE, ADDPOSE etc.)
geometry_msgs/PoseStamped pose

# Only used if the type specified has some use for them (eg. FIND etc.)
rw_planning_msg/FindObject[] object
