//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2019-2021, AMBF
    (https://github.com/WPI-AIM/ambf)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of authors nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    \author    <amunawar@wpi.edu>
    \author    Adnan Munawar
*/
//==============================================================================

#include "ambf_server/RigidBodyRosCom.h"

RigidBodyRosCom::RigidBodyRosCom(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out): RosComBase(a_name, a_namespace, a_freq_min, a_freq_max, time_out){
    init();
}

void RigidBodyRosCom::init(){
    m_State.name.data = m_name;
    m_State.sim_step = 0;

    ambf_ral::create_publisher<AMBF_RAL_MSG(ambf_msgs, RigidBodyState)>
      (m_pubPtr,
       m_nodePtr,
       m_namespace + m_name + "/State",
       10, false);
    ambf_ral::create_subscriber<AMBF_RAL_MSG(ambf_msgs, RigidBodyCmd), RigidBodyRosCom>
      (m_subPtr,
       m_nodePtr,
       m_namespace + m_name + "/Command",
       10,
       &RigidBodyRosCom::sub_cb, this);

    m_thread = std::thread(std::bind(&RigidBodyRosCom::run_publishers, this));
    std::cerr << "INFO! Thread Joined: " << m_name << std::endl;
}

void RigidBodyRosCom::reset_cmd(){
    // For cartesian control, the TYPE_FORCE indicates wrench.
    m_Cmd.cartesian_cmd_type = AMBF_RAL_MSG(ambf_msgs, RigidBodyCmd)::TYPE_FORCE;
    m_Cmd.wrench.force.x = 0.0;
    m_Cmd.wrench.force.y = 0.0;
    m_Cmd.wrench.force.z = 0.0;
    m_Cmd.wrench.torque.x = 0.0;
    m_Cmd.wrench.torque.y = 0.0;
    m_Cmd.wrench.torque.z = 0.0;
    for(size_t idx = 0 ; idx < m_Cmd.joint_cmds.size() ; idx++){
        m_Cmd.joint_cmds[idx] = 0.0;
    }
    for(size_t idx = 0 ; idx < m_Cmd.joint_cmds_types.size() ; idx++){
        // For joint control, the TYPE_FORCE indicates joint effort, which could be force for prismatic joint, or torque for revolute joint.
        m_Cmd.joint_cmds_types[idx] = AMBF_RAL_MSG(ambf_msgs, RigidBodyCmd)::TYPE_FORCE;
    }
}

void RigidBodyRosCom::sub_cb(const AMBF_RAL_MSG(ambf_msgs, RigidBodyCmd) & msg){
    m_Cmd = msg;
    m_watchDogPtr->acknowledge_wd();
}