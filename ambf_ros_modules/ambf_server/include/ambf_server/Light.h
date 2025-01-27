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

#ifndef AFLIGHTCOMM_H
#define AFLIGHTCOMM_H

#include <string>
#include "ambf_server/LightRosCom.h"

namespace ambf_comm{


enum class LightType{
    SPOT,
    POINT,
    DIRECTIONAL
};


enum class LightParamsEnum{
    cuttoff_angle,
    parent_name,
    type,
    attenuation
};


class LightParams{

    friend class Light;

public:

    LightParams();

    inline void set_qualified_namespace(std::string a_base_prefix){m_base_prefix = a_base_prefix;}

    // Setters
    void set_type(LightType val){m_light_type = val;}
    void set_cuttoff_angle(double val){m_cuttoff_angle = val;}
    void set_attenuation(double cons, double lin, double quad);

    // Getters
    LightType get_type(){return m_light_type;}
    double get_cuttoff_angle(){return m_cuttoff_angle;}
    double get_constant_attenuation(){return m_attenuation["constant"];}
    double get_linear_attenuation(){return m_attenuation["linear"];}
    double get_quadratic_attenuation(){return m_attenuation["quadratic"];}

    // This a flag to check if any param has been updated
    bool m_paramsChanged;

protected:

    // Namespace + obj_name is the base_prefix. E.g. /ambf/env/ + Light1 = /ambf/env/Light1 -> Base Prefix
    std::string m_base_prefix;

    // Datatyped Variables for params defined on the server
    double m_type;
    double m_cuttoff_angle;
    std::map<std::string, double> m_attenuation; // Constant, Linear and Quadratic Attenuation
    LightType m_light_type;
};

class Light: public LightRosCom, public LightParams{
public:
    Light(std::string a_name, std::string a_namespace, int a_freq_min, int a_freq_max, double time_out);
    void cur_position(double px, double py, double pz);
    void cur_orientation(double roll, double pitch, double yaw);
    void cur_orientation(double qx, double qy, double qz, double qw);
    void set_parent_name(std::string name){m_State.parent_name.data = name;}

    std::string get_parent_name(){return m_State.parent_name.data;}

    // This method updates from the ROS param server instead of topics
    void update_params_from_server();
    // This method may be called when AMBF starts to load the existing
    void set_params_on_server();
};
}

#endif
