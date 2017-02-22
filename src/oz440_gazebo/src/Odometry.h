//==================================================================================================
//
//  Copyright(c)  2016  Naïo Technologies
//
//  These coded instructions, statements, and computer programs contain unpublished proprietary
//  information written by Naio Technologies and are protected by copyright law. They may not be
//  disclosed to third parties or copied or duplicated in any form, in whole or in part, without
//  the prior written consent of Naio Technologies.
//
//==================================================================================================

#ifndef PROJECT_ODOMETRY_H
#define PROJECT_ODOMETRY_H

//==================================================================================================
// I N C L U D E   F I L E S

#include "Can.h"
#include <tf/transform_listener.h>

#include <memory>

//==================================================================================================
// F O R W A R D   D E C L A R A T I O N S

//==================================================================================================
// C O N S T A N T S

//==================================================================================================
// C L A S S E S

class Odometry {

public:
//-- Methods ---------------------------------------------------------------------------------------

    Odometry( std::shared_ptr<Can> can_ptr );
    ~Odometry();

    void init();
    void cleanup();

private:

    void odometry_thread();
    double getPitch( std::string wheel);
    bool odo_wheel( bool & wheel, double& pitch, double& pitch_last_tic, int& forward_backward);

//-- Attributs ---------------------------------------------------------------------------------------

    std::atomic<bool> stop_;

    std::thread odometry_thread_;

    std::shared_ptr<tf::TransformListener> listener_ptr_;

    std::shared_ptr<Can> can_ptr_;

};


#endif //PROJECT_ODOMETRY_H
