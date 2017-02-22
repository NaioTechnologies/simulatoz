//==================================================================================================
//
//  Copyright(c)  2016  Naio Technologies. All rights reserved.
//
//  These coded instructions, statements, and computer programs contain unpublished proprietary
//  information written by Naio Technologies and are protected by copyright law. They may not be
//  disclosed to third parties or copied or duplicated in any form, in whole or in part, without
//  the prior written consent of Naio Technologies.
//
//==================================================================================================

//==================================================================================================
// I N C L U D E   F I L E S

#include "Odometry.h"

//==================================================================================================
// C O N S T R U C T O R (S) / D E S T R U C T O R   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

Odometry::Odometry( std::shared_ptr<Can> can_ptr )
        : stop_ {false}
        , can_ptr_ { can_ptr }

{
    listener_ptr_ = std::make_shared< tf::TransformListener >( ros::Duration( 10 ) );
}

Odometry::~Odometry(){}

//==================================================================================================
// M E T H O D S   C O D E   S E C T I O N

//--------------------------------------------------------------------------------------------------

void Odometry::init()
{
    odometry_thread_ = std::thread( &Odometry::odometry_thread, this );
    odometry_thread_.detach();
}

//--------------------------------------------------------------------------------------------------

void Odometry::cleanup() {
    stop_= true;
}

//--------------------------------------------------------------------------------------------------

void Odometry::odometry_thread()
{

    std::array<bool, 4> ticks{ { false, false, false, false} };

    using namespace std::chrono_literals;

    std::this_thread::sleep_for( 3000ms );

    while( !stop_ )
    {
        if( can_ptr_->connected() )
        {
            // Initialisation
            double pitch_bl = getPitch( "/back_left_wheel" );
            double pitch_fl = getPitch( "/front_left_wheel" );
            double pitch_br = getPitch( "/back_right_wheel" );
            double pitch_fr = getPitch( "/front_right_wheel" );

            double pitch_last_tic_bl = pitch_bl;
            int forward_backward_bl = 0;
            double pitch_last_tic_fl = pitch_fl;
            int forward_backward_fl = 0;
            double pitch_last_tic_fr = pitch_fr;
            int forward_backward_fr = 0;
            double pitch_last_tic_br = pitch_br;
            int forward_backward_br = 0;

            while( !stop_ )
            {
                bool tic = false;

                while( not tic and !stop_ )
                {

                    std::this_thread::sleep_for( 10ms );

                    pitch_bl = getPitch( "/back_left_wheel" );
                    pitch_fl = getPitch( "/front_left_wheel" );
                    pitch_br = getPitch( "/back_right_wheel" );
                    pitch_fr = getPitch( "/front_right_wheel" );

                    tic = odo_wheel( ticks[3], pitch_bl, pitch_last_tic_bl, forward_backward_bl );
                    tic = odo_wheel( ticks[1], pitch_fl, pitch_last_tic_fl, forward_backward_fl ) or tic;
                    tic = odo_wheel( ticks[2], pitch_br, pitch_last_tic_br, forward_backward_br ) or tic;
                    tic = odo_wheel( ticks[0], pitch_fr, pitch_last_tic_fr, forward_backward_fr ) or tic;
                }

                can_ptr_->add_odo_packet( ticks );

            }
        }
        else
        {
            ticks = { { false, false, false, false} };

            std::this_thread::sleep_for( 100ms );
        }
    }
}

//--------------------------------------------------------------------------------------------------

double Odometry::getPitch( std::string wheel )
{
    double roll, pitch, yaw;
    tf::StampedTransform transform;

    try
    {
        (*listener_ptr_).lookupTransform( wheel, "/chassis_bottom", ros::Time( 0 ), transform );
    }
    catch( tf::TransformException& ex )
    {
        ROS_ERROR( "%s", ex.what() );
    }

    transform.getBasis().getRPY( roll, pitch, yaw );

    if( (roll == M_PI and yaw == M_PI) or (roll == -M_PI and yaw == -M_PI) )
    {
        if( pitch >= 0 )
        {
            pitch = pitch - M_PI;
        }
        else
        {
            pitch = pitch + M_PI;
        }
    }
    else
    {
        pitch = -pitch;
    }
    return pitch;
}

//--------------------------------------------------------------------------------------------------

bool Odometry::odo_wheel( bool& odo_wheel, double& pitch, double& pitch_last_tic, int& forward_backward )
{
    double angle_tic = 6.465 / 14.6;
    bool tic = false;

    if( forward_backward == 0 and pitch - pitch_last_tic > 0.001 )
    {
        forward_backward = 1;
    }
    else if( forward_backward == 0 and pitch - pitch_last_tic < -0.001 )
    {
        forward_backward = -1;
    }

    // si on a commencé par avancer
    if( forward_backward == 1 and
        (pitch - pitch_last_tic >= angle_tic or pitch - pitch_last_tic <= 0) )
    {
        pitch_last_tic = pitch;
        tic = true;
        odo_wheel = !odo_wheel;
        forward_backward = 0;
    }

    // Si on a commencé par reculer
    if( forward_backward == -1 and
        (pitch - pitch_last_tic <= -angle_tic or pitch - pitch_last_tic >= 0) )
    {
        pitch_last_tic = pitch;
        tic = true;
        odo_wheel = !odo_wheel;
        forward_backward = 0;
    }

    return tic;
}