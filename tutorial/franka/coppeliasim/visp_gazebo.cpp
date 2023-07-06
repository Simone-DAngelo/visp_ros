/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2021 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *****************************************************************************/

//! \example visp-gazebo-apriltag.cpp

#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#include <visp_ros/vpROSGrabber.h>

using namespace std;

void
display_point_trajectory( const vpImage< unsigned char > &I, const std::vector< vpImagePoint > &vip,
                          std::vector< vpImagePoint > *traj_vip )
{
  for ( size_t i = 0; i < vip.size(); i++ )
  {
    if ( traj_vip[i].size() )
    {
      // Add the point only if distance with the previous > 1 pixel
      if ( vpImagePoint::distance( vip[i], traj_vip[i].back() ) > 1. )
      {
        traj_vip[i].push_back( vip[i] );
      }
    }
    else
    {
      traj_vip[i].push_back( vip[i] );
    }
  }
  for ( size_t i = 0; i < vip.size(); i++ )
  {
    for ( size_t j = 1; j < traj_vip[i].size(); j++ )
    {
      vpDisplay::displayLine( I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2 );
    }
  }
}

int
main( int argc, char **argv )
{
  double opt_tagSize             = 0.1;
  bool display_tag               = true;
  int opt_quad_decimate          = 2;
  bool opt_verbose               = false;
  bool opt_plot                  = false;
  bool opt_adaptive_gain         = false;
  bool opt_task_sequencing       = false;
  double convergence_threshold_t = 0.0005, convergence_threshold_tu = vpMath::rad( 0.5 );
  bool opt_coppeliasim_sync_mode = false;

  for ( int i = 1; i < argc; i++ )
  {
    if ( std::string( argv[i] ) == "--tag_size" && i + 1 < argc )
    {
      opt_tagSize = std::stod( argv[i + 1] );
    }
    else if ( std::string( argv[i] ) == "--verbose" || std::string( argv[i] ) == "-v" )
    {
      opt_verbose = true;
    }
    else if ( std::string( argv[i] ) == "--plot" )
    {
      opt_plot = true;
    }
    else if ( std::string( argv[i] ) == "--adaptive_gain" )
    {
      opt_adaptive_gain = true;
    }
    else if ( std::string( argv[i] ) == "--task_sequencing" )
    {
      opt_task_sequencing = true;
    }
    else if ( std::string( argv[i] ) == "--quad_decimate" && i + 1 < argc )
    {
      opt_quad_decimate = std::stoi( argv[i + 1] );
    }
    else if ( std::string( argv[i] ) == "--no-convergence-threshold" )
    {
      convergence_threshold_t  = 0.;
      convergence_threshold_tu = 0.;
    }
    else if ( std::string( argv[i] ) == "--enable-coppeliasim-sync-mode" )
    {
      opt_coppeliasim_sync_mode = true;
    }
    else if ( std::string( argv[i] ) == "--help" || std::string( argv[i] ) == "-h" )
    {
      std::cout << argv[0] << "[--tag_size <marker size in meter; default " << opt_tagSize << ">] "
                << "[--quad_decimate <decimation; default " << opt_quad_decimate << ">] "
                << "[--adaptive_gain] "
                << "[--plot] "
                << "[--task_sequencing] "
                << "[--no-convergence-threshold] "
                << "[--enable-coppeliasim-sync-mode] "
                << "[--verbose] [-v] "
                << "[--help] [-h]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  try
  {
    // ROS node
    ros::init( argc, argv, "gazebo_visp_ros" );
    ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
    ros::Rate loop_rate( 1000 );
    ros::spinOnce();

    vpImage< unsigned char > I;
    vpROSGrabber g;
    g.setImageTopic( "/camera/image_raw" );
    g.setCameraInfoTopic( "/camera/camera_info" );
    g.open( argc, argv );

    g.acquire( I );

    std::cout << "Image size: " << I.getWidth() << " x " << I.getHeight() << std::endl;
    vpCameraParameters cam;

    g.getCameraInfo( cam );
    std::cout << cam << std::endl; //show camera param
    vpDisplayOpenCV dc( I, 10, 10, "Color image" );

    // tag detector param 
    vpDetectorAprilTag::vpAprilTagFamily tagFamily                  = vpDetectorAprilTag::TAG_36h11;
    vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    vpDetectorAprilTag detector( tagFamily );
    detector.setAprilTagPoseEstimationMethod( poseEstimationMethod );
    detector.setDisplayTag( display_tag );
    detector.setAprilTagQuadDecimate( opt_quad_decimate );

    // Servo
    vpHomogeneousMatrix cdMc, cMo, oMo;

    // Desired pose used to compute the desired features
    vpHomogeneousMatrix cdMo( vpTranslationVector( 0, 0.0, opt_tagSize * 3 ),
                              vpRotationMatrix( { 1, 0, 0, 0, -1, 0, 0, 0, -1 } ) );
    cdMc = cdMo * cMo.inverse();

    // Create visual features
    vpFeatureTranslation t( vpFeatureTranslation::cdMc );
    vpFeatureThetaU tu( vpFeatureThetaU::cdRc );
    t.buildFrom( cdMc );
    tu.buildFrom( cdMc );

    vpFeatureTranslation td( vpFeatureTranslation::cdMc );
    vpFeatureThetaU tud( vpFeatureThetaU::cdRc );

    bool final_quit                           = false;
    bool has_converged                        = false;
    bool send_velocities                      = false;
    bool servo_started                        = false;
    std::vector< vpImagePoint > *traj_corners = nullptr; // To memorize point trajectory

    while ( !final_quit )
    {
      g.acquire( I );
      vpDisplay::display( I );

      std::vector< vpHomogeneousMatrix > cMo_vec; //posa del tag stimata
      detector.detect( I, opt_tagSize, cam, cMo_vec );
     
      {
        std::stringstream ss;
        ss << "Left click to " << ( send_velocities ? "stop the robot" : "servo the robot" )
           << ", right click to quit.";
        vpDisplay::displayText( I, 20, 20, ss.str(), vpColor::red );
      }

      // Only one tag is detected
      if ( cMo_vec.size() == 1 )
      {
        cMo = cMo_vec[0];

        static bool first_time = true;
        if ( first_time )
        {
          // Introduce security wrt tag positionning in order to avoid PI rotation
          std::vector< vpHomogeneousMatrix > v_oMo( 2 ), v_cdMc( 2 );
          v_oMo[1].buildFrom( 0, 0, 0, 0, 0, M_PI );
          // v_oMo[1].buildFrom( 0, 0, 0, 0, 0, 0.0 );

          for ( size_t i = 0; i < 2; i++ )
          {
            v_cdMc[i] = cdMo * v_oMo[i] * cMo.inverse();
          }
          if ( std::fabs( v_cdMc[0].getThetaUVector().getTheta() ) <
               std::fabs( v_cdMc[1].getThetaUVector().getTheta() ) )
          {
            oMo = v_oMo[0];
          }
          else
          {
            std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
            oMo = v_oMo[1]; // Introduce PI rotation
          }
        } // end first_time

        // Update visual features
        cdMc = cdMo * oMo * cMo.inverse();
        t.buildFrom( cdMc );
        tu.buildFrom( cdMc );

        // Display the current and desired feature points in the image display
        // Display desired and current pose features
        vpDisplay::displayFrame( I, cdMo * oMo, cam, opt_tagSize / 1.5, vpColor::yellow, 2 );
        vpDisplay::displayFrame( I, cMo, cam, opt_tagSize / 2, vpColor::none, 3 );

        // Get tag corners
        std::vector< vpImagePoint > corners = detector.getPolygon( 0 );

        // Get the tag cog corresponding to the projection of the tag frame in the image
        corners.push_back( detector.getCog( 0 ) );
        // Display the trajectory of the points
        if ( first_time )
        {
          traj_corners = new std::vector< vpImagePoint >[corners.size()];
        }

        // Display the trajectory of the points used as features
        display_point_trajectory( I, corners, traj_corners );

        vpTranslationVector cd_t_c = cdMc.getTranslationVector();
        vpThetaUVector cd_tu_c     = cdMc.getThetaUVector();
        double error_tr            = sqrt( cd_t_c.sumSquare() );
        double error_tu            = vpMath::deg( sqrt( cd_tu_c.sumSquare() ) );

        // std::stringstream ss;
        // ss << "error_t: " << error_tr;
        // vpDisplay::displayText( I, 20, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );
        // ss.str( "" );
        // ss << "error_tu: " << error_tu;
        // vpDisplay::displayText( I, 40, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );

        if ( opt_verbose )
          std::cout << "error translation: " << error_tr << " ; error rotation: " << error_tu << std::endl;

        if ( !has_converged && error_tr < convergence_threshold_t && error_tu < convergence_threshold_tu )
        {
          has_converged = true;
          std::cout << "Servo task has converged"
                    << "\n";
          vpDisplay::displayText( I, 100, 20, "Servo task has converged", vpColor::red );
        }

        if ( first_time )
        {
          first_time = false;
        }
      } 
    
      //--mappa il click del mouse per iniziare/fermare il controllo
      vpMouseButton::vpMouseButtonType button;
      if ( vpDisplay::getClick( I, button, false ) )
      {
        switch ( button )
        {
        case vpMouseButton::button1:
          send_velocities = !send_velocities;
          break;

        case vpMouseButton::button3:
          final_quit = true;
          break;

        default:
          break;
        }
      }
      //--
      
      vpDisplay::flush( I );
    }                                // end while

    if ( !final_quit )
    {
      while ( !final_quit )
      {
        g.acquire( I );
        vpDisplay::display( I );

        vpDisplay::displayText( I, 20, 20, "Click to quit the program.", vpColor::red );
        vpDisplay::displayText( I, 40, 20, "Visual servo converged.", vpColor::red );

        if ( vpDisplay::getClick( I, false ) )
        {
          final_quit = true;
        }

        vpDisplay::flush( I );
      }
    }
    if ( traj_corners )
    {
      delete[] traj_corners;
    }
  }
  catch ( const vpException &e )
  {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
