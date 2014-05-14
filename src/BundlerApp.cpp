/*============================= BundlerApp =============================*/
/*
   Copyright (c) 2008-2010  Noah Snavely (snavely (at) cs.cornell.edu)
     and the University of Washington
 
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.
 
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

*/
/*============================= BundlerApp =============================*/
/*
   Revised coding to suit my particular style and to slowly include
   changes that will allow me to repurpose a good portion of the
   code towards other uses.  The repurposed version will be available 
   in github under same license as original.  The goal is to repurpose
   enough to work in Matlab.

   Author:      Noah Snavely
   Modified:    Patricio A. Vela,       pvela@gatech.edu

   Date:        2014/05/12
*/
/*============================= BundlerApp =============================*/

#include <assert.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <time.h>

#ifdef WIN32
#  include "getopt.h"
#  include <conio.h>
#else
#  include <getopt.h>
#endif

#include "BundlerApp.h"

#include "Epipolar.h"
#include "Register.h"
#include "SifterUtil.h"

#ifndef __USE_ANN__
#include "BruteForceSearch.h"
#endif

#include "defines.h"
#include "fit.h"
#include "fmatrix.h"
#include "image.h"
#include "matrix.h"
#include "qsort.h"
#include "util.h"

// #define WRITE_XML    // TODO: What purpose does this serve.  Delete if never needed. PAV 2014/05/11
#define SUBSAMPLE_LEVEL 1
#define MIN_MATCHES 10




extern int optind;


/*---------------------------- BundlerApp ----------------------------*/
/*
   Constructor for the bundler app.  A good portion of the constructor
   is spent setting member variables to default values.
*/
BundlerApp::BundlerApp()
{
/* Set initial values */
m_bundle_version = 0.1;     
//TODO: Seems to be version 0.1?  So later versions are not publicized?
//TODO: Could they be what photosynth is, and actually improved over this?
//TODO: If so, then what value is there in getting Bundler running?
//NOTE: Downloaded v0.4 and it seems to be labelled as v0.1 like this one.
//NOTE: Possible that changing it changes its execution.  There are if
//NOTE: checks all over the map for version.
//TODO: Compare with v0.3 to see how that one is.

m_fisheye = false;
m_fixed_focal_length = true;
m_estimate_distortion = false;
m_construct_max_connectivity = false;
m_bundle_provided = false;
m_analyze_matches = false;

m_estimate_ignored = false;

m_use_constraints = false;
m_constrain_focal = false;
m_constrain_focal_weight = 100.0;
m_distortion_weight = 1.0e2;
    
m_use_point_constraints = false;
m_point_constraint_weight = 0.0;
m_point_constraints = NULL;
m_point_constraint_file = NULL;

m_only_bundle_init_focal = false;
m_init_focal_length = 532.0;
m_initial_pair[0] = -1;
m_initial_pair[1] = -1;

m_panorama_mode = false;
m_homography_threshold = 6.0;
m_homography_rounds = 256;
m_fmatrix_threshold = 9.0;
m_fmatrix_rounds = 2048;
m_skip_fmatrix = false;
m_skip_homographies = false;
m_projection_estimation_threshold = 4.0; // 1.8;
m_min_proj_error_threshold = 8.0;
m_max_proj_error_threshold = 16.0;
m_min_camera_distance_ratio = 0.0;
m_baseline_threshold = -1.0;
m_optimize_for_fisheye = false;
m_use_focal_estimate = false;
m_trust_focal_estimate = false;
m_factor_essential = true;
m_up_image = -1;
// m_start_camera = -1;
m_min_track_views = 2;
m_max_track_views = 100000;
m_min_num_feat_matches = 16;
m_min_max_matches = 16;
m_ray_angle_threshold = 2.0;

m_keypoint_border_width = 0;
m_keypoint_border_bottom = 0;

m_fisheye_params = NULL;
m_bundle_output_file = m_bundle_output_base = NULL;
m_bundle_file = NULL;
m_intrinsics_file = NULL;
m_match_directory = ".";
m_match_index_dir = NULL;
m_match_table = NULL;
m_key_directory = ".";
m_image_directory = ".";
m_output_directory = ".";
// m_sift_binary = SIFT_COMMAND;
m_use_intrinsics = false;
    
// m_matches = NULL;
// m_match_lists = NULL;
m_matches_computed = false;
m_match_global = false;
m_ann_max_pts_visit = 400;
m_global_nn_sigma = 16.0;
m_global_knn = 200;
    
// m_transforms = NULL;
// m_set_transforms = NULL;
// m_images_per_set = 0;

m_matches_loaded = false;
m_features_coalesced = false;

m_assemble = false;
m_server_mode = false;
m_server_port = -1;
m_run_bundle = false;
m_rerun_bundle = false;
m_fast_bundle = true;
m_skip_full_bundle = false;
m_skip_add_points = false;
m_use_angular_score = false;

m_compress_list = false;
m_reposition_scene = false;
m_prune_bad_points = false;
m_predict_next_image = false;
m_prediction_image = NULL;
m_scale_focal = 1.0;
m_scale_focal_file = NULL;
m_rotate_cameras_file = NULL;
m_output_relposes = false;
m_output_relposes_file = NULL;

m_sky_model_file = NULL;

m_compute_covariance = false;
m_covariance_fix1 = -1;
m_covariance_fix2 = -1;

m_track_file = NULL;
m_zero_distortion_params = false;
m_enrich_points = false;
m_fix_necker = false;

m_ignore_file = NULL;
m_add_image_file = NULL;
m_add_images_fast = false;

m_scale = 1.0;
// matrix_ident(3, m_repos_R);
m_repos_R[0] = 1.0; m_repos_R[1] = 0.0; m_repos_R[2] = 0.0;
m_repos_R[3] = 0.0; m_repos_R[4] = 1.0; m_repos_R[5] = 0.0;
m_repos_R[6] = 0.0; m_repos_R[7] = 0.0; m_repos_R[8] = 1.0;

m_repos_d[0] = m_repos_d[1] = m_repos_d[2] = 0.0;
m_repos_scale = 1.0;

m_metric = false;

m_estimate_up_vector_szeliski = false;

// bool load_file = false;
}


/*---------------------------- PrintUsage ----------------------------*/
/*
   Prints the flags and overall execution usage of the bundler
   executable.  You can read this to see what the options are.
*/

void PrintUsage() 
{
printf("Usage:  bundler <input.txt> [options]\n"
   "  Options:\n"
   "\n"
   "  [Action options]\n"
   "     --run_bundle\n"
   "        Run structure from motion (usually what you want to do)\n"
   "     --rerun_bundle\n"
   "        Reoptimize a reconstruction specified by the --bundle option\n"
   "     --compress_list\n"
   "        Create a 'compressed' list and bundle file, removing images\n"
   "        that weren't reconstructed.  The list and bundle files are \n"
   "        written to list.compressed.txt and bundle.compressed.out\n"
   "\n"
   "  [Bundle adjustment options]\n"
   "    [Focal length options]\n"
   "     --init_focal_length <f>\n"
   "        Set initial focal length to <f>\n"
   "     --variable_focal_length\n"
   "        Allow focal lengths to vary for each image\n"
   "     --fixed_focal_length\n"
   "        Fix the focal length for all cameras\n"
   "        (set to init_focal_length)\n"
   "     --use_focal_estimate\n"
   "        Initialize using focal length estimates specified\n"
   "        in the list file\n"
   "     --trust_focal_estimate\n"
   "        Trust the provided focal length estimates (i.e.,\n"
   "        don't attempt to cross-check with self-calibration)\n"
   "     --constrain_focal\n"
   "        Add a soft constraint on focal lengths to stay near\n"
   "        their estimated values\n"
   "     --constrain_focal_weight <weight>\n"
   "        Strength of the focal length constraints.\n"
   "        Default is 0.0001.\n"
   "\n"
   "  [Other bundle adjustment options]\n"
   "     --fisheye <paramfile>\n"
   "        Read fisheye parameters from given file\n"
   "     --init_pair1 <img1>\n"
   "     --init_pair2 <img2>\n"
   "        Indices of the images with which to seed bundle adjustment\n"
   "     --estimate_distortion\n"
   "        Estimate radial distortion parameters (2 coefficients)\n"
   "     --ray_angle_threshold <degrees>\n"
   "        Don't triangulate points whose rays have an angle less\n"
   "        than <degrees>.  Default is 2 degrees.\n"
   "     --projection_estimation_threshold <thres>\n"
   "        Use a RANSAC threshold of <thres> when doing\n"
   "        pose estimation to add in a new image.  Default is 4.\n"
   "     --min_proj_error_threshold <min>\n"
   "     --max_proj_error_threshold <max>\n"
   "        The minimum and maximum values of the adaptive outlier\n"
   "        threshold.  Defaults are 8 and 16.\n"
   "     --bundle <file>\n"
   "        Read previous bundle adjustment results from <file>\n"
   "     --ignore_file <file>\n"
   "        Don't try to register any image whose index appears in <file>\n"
   "     --slow_bundle\n"
   "        Run slow version of bundle adjustment (adds an image at a time)\n"
   "\n"
   "  [Output options]\n"
   "     --output <file>\n"
   "       Save bundle adjustment output to <file>\n"
   "     --output_all <base>\n"
   "       Save intermediate bundle adjustment results\n"
   "     --output_dir\n"
   "       Specifies the directory in which to save output files\n"
   "\n"
   "  [Other options]\n"
   "     --options_file <file>\n"
   "       Read options from <file>.\n"
   "     --match_dir <dir>\n"
   "       Specifies the directory where the match-*-*.txt files are stored.\n"
   "     --help\n"
   "       Print this message\n\n");
}


/*----------------------- ReadFisheyeParameters ----------------------*/
/*
   Appears to read fisheye camera/lens parameters from a file and
   returns then as a series of parameter variables.  Expects the
   parameters to be specified by a particular set of strings followed
   by the parameters themselves (all elements/terms separated by 
   spaces).
*/
static void ReadFisheyeParameters(char *filename, double &fCx, double &fCy, 
                                  double &fRad, double &fAngle, double &fFocal)
{

//  Clear parameter values.
fCx = 0.0;
fCy = 0.0;

/* Read fisheye params */
FILE *f = fopen(filename, "r");
if (f == NULL) 
 {
  printf("Error opening fisheye parameters file %s for reading\n", filename);
  exit(1);
 }

char buf[256];
while (fgets(buf, 256, f) != NULL) /* Split the command into tokens */
 {
  std::string str(buf);
  std::vector<std::string> toks;

  Tokenize(str, toks, " ");

  if (strcmp(toks[0].c_str(), "FisheyeCenter:") == 0) 
   {
    if (toks.size() != 3) 
     {
	  printf("FisheyeCenter needs two arguments!\n");
	  exit(1);
     } 
    else 
     {
	  fCx = atof(toks[1].c_str());
	  fCy = atof(toks[2].c_str());
     }
   } 
  else if (strcmp(toks[0].c_str(), "FisheyeRadius:") == 0) 
   {
    if (toks.size() != 2) 
     {
	  printf("FisheyeRadius needs one argument!\n");
	  exit(1);
     } 
    else 
     {
	  fRad = atof(toks[1].c_str());
     }
   } 
  else if (strcmp(toks[0].c_str(), "FisheyeAngle:") == 0) 
   {
    if (toks.size() != 2) 
     {
	  printf("FisheyeAngle needs one argument!\n");
	  exit(1);
     } 
    else 
     {
	  fAngle = atof(toks[1].c_str());
     }	    
   } 
  else if (strcmp(toks[0].c_str(), "FisheyeFocal:") == 0) 
   {
    if (toks.size() != 2) 
     {
	  printf("FisheyeFocal needs one argument!\n");
	  exit(1);
     } 
    else 
     {
	  fFocal = atof(toks[1].c_str());
     }
   } 
  else 
   {
    printf("Unrecognized fisheye field %s\n", toks[0].c_str());
   }
 }

fclose(f);

}

/*-------------------------- ProcessOptions --------------------------*/
/*
   Member function of the BundlerApp class that processes the bundler
   options.
*/
void BundlerApp::ProcessOptions(int argc, char **argv) 
{
/* Read options */
while (1) 
 {
  static struct option long_options[] = 
   {
    {"fisheye",      1, 0, 'f'},
    {"intrinsics",   1, 0, 357}, 
    
    {"init_pair1",   1, 0, 'p'},
    {"init_pair2",   1, 0, 'q'},
    {"output",       1, 0, 'o'},
    {"output_all",   1, 0, 'a'},
    {"init_focal_length",  1, 0, 'i'},
    {"variable_focal_length", 0, 0, 'v'},
    {"fixed_focal_length", 0, 0, 'x'},
    {"run_bundle",   0, 0, 'r'},
    {"rerun_bundle", 0, 0, 'j'},
    {"slow_bundle",  0, 0, 'D'},
    {"skip_full_bundle", 0, 0, 321},
    {"skip_add_points", 0, 0, 322},
    
    {"compress_list", 0, 0, '4'},
    {"scale_focal", 1, 0, 305},
    {"scale_focal_file", 1, 0, 307},
    
    {"write_tracks", 1, 0, 311},
    {"rotate_cameras", 1, 0, 309},
    {"zero_distortion_params",0, 0, 363},
    {"enrich_points", 0, 0, 361},
    {"output_relposes", 1, 0, 368},
    
    {"compute_color_statistics", 0, 0, 323},
    {"detect_duplicates", 0, 0, 352},
    {"bundle_from_tracks", 0, 0, 353},
    {"bundle_from_points", 0, 0, 354},
    {"stretch_factor", 1, 0, 356},
    
    {"classify_photos", 0, 0, 324},
    {"compare_histograms", 0, 0, 334},
    {"panorama_mode", 0, 0, 332},
    {"use_fit_plane", 0, 0, 'l'},
    {"day_photos", 1, 0, 325},
    {"night_photos", 1, 0, 326},
    {"cloudy_photos", 1, 0, 327},
    {"projective_cameras", 1, 0, 315},
    {"projective_points", 1, 0, 316},
    {"prune_bad_points", 0, 0, 351},
    
    {"compute_covariance", 0, 0, 340},
    {"covariance_fix1", 1, 0, 341},
    {"covariance_fix2", 1, 0, 342},
    
    {"ignore_file",  1, 0, 'L'},
    {"use_focal_estimate", 0, 0, 'U'},
    {"trust_focal_estimate", 0, 0, '_'},
    {"estimate_ignored", 0, 0, 344},
    {"reposition_scene", 0, 0, 'R'},
    {"segment_sky", 1, 0, 350},
    {"keypoint_border_width", 1, 0, 355},
    {"keypoint_border_bottom", 1, 0, 365},
    
    {"min_track_views", 1, 0, 'V'},
    {"max_track_views", 1, 0, 320},
    {"min_max_matches", 1, 0, 362},
    {"min_feature_matches", 1, 0, 369},
    
    {"ray_angle_threshold", 1, 0, 'N'},
    {"estimate_distortion", 0, 0, 347},
    {"distortion_weight", 1, 0, 348},
    {"construct_max_connectivity", 0, 0, '*'},
    
    {"homography_threshold", 1, 0, 'H'},
    {"homography_rounds",    1, 0, 345},
    {"fmatrix_threshold",    1, 0, 'F'},
    {"fmatrix_rounds",       1, 0, 'S'},
    {"skip_fmatrix",         0, 0, 306},
    {"skip_homographies",         0, 0, 319},
    {"projection_estimation_threshold", 1, 0, 'P'},
    {"min_proj_error_threshold", 1, 0, 317},
    {"max_proj_error_threshold", 1, 0, 318},
    {"use_angular_score", 0, 0, 349},
         
    {"up_image",             1, 0, 'E'},
    {"estimate_up_vector_szeliski", 0, 0, 'Z'},
    {"min_camera_distance_ratio", 1, 0, 'C'},
        // {"baseline_threshold",   1, 0, 'B'},
    {"optimize_for_fisheye",  0, 0, 'B'},
    
    {"assemble",     0, 0, 308},
    {"bundle",       1, 0, 'b'},
    {"server_mode",  1, 0, 346},
    {"match_dir",    1, 0, 'm'},
    {"match_index_dir", 1, 0, 366},
    {"match_table",  1, 0, 364},
    {"image_dir",    1, 0, 300},
    {"key_dir",      1, 0, 301},
    
    {"analyze_matches", 0, 0, 'M'},
    {"match_global", 0, 0, '<'},
    {"ann_max_pts_visit", 1, 0, 302},
    {"global_knn", 1, 0, 303},
    {"global_nn_sigma", 1, 0, 304},
    
    {"output_dir",   1, 0, 'u'},
    {"use_constraints", 0, 0, '=' },
    {"constrain_focal", 0, 0, '$'},
    {"constrain_focal_weight", 1, 0, 'J'},
    {"only_bundle_init_focal", 0, 0, 'y'},
    {"no_factor_essential", 0, 0, 339},
    
    {"point_constraint_file", 1, 0, 'Y'},
    {"point_constraint_weight", 1, 0, 'w'},
    
    {"fix_necker",   0, 0, 'n'},
    {"sift_binary",  1, 0, 's'},
    {"options_file", 1, 0, 'z'},
    {"add_images",   1, 0, '@'},
    
    {"scale",        1, 0, '9'},
    {"morph_steps",  1, 0, '0'},
    {"image_rescale", 1, 0, '+'},
    
    {"help",         0, 0, 'h'},
    
    {0, 0, 0, 0}
   };

  int option_index;
  int c = getopt_long(argc, argv, "f:do:a:i:x", long_options, &option_index);

  if (c == -1)
    break;
	
  switch (c) 
   {
    case 'h':
      /* Print usage */
      PrintUsage();
      exit(0);
      break;
    case 'f':
      m_fisheye = true;
      printf("Using fisheye lens, param file: %s\n", optarg);
      m_fisheye_params = strdup(optarg);
      break;
    case 357:
      m_use_intrinsics = true;
      m_intrinsics_file = strdup(optarg);
      break;

    case 'p':
      m_initial_pair[0] = atoi(optarg);
      break;
    case 'q':
      m_initial_pair[1] = atoi(optarg);
      break;
    case 347:
      m_estimate_distortion = true;
      break;
    case 348:
      m_distortion_weight = atof(optarg);
      break;
    case 349:
      m_use_angular_score = true;
      break;
    case '*':
      m_construct_max_connectivity = true;
      break;
    case 'o':
      m_bundle_output_file = strdup(optarg);
      break;
    case 'a':
      m_bundle_output_base = strdup(optarg);
      break;
    case 'i':
      m_init_focal_length = atof(optarg);
      break;
    case 'v':
      m_fixed_focal_length = false;
      break;
    case 'x':
      m_fixed_focal_length = true;
      break;
    case 'r':
      m_run_bundle = true;
      break;
    case 'j':
      m_rerun_bundle = true;
      break;
    case 'D':
      m_fast_bundle = false;
      break;
    case 321:
      m_skip_full_bundle = true;
      break;
    case 322:
      m_skip_add_points = true;
      break;
    
    case '4':
      m_compress_list = true;
      break;
    case 'R':
      m_reposition_scene = true;
      break;
    case 351:
      m_prune_bad_points = true;
      break;
    
    case 368:
      m_output_relposes = true;
      m_output_relposes_file = strdup(optarg);
      break;
    case 350:
      m_segment_sky = true;
      m_sky_model_file = strdup(optarg);
      break;
    case 305:
      m_scale_focal = atof(optarg);
      break;
    case 307:
      m_scale_focal_file = strdup(optarg);
      break;

    case 311:
      m_track_file = strdup(optarg);
      break;
    case 309:
      m_rotate_cameras_file = strdup(optarg);
      break;
    
    case 340:
      m_compute_covariance = true;
      break;
    case 341:
      m_covariance_fix1 = atoi(optarg);
      break;
    case 342:
      m_covariance_fix2 = atoi(optarg);
      break;
    
#if 0
    case 352:
      m_detect_duplicates = true;
      break;
    case 353:
      m_bundle_from_tracks = true;
      break;
    case 354:
      m_bundle_from_points = true;
      m_bundle_from_tracks = true;
      break;
    case 356:
      m_stretch_factor = atof(optarg);
      break;
#endif
    
    case 344:
      m_estimate_ignored = true;
      break;
    case 355:
      m_keypoint_border_width = atoi(optarg);
      break;
    case 365:
      m_keypoint_border_bottom = atoi(optarg);
      break;
    
    case 363:
      m_zero_distortion_params = true;
      break;
    case 361:
      m_enrich_points = true;
      break;
    
    case 'L':
      m_ignore_file = strdup(optarg);
      break;
    case '@':
      m_add_image_file = strdup(optarg);
      break;
    case 'U':
      m_use_focal_estimate = true;
      break;
    case '_':
      m_trust_focal_estimate = true;
      break;
    
    case 'H':
      m_homography_threshold = atof(optarg);
      break;
    case 345:
      m_homography_rounds = atoi(optarg);
      break;
    case 'F':
      m_fmatrix_threshold = atof(optarg);
      break;
    case 'S':
      m_fmatrix_rounds = atoi(optarg);
      break;
    case 306:
      m_skip_fmatrix = true;
      break;
    case 319:
      m_skip_homographies = true;
      break;
    
    case 'P':
      m_projection_estimation_threshold = atof(optarg);
      break;
    case 317:
      m_min_proj_error_threshold = atof(optarg);
      break;
    case 318:
      m_max_proj_error_threshold = atof(optarg);
      break;
    case 'C':
      m_min_camera_distance_ratio = atof(optarg);
      break;
    case 'B':
      // m_baseline_threshold = atof(optarg);
      m_optimize_for_fisheye = true;
      break;
    case 'E':
      m_up_image = atoi(optarg);
      break;
    
#if 0
    case 'T':
      m_start_camera = atoi(optarg);
      break;
#endif
    
    case 'V':
      m_min_track_views = atoi(optarg);
      break;
    case 320:
      m_max_track_views = atoi(optarg);
      break;
    case 362:
      m_min_max_matches = atoi(optarg);
      break;
    case 369:
      m_min_num_feat_matches = atoi(optarg);
      break;
    
    case 'N':
      m_ray_angle_threshold = atof(optarg);
      break;
    
    case 308: /* assemble */
      m_assemble = true;
      break;
    
    case 346:
      m_server_mode = true;
      m_server_port = atoi(optarg);
      break;
    case 'b':
      m_bundle_provided = true;
      m_bundle_file = strdup(optarg);
      break;
    case 'm':
      m_match_directory = strdup(optarg);
      break;
    case 366:
      m_match_index_dir = strdup(optarg);
      break;
    case 364:
      m_match_table = strdup(optarg);
      break;
    case 300:
      m_image_directory = strdup(optarg);
      break;
    case 301:
      m_key_directory = strdup(optarg);
      break;
    case 'M':
      m_analyze_matches = true;
      break;
    case 332:
      m_panorama_mode = true;
      break;
    case 339:
      m_factor_essential = false;
      break;
    case '<':
      m_match_global = true;
      break;
    
    case 302:
      m_ann_max_pts_visit = atoi(optarg);
      printf("  ann_max_pts_visit: %d\n", m_ann_max_pts_visit);
      break;
    case 303:
      m_global_knn = atoi(optarg);
      printf("  global_knn: %d\n", m_global_knn);
      break;
    case 304:
      m_global_nn_sigma = atof(optarg);
      printf("  global_nn_sigma: %0.3f\n", m_global_nn_sigma);
      break;
    
    case 'u':
      m_output_directory = strdup(optarg);
      break;
    case '=':
      m_use_constraints = true;
      break;
    case '$':
      m_constrain_focal = true;
      break;
    case 'J':
      m_constrain_focal_weight = atof(optarg);
      break;
    case 'y':
      m_only_bundle_init_focal = true;
      break;
    case 'Y':
      m_point_constraint_file = strdup(optarg);
      break;
    case 'w':
      m_point_constraint_weight = atof(optarg);
      break;
    case 'n':
      m_fix_necker = true;
      break;
    case 's':
      m_sift_binary = strdup(optarg);
      break;
    case 'Z':
      m_estimate_up_vector_szeliski = true;
      break;
    case '9':
      m_scale = atof(optarg);
      break;
    case 'W':
      m_metric = true;
      break;
    
    case 'z': 
     {
      int optind_curr = optind;
      optind = 1;
    
      /* Load options from file */
      std::vector<std::string> tokens;
      char *filename = optarg;
      FILE *f = fopen(optarg, "r");
      if (f == NULL) 
       {
        printf("Error reading options file %s\n", filename);
        exit(1);
       }
    
      char *opt_str = new char[4096];
      size_t numread;
      numread = fread(opt_str, 1, 4096, f);
      fclose(f);
    
      std::string str(opt_str);
      // wxStringTokenizer t(str, wxT(" \n"));
      Tokenize(str, tokens, " \n");
    
      #if 0
      while (t.HasMoreTokens()) 
       {
        wxString tok = t.GetNextToken();
        tokens.push_back(tok);
       }
      #endif
    
      int argc_new = (int) tokens.size() + 1;
      char **argv_new = new char * [argc_new];
    	
      argv_new[0] = strdup("test");
      for (int i = 1; i < argc_new; i++) 
       {
        argv_new[i] = strdup(tokens[i-1].c_str());
       }
    
      ProcessOptions(argc_new, argv_new);
    	
      for (int i = 0; i < argc_new; i++) 
       {
        free(argv_new[i]);
       }
    	
      delete [] argv_new;
      delete [] opt_str;
    
      optind = optind_curr;
    
      #ifdef WIN32
      return;
      #else
      break;
      #endif
     }
    
    default:
      printf("Unrecognized option %d\n", c);
      break;
   }
 }
}


#ifdef WIN32
static void sifter_die() 
{
getch();
}
#endif

/*------------------------------ OnInit ------------------------------*/
/*
    This is the main execution thread of bundler.  It takes the 
    processed arguments and runs the optimization described in the
    paper.

    TODO: OnInit seems to be a pretty bad name.  Shouldn't it be
    TODO:  called genBundle or something that more readily indicates
    TODO:  what is going on?
    TODO: It seems to be the catchall/doall member function with multiple
    TODO:  outcomes depending on chosen flags.  Need to identify what
    TODO:  is the baseline execution for a first time bundle run.
    TODO: That code should probably be in its own member function.
    TODO: Should then slowly deconstruct this particular function and
    TODO:  break out into a few.
*/
bool BundlerApp::OnInit()
{

char *imageList;
bool load_file = false;

//--[1] Process the command line options.
printf("[BundlerApp::OnInit] Running program %s\n", argv[0]);

if (argc >= 2) 
 {
  printf("[BundlerApp::OnInit] Loading images from file '%s'\n", argv[1]);
  imageList = argv[1];
  load_file = true;
 } 
else 
 {
  PrintUsage();
  exit(0);
 }

printf("[BundlerApp::OnInit] Processing options...\n");
ProcessOptions(argc - 1, argv + 1);

if (m_use_intrinsics && m_estimate_distortion) 
 {
  printf("Error: --intrinsics and --estimate_distortion are incompatible\n");
  exit(1);
 }

if (m_fixed_focal_length && m_estimate_distortion) 
 {
  printf("Error: --fixed_focal_length and --estimate_distortion "
         "are currently incompatible\n");
  exit(1);
 }

//--[X] Code appears to be for instantiating a window interface.
//        It is not used here as the execution has only text output.

//printf("[BundlerApp::OnInit] Loading frame...\n");

//TODO: More commented code.  Delete/move to patch? PAV 2014/05/11
// bool use_window = false;
// bool use_window = true;

/* Initialize the image handlers */
// wxImage::AddHandler(new wxPGMHandler());

#if 0
wxString str = wxT("wxWidgets OpenGL Cube Sample");
wxFrame *frame2 = new wxFrame(NULL, wxID_ANY, str, 
                              wxDefaultPosition, wxSize(400, 300));
printf("tested frame\n");
#endif

//--[X] Output indicates thatimages are loaded, but here it appears that
//        only one image is loaded, the first, unless the fisheye flag
//        is set.
//        Or, as it seems, the list of images is loaded from a file
//        for true image loading at a later time.
//        The text output is misleading.
printf("[BundlerApp::OnInit] Loading images...\n");
fflush(stdout);
if (load_file) 
 {
  FILE *f = fopen(imageList, "r");
  if (f == NULL) 
   {
    printf("[BundlerApp::OnInit] Error opening file %s for reading\n", 
           imageList);
    exit(1);
   }

  LoadImageNamesFromFile(f);

  int num_images = GetNumImages();

  if (m_fisheye) 
   {
    double fCx = 0.0, fCy = 0.0, fRad = 0.0, fAngle = 0.0, fFocal = 0.0;
    ReadFisheyeParameters(m_fisheye_params, fCx, fCy, fRad, fAngle, fFocal);
 
    for (int i = 0; i < num_images; i++) 
     {
      if (m_image_data[i].m_fisheye) 
       {
        m_image_data[i].m_fCx = fCx;
        m_image_data[i].m_fCy = fCy;
        m_image_data[i].m_fRad = fRad;
        m_image_data[i].m_fAngle = fAngle;
        m_image_data[i].m_fFocal = fFocal;
       }                
     }
   }
  fclose(f);
 }

#if 0  //TODO: If this part is not to be run, then why bother?
if (input_model == MODEL_PANORAMA) 
 {
  printf("[BundlerApp::OnInit] Computing homographies\n");
  ComputeTransforms(true);
  MakeMatchListsSymmetric();
  DumpCorrespondenceImages();
 } 
else if (input_model == MODEL_OBJECT_MOVIE) 
 {
  printf("[BundlerApp::OnInit] Computing epipolar geometry\n");

  /* Compute homographies between images */
  #ifdef SBK_OUTPUT
  ComputeTransforms(true);
  #else
  ComputeTransforms(false);
  #endif

  ComputeEpipolarGeometry();

  MakeMatchListsSymmetric();
  ComputeMatchPoints();

  DumpCorrespondenceImages();

  BundleAdjust(output_file, output_base);
 }
#else

#ifndef __DEMO__

//--[X] Re-run bundler then exit.
if (m_rerun_bundle) 
 {
  //ReadCameraConstraints();
  assert(m_bundle_provided);

  printf("[BundlerApp::OnInit] Reading bundle file...\n");
  ReadBundleFile(m_bundle_file);

  if (m_bundle_version < 0.3) 
   {
    printf("[BundlerApp::OnInit] Reflecting scene...\n");
    FixReflectionBug();
   }

  ReRunSFM();
  exit(0);
 }


//--[X] If not re-running, then this should be first run, no?
if (m_use_constraints) 
  ReadCameraConstraints();

if (m_ignore_file != NULL) 
 {
  printf("[BundlerApp::OnInit] Reading ignore file...\n");
  ReadIgnoreFile();
 }
#endif
//TODO: Above/below code is the same for NOT DEMO mode.
//TODO: Of course bundler compiles in NOT DEMO mode.
if (m_ignore_file != NULL) 
 {
  printf("[BundlerApp::OnInit] Reading ignore file...\n");
  ReadIgnoreFile();
 }

/* Do bundle adjustment (or read from file if provided) */
// ParseCommand("UndistortAll", NULL);
if (m_bundle_provided) 
 {
  printf("[BundlerApp::OnInit] Reading bundle file...\n");
  ReadBundleFile(m_bundle_file);

  if (m_bundle_version < 0.3) 
   {
    printf("[BundlerApp::OnInit] Reflecting scene...\n");
    FixReflectionBug();
   }

  // AutoTagImages();

  if (m_compress_list) 
   {
    // ParseCommand("UndistortAll", NULL);
    OutputCompressed();
    return 0;
   }

  if (m_reposition_scene) 
   {
    double center[3], R[9], scale;
    RepositionScene(center, R, scale);
    OutputCompressed("reposition");
    return 0;
   }

  if (m_prune_bad_points) 
   {
    SetupImagePoints(3);
    RemoveBadImages(24);
    PruneBadPoints();
    OutputCompressed("pruned");
    return 0;
   }

  if (m_scale_focal != 1.0) 
   {
    ScaleFocalLengths(m_scale_focal);
    return 0;
   }

  if (m_scale_focal_file != NULL) 
   {
    ScaleFocalLengths(m_scale_focal_file);
    return 0;
   }

  if (m_rotate_cameras_file != NULL) 
   {
    RotateCameras(m_rotate_cameras_file);
   }

  if (m_track_file != NULL) 
   {
    // ReadGeometricConstraints("constraints.txt");
    CreateTracksFromPoints();
    WriteTracks(m_track_file);
   }

  if (m_zero_distortion_params) 
   {
    ZeroDistortionParams();
    OutputCompressed("nord");
    return 0;
   }
 
  if (m_output_relposes) 
   {
    double center[3], R[9], scale;
    RepositionScene(center, R, scale);
    RepositionScene(center, R, scale);
    // OutputRelativePoses2D(m_output_relposes_file); //TODO: Remove?
    OutputRelativePoses3D(m_output_relposes_file);
    return 0;
   }

  if (m_compute_covariance) 
   {
    ComputeCameraCovariance();
    return 0;
   }
 
  #define MIN_POINT_VIEWS 3 // 0 // 2
  if (!m_run_bundle) 
   {
    SetMatchesFromPoints(MIN_POINT_VIEWS);
    // WriteMatchTableDrew(".final");            

    printf("[BundlerApp::OnInit] Setting up image points and lines...\n");
    SetupImagePoints(/*2*/ MIN_POINT_VIEWS);
    RemoveBadImages(6);

    if (m_point_constraint_file != NULL) 
     {
      printf("[BundlerApp::OnInit] Reading point constraints...\n");
      m_use_point_constraints = true;
      ReadPointConstraints();
     }

    printf("[BundlerApp::OnInit] Scaling world...\n");
    
    printf("[BundlerApp::OnInit] Computing camera orientations...\n");
    ComputeImageRotations();

    // printf("[BundlerApp::OnInit] Computing ground plane...\n");
    // FindGroundPlane();

    double center[3], R[9], scale;
    RepositionScene(center, R, scale);

    #ifndef __DEMO__
    if (m_rerun_bundle) 
     {
      ReRunSFM();
     }
    #endif
   }

  if (m_add_image_file != NULL) 
   {
    printf("[BundlerApp::OnInit] Adding additional images...\n");
    FILE *f = fopen(m_add_image_file, "r");

    if (f == NULL) 
     {
      printf("[BundlerApp::OnInit] Error opening file %s for reading\n",
                                                             m_add_image_file);
     } 
    else 
     {
      #ifndef __DEMO__
      BundleImagesFromFile(f);

      /* Write the output */
      OutputCompressed("added");

      if (m_bundle_version < 0.3)
        FixReflectionBug();

      // RunSFMWithNewImages(4);
      #else
      InitializeImagesFromFile(f);
      #endif

      fclose(f);
     }
   }
 }

if (m_run_bundle) 
 {
  #ifndef __DEMO__
  if (!m_fast_bundle)
    BundleAdjust();
  else
    BundleAdjustFast();
 
  if (m_bundle_version < 0.3)
    FixReflectionBug();

  exit(0);
  #endif
 }

#endif

return true;
}

static BundlerApp *bundler_app = NULL;

//TODO: THE FUNCTION BELOW IS NOT USED AT ALL.  MAYBE ONCE WHEN THERE
//        WAS GRAPHICAL OUTPUT.  NOW THAT IS NOT THE CASE.
BundlerApp &wxGetApp() 
{
return *bundler_app;
}

/*------------------------------- main -------------------------------*/
/*
   Main function.  Basically intantiates a bundler class object, then
   initializes it.  Upon return from initializtion, the bundler has
   done its job and execution ends.  All output should live in some
   file somewhere.

   TODO: Will have to modify so that output can be returned to Matlab
   TODO:  somehow.   This will be the mexFunction eventually ...
*/
int main(int argc, char **argv) 
{
// mtrace();

bundler_app = new BundlerApp();
bundler_app->argc = argc;   //TODO: In my mind, it is kind of sloppy to
bundler_app->argv = argv;   //TODO:  set member variables argc/argv.
                            //TODO: They should be processed through a
                            //TODO:  member function.

bundler_app->OnInit();
}


/*============================= BundlerApp =============================*/
