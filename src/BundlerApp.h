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

   Header file for the BundlerApp class.  Also includes a sub-class of
   the BundlerApp class called SkeletalApp.  Presumably this sub-class
   has the minimal functionality

   Author:      Noah Snavely
   Modified:    Patricio A. Vela,       pvela@gatech.edu

   Date:        2014/05/11
*/
/*============================= BundlerApp =============================*/
#ifndef __bundlerapp_h__
#define __bundlerapp_h__

#include "BaseApp.h"
#include "LinkDirection.h"
#include "TwoFrameModel.h"

typedef std::pair<int,int> ImagePair;
typedef std::pair<int,int> IntPair;

/*------------------------- BundlerApp Class-------------------------*/
/*
*/
class BundlerApp : public BaseApp
{
public:
  BundlerApp();

  virtual bool OnInit();

  /* Process command line options */
  virtual void ProcessOptions(int argc, char **argv);

  /* Create a search tree for all keypoints */

  /* Return the number of parameters used to model each camera */
  int GetNumCameraParameters();

  /* Enrich the set of correspondences */
  void EnrichCorrespondences(double alpha, double threshold);

  /* Prune image matches that are not well-supported */
  void PruneMatchesThreshold(int threshold);
  /* Remove matches close to the edges of the two given images */
  void RemoveMatchesNearBorder(int i1, int i2, int border_width);
  /* Remove matches close to the bottom edge of the two given images */
  void RemoveMatchesNearBottom(int i1, int i2, int border_width);

  /* Compute a transform between a given pair of images */
  bool ComputeTransform(int idx1, int idx2, bool removeBadMatches);

  /* Compute transforms between all matching images */
  void ComputeTransforms(bool removeBadMatches, int new_image_start = 0);

  /* Compute epipolar geometry between a given pair of images */
  bool ComputeEpipolarGeometry(int idx1, int idx2, bool removeBadMatches);

  /* Compute epipolar geometry between all matching images */
  void ComputeEpipolarGeometry(bool removeBadMatches, int new_image_start = 0);

  /* Compute a set of tracks that explain the matches */
  void ComputeTracks(int new_image_start = 0);

  /* Compute geometric information about image pairs */
  void ComputeGeometricConstraints(bool overwrite = false, 
				     int new_image_start = 0);

#ifndef __DEMO__
  /* Set constraints on cameras */
   void SetCameraConstraints(int cam_idx, camera_params_t *params);
   void SetFocalConstraint(const ImageData &data, camera_params_t *params);
   void ClearCameraConstraints(camera_params_t *params);
#endif /* __DEMO__ */

   void CheckPointKeyConsistency(const std::vector<ImageKeyVector> pt_views,
                                  int *added_order);

#ifndef __DEMO__
  /* Initialize the bundle adjustment procedure (loading an existing
   * model if one exists) */
  void InitializeBundleAdjust(int &num_init_cams, int *added_order,
				int *added_order_inv, camera_params_t *cameras,
				v3_t *points, v3_t *colors,
				std::vector<ImageKeyVector> &pt_views,
				bool use_constraints);

  /* Set up the matrix of projections and the visibility mask */
  void SetupProjections(int num_cameras, int num_points, int *added_order,
			  v2_t *projections, char *vmask);

  /* Find the camera with the most matches to existing points */
  int FindCameraWithMostMatches(int num_cameras, int num_points,
				  int *added_order, int &parent_idx, int &max_matches,
				  const std::vector<ImageKeyVector> &pt_views);

  /* Find all cameras with at least N matches to existing points */
  std::vector<ImagePair> FindCamerasWithNMatches(int n, 
						   int num_cameras, int num_points, int *added_order,
						   const std::vector<ImageKeyVector> &pt_views);

  /* Find the camera that would allow use to "grow" the scene as
   * much as possible */
  int FindCameraWithMostConnectivity(int num_cameras, int num_points,
				       int *added_order, int &parent_idx, int &max_matches);

  /* Triangulate a subtrack */
  v3_t TriangulateNViews(const ImageKeyVector &views, 
			   int *added_order, camera_params_t *cameras,
			   double &error, bool explicit_camera_centers);

  v3_t GeneratePointAtInfinity(const ImageKeyVector &views, 
               int *added_order, camera_params_t *cameras, double &error, 
               bool explicit_camera_centers);
    
  /* Add new points to the bundle adjustment */
  int BundleAdjustAddNewPoints(int camera_idx, int num_points, int num_cameras,
				 int *added_order, camera_params_t *cameras, v3_t *points, 
                 v3_t *colors, double reference_baseline,
				 std::vector<ImageKeyVector> &pt_views);

  /* Add new points to the bundle adjustment */
  int BundleAdjustAddAllNewPoints(int num_points, int num_cameras,
				    int *added_order, camera_params_t *cameras,
				    v3_t *points, v3_t *colors, double reference_baseline,
				    std::vector<ImageKeyVector> &pt_views,
				    double max_reprojection_error = 16.0, int min_views = 2);
    
  /* Remove bad points and cameras from a reconstruction */
  int RemoveBadPointsAndCameras(int num_points, int num_cameras, 
                    int *added_order, camera_params_t *cameras, v3_t *points, 
                    v3_t *colors, std::vector<ImageKeyVector> &pt_views);

  /* Compute pose of all cameras */
  void BundleAdjust();

  /* Quickly compute pose of all cameras */
  void BundleAdjustFast();
    
  /* Estimate poses of all ignored cameras */
  void EstimateIgnoredCameras(int &curr_num_cameras, camera_params_t *cameras,
                     int *added_order, int &pt_count, v3_t *points, 
                     v3_t *colors, std::vector<ImageKeyVector> &pt_views);

  /* Pick a good initial pair of cameras to bootstrap the bundle
   * adjustment */
  void BundlePickInitialPair(int &i_best, int &j_best, 
                             bool use_init_focal_only);

  /* Setup the initial camera pair for bundle adjustment */
  int SetupInitialCameraPair(int i_best, int j_best,
			       double &init_focal_length_0, double &init_focal_length_1,
			       camera_params_t *cameras, v3_t *points, v3_t *colors,
			       std::vector<ImageKeyVector> &pt_views);

  /* Initialize a single image */
  void BundleImage(char *filename, int parent_img);
  /* Initialize images read from a file */
  void BundleImagesFromFile(FILE *f);

  /* Initialize an image for bundle adjustment */
  camera_params_t BundleInitializeImage(ImageData &data, int image_idx, 
                        int camera_idx, int num_cameras, int num_points,
                        int *added_order, v3_t *points,
                        camera_params_t *parent, camera_params_t *cameras, 
                        std::vector<ImageKeyVector> &pt_views,
                        bool *success_out = NULL,
			            bool refine_cameras_and_points = false);

  /* Initialize an image for bundle adjustment (running a full
   * optimization) */
  void BundleInitializeImageFullBundle(int image_idx, int parent_idx,
                   int num_cameras, int num_points, int *added_order,
                   camera_params_t *cameras, v3_t *points, v3_t *colors,
                   std::vector<ImageKeyVector> &pt_views);

  /* Refine a set of 3D points */
  double RefinePoints(int num_points, v3_t *points, v2_t *projs,
			   int *pt_idxs, camera_params_t *cameras, int *added_order,
			   const std::vector<ImageKeyVector> &pt_views,
			   camera_params_t *camera_out);

  /* Refine a given camera and the points it observes */
  std::vector<int> RefineCameraAndPoints(const ImageData &data, int num_points,
					     v3_t *points, v2_t *projs, int *pt_idxs, 
					     camera_params_t *cameras, int *added_order,
					     const std::vector<ImageKeyVector> &pt_views,
					     camera_params_t *camera_out, bool remove_outliers);
    
  void MatchCloseImagesAndAddTracks(ImageData &data, int this_cam_idx,
                         int added_order_idx, 
                                      std::vector<ImageKeyVector> &pt_views);
  void RunSFMWithNewImages(int new_images, double *S = NULL, double *U = NULL,
                           double *V = NULL, double *W = NULL);
  void ReRunSFM(double *S = NULL, double *U = NULL, double *V = NULL, 
                double *W = NULL);
  double RunSFM(int num_pts, int num_cameras, int start_camera,
		        bool fix_points, camera_params_t *init_camera_params,
		        v3_t *init_pts, int *added_order, v3_t *colors,
		        std::vector<ImageKeyVector> &pt_views, double eps2 = 1.0e-12,
                double *S = NULL, double *U = NULL, double *V = NULL,
                double *W = NULL, bool remove_outliers = true);
  double RunSFMNecker(int i1, int i2, camera_params_t *cameras, int num_points,
                v3_t *points, v3_t *colors,
                std::vector<ImageKeyVector> &pt_views,
                camera_params_t *cameras_new, v3_t *points_new, 
                double threshold);
#endif /* __DEMO__ */

  bool BundleTwoFrame(int i1, int i2, TwoFrameModel *model, 
                      double &angle_out, int &num_pts_out, 
                      bool bundle_from_tracks);
  bool EstimateRelativePose(int i1, int i2, 
                            camera_params_t &camera1, 
                            camera_params_t &camera2);

  bool EstimateRelativePose2(int i1, int i2, 
                             camera_params_t &camera1, 
                             camera_params_t &camera2);

  /* Register a new image with the existing model */
  bool BundleRegisterImage(ImageData &data, bool init_location);
  void RunBundleServer();

#ifdef __USE_BOOST__
  /* Graph operations */
  ImageGraph ComputeMSTWorkingGraph(std::vector<int> &interior);
  void PartitionGraph(ImageGraph &graph, std::vector<int> interior);
#endif /* __USE_BOOST__ */

  /* Output a compressed version of the bundle file */
  void OutputCompressed(const char *ext = "compressed");

  /* Other operations on bundle files */
  void ScaleFocalLengths(double focal);    
  void ScaleFocalLengths(char *focal_file);
  void RotateCameras(char *rotate_file);
  void PruneBadPoints();
  void ZeroDistortionParams();
  void OutputRelativePoses2D(const char *outfile);
  void OutputRelativePoses3D(const char *outfile);
  void ComputeCameraCovariance();

  /* Analyze point statistics */
  void AnalyzePoints();

  /* Find a ground plane in the scene */
  void FindGroundPlane();
  /* Find a sky plane in the scene */
  void FindSkyPlane();

  /* Coalesce feature descriptors for each feature point */
  void CoalesceFeatureDescriptors();
  void CoalesceFeatureDescriptorsMedian();

  /* Compute likely matches between a set of keypoints and the
   * reconstructed points */
  std::vector<KeypointMatch>
        MatchKeysToPoints(const std::vector<KeypointWithDesc> &k1, 
                          double ratio = 0.6);

  std::vector<KeypointMatch>
        MatchPointsToKeys(const std::vector<KeypointWithDesc> &keys, 
			  double ratio = 0.6);

  void ReadProjectivePoints();
  void ReadProjectiveCameras();

  /* Routines for predicting the next images that should be captured */
  bool ImageVerifiesRay(int img, const double *p0, const double *p1);
  std::vector<int> GetVerifiersForImage(int img);
  void GetPointCoverage(int img, double &left, double &right,
                        double &up, double &down);
  int PredictNextImage(LinkDirection &dir);
  void RenderPredictedImage(int idx, LinkDirection dir, 
                            const char *out_file);


  /*========== BundlerApp Member Variables =========*/

  /* ----- Bundler Options ----{ */
  bool m_panorama_mode;        /* Are we reconstructing a panorama? */
  bool m_add_images_fast;
  bool m_estimate_ignored;
  bool m_analyze_matches;      /* Analyze matches */

  int m_ann_max_pts_visit;     /* Max. points to visit during global matching */

  bool m_match_global;         /* Compute matches using global matcher */
  double m_global_nn_sigma;    /* Threshold from expected variance
                                  where features match */
  int m_global_knn;            /* No. neighbors to find in global matching */

  bool m_optimize_for_fisheye; /* Optimize for fisheye-distorted points */

  int m_homography_rounds;     /* Homography RANSAC params */
  double m_homography_threshold;

  int m_fmatrix_rounds;        /* F-matrix RANSAC params */
  double m_fmatrix_threshold;
  bool m_skip_fmatrix;
  bool m_skip_homographies;
  bool m_use_angular_score;

  double m_projection_estimation_threshold; /* RANSAC threshold for estimating 
                                                projection matrix */

  double m_min_proj_error_threshold;
  double m_max_proj_error_threshold;

  double m_min_camera_distance_ratio;   /* Min. dist. for a non-panorama */

  double m_baseline_threshold;          /* Smallest permissible dist. between 
                                            two camera centers */

  double m_ray_angle_threshold;     /* Ray angle threshold */

  bool m_use_focal_estimate;        /* Estimate focal length of cameras */
  bool m_trust_focal_estimate;

  int m_min_max_matches;            /* Min. # matches to register an image */

  char *m_bundle_output_file;       /* Output file names for BA */
  char *m_bundle_output_base;
  char *m_output_directory;

  bool m_compute_covariance;   /* Compute covariance of a reconstruction */
  int m_covariance_fix1;       /* Image to fix when computing covariance */
  int m_covariance_fix2;       /* Image to fix translation of when
                                * computing covariance */

  int m_keypoint_border_width; /* Throw out keypoints too close to
                                * the border of an image */
  int m_keypoint_border_bottom; /* Throw out keypoints too close to
                                 * the bottom of an image */
  double m_init_focal_length;  /* Initial focal length for BA */
  bool m_fixed_focal_length;   /* Is the focal length constant? */
  bool m_use_constraints;      /* Should we use camera constraints? */
  bool m_constrain_focal;      /* Should we constrain the focal
                                * length of calibrated cameras? */
  double m_constrain_focal_weight;  /* The weight for the focal
                                     * length constraint */

  bool m_factor_essential;     /* Should the model be initialized by
                                * factoring the essential matrix? */

  bool m_estimate_distortion;  /* Should we estimate distortion for
                                * each camera? */
  double m_distortion_weight;  /* Weight on distortion parameter
                                * constraints */

  bool m_construct_max_connectivity;  /* Do bundle adjustment using
                                       * the connectivity score? */

  bool m_only_bundle_init_focal;  /* Only bundle adjust camera with
                                   * initialized focal lengths */

  bool m_fix_necker;        /* Fix Necker reversal during bundle adjustment? */ 

  int m_initial_pair[2];    /* Images to use as the initial pair
				             * during bundle adjustment */

  bool m_features_coalesced;   /* Have features been coalesced */

  int m_server_port;        /* Port to use when in server mode */
  bool m_server_mode;       /* Run bundler as a server? */
  bool m_assemble;          /* Assemble the scene from the bottom up    */
  bool m_run_bundle;        /* Run bundle adjustment automatically?     */
  bool m_rerun_bundle;      /* Rerun bundle adjustment automatically?   */
  bool m_fast_bundle;       /* Run fast version of * bundle adjustment? */
  bool m_skip_full_bundle;  /* Skip full optimization stages */
  bool m_skip_add_points;   /* Don't add new points to the optimization */

  /* }---- Operations on bundle files ----{ */
  bool m_compress_list;        /* Output a compressed list and bundle file */
  bool m_reposition_scene;     /* Reposition the scene? */
  bool m_prune_bad_points;     /* Prune bad points?     */

  double m_scale_focal;        /* Amount by which to scale the focal lengths */

  bool m_predict_next_image;
  char *m_prediction_image;

  char *m_add_image_file;      /* Additional images to add */
  char *m_scale_focal_file;
  char *m_rotate_cameras_file;
  char *m_track_file;

  bool m_output_relposes;
  char *m_output_relposes_file;

  bool m_segment_sky;          /* Activative sky segmentation */
  char *m_sky_model_file; 

  bool m_enrich_points;             /* Enrich the point set? */
  bool m_zero_distortion_params;    /* Zero the distortion parameters? */

  int argc;
  char **argv;
  /*}----- */
};

#endif /* __bundlerapp_h__ */

/*
*/
/*============================= BundlerApp =============================*/
