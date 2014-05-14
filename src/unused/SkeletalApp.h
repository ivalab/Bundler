



/*------------------------- SkeletalApp Class ------------------------*/
/*
*/
class SkeletalApp : public BundlerApp
{
public:
  SkeletalApp() 
   {
    BundlerApp();
    m_start_camera = -1;
   }

  virtual bool OnInit();

  /* Process command line options */
  virtual void ProcessOptions(int argc, char **argv);

  /* Estimate the global scene orientation based on geotags */
  void EstimateGlobalOrientation(ModelMap &models);

  /* Read in a set of initial rotations */
  void ReadInitialRotations(const char *filename, double *Rout);
  void ReadInitialTranslations(const char *filename, double *tout);
  void ReadGlobalOrientation(const char *filename, double *R);
  void WriteGlobalOrientation(const char *filename, double *R);

  /* Refine the current set of rotations using pairwise rotations */
  void RefineInitialRotations(ModelMap &models);
  void RefineInitialTranslations(ModelMap &models);

  /* Read in a set of rotations produced by a previous run */
  void ReadRotationsFile(const char *filename, double *Rout);
    
  /* Estimate confidences of geotags based on agreement with 
     global translation */
  void ComputeGeotagConfidence(std::vector<IntPair> &pairs,
                               std::vector<bool> &pairs_correct,
                               std::vector<double> &confidence);

#ifndef __DEMO__
  /* Two-frame bundle adjustment */
  ModelMap BundleAllPairs(char *out_file, 
                          bool bundle_from_tracks, bool detect_duplicates);
  // ModelMap ReadModels(FILE *f);

  /* Estimate a similarity transform between two 2-frame models */
  bool EstimateSimilarityTransform(const TwoFrameModel &m0, 
               const TwoFrameModel &m1, MatchIndex idx1, MatchIndex idx2,
               // double *S, 
               double &scale, bool verbose = false, bool clip_inliers = false,
               int *num_inliers = NULL, int *num_total = NULL);

  void ComputeInitialTransforms(ModelMap &models, bool test_triangles);

  double EvaluateModelPath(ModelMap &models, const std::vector<int> nodes,
                              double *Cout);
  bool ComputeShortestPath(ModelMap &models, int i1, int i2,
              double &dist_final, double &scale_final, std::vector<int> &path, 
              bool &bounds_exceeded, bool check_bounds = false,
              bool verbose = false, bool exit_when_reached = false, 
              int second_node = -1, int max_depth = -1,
              double bound_factor = 1.0);

  void PrunePairGraph(ModelMap &models, PEdgeMap &p_edges, double prune_factor);
  void ConstructTSpanner(ModelMap &models, double t);
  void ConstructTSpanner2(ModelMap &models, PEdgeMap &p_edges, double t);
  void ConstructTSpanner3(ModelMap &models, PEdgeMap &p_edges, double t);
  void ConstructTSpanner4(ModelMap &models, PEdgeMap &p_edges, double t);

  bool CheckEdge(int i1, int i2, ModelMap &models, 
                                 ModelMap &subgraph, double t);
  bool EdgeIsShortestPath(ModelMap &models, int i1, int i2, 
                                                    double bound_factor = 1.0);

  typedef enum { White = 0, Gray = 1, Black = 2, PNode = 3} NodeState;
  bool NodeConnected(int image, NodeState *states,
                     ModelMap &models, ModelMap &subgraph);
  void NodeConnectedThrough(int image, int link, NodeState *states,
                            ModelMap &models, ModelMap &subgraph,
                            int &num_connected, int &num_total);
  bool EdgeCanBePruned(ModelMap &models, PEdgeMap &p_edges, 
                       unsigned int i1, unsigned int i2);
  void ColorIncidentEdgesGray(ModelMap &models, NodeState *states, 
                              int i1, int i2, bool symmetric = false);
  void ColorPNodes(ModelMap &models, PEdgeMap &p_edges, bool *p_node_flags, 
                   int i1, int i2);
  void ComputeEdgeIncidence(int i1, int i2, ModelMap &models, NodeState *states,
                            int &white_degree, int &incidence);
  void TestTriangles(ModelMap &models);
  void AddLinksFromGrayToBlack(ModelMap &models, ModelMap &subgraph,
                               NodeState *states, int *degrees_inter);
  void AddStrongLinksFromGrayToBlack(ModelMap &models, ModelMap &subgraph,
                     NodeState *states, std::vector<int> components, 
                                                          int *degrees_inter);
  void AddRequiredLinksFromBlackToBlackByWeight(ModelMap &models,
                     ModelMap &subgraph, int num_images, NodeState *states,
                                                      int *degrees, double t);
  void AddRequiredLinksFromBlackToGray(ModelMap &models,
                     ModelMap &subgraph, int num_images, NodeState *states,
                                                      int *degrees, double t);
  void AddRequiredLinksFromBlackToGrayByWeight(ModelMap &models,
                     ModelMap &subgraph, int num_images, NodeState *states,
                                                      int *degrees, double t);
  void AddRequiredLinksFromGrayToGray(ModelMap &models, ModelMap &subgraph,
                     int num_images, NodeState *states, int *degrees, double t);
  void AddRequiredLinksFromGrayToGrayByWeight(ModelMap &models,
                     ModelMap &subgraph, int num_images, NodeState *states,
                                                      int *degrees, double t);
#endif /* __DEMO__ */

  /* ---- Options ----{ */
  bool m_bundle_from_tracks;   /* ... when computing pairwise recons. */
  bool m_bundle_from_points;   /* ... when computing pairwise recons. */
  double m_stretch_factor;     /* ... when computing spanner */    

  bool m_detect_duplicates;    /* ... when computing pairwise recons. */

  bool m_dump_pairs_sparse;    /* Write the pairs file to a nicer format */
  bool m_estimate_orientation; /* Estimate global orientation */
  bool m_threshold_twists;     /* Get rid of cameras with large twist */

  char *m_initial_rotations_file;  /* Refine rotation matrices using
                                    * this file for initialization */
  char *m_initial_translations_file;  /* Refine translations using
                                       * this file for initialization */
  char *m_rotations_file;           /* Refined rotations from previous run */
  char *m_global_orientation_file;  /* Contains global scene orientation */

  char *m_pairs_file;       /* File to read pairwise reconstructions from */

  // int m_start_camera;    /* Camera to seed the t-spanner */
  /* }---- */
};


