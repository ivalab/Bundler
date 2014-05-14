/*============================ TwoFrameModel ===========================*/
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
/*============================ TwoFrameModel ===========================*/
/*
   Revised coding to suit my particular style.   The two frame model
   is the initializer for the full Bundler system.  Two good views
   are first found and used to create the initial map.  After that,
   new image frames are added based on how they relate to this initial
   map.  There should be strong overlap (in Snavely's paper, strong
   overlap is depicted by proximity in the viewpoint graph to the
   two base frames that consistute the two frame model).


   Author:      Noah Snavely            [original]
   Maintainer:  Patricio A. Vela,       pvela@gatech.edu

   Date:        2014/05/13

   TODO:        Figure out why TwoFrameModel and ModelMap are merged 
                into this one file.  Can the two be extracted?
                Can some of the non-class code be extracted and put
                into a ModelIO file, or is it best to have some
                of the utility functions here with the classes?
*/
/*============================ TwoFrameModel ===========================*/
#ifndef __TWOFRAMEMODEL_H__
#define __TWOFRAMEMODEL_H__

#include <stdio.h>

#include "Geometry.h"
#include "ImageData.h"
#include "BaseApp.h"

#include "sfm.h"

#ifndef WIN32
#  include <ext/hash_map>
#else
#  include <hash_map>
#endif

class TwoFrameModel 
{

public:
  /*========= Member Functions ==========*/
  TwoFrameModel(); 
    
  void Read(FILE *f);
  void Write(FILE *f) const;
  void WriteSparse(FILE *f);
  void WriteBrief(FILE *f) const;

  void WriteWithProjections(FILE *f, const std::vector<TrackData> &tracks,
                            int i1, int i2, const ImageData &img1, 
                                            const ImageData &img2) const;

  double ComputeTrace(bool flip) const;

  double AverageDistanceToPoints() const;

  void ComputeTransformedCovariance(bool flip, double *S, double *C) const;

  void SetPrevious(bool flip, int prev);
  int GetPrevious(bool flip);
  void SetPredecessor(bool flip, int pred);
  int GetPredecessor(bool flip);
  void SetFlag(bool flip, int flag);
  int GetFlag(bool flip);

  /*========== Member Variables ==========*/
  //TODO: Should look into making there protected at minimum.
  //TODO: Don't really want access to variables that should 
  //TODO: be accessible, especially since want to avoid potentially
  //TODO: damaging external modifications.
  int   m_num_points;
  v3_t *m_points;
  int  *m_tracks;
  int  *m_keys1, *m_keys2;

  camera_params_t m_camera0, m_camera1;
  double m_C0[9], m_C1[9];
  double m_angle;                       /* in degrees */
  double m_error;                       /* mean reprojection error */

  double m_scale0, m_scale1;
  int  m_flag0, m_flag1;
  int  m_prev0, m_prev1;
  int  m_pred0, m_pred1;
  bool m_shortest_computed0, m_shortest_computed1;
  bool m_shortest0, m_shortest1;        /* Is this edge a shortest path */
};


#ifdef WIN32
typedef stdext::hash_map<unsigned int, TwoFrameModel > ModelTable;
#else
typedef __gnu_cxx::hash_map<unsigned int, TwoFrameModel > ModelTable;
#endif

class ModelMap
{
public:

  ModelMap() 
    { }

  ModelMap(int num_images) 
   {
    m_models.resize(num_images);
    m_neighbors.resize(num_images);
   }

  void AddModel(MatchIndex idx, const TwoFrameModel &model) 
   {
    assert(idx.first < idx.second);

    if (Contains(idx))
      return;  // already set

    /* Add the model to the hash */
    m_models[idx.first][idx.second] = model;

    std::list<unsigned int> tmp;
    tmp.push_back(idx.second);
    m_neighbors[idx.first].merge(tmp);

    // tmp.pop_back();
    tmp.clear();
    tmp.push_back(idx.first);
    m_neighbors[idx.second].merge(tmp);
   }

  void RemoveModel(MatchIndex idx) 
   {
    assert(idx.first < idx.second);

    if (Contains(idx)) 
     {
      m_models[idx.first].erase(idx.second);

      // Remove the neighbor
      std::list<unsigned int> &l = m_neighbors[idx.first];
      std::pair<std::list<unsigned int>::iterator,
      std::list<unsigned int>::iterator> p = 
      equal_range(l.begin(), l.end(), idx.second);

      assert(p.first != p.second);
      l.erase(p.first, p.second);

      std::list<unsigned int> &l2 = m_neighbors[idx.second];
      p = equal_range(l2.begin(), l2.end(), idx.first);
 
      assert(p.first != p.second);
      l2.erase(p.first, p.second);
     }
   }

  TwoFrameModel &GetModel(MatchIndex idx) 
   {
    assert(idx.first < idx.second);
    assert(Contains(idx));
    return m_models[idx.first][idx.second];
   }
 
  bool Contains(MatchIndex idx) const 
   {
    assert(idx.first < idx.second);
    return (m_models[idx.first].find(idx.second) != 
    m_models[idx.first].end());
   }

  void RemoveAll() 
   {
    int num_lists = m_models.size();
    for (int i = 0; i < num_lists; i++) 
     {
      m_models[i].clear();
      m_neighbors[i].clear();
     }
   }

  std::list<unsigned int> &GetNeighbors(unsigned int i) 
   {
    return m_neighbors[i];
   }
 
  ModelTable::iterator Begin(unsigned int i) 
   {
    return m_models[i].begin();
   }
 
  ModelTable::iterator End(unsigned int i) 
   {
    return m_models[i].end();
   }
    
private:
  std::vector<ModelTable> m_models;
  std::vector<std::list<unsigned int> > m_neighbors;
};

#ifndef WIN32
typedef __gnu_cxx::hash_map<int, bool> PEdgeMap;
#else
typedef stdext::hash_map<int, bool> PEdgeMap;
#endif

void WriteModels(ModelMap &models, int num_images, char *out_file);
void WriteModelsSparse(ModelMap &models, int num_images, char *out_file);
void WritePEdges(PEdgeMap &p_edges, int num_images, char *out_file);
ModelMap ReadModels(FILE *f, int *num_images_out = NULL);
PEdgeMap ReadPEdges(FILE *f, int num_images);

void ThresholdTwists(int num_images, ModelMap &models, 
                     std::vector<ImageData> &image_data, bool panos_only);


#endif /* __TWOFRAMEMODEL_H__ */

/*
*/
/*============================ TwoFrameModel ===========================*/
