
ModelMap SkeletalApp::BundleAllPairs(char *out_file, 
                                     bool bundle_from_tracks, 
                                     bool detect_duplicates) 
{
    unsigned int num_images = GetNumImages();

    if (out_file != NULL) {
        FILE *f = fopen(out_file, "r");

        bool load_pruned = false;
        if (strstr(out_file, "pruned"))
            load_pruned = true;

        if (f != NULL) {
            ModelMap models = ReadModels(f);
            // p_edges_out = ReadPEdges(fp, num_images);
            FixScaffoldEdges(num_images, models);
            ThresholdTwists(num_images, models, m_image_data, true);
            fclose(f);
            return models;
        }
    }

    /* Sort images by connectivity */
    double *connectivity = new double[num_images];
    // bool *connected = new bool[num_images * num_images];

#define MATCH_THRESHOLD 28   // 16

    for (unsigned int i = 0; i < num_images; i++)
        connectivity[i] = 0.0;

    // for (int i = 0; i < num_images * num_images; i++) { 
    //     connected[i] = false;
    // }

    unsigned long long num_connections = 0;
    for (unsigned int i = 0; i < num_images; i++) {
        if (!m_image_data[i].m_has_init_focal)
            continue;

        MatchAdjList::iterator iter;

        // for (int j = i+1; j < num_images; j++) { //}
        for (iter = m_matches.Begin(i); iter != m_matches.End(i); iter++) {
            unsigned int j = iter->m_index;

            if (!m_image_data[j].m_has_init_focal)
                continue;

            if (i >= j) 
                continue;

            int num_matches; 
            
            if (bundle_from_tracks) {
                if (!ImagesMatch(i, j))
                    continue;
                num_matches = GetNumTrackMatches(i,j);
            } else { 
                num_matches = GetNumMatches(i, j);
            }
            
            if (num_matches >= MATCH_THRESHOLD) {
                connectivity[i] += 1.0;
                connectivity[j] += 1.0;
                // connected[i * num_images + j] = true;
                // connected[j * num_images + i] = true;
                num_connections++;
            }
        }
    }

#if 0
    int *perm = new int[num_images];
    qsort_descending();
    qsort_perm(num_images, connectivity, perm);

    for (int i = 0; i < num_images; i++) {
        printf("connect[ %d ] = %0.3f\n", perm[i], connectivity[i]);
    }

    delete [] connectivity;
#else
    for (unsigned int i = 0; i < num_images; i++) {
        printf("connect[ %d ] = %0.3f\n", i, connectivity[i]);
    }
#endif

    double *match_array = new double[num_connections];
    unsigned long *indices = new unsigned long [num_connections];
    
    int count = 0;
    for (unsigned int i = 0; i < num_images; i++) {
        if (!m_image_data[i].m_has_init_focal)
            continue;

        // for (int j = i+1; j < num_images; j++) {
        MatchAdjList::iterator iter;

        for (iter = m_matches.Begin(i); iter != m_matches.End(i); iter++) {
            unsigned int j = iter->m_index;

            if (!m_image_data[j].m_has_init_focal)
                continue;

            if (i >= j) 
                continue;

            int num_matches;
            
            if (bundle_from_tracks) {                    
                if (!ImagesMatch(i, j))
                    continue;
                num_matches = GetNumTrackMatches(i,j);
            } else { 
                num_matches = GetNumMatches(i, j);
            }
            
            if (num_matches >= MATCH_THRESHOLD) {
                match_array[count] = (double) num_matches;
                indices[count] = 
                    (unsigned long long) i * num_images + 
                    (unsigned long long) j;
                count++;
            }
        }
    }
    
    int *order = new int[num_connections];
    qsort_perm((int) num_connections, match_array, order);
    delete [] match_array;

    bool preload_keys = true;
    if (num_images > 30000)
        preload_keys = false;

    if (preload_keys) {
        for (unsigned int i = 0; i < num_images; i++) {
            if (m_image_data[i].m_has_init_focal) {
                printf("[BundleAllPairs] Loading keys for image %d\n", i);
                m_image_data[i].LoadKeys(false, !m_optimize_for_fisheye);

                if (bundle_from_tracks) {
                    printf("[BundleAllPairs] Setting tracks for image %d\n", 
                           i);
                    SetTracks(i);
                }

                fflush(stdout);
            }
        }
    }

    ModelMap models(num_images);

    int *depends = new int[num_images];

    for (unsigned int i = 0; i < num_images; i++) {
        depends[i] = -1;
    }
    
    // for (int i = 0; i < num_images; i++) { //}
    for (unsigned long i = 0; i < num_connections; i++) {
        unsigned long long index = indices[order[i]];

        unsigned long i1 = index / num_images;
        unsigned long i2 = index - i1 * num_images;
        
        if (connectivity[i2] > connectivity[i1]) {
            int tmp = i1;
            i1 = i2;
            i2 = tmp;
        }

        // int i1 = perm[i];
        if (depends[i1] != -1)
            continue;

        if (!m_image_data[i1].m_has_init_focal)
            continue;
        
        if (!preload_keys) {
            m_image_data[i1].LoadKeys(false, !m_optimize_for_fisheye);
            SetTracks(i1);
        }

        // for (int j = i+1; j < num_images; j++) {
        // int i2 = perm[j]; //}

        if (depends[i2] != -1)
            continue;

        if (!m_image_data[i2].m_has_init_focal)
            continue;

        if (bundle_from_tracks) {
            if (GetNumTrackMatches(i1, i2) < MATCH_THRESHOLD)
                continue;
        } else {
            if (GetNumMatches(i1, i2) < MATCH_THRESHOLD)
                continue;
        }

        printf("[SifterApp::BundleAllPairs] Bundling (%lu,%lu)\n", i1, i2);
        fflush(stdout);

        if (!preload_keys) {
            m_image_data[i2].LoadKeys(false, !m_optimize_for_fisheye);
            SetTracks(i2);
        }
            
        unsigned int i_min = MIN(i1, i2);
        unsigned int i_max = MAX(i1, i2);

        TwoFrameModel model;

        clock_t start = clock();
        double angle = 0.0;
        int num_matches = 0;
        bool success = 
            BundleTwoFrame(i_min, i_max, 
                           &model, angle, num_matches, 
                           bundle_from_tracks);
        fflush(stdout);

#define MAX_DUPLICATE_ANGLE 0.5 // 1.0 // 0.5 // 1.0 // 2.5
#define MIN_DUPLICATE_MATCHES 64

        bool dependent = false;
        if (detect_duplicates && angle < MAX_DUPLICATE_ANGLE && 
            num_matches > MIN_DUPLICATE_MATCHES) {
            
            /* Check if i2 is adjacent to any nodes that i1 is not
             * adjacent to */

            // bool found = false;
            int num_found = 0;
            // for (int k = 0; k < num_images; k++) {
            //     if (k == i1 || k == i2)
            //         continue; //}

            MatchAdjList::iterator iter;
            
            for (iter = m_matches.Begin(i2); 
                 iter != m_matches.End(i2); 
                 iter++) {

                unsigned long k = iter->m_index;

                if (k == i1 || k == i2)
                    continue;

                // if (!connected[i1 * num_images + k] &&
                //     connected[i2 * num_images + k]) { //}
                if ((bundle_from_tracks && 
                     GetNumTrackMatches(i1,k) < MATCH_THRESHOLD &&
                     GetNumTrackMatches(i2,k) >= MATCH_THRESHOLD) ||
                    (!bundle_from_tracks &&
                     (GetNumMatches(i1,k) < MATCH_THRESHOLD || 
                      GetNumMatches(i2,k) >= MATCH_THRESHOLD))) {

                    int num_matches_i2k;
                    if (bundle_from_tracks)
                        num_matches_i2k = GetNumTrackMatches(i2, k);
                    else
                        num_matches_i2k = GetNumMatches(i2, k);

                    if (num_matches_i2k > 64) {
                        printf("  Image %lu is connected to %lu (%d matches) "
                               "but not %lu\n",
                               k, i2, num_matches_i2k, i1);

                        // found = true;
                        // break;

                        num_found++;
                    }
                }
            }
                
            if (num_found < 1 /*MAX(1.0, 0.002 * num_images)*/) {
                /* Make node i2 dependent on node i1 */
                printf("[SifterApp::BundleAllPairs] "
                       "Node %lu depends on node %lu\n", i2, i1);

                dependent = true;
                depends[i2] = i1;

                /* Get rid of other models with i2 */
                // for (int k = 0; k < i2; k++) {
                for (iter = m_matches.Begin(i2); 
                     iter != m_matches.End(i2); 
                     iter++) {
                    
                    unsigned long k = iter->m_index;

                    // int idx = k * num_images + i2;
                    MatchIndex idx = GetMatchIndexUnordered(k, i2);
                    if (models.Contains(idx)) {
                        printf("  Removed model (%lu,%lu)\n", k, i2);
                        models.RemoveModel(idx);
                    }
                }
                
#if 0
                // FIXME LOOP
                for (int k = i2 + 1; k < num_images; k++) { //}
                    // int idx = i2 * num_images + k;
                    MatchIndex idx = GetMatchIndex(i2, k);
                    if (models.Contains(idx)) { //}
                        printf("  Removed model (%d,%d)\n", i2, k);
                        models.RemoveModel(idx);
                    }
                }
#endif
            }
        }

        if (!dependent && success) {
            printf("[SifterApp::BundleAllPairs] "
                   "(%lu,%lu) successfully bundled\n", i1, i2);
            
            if (model.ComputeTrace(true) < 0.0) {
                printf("  Error! trace2 < 0.0!\n");
            }
                
            if (model.ComputeTrace(false) < 0.0) {
                printf("  Error! trace1 < 0.0!\n");
            } else {
                MatchIndex idx = GetMatchIndex(i_min, i_max);
                models.AddModel(idx, model);
            }
        } else if (!dependent) {
            printf("[BundleAllPairs] (%lu,%lu) bundle FAILED\n", i1, i2);
        }

        clock_t end = clock();
        
        double t = (end - start) / (double) CLOCKS_PER_SEC;
        printf("[BundleAllPairs] Bundle took %0.3fs\n", t);
        
        if (!preload_keys) {
            m_image_data[i2].UnloadKeys();
        }
        //added for balance {//}

        if (!preload_keys) {
            m_image_data[i1].UnloadKeys();
        }
    }

    delete [] indices;
    delete [] order;

    if (out_file != NULL) {
        WriteModels(models, num_images, out_file);
        // WriteModelsProjections(models, num_images, 
        //                        m_track_data, m_image_data, 
        //                        "pairs.proj.out");
        // WritePEdges(p_edges, num_images, "pedges.out");
    }
    fflush(stdout);

    // p_edges_out = p_edges;

    delete [] depends;
#if 0
    delete [] perm;
#endif
    delete [] connectivity;

    return models;
}

