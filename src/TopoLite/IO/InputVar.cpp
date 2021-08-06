//
// Created by ziqwang on 10.03.19.
//
#include "InputVar.h"

void InitVar_Assembly3D(InputVarList *varList){
    varList->clear();

    //Assembly2DBoundary.cpp
    varList->add(false, "previewMode", "previewMode") = "Visible";
    varList->add(1.0, "block_density", "block_density");
    varList->add(10.0 / 180 * M_PI, Eigen::Vector2f(5.0 / 180 * M_PI, 20.0 / 180 * M_PI),
                 "minimum_assembling_cone_angle", "min. Angle");
    varList->add(0.01, "min_contact_area", "min_contact_area");
    varList->add(10, "contact_sample", "contact_sample");
    varList->add(0.0 / 180 * M_PI, "gravity_tilting_angle", "gravity_tilting_angle") = "Visible";
    varList->add(false, "remove_last_part_from_assembling_sequence", "remove_last_part_from_assembling_sequence");

    //ImplicitBlock_3DBoundary.cpp
    varList->add(10, "block_rendering_preview_sample", "block_rendering_preview_sample");
    varList->add(100, "block_rendering_none_preview_sample", "block_rendering_none_preview_sample");

    //ImplicitFaceBicubic
    varList->add(0.0, Vector2f(-M_PI / 3.0, M_PI / 3.0), "face_bicubic_angle_y", "face_bicubic_angle_y");
    varList->add(0.0, Vector2f(-M_PI/2, M_PI /2), "face_bicubic_angle_z", "face_bicubic_angle_z");
    varList->add(0.2, Vector2f(0.1, 0.4), "face_bicubic_sx", "face_bicubic_sx");
    varList->add(0.2, Vector2f(0.1, 0.4), "face_bicubic_sy", "face_bicubic_sy");
    varList->add(0.0, Vector2f(-0.3, 0.3), "face_bicubic_h", "face_bicubic_h");
    varList->add(0.5,  "face_bicubic_maxz", "face_bicubic_maxz");

    //ImplicitFaceBicubic_MultiContact
    varList->add(1E6, "face_bicubic_max_width", "face_bicubic_max_width");


    //MotionCone3D
    varList->add(5, "approx_motion_cone_sample", "approx_motion_cone_sample");
    varList->add(0.0, "approx_motion_cone_bound_refine_coeff" , "approx_motion_cone_bound_refine_coeff");

    //DiffConceptualDesign
    varList->add(false, "activate_tilting_stability", "activate_tilting_stability");
    varList->add(M_PI / 4, "tilting_angle", "tilting_angle");


    //DiffImplicitFaceBicubic_MultiContacts
    varList->add(10, "diff_face_contact_sample", "diff_face_contact_sample");

    //DiffSurfaceRealization
    varList->add(0.0 / 180 * M_PI, "assemblability_tolerance_angle", "assemblability_tolerance_angle" );

    varList->add(5, "uniform_sample_geometry_search", "uniform_sample_geometry_search");
    varList->add(2, "two_points_sample_geometry_search", "two_points_sample_geometry_search");
    varList->add(2, "one_point_sample_geometry_search", "one_point_sample_geometry_search");

    //DiffMotionCone
    varList->add(false, "motion_cone_simplest_version", "motion_cone_simplest_version");

    //Assembly3DBoundary_Opt.cpp
    varList->add(1e-6, "equilibrium_infeasibility_tol", "equilibrium_infeasibility_tol");
    varList->add(1e-6, "geometric_design_tol", "equilibrium_infeasibility_tol");

    varList->add(0.0, "conceptual_design_regularization_ceoff", "conceptual_design_regularization_ceoff");

    varList->add(10, "conceptual_geometric_iteration", "conceptual_geometric_iteration");
    varList->add(200, "conceptual_design_iteration", "conceptual_design_iteration");
    varList->add(50, "geometric_design_iteration", "geometric_design_iteration");
}

void InitVar_Assembly2D(InputVarList *varList){
    varList->clear();

    //Assembly2DBoundary.cpp
    varList->add(false, "previewMode", "previewMode") = "Visible";
    varList->add(1.0, "block_density", "block_density");
    varList->add(10.0 / 180 * M_PI, Eigen::Vector2f(5.0 / 180 * M_PI, 20.0 / 180 * M_PI),
                 "minimum_assembling_cone_angle", "min. Angle");
    varList->add(0.01, "minimum_segment_length", "minimum_segment_length");

    //ImplicitBlock_2DBoundary.cpp
    varList->add(50, "block_rendering_preview_sample", "block_rendering_preview_sample");
    varList->add(100, "block_rendering_none_preview_sample", "block_rendering_none_preview_sample");
    varList->add(50, "block_collision_region_sample", "block_collision_region_sample");
    varList->add(50, "contact_sample", "contact_sample");
    varList->add((std::string)"tenon", "contact_init_joint_type", "contact_init_joint_type");


    //CurveEdgeJointBase
    varList->add(0.0, Vector2f(-M_PI/180 * 30.0, M_PI/180 * 30.0), "joint_based_tilting_angle", "joint_based_tilting_angle");
    varList->add(0.3, "joint_sum_of_all_w", "joint_sum_of_all_w");
    varList->add(0.2, "minimum_joint_length", "minimum_joint_length");


    //CurvedEdgeSplineTenon
    varList->add(0.01, Vector2f(0.01, 1), "tenon_joint_w1", "tenon_joint_w1");
    varList->add(0.01, Vector2f(0.01, 1), "tenon_joint_w2", "tenon_joint_w2");
    varList->add(0.0, Vector2f(-0.5, 0.5), "tenon_joint_h", "tenon_joint_h");

    //CurvedEdgeSplineWave
    varList->add(0.01, Vector2f(0.01, 1), "wave_joint_w1", "wave_joint_w1");
    varList->add(0.01, Vector2f(0.01, 1), "wave_joint_w2", "wave_joint_w2");
    varList->add(0.3, Vector2f(0.1, 1), "wave_joint_w3", "wave_joint_w2");
    varList->add(0.0, Vector2f(-0.5, 0.5), "wave_joint_h", "wave_joint_h");

    //MotionCone2D.cpp
    varList->add(1E2, "max_cone_size", "max_cone_size");
    varList->add(10, "motion_cone_bound_sample", "motion_cone_bound_sample");

    //DiffMotionCone2D
    varList->add(0.2, "approx_motion_cone_bound_refine_coeff", "approx_motion_cone_bound_refine_coeff");

    //DiffAssemblabilityCone2D
    varList->add(0.0 / 180 * M_PI, "assemblability_tolerance_angle", "assemblability_tolerance_angle" );

    //DiffCurveRealization.cpp
    varList->add(5, "search_init_geometry_sample_uniform", "search_init_geometry_sample_uniform");
    varList->add(2, "search_init_geometry_sample_two_points", "search_init_geometry_sample_two_points");

    varList->add(0.0, "min_collision_distance", "min_collision_distance");

    //Assembly2DBoundary_Opt.cpp
    varList->add((double)1E-6, "equilibrium_infeasibility_tol", "equilibrium_infeasibility_tol");
    varList->add((double)1E10, "geometric_design_max_error_tol", "geometric_design_max_error_tol");

    varList->add(1e-6, "geometric_design_tol", "geometric_design_tol");
    varList->add((double)0.0, "conceptual_design_regularization_ceoff", "conceptual_design_regularization_ceoff");

    varList->add(10, "conceptual_geometric_iteration", "conceptual_geometric_iteration");
    varList->add(200, "conceptual_design_iteration", "conceptual_design_iteration");
    varList->add(50, "geometric_design_iteration", "geometric_design_iteration");
}

void InitVar_backward(InputVarList *varList)
{
    varList->clear();

    // Show_TI_Structure
    varList->add(true, "showStruc", "Struc [F1]") = "Show_TI_Structure";
    varList->add(true, "showStrucWire", "Wireframe [F2]") = "Show_TI_Structure";
    varList->add(false, "showPickParts", "Show Select") = "Show_TI_Structure";
    varList->add(false, "showStruc3DText", "Block IDs") = "Show_TI_Structure";
    varList->add(false, "showStrucBBox", "StrucBBox") = "Show_TI_Structure";
    //	varList->add(false, "showStrucNormal", "StrucNormal") =  "Show_TI_Structure";
    //	varList->add(false, "showStrucGraph", "StrucGraph") =  "Show_TI_Structure";

    // Show_Input_Model
    varList->add(false, "showModel", "Model") = "Show_Input_Model";
    varList->add(false, "showModelTex", "Texture") = "Show_Input_Model";
    varList->add(false, "showAxes", "Axes") = "Show_Input_Model";
    varList->add(false, "showGround", "Ground") = "Show_Input_Model";
    varList->add(false, "showGravity", "Gravity") = "Show_Input_Model";
    varList->add(false, "lockTiltAngle", "Lock Angle") = "Show_Input_Model";

    // Show_Base_Mesh
    varList->add(false, "showCross3D", "Tessellation [F3]") = "Show_Base_Mesh";
    varList->add(false, "showCrossTilt", "Augm. Vectors") = "Show_Base_Mesh";
    varList->add(false, "showCrossTiltBase", "TiltBase") = "Show_Base_Mesh";
    varList->add(false, "showCrossTiltFan", "TiltFan") = "Show_Base_Mesh";
    // varList->add(false, "showCrossDual", "CrossDual") =  "Show_Base_Mesh";
    // varList->add(false, "showCrossVertex", "CrossVertex") =  "Show_Base_Mesh";
    varList->add(false, "showInnerPolys", "InnerPolys") = "Show_Base_Mesh";

    // Show_TI_construction
    varList->add(false, "showGeomFace", "GeomFace") = "Show_TI_Construction";
    varList->find("showGeomFace")->visible = false;
    varList->add(false, "showGeomVertex", "GeomVertex ") = "Show_TI_Construction";

    // Show_TI_Mobility
    varList->add(false, "showMobiliFace", "Face\t") = "Show_TI_Mobility";
    varList->add(false, "showMobiliEdge", "Edge") = "Show_TI_Mobility";
    varList->add(false, "showMobiliVertex", "Vertex") = "Show_TI_Mobility";
    varList->add(false, "showMobiliRay", "Rays\t") = "Show_TI_Mobility";
    varList->add(false, "showMobiliMesh", "Mesh") = "Show_TI_Mobility";
    varList->add(false, "showMobiliVec", "Vector") = "Show_TI_Mobility";

    // Show TI Assembly
    varList->add(false, "showStrucContact", "Contact [F4]") = "Show_TI_Assembly";
    varList->add(false, "showEdgeContctNrm", "EdgeContctNrm") = "Show_TI_Assembly";
    varList->add(false, "showStrucForces", "Tension [F5]") = "Show_TI_Assembly";
    varList->add(false, "showGravityCone", "Feasible Cone [F6]") = "Show_TI_Assembly";
    varList->add(false, "showCompression", "Compression Only") = "Show_TI_Assembly";
    varList->add(false, "showOptColorMap", "Opt ColorMap") = "Show_TI_Assembly";
    varList->add(false, "showOptGraident", "Opt Gradient") = "Show_TI_Assembly";
    varList->add(false, "mult_move", "Mult. Move") = "Show_TI_Assembly";

    // Para Basic
    varList->add(4, Vector2f(1, 15), "patternID", "Pattern\t:") << "Pattern_ID" = "Para_Basic";
    varList->add(20.0f, Vector2f(0, 90), "tiltAngle", "Alpha\t:") << "Tile_Angle_Upper" = "Para_Basic";
    varList->add(0.02f, Vector2f(0, 0.2), "cutUpper", "Upper Depth\t:") << "Cut_Plane_Height" = "Para_Basic";
    varList->add(0.02f, Vector2f(0, 0.2), "cutLower", "Lower Depth\t:") << "Cut_Plane_Height" = "Para_Basic";
    varList->add((int)15, Vector2f(10, 30), "patternRadius", "Pattern Radius\t:") << "Pattern_Radius" = "Para_Basic";
    varList->add(0.001f, Vector2f(0, 0.1), "minCrossArea", "Minimum Cross Area (%)\t:") = "Para_Basic";
    varList->add(0.03f, Vector2f(0, 0.5), "minBoundaryEdge", "Minimum Bdry Edge\t:") = "Para_Basic";
    varList->add(1.0f, Vector2f(0, 1), "textureScaleFactor", "Texture Scale Factor\t:") = "Para_Basic";
    varList->add((int)(10), Vector2f(0, 32), "slopSample", "Slop Sample\t:") = "Para_Basic";
    varList->add(100.0f, Vector2f(0, 1000), "Unit", "Fabrication Scale\t:") = "Para_Basic";
    varList->add((float)1, Vector2f(0, 10), "mult_move_scale", "Mult Move\t:") = "Para_Basic";

    varList->add(true, "smooth_bdry", "Extend bdry") = "Para_Basic";
    varList->add(true, "ground_touch_bdry", "Smooth bdry") = "Para_Basic";
    varList->add(true, "faceface_contact", "Face Face") = "Para_Basic";
    varList->add(true, "edgeedge_contact", "Edge Edge") = "Para_Basic";

    varList->add(false, "only_cut_bdry", "Only Cut bdry") = "Para_Basic";
    varList->add(false, "texturedModel", "Textured Model") = "Para_Basic";
    varList->add(false, "penetration_check", "Penetration Check") = "Para_Basic";
    varList->add(1e-7f, "small_zero_eps", "") = "Para_Basic";
    varList->add(1e-5f, "big_zero_eps", "") = "Para_Basic";
    varList->add(1e8f, "clipper_scale", "") = "Para_Basic";
    varList->add(0.0f, "tilt_face_angle_min", "") = "Para_Basic";
    varList->find("tilt_face_angle_min")->visible = false;
    varList->add(0.0f, "tilt_face_angle_max", "") = "Para_Basic";
    varList->find("tilt_face_angle_max")->visible = false;

    varList->find("ground_touch_bdry")->visible = true;
    varList->find("smooth_bdry")->visible = true;
    varList->find("only_cut_bdry")->visible = false;
    varList->find("big_zero_eps")->visible = false;
    varList->find("small_zero_eps")->visible = false;
    varList->find("clipper_scale")->visible = false;
    varList->find("texturedModel")->visible = false;
    varList->find("penetration_check")->visible = false;

    // ShapeOp Para
    varList->add(1.0f, Vector2f(0, 2), "shapeop_meshFit", "Mesh Fitness          \t:") = "Para_ShapeOp";
    varList->add(1.0f, Vector2f(0, 2), "shapeop_regularity", "Regularity            \t:") = "Para_ShapeOp";
    varList->add(1.0f, Vector2f(0, 2), "shapeop_planarity", "Planarity             \t:") = "Para_ShapeOp";
    varList->add(0.0f, Vector2f(0, 2), "shapeop_bending", "Bending               \t:") = "Para_ShapeOp";
    varList->add(0.0f, Vector2f(0, 2), "shapeop_boundary", "Boundary              \t:") = "Para_ShapeOp";
    varList->add((int)0, Vector2f(0, 300), "shapeop_iterator", "Iterations              \t:") = "Para_ShapeOp";

    // Mitsuba Para
    varList->add(false, "output_mitsuba", "Output Mitsuba") = "Para_Mitsuba";
    varList->add(0.001f, Vector2f(0, 0.01), "wireframe_thickness", "Wireframe Thinkness") = "Para_Mitsuba";
    varList->add((int)16, Vector2f(0, 32), "wireframe_circle_vertex_num", "Wireframe Subdivision") = "Para_Mitsuba";
    varList->add(50.0f, Vector2f(0, 360), "mitsuba_PartYRotation", "Part Rotation Y\t:") = "Para_Mitsuba";
    varList->add(30.0f, Vector2f(0, 180), "mitsuba_CameraRotation", "Camera Rotation\t:") = "Para_Mitsuba";
    varList->add(4.0f, Vector2f(0, 7), "mitsuba_CameraDistance", "Camera Distance\t:") = "Para_Mitsuba";
    varList->add((int)768, Vector2f(0, 5000), "mitsuba_width", "Render Width\t:") = "Para_Mitsuba";
    varList->add((int)576, Vector2f(0, 5000), "mitsuba_height", "Render Height\t:") = "Para_Mitsuba";
    varList->add((int)128, Vector2f(0, 1024), "mitsuba_sample", "Render Sample\t:") = "Para_Mitsuba";
    varList->add(12.0f, Vector2f(0, 24), "mitsuba_SunTime", "Sun Time\t:") = "Para_Mitsuba";
    varList->add(5.0f, Vector2f(0, 10.0), "mitsuba_SunStrength", "Sun Strength\t:") = "Para_Mitsuba";
    varList->find("mitsuba_SunTime")->visible = false;
    // varList->find("mitsuba_SunStrength")->visible = false;

    // Interlocking Para
    varList->add(1e-12f, "mosek_intpntCoTolRelGap", "IntPnt RelGap") = "Para_ContactGraph";
    varList->find("mosek_intpntCoTolRelGap")->visible = false;
    varList->add(1e-12f, "mosek_intpntCoTolInfeas", "") = "Para_ContactGraph";
    varList->find("mosek_intpntCoTolInfeas")->visible = false;
    varList->add(1e-7f, "mosek_interlocking_eps", "Interlock Eps") = "Para_ContactGraph";
    varList->find("mosek_interlocking_eps")->visible = false;
    varList->add(1e-7f, "mosek_rbe_eps", "RBE Eps") = "Para_ContactGraph";
    varList->find("mosek_rbe_eps")->visible = false;
    varList->add(0.0f, Vector2f(0, 1), "mosek_rbe_friction_coeff", "Friction") = "Para_ContactGraph";
    varList->add(1e7f, "mosek_rbeForce_upperBound", "") = "Para_ContactGraph";
    varList->find("mosek_rbeForce_upperBound")->visible = false;
    varList->add(0.1f, "slope_binarySearch_eps", "") = "Para_ContactGraph";
    varList->find("slope_binarySearch_eps")->visible = false;
    varList->add(0.1f, "slope_bound_eps", "") = "Para_ContactGraph";
    varList->find("slope_bound_eps")->visible = false;
    varList->add(int(4), "slope_num_init_sample", "") = "Para_ContactGraph";
    varList->find("slope_num_init_sample")->visible = false;
    varList->add((int)100, "kmeans_iteration", "") = "Para_ContactGraph";
    varList->find("kmeans_iteration")->visible = false;
    varList->add(0.0f, Vector2f(-1, 1), "GravityX", "Gravity X") = "Para_ContactGraph";
    varList->add(-1.0f, Vector2f(-1, 1), "GravityY", "Gravity Y") = "Para_ContactGraph";
    varList->add(0.0f, Vector2f(-1, 1), "GravityZ", "Gravity Z") = "Para_ContactGraph";

    // CrossMesh_Para
    varList->add(1e-2f, "crossmesh_quadTree_leafNodeSize", "") = "Para_CrossMesh";

    // Optimization_Para
    varList->add(0.01f, "opt_compression_weight", "") = "Para_Opt";
    varList->add(10000.0f, "opt_tension_weight", "") = "Para_Opt";
    varList->add(1e-4f, "opt_activeset_eps", "") = "Para_Opt";
    varList->add(1e2f, "opt_gravity_scale_factor", "") = "Para_Opt";
    varList->add(1e-7f, "opt_knitro_opttol", "") = "Para_Opt";
    varList->add((int)200, "opt_iterations_times", "") = "Para_Opt";
    varList->add(30.0f, "opt_tiltAngle_width", "") = "Para_Opt";
    varList->add((int)0, "opt_mod", "") = "Para_Opt";
    varList->add((float)1e-3, "opt_init_levelset", "") = "Para_Opt";
    varList->add((float)0.2, "opt_radius_scale_factor", "") = "Para_Opt";
    varList->add((float)0.01, "opt_rs_factor_lower_bound", "") = "Para_Opt";
    varList->add((int)5, "opt_poly_size", "") = "Para_Opt";
    varList->add((int)10, "opt_edge_sample", "") = "Para_Opt";
    varList->add((float)0, "opt_tot_time", "") = "Para_Opt";

    // Assembly_Para
    varList->add(false, "showAssembly", "Show Assembly") = "Para_Assembly";
    varList->add((int)1, Vector2f(1, 30), "assembling_layer", "Num Layer") = "Para_Assembly";
    varList->add((int)(-1), Vector2f(-2, 200), "assembling_Kth_part", "Kth Part") = "Para_Assembly";
    varList->add(0.2f, Vector2f(-2, 200), "assembling_vec_length", "Vec Length") = "Para_Assembly";

    varList->add(false, "showSupport", "Show Support") = "Para_Support";
    varList->add(0.0f, Vector2f(-1, 1), "ground_height", "Ground") = "Para_Support";
    varList->add(0.1f, Vector2f(0, 1), "support_thickness", "Thickness") = "Para_Support";
    varList->add((int)(-1), Vector2f(-2, 100), "support_layer", "Num Layer") = "Para_Support";
    varList->add(0.01f, Vector2f(0, 1), "support_smallest_area", "Smallest Area") = "Para_Support";
}

void InitVar(InputVarList *varList)
{

    //Pattern
    {
        varList->add(4, Vector2f(1, 12), "patternID", "Ptn. ID") = "Pattern";
        varList->add((int)15, Vector2f(10, 30), "patternRadius", "Ptn. Radius") = "Pattern";

    }

    //Cross Mesh
    {
        varList->add((int)1, Vector2f(1, 5), "layerOfBoundary", "L. Brdy") = "Cross_Mesh";
        varList->add((double)0.001, Vector2f(0, 0.1), "minCrossArea", "Min. Cross") = "Cross_Mesh";
        varList->add(true, "smooth_bdry", "Smooth Brdy") = "Cross_Mesh";
        varList->add((double)1, "textureScaleFactor", "Texture Scale Factor") = "Cross_Mesh";
        varList->add(true, "ground_touch_bdry", "Touch Grd.") = "CrossMesh";
    }

    //Block
    {
        varList->add((double)20.0, Vector2f(0, 40), "tiltAngle", "Aug. Angle") = "Block";
        varList->add((double)0.05, Vector2f(0, 0.14), "cutUpper", "Cut Up.") = "Block";
        varList->add((double)0.05, Vector2f(0, 0.14), "cutLower", "Cut Lo.") = "Block";
    }

    //CurvedBlock

    {
        //Curved Edge Base
        {
        varList->add((int) 10, "CurvedEdgeBase_CurveCurveIntersection_InitialSegments",
                     "The initial number of segments to compute curve curve intersection");
        varList->add((double) 1e-5, "CurveEdgeBase_PlaneIntersection_BisectionEPS",
                     "The Epsilon of bisection method for the curve-plane intersection");
        varList->add((int) 50, "CurvedEdgeBase_EvaluateBBox_Segments",
                     "The number of segment to compute the curve's bounding box");
        }

        //Curved Face Base
        {
            varList->add((double) 0.01, "CurvedFaceBase_MaximumBorderSegmentLength",
                         "The maximum edge length when discretizing a curve in uv space");
            varList->add((int) 200, "CurvedFaceBase_MaximumODEIterations",
                         "The maximum iteration for solving ode when computing surface-surface intersection");
            varList->add((double) 0.005, "CurvedFaceBase_MaximumODEStep_ArcLength", "");
            varList->add((double) 0.0005, "CurvedFaceBase_MinimumODEStep_ArcLength", "");
        }

        //Curved Block
        {
            varList->add((double) 1e-3, "CurvedBlock_FaceTriangulationArea", "the maxmium area for each triangulated faces");
        }

        //Impicit Face
        {
            varList->add((int) 1, Eigen::Vector2f(0, 2),"MarchingSquares_Type", "Uni/Quad/Volume");
        }
    }

    //Debug UI
    {
        varList->add(true, "previewMode", "Preview Mode");
        varList->add((double)0.1, Vector2f(0.05, 0.15), "base_poly_radius", "Base Radius");
        //varList->add((float )0.16, Vector2f(0.16, 0.53), "based_curve_w", "Wire Width");
        varList->add((double )0.16, Vector2f(0.01, 0.53), "based_curve_w", "Wire Width");
        varList->add((double )-1, Vector2f(-1, 1), "based_curve_h", "Wire Height");

        varList->add((double)1, Vector2f(0.5 , 1), "trajectory_curve_w", "Traj. Width");
        varList->add((double)0.01, Vector2f(0.01, 0.08), "trajectory_curve_h", "Traj. Height");
        varList->add((double)0, Vector2f(-1 , 1), "trajectory_curve_t", "Traj. Trans");

        varList->add((int)0, Vector2f(0, 1), "trajectory_type", "Traj. Type");
        varList->add((int)6, Vector2f(3, 8), "cross_edge_num", "Num. Edge");

        {
            varList->add((double)0, Vector2f(-0.14, 0.14), "cross_layer_height", "Cross Height");
            varList->add((int)(-1),Vector2f(-1, 100),  "debug_id", "debugID");
            varList->add((int)20,Vector2f(0, 100),  "num_sample", "num. sample");
        }

    }
}
