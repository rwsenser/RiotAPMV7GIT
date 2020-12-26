//
// mission.h
//
// 2020-10-10: Add WIGWAM_MULT_FACTOR to hold the rudder
//             multiplier used by wigwam.h
//
// 2020-06-24 Merge Completed
//
#ifdef P_TESTBED
#define WIGWAM_MULT_FACTOR 4
#else
#define WIGWAM_MULT_FACTOR 2
#endif

// WHICH PLANE?????????
#ifdef P_RIOTIII  
//                            action action  max    mission mission       jump num   {step   step   step}       
//                            time   cmd     alt    purpose name               steps  head   alt    durr
//                            (ms)          (ft)                          (ft)        (deg)  (ft)   (ms) 
// very simple (02/17/2020) climb 25 feet and same course!!!
const planeMission mission = {5000, A_TEMP, 6200, O_HOLD, "M_RIOTIII_1", 25,  1,    DYN_VAL, DYN_VAL, 0};   // <===== !!
#endif

//################################

   
