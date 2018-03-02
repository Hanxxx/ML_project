// turn icons on
view_icons ON

// Declare the locations of important places:

//Military Hospital
define mil_hosp "-2050 -2990"

//Kuwait Regency Plaza 
define krp_hotel "3700 4100"

//US Embassy
define us_embassy "0 0"

//
// Definition of highway parameters
//
define highway "linearobject EDU.cmu.cs.coral.simulation.HighwaySim" 
//                     width   fg color  bg color  visionclass
define highway_params "  100    x009000   x000000            2"

//
// Definition of Road parameters
//
define road "linearobject EDU.cmu.cs.coral.simulation.RoadSim" 
//                     width   fg color  bg color  visionclass
define road_params    "  100    xFFFFFF   x000000            2"


//======
// SIMULATION BOUNDARY
//======
//
// bounds left right bottom top
//
// bounds statements set the bounds of the visible "playing field" in
// meters for a simulation.   If the aspect ratio of the bounds are not
// the same as the graphical area set aside by the simulation, then
// the robots may wander off the screen. 

bounds -25000 7200 -10000 12800


//======
// SEED
//======
//
// seed number
//
// The seed statement sets the random number seed.  The default is
// -1

seed 3


//======
// TIME
//======
//
// time accel_rate
//
// The time statement sets the rate at which simulation time progresses
// with respect to real time.  "time 0.5" will cause the simulation to
// run at half speed, "time 1.0" will cause it to run at real time,
// while "time 4.0" will run at 4 times normal speed.  Be careful
// about too high of a value though because the simulation will
// lose fidelity.  In fact, for slow computers, values less than 1.0
// may be necessary to avoid jumpy behavior.  Here, we try to run 
// at ten times real time.  In practicality, however, we are limited
// by the maxtimestep, below.

time 5.0  // one times realtime


//======
// TIMEOUT
//======
//
// timeout time
//
// The timeout statement indicates when the simulation will terminate in
// milliseconds.  The program automatically terminates when this time
// is reached.  If no timeout statement is given, the default is no
// termination.  NOTE: you *must* use a timeout with a trials statement.
//
// timeout 10000 // ten seconds


//======
// TRIALS
//======
//
// trials num_trials
//
// The trials statement indicates that the simulation should be run
// a certain number of times.  Each trial automatically terminates when the
// timeout time is reached, then a new trial is begun.  Note: certain hooks
// are available in the ControlSystem class for you to know when trials
// begin and end.  See the javadoc documentation.
//
// trials 100 // 100 trials


//======
// MAX TIME STEP
//======
//
// maxtimestep milliseconds
//
// maxtimestep statements set the maximum time (in milliseconds) that can
// transpire between discrete simulation steps.  This will keep the simulation
// from getting jumpy on slow machines, or when/if your process gets
// swapped out.

maxtimestep 100 // 1/10th of a second

//======
// WINDOWSIZE
//======
//
// windowsize width height
//
// The windowsize statement gives a default window size.  This can be
// overridden on the command line.

windowsize 664 470


//======
// BACKGROUND COLOR
//======
//
// background color
//
// A background statement sets the background color for the simulation.
// The color must be given in hex format as "xRRGGBB" where RR indicates
// the red component (00 for none, FF for full), GG is the green component,
// and BB is the blue.  Here we use white:

background xFFFFFF

//======
// BACKGROUND IMAGE
//======
//
// background_image imagefile
//
// A background_image statement sets the background image for the simulation.

background_image "metro.jpg"


//embassy
object EDU.gatech.cc.is.simulation.BinSim us_embassy 0 300 x0000c0 x000000 3

//mil_hosp
object EDU.gatech.cc.is.simulation.BinSim mil_hosp 0 300 x00FF00 x000000 3

//krp_hotel
object EDU.gatech.cc.is.simulation.BinSim krp_hotel 0 300 xFFFFFF x000000 3

//highway declarations


//Fifth Ring Motorway
define frm_a  "-24500 1000"
define frm_b  "-23000 800"
define frm_c  "-17900 900"
define frm_d  "-10200 1850"
define frm_e  "-8350  2050"
define frm_f  "-6800  2200"
define frm_g  "-4900  2400"
define frm_h  "-2600  2650"
define frm_i  " 200   3700"

highway frm_a frm_b highway_params
highway frm_b frm_c highway_params
highway frm_c frm_d highway_params
highway frm_d frm_e highway_params
highway frm_e frm_f highway_params
highway frm_f frm_g highway_params
highway frm_g frm_h highway_params
highway frm_h frm_i highway_params


//Sixth Ring Road
define srr_a  "-24500 1000"
define srr_b  "-23000 -1300"
define srr_c  "-19500 -2300"
define srr_d  "-16700 -2100"
define srr_e  " -7500 -1950"
define srr_f  " -5900 -2000"
define srr_g  " -2050 -2100"
define srr_h  "  -550 -1990"
define srr_i  "  3500 -1500"

highway srr_a srr_b highway_params
highway srr_b srr_c highway_params
highway srr_c srr_d highway_params
highway srr_d srr_e highway_params
highway srr_e srr_f highway_params
highway srr_f srr_g highway_params
highway srr_g srr_h highway_params
highway srr_h srr_i highway_params


//King Faisal Motorway
define kfm_a  "-7400 8300"
define kfm_b  "-7600 6800"
define kfm_c  "-7700 5400"
define kfm_d  "-7300 3800"
define kfm_e  "-6800 2200"
define kfm_f  "-6200 0"
define kfm_g  "-5900 -2000"
define kfm_h  "-5900 -3400"
define kfm_i  "-7700 -3200"
define kfm_j  "-7700 -3900"

highway kfm_a kfm_b highway_params
highway kfm_b kfm_c highway_params
highway kfm_c kfm_d highway_params
highway kfm_d kfm_e highway_params
highway kfm_e kfm_f highway_params
highway kfm_f kfm_g highway_params
highway kfm_g kfm_h highway_params
highway kfm_h kfm_i highway_params
highway kfm_i kfm_j highway_params


//Subhan Road
define sr_a  "-5900 -3400"
define sr_b  "-5900 -4500"
define sr_c  "-4800 -7000"
define sr_d  "-3700 -9000"
define sr_e  "-3800 -9700"
define sr_f  "-3000 -9700"
define sr_g  " 1400 -8400"

highway sr_a sr_b highway_params
highway sr_b sr_c highway_params
highway sr_c sr_d highway_params
highway sr_d sr_e highway_params
highway sr_e sr_f highway_params
highway sr_f sr_g highway_params


//Two Holy Mosques Rd
define thmr_a "-6200 8600"
define thmr_b "-5500 7500"
define thmr_c "-3700 4400"
define thmr_d "-2600 2650"
define thmr_e "-1100 -550"
define thmr_f "-550 -1990"
define thmr_g " 400 -5000"
define thmr_h "1400 -8400"

highway thmr_a thmr_b highway_params
highway thmr_b thmr_c highway_params
highway thmr_c thmr_d highway_params
highway thmr_d thmr_e highway_params
highway thmr_e thmr_f highway_params
highway thmr_f thmr_g highway_params
highway thmr_g thmr_h highway_params


//Fahaheel Expressway
define fe_a "-5000 10000"
define fe_b "-3900 9300"
define fe_c "-1800 7000"
define fe_d "-1000 5400"
define fe_e "200   3700"
define fe_e "1900 1300"
define fe_f "3500 -1500"
define fe_g "4200 -3600"
define fe_h "5400 -7500"
define fe_i "6100 -9500"

highway fe_a fe_b highway_params
highway fe_b fe_c highway_params
highway fe_c fe_d highway_params
highway fe_d fe_e highway_params
highway fe_e fe_f highway_params
highway fe_f fe_g highway_params
highway fe_g fe_h highway_params
highway fe_h fe_i highway_params


//Khaled Ben Abdul Ali Street
define kbaas_a "-1100 -550"
define kbaas_b "0 0"
define kbaas_c "1900 1300"

road kbaas_a kbaas_b road_params
road kbaas_b kbaas_c road_params


robot EDU.gatech.cc.is.abstractrobot.RescueVanSim 
	gorescue 0 500 1.0 xFFFFFF x0000c0 2 
robot EDU.gatech.cc.is.abstractrobot.RescueVanSim 
	gorescue 1500 -500 1.0 xFFFFFF x0000c0 2 
robot EDU.gatech.cc.is.abstractrobot.RescueVanSim 
	gorescue2 500 0 1.0 xFFFFFF x0000c0 2 

//hurt person
object EDU.cmu.cs.coral.simulation.AttractorHurtPersonSim -4000 5000 0 1 x0000c0 xc00000 0
object EDU.cmu.cs.coral.simulation.AttractorHurtPersonSim -4200 5000 0 1 x0000c0 xc00000 0
object EDU.cmu.cs.coral.simulation.AttractorHurtPersonSim -3800 5100 0 1 x0000c0 xc00000 0
object EDU.cmu.cs.coral.simulation.AttractorHurtPersonSim -3900 4900 0 1 x0000c0 xc00000 0

//regular person
object EDU.cmu.cs.coral.simulation.AttractorPersonSim krp_hotel 0 1 x0000c0 xc00000 0


