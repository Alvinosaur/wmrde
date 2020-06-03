#include "wmrde/test.h"
//#include <wmrde/ode/test_ode.h>
//int main()
int main(int argc, char *argv[]) //use this for console output
{
	//options
	bool do_dyn = true; //do dynamic simulation, else kinematic
	bool ideal_actuators = true;
	bool do_anim = true; //do animation

	const Real dt = .04;
	const int nsteps = (int) floor(10.0/dt);
	Real time = 0;

	//for allocation
	const int MAXNS = NUMSTATE(WmrModel::MAXNF);
	const int MAXNV = NUMQVEL(WmrModel::MAXNF);
	const int MAXNY = MAXNS+MAXNV+WmrModel::MAXNA; //for dynamic sim

	//make WmrModel object
	WmrModel mdl;
	Real state[MAXNS];
	Real qvel[MAXNV]; //for dynamic sim

	//uncomment one of the following:
	//for animation, must also uncomment the corresponding scene function below
//	zoe(mdl,state,qvel);
	rocky(mdl,state,qvel);
//	talon(mdl,state,qvel);

	//also uncomment the corresponding scene function below!
	//initialize wheel-ground contact model
	mdl.wheelGroundContactModel(0, mdl.wgc_p, 0, 0, 0, //inputs
		0, 0); //outputs

	if (ideal_actuators)
		mdl.actuatorModel=0;

	//get from WmrModel
	const int nf = mdl.get_nf();
	const int nw = mdl.get_nw();
	const int nt = mdl.get_nt();
	const int ns = NUMSTATE(nf); //number of states
	//for dynamic sim
	const int nv = NUMQVEL(nf); //size of joint space vel
	const int na = mdl.get_na(); 
	int ny;

	//terrain
	SurfaceVector surfs;
	//flat(surfs);
	//ramp(surfs); //must also uncomment flat

	grid(surfs, ResourceDir() + std::string("gridsurfdata.txt") );

	//init contact geometry
	WheelContactGeom wcontacts[WmrModel::MAXNW];
	TrackContactGeom tcontacts[WmrModel::MAXNW];
	ContactGeom* contacts =0; //base class

	contacts = static_cast<ContactGeom*>(wcontacts);
	// } else if (nt > 0) {
	// 	sub_initTrackContactGeom(mdl, tcontacts);
	// 	contacts = static_cast<ContactGeom*>(tcontacts);
	// }
	initTerrainContact(mdl, surfs, contacts, state); //DEBUGGING
	//allocate
	Real y[MAXNY];
	Real ydot[MAXNY];

	//init y
	if (do_dyn) { //dynamic sim
		copyVec(ns,state,y);
		copyVec(nv,qvel,y+ns);
		setVec(na,0.0,y+ns+nv); //interr
		ny = ns + nv + na;
	} else {
		copyVec(ns,state,y);
		ny = ns;
	}
	//backup
	Real y0[MAXNY];
	copyVec(ny,y,y0); 

	//allocate
	HomogeneousTransform HT_parent[WmrModel::MAXNF];

	std::cout << "state(" << time << ")=\n"; printMatReal(ns,1,y,-1,-1); std::cout << std::endl;

	for (int i=0; i<nsteps; i++) {

		if (do_dyn) {
			odeDyn(time, y, mdl, surfs, contacts, dt, ydot, HT_parent);
		} else {
			odeKin(time, y, mdl, surfs, contacts, ydot, HT_parent);
		}
		addmVec(ny,ydot,dt,y);
		time += dt;

	}

	std::cout << "state(" << time << ")=\n"; printMatReal(ns,1,y,-1,-1); std::cout << std::endl;
}