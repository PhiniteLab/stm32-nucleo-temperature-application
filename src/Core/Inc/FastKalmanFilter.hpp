#ifndef __FAST_KALMAN_FILTER_HPP__
#define __FAST_KALMAN_FILTER_HPP__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}
#endif

class fast_kalman_filter_t {

private:

public:

	// system parameters
	double Bparam;
	double Fparam;
	double Qparam;
	double Hparam;
	double Rparam;
	double Kparam;

	// system dynamic parameters
	double XHatPresent;
	double XHatForward;
	double SPresent;
	double SForward;
	double PPresent;
	double PForward;
	double Y;


	double RefXHatPresent;
	double RefXHatForward;
	double RefSPresent;
	double RefSForward;
	double RefPPresent;
	double RefPForward;
	double RefY;


	fast_kalman_filter_t() {

	    this->Bparam = 0.001;
	    this->Fparam = 1;
	    this->Qparam = 0.0001;
	    this->Hparam = 1;
	    this->Rparam = 0.1;

	}

	fast_kalman_filter_t(double Qparam, double Rparam, double samplingPeriod,
			double PNStd, double MNstd, double initialValue) {

	    this->Fparam = 1;
	    this->Hparam = 1;
	    this->Y = 0.0;
	    this->Bparam = samplingPeriod;
	    this->Qparam = Qparam * PNStd * PNStd;
	    this->Rparam = Rparam * MNstd * MNstd;
	    this->PPresent = MNstd * MNstd;
	    // system dynamic parameters
	    this->XHatPresent = initialValue;
	    this->SPresent = MNstd * MNstd;

	    this->RefPPresent = MNstd * MNstd;
	    this->RefY = 0;
	    this->RefXHatPresent = initialValue;
	    this->RefSPresent = 0;

	}
	double get_tmp36_estimation(double measuredData, double FinputValue ){

	    // state
	    this->RefXHatForward = this->Fparam * this->RefXHatPresent + this->Bparam * FinputValue;

	    // state uncertainty
	    this->RefPForward = this->Fparam * this->RefPPresent * this->Fparam + this->Qparam;

	    // update
	    this->RefY = measuredData - this->Hparam * this->RefXHatForward;

	    // innovation
	    this->RefSForward = this->Hparam * this->RefPForward * this->Hparam + this->Rparam;

	    // coefficient
	    this->Kparam = this->RefPForward * this->Hparam * 1.0 / this->RefSForward;
	    this->RefXHatForward = this->RefXHatForward + this->Kparam * this->RefY;
	    this->RefPForward = (1.0 - this->Kparam * this->Hparam) * this->RefPForward;

	    // update previous values
	    this->RefPPresent = this->RefPForward;
	    this->RefXHatPresent = this->RefXHatForward;
	    this->RefSPresent = this->RefSForward;

	    return this->RefXHatForward;

	}

	double get_thermistor_estimation(double measuredData, double FinputValue){

	    // state
	    this->XHatForward = this->Fparam * this->XHatPresent + this->Bparam * FinputValue;

	    // state uncertainty
	    this->PForward = this->Fparam * this->PPresent * this->Fparam + this->Qparam;

	    // update
	    this->Y = measuredData - this->Hparam * this->XHatForward;

	    // innovation
	    this->SForward = this->Hparam * this->PForward * this->Hparam + this->Rparam;

	    // coefficient
	    this->Kparam = this->PForward * this->Hparam * 1.0 / this->SForward;
	    this->XHatForward = this->XHatForward + this->Kparam * this->Y;
	    this->PForward = (1.0 - this->Kparam * this->Hparam) * this->PForward;

	    // update previous values
	    this->PPresent = this->PForward;
	    this->XHatPresent = this->XHatForward;
	    this->SPresent = this->SForward;

	    return this->XHatForward;


	}

};

#endif
