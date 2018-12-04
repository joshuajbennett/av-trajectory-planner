#ifndef AVCONVERSIONS_HPP
#define AVCONVERSIONS_HPP

#include "AvStructs.hpp"

#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xview.hpp"

namespace av_conversions
{

// Convert an AvState to xtensor format.
xt::xarray<double> av_state_to_xtensor(av_structs::AvState state)
{
	xt::xarray<double> state_out {state.x, state.y, state.psi, state.delta_f, state.vel_f};

	state_out.reshape({5, 1});

	return state_out;
}

av_structs::AvState xtensor_to_av_state(xt::xarray<double> state)
{
	av_structs::AvState state_out {state(0, 0), state(1, 0), state(2, 0), state(3, 0), state(4, 0)};

	return state_out;
}
} // namespace av_conversions

#endif
