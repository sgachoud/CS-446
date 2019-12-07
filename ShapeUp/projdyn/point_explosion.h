#pragma once

#include "projdyn_types.h"

namespace ProjDyn {

	class PointExplosion
	{
	public:
		PointExplosion(Vector3 center = Vector3(0,0,0), Scalar strength = 1.0,
			Scalar speed = 0.0)
			: m_center(center), m_strength(strength),
			m_detonated(false)
		{}

		~PointExplosion()
		{}

		Scalar getStrength() const {
			return m_strength;
		}

		void setStrength(Scalar strength) {
			m_strength = strength;
		}

		Vector3 const& getCenter() const {
			return m_center;
		}

		void setCenter(Vector3 newCenter) {
			m_center = newCenter;
		}

		void update(Scalar time_step) {
			if (m_detonated) m_detonated = false;
		}

		void detonate() {
			m_detonated = true;
		}

		/** Applies the explosion force to all positions and return the force matrix
		*/
		Positions applyOn(Positions positions) const {
			Positions explosionForces(positions.rows(), positions.cols());
			if (m_detonated) for (Index i(0); i < positions.rows(); i++) {
				Vector3 direction(positions.row(i) - m_center.transpose());
				explosionForces.row(i) = direction.normalized() * m_strength / direction.squaredNorm();
			}
			else explosionForces.setZero();
			return explosionForces;
		}

	private:
		Vector3 m_center;
		Scalar m_strength;
		bool m_detonated;
	};
}