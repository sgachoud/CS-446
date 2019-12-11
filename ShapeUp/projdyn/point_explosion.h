#pragma once

#include "projdyn_types.h"
#include <nanogui/common.h>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795028841971
#endif // !M_PI


namespace ProjDyn {
	using MatrixXd = Eigen::MatrixXd;
	using MatrixXu = nanogui::MatrixXu;

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

	class DrawableSphere {
	public:
		DrawableSphere(size_t precision = 360, double radius = 1.0) {
			setPrecision(precision);
			scale(radius);
		}

		void setPrecision(size_t precision) {
			m_precision = precision;
			squareToUniformSphere(m_unifSpherePoints, m_unifSphereNormals, m_unifSphereIndices, precision);
		}

		size_t getNumFace() const {
			return (m_precision - 1) * (m_precision - 1);
		}

		void translate(Vector3 translation) {
			translateSphere(m_unifSpherePoints, translation);
		}

		void translateTo(Vector3 position) {
			translate(position - m_center);
		}

		void scale(double ratio) {
			for (Index index(0); index < m_unifSpherePoints.cols(); index++) {
				m_unifSpherePoints.col(index) = m_center + ratio * (m_unifSpherePoints.col(index) - m_center);
			}
		}

		void scaleTo(double size) {
			for (Index index(0); index < m_unifSpherePoints.cols(); index++) {
				m_unifSpherePoints.col(index) = m_center + size * m_unifSphereNormals.col(index);
			}
		}

		MatrixXd const& getSpherePointsGrid() {
			return m_unifSpherePoints;
		}

		MatrixXd const& getSphereNormalsGrid() {
			return m_unifSphereNormals;
		}

		MatrixXu const& getSphereIndicesGrid() {
			return m_unifSphereIndices;
		}

	private:
		static void squareToUniformSphere(MatrixXd& unifSpherePoints, MatrixXd& unifSphereNormals,
			MatrixXu& unifSphereIndices, size_t size) {
			unifSpherePoints = MatrixXd(3, size * size);
			unifSphereNormals = MatrixXd(3, size * size);
			unifSphereIndices = MatrixXu(3, size * (size - 1) * 2);
			for (Index x(0); x < size; x++) {
				for (Index y(0); y < size; y++) {
					Index index(x * size + y);
					unifSpherePoints.col(index) = squareToUniformSphere(float(x) / (size - 1), float(y) / (size - 1));
					unifSphereNormals.col(index) = unifSpherePoints.col(index).normalized();
				}
			}

			//generate faces
			for (Index x(0); x < size - 1; x++) {
				for (Index y(0); y < size - 1; y++) {
					Index index(x * size + y);
					Index cell_ind(2 * index);
					unifSphereIndices(0, cell_ind) = index;
					unifSphereIndices(1, cell_ind) = (x + 1) * size + y;
					unifSphereIndices(2, cell_ind) = index + 1;
					unifSphereIndices(0, cell_ind + 1) = (x + 1) * size + y;
					unifSphereIndices(1, cell_ind + 1) = (x + 1) * size + (y + 1);
					unifSphereIndices(2, cell_ind + 1) = index + 1;
				}
			}

			//fixes boundary
			/*for (Index x(0); x < size - 1; x++) {
				Index y(size - 1);
				Index cell_ind(2 * (x * size + y));
				unifSphereIndices(0, cell_ind) = x * size + y;
				unifSphereIndices(1, cell_ind) = (x + 1) * size + y;
				unifSphereIndices(2, cell_ind) = x * size;
				unifSphereIndices(0, cell_ind + 1) = (x + 1) * size + y;
				unifSphereIndices(1, cell_ind + 1) = (x + 1) * size;
				unifSphereIndices(2, cell_ind + 1) = x * size;
			}*/
			/*for (Index y(0); y < size - 1; y++) {
				Index x(size - 1);
				Index cell_ind(2 * (x * size + y));
				unifSphereIndices(0, cell_ind) = x * size + y;
				unifSphereIndices(1, cell_ind) = y;
				unifSphereIndices(2, cell_ind) = x * size + y + 1;
				unifSphereIndices(0, cell_ind + 1) = y;
				unifSphereIndices(1, cell_ind + 1) = y + 1;
				unifSphereIndices(2, cell_ind + 1) = x * size + y + 1;
			}*/
			/*
			Index cell_ind(2 * (size * size - 1));
			unifSphereIndices(0, cell_ind) = size * size - 1;
			unifSphereIndices(1, cell_ind) = size - 1;
			unifSphereIndices(2, cell_ind) = (size - 1) * size;
			unifSphereIndices(0, cell_ind + 1) = size - 1;
			unifSphereIndices(1, cell_ind + 1) = 0;
			unifSphereIndices(2, cell_ind + 1) = (size - 1) * size;
			*/
		}

		static Vector3 squareToUniformSphere(double x, double y) {
			float z = 2 * x - 1;
			float r = sqrt(1 - z * z);
			float theta = y * 2 * M_PI;
			return Vector3(r * cos(theta), r * sin(theta), z);
			/*float theta = x * 2 * M_PI;
			float phi = y * M_PI;
			return Vector3(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));*/
		}

		static void translateSphere(MatrixXd& unifSpherePoints, Vector3 translation) {
			for (Index index(0); index < unifSpherePoints.cols(); index++) {
				unifSpherePoints(0, index) += translation(0);
				unifSpherePoints(1, index) += translation(1);
				unifSpherePoints(2, index) += translation(2);
			}
		}

		static void expandSphere(MatrixXd& unifSpherePoints, MatrixXd const& unifSphereNormals, float expantion) {
			for (Index index(0); index < unifSpherePoints.cols(); index++) {
				unifSpherePoints.col(index) += expantion * unifSphereNormals;
			}
		}

	private:
		Vector3 m_center = { 0.0, 0.0, 0.0 };
		size_t m_precision;
		MatrixXd m_unifSpherePoints;
		MatrixXd m_unifSphereNormals;
		MatrixXu m_unifSphereIndices;
	};
}