// DGP 2019 Project
// ShapeUp and Projective Dynamics
// Author: Christopher Brandt

// This file contains the Constraint class, which implements ShapeUp
// and Projective Dynamics style constraints.
// First, an abstract Constraint class is defined, which acts as a
// blueprint for specific constraints.
// Then, a few specific constraints are implemented.

#pragma once

#include "projdyn_types.h"
#include "projdyn_common.h"
#include <limits>

// Vertices cannot be assigned less weight then this:
constexpr double PROJDYN_MIN_WEIGHT = 1e-6;
// If set to true, the TriangleBendingConstraints prevent the mean curvature
// normal from pointing into the opposite direction as in their original state
constexpr bool PROJDYN_PREVENT_BENDING_FLIPS = false;
// The flipping prevention above is only enforced when the total mean curvature is
// above the value below
constexpr ProjDyn::Scalar PROJDYN_BENDING_FLIP_THRESHOLD = 1e-5;

namespace ProjDyn {

    /* Constraints are grouped into ConstraintGroups, which can have a name, and an additional multiplier.
       This allows for easier accessing from outside. */
    struct ConstraintGroup {
        ConstraintGroup(std::string groupName, const std::vector<ConstraintPtr>& groupConstraints, Scalar groupWeight) {
            name = groupName;
            constraints = groupConstraints;
            weight = groupWeight;
        }

        std::string name = "Unknown";
        std::vector<ConstraintPtr> constraints;
        Scalar weight = 1.;
    };

    /* Blueprint of a Projectiv Dynamics constaint (weight, p, A) */
    class Constraint {
    public:
        /** Constraint constructor.
                vertex_indices - a list of indices of vertices that take part in this constraint
        */
        Constraint(const std::vector<Index>& vertex_indices, Scalar weight) {
            m_vertex_indices = vertex_indices;
            m_weight = (std::max)(PROJDYN_MIN_WEIGHT, weight);
        }

        virtual ~Constraint() {}

        /** Performs the constraint projection
                positions - current positions of the mesh vertices
                projections - a matrix that will be filled by the output of the constraint projection
                            - NOTE: this matrix contains all constraints, and this projection should
                              fill the rows starting from m_constraint_id
        */
        virtual void project(const Positions& positions, Positions& projections) = 0;

		/** Call after solving local-global algorithm such that the constraint can be updated
		*/
		virtual void update(const Positions& positions, bool plasticity = false){}
    
        /** Reset the updated constraints
         */
        virtual void reset(){}
    
        /** Add the constraint to the matrix A that maps vertex positions to the linear part of the constraint projection.
            Specifically, adds a row w_i S_i A_i to the matrix (as triplets), where the notation of the paper is used.
                triplets - A list of all triplets (row, col, entry) that is being built to construct the full matrix A
                currentRow - The first row index of the constraint in the sparse matrix, i.e. the first row in which to add the entries
                    On return this will containt the next free row in the matrix
                sqrtWeight - If true, sqrt(w_i) instead of w_i is used in the formula above
                transpose - If true, the triplets for the transpose of this matrix are returned
        */
        void addConstraint(std::vector<Triplet>& triplets, Index& currentRow, bool sqrtWeight = false, bool transpose = false) {
            m_constraint_id = currentRow;
            std::vector<Triplet> raw_triplets = getTriplets(currentRow);
            for (auto& t : raw_triplets) {
                Scalar v;
                if (sqrtWeight) {
                    v = t.value() * std::sqrt(getWeight());
                }
                else {
                    v = t.value() * getWeight();
                }
                Index row, col;
                if (transpose) {
                    col = t.row();
                    row = t.col();
                }
                else {
                    col = t.col();
                    row = t.row();
                }
                triplets.push_back(Triplet(row, col, v));
            }
            currentRow += getNumConstraintRows();
        }

        /** Returns number of indices of vertices involved in the constraint. */
        Index numIndices() const { return m_vertex_indices.size(); }

        const std::vector<Index>& getIndices() const { return m_vertex_indices; }

        /** Returns the weight of the constraint */
        Scalar getWeight() const { return m_weight * m_weight_mult; }

        /** Change the weight of this constraint.
            NOTE: this change will only affect the simulation once initializeSystem() is called again. */
        void setWeight(Scalar weight) { m_weight = weight; }

        /** Change the weight multiplier of this constraint.
            NOTE: this change will only affect the simulation once initializeSystem() is called again. */
        void setWeightMultiplier(Scalar mult) { m_weight_mult = mult; }

        virtual ConstraintPtr copy() = 0;

    protected:
        /** Add the constraint to the matrix A that maps vertex positions to the linear part of the constraint projection.
            currentRow - the first row index of the constraint in the sparse matrix, i.e. the first row in which to add the entries
            return - a vector of triplets each representing an entry in a sparse matrix
        */
        virtual std::vector<Triplet> getTriplets(Index currentRow) = 0;

        /** Returns the number of rows of the constraint projection auxiliary variable
        */
        virtual Index getNumConstraintRows() = 0;

        /** Indices of the vertices involved in this constraint. */
        std::vector<Index> m_vertex_indices;

        /** Weight for the constraint.*/
        Scalar m_weight = 1.;

        /** Weight multiplier for the constraint. */
        Scalar m_weight_mult = 1.;

        /** Location of this constraint in the linear system. */
        Index m_constraint_id = 0;
    };



    /**
    As an example for a constraint, the following constraint implements a spring force for an edge of the mesh:
    We have to implement the following methods:
    - the constructor, which initializes the rest data required for this constraint
        (e.g., in this case, the rest length of the spring)
    - the constraint projection, which takes the current positions and computes the "optimal" auxiliary
        projection variable (e.g., in this case, the current edge vector, rescaled to the rest-length)
    - the method that adds the entries for the linear part of the constraint projection
        (e.g., in this case, +1/-1 for the coordinates corresponding to the first/second vertex' coordinate)
    - a method that returns the number of rows for the constraint projection auxiliary variables
        (e.g., in this case, 1, since the auxiliary variable is the edge-vector, with 1 entry per coordinate)
    */
    class EdgeSpringConstraint : public Constraint {
    public:
        EdgeSpringConstraint(const std::vector<Index>& edge_vertices, Scalar weight,
            const Positions& positions)
            :
            Constraint(edge_vertices, weight)
        {
            // Make sure there are at most two vertices in the edge
            assert(m_vertex_indices.size() == 2);
            // Compute rest edge length
            m_rest_length = (positions.row(m_vertex_indices[1]) - positions.row(m_vertex_indices[0])).norm();
            m_base_rest_length = m_rest_length;
        }

        virtual void project(const Positions& positions, Positions& projection) override {
            // Check for correct size of the projection auxiliary variable;
            assert(projection.rows() > m_constraint_id);
            // Compute the current edge
            projection.row(m_constraint_id) = positions.row(m_vertex_indices[1]) - positions.row(m_vertex_indices[0]);
            // Rescale to rest length
            projection.row(m_constraint_id) /= projection.row(m_constraint_id).norm();
            projection.row(m_constraint_id) *= m_rest_length;
        }
        
        virtual void update(const Positions& positions, bool plasticity) override {
            if (plasticity) {
                Scalar plasticity_threshold = 0.1;
                Scalar plasticity_factor = 0.5;
                Scalar new_rest_length = (positions.row(m_vertex_indices[1]) - positions.row(m_vertex_indices[0])).norm();
                if (new_rest_length - m_rest_length > plasticity_threshold) {
                    m_rest_length += (new_rest_length - m_rest_length) * plasticity_factor;
                }
            }
        }
        
        virtual void reset() override {
            m_rest_length = m_base_rest_length;
        }

        virtual Index getNumConstraintRows() override { return 1; };

        virtual ConstraintPtr copy() {
            return std::make_shared<EdgeSpringConstraint>(*this);
        }
    protected:
        Scalar m_rest_length = 0;
        Scalar m_base_rest_length = 0;

        virtual std::vector<Triplet> getTriplets(Index currentRow) override {
            std::vector<Triplet> triplets;
            // The current edge is computed as v_k - v_j, where j/k are the
            // indices of the first/second vertex of this edge.
            // In terms of a matrix row A_i and vertex x/y/z-positions v_x/y/z this means
            // A_ij = -1, A_ik = 1 and A_il = 0 for l =/= j,k
            // A_i v_x/y/z = e_x/y/z
            // Thus:
            triplets.push_back(Triplet(currentRow, m_vertex_indices[0], -1));
            triplets.push_back(Triplet(currentRow, m_vertex_indices[1], 1));

            return triplets;
        }
    };

    /**
        Constraint that specifies the position of a group of vertices
    */
    class PositionConstraintGroup : public Constraint {
    public:
        PositionConstraintGroup(std::vector<Index> inds, Scalar weight, const Positions& targetPositions)
            :
            Constraint(inds, weight)
        {
            assert(inds.size() == targetPositions.rows());
            m_target_positions = targetPositions;
        }

        virtual void project(const Positions& positions, Positions& projection) override {
            // Check for correct size of the projection auxiliary variable;
            assert(projection.rows() >= m_constraint_id + getNumConstraintRows());
            // Set given positions for selected vertices
            for (Index loc_v_ind = 0; loc_v_ind < m_vertex_indices.size(); loc_v_ind++) {
                projection.row(m_constraint_id + loc_v_ind) = m_target_positions.row(loc_v_ind);
            }
        }

        virtual Index getNumConstraintRows() override { return m_vertex_indices.size(); }

        const Positions& getTargetPositions() {
            return m_target_positions;
        }

        void setTargetPositions(const Positions& targetPos) {
            assert(targetPos.rows() == m_target_positions.rows());
            m_target_positions = targetPos;
        }

        virtual ConstraintPtr copy() {
            return std::make_shared<PositionConstraintGroup>(*this);
        }

    protected:
        virtual std::vector<Triplet> getTriplets(Index currentRow) override {
            std::vector<Triplet> triplets;
            Index row = 0;
            for (Index v_ind : m_vertex_indices) {
                triplets.push_back(Triplet(currentRow + row, v_ind, 1.));
                row++;
            }
            return triplets;
        }

    private:
        Positions m_target_positions;
    };

	/**
		Pure virtual constraint to generalize axis aligned walls
	*/
	class WallConstraint : public Constraint {
	public:
		WallConstraint(Index ind, Scalar weight, Axis normalAxis, 
			Scalar firstPos = -std::numeric_limits<Scalar>::infinity(), 
			Scalar secondPos = std::numeric_limits<Scalar>::infinity(), 
			Scalar forceFactor = 1.)
			:
			Constraint({ ind }, weight),
			m_normal_axis(normalAxis),
			m_first_pos(firstPos),
			m_second_pos(secondPos),
			m_vert_ind(ind),
			m_force_factor(forceFactor)
		{
			if (m_first_pos > m_second_pos) 
				std::swap(m_first_pos, m_second_pos);
		}

		virtual Index getNumConstraintRows() override { return 1; }

		virtual void project(const Positions& positions, Positions& projection) override {
			// Check for correct size of the projection auxiliary variable;
			assert(projection.rows() > m_constraint_id);
			// Set corrected positions for vertices that are below the floor height
			projection.row(m_constraint_id) = positions.row(m_vert_ind);

			if (positions(m_vert_ind, m_normal_axis) > m_second_pos) {
				projection(m_constraint_id, m_normal_axis) =
					(1 + m_force_factor) * m_second_pos - m_force_factor * positions(m_vert_ind, m_normal_axis);
			}
			else if (positions(m_vert_ind, m_normal_axis) < m_first_pos) {
				projection(m_constraint_id, m_normal_axis) =
					(1 + m_force_factor) * m_first_pos - m_force_factor * positions(m_vert_ind, m_normal_axis);
			}

		}

		/** Test for collision, i.e. if point is behind or touching the wall
		*/
		virtual bool isColliding(Vector3 point) const {
			return point(m_normal_axis) >= m_second_pos || point(m_normal_axis) <= m_first_pos;
		}

		/** Store in i and j the directions of the friction (x:0,y:1,z:2)
		*/
		virtual void frictionAxes(Axis& i, Axis& j) const {
			i = Axis((m_normal_axis + 1) % 3);
			j = Axis((m_normal_axis + 2) % 3);
		};

	protected:
		virtual std::vector<Triplet> getTriplets(Index currentRow) override {
			std::vector<Triplet> triplets;
			triplets.push_back(Triplet(currentRow, m_vert_ind, 1.));
			return triplets;
		}

	private:
		Axis m_normal_axis;
		Scalar m_first_pos;
		Scalar m_second_pos;
		Index m_vert_ind;
		Scalar m_force_factor;
	};

    /**
        Constraint that keeps the y position of a vertex above a certain coordinate
    */
    class FloorConstraint : public WallConstraint {
    public:
        FloorConstraint(Index ind, Scalar weight, Scalar floorHeight, Scalar forceFactor = 1.)
            :
			WallConstraint(ind, weight, Axis::Y, floorHeight, std::numeric_limits<Scalar>::infinity(), forceFactor)
        {
        }

        virtual ConstraintPtr copy() {
            return std::make_shared<FloorConstraint>(*this);
        }

		virtual void frictionAxes(Axis& i, Axis& j) const override {
			i = Axis::X; j = Axis::Z;
		}
    };

    /**
        Constraint that keeps the x position of a vertex between wallDistance and -wallDistance
    */
    class XWallsConstraint : public WallConstraint {
    public:
        XWallsConstraint(Index ind, Scalar weight, Scalar wallDistance, Scalar forceFactor = 1.)
            :
			WallConstraint(ind, weight, Axis::X, -wallDistance, wallDistance, forceFactor)
        {
        }

        virtual ConstraintPtr copy() {
            return std::make_shared<XWallsConstraint>(*this);
        }

		virtual void frictionAxes(Axis& i, Axis& j) const override {
			i = Axis::Y; j = Axis::Z;
		}
    };

	/**
		Constraint that keeps the y position of a vertex between wallDistance and -wallDistance
	*/
	class YWallsConstraint : public WallConstraint {
	public:
		YWallsConstraint(Index ind, Scalar weight, Scalar wallDistance, Scalar forceFactor = 1.)
			:
			WallConstraint(ind, weight, Axis::Y, -wallDistance, wallDistance, forceFactor)
		{
		}

		virtual ConstraintPtr copy() {
			return std::make_shared<YWallsConstraint>(*this);
		}

		virtual void frictionAxes(Axis& i, Axis& j) const override {
			i = Axis::X; j = Axis::Z;
		}
	};

    /**
        Constraint that keeps the z position of a vertex between wallDistance and -wallDistance
    */
    class ZWallsConstraint : public WallConstraint {
    public:
        ZWallsConstraint(Index ind, Scalar weight, Scalar wallDistance, Scalar forceFactor = 1.)
            :
			WallConstraint(ind, weight, Axis::Z, -wallDistance, wallDistance, forceFactor)
        {
        }

        virtual ConstraintPtr copy() {
            return std::make_shared<ZWallsConstraint>(*this);
        }

		virtual void frictionAxes(Axis& i, Axis& j) const override {
			i = Axis::X; j = Axis::Y;
		}
    };

	/**
		Constraint that simulates friction on the floor
	*/
	class FrictionConstraint : public Constraint {
	public:
		FrictionConstraint(Index ind, Scalar weight, const Positions& positions, std::shared_ptr<WallConstraint> wallConstraint = nullptr)
			:
			Constraint({ ind }, weight),
			m_wall_constraint(wallConstraint),
			m_vert_ind(ind),
			m_prev_pos(positions.row(ind))
		{
		}

		virtual void project(const Positions& positions, Positions& projection) override {
			// Check for correct size of the projection auxiliary variable;
			assert(projection.rows() > m_constraint_id);
			// Set corrected positions for vertices that are below the floor height
			projection.row(m_constraint_id) = positions.row(m_vert_ind);
			if (m_wall_constraint && m_wall_constraint->isColliding(positions.row(m_vert_ind))) {
				Axis i(Axis::X), j(Axis::X);
				m_wall_constraint->frictionAxes(i,j);
				projection(m_constraint_id, i) = m_prev_pos(i);
				projection(m_constraint_id, j) = m_prev_pos(j);
			}
		}
		virtual void update(const Positions& positions, bool plasticity) override {
			m_prev_pos = positions.row(m_vert_ind);
		}

		virtual Index getNumConstraintRows() override { return 1; }

		virtual ConstraintPtr copy() {
			return std::make_shared<FrictionConstraint>(*this);
		}

	protected:
		virtual std::vector<Triplet> getTriplets(Index currentRow) override {
			std::vector<Triplet> triplets;
			triplets.push_back(Triplet(currentRow, m_vert_ind, 1.));
			return triplets;
		}

	private:
		std::shared_ptr<WallConstraint> m_wall_constraint;
		Vector3 m_prev_pos;
		Index m_vert_ind;
	};

    /**
        Tet-strain constraints
    */
    class TetStrainConstraint : public Constraint {
    public:
        TetStrainConstraint(const std::vector<Index>& tetVertices, Scalar weight,
            const Positions& positions, Scalar strain_freedom = 0.)
            :
            Constraint(tetVertices, weight)
        {
            m_strain_freedom = strain_freedom;

            // 3d edges of tet
            Eigen::Matrix<Scalar, 3, 3> edges;
            edges.col(0) = (positions.row(m_vertex_indices[1]) - positions.row(m_vertex_indices[0]));
            edges.col(1) = (positions.row(m_vertex_indices[2]) - positions.row(m_vertex_indices[0]));
            edges.col(2) = (positions.row(m_vertex_indices[3]) - positions.row(m_vertex_indices[0]));

            // Inverse of edges matrix for computation of the deformation gradient
            m_rest_edges_inv = edges.inverse();
            bool didCorrect = false;
            while (!m_rest_edges_inv.allFinite()) {
                std::cout << "Illegal edges in mesh!" << std::endl;
                for (int c = 0; c < 3; c++) {
                    if (edges.col(c).norm() < 1e-12) {
                        for (int r = 0; r < 3; r++) {
                            edges(r, c) = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 1e-11;
                        }
                    }
                }
                for (int r = 0; r < 3; r++) {
                    if (edges.row(r).norm() < 1e-12) {
                        for (int c = 0; c < 3; c++) {
                            edges(r, c) = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX)) * 1e-11;
                        }
                    }
                }
                m_rest_edges_inv = edges.inverse();
                didCorrect = true;
            }
            if (didCorrect) std::cout << "Fixed!" << std::endl;
        }

        virtual void project(const Positions& positions, Positions& projection) override {
            // Compute deformation gradient, clamp its singular values and output
            // corrected deformation gradient as the projection

            // 3d edges of tet
            Eigen::Matrix<Scalar, 3, 3> edges;
            edges.col(0) = (positions.row(m_vertex_indices[1]) - positions.row(m_vertex_indices[0]));
            edges.col(1) = (positions.row(m_vertex_indices[2]) - positions.row(m_vertex_indices[0]));
            edges.col(2) = (positions.row(m_vertex_indices[3]) - positions.row(m_vertex_indices[0]));

            // Compute the deformation gradient (current edges times inverse of original edges)
            Eigen::Matrix<Scalar, 3, 3> F = edges * m_rest_edges_inv;
            // Compute SVD
            Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3>> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vector3 S = svd.singularValues();
            Scalar detU = svd.matrixU().determinant();
            Scalar detV = svd.matrixV().determinant();

            // Clamp singular values
            S(0) = clamp(S(0), 1. - m_strain_freedom, 1. + m_strain_freedom);
            S(1) = clamp(S(1), 1. - m_strain_freedom, 1. + m_strain_freedom);
            S(2) = clamp(S(2), 1. - m_strain_freedom, 1. + m_strain_freedom);
            // Reverse largest singular value if tet is inverted:
            if (detU * detV < 0.0f) S(2) = -S(2);

            // Compute optimal deformation gradient
            F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();

            if (F.hasNaN()) {
                F.setIdentity();
            }

            // Output corrected deformation gradient
            projection.block<3, 3>(m_constraint_id, 0) = F.transpose();
        }

        virtual Index getNumConstraintRows() override { return 3; }

        virtual ConstraintPtr copy() {
            return std::make_shared<TetStrainConstraint>(*this);
        }

    protected:
        Eigen::Matrix<Scalar, 3, 3> m_rest_edges_inv;
        Scalar m_strain_freedom;

        virtual std::vector<Triplet> getTriplets(Index currentRow) override {
            // The selection matrix computes the current deformation gradient w.r.t the current positions
            // (i.e. multiplication of the current edges with the inverse of the original edge matrix)
            std::vector<Triplet> triplets;

            for (int coord3d = 0; coord3d < 3; coord3d++) {
                triplets.push_back(Triplet(currentRow + coord3d, m_vertex_indices[0], -(m_rest_edges_inv(0, coord3d) + m_rest_edges_inv(1, coord3d) + m_rest_edges_inv(2, coord3d))));
                triplets.push_back(Triplet(currentRow + coord3d, m_vertex_indices[1], m_rest_edges_inv(0, coord3d)));
                triplets.push_back(Triplet(currentRow + coord3d, m_vertex_indices[2], m_rest_edges_inv(1, coord3d)));
                triplets.push_back(Triplet(currentRow + coord3d, m_vertex_indices[3], m_rest_edges_inv(2, coord3d)));
            }

            return triplets;
        }
    };

    /**
        Triangle-strain constraints
    */
    class TriangleStrainConstraint : public Constraint {
    public:
        TriangleStrainConstraint(const std::vector<Index>& triVertices, Scalar weight,
            const Positions& positions, Scalar strain_freedom = 0.)
            :
            Constraint(triVertices, weight)
        {
            m_strain_freedom = 0.;

            Eigen::Matrix<Scalar, 3, 2> edges, P;
            // 3d edges of triangle
            edges.col(0) = (positions.row(m_vertex_indices[1]) - positions.row(m_vertex_indices[0]));
            edges.col(1) = (positions.row(m_vertex_indices[2]) - positions.row(m_vertex_indices[0]));

            // Projection that embeds these edges isometrically in 2d, in a way that the first edge is aligned to the x-axis
            P.col(0) = edges.col(0).normalized();
            P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();

            // Compute the 2d rest edges
            Eigen::Matrix<Scalar, 2, 2> restEdges = P.transpose() * edges;

            // ... and their inverse
            m_rest_edges_inv = restEdges.inverse();
        }

        virtual void project(const Positions& positions, Positions& projection) override {
            // Project the current edges isometrically into 2d, compute the deformation gradient there
            // then perform the SVD on the def.grad., clamp the singular values, and project the deformation
            // gradient back.
            Eigen::Matrix<Scalar, 3, 2> edges, P;

            // 3d edges of triangle
            edges.col(0) = (positions.row(m_vertex_indices[1]) - positions.row(m_vertex_indices[0]));
            edges.col(1) = (positions.row(m_vertex_indices[2]) - positions.row(m_vertex_indices[0]));

            // Projection that embeds these edges isometrically in 2d
            P.col(0) = edges.col(0).normalized();
            P.col(1) = (edges.col(1) - edges.col(1).dot(P.col(0)) * P.col(0)).normalized();
            // Compute the deformation gradient (current 2d edges times inverse of original 2d edges)
            Eigen::Matrix<Scalar, 2, 2> F = P.transpose() * edges * m_rest_edges_inv;
            // Compute SVD
            Eigen::JacobiSVD<Eigen::Matrix<Scalar, 2, 2>> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Vector2 S = svd.singularValues();
            // Clamp singular values
            S(0) = clamp(S(0), 1. - m_strain_freedom, 1. + m_strain_freedom);
            S(1) = clamp(S(1), 1. - m_strain_freedom, 1. + m_strain_freedom);
            // Compute clamped deformation gradient
            F = svd.matrixU() * S.asDiagonal() * svd.matrixV().transpose();

            // Project deformation gradient to 3d
            projection.block<2, 3>(m_constraint_id, 0) = (P * F).transpose();
        }

        virtual Index getNumConstraintRows() override { return 2; }

        virtual ConstraintPtr copy() {
            return std::make_shared<TriangleStrainConstraint>(*this);
        }
    protected:
        Eigen::Matrix<Scalar, 2, 2> m_rest_edges_inv;
        Scalar m_strain_freedom;
        virtual std::vector<Triplet> getTriplets(Index currentRow) override {
            std::vector<Triplet> triplets;

            // The selection matrix computes, from the current vertex positions,
            // the deformation gradient (projected to 3d).
            for (int coord2d = 0; coord2d < 2; coord2d++) {
                triplets.push_back(Triplet(currentRow + coord2d, m_vertex_indices[0], -(m_rest_edges_inv(0, coord2d) + m_rest_edges_inv(1, coord2d))));
                triplets.push_back(Triplet(currentRow + coord2d, m_vertex_indices[1], m_rest_edges_inv(0, coord2d)));
                triplets.push_back(Triplet(currentRow + coord2d, m_vertex_indices[2], m_rest_edges_inv(1, coord2d)));
            }

            return triplets;
        }
    };

    /**
        Bending constraints, to be implemented in exercises.cpp
    */
    class BendingConstraint : public Constraint {
    public:
        BendingConstraint(VertexStar vertexStar, Scalar weight, Scalar voronoiArea,
            const Positions& positions, const Triangles& triangles)
            :
            Constraint(getStarIndices(vertexStar), weight)
        {
            m_vertex_star = vertexStar;

            // Compute cotan weights and collect list of triangles in 1-ring
            Vector cotanWeights(m_vertex_star.size());
            int nb = 0, nb2 = 0;
            std::vector<Index> trisSeen;
            trisSeen.reserve(m_vertex_star.size());
            m_triangles.resize(m_vertex_star.size(), 3);
            for (Edge& e : m_vertex_star) {
                Vector3 edgeT11 = (positions.row(m_vertex_indices[0]) - positions.row(e.vOtherT1));
                Vector3 edgeT12 = (positions.row(e.v2) - positions.row(e.vOtherT1));
                Scalar angle1 = std::acos(edgeT11.dot(edgeT12) / (edgeT11.norm() * edgeT12.norm()));
                Scalar cotWeight = (.5 / std::tan(angle1));
                if (e.t2 > 0) {
                    Vector3 edgeT21 = (positions.row(m_vertex_indices[0]) - positions.row(e.vOtherT2));
                    Vector3 edgeT22 = (positions.row(e.v2) - positions.row(e.vOtherT2));
                    Scalar angle2 = std::acos(edgeT21.dot(edgeT22) / (edgeT21.norm() * edgeT22.norm()));
                    cotWeight += (.5 / std::tan(angle2));
                }

                cotanWeights(nb) = cotWeight / voronoiArea;

                if (std::find(trisSeen.begin(), trisSeen.end(), e.t1) == trisSeen.end()) {
                    trisSeen.push_back(e.t1);
                    m_triangles.row(nb2) = triangles.row(e.t1);
                    nb2++;
                }
                if (e.t2 >= 0 && std::find(trisSeen.begin(), trisSeen.end(), e.t2) == trisSeen.end()) {
                    trisSeen.push_back(e.t2);
                    m_triangles.row(nb2) = triangles.row(e.t2);
                    nb2++;
                }
                nb++;
            }
            if (nb2 < nb) {
                Triangles triTemp = m_triangles.block(0, 0, nb2, 3);
                m_triangles = triTemp;
            }

            // Store cotan weights
            m_cotan_weights = cotanWeights;

            // Compute and store mean-curvature (vector)
            Eigen::Matrix<Scalar, 1, 3> meanCurvatureVector;
            meanCurvatureVector.setZero();
            nb = 0;
            for (Edge e : vertexStar) {
                meanCurvatureVector += (positions.row(m_vertex_indices[0]) - positions.row(e.v2)) * cotanWeights(nb);
                nb++;
            }
            m_rest_mean_curv_vec = meanCurvatureVector;
            m_rest_mean_curv = meanCurvatureVector.norm();
            m_base_rest_mean_curv = m_rest_mean_curv;

            // Compute and store the dot product of the mean curvature vector with the
            // normal.
            Vector3 triNormal = getTriangleNormal(m_triangles, positions);
            m_dot_with_normal = triNormal.dot(meanCurvatureVector);

            if (m_cotan_weights.hasNaN()) {
                std::cout << "NaN value in cotan weights!" << std::endl;
            }
        }

        virtual void project(const Positions& positions, Positions& projection) override {
            if (m_rest_mean_curv < 1e-12) {
                projection.row(m_constraint_id).setZero();
            }

            // Compute mean curvature vector
            Eigen::Matrix<Scalar, 1, 3> meanCurvatureVector;
            meanCurvatureVector.setZero();
            int nb = 0;
            for (Edge e : m_vertex_star) {
                meanCurvatureVector += (positions.row(m_vertex_indices[0]) - positions.row(e.v2)) * m_cotan_weights(nb);
                nb++;
            }

            // Rescale mean curvature vector to rest-length
            Scalar norm = meanCurvatureVector.norm();
            if (norm < 1e-10) {
                meanCurvatureVector = getTriangleNormal(m_triangles, positions) * m_rest_mean_curv;
            }
            else {
                meanCurvatureVector *= m_rest_mean_curv / norm;
            }

            // If bending flips should be prevented, check if the sign of the dot product
            // with the normal is the same as in the rest configuration, otherwise flip
            // the mean curvature vector.
            if (PROJDYN_PREVENT_BENDING_FLIPS) {
                Vector3 triNormal = getTriangleNormal(m_triangles, positions);
                Scalar dot = triNormal.dot(meanCurvatureVector);
                if (norm > PROJDYN_BENDING_FLIP_THRESHOLD&& dot* m_dot_with_normal < 0) meanCurvatureVector *= -1;
            }

            projection.row(m_constraint_id) = meanCurvatureVector;
        }

        virtual void update(const Positions& positions, bool plasticity) override {
            if (plasticity){
                Scalar plasticity_threshold = 10;
                Scalar plasticity_factor = 0.1;
                
                Eigen::Matrix<Scalar, 1, 3> meanCurvatureVector;
                meanCurvatureVector.setZero();
                int nb = 0;
                for (Edge e : m_vertex_star) {
                    meanCurvatureVector += (positions.row(m_vertex_indices[0]) - positions.row(e.v2)) * m_cotan_weights(nb);
                    nb++;
                }
                Scalar norm = meanCurvatureVector.norm();
                
                if (abs(m_rest_mean_curv - norm) > plasticity_factor) {
                    m_rest_mean_curv += (norm - m_rest_mean_curv) * plasticity_factor;
                }
            }
        }
        
        virtual void reset() override {
            m_rest_mean_curv = m_base_rest_mean_curv;
        }
        
        virtual Index getNumConstraintRows() override { return 1; }

        virtual ConstraintPtr copy() {
            return std::make_shared<BendingConstraint>(*this);
        }
    protected:
        VertexStar m_vertex_star;
        Vector m_cotan_weights;
        Scalar m_dot_with_normal;
        Scalar m_rest_mean_curv;
        Scalar m_base_rest_mean_curv;
        Eigen::Matrix<Scalar, 1, 3> m_rest_mean_curv_vec;
        Triangles m_triangles;
        virtual std::vector<Triplet> getTriplets(Index currentRow) override {
            // For bending constraints, the selection matrix computes the current mean curvature
            // vector of each vertex.
            std::vector<Triplet> triplets;
            triplets.push_back(Triplet(currentRow, m_vertex_indices[0], m_cotan_weights.sum()));
            Index nb = 0;
            for (Edge e : m_vertex_star) {
                triplets.push_back(Triplet(currentRow, e.v2, -m_cotan_weights(nb)));
                nb++;
            }
            return triplets;
        }
    };

    class FlatteningConstraint : public BendingConstraint {
    public:
        FlatteningConstraint(VertexStar vertexStar, Scalar weight, Scalar voronoiArea,
            const Positions& positions, const Triangles& triangles)
            : BendingConstraint(vertexStar, weight, voronoiArea, positions, triangles) {};

        virtual void project(const Positions& positions, Positions& projection) override {
            projection.row(m_constraint_id).setZero();
        }

        virtual ConstraintPtr copy() {
            return std::make_shared<FlatteningConstraint>(*this);
        }
    };

    class CenterConstraint : public Constraint {
    public:
        CenterConstraint(const std::vector<Index>& vertex_indices, Scalar weight, Vector3 center)
            :
            Constraint(vertex_indices, weight)
        {
            m_center = center;
        }

        virtual void project(const Positions& positions, Positions& projection) override {
            projection.row(m_constraint_id) = m_center.transpose();
        }

        virtual Index getNumConstraintRows() override { return 1; }

        void setCenter(const Vector3& center) {
            m_center = center;
        }

        const Vector3& getCenter() {
            return m_center;
        }

        virtual ConstraintPtr copy() {
            return std::make_shared<CenterConstraint>(*this);
        }
    protected:
        virtual std::vector<Triplet> getTriplets(Index currentRow) override {
            // Compute the average of all positions listed in m_vertex_indices
            std::vector<Triplet> triplets;
            Scalar fac = m_vertex_indices.size();
            for (Index ind : m_vertex_indices) {
                triplets.push_back(Triplet(currentRow, ind, fac));
            }
            return triplets;
        }

        Vector3 m_center;
    };

    class IsometricOneRing : public Constraint {
    public:
        IsometricOneRing(const std::vector<Index>& ring_vertices, Scalar weight,
            const Positions& positions)
            :
            Constraint(ring_vertices, weight),
			m_rest_edges(ring_vertices.size() - 1)
        {
          Vector3 center(positions.row(m_vertex_indices[0]));

          for (Index index(0); index < m_rest_edges.size(); index++) {
            m_rest_edges[index] = Vector3(positions.row(m_vertex_indices[index + 1])) - center;
          }
        }

        virtual void project(const Positions& positions, Positions& projection) override {
            // Compute best fit of original one ring edges to current one ring edges
            // Fill these edges into the projection matrix, starting from the row
            // given by m_constraint_id
            Vector3 center(positions.row(m_vertex_indices[0]));
            MatrixT<3, 3> E;
			      E << 0, 0, 0,
				         0, 0, 0,
				         0, 0, 0;

            for (Index index(0); index < m_rest_edges.size(); index++) {
              Vector3 edge = Vector3(positions.row(m_vertex_indices[index + 1])) - center;
              E += m_rest_edges[index] * edge.transpose();
            }

            Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3>> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix<Scalar, 3, 3> U = svd.matrixU();
            Eigen::Matrix<Scalar, 3, 3> V = svd.matrixV();

            if (svd.singularValues().coeff(0) < 0) {
              U.col(0) *= -1;
            }

            for (Index index(0); index < m_rest_edges.size(); index++) {
              projection.row(m_constraint_id + index) = V * U.transpose() * m_rest_edges[index];
            }
        }

        virtual Index getNumConstraintRows() override {
            return m_rest_edges.size();
        };

        virtual ConstraintPtr copy() {
            return std::make_shared<IsometricOneRing>(*this);
        }
    protected:
        std::vector<Vector3> m_rest_edges;

        virtual std::vector<Triplet> getTriplets(Index currentRow) override {
            std::vector<Triplet> triplets;
            // Compute the k edges of the current 1-Ring
            // [Add code here!]
            for (Index index(1); index < m_vertex_indices.size(); index++) {
                triplets.push_back(Triplet(currentRow + index - 1, m_vertex_indices[0], -1));
                triplets.push_back(Triplet(currentRow + index - 1, m_vertex_indices[index], 1));
            }

            return triplets;
        }
    };


}
