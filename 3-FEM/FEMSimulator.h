#pragma once

#include <array>
#include <cstddef>
#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::FEM {

    enum class ElasticModel {
        Linear     = 0,
        StVK       = 1,
        NeoHookean = 2,
        Corotated  = 3
    };

    struct Tet {
        std::array<int, 4> ids {};
        glm::mat3          DmInv { 1.0f };
        float              volume { 0.0f };
        glm::vec3          gradN[4] {};
    };

    struct CollisionParams {
        float stiffness { 5000.0f }; 
        float damping { 50.0f };   
        float maxDepth { 0.03f };  
    };

    struct PlaneCollider {
        bool      enabled { false };
        glm::vec3 n { 0, 1, 0 }; 
        float     d { 0.0f };   
    };

    struct SphereCollider {
        bool      enabled { false };
        glm::vec3 center { 0, 0, 0 };
        float     radius { 0.2f };
    };

    struct AABBCollider {
        bool      enabled { false };
        glm::vec3 bmin { -0.2f };
        glm::vec3 bmax { 0.2f };
    };

    class Simulator {
    public:
        void SetupBox(std::size_t wx, std::size_t wy, std::size_t wz, float dx, glm::vec3 origin);
        void ResetToRest();

        void SetMaterial(float young, float nu, float density);
        void SetGravity(glm::vec3 g) { m_gravity = g; }
        void SetDamping(float d) { m_damping = d; }

        void         SetModel(ElasticModel m) { m_model = m; }
        ElasticModel GetModel() const { return m_model; }

        void SetGrabVertex(int vid) { m_grabVid = vid; }
        int  GetGrabVertex() const { return m_grabVid; }
        void SetGrabForce(glm::vec3 f) { m_grabForce = f; }
        void                    SetCollisionParams(CollisionParams const & p) { m_col = p; }
        CollisionParams const & GetCollisionParams() const { return m_col; }

        void EnableGround(bool on, float y = 0.0f); 
        void EnableWallX(bool on, float x = 0.0f);  
        void SetSphere(bool on, glm::vec3 c, float r);
        void SetAABB(bool on, glm::vec3 bmin, glm::vec3 bmax);

        void SimulateTimestep(float dt);

    public:
        std::vector<glm::vec3> m_X;
        std::vector<glm::vec3> m_x;
        std::vector<glm::vec3> m_v;
        std::vector<float>     m_mass;
        std::vector<bool>      m_fixed;
        std::vector<Tet>       m_tets;

    private:
        int GetID(std::size_t i, std::size_t j, std::size_t k) const;

        void BuildTets();
        void PrecomputeTetData();
        void ComputeForces(std::vector<glm::vec3> & f);

        glm::mat3 DeformationGradient(Tet const & t) const;
        glm::mat3 ComputeP(glm::mat3 const & F) const;

        void AddCollisionForces(std::vector<glm::vec3> & f);
        void AddPenalty(std::vector<glm::vec3> & f, std::size_t i, float phi, glm::vec3 const & n);

        bool AABBPenetration(glm::vec3 const & x, float & phi, glm::vec3 & n) const;

    private:
        std::size_t m_wx { 8 }, m_wy { 2 }, m_wz { 2 };
        float       m_dx { 0.05f };

        float m_young { 20000.0f };
        float m_nu { 0.2f };
        float m_rho { 400.0f };
        float m_mu { 0.0f };
        float m_lambda { 0.0f };

        ElasticModel m_model { ElasticModel::Linear };

        glm::vec3 m_gravity { 0.0f, -0.05f, 0.0f };
        float     m_damping { 0.03f };

        int       m_grabVid { 0 };
        glm::vec3 m_grabForce { 0.0f };
        CollisionParams m_col {};
        PlaneCollider   m_ground;
        PlaneCollider   m_wallX;
        SphereCollider  m_sphere;
        AABBCollider    m_aabb;
    };

} // namespace VCX::Labs::FEM