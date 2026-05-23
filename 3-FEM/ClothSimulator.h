#pragma once

#include <array>
#include <cstddef>
#include <glm/glm.hpp>
#include <vector>

namespace VCX::Labs::FEM {

    struct Tri {
        std::array<int, 3> ids {};
        glm::mat2          DmInv { 1.0f }; 
        float              area { 0.0f };  
        glm::vec2          gradN[3] {};  
    };

    class ClothSimulator {
    public:
        void SetupCloth(std::size_t w, std::size_t h, float dx, glm::vec3 origin);
        void ResetToRest();
        void SetMaterial(float young, float nu, float areaDensity);
        void SetGravity(glm::vec3 g) { m_gravity = g; }
        void SetDamping(float d) { m_damping = d; }

        void SetPinnedCorners(bool on);
        void SetGrabVertex(int vid) { m_grabVid = vid; }
        int  GetGrabVertex() const { return m_grabVid; }
        void SetGrabForce(glm::vec3 f) { m_grabForce = f; }

        void SimulateTimestep(float dt);

    public:
        std::vector<glm::vec3> m_X, m_x, m_v;
        std::vector<glm::vec2> m_UV;
        std::vector<float>     m_mass;
        std::vector<bool>      m_fixed;
        std::vector<Tri>       m_tris;

        std::size_t m_w { 25 }, m_h { 25 };
        float       m_dx { 0.05f };

    private:
        void BuildTris();
        void PrecomputeTriData();
        void ApplyPin();
        void ComputeForces(std::vector<glm::vec3> & f);
        void DeformationGradientCols(Tri const & t, glm::vec3 & f0, glm::vec3 & f1) const;
    private:
        float m_young { 50.0f };
        float m_nu { 0.3f };
        float m_rhoA { 0.5f }; 
        float m_mu { 0.0f };
        float m_lambda { 0.0f };

        glm::vec3 m_gravity { 0.0f, -9.8f, 0.0f };
        float     m_damping { 0.02f };

        bool m_pinCorners { true };

        int       m_grabVid { 0 };
        glm::vec3 m_grabForce { 0.0f };
    };

} // namespace VCX::Labs::FEM