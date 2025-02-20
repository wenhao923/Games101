#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        Vector2D interval = (end - start) / (num_nodes-1.0);
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        //Comment-in this part when you implement the constructor
        for (int i = 0;i < num_nodes;i++){
            Mass* node = new Mass(start + interval*i, node_mass, false); 
            masses.push_back(node);
            if (i!= 0)
            {
                Spring* sprin = new Spring(masses[i-1], masses[i], k); 
                springs.push_back(sprin);
            }
        }
        for (int i = 0;i < 1;i++) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        // for (auto &s : springs)
        // {
        //     // TODO (Part 2): Use Hooke's law to calculate the force on a node
        //     Vector2D m2_minus_m1 = s->m2->position - s->m1->position;
        //     s->m1->forces +=  (s->k) *  (m2_minus_m1 / m2_minus_m1.norm()) * (m2_minus_m1.norm() - s->rest_length);
        //     s->m2->forces +=  -((s->k) *  (m2_minus_m1 / m2_minus_m1.norm()) * (m2_minus_m1.norm() - s->rest_length));
        // }

        // for (auto &m : masses)
        // {
        //     if (!m->pinned)
        //     {
        //         // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
        //         m->forces += gravity;
        //         Vector2D accel = m->forces / m->mass;
        //         m->velocity += accel * delta_t;
        //         //m->position += m->velocity * delta_t;   //explicit
        //         auto temp = accel * delta_t + m->velocity;
        //         m->position += temp * delta_t;   //semi-implicit 
        //         // TODO (Part 2): Add global damping
                
        //     }
        //     // Reset all forces on each mass
        //     m->forces = Vector2D(0, 0); 
        // }
    }
    
    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {   
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                double damping = 0.000005;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->start_position = m->last_position;   //position(t-1)
                m->last_position = m->position;         //positon(t)
                m->position = m->last_position + (1 - damping) * (m->last_position - m->start_position) + (gravity / m->mass) * delta_t * delta_t;
                //position(t+1)
                // TODO (Part 4): Add global Verlet damping
            }            
        }
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            if (!s->m2->pinned)
            {
                auto dis = (s->m1->position - s->m2->position); //当前绳子长度
                s->m2->position += (dis / dis.norm()) * (dis.norm() - s->rest_length) *0.5; //矫正绳长
            }
            if (!s->m1->pinned)
            {
                auto dis = (s->m1->position - s->m2->position);
                s->m1->position += -(dis / dis.norm()) * (dis.norm() - s->rest_length) *0.5;
            }

            //其实，这样算法还是有问题：当绳子一端不动，另一端动的话，绳长会改变、
            //                      且一个短点会被计算两次，又导致绳长改变。
            //                      此外，先计算端点，再根据绳长修改位置和线更新spring再更新质点应该不一样的。
        }
    }
}
