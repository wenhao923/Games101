//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Vector3f L_dir = Vector3f(0.0, 0.0 ,0.0);
    Vector3f L_indir = Vector3f(0.0, 0.0 ,0.0);
    // if (depth > this->maxDepth) {
    //     return L_dir;
    // }
    
    // TO DO Implement Path Tracing Algorithm here
    Intersection point1 = this->intersect(ray);
    if (!point1.happened) {
        return L_dir;
    }

    if (point1.m->hasEmission()) {
        return point1.m->getEmission();
    }
           
    // 直接光
    //求直射光的pdf和pos    
    float pdf = 0.0;
    Intersection light_pos;
    this->sampleLight(light_pos, pdf);
    //光源面的法线
    Vector3f NN = light_pos.normal.normalized();

    //目光
    Vector3f wo = (Vector3f(-ray.direction)).normalized();    
    //point1 ---> light
    Vector3f ws1 = light_pos.coords - point1.coords;
    // ||p-x||
    double dis_fromL2P = sqrt(ws1.x*ws1.x + ws1.y*ws1.y +ws1.z*ws1.z);
    ws1 = ws1.normalized();
    Intersection reflect_light = this->intersect(Ray(point1.coords, ws1));

    if (reflect_light.happened && reflect_light.m->hasEmission())
    {
        L_dir = light_pos.emit * point1.m->eval(wo, ws1, point1.normal.normalized()) 
                            * dotProduct(ws1, point1.normal.normalized()) * dotProduct(-ws1, NN)
                            / pdf / pow(dis_fromL2P, 2);
        //std::cout<< L_dir<<std::endl;
    }
    
    //间接光照
    //俄罗斯轮盘赌
    if (get_random_float() < this->RussianRoulette)
    {
        //根据入射和法线，计算采样的线
        Vector3f wi_direction = (point1.m->sample(wo, point1.normal)).normalized();
        Ray wi(point1.coords, wi_direction);
        Intersection point2 = this->intersect(wi);   
        
        if (point2.happened && !point2.m->hasEmission())
        {                
            L_indir = this->castRay(wi,depth + 1) * point1.m->eval( wo, wi.direction, point1.normal) 
                                                            * dotProduct(wi.direction, point1.normal) 
                                                            / point2.m->pdf(wo, wi.direction , point1.normal)                                                           
                                                            / this->RussianRoulette;                                                 
        }
    }

    //std::cout<<L_dir + L_indir<<std::endl;
    return L_dir + L_indir;

}