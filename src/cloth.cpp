#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
    
    double z = 0.0;
    double xStep = width / double(num_width_points - 1);
    double yStep = height / double(num_height_points - 1);
    for (int j = 0;  j < num_width_points; j++){
        for (int i = 0; i < num_height_points; i++){
//            if the cloth's orientation is HORIZONTAL, then set the y coordinate for all point masses to 1 while varying positions over the xz plane
            double x = i * xStep;
            double y = j * yStep;
            if (orientation == VERTICAL) {
//                -1/1000 and 1/1000
                z = -1.0/1000.0 + double(rand()) / RAND_MAX * (1.0/1000.0 - -1.0/1000.0);
            } else if (orientation == HORIZONTAL){
                z = y;
                double y = 1.0;
            }
//            Vector3D coordinate = Vector3D(x,y,z);
//            point_masses.emplace_back(coordinate, false);
            vector<int> check = {i, j};
            Vector3D coordinate = Vector3D(x,y,z);
//            point_masses.emplace_back(coordinate, false);
            if (find(pinned.begin(), pinned.end(), check) != pinned.end()){
                point_masses.emplace_back(coordinate, true);
            } else{
                point_masses.emplace_back(coordinate, false);
            }
        
        }
    }
    
    
    
        for (int j = 0;  j < num_width_points; j++){
            for (int i = 0; i < num_height_points; i++){
            //    1.Structural constraints exist between a point mass and the point mass to its left as well as the point mass above it.
            PointMass *thisPoint = &point_masses[num_width_points * i + j];
            if (i > 0) {
                PointMass *abovePoint = &point_masses[num_width_points * (i - 1) + j];
                springs.emplace_back(thisPoint, abovePoint, STRUCTURAL);
            }
            if (j > 0){
                PointMass *leftPoint = &point_masses[num_width_points * i + (j - 1)];
                springs.emplace_back(thisPoint, leftPoint, STRUCTURAL);
            }
            
             //    2.Shearing constraints exist between a point mass and the point mass to its diagonal upper left as well as the point mass to its diagonal upper right.
            if (i > 0) {
                if (j > 0) {
                    PointMass *topLeft = &point_masses[num_width_points * (i - 1) + (j - 1)];
                    springs.emplace_back(thisPoint, topLeft, SHEARING);
                    if (j < num_width_points - 1) {
                        PointMass *topRight = &point_masses[num_width_points * (i - 1) + (j + 1)];
                        springs.emplace_back(thisPoint, topRight, SHEARING);
                    }
                    
                } else {
                    if (j < num_width_points - 1) {
                        PointMass *topRight = &point_masses[num_width_points * (i - 1) + (j + 1)];
                        springs.emplace_back(thisPoint, topRight, SHEARING);
                    }
                }
            }
            //    3.Bending constraints exist between a point mass and the point mass two away to its left as well as the point mass two above it.
            if (i >= 2){
                if (j >= 2) {
                    PointMass *twoLeft = &point_masses[num_width_points * i + (j - 2)];
                    springs.emplace_back(thisPoint, twoLeft, SHEARING);
                    PointMass *twoAbove = &point_masses[num_width_points * (i -2) + j];
                    springs.emplace_back(thisPoint, twoAbove, SHEARING);
                } else {
                    PointMass *twoAbove = &point_masses[num_width_points * (i -2) + j];
                    springs.emplace_back(thisPoint, twoAbove, SHEARING);
                }
            } else {
                if (j >= 2) {
                    PointMass *twoLeft = &point_masses[num_width_points * i + (j - 2)];
                    springs.emplace_back(thisPoint, twoLeft, SHEARING);
                }
            }
        }
    }
    
        
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
//    point_masses
    for (int i = 0; i < num_height_points; i++){
        for (int j = 0;  j < num_width_points; j++){
        PointMass *thisPoint = &point_masses[num_width_points * i + j];
            thisPoint->forces = 0.0;
            for (int x = 0; x < external_accelerations.size(); x++){
                thisPoint->forces += mass * external_accelerations[x];
            }
        }
    }
    for (int i = 0; i < springs.size(); i++){
        Spring *thisSpring = &springs[i];
        Vector3D correctionForce1;
        Vector3D pa = thisSpring->pm_a->position;
        Vector3D pb = thisSpring->pm_b->position;
        int count = 0;
        if (cp->enable_structural_constraints){
            count +=1;
        }
        if (cp->enable_shearing_constraints){
            count +=1;
        }
        if (cp->enable_bending_constraints){
            count +=1;
        }
        if (count > 0) {
            float correctionForce1 = cp->ks * ((pa - pb).norm() - thisSpring->rest_length);
            thisSpring->pm_a->forces += correctionForce1 * (pb - pa).unit() * count;
            thisSpring->pm_b->forces -= correctionForce1 * (pb - pa).unit() * count;
        }
        
    }
  // TODO (Part 2): Use Verlet integration to compute new point mass positions
    for (int i = 0; i < num_height_points; i++){
        for (int j = 0;  j < num_width_points; j++){
            PointMass *thisPoint = &point_masses[num_width_points * i + j];
            if (thisPoint->pinned){
                continue;
            } else{
                Vector3D currentPosition = thisPoint->position;
                Vector3D nextPosition = currentPosition + (1.0 - (cp->damping / 100.0)) * (currentPosition - thisPoint->last_position) + ((thisPoint->forces / mass) * delta_t * delta_t);
                thisPoint->last_position = currentPosition;
                thisPoint->position = nextPosition;
            }
            thisPoint->forces = 0.0;
        }
    }

  // TODO (Part 4): Handle self-collisions.
    build_spatial_map();
    for (int i = 0; i < num_height_points; i++){
        for (int j = 0;  j < num_width_points; j++){
            PointMass *thisPoint = &point_masses[num_width_points * i + j];
            self_collide(*thisPoint, simulation_steps);
        }
    }

  // TODO (Part 3): Handle collisions with other primitives.
    
    for (int i = 0; i < num_height_points; i++){
        for (int j = 0;  j < num_width_points; j++){
            PointMass *thisPoint = &point_masses[num_width_points * i + j];
            for (int x = 0; x < collision_objects->size(); x++){
                CollisionObject *thisCollision = (*collision_objects)[x];
                thisCollision->collide(*thisPoint);
            }
        }
    }
    
  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
    for (int i = 0; i < springs.size(); i++){
        Spring *thisSpring = &springs[i];
        PointMass *pa = thisSpring->pm_a;
        PointMass *pb = thisSpring->pm_b;
        float length = (pb->position - pa->position).norm();
        if (length > thisSpring->rest_length * 1.1){
            if (!pa->pinned){
                if (!pb->pinned){
                    pa->position += ((length - (thisSpring->rest_length * 1.1)) * 1.0/2.0) *(pb->position - pa->position).unit();
                    pb->position -= ((length - (thisSpring->rest_length * 1.1)) * 1.0/2.0) *(pb->position - pa->position).unit();
                } else{
                    pa->position += ((length - (thisSpring->rest_length * 1.1))) *(pb->position - pa->position).unit();
                }
            }else{
                pb->position -= ((length - (thisSpring->rest_length * 1.1))) *(pb->position - pa->position).unit();
            }
        }
    }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
    for (int i = 0; i < num_height_points; i++){
        for (int j = 0;  j < num_width_points; j++){
            PointMass *thisPoint = &point_masses[num_width_points * i + j];
            float hashIndex = hash_position(thisPoint->position);
            if (map.count(hashIndex) == 0){
                map[hashIndex] = new vector<PointMass *>();
            }
            map[hashIndex]->emplace_back(thisPoint);
        }
    }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
//    For each pair between the point mass and a candidate point mass, determine whether they are within 2 * thickness distance apart.
    float cutOff = 2.0 * thickness;
    vector<PointMass *> *thisMap = map[hash_position(pm.position)];
    int n = 0;
    Vector3D newDir = Vector3D();
    if (map.count(hash_position(pm.position)) != 0){
        for (int i = 0; i < thisMap->size(); i++){
            PointMass *candidate = (*thisMap)[i];
            float candidateDist = (candidate->position - pm.position).norm();
            //    If so, compute a correction vector that can be applied to the point mass (not the candidate one) such that the pair would be 2 * thickness2∗thickness distance apart
            if (candidateDist < cutOff and candidate != &pm){
                newDir += (candidate->position - pm.position) - cutOff * (candidate->position - pm.position).unit();
                n += 1;
            }
        }
    }
    if (n > 0){
        //The final correction vector to the point mass's position is the average of all of these pairwise correction vectors, scaled down by simulation_steps
        pm.position += newDir / (simulation_steps * n);
    }
    
    

    
    
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    
    //w∗h∗t where ww = 3 * width / num_width_points, hh = 3 * height / num_height_points, and t = max(w, h)t=max(w,h)
    float ww = 3 * width / num_width_points;
    float hh = 3 * height / num_height_points;
    float t = max(ww, hh);
    float tx = floor(pos.x/ ww);
    float ty = 2.0 * floor(pos.y/ hh);
    float tz = 3.0 * floor(pos.z/ t);
    return tx + ty + tz;
    
//  return 0.f;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
