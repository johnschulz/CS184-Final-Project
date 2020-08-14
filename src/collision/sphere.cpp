#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
    
//    1.Compute where the point mass should have intersected the sphere by extending the path between its 'position' and the sphere's origin to the sphere's surface. Call the surface intersection point the tangent point.
    Vector3D path = pm.position - origin;
    Vector3D tangentPoint = radius * path.unit() + origin;
    
    if (path.norm() <= radius) {
        //    2. Compute the correction vector needed to be applied to the point mass's last_position in order to reach the tangent point.
        Vector3D correctionVector = tangentPoint - pm.last_position;
        //    3. Finally, let the point mass's new position be its last_position adjusted by the above correction vector, scaled down by friction (i.e. scaled by (1 - f)).
            
        pm.position = pm.last_position + correctionVector * (1 - friction);
    }
    
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
