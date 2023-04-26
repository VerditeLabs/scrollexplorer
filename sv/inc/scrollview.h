#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>

#define SVAPI __attribute__((overloadable)) static inline

typedef struct vec3 {
  float x, y, z;
} vec3;

typedef struct Material {
  float refractive_index;
  float albedo[4];
  vec3 diffuse_color;
  float specular_exponent;
} Material;

static const Material MATBASE = {1.0f, {2.0f,0.f,0.f,0.f}, {0.f,0.f,0.f}, 0.f};

typedef struct Sphere {
  vec3 center;
  float radius;
  Material material;
} Sphere;

static const Material ivory = (Material) {1.0f, {0.9f, 0.5f, 0.1f, 0.0f}, (vec3) {0.4f, 0.4f, 0.3f}, 50.0f};
static const Material glass = (Material) {1.5f, {0.0f, 0.9f, 0.1f, 0.8f}, (vec3) {0.6f, 0.7f, 0.8f}, 125.0f};
static const Material red_rubber = (Material) {1.0f, {1.4f, 0.3f, 0.0f, 0.0f}, (vec3) {0.3f, 0.1f, 0.1f}, 10.0f};
static const Material mirror = (Material) {1.0f, {0.0f, 16.0f, 0.8f, 0.0f}, (vec3) {1.0f, 1.0f, 1.0f}, 1425.0f};

#define NUMSPHERES 4ULL
static const Sphere spheres[NUMSPHERES] = {
    {{-3.0f, 0.0f, -16.0f},  2.0f, {1.0f, {0.9f, 0.5f, 0.1f, 0.0f}, {0.4f, 0.4f, 0.3f}, 50.0f}},
    {{-1.0f, -1.5f, -12.0f}, 2.0f, {1.5f, {0.0f, 0.9f, 0.1f, 0.8f}, {0.6f, 0.7f, 0.8f}, 125.0f}},
    {{1.5f, -0.5f, -18.0f},  3.0f, {1.0f, {1.4f, 0.3f, 0.0f, 0.0f}, {0.3f, 0.1f, 0.1f}, 10.0f}},
    {{7.0f, 5.0f, -18.0f},   4.0f, {1.0f, {0.0f, 16.0f, 0.8f, 0.0f}, {1.0f, 1.0f, 1.0f}, 1425.0f}}
};

#define NUMLIGHTS 3
static const vec3 lights[NUMLIGHTS] = {
  {-20.0f, 20.0f, 20.0f},
  {30.0f,  50.0f, -25.0f},
  {30.0f,  20.0f, 30.0f}
};

SVAPI float at(vec3 v, int i) {
  return i == 0 ? v.x : (i == 1 ? v.y : v.z);
}

SVAPI vec3 vec3new(float x, float y, float z) { return (vec3) {x, y, z}; }
SVAPI  vec3 mul(vec3 v, float f) { return (vec3) {v.x * f, v.y * f, v.z * f}; }
SVAPI  vec3 mul(vec3 v, vec3 v2) { return (vec3) {v.x * v2.x, v.y * v2.y, v.z * v2.z}; }
SVAPI  vec3 add(vec3 v, vec3 v2) { return (vec3) {v.x + v2.x, v.y + v2.y, v.z + v2.z}; }
SVAPI  vec3 sub(vec3 v, vec3 v2) { return (vec3) {v.x - v2.x, v.y - v2.y, v.z - v2.z}; }
SVAPI  vec3 neg(vec3 v) { return (vec3) {v.x * -1.0f, v.y * -1.0f, v.z * -1.0f}; }
SVAPI  float dot(vec3 v, vec3 v2) { return v.x * v2.x + v.y * v2.y + v.z * v2.z;}
SVAPI float norm(vec3 v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }
SVAPI vec3 normalized(vec3 v) { return mul(v, 1.0f / norm(v)); }
SVAPI vec3 cross(const vec3 v1, const vec3 v2) {
  return (vec3) {v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x};
}

SVAPI vec3 reflect(vec3 I, vec3 N) { return sub(I, mul(N, 2.0f * dot(I, N))); }

SVAPI vec3 refract(vec3 I, vec3 N, float eta_t, float eta_i) {
  // Snell's law=1.f
  float cosi = -fmaxf(-1.0f, fminf(1.0f, dot(I, N)));
  // if the ray comes from the inside the object, swap the air and the media
  if (cosi < 0.0f) return refract(I, neg(N), eta_i, eta_t);
  float eta = eta_i / eta_t;
  float k = 1.0f - eta * eta * (1.0f - cosi * cosi);
  // k<0 = total reflection, no ray to refract. I refract it anyways, this has no physical meaning
  return k < 0.0f ? (vec3){1.0f, 0.0f, 0.0f} : add(mul(I, eta), mul(N, (eta * cosi - sqrtf(k))));
}

SVAPI bool ray_sphere_intersect(vec3 orig, vec3 dir, Sphere s, float *f) {
  vec3 L = sub(s.center, orig);
  float tca = dot(L, dir);
  float d2 = dot(L, L) - tca * tca;
  if (d2 > s.radius * s.radius) {
    *f = 0.0f;
    return false;
  }
  float thc = sqrtf(s.radius * s.radius - d2);
  float t0 = tca - thc, t1 = tca + thc;
  if (t0 > .001f) {
    *f = t0;
    return true;
  }  // offset the original point by .001 to avoid occlusion by the object itself
  if (t1 > .001f) {
    *f = t1;
    return true;
  }
  *f = 0.0f;
  return false;
}

SVAPI bool scene_intersect(vec3 orig, vec3 dir, vec3 *v, vec3 *v2, Material *mat) {
  vec3 pt, N;
  Material material = MATBASE;

  float nearest_dist = 1e10f;
  if (fabsf(dir.y) > .001f) {
    // intersect the ray with the checkerboard, avoid division by zero
    // the checkerboard plane has equation y = -4
    float d = -(orig.y + 4) / dir.y;
    vec3 p = add(orig, mul(dir, d));
    if (d > .001f && d < nearest_dist && fabsf(p.x) < 10.0f && p.z < -10.0f && p.z > -30.f) {
      nearest_dist = d;
      pt = p;
      N = (vec3) {0.0f, 1.0f, 0.0f};
      material.diffuse_color = (int)(.5f * pt.x + 1000.f) + (int)(.5f * pt.z) & 1 ? (vec3) {.3f, .3f, .3f} : (vec3) {.3f, .2f, .1f};
    }
  }

  for (unsigned long long i = 0; i < NUMSPHERES; i++) {
    // intersect the ray with all spheres
    Sphere s = spheres[i];
    float d;
    bool intersection = ray_sphere_intersect(orig, dir, s, &d);
    if (!intersection || d > nearest_dist) continue;
    nearest_dist = d;
    pt = add(orig, mul(dir, nearest_dist));
    N = normalized(sub(pt, s.center));
    material = s.material;
  }
  *v = pt;
  *v2 = N;
  *mat = material;
  return nearest_dist < 1000;
}

SVAPI vec3 cast_ray(vec3 orig, vec3 dir, int depth) {
  vec3 point;
  vec3 N;
  Material material;
  bool hit = scene_intersect(orig, dir, &point, &N, &material);
  if (depth > 4 || !hit)
    return (vec3) {0.2f, 0.7f, 0.8f}; // background color

  vec3 reflect_dir = normalized(reflect(dir, N));
  vec3 refract_dir = normalized(refract(dir, N, material.refractive_index, 1.0f));
  vec3 reflect_color = cast_ray(point, reflect_dir, depth + 1);
  vec3 refract_color = cast_ray(point, refract_dir, depth + 1);

  float diffuse_light_intensity = 0, specular_light_intensity = 0;
  for (long long i = 0; i < NUMLIGHTS; i++) {
    // checking if the point lies in the shadow of the light
    vec3 light = lights[i];
    vec3 light_dir = normalized(sub(light, point));
    vec3 shadow_pt;
    vec3 trashnrm;
    Material trashmat;
    bool hit = scene_intersect(point, light_dir, &shadow_pt, &trashnrm, &trashmat);
    if (hit && norm(sub(shadow_pt, point)) < norm(sub(light, point)))
      continue;
    diffuse_light_intensity += fmaxf(0.0f, dot(light_dir, N));
    specular_light_intensity += powf(fmaxf(0.f, dot(neg(reflect(neg(light_dir), N)), dir)), material.specular_exponent);
  }
  return add(add(mul(mul(material.diffuse_color, diffuse_light_intensity), material.albedo[0]),
                 mul(mul((vec3) {1.f, 1.f, 1.f}, specular_light_intensity), material.albedo[1])),
             add(mul(reflect_color, material.albedo[2]), mul(refract_color, material.albedo[3])));
}

int main(int argc, char** argv) {
  int width = 1024;
  int height = 768;
  // 60 degrees field of view in radians
  float fov = 1.05f;
  vec3 *framebuffer = malloc((size_t)width * (size_t)height * sizeof(vec3));
  for (int pix = 0; pix < width * height; pix++) {
    float dir_x = ((float) (pix % width) + 0.5f) - (float) width / 2.0f;
    float dir_y = -((float) (pix / width) + 0.5f) + (float) height / 2.0f;
    float dir_z = -(float) height / (2.f * tanf(fov / 2.f));
    vec3 ray = cast_ray((vec3){0, 0, 0}, normalized((vec3) {dir_x, dir_y, dir_z}), 0);
    framebuffer[pix].x = ray.x;
    framebuffer[pix].y = ray.y;
    framebuffer[pix].z = ray.z;
  }

  FILE *fp = fopen("out.ppm", "wb+");
  fprintf(fp,"P6\n");
  fprintf(fp, "%d %d",width, height);
  fprintf(fp, "\n255\n");

  for (int i = 0; i < width * height; i++) {
    vec3 color = framebuffer[i];
    float max = fmaxf(1.f, fmaxf(color.x, fmaxf(color.y, color.z)));
    char x = (char)(255.f * color.x / max);
    char y = (char)(255.f * color.y / max);
    char z = (char)(255.f * color.z / max);
    fprintf(fp,"%c%c%c",x,y,z);
  }
  fclose(fp);
  return 0;
}

