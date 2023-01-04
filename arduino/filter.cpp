// IMPORTANT
// this code is based on filter design by Philip Schmidt 
// http://philstech.blogspot.com/2014/09/fast-quaternion-integration-for.html 

#include <math.h>
#include "filter.h"

static quat gyro_quat, acc_magn_quat;
static vec3 gyro_vec;
static vec3 accel_body, accel_world;
static vec3 mag_body, mag_world;
static vec3 correction_world, correction_world_a, correction_world_m, correction_body;
static quat orientation, orientation_last;
static float bx, by, bz;

quat update_filter(float ax, float ay, float az, float gx, float gy, float gz,
		   float mx, float my, float mz, float dt)
{
	gyro_vec.x = gx;
	gyro_vec.y = gy;
	gyro_vec.z = gz;

	accel_body.x = ax;
	accel_body.y = ay;
	accel_body.z = az;

	mag_body.x = mx;
	mag_body.y = my;
	mag_body.z = mz;

	// pitch & roll corrections from accel 
	accel_world = rotate(orientation_last, accel_body);
	correction_world_a = cross_product_earth(accel_world);

#if 0
	// magnetometer corrections  
	mag_world = rotate(orientation_last, mag_body);
	mag_world.z = 0.0f;
	normalize(&mag_world);	
	correction_world_m = cross_product_north(mag_world);
	correction_world = sum(correction_world_a, correction_world_m); 
#else 
	correction_world = correction_world_a;
#endif

	// fuse corrections with gyroscope readings
	correction_body = rotate(correction_world, orientation_last);
	vec3 gyro_vec_corrected = sum(gyro_vec, correction_body);
	gyro_quat = gyro_to_quat(&orientation_last, gyro_vec_corrected.x,
				gyro_vec_corrected.y, gyro_vec_corrected.z, dt);
	orientation = gyro_quat;
	orientation_last = gyro_quat;
	normalize(&orientation);

	return orientation;
}

vec3 rotate(quat q, vec3 v)
{
	vec3 r, res, res1;
	r.x = q.x;
	r.y = q.y;
	r.z = q.z;

	res1 = cross_product(sum(r, r), sum(cross_product(r, v), mult(q.w, v)));
	res = sum(v, res1);
	return res;
}

vec3 rotate(vec3 v, quat q)
{
	vec3 r, res, res1;
	r.x = -q.x;
	r.y = -q.y;
	r.z = -q.z;

	res1 = cross_product(sum(r, r), sum(cross_product(r, v), mult(q.w, v)));
	res = sum(v, res1);
	return res;
}

vec3 cross_product_earth(vec3 v1)
{
	// VERTICAL = {0.0f, 0.0f, 1.0f} 
	vec3 res;
	res.x = v1.y * 1.0f - v1.z * 0.0f;
	res.y = v1.z * 0.0f - v1.x * 1.0f;
	res.z = v1.x * 0.0f - v1.y * 0.0f;
	return res;
}

vec3 cross_product(vec3 v1, vec3 v2)
{
	vec3 res;
	res.x = v1.y * v2.z - v1.z * v2.y;
	res.y = v1.z * v2.x - v1.x * v2.z;
	res.z = v1.x * v2.y - v1.y * v2.x;
	return res;
}

quat mult(quat *a, quat *b)
{
	quat res;
	res.w = a->w * b->w - a->x * b->x - a->y * b->y - a->z * b->z;
	res.x = a->w * b->x + a->z * b->y - a->y * b->z + a->x * b->w;
	res.y = a->w * b->y + a->x * b->z + a->y * b->w - a->z * b->x;
	res.z = a->y * b->x - a->x * b->y + a->z * b->z + a->z * b->w;
	return res;
}

vec3 mult(float a, vec3 v)
{
	vec3 res;
	res.x = v.x * a;
	res.y = v.y * a;
	res.z = v.z * a;
	return res;
}

quat sum(quat q, vec3 v)
{
	quat res;
	res.x = q.x + v.x;
	res.y = q.y + v.y;
	res.z = q.z + v.z;
	res.w = q.w;
	return res;
}

vec3 sum(vec3 v1, vec3 v2)
{
	vec3 res;
	res.x = v1.x + v2.x;
	res.y = v1.y + v2.y;
	res.z = v1.z + v2.z;
	return res;
}

quat gyro_to_quat(quat *last, float gx, float gy, float gz, float dt)
{
	float t_2 = dt * 0.5f;

	quat res;
	res.w = last->w - t_2 * gx * last->x - t_2 * gy * last->y -
		t_2 * gz * last->z;
	res.x = last->x + t_2 * gx * last->w - t_2 * gy * last->z +
		t_2 * gz * last->y;
	res.y = last->y + t_2 * gx * last->z + t_2 * gy * last->w -
		t_2 * gz * last->x;
	res.z = last->z - t_2 * gx * last->y + t_2 * gy * last->x +
		t_2 * gz * last->w;
	return res;
}

void normalize(quat *q)
{
	float norm =
		sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);

	if (norm == 0.0f)
		return;

	q->w /= norm;
	q->x /= norm;
	q->y /= norm;
	q->z /= norm;
}

void normalize(vec *v)
{
	float norm =
		sqrtf(v->x * v->x + v->y * v->y + v->z * v->z) ;

	if (norm == 0.0f)
		return;

	v->x /= norm;
	v->y /= norm;
	v->z /= norm;
}
